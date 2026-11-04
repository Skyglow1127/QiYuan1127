#!/usr/bin/env python3
"""Read-only perception -> fixed-plane cube pose, with measured backend contracts.

Produces input for run_grasp_cycle.py; never sends a robot motion/power command.
Saved-image mode is replay evidence only. Live mode still needs plan approval.
"""
import argparse
import base64
import datetime
import hashlib
import json
from pathlib import Path
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parents[1]))
sys.path.insert(0, str(ROOT/'ros2_ws/src/d1_pick_place'))
from app.hsv import infer_remote
from app.device_control import status_fields
from app.service_errors import ServiceRequestError, service_request
from d1_pick_place.sdk_executor import (Mapping, NAMES, CliTransport, LIMITS, HOST_BINARY,
    FEEDBACK_JOINT_TOLERANCE_DEG, feedback_limit_errors)
from d1_pick_place.kinematics import forward
from cube_mask_pose import fit_pose
from grasp_timing import Trace, boot_id, load_rows

DEFAULT_CANDIDATE = ROOT/'logs/handeye_resume_20261022_145044/empirical_handeye_candidate.json'
DEFAULT_TABLE = ROOT/'logs/first_grasp_execution_20261022/at_pregrasp/cube_pose.json'


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def sha(raw): return hashlib.sha256(raw).hexdigest()


def warmup_npu(endpoint, trace):
    # One explicit task-start warmup. Its cost stays in the overall timing and
    # overlaps read-only DDS discovery; no hidden keepalive/power-policy change.
    with trace.span('startup','npu_task_start_warmup'):
        ok, encoded=cv2.imencode('.png',np.zeros((480,640,3),np.uint8))
        if not ok:raise RuntimeError('Warmup encoding failed')
        result=infer_remote(encoded.tobytes(),endpoint=endpoint)
        if not result.get('npu_execution_confirmed'):raise RuntimeError('Warmup did not execute on NPU')
        return result['timing_ms']


def robot_readiness_errors(snapshot, now):
    """Report all observed blockers without altering state, feedback or limits."""
    errors = []
    if snapshot.get('fault'):
        errors.append('控制通信报告故障：' + str(snapshot['fault']))
    q = snapshot.get('q')
    errors.extend(feedback_limit_errors(q))
    age = now - snapshot.get('q_time', -float('inf'))
    if not np.isfinite(age) or not 0 <= age <= .6:
        errors.append('关节实际反馈缺失或超过 0.6 秒')
    for key, expected in [('power_status',1), ('enable_status',(0,1)), ('error_status',0)]:
        actual = snapshot.get('status',{}).get(key)
        allowed=expected if isinstance(expected,tuple) else (expected,)
        if type(actual) is not int or actual not in allowed:
            errors.append(f'控制协议软件标志 {key}={actual!r}，执行器要求 {expected}；需恢复并核实控制状态，软件标志不证明硬件扭矩')
        age = now - snapshot.get('status_time',{}).get(key,-float('inf'))
        if not np.isfinite(age) or not 0 <= age <= .6:
            errors.append(f'{key} 状态反馈缺失或超过 0.6 秒')
    return errors


def save_diagnostic(folder, raw, foreground):
    image = cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    if image is None: return None
    overlay=image.copy()
    box=foreground.get('bounding_box_xywh')
    if box:
        x,y,w,h=box
        cv2.rectangle(overlay,(x,y),(min(639,x+w-1),min(479,y+h-1)),(0,220,255),2)
    for edge in foreground.get('clipped_edges',[]):
        a,b={'top':((0,1),(639,1)), 'bottom':((0,478),(639,478)),
             'left':((1,0),(1,479)), 'right':((638,0),(638,479))}[edge]
        cv2.line(overlay,a,b,(0,0,255),4)
    label=foreground.get('rejection_code') or 'foreground_candidate'
    cv2.rectangle(overlay,(0,0),(639,35),(22,35,48),-1)
    cv2.putText(overlay,label,(12,24),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
    path=folder/'diagnostic_overlay.jpg';cv2.imwrite(str(path),overlay)
    return str(path)


def live_observation(folder, trace):
    transport = CliTransport(HOST_BINARY,'enp99s0',folder/'readonly.jsonl')
    try:
        samples=[]; sequences=[]
        with trace.span('stage_wait','read_only_feedback_discovery_and_stationarity'):
            deadline=time.monotonic()+4
            while time.monotonic()<deadline and len(samples)<10:
                snapshot=transport.snapshot()
                if snapshot['q'] is not None and snapshot['q_seq'] not in sequences:
                    samples.append(snapshot['q'].tolist()); sequences.append(snapshot['q_seq'])
                time.sleep(.1)
            early=transport.snapshot()
            save(folder/'robot_status.json',dict(sdk_angles=None if early['q'] is None else early['q'].tolist(),
                 **{key:value for key,value in status_fields(early,transport.now()).items() if key!='sdk_angles'},
                 robot_readiness_errors=robot_readiness_errors(early,transport.now()), robot_commands_sent=transport.commands))
            if len(samples)<10: raise ValueError('Insufficient distinct feedback samples')
            if np.ptp(samples,axis=0).max()>.3+1e-5: raise ValueError('Robot moving during observation')
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with trace.span('acquisition','wrist_http_status_and_image'):
            with service_request(opener,'http://127.0.0.1:18760/status','腕部相机服务') as response: status=json.load(response)
            if status.get('error') or status.get('stopped') or status['device']['serial']!='CC1365301EY':
                raise ValueError('Unexpected wrist camera')
            fetch_start=time.monotonic()
            with service_request(opener,'http://127.0.0.1:18760/color.jpg','腕部相机服务') as response:
                raw=response.read(); headers=dict(response.headers)
            received=time.monotonic()
            age=float(headers['X-Board-Age-Ms'])+(received-fetch_start)*1000
            if not 0<=age<=250: raise ValueError('Wrist frame stale at reception')
        snapshot=transport.snapshot()
        readiness=robot_readiness_errors(snapshot,transport.now())
        # Check the entire observed window, not whichever boundary sample was last.
        for sample in samples:
            for message in feedback_limit_errors(sample):
                if message not in readiness: readiness.append(message)
        fault='；'.join(readiness) if readiness else None
        if snapshot['q'] is None: raise ValueError('Missing robot feedback')
        image=folder/'wrist.jpg'; image.write_bytes(raw)
        observation=dict(image=str(image),image_sha256=sha(raw),sdk_angles=snapshot['q'].tolist(),
            **{key:value for key,value in status_fields(snapshot,transport.now()).items() if key!='sdk_angles'},
            feedback_age_s=transport.now()-snapshot['q_time'],
            camera_serial='CC1365301EY',headers=headers,
            captured_at=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
            source_kind='live_camera',frame_index=int(headers['X-Frame-Index']),
            board_age_at_fetch_ms=age,received_host_monotonic=received,
            robot_ready=not readiness,robot_readiness_error=fault,robot_readiness_errors=readiness,
            stationary_span_deg=np.ptp(samples,axis=0).tolist(),
            feedback_joint_tolerance_deg=FEEDBACK_JOINT_TOLERANCE_DEG,
            hardware_commands_sent=transport.commands)
        save(folder/'wrist_observation.json',observation)
        return observation,raw
    finally: transport.close()


def prepare(args):
    args.output.mkdir(parents=True,exist_ok=False)
    trace=Trace(args.output/'timing.jsonl'); start=time.monotonic();trace.mark('recognition_start')
    result=dict(schema='d1-accelerated-perception-v1',completed=False,robot_commands_sent=0,
                physical_grasp_executed=False,source_kind='live_camera' if args.capture_live else 'saved_image',
                backend=args.backend,preprocessing=args.preprocess,blockers=[])
    endpoint=args.endpoint or {'npu':'http://127.0.0.1:18820','cpu':'http://127.0.0.1:18821','hsv':'http://127.0.0.1:18822'}[args.backend]
    result.update(endpoint=endpoint,fpga_url=args.fpga_url,candidate=str(args.candidate.resolve()),
                  table_pose=str(args.table_pose.resolve()))
    try:
        if args.capture_live:
            if args.backend=='npu':
                with ThreadPoolExecutor(max_workers=1) as pool:
                    warming=pool.submit(warmup_npu,endpoint,trace)
                    observation,raw=live_observation(args.output,trace)
                    result['warmup_timing_ms']=warming.result()
            else:observation,raw=live_observation(args.output,trace)
        else:
            with trace.span('acquisition','saved_image_and_observation_read'):
                observation=json.loads(args.observation.read_text());raw=Path(observation['image']).read_bytes()
            if sha(raw)!=observation['image_sha256']:raise ValueError('Saved image identity mismatch')
        result.update(robot_ready=observation.get('robot_ready',False),
                      robot_readiness_error=observation.get('robot_readiness_error'),
                      robot_readiness_errors=observation.get('robot_readiness_errors',[]),
                      sdk_angles=observation.get('sdk_angles'), image=observation.get('image'))
        if args.capture_live:
            result['blockers'].extend(dict(code='robot_not_ready',message=v,action='恢复有效初态并核实控制状态后重新准备')
                                      for v in observation.get('robot_readiness_errors',[]))
        metadata=dict(image_sha256=sha(raw),source_kind=result['source_kind'])
        if args.capture_live:
            metadata.update({key:observation[key] for key in ['camera_serial','frame_index','board_age_at_fetch_ms','received_host_monotonic']})
        preprocessing=None
        if args.preprocess=='host':
            with trace.span('recognition','local_opencv_foreground'):
                foreground=infer_remote(raw,metadata,endpoint)
        else:
            from grasp_accel_fpga import process_remote
            source=dict(source_kind='live_wrist' if args.capture_live else 'replay')
            if args.capture_live:
                source['frame']={key:metadata[key] for key in ['camera_serial','frame_index','board_age_at_fetch_ms','received_host_monotonic']}
            with trace.span('serialization','board_preprocess_transport_and_compute'):
                rgb,preprocessing=process_remote(raw,url=args.fpga_url,source=source,
                    require_hardware=args.preprocess=='fpga')
            save(args.output/'preprocessor.json',preprocessing)
            with trace.span('recognition','foreground_preprocessed_http_inference'):
                foreground=infer_remote(rgb.tobytes(),preprocessing,endpoint,preprocessed=True)
        save(args.output/'foreground.json',foreground)
        if foreground['image_sha256']!=sha(raw):raise ValueError('Foreground belongs to another image')
        if foreground['backend']!=args.backend:raise ValueError('Unexpected inference backend')
        if args.backend=='npu' and not foreground.get('npu_execution_confirmed'):
            raise ValueError('NPU execution not confirmed; no fallback permitted')
        mask_raw=base64.b64decode(foreground['mask_png_base64'],validate=True)
        if sha(mask_raw)!=foreground['mask_png_sha256']:raise ValueError('Mask checksum mismatch')
        mask=cv2.imdecode(np.frombuffer(mask_raw,np.uint8),cv2.IMREAD_GRAYSCALE)
        (args.output/'mask.png').write_bytes(mask_raw)
        result['diagnostic_image']=save_diagnostic(args.output,raw,foreground)
        if foreground['status']!='FOREGROUND_CANDIDATE':
            reason=foreground.get('rejection_reason','Invalid foreground')
            result['blockers'].insert(0,dict(code=foreground.get('rejection_code','vision_rejected'),message=reason,action='查看诊断图并重新准备'))
            raise ValueError(reason)
        if foreground.get('significant_component_count',1)>1:raise ValueError('Multiple red foreground candidates')
        candidate=json.loads(args.candidate.read_text()); table_pose=json.loads(args.table_pose.read_text())
        with trace.span('pose','fixed_plane_silhouette_fit_and_coordinate_transform'):
            mapping=Mapping.from_candidate(args.candidate)
            values=dict(zip(NAMES,mapping.to_model(observation['sdk_angles'])))
            camera=forward(ROOT/'ros2_ws/src/d1_pick_place/config/d1.urdf',values,'Link6') @ np.array(candidate['T_end_camera'])
            pose=fit_pose(mask,camera,np.array(table_pose['T_base_table']),
                np.array(candidate['camera_matrix']),np.array(candidate['distortion']))
        pose.update(source_observation=str(args.observation.resolve()) if args.observation else str(args.output/'wrist_observation.json'),
                    image_sha256=sha(raw),sdk_start=observation['sdk_angles'],
                    source_table=str(args.table_pose.resolve()),source_table_sha256=sha(args.table_pose.read_bytes()),
                    candidate_sha256=sha(args.candidate.read_bytes()),
                    foreground_sha256=sha((args.output/'foreground.json').read_bytes()),
                    uses_npu=args.backend=='npu',uses_fpga=args.preprocess=='fpga',
                    source_kind=result['source_kind'])
        save(args.output/'pose.json',pose)
        image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        cv2.polylines(image,[np.rint(pose['predicted_hull_px']).astype(np.int32)],True,(0,255,255),2)
        cv2.putText(image,f"FIT IoU {pose['silhouette_iou']:.3f} / {pose['silhouette_boundary_rms_px']:.2f}px",(10,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
        cv2.imwrite(str(args.output/'pose_overlay.jpg'),image)
        if not pose['geometric_fit_accepted']:
            details='；'.join(pose.get('fit_rejection_reasons',[]))
            raise ValueError('Silhouette fit rejected'+('：'+details if details else ''))
        age=None
        if args.capture_live:
            age=observation['board_age_at_fetch_ms']+(time.monotonic()-observation['received_host_monotonic'])*1000
            result['final_pose_frame_age_ms']=age
            if age>250:raise ValueError('Final pose uses a frame older than 250 ms')
        source_observation=args.observation or args.output/'wrist_observation.json'
        dependencies=[args.candidate,args.table_pose,source_observation,Path(observation['image']),
                      args.output/'foreground.json',args.output/'mask.png',Path(__file__),
                      Path(__file__).with_name('cube_mask_pose.py'),
                      ROOT.parents[1] / 'app/hsv.py', ROOT.parents[1] / 'app/device_control.py']
        if preprocessing:
            dependencies.extend([args.output/'preprocessor.json',Path('/home/aup/Camera/scripts/grasp_accel_fpga.py')])
        if foreground.get('session_evidence'):
            evidence=Path(foreground['session_evidence'])
            if sha(evidence.read_bytes())!=foreground['session_evidence_sha256']:
                raise ValueError('Inference session evidence changed')
            dependencies.append(evidence)
        result.update(completed=True,pose=str(args.output/'pose.json'),pose_sha256=sha((args.output/'pose.json').read_bytes()),
            foreground_consumed_by='cube_mask_pose.fit_pose; local OpenCV HSV mask supplies the silhouette',
            uses_npu=args.backend=='npu',uses_fpga=args.preprocess=='fpga',
            robot_ready=observation.get('robot_ready',False),robot_readiness_error=observation.get('robot_readiness_error'),
            execution_candidate_eligible=args.capture_live and observation.get('robot_ready',False),
            dependencies={str(p.resolve()):sha(p.read_bytes()) for p in dependencies},
            live_execution_eligible=False,reason='Specific motion plan and explicit user approval still required',
            inference_timing_ms=foreground['timing_ms'])
    except Exception as error:
        result['error']=str(error)
        if isinstance(error,ServiceRequestError):
            result['technical_error']=error.technical_error
            result['blockers'].append(error.blocker())
        elif not any(b['message']==str(error) for b in result['blockers']):
            result['blockers'].append(dict(code='preparation_error',message=str(error),action='查看保存的诊断图与原始记录'))
    finally:
        end=time.monotonic();trace.mark('perception_complete')
        result['timing']=dict(boot_id=boot_id(),start_monotonic=start,end_monotonic=end,
            elapsed_s=end-start,spans=[r for r in load_rows(trace.path) if r['kind']=='span'])
        if not result.get('robot_readiness_errors') and (args.output/'robot_status.json').exists():
            robot=json.loads((args.output/'robot_status.json').read_text())
            result['robot_readiness_errors']=robot.get('robot_readiness_errors',[])
            if args.capture_live:
                result['blockers'].extend(dict(code='robot_not_ready',message=v,action='核实控制状态后重新准备')
                                          for v in result['robot_readiness_errors'])
        save(args.output/'result.json',result)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    src=p.add_mutually_exclusive_group(required=True)
    src.add_argument('--observation',type=Path);src.add_argument('--capture-live',action='store_true')
    p.add_argument('--candidate',type=Path,default=DEFAULT_CANDIDATE)
    p.add_argument('--table-pose',type=Path,default=DEFAULT_TABLE)
    p.add_argument('--backend',choices=['npu','cpu','hsv'],default='hsv')
    p.add_argument('--preprocess',choices=['host','board-cpu','fpga'],default='host')
    p.add_argument('--endpoint');p.add_argument('--fpga-url',default='http://127.0.0.1:18862/preprocess')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();result=prepare(args)
    print(json.dumps(result,indent=2));return 0 if result['completed'] else 1


if __name__=='__main__':raise SystemExit(main())

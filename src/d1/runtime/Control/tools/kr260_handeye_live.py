#!/usr/bin/env python3
"""Read-only RGB hand-eye observations from the existing KR260 preview.

Does not move the robot, restart cameras, open USB devices, require sudo, or alter
D2C/FPGA capture. RGB and joint stationarity are checked separately; exposure and
robot feedback are NOT synchronized. Export requires measured mapping evidence.
"""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import time
import urllib.request

import cv2
import numpy as np
import kr260_handeye as helper

CONTROL = Path(__file__).resolve().parents[1]
DEFAULT_BOARD = CONTROL/'calibration/kr260/board.a4_chessboard.json'


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def geometry(board):
    cols, rows, size = board['inner_columns'], board['inner_rows'], board['square_size_m']
    if board.get('type') != 'chessboard' or type(cols) is not int or type(rows) is not int or not 3 <= cols <= 30 or not 3 <= rows <= 30:
        raise ValueError('Measured chessboard dimensions required')
    if type(size) not in (int, float) or not math.isfinite(size) or not .001 < size < .1:
        raise ValueError('Measured square size in metres required')
    obj = np.array([[x*size,y*size,0] for y in range(rows) for x in range(cols)], np.float64)
    return (cols, rows), obj


def inspect(data, board):
    if data.get('schema') != 'kr260-color-handeye-v1' or data['device']['serial'] != board['camera_serial']:
        raise ValueError('Unexpected snapshot schema/camera serial')
    age = data['board_frame_age_ms']
    if not isinstance(age, (int,float)) or not math.isfinite(age) or not 0 <= age <= 250:
        raise ValueError('Stale camera frame')
    raw = base64.b64decode(data['image_base64'], validate=True)
    if hashlib.sha256(raw).hexdigest() != data['image_sha256']:
        raise ValueError('Image checksum mismatch')
    image = cv2.imdecode(np.frombuffer(raw,np.uint8), cv2.IMREAD_COLOR)
    cal, frame = data['calibration'], data['frame']
    if image is None or image.shape != (cal['height'],cal['width'],3) or (frame['width'],frame['height']) != (cal['width'],cal['height']):
        raise ValueError('Frame/calibration dimensions differ')
    if cal.get('mirrored') is not False or cal.get('distortion_order') != 'k1,k2,p1,p2,k3,k4,k5,k6':
        raise ValueError('Unverified mirroring/distortion convention')
    k=np.asarray(cal['camera_matrix'],np.float64);d=np.asarray(cal['distortion'],np.float64)
    if k.shape != (3,3) or d.shape != (8,) or not np.isfinite(k).all() or not np.isfinite(d).all() or min(k[0,0],k[1,1])<=0:
        raise ValueError('Invalid intrinsics')
    pattern,obj=geometry(board)
    ok,corners=cv2.findChessboardCornersSB(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),pattern,cv2.CALIB_CB_NORMALIZE_IMAGE|cv2.CALIB_CB_EXHAUSTIVE)
    if not ok:raise ValueError('Full chessboard not detected')
    ok,rv,tv=cv2.solvePnP(obj,corners,k,d)
    if not ok:raise ValueError('PnP failed')
    rotation=cv2.Rodrigues(rv)[0]
    if np.any((obj@rotation.T+tv.reshape(1,3))[:,2]<=0):raise ValueError('Board behind camera')
    projected=cv2.projectPoints(obj,rv,tv,k,d)[0]
    rms=float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=2))))
    if not math.isfinite(rms) or rms>1:raise ValueError(f'Reprojection RMS >1 pixel: {rms}')
    pose=np.eye(4);pose[:3,:3]=rotation;pose[:3,3]=tv.ravel()
    annotated=image.copy();cv2.drawChessboardCorners(annotated,pattern,corners,True)
    origin=tuple(np.rint(corners[0,0]).astype(int));cv2.putText(annotated,'O',origin,cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
    return dict(raw=raw,corners=corners,pose=pose,reprojection_rms_px=rms,annotated=annotated)


def capture(args):
    board=json.loads(args.board.read_text());geometry(board)
    quantized=getattr(args,'sampling_mode','strict')=='quantized'
    if quantized:
        from quantized_stationarity import assess_window as assess_quantized
        def assess(events,begin,end,final=False):
            return assess_quantized(events,begin,end,final=final)
    else:
        def assess(events,begin,end,final=False):
            return helper.assess_window(events,begin,end)
    args.output.mkdir(parents=True,exist_ok=False)
    out=args.output;observer=None
    result=dict(schema='kr260-live-quantized-paired-v1' if quantized else 'kr260-live-paired-v1',
                complete=False,robot_motion_commands_sent=False,
                handeye_calibrated=False,orientation_review_required=True,exposure_synchronized=False,
                uses_depth=False,uses_fpga=False,board_sha256=helper.digest(args.board))
    try:
        helper.check_link(args.nic)
        observer=helper.Observer(CONTROL/'build/arm_control',args.nic,out)
        observer.pump(2);begin=time.monotonic();observer.pump(1.8)
        assess(observer.events,begin,time.monotonic())
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        checked=[];metas=[];windows=[]
        for i in range(4):
            request_begin=time.monotonic()
            with opener.open(args.url.rstrip('/')+'/handeye-snapshot',timeout=3) as response:
                data=json.load(response)
            request_end=time.monotonic()
            if request_end-request_begin>1:raise ValueError('Camera request latency >1s')
            item=inspect(data,board);meta={k:v for k,v in data.items() if k!='image_base64'}
            (out/f'color_{i}.jpg').write_bytes(item['raw']);save(out/f'camera_{i}.json',meta)
            cv2.imwrite(str(out/f'corners_{i}.jpg'),item['annotated'])
            checked.append(item);metas.append(meta);windows.append([request_begin,request_end])
            observer.pump(1.0 if quantized else .3)
            assess(observer.events,begin,time.monotonic())
        observer.pump(1.1);end=time.monotonic()
        observation=assess(observer.events,begin,end,final=True)
        ids=[m['frame']['index'] for m in metas]
        if any(b<=a for a,b in zip(ids,ids[1:])):raise ValueError('Duplicate/reordered camera frames')
        if any(m['calibration']!=metas[0]['calibration'] for m in metas):raise ValueError('Intrinsics changed within observation')
        if windows[-1][0]-windows[0][1]<.6:raise ValueError('Insufficient visual stationarity interval')
        shifts=[float(np.max(np.linalg.norm(c['corners']-checked[0]['corners'],axis=2))) for c in checked]
        visual_limit=.5 if quantized else .75
        if max(shifts)>visual_limit:raise ValueError(f'Board moving or corner ordering changed (>{visual_limit} px)')
        if quantized:
            centers=np.array([c['corners'].reshape(-1,2).mean(axis=0) for c in checked])
            center_shift=float(np.max(np.linalg.norm(centers-centers[0],axis=1)))
            if center_shift>.15:raise ValueError('Board center drift >0.15 px')
            result.update(visual_corner_shift_limit_px=.5,visual_center_shift_limit_px=.15,
                          visual_center_shift_px=center_shift,sampling_mode='quantized',
                          quality_assessor_sha256=helper.digest(Path(__file__).with_name('quantized_stationarity.py')))
        observer.close();observer=None
        paths=['feedback.jsonl']+[f'{kind}_{i}.{ext}' for i in range(4) for kind,ext in [('color','jpg'),('camera','json')]]
        result.update(complete=True,observation=observation,request_windows=windows,
                      T_camera_target=checked[-1]['pose'].tolist(),reprojection_rms_px=checked[-1]['reprojection_rms_px'],
                      camera_serial=board['camera_serial'],frame_indices=ids,visual_max_corner_shift_px=max(shifts),
                      selected_image='color_3.jpg',raw_sha256={name:helper.digest(out/name) for name in paths})
        save(out/'orientation_review.json',dict(reviewed=False,image_sha256=metas[-1]['image_sha256'],
                                               physical_origin_label='',reviewer='',note='Compare O with the same physical marked corner in every sample.'))
    except Exception as error:
        result['error']=str(error)
    finally:
        if observer is not None:observer.close()
        save(out/'capture.json',result)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['complete'] else 1


def export(args):
    board=json.loads(args.board.read_text());geometry(board)
    helper.verified_evidence(board,args.board.parent)
    if board.get('fixed_board_and_rigid_camera_confirmed') is not True:
        raise ValueError('Fixed board and rigid camera mounting evidence required')
    if len(args.sessions)<8 or len({p.resolve() for p in args.sessions})!=len(args.sessions):
        raise ValueError('At least 8 unique poses required')
    pairs=[];origin=None;medians=[];references=[]
    for folder in args.sessions:
        record=json.loads((folder/'capture.json').read_text())
        if record.get('schema')!='kr260-live-paired-v1' or not record.get('complete') or record['board_sha256']!=helper.digest(args.board):
            raise ValueError('Incomplete/incompatible capture: '+str(folder))
        expected={'feedback.jsonl'}|{f'{kind}_{i}.{ext}' for i in range(4) for kind,ext in [('color','jpg'),('camera','json')]}
        if set(record['raw_sha256'])!=expected:raise ValueError('Missing source evidence')
        for name,sha in record['raw_sha256'].items():
            if helper.digest(folder/name)!=sha:raise ValueError('Changed evidence: '+name)
        events=[]
        for raw in map(json.loads,(folder/'feedback.jsonl').read_text().splitlines()):
            event=helper.parse_feedback(raw['line'],raw['t'])
            if event:events.append(event)
        obs=record['observation'];begin=obs['begin_aipc_monotonic_s'];end=obs['end_aipc_monotonic_s']
        checked_obs=helper.assess_window(events,begin,end)
        windows=record['request_windows']
        if len(windows)!=4 or any(not begin+1.5<=a<b<=end-1 for a,b in windows) or any(b-a>1 for a,b in windows):
            raise ValueError('Frame requests not enclosed by stationary feedback')
        visuals=[];metas=[]
        for i in range(4):
            meta=json.loads((folder/f'camera_{i}.json').read_text());metas.append(meta)
            visuals.append(inspect(dict(meta,image_base64=base64.b64encode((folder/f'color_{i}.jpg').read_bytes()).decode()),board))
        if any(b['frame']['index']<=a['frame']['index'] for a,b in zip(metas,metas[1:])) or any(m['calibration']!=metas[0]['calibration'] for m in metas):
            raise ValueError('Frame ordering/calibration mismatch')
        if windows[-1][0]-windows[0][1]<.6 or max(float(np.max(np.linalg.norm(v['corners']-visuals[0]['corners'],axis=2))) for v in visuals)>.75:
            raise ValueError('Visual stationarity failed')
        review=json.loads((folder/'orientation_review.json').read_text())
        if review.get('reviewed') is not True or review.get('image_sha256')!=metas[-1]['image_sha256'] or not review.get('reviewer') or not review.get('physical_origin_label'):
            raise ValueError('Per-image physical origin review required')
        if origin is not None and origin!=review['physical_origin_label']:raise ValueError('Mixed physical board origins')
        origin=review['physical_origin_label'];q=checked_obs['joint_median']
        if any(np.max(np.abs(np.asarray(q[:6])-np.asarray(previous[:6])))<.5 for previous in medians):raise ValueError('Duplicate/near-identical robot pose')
        medians.append(q);pairs.append((helper.mapped_pose(args.mapping,q),visuals[-1]['pose']))
        references.append(dict(session=str(folder.resolve()),capture_sha256=helper.digest(folder/'capture.json'),orientation_sha256=helper.digest(folder/'orientation_review.json')))
    # Reject rotationally degenerate samples before producing a solver input.
    rv=np.array([cv2.Rodrigues(pairs[0][0][:3,:3].T@a[:3,:3])[0].ravel() for a,_ in pairs[1:]])
    singular=np.linalg.svd(rv,compute_uv=False)
    if len(singular)<2 or singular[1]<np.deg2rad(5):raise ValueError('Insufficient multi-axis rotational diversity')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x'):pass
    fs=cv2.FileStorage(str(args.output),cv2.FILE_STORAGE_WRITE)
    fs.write('translation_unit','m');fs.write('board_type','chessboard');fs.write('base_frame','base_link');fs.write('end_frame','Link6')
    fs.write('board_columns',board['inner_columns']);fs.write('board_rows',board['inner_rows']);fs.write('square_size_m',board['square_size_m'])
    fs.startWriteStruct('samples',cv2.FileNode_SEQ)
    for a,b in pairs:
        fs.startWriteStruct('',cv2.FileNode_MAP);fs.write('T_base_end',a);fs.write('T_camera_target',b);fs.endWriteStruct()
    fs.endWriteStruct();fs.release()
    save(Path(str(args.output)+'.evidence.json'),dict(board_sha256=helper.digest(args.board),mapping_sha256=helper.digest(args.mapping),sessions=references,solver_input_sha256=helper.digest(args.output),verified=False))
    print('Exported measured pose pairs; independent hand-eye validation remains required.')
    return 0


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='mode',required=True)
    p=sub.add_parser('capture');p.add_argument('--url',default='http://127.0.0.1:18760');p.add_argument('--nic',default='enp99s0')
    p.add_argument('--sampling-mode',choices=['strict','quantized'],default='strict')
    p.add_argument('--board',type=Path,default=DEFAULT_BOARD);p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('export');p.add_argument('--board',type=Path,default=DEFAULT_BOARD)
    p.add_argument('--mapping',type=Path,default=Path('/home/aup/Camera/calibration/d1_joint_mapping.json'));p.add_argument('--output',type=Path,required=True);p.add_argument('sessions',type=Path,nargs='+')
    args=parser.parse_args()
    try:return capture(args) if args.mode=='capture' else export(args)
    except (ValueError,OSError,KeyError) as e:print('Rejected:',e);return 1
if __name__=='__main__':raise SystemExit(main())

#!/usr/bin/env python3
"""Run one saved, collision-checked empty-gripper observation/setup stage.

The default is an offline dry run. Only --execute opens cameras and the SDK CLI.
Only the selected joint receives `j` commands, including cancellation holds;
other axes and the gripper receive no position commands. All feedback, camera,
cancellation, drift and endpoint checks use the existing sdk_executor.execute.
"""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

import numpy as np

WS=Path(__file__).resolve().parents[1]
CONTROL=WS.parent
sys.path[:0]=[str(WS/'src/d1_pick_place'),str(CONTROL/'tools')]
from d1_pick_place.sdk_executor import CliTransport,Cancelled,LIMITS,Mapping,execute,healthy,references

SEMANTICS='single_joint_command_no_updates_to_other_axes'


class SingleJointTransport(CliTransport):
    def __init__(self,binary,nic,output,joint):
        if joint not in (0,1,4):raise ValueError('Only J0/J4 observation or J1 setup commands supported')
        self.joint=joint
        super().__init__(binary,nic,output)

    def send(self,q,check):
        check()
        values=np.asarray(q,dtype=float)
        if values.shape!=(7,):raise ValueError('Seven-axis executor reference required')
        target=float(values[self.joint])
        if not math.isfinite(target) or not LIMITS[self.joint,0]<=target<=LIMITS[self.joint,1]:
            raise ValueError('Selected joint target outside limits')
        # Log exactly the transmitted precision; None explicitly means no update.
        target=round(target,5)
        command=f'j {self.joint} {target:.5f}\n'
        sdk=[None]*7;sdk[self.joint]=target
        with self.lock:
            before=self.acks
            self.log.write(json.dumps(dict(t=self.now(),kind='command',sdk=sdk,
                joint=self.joint,commanded_joint_mask=[i==self.joint for i in range(7)],
                action_semantics=SEMANTICS,cli_command=command.strip()))+'\n')
            self.log.flush()
            self.proc.stdin.write(command);self.proc.stdin.flush();self.commands+=1
        until=self.now()+.6
        while True:
            check()
            with self.lock:
                if self.acks>before:return
            if self.now()>until:raise TimeoutError('Receipt timeout; no next command')
            self.wait(.01)


def validate_axis_plan(plan,source,joint,expected):
    expected=np.asarray(expected,dtype=float)
    if expected.shape!=(7,) or not np.isfinite(expected).all() or np.any(expected<LIMITS[:,0]) or np.any(expected>LIMITS[:,1]):
        raise ValueError('Seven finite reviewed angles within SDK limits required')
    if joint not in (0,1,4) or source.get('joint')!=joint or source.get('held') is not False:
        raise ValueError('Source must be an unheld plan for the requested J0/J1/J4')
    if not 35<=expected[6]<=37:
        raise ValueError('Reviewed gripper feedback is outside the open observation range')
    q=np.asarray(plan['sdk'],dtype=float)
    if q.ndim!=2 or q.shape[1]!=6 or len(q)<2 or not np.isfinite(q).all():
        raise ValueError('Malformed SDK plan')
    if plan['active_joints']!=[joint] or np.max(np.ptp(q[:,[i for i in range(6) if i!=joint]],axis=0))>1e-7:
        raise ValueError('Only the selected joint may vary in the source path')
    if np.max(abs(q[:,joint]-q[0,joint]))>3.5+1e-7 or abs(q[-1,joint]-q[0,joint])<.3:
        raise ValueError('Selected-axis displacement must be useful and at most 3.5 degrees')
    if joint==1 and (source.get('setup_only') is not True or
        source.get('finger_clearance_monotonic_increase') is not True or q[-1,1]>=q[0,1]):
        raise ValueError('J1 is restricted to a reviewed setup plan with negative motion and increasing finger clearance')
    if np.max(abs(q[0]-expected[:6]))>.3+1e-5:
        raise ValueError('Reviewed expected angles differ from trajectory start by >0.3 degrees')
    for key,index in [('sdk_start',0),('sdk_target',-1)]:
        saved=np.asarray(source[key],dtype=float)
        if saved.shape!=(7,) or not np.isfinite(saved).all() or np.max(abs(saved[:6]-q[index]))>1e-5:
            raise ValueError('Source SDK endpoints do not match mapped trajectory')
        if abs(saved[6]-expected[6])>.3+1e-5:
            raise ValueError('Reviewed open gripper differs from source observation')
    if abs(source['sdk_target'][6]-source['sdk_start'][6])>1e-7:
        raise ValueError('Source must not request a gripper position change')
    if (plan['rate']!=5. or not 0<plan['max_velocity_deg_s']<=.5 or
        not 0<plan['max_acceleration_deg_s2']<=1. or
        abs(plan['duration_s']-20.)>1e-7 or abs(source['duration_s']-20.)>1e-7):
        raise ValueError('Expected 20-second plan at 5Hz, speed <=0.5 and acceleration <=1')
    refs=list(references(plan));times=np.asarray([t for t,_ in refs]);values=np.asarray([v for _,v in refs])
    if len(refs)!=101 or not np.allclose(np.diff(times),.2,atol=1e-9,rtol=0):
        raise ValueError('Reference schedule must retain the reviewed 101 ticks')
    if source.get('checked_reference_count')!=len(refs):
        raise ValueError('Source collision-check count does not match execution references')
    velocity=np.diff(values,axis=0)/np.diff(times)[:,None]
    acceleration=np.diff(velocity,axis=0)/np.diff(times)[1:,None]
    if np.max(abs(velocity))>.5+1e-7 or np.max(abs(acceleration))>1.+1e-7:
        raise ValueError('Reference derivative exceeds observation bounds')
    return dict(reference_count=len(refs),max_displacement_deg=float(np.max(abs(values[:,joint]-values[0,joint]))),
        reference_speed_max_deg_s=float(np.max(abs(velocity))),reference_acceleration_max_deg_s2=float(np.max(abs(acceleration))))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--expected-angles',type=float,nargs=7,required=True)
    p.add_argument('--joint',type=int,choices=(0,1,4),required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--candidate',type=Path)
    p.add_argument('--stop-file',type=Path)
    p.add_argument('--nic',default='enp99s0')
    p.add_argument('--execute',action='store_true')
    a=p.parse_args(argv)
    from rclpy.serialization import deserialize_message
    from moveit_msgs.msg import RobotTrajectory
    from d1_pick_place.configuration import load_config
    from d1_pick_place.hardware_bridge import trajectory_digest,trajectory_plan
    try:
        source_bytes=a.plan.read_bytes();source=json.loads(source_bytes)
        if source.get('completed') is not True:raise ValueError('Source planning did not complete')
        stages=source.get('stages',[])
        if len(stages)!=1 or stages[0].get('name')!='unheld_view' or stages[0].get('gripper_cad_position') is not None:
            raise ValueError('Exactly one unheld_view arm stage required')
        stage=stages[0]
        trajectory=deserialize_message(base64.b64decode(stage['trajectory_cdr_base64'],validate=True),RobotTrajectory)
        digest=trajectory_digest(trajectory)
        if digest!=stage['sha256']:raise ValueError('Stored trajectory digest mismatch')
        candidate=a.candidate or Path(load_config(WS/'src/d1_pick_place/config/first_pick.yaml')['hardware_mapping_candidate'])
        candidate_hash=hashlib.sha256(candidate.read_bytes()).hexdigest()
        if source.get('source_hashes',{}).get(str(candidate.resolve()))!=candidate_hash:
            raise ValueError('Mapping candidate differs from the collision-checked source plan')
        mapping=Mapping.from_candidate(candidate)
        plan=trajectory_plan(trajectory,mapping,source['timing_profile'])
        bounds=validate_axis_plan(plan,source,a.joint,a.expected_angles)
        mask=[i==a.joint for i in range(7)]
        report_plan=dict(stage='unheld_view',joint=a.joint,source_plan=str(a.plan.resolve()),
            source_plan_sha256=hashlib.sha256(source_bytes).hexdigest(),source_trajectory_sha256=digest,
            candidate=str(candidate.resolve()),candidate_sha256=candidate_hash,
            expected_angles=a.expected_angles,sdk_plan=plan,commanded_joint_mask=mask,
            action_semantics=SEMANTICS,bounds=bounds,
            stop_file=None if a.stop_file is None else str(a.stop_file.resolve()),
            automatic_next_stage=False,automatic_return=False,gripper_commanded=False,
            gripper_target_changed=False,held=False,setup_only=source.get('setup_only',False))
        a.output.mkdir(parents=True,exist_ok=False)
        (a.output/'plan.json').write_text(json.dumps(report_plan,indent=2)+'\n')
    except (ValueError,KeyError,OSError) as error:p.error(str(error))
    print(json.dumps(dict(plan_file=str(a.output/'plan.json'),joint=a.joint,target_sdk=plan['sdk'][-1],
        duration_s=plan['duration_s'],commanded_joint_mask=mask,bounds=bounds)),flush=True)
    report=dict(stage='unheld_view',joint=a.joint,completed=False,executed=False,commands=0,receipts=0,
        hardware_commands_sent=False,session=None,progress=[],commanded_joint_mask=mask,
        action_semantics=SEMANTICS,gripper_commanded=False,gripper_target_changed=False)
    if not a.execute:
        report['plan_only']=True
        (a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        return 0
    from handeye_joint_step import Cameras
    transport=cameras=None;stopping=threading.Event();previous_handlers={}
    def canceled():return stopping.is_set() or (a.stop_file is not None and a.stop_file.exists())
    def check_cancel():
        if canceled():raise Cancelled('Stop file or process cancellation requested')
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):previous_handlers[sig]=signal.signal(sig,lambda *_:stopping.set())
        check_cancel();cameras=Cameras(a.output/'frames');deadline=time.monotonic()+5
        while not cameras.ready:
            check_cancel()
            if cameras.error or time.monotonic()>deadline:cameras.check()
            time.sleep(.05)
        cameras.check();check_cancel()
        transport=SingleJointTransport(CONTROL/'build/arm_control',a.nic,a.output/'transport.jsonl',a.joint)
        report['session']=dict(binary=str(CONTROL/'build/arm_control'),nic=a.nic,pid=transport.proc.pid,
            ack_mode='recv',position_command='j',joint=a.joint)
        deadline=time.monotonic()+5
        while True:
            check_cancel();cameras.check();snapshot=transport.snapshot()
            try:measured=healthy(snapshot,transport.now());break
            except RuntimeError:
                if snapshot['fault'] or time.monotonic()>deadline:raise
                transport.wait(.05)
        if any(abs(q-expected)>(.5 if i==6 else .3)+1e-5 for i,(q,expected) in enumerate(zip(measured,a.expected_angles))):
            raise ValueError('Fresh feedback differs from reviewed pose; no motion')
        report['measured_start']=measured.tolist();begin=transport.now();last_print=begin-5;last_sequence=-1
        def feedback(q):
            nonlocal last_print,last_sequence
            now=transport.now();current=transport.snapshot()
            if now-last_print>=5 and current['q_seq']!=last_sequence:
                last_print=now;last_sequence=current['q_seq']
                item=dict(elapsed_s=round(now-begin,3),sdk_feedback=q.tolist(),feedback_sequence=current['q_seq'],
                    feedback_age_s=now-current['q_time'],commands=transport.commands)
                report['progress'].append(item);print(json.dumps(item),flush=True)
        report['executed']=True
        result=execute(plan,transport,cancel=canceled,external_check=cameras.check,feedback=feedback)
        # execute uses J6 as a guard reference; this transport never sends it.
        result['gripper_reference_sdk']=result.pop('gripper_target_sdk')
        result.pop('gripper_target_source',None)
        report.update(result)
    except (Exception,KeyboardInterrupt) as error:
        report.update(error=str(error) or type(error).__name__,hold_command_received=getattr(error,'hold_command_received',False))
    finally:
        if transport is not None:
            snapshot=transport.snapshot()
            report.update(commands=transport.commands,receipts=transport.acks,hardware_commands_sent=transport.commands>0,
                final_angles=None if snapshot['q'] is None else snapshot['q'].tolist(),final_status=snapshot['status'])
            try:transport.close()
            except Exception as error:report.update(completed=False,transport_cleanup_error=str(error))
        if cameras is not None:
            try:cameras.close()
            except Exception as error:report.update(completed=False,camera_cleanup_error=str(error))
            report['camera_error']=cameras.error
        for sig,handler in previous_handlers.items():signal.signal(sig,handler)
        (a.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='progress'},indent=2),flush=True)
    return 0 if report['completed'] else 1


if __name__=='__main__':raise SystemExit(main())

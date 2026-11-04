#!/usr/bin/env python3
"""Plan the recorded 40 mm cube in an isolated graph; never loads hardware.

Finger opening and TCP are CAD assumptions. Successful planning is an offline
candidate, not permission to execute and not proof of the measured jaw width.
"""
import argparse
import base64
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation

WS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WS/'src/d1_pick_place'))
import rclpy
from rclpy.serialization import serialize_message
from moveit_msgs.msg import RobotState
from d1_pick_place.task import Task
from d1_pick_place.configuration import load_config
from d1_pick_place.sdk_executor import Mapping, NAMES
from d1_pick_place.hardware_bridge import trajectory_digest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pose', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--style', choices=['top', 'side'], default='top')
    args = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '79':
        p.error('This offline planner requires the separate ROS_DOMAIN_ID=79')
    listing = subprocess.run(['ros2','node','list','--no-daemon'],capture_output=True,text=True,timeout=10)
    if listing.returncode or listing.stdout.strip():
        raise RuntimeError('Offline domain 79 unavailable/in use; no process was replaced')
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(hardware_executed=False, human_grasp_approval_received=False,
                  source_pose=str(args.pose.resolve()), attempts=[], completed=False,
                  caveats=['nominal CAD TCP and finger positions',
                           'table and target collision geometry only; desk fixtures not reconstructed'])
    cfg = load_config(WS/'src/d1_pick_place/config/first_pick.yaml')
    cfg.update(grasp_style=args.style, side_pitch_deg=20.)
    report['timing_profile'] = cfg.get('timing_profile', 'collinear_bounded_v1')
    report['grasp_style'] = args.style
    pose = json.loads(args.pose.read_text())
    mapping = Mapping.from_candidate(cfg['hardware_mapping_candidate'])
    stream = (args.output/'moveit.log').open('w')
    process = subprocess.Popen(['ros2','launch','d1_pick_place','planning.launch.py'],
                               stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    node = None
    try:
        rclpy.init();node=Task(cfg,simulated=True)
        if not node.move.wait_for_server(timeout_sec=12.):
            raise RuntimeError('Offline planner did not start')
        for angle in [0,90,180,270]:
            cube=np.array(pose['T_base_cube']);table=np.array(pose['T_base_table'])
            cube[:3,:3] = cube[:3,:3] @ Rotation.from_euler('z',angle,degrees=True).as_matrix()
            state=RobotState()
            state.joint_state.name = NAMES + ['Joint7_1','Joint7_2']
            state.joint_state.position = mapping.to_model(pose['sdk_start']).tolist()+[cfg['gripper_open'],-cfg['gripper_open']]
            try:
                stages=node.plan(state,cube,table)
                saved=[]
                for stage in stages:
                    trajectory=stage['trajectory'];end=trajectory.joint_trajectory.points[-1].time_from_start
                    saved.append(dict(name=stage['name'],sha256=trajectory_digest(trajectory),
                        trajectory_cdr_base64=base64.b64encode(serialize_message(trajectory)).decode(),
                        scene_cdr_base64=base64.b64encode(serialize_message(stage['scene'])).decode(),
                        start_state_cdr_base64=base64.b64encode(serialize_message(stage['state'])).decode(),
                        gripper_cad_position=stage['gripper'],duration_s=end.sec+end.nanosec*1e-9))
                report.update(completed=True,selected_yaw_deg=angle,stages=saved)
                report['attempts'].append(dict(yaw_deg=angle,success=True))
                break
            except (ValueError,RuntimeError,TimeoutError) as error:
                report['attempts'].append(dict(yaw_deg=angle,error=str(error)))
                print('Candidate',angle,str(error),flush=True)
    finally:
        (args.output/'plan.json').write_text(json.dumps(report,indent=2))
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=3)
        stream.close()
    print(json.dumps({k:v for k,v in report.items() if k!='stages'},indent=2))
    return 0 if report['completed'] else 2


if __name__=='__main__':
    raise SystemExit(main())

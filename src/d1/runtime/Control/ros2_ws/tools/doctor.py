#!/usr/bin/env python3
"""Read-only deployment report; never tries to enable or move an actuator."""
from pathlib import Path
import importlib.util
import json
import argparse
from d1_pick_place.diagnostics import live_check
from d1_pick_place.configuration import load_config
ws = Path(__file__).resolve().parents[1]
c = load_config(ws/'src/d1_pick_place/config/task.yaml')
report = {'ros_distro': 'jazzy', 'model': 'Unitree CDN D1-550 gripper URDF',
          'cube_edge_m': c['cube_size_m'], 'place_marker': 'ArUco DICT_4X4_50 ID 23, 60 mm',
          'protected_motor_ids': [0, 1, 2, 3], 'sdk_modified': False,
          'modules': {x: importlib.util.find_spec(x) is not None for x in ['rclpy','moveit_msgs','control_msgs','cv2','cv_bridge']},
          'validation': {x: c[x] for x in ['hardware_validated','handeye_validated','joint_mapping_validated','tcp_validated','gripper_validated']},
          'execution_ready': False,
          'remaining': ['D1 motor execution/position loop still unverified',
                        'SDK J0..J5 -> URDF axes/zero offsets need measured mapping',
                        'Eye-in-hand extrinsics and actual TCP are not calibrated',
                        'Gripper SDK units, opening and force feedback need measured mapping',
                        'Table dimensions/obstacles and printed marker size need physical verification',
                        'No validated D1 FollowJointTrajectory / GripperCommand ROS driver is connected',
                        'Locked J0..J3 generally prevent the required arbitrary spatial pick/place motion']}
parser = argparse.ArgumentParser(description='只读实时诊断，不使能、不上电、不运动')
parser.add_argument('--nic')
parser.add_argument('--offline', action='store_true', help='仅检查本地软件配置，不读取机械臂')
args = parser.parse_args()
report['live'] = {'checked': False} if args.offline else live_check(ws.parent, args.nic)
print(json.dumps(report, indent=2, ensure_ascii=False))

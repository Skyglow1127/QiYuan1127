#!/usr/bin/env python3
"""Plot the exported MoveIt trajectory via URDF FK, never a hardware trace."""
from pathlib import Path
import base64
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from moveit_msgs.msg import RobotTrajectory
from d1_pick_place.kinematics import forward

ws = Path(__file__).resolve().parents[1]
cfg = ws/'src/d1_pick_place/config'
plan = json.loads((ws.parent/'logs/moveit_synthetic_plan.json').read_text())
fixture = json.loads((cfg/'synthetic_scene.json').read_text())
state = dict(zip([f'Joint{i}' for i in range(1, 7)], fixture['joints']))
state.update(Joint7_1=.02, Joint7_2=-.02)
fig = plt.figure(figsize=(10, 6), layout='constrained')
ax = fig.add_subplot(projection='3d')
colors = plt.cm.viridis(np.linspace(.05,.9,6))
i = 0
for step in plan['stages']:
    traj = deserialize_message(base64.b64decode(step['trajectory_ros_cdr_base64']), RobotTrajectory)
    points = []
    for point in traj.joint_trajectory.points:
        state.update(zip(traj.joint_trajectory.joint_names, point.positions))
        points.append(forward(cfg/'d1.urdf', state)[:3, 3])
    if len(traj.joint_trajectory.joint_names) == 6:
        xyz = np.array(points)
        ax.plot(*xyz.T, label=step['name'], color=colors[i], linewidth=2.3)
        i += 1
cube = np.array(fixture['cube'])[:3, 3]
marker = np.array(fixture['marker'])[:3, 3]
place = marker + [.1, 0, .0175]
ax.scatter(*cube, s=90, marker='s', color='#d83a3a', label='pick cube centre')
ax.scatter(*place, s=90, marker='s', color='#488c5b', label='place centre')
ax.scatter(*marker, s=70, marker='x', color='black', label='table marker')
ax.set(xlabel='Base X (m)', ylabel='Base Y (m)', zlabel='Base Z (m)',
       title='D1-T MoveIt planned TCP path\nSynthetic scene / all six joints in isolated planning / no hardware execution')
ax.view_init(elev=24, azim=-56)
ax.legend(loc='upper left', bbox_to_anchor=(.97,1), fontsize=9)
path=ws.parent/'logs/planned_path.png'
fig.savefig(path, dpi=160)
print(path)

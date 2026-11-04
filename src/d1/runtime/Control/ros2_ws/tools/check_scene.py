#!/usr/bin/env python3
"""Real MoveIt negative collision test; read/plan only on offline domain 77."""
import os
import json
from pathlib import Path
import numpy as np
import rclpy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents, RobotState
from d1_pick_place.task import Task, scene_for, box_object
from d1_pick_place.geometry import transform
from d1_pick_place.configuration import load_config

if os.environ.get('ROS_DOMAIN_ID') != '77': raise SystemExit('Use offline domain 77')
ws = Path(__file__).resolve().parents[1]
c = load_config(ws/'src/d1_pick_place/config/simulation.yaml')
data = json.loads((ws/'src/d1_pick_place/config/synthetic_scene.json').read_text())
rclpy.init()
node = Task(c, simulated=True)
try:
    client = node.create_client(GetPlanningScene, '/get_planning_scene')
    assert client.wait_for_service(timeout_sec=5)
    request = GetPlanningScene.Request()
    request.components.components = (PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
                                     PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                     PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
    actual = node.await_future(client.call_async(request), 5).scene
    assert not actual.world.collision_objects, 'Plan-only changed global world'
    assert not actual.robot_state.attached_collision_objects, 'Plan-only changed global attachment'
    state = RobotState()
    state.joint_state.name = c['arm_joints'] + ['Joint7_1', 'Joint7_2']
    state.joint_state.position = data['joints'] + [.02, -.02]
    scene = scene_for(c, np.array(data['cube']), np.array(data['marker']), state,
                      baseline_acm=actual.allowed_collision_matrix)
    scene.world.collision_objects.append(box_object('test_obstacle', c['base_frame'],
        transform(translation=[.28, .1, .13]), [.2, .2, .2]))
    try:
        node.plan_stage('collision_must_fail', state, None, scene, gripper=.02)
    except RuntimeError:
        print('PASS: collision blocked; plan-only left global world and attachments unchanged')
    else:
        raise AssertionError('Collision planning unexpectedly succeeded')
finally:
    node.destroy_node()
    rclpy.try_shutdown()

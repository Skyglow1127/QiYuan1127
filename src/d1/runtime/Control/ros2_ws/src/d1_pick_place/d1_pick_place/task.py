"""Staged MoveIt planning. No SDK imports, no motor command topic.

Every stage is planned before execution. The default deployment is planning-only;
execution requires a separately validated ROS controller and measured calibration.
"""
import argparse
import copy
import json
import os
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, JointConstraint, PositionConstraint, OrientationConstraint,
                             CollisionObject, AttachedCollisionObject, AllowedCollisionEntry,
                             RobotState, PlanningScene, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive
from rclpy.serialization import serialize_message
import base64
from .configuration import default_config, load_config
from .geometry import to_pose, from_pose, transform, validate_transform


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def validate_trajectory(traj, start, protected, tolerance, expected):
    jt = traj.joint_trajectory
    if traj.multi_dof_joint_trajectory.points:
        raise ValueError('Unexpected floating-base trajectory')
    if not jt.points or not jt.joint_names or len(set(jt.joint_names)) != len(jt.joint_names):
        raise ValueError('Empty or malformed trajectory')
    if set(jt.joint_names) != set(expected):
        raise ValueError('Planner returned unexpected/missing group joints')
    prior_time = -1.
    previous = None
    for point in jt.points:
        q = np.array(point.positions)
        t = seconds(point.time_from_start)
        if len(q) != len(jt.joint_names) or not np.isfinite(q).all() or t < 0 or t <= prior_time:
            raise ValueError('Invalid trajectory values/timestamps')
        for derivative in [point.velocities, point.accelerations, point.effort]:
            if derivative and (len(derivative) != len(q) or not np.isfinite(derivative).all()):
                raise ValueError('Invalid trajectory derivatives')
        state = dict(zip(jt.joint_names, q))
        for name in protected:
            if name in state and abs(state[name] - start[name]) > tolerance:
                raise ValueError(f'Protected joint moved in plan: {name}')
        if previous is None and any(abs(v - start[n]) > .002 for n, v in state.items()):
            raise ValueError('Trajectory does not start at supplied state')
        if previous is not None and np.max(abs(q - previous)) > .35:
            raise ValueError('Joint-space jump exceeds 0.35 rad')
        previous, prior_time = q, t
    return dict(zip(jt.joint_names, jt.points[-1].positions))


def box_object(name, frame, pose, size):
    obj = CollisionObject()
    obj.id, obj.header.frame_id = name, frame
    shape = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size)))
    obj.primitives, obj.primitive_poses, obj.operation = [shape], [to_pose(pose)], CollisionObject.ADD
    return obj


def scene_for(c, cube, marker, state, attached=False, baseline_acm=None):
    scene = PlanningScene(is_diff=True)
    scene.robot_state = copy.deepcopy(state)
    scene.robot_state.is_diff = False
    table = marker @ transform(translation=[*c['table_center_xy_m'], -c['table_size_m'][2] / 2 - .001])
    scene.world.collision_objects = [box_object('d1_task_table', c['base_frame'], table, c['table_size_m'])]
    if not attached:
        scene.world.collision_objects.append(box_object('d1_task_cube', c['base_frame'], cube, [c['cube_size_m']] * 3))
    else:
        scene.world.collision_objects.append(CollisionObject(id='d1_task_cube', operation=CollisionObject.REMOVE))
    # Permit finger contact with the selected object only, never table/other links.
    acm = copy.deepcopy(baseline_acm) if baseline_acm is not None else scene.allowed_collision_matrix
    for name in ['d1_task_cube'] + c['finger_links']:
        if name not in acm.entry_names:
            acm.entry_names.append(name)
            for row in acm.entry_values:
                row.enabled.append(False)
            acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
    for finger in c['finger_links']:
        i, j = acm.entry_names.index('d1_task_cube'), acm.entry_names.index(finger)
        acm.entry_values[i].enabled[j] = acm.entry_values[j].enabled[i] = True
    scene.allowed_collision_matrix = acm
    return scene


def task_poses(c, cube, marker):
    cube, marker = validate_transform(cube), validate_transform(marker)
    cube_in_marker = np.linalg.inv(marker) @ cube
    if abs(cube_in_marker[2, 3] - c['cube_size_m'] / 2) > .006:
        raise ValueError('Cube and marker are not on the same table')
    if cube[:3, 2] @ marker[:3, 2] < .99:
        raise ValueError('Cube normal inconsistent with table')
    place = marker @ transform(translation=[*c['place_offset_m'], c['cube_size_m'] / 2 + c['place_release_gap_m']])
    return_to_pick = c.get('task_mode', 'transfer') == 'lift_return'
    if return_to_pick:
        place = cube.copy()
        place[:3, 3] += marker[:3, 2] * c['place_release_gap_m']
    for target in [cube_in_marker[:2, 3], np.array(c['place_offset_m'])]:
        if np.any(abs(target - c['table_center_xy_m']) + c['cube_size_m'] / 2 + .01 > np.array(c['table_size_m'][:2]) / 2):
            raise ValueError('Target outside configured table footprint')
    if not return_to_pick and np.linalg.norm(cube[:3, 3] - place[:3, 3]) < c['cube_size_m'] * 2:
        raise ValueError('Pick/place overlap')
    flip = transform(np.diag([1., -1., -1.]))  # TCP +Z approaches table.
    side_grasp = c.get('grasp_style', 'top') == 'side'
    if side_grasp:
        pitch = np.deg2rad(c.get('side_pitch_deg', 20.))
        flip = transform(np.array([[-1., 0., 0.],
            [0., np.sin(pitch), np.cos(pitch)], [0., np.cos(pitch), -np.sin(pitch)]]))
    grasp, release = cube @ flip, place @ flip
    pre, lift, above, retreat = [t.copy() for t in [grasp, grasp, release, release]]
    for t in [pre, lift, above, retreat]:
        t[:3, 3] += marker[:3, 2] * c['clearance_m']
    if side_grasp:
        pre[:3, 3] = grasp[:3, 3] - grasp[:3, 2] * c['clearance_m']
        retreat[:3, 3] = release[:3, 3] - release[:3, 2] * c['clearance_m']
    if return_to_pick:
        lift[:3, 3] = grasp[:3, 3] + marker[:3, 2] * c['lift_height_m']
        return place, [('pregrasp', pre, False), ('approach', grasp, True),
                       ('lift', lift, True), ('lower', release, True), ('retreat', retreat, True)]
    return place, [('pregrasp', pre, False), ('approach', grasp, True),
                   ('lift', lift, True), ('transfer', above, False),
                   ('lower', release, True), ('retreat', retreat, True)]


class Task(Node):
    def __init__(self, c, simulated=False):
        super().__init__('d1_pick_place_task')
        self.c, self.simulated = c, simulated
        self.observation = None
        self.joints = None
        self.joint_receive = 0.
        self.create_subscription(PoseArray, c['observations_topic'], self.on_observation, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.move = ActionClient(self, MoveGroup, c['move_action'])
        self.protected = {}  # Filled once from current measured state.
        self.history = []

    def on_observation(self, msg):
        if msg.header.frame_id == self.c['base_frame'] and len(msg.poses) == 2:
            self.observation = msg

    def on_joints(self, msg):
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            return
        if not np.isfinite(msg.position).all():
            return
        if not set(self.c['arm_joints'] + [self.c['gripper_joint']]).issubset(msg.name):
            return
        if seconds(msg.header.stamp) <= 0 or (self.joints is not None and seconds(msg.header.stamp) <= seconds(self.joints.header.stamp)):
            return
        self.joints, self.joint_receive = msg, time.monotonic()
        self.history.append((time.monotonic(), dict(zip(msg.name, msg.position))))
        self.history = self.history[-20:]

    def await_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
        if not future.done():
            raise TimeoutError('ROS request timed out; no automatic replay')
        if future.exception():
            raise future.exception()
        return future.result()

    def fresh_state(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.joints is None or time.monotonic() - self.joint_receive > 1.:
            raise ValueError('No recent joint feedback')
        if not 0 <= now - seconds(self.joints.header.stamp) <= 1.:
            raise ValueError('Stale/future joint state header')
        values = dict(zip(self.joints.name, self.joints.position))
        for name, value in self.protected.items():
            if abs(values[name] - value) > self.c['protected_tolerance_rad']:
                raise ValueError(f'Protected joint drift: {name}')
        return values

    def acquire(self):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            if self.observation is None or self.joints is None:
                continue
            age = self.get_clock().now().nanoseconds * 1e-9 - seconds(self.observation.header.stamp)
            if not 0 <= age <= self.c['max_frame_age_s']:
                continue
            values = self.fresh_state()
            # Stop-and-look: several distinct feedback frames, stable arm.
            recent = [q for ts, q in self.history if time.monotonic() - ts < 3.]
            if len(recent) < 3 or any(np.ptp([q[n] for q in recent]) > .001 for n in self.c['arm_joints']):
                continue
            self.protected = {n: values[n] for n in self.c['protected_joints']}
            state = RobotState()
            state.joint_state = copy.deepcopy(self.joints)
            return state, from_pose(self.observation.poses[0]), from_pose(self.observation.poses[1])
        raise TimeoutError('No fresh paired target + stationary joint state in 10 s')

    def constraints(self, pose, tolerance=.001):
        c = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.c['base_frame']
        position.link_name = self.c['tcp_link']
        position.weight = 1.
        position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance])]
        region_pose = to_pose(transform(translation=pose[:3, 3]))
        position.constraint_region.primitive_poses = [region_pose]
        orientation = OrientationConstraint()
        orientation.header.frame_id, orientation.link_name = self.c['base_frame'], self.c['tcp_link']
        orientation.orientation = to_pose(pose).orientation
        orientation.absolute_x_axis_tolerance = .01
        orientation.absolute_y_axis_tolerance = .01
        orientation.absolute_z_axis_tolerance = .01
        orientation.weight = 1.
        c.position_constraints, c.orientation_constraints = [position], [orientation]
        return c

    def plan_stage(self, name, state, target, scene, previous=None, straight=False, gripper=None, joint_target=None):
        c = self.c
        if not self.move.wait_for_server(timeout_sec=5.):
            raise TimeoutError('MoveIt /move_action unavailable')
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = c['arm_group'] if gripper is None else 'gripper'
        req.pipeline_id, req.planner_id = 'ompl', 'RRTConnectkConfigDefault'
        req.num_planning_attempts, req.allowed_planning_time = 3, c['planning_time_s']
        req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor = c['velocity_scaling'], c['acceleration_scaling']
        req.start_state = copy.deepcopy(state)
        req.start_state.is_diff = False
        req.workspace_parameters.header.frame_id = c['base_frame']
        req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -1.5
        req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z = 1.5
        if joint_target is not None:
            if gripper is not None or len(joint_target) != len(c['arm_joints']) or not np.isfinite(joint_target).all():
                raise ValueError('Six finite arm joint targets required')
            req.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
                joint_name=joint, position=float(value), tolerance_above=.0001,
                tolerance_below=.0001, weight=1.) for joint, value in zip(c['arm_joints'], joint_target)])]
        elif gripper is None:
            req.goal_constraints = [self.constraints(target)]
        else:
            req.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
                joint_name=c['gripper_joint'], position=gripper,
                tolerance_above=.0001, tolerance_below=.0001, weight=1.)])]
        for joint, value in self.protected.items():
            req.path_constraints.joint_constraints.append(JointConstraint(
                joint_name=joint, position=value, tolerance_above=c['protected_tolerance_rad'],
                tolerance_below=c['protected_tolerance_rad'], weight=1.))
        if straight and previous is not None:
            # Constrain the TCP to a narrow straight corridor and fixed orientation.
            # This uses OMPL constrained rejection sampling; failure is explicit.
            path = self.constraints(target)
            region = path.position_constraints[0].constraint_region
            delta = target[:3, 3] - previous[:3, 3]
            direction = delta / np.linalg.norm(delta)
            seed = np.eye(3)[np.argmin(np.abs(direction))]
            x = np.cross(seed, direction); x /= np.linalg.norm(x)
            axes = np.column_stack((x, np.cross(direction, x), direction))
            region.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                dimensions=[.004, .004, float(np.linalg.norm(delta) + .006)])]
            region.primitive_poses = [to_pose(transform(axes, (target[:3, 3] + previous[:3, 3]) / 2))]
            req.path_constraints.position_constraints = path.position_constraints
            req.path_constraints.orientation_constraints = path.orientation_constraints
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff = scene
        goal.planning_options.replan = False
        self.get_logger().info(f'Planning {name}')
        handle = self.await_future(self.move.send_goal_async(goal), 5)
        if not handle.accepted:
            raise RuntimeError(f'{name}: MoveIt rejected goal')
        try:
            result = self.await_future(handle.get_result_async(), c['planning_time_s'] + 15)
        except BaseException:
            self.await_future(handle.cancel_goal_async(), 3)
            raise
        if result.status != 4 or result.result.error_code.val != 1:
            raise RuntimeError(f'{name}: MoveIt error {result.result.error_code.val}; no trajectory executed')
        trajectory = result.result.planned_trajectory
        expected = c['arm_joints'] if gripper is None else [c['gripper_joint']]
        start_values = dict(zip(state.joint_state.name, state.joint_state.position))
        updates = validate_trajectory(trajectory, start_values, self.protected, c['protected_tolerance_rad'], expected)
        if gripper is None and c.get('hardware_mapping_candidate'):
            from .hardware_bridge import retime_trajectory
            from .sdk_executor import Mapping
            trajectory = retime_trajectory(trajectory, Mapping.from_candidate(c['hardware_mapping_candidate']),
                timing_profile=c.get('timing_profile', 'collinear_bounded_v1'),
                rate=c.get('rate_hz', 10.), speed=c.get('speed_deg_s', .5),
                acceleration=c.get('acceleration_deg_s2', 1.),
                respect_source_timing=c.get('respect_source_timing', True))
        new_state = copy.deepcopy(state)
        for i, joint in enumerate(new_state.joint_state.name):
            if joint in updates:
                new_state.joint_state.position[i] = updates[joint]
            if joint == 'Joint7_2' and c['gripper_joint'] in updates:
                new_state.joint_state.position[i] = -updates[c['gripper_joint']]
        return trajectory, new_state

    def plan(self, state, cube, marker):
        c = self.c
        self.initial_geometry = (cube.copy(), marker.copy())
        place, poses = task_poses(c, cube, marker)
        client = self.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=5.):
            raise TimeoutError('Cannot obtain collision matrix')
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        baseline_acm = self.await_future(client.call_async(request), 5).scene.allowed_collision_matrix
        stages, attached, previous = [], False, None
        def add(name, pose=None, straight=False, gripper=None):
            nonlocal state, previous
            scene = scene_for(c, cube, marker, state, attached, baseline_acm)
            before = copy.deepcopy(state)
            traj, state = self.plan_stage(name, state, pose, scene, previous, straight, gripper)
            stages.append({'name': name, 'trajectory': traj, 'state': before, 'scene': scene, 'gripper': gripper})
            if pose is not None:
                previous = pose
        add('open', gripper=c['gripper_open'])
        for name, pose, straight in poses:
            if name == 'lift':
                add('close', gripper=c['gripper_closed'])
                attachment = AttachedCollisionObject()
                attachment.link_name, attachment.touch_links = c['tcp_link'], c['finger_links']
                attachment.object = box_object('d1_task_cube', c['tcp_link'],
                    np.linalg.inv(previous) @ cube, [c['cube_size_m']] * 3)
                state.attached_collision_objects = [attachment]
                attached = True
            if name == 'retreat':
                # Keep object attached until release, then model it resting on table.
                add('release', gripper=c['gripper_open'])
                state.attached_collision_objects = []
                attached = False
                cube = place.copy()
                cube[:3, 3] -= marker[:3, 2] * c['place_release_gap_m']
            add(name, pose, straight)
        return stages

    def execute(self, stages):
        c = self.c
        checks = ['hardware_validated', 'handeye_validated', 'joint_mapping_validated', 'tcp_validated', 'gripper_validated']
        if self.simulated or not all(c.get(k) is True for k in checks):
            raise ValueError('Execution disabled: measured hardware/calibration validation is incomplete')
        if not 0 < c.get('grasp_min_effort', 0) < c['gripper_max_effort']:
            raise ValueError('A measured, bounded gripper force limit is required')
        # A fresh observation must still match the geometry used in planning.
        deadline = time.monotonic() + 3
        matched = False
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if self.observation is None:
                continue
            age = self.get_clock().now().nanoseconds * 1e-9 - seconds(self.observation.header.stamp)
            if not 0 <= age <= c['max_frame_age_s']:
                continue
            current = [from_pose(p) for p in self.observation.poses]
            matched = all(np.linalg.norm(a[:3, 3] - b[:3, 3]) < .005 and
                          np.linalg.norm(a[:3, :3] - b[:3, :3]) < .05
                          for a, b in zip(current, self.initial_geometry))
            if matched:
                break
        if not matched:
            raise ValueError('Target/marker changed or became stale after planning')
        executor = ActionClient(self, ExecuteTrajectory, c['trajectory_action'])
        gripper = ActionClient(self, GripperCommand, c['gripper_action'])
        apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not executor.wait_for_server(timeout_sec=3.) or not gripper.wait_for_server(timeout_sec=3.) or not apply_scene.wait_for_service(timeout_sec=3.):
            raise TimeoutError('Validated ROS trajectory/gripper controllers and planning scene service required')
        for stage in stages:
            actual = self.fresh_state()
            expected = dict(zip(stage['state'].joint_state.name, stage['state'].joint_state.position))
            if any(abs(actual[n] - expected[n]) > .01 for n in c['arm_joints']):
                raise ValueError('Arm state changed after planning; abort without replay')
            # Apply world/attachment changes only; never overwrite measured joint state.
            scene = copy.deepcopy(stage['scene'])
            scene.robot_state.joint_state = JointState()
            scene.robot_state.is_diff = True
            if not scene.robot_state.attached_collision_objects:
                removed = AttachedCollisionObject(link_name=c['tcp_link'])
                removed.object.id, removed.object.operation = 'd1_task_cube', CollisionObject.REMOVE
                scene.robot_state.attached_collision_objects = [removed]
            if not self.await_future(apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)), 3).success:
                raise RuntimeError('Scene update rejected')
            if stage['gripper'] is not None:
                goal = GripperCommand.Goal()
                goal.command.position, goal.command.max_effort = stage['gripper'], c['gripper_max_effort']
                client = gripper
            else:
                goal = ExecuteTrajectory.Goal(trajectory=stage['trajectory'])
                client = executor
            handle = self.await_future(client.send_goal_async(goal), 3)
            if not handle.accepted:
                raise RuntimeError(f"{stage['name']}: execution rejected")
            future = handle.get_result_async()
            duration = seconds(stage['trajectory'].joint_trajectory.points[-1].time_from_start)
            deadline = time.monotonic() + max(10, duration + 5)
            try:
                while not future.done():
                    rclpy.spin_once(self, timeout_sec=.05)
                    self.fresh_state()  # Stale feedback/protected drift cancels active action.
                    if time.monotonic() > deadline:
                        raise TimeoutError('Execution timeout')
                result = future.result()
                if result.status != 4:
                    raise RuntimeError('Action did not succeed')
                if stage['gripper'] is None and result.result.error_code.val != 1:
                    raise RuntimeError('MoveIt execution failed')
                if stage['gripper'] is not None:
                    if stage['name'] == 'close':
                        if (not (result.result.reached_goal or result.result.stalled)
                                or not c['grasp_min_effort'] <= abs(result.result.effort) <= c['gripper_max_effort']
                                or abs(result.result.position - c['gripper_closed']) > .0005):
                            raise RuntimeError('Grasp contact/width/force not verified; no lift')
                    elif not result.result.reached_goal:
                        raise RuntimeError('Gripper did not reach open position')
                # Require THREE NEW feedback frames, not just an action receipt.
                target = dict(zip(stage['trajectory'].joint_trajectory.joint_names,
                                  stage['trajectory'].joint_trajectory.points[-1].positions))
                verified, last_stamp, stop = 0, seconds(self.joints.header.stamp), time.monotonic() + 3
                while verified < 3 and time.monotonic() < stop:
                    rclpy.spin_once(self, timeout_sec=.05)
                    values = self.fresh_state()
                    stamp = seconds(self.joints.header.stamp)
                    if stamp <= last_stamp:
                        continue
                    last_stamp = stamp
                    verified = verified + 1 if all(abs(values[n] - q) < (.0005 if n == c['gripper_joint'] else .005)
                                                   for n, q in target.items()) else 0
                if verified < 3:
                    raise RuntimeError('Position feedback failed closed-loop verification')
            except BaseException:
                self.await_future(handle.cancel_goal_async(), 3)
                raise
        self.verify_placement()
        self.get_logger().info('Placement verified by three new RGB-D observations')

    def verify_placement(self):
        c = self.c
        deadline, count = time.monotonic() + 8, 0
        last_stamp = seconds(self.observation.header.stamp) if self.observation else 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            self.fresh_state()
            if self.observation is None:
                continue
            stamp = seconds(self.observation.header.stamp)
            if stamp <= last_stamp:
                continue
            last_stamp = stamp
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if not 0 <= age <= c['max_frame_age_s']:
                continue
            cube, marker = [from_pose(p) for p in self.observation.poses]
            original_marker = self.initial_geometry[1]
            if np.linalg.norm(marker[:3, 3] - original_marker[:3, 3]) > .005:
                raise ValueError('Placement marker moved during execution')
            relative = np.linalg.inv(marker) @ cube
            half_margin = (np.array(c['place_region_m']) - c['cube_size_m']) / 2 - .005
            good = (np.all(abs(relative[:2, 3] - c['place_offset_m']) <= half_margin)
                    and abs(relative[2, 3] - c['cube_size_m'] / 2) < .006)
            count = count + 1 if good else 0
            if count >= 3:
                return
        raise RuntimeError('Motion ended but visual placement NOT verified; no automatic retry')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=default_config())
    parser.add_argument('--output', default='d1_plan.json')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--synthetic', help='Offline fixture JSON; only ROS domain 77, never executable')
    parser.add_argument('--simulation-unlocked', action='store_true', help='Offline domain 77 only, permit six joints in PLANNING')
    args, ros_args = parser.parse_known_args()
    c = load_config(args.config)
    simulated = bool(args.synthetic or args.simulation_unlocked)
    if simulated and (os.environ.get('ROS_DOMAIN_ID') != '77' or args.execute):
        parser.error('Simulation requires ROS_DOMAIN_ID=77 and forbids --execute')
    if args.execute and not all(c[k] for k in ['hardware_validated', 'handeye_validated', 'joint_mapping_validated', 'tcp_validated', 'gripper_validated']):
        parser.error('Hardware execution is disabled in this deployment; calibration/driver verification incomplete')
    rclpy.init(args=ros_args)
    node = Task(c, simulated)
    try:
        if args.synthetic:
            data = json.loads(Path(args.synthetic).read_text())
            state = RobotState()
            state.joint_state.name = c['arm_joints'] + ['Joint7_1', 'Joint7_2']
            state.joint_state.position = list(map(float, data['joints'])) + [c['gripper_open'], -c['gripper_open']]
            cube, marker = np.array(data['cube']), np.array(data['marker'])
            if not args.simulation_unlocked:
                node.protected = dict(zip(c['protected_joints'], state.joint_state.position[:4]))
        else:
            state, cube, marker = node.acquire()
        stages = node.plan(state, cube, marker)
        report = {'simulation': simulated, 'executed': False, 'protected_joints': list(node.protected),
                  'timing_profile': c.get('timing_profile', 'collinear_bounded_v1'),
                  'stages': [{'name': s['name'], 'points': len(s['trajectory'].joint_trajectory.points),
                              'duration_s': seconds(s['trajectory'].joint_trajectory.points[-1].time_from_start),
                              'trajectory_ros_cdr_base64': base64.b64encode(serialize_message(s['trajectory'])).decode()}
                             for s in stages]}
        Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
        node.get_logger().info(f'All {len(stages)} stages planned; saved {args.output}')
        if args.execute:
            node.execute(stages)
    except (ValueError, RuntimeError, TimeoutError) as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

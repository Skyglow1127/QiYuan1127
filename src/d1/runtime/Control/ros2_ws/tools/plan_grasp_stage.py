#!/usr/bin/env python3
"""Offline single-stage planning on isolated ROS domain 79; no hardware clients.

Approach offsets change only the TCP target, never the observed cube or table.
Lift records its actual-start TCP-to-cube transform. Lower must reuse that lift
plan with --attachment-plan, so an offset grasp is not mistaken for cube-centred.
Finger positions and the TCP are nominal CAD geometry, not measured calibration.
"""
import argparse
import base64
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys

import numpy as np

WS = Path(__file__).resolve().parents[1]
PACKAGE = WS / 'src/d1_pick_place'
sys.path.insert(0, str(PACKAGE))
from d1_pick_place.endpoint_control import ENDPOINT_RETRY_POLICY, checked_endpoint_policy


def validate_lift_height(value):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not 0 < value <= .10):
        raise ValueError('Lift distance must be positive and at most 0.10 m')
    return value


def stage_geometry(stage, cube, table, start_tcp, grasp_tcp, lift_height, clearance,
                   release_gap, attachment=None, keep_start_orientation=False):
    """Return TCP target and constant attached-object transform without mutating inputs."""
    normal = table[:3, 2]
    if stage == 'pregrasp':
        target = grasp_tcp.copy()
        target[:3, 3] += normal * clearance
        return target, None
    if stage == 'approach':
        return grasp_tcp.copy(), None
    if stage == 'lift':
        target = start_tcp.copy()
        target[:3, 3] += normal * validate_lift_height(lift_height)
        return target, np.linalg.inv(start_tcp) @ cube
    if stage == 'lower':
        if attachment is None:
            raise ValueError('Lower requires --attachment-plan from the preceding lift')
        placed = cube.copy()
        placed[:3, 3] += normal * release_gap
        if keep_start_orientation:
            target = start_tcp.copy()
            target[:3, 3] = placed[:3, 3] - start_tcp[:3, :3] @ attachment[:3, 3]
            return target, attachment.copy()
        return placed @ np.linalg.inv(attachment), attachment.copy()
    if stage == 'retreat':
        target = start_tcp.copy()
        target[:3, 3] += normal * clearance
        return target, None
    raise ValueError('Unsupported stage')


def simplify_collinear(trajectory, mapping, timing_profile='collinear_bounded_v1', **timing_options):
    """Remove only numerically collinear, ordered points; keep each edge <=20 deg."""
    from d1_pick_place.hardware_bridge import trajectory_plan, apply_plan_timing
    from d1_pick_place.sdk_executor import prepare
    full = trajectory_plan(trajectory, mapping, timing_profile, **timing_options)
    q = np.asarray(full['sdk'])
    keep = [0]
    max_residual = 0.
    while keep[-1] < len(q) - 1:
        start = keep[-1]
        end = start + 1
        for candidate in range(start + 2, len(q)):
            delta = q[candidate] - q[start]
            axis = int(np.argmax(abs(delta)))
            if abs(delta[axis]) > 20 or abs(delta[axis]) < 1e-12:
                break
            u = (q[start:candidate+1, axis] - q[start, axis]) / delta[axis]
            residual = float(np.max(abs(q[start:candidate+1] - (q[start] + u[:, None] * delta))))
            if residual > 1e-10 or np.min(np.diff(u)) < -1e-12:
                break
            end = candidate
            max_residual = max(max_residual, residual)
        keep.append(end)
    result = copy.deepcopy(trajectory)
    if len(keep) < len(q):
        result.joint_trajectory.points = [copy.deepcopy(trajectory.joint_trajectory.points[i]) for i in keep]
        jt = result.joint_trajectory
        # Old timestamps include dense-point stops; replace them after removing
        # redundant geometry, retaining all velocity/acceleration bounds.
        bounded = prepare(list(jt.joint_names), [list(p.positions) for p in jt.points],
                          [i / timing_options.get('rate', 10.) for i in range(len(jt.points))],
                          mapping, timing_profile=timing_profile, **timing_options)
        apply_plan_timing(result, bounded, mapping)
    bounded = trajectory_plan(result, mapping, timing_profile, **timing_options)
    return result, dict(original_points=len(q), kept_original_indices=keep,
                       max_line_residual_deg=max_residual, original_duration_s=full['duration_s'],
                       duration_s=bounded['duration_s'], geometry_preserved=True)


def cad_clearance(target_tcp, table, finger_position):
    from d1_pick_place.kinematics import forward
    urdf = PACKAGE / 'config/d1.urdf'
    values = {'Joint7_1': finger_position}
    model_tcp = forward(urdf, values)
    result = {}
    for link in ['Link7_1', 'Link7_2']:
        raw = (PACKAGE / 'meshes' / (link + '.STL')).read_bytes()
        count = struct.unpack_from('<I', raw, 80)[0]
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        vertices = np.ndarray((count,), dtype=dtype, buffer=raw, offset=84)['vertices'].reshape(-1, 3)
        tcp_finger = np.linalg.inv(model_tcp) @ forward(urdf, values, tip=link)
        local = vertices @ tcp_finger[:3, :3].T + tcp_finger[:3, 3]
        world = local @ target_tcp[:3, :3].T + target_tcp[:3, 3]
        result[link] = dict(extension_past_tcp_m=float(local[:, 2].max()),
                            minimum_table_plane_clearance_m=float(((world - table[:3, 3]) @ table[:3, 2]).min()))
    return dict(nominal_cad_only=True, finger_position_m=finger_position, fingers=result)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose', type=Path, required=True)
    parser.add_argument('--expected-angles', type=float, nargs=7, required=True,
                        help='Fresh supplied SDK feedback; this tool never queries hardware')
    parser.add_argument('--stage', choices=['pregrasp', 'fixed_pregrasp', 'approach', 'lift', 'lower', 'retreat'], required=True)
    parser.add_argument('--reference-plan', type=Path, help='Inherit chosen yaw/style from previous full plan')
    parser.add_argument('--attachment-plan', type=Path, help='Required for lower: preceding lift plan.json')
    parser.add_argument('--yaw-deg', type=float, help='Overrides reference selected yaw')
    parser.add_argument('--style', choices=['top', 'side'])
    parser.add_argument('--grasp-height-offset-m', type=float, default=0.)
    parser.add_argument('--lift-height-m', type=float, help='Single lift distance along table normal, at most 0.10 m')
    parser.add_argument('--retreat-height-m', type=float, help='Open-gripper retreat distance, at most 0.15 m')
    parser.add_argument('--gripper-cad-position', type=float, help='Nominal URDF finger displacement, not SDK J6')
    parser.add_argument('--keep-start-orientation', action='store_true',
                        help='Approach/lower keep current FK orientation; lower still places attached cube centre at original centre plus release gap')
    parser.add_argument('--no-simplify', action='store_true')
    parser.add_argument('--timing-profile', choices=['edge_quintic_v1', 'collinear_bounded_v1'],
                        default='collinear_bounded_v1', help='Original baseline or geometry-preserving continuous timing')
    parser.add_argument('--scheduling-policy', choices=['ack_then_period_v1', 'ack_overlap_v1', 'shared_progress_v1', 'timed_group_v1'],
                        default='ack_then_period_v1', help='Reviewed command-clock policy')
    parser.add_argument('--speed-deg-s', type=float, default=.5,
                        help='User requested arm reference speed; positive finite value')
    parser.add_argument('--acceleration-deg-s2', type=float, default=1.,
                        help='User requested arm reference acceleration; positive finite value')
    parser.add_argument('--rate-hz', type=float, default=10.,
                        help='Command sampling rate, positive and at most 10 Hz')
    parser.add_argument('--respect-source-timing', action='store_true',
                        help='Keep planner timestamps as lower duration bounds; default retimes geometry')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if (any(not math.isfinite(value) or value <= 0 for value in
            (args.speed_deg_s, args.acceleration_deg_s2, args.rate_hz)) or args.rate_hz > 10):
        parser.error('Speed/acceleration must be positive finite; rate must be in (0, 10] Hz')
    if os.environ.get('ROS_DOMAIN_ID') != '79':
        parser.error('Requires isolated ROS_DOMAIN_ID=79; no hardware graph permitted')
    if args.scheduling_policy in ('ack_overlap_v1', 'shared_progress_v1') and args.stage != 'fixed_pregrasp':
        parser.error('ACK overlap is restricted to fixed_pregrasp')

    import rclpy
    from rclpy.serialization import serialize_message
    from scipy.spatial.transform import Rotation
    from moveit_msgs.msg import RobotState, AttachedCollisionObject, PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene
    from d1_pick_place.configuration import load_config
    from d1_pick_place.geometry import validate_transform
    from d1_pick_place.kinematics import forward
    from d1_pick_place.task import Task, task_poses, scene_for, box_object
    from d1_pick_place.sdk_executor import (Mapping, NAMES, LIMITS,
        FEEDBACK_JOINT_TOLERANCE_DEG, nominal_planning_start)
    from d1_pick_place.hardware_bridge import trajectory_digest, trajectory_plan

    cfg = load_config(PACKAGE / 'config/first_pick.yaml')
    cfg['timing_profile'] = args.timing_profile
    cfg.update(speed_deg_s=args.speed_deg_s, acceleration_deg_s2=args.acceleration_deg_s2,
               rate_hz=args.rate_hz, respect_source_timing=args.respect_source_timing)
    execution_timing = dict(rate=args.rate_hz, speed=args.speed_deg_s,
                            acceleration=args.acceleration_deg_s2,
                            respect_source_timing=args.respect_source_timing)
    if args.lift_height_m is not None:
        try:
            cfg['lift_height_m'] = validate_lift_height(args.lift_height_m)
        except ValueError as error:
            parser.error(str(error))
    if args.retreat_height_m is not None:
        if not math.isfinite(args.retreat_height_m) or not 0 < args.retreat_height_m <= .15:
            parser.error('Retreat distance must be positive and at most 0.15 m')
        cfg['clearance_m'] = args.retreat_height_m
    report = dict(completed=False, hardware_executed=False, human_grasp_approval_received=False,
                  source_pose=str(args.pose.resolve()), sdk_start=args.expected_angles, stage=args.stage,
                  lift_height_m=cfg['lift_height_m'],
                  timing_profile=args.timing_profile,
                  scheduling_policy=args.scheduling_policy,
                  execution_timing=execution_timing,
                  caveats=['nominal CAD TCP and finger positions',
                           'caller supplies fresh feedback; no hardware connection by planner',
                           'table and cube only; desk fixtures not reconstructed'])
    if args.scheduling_policy=='timed_group_v1':
        report.update(position_delay_policy='timed_segment_interval',progress_policy='scheduled',
            endpoint_policy=None,gripper_mode='preserve')
    try:
        nominal_start = nominal_planning_start(args.expected_angles)
        report.update(sdk_measured_start=list(args.expected_angles),
                      sdk_nominal_start=nominal_start.tolist(),
                      feedback_joint_tolerance_deg=FEEDBACK_JOINT_TOLERANCE_DEG,
                      start_reference_policy='bounded_reference_with_separate_raw_feedback',
                      start_reference_adjustments=[dict(joint=i,measured_sdk=float(raw),
                          nominal_sdk=float(nominal),difference_deg=float(nominal-raw))
                          for i,(raw,nominal) in enumerate(zip(args.expected_angles,nominal_start))
                          if i<6 and abs(raw-nominal)>1e-9])
        if not math.isfinite(args.grasp_height_offset_m):
            raise ValueError('Non-finite grasp height offset')
        reference = json.loads(args.reference_plan.read_text()) if args.reference_plan else {}
        yaw = args.yaw_deg if args.yaw_deg is not None else reference.get('selected_yaw_deg', 0.)
        if not math.isfinite(yaw):
            raise ValueError('Non-finite yaw')
        cfg.update(grasp_style=args.style or reference.get('grasp_style', 'top'), side_pitch_deg=20.)
        finger = args.gripper_cad_position
        if finger is None:
            finger = cfg['gripper_closed'] if args.stage in ['lift', 'lower'] else cfg['gripper_open']
        if not math.isfinite(finger) or not 0 <= finger <= .03:
            raise ValueError('Nominal CAD finger joint outside [0, .03] metres')
        pose_bytes = args.pose.read_bytes()
        pose = json.loads(pose_bytes)
        cube = validate_transform(np.array(pose['T_base_cube'], dtype=float)).copy()
        table = validate_transform(np.array(pose['T_base_table'], dtype=float)).copy()
        if abs(pose.get('cube_size_m', cfg['cube_size_m']) - cfg['cube_size_m']) > 1e-9:
            raise ValueError('Source cube size differs from configured 40 mm cube')
        mapping_path = Path(cfg['hardware_mapping_candidate'])
        mapping = Mapping.from_candidate(mapping_path)
        model = mapping.to_model(nominal_start)
        values = dict(zip(NAMES, model))
        values['Joint7_1'] = finger
        start_tcp = forward(PACKAGE / 'config/d1.urdf', values)
        measured_values = dict(zip(NAMES, mapping.to_model(args.expected_angles)))
        measured_values['Joint7_1'] = finger
        report.update(T_base_measured_start_tcp=forward(
            PACKAGE / 'config/d1.urdf', measured_values).tolist(),
            T_base_nominal_start_tcp=start_tcp.tolist(),
            start_tcp_reference='nominal_bounded_planning_reference; raw feedback retained separately')
        # Yaw rotates the grasp candidate; the world collision cube stays exactly
        # at its supplied pose (90-degree cube symmetry does not require moving it).
        oriented = cube.copy()
        oriented[:3, :3] = oriented[:3, :3] @ Rotation.from_euler('z', yaw, degrees=True).as_matrix()
        _, poses = task_poses(cfg, oriented, table)
        grasp = next(target.copy() for name, target, _ in poses if name == 'approach')
        grasp[:3, 3] += table[:3, 2] * args.grasp_height_offset_m
        if args.keep_start_orientation and args.stage != 'pregrasp':
            grasp[:3, :3] = start_tcp[:3, :3]
        attachment = None
        if args.attachment_plan:
            prior = json.loads(args.attachment_plan.read_text())
            if not prior.get('completed') or prior.get('stage') != 'lift':
                raise ValueError('Attachment source must be a completed lift plan')
            attachment = validate_transform(np.array(prior['T_tcp_cube'], dtype=float))
            if not np.allclose(cube, np.asarray(prior['T_base_cube']), atol=1e-9, rtol=0):
                raise ValueError('Lower cube placement differs from attachment source')
        fixed_joint_target = None
        if args.stage == 'fixed_pregrasp':
            profile_path = WS.parents[2] / 'app/fixed_ready_pose.json'
            profile = json.loads(profile_path.read_text())
            fixed_joint_target = np.asarray(profile['model_target_rad'], dtype=float)
            fixed_values = dict(zip(NAMES, fixed_joint_target))
            fixed_values['Joint7_1'] = finger
            target = forward(PACKAGE / 'config/d1.urdf', fixed_values)
            report.update(fixed_ready_profile=profile, fixed_model_target_rad=fixed_joint_target.tolist(),
                          fixed_sdk_target_deg=mapping.to_sdk(fixed_joint_target).tolist(),
                          position_delay_policy='timed_segment_interval' if args.scheduling_policy=='timed_group_v1' else 'sdk_minimum',
                          progress_policy='scheduled' if args.scheduling_policy=='timed_group_v1' else 'feedback_paced',
                          endpoint_policy=None if args.scheduling_policy=='timed_group_v1' else checked_endpoint_policy(ENDPOINT_RETRY_POLICY),
                          collision_scene_scope='robot and calibrated static table; no live object position assumed')
        else:
            target, attachment = stage_geometry(args.stage, cube, table, start_tcp, grasp,
                cfg['lift_height_m'], cfg['clearance_m'], cfg['place_release_gap_m'], attachment,
                keep_start_orientation=args.keep_start_orientation)
        report.update(selected_yaw_deg=yaw, grasp_style=cfg['grasp_style'],
                      grasp_height_offset_m=args.grasp_height_offset_m,
                      keep_start_orientation=args.keep_start_orientation,
                      orientation_policy=('keep_actual_start_and_preserve_placed_cube_centre'
                          if args.stage == 'lower' and args.keep_start_orientation
                          else 'keep_actual_start' if args.keep_start_orientation and args.stage != 'pregrasp'
                          else 'existing_stage_target'),
                      T_base_cube=cube.tolist(), T_base_table=table.tolist(),
                      T_base_start_tcp=start_tcp.tolist(), T_base_target_tcp=target.tolist(),
                      T_tcp_cube=None if attachment is None else attachment.tolist(),
                      source_pose_sha256=hashlib.sha256(pose_bytes).hexdigest(),
                      mapping_candidate=str(mapping_path),
                      mapping_sha256=hashlib.sha256(mapping_path.read_bytes()).hexdigest(),
                      target_cad_clearance=cad_clearance(target, table, finger))
        listing = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], capture_output=True, text=True, timeout=10)
        if listing.returncode or listing.stdout.strip():
            raise ValueError('Offline domain 79 unavailable/in use; no process replaced')
        args.output.mkdir(parents=True, exist_ok=False)
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))

    stream = (args.output / 'moveit.log').open('w')
    process = node = None
    try:
        process = subprocess.Popen(['ros2', 'launch', 'd1_pick_place', 'planning.launch.py'],
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Task(cfg, simulated=True)
        if not node.move.wait_for_server(timeout_sec=12.):
            raise RuntimeError('Offline planner did not start')
        state = RobotState()
        state.joint_state.name = NAMES + ['Joint7_1', 'Joint7_2']
        state.joint_state.position = model.tolist() + [finger, -finger]
        node.protected = {name: values[name] for name in cfg['protected_joints']}
        if attachment is not None:
            held = AttachedCollisionObject()
            held.link_name, held.touch_links = cfg['tcp_link'], cfg['finger_links']
            held.object = box_object('d1_task_cube', cfg['tcp_link'], attachment, [cfg['cube_size_m']] * 3)
            state.attached_collision_objects = [held]
        client = node.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Cannot obtain collision matrix')
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = node.await_future(client.call_async(request), 5).scene.allowed_collision_matrix
        scene = scene_for(cfg, cube, table, state, attachment is not None, acm)
        if fixed_joint_target is not None:
            scene.world.collision_objects = [obj for obj in scene.world.collision_objects if obj.id != 'd1_task_cube']
        if np.linalg.norm(target[:3, 3] - start_tcp[:3, 3]) < 1e-6:
            raise ValueError('Stage displacement is below one micrometre; no motion plan required')
        trajectory, _ = node.plan_stage(args.stage, state, target, scene, previous=start_tcp,
            straight=args.stage not in ('pregrasp', 'fixed_pregrasp'), joint_target=fixed_joint_target)
        if not args.no_simplify:
            trajectory, report['simplification'] = simplify_collinear(
                trajectory, mapping, args.timing_profile, **execution_timing)
        bounded = trajectory_plan(trajectory, mapping, args.timing_profile, **execution_timing)
        if fixed_joint_target is not None and np.max(abs(np.asarray(bounded['sdk'][-1])-mapping.to_sdk(fixed_joint_target))) > .01:
            raise ValueError('Fixed joint goal was not preserved by the planner')
        report.update(completed=True, stages=[dict(name=args.stage, sha256=trajectory_digest(trajectory),
            trajectory_cdr_base64=base64.b64encode(serialize_message(trajectory)).decode(),
            scene_cdr_base64=base64.b64encode(serialize_message(scene)).decode(),
            start_state_cdr_base64=base64.b64encode(serialize_message(state)).decode(),
            gripper_cad_position=None, duration_s=bounded['duration_s'])])
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error) or type(error).__name__
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        stream.close()
        (args.output / 'plan.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'stages'}, indent=2))
    return 0 if report['completed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

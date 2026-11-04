#!/usr/bin/env python3
"""Offline collision-checked, open-gripper observation motion. No hardware API."""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import numpy as np

CONTROL = Path(__file__).resolve().parents[1]
WS = CONTROL / 'ros2_ws'
sys.path[:0] = [str(WS / 'src/d1_pick_place'), str(WS / 'tools')]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pose', type=Path, required=True)
    p.add_argument('--observation', type=Path, required=True)
    p.add_argument('--anchor', type=Path, required=True)
    p.add_argument('--joint', type=int, choices=[0, 1, 4], required=True)
    p.add_argument('--setup-retreat', action='store_true', help='J1 negative only: monotonically increase finger/table clearance before sampling')
    p.add_argument('--offset', type=float, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '79':
        p.error('Offline ROS domain 79 required')
    if not np.isfinite(a.offset) or abs(a.offset) > 3:
        p.error('Offsets must be within +/-3 degrees of the observed anchor')
    if (a.joint == 1) != a.setup_retreat or (a.setup_retreat and a.offset >= 0):
        p.error('J1 is allowed only for a negative setup retreat')

    import rclpy
    from rclpy.serialization import serialize_message
    from moveit_msgs.msg import RobotState, RobotTrajectory, PlanningSceneComponents
    from moveit_msgs.srv import GetStateValidity, GetPlanningScene, ApplyPlanningScene
    from trajectory_msgs.msg import JointTrajectoryPoint
    from d1_pick_place.task import Task, scene_for
    from d1_pick_place.configuration import load_config
    from d1_pick_place.sdk_executor import Mapping, NAMES, LIMITS, prepare, references
    from d1_pick_place.hardware_bridge import trajectory_digest, apply_plan_timing
    from d1_pick_place.kinematics import forward
    from plan_grasp_stage import cad_clearance
    from cube_mask_pose import projected_hull

    obs = json.loads(a.observation.read_text())
    anchor = json.loads(a.anchor.read_text())
    pose = json.loads(a.pose.read_text())
    if not pose.get('geometric_fit_accepted'):
        raise ValueError('An accepted observed cube pose is required')
    q = np.array(obs['sdk_angles'], float)
    base = np.array(anchor['sdk_angles'], float)
    if q.shape != (7,) or not np.isfinite(q).all() or np.any(q < LIMITS[:, 0]) or np.any(q > LIMITS[:, 1]):
        raise ValueError('Invalid current observation')
    if not 35 <= q[6] <= 37 or abs(q[6] - base[6]) > .5:
        raise ValueError('Open gripper state changed')
    other = [i for i in range(6) if i != a.joint]
    if max(abs(q[other] - base[other])) > .5 + 1e-5 or abs(q[a.joint] - base[a.joint]) > 3.5:
        raise ValueError('Observed pose is outside the small sampling envelope')
    if obs['status'] != dict(power_status=1, enable_status=1, error_status=0):
        raise ValueError('Observation was not healthy')
    target = q.copy()
    target[a.joint] = base[a.joint] + a.offset
    if abs(target[a.joint] - q[a.joint]) < .3:
        raise ValueError('No useful motion; do not create duplicate hold samples')
    if np.any(target < LIMITS[:, 0]) or np.any(target > LIMITS[:, 1]):
        raise ValueError('Target outside joint limits')
    listing = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], capture_output=True, text=True, timeout=10)
    if listing.returncode or listing.stdout.strip():
        raise ValueError('Offline domain 79 is occupied')
    cfg = load_config(WS / 'src/d1_pick_place/config/first_pick.yaml')
    candidate = Path(cfg['hardware_mapping_candidate'])
    mapping = Mapping.from_candidate(candidate)
    calibration = json.loads(candidate.read_text())
    bounded = prepare(NAMES, [mapping.to_model(q), mapping.to_model(target)], [0., 20.], mapping)
    all_refs = list(references(bounded))
    a.output.mkdir(parents=True, exist_ok=False)
    report = dict(completed=False, hardware_executed=False, stage='unheld_view', joint=a.joint,
                  anchor_offset_deg=a.offset, sdk_start=q.tolist(), sdk_target=target.tolist(),
                  timing_profile=bounded['timing_profile'], duration_s=bounded['duration_s'],
                  gripper_target_sdk=float(q[6]), held=False, held_basis='open gripper and observed cube on table',
                  source_pose=str(a.pose.resolve()), source_observation=str(a.observation.resolve()),
                  source_hashes={str(f.resolve()): hashlib.sha256(f.read_bytes()).hexdigest()
                                 for f in [a.pose, a.observation, a.anchor, candidate]},
                  sampling_bounds=dict(max_offset_deg=3, speed_deg_s=.5, acceleration_deg_s2=1, rate_hz=5),
                  setup_only=a.setup_retreat,
                  collision_scope=('whole robot, table, actual 40mm cube; finger/cube contact forbidden; monotonic upward clearance' if a.setup_retreat
                                   else 'whole robot, table, cube enlarged to 60mm; finger/cube contact forbidden'),
                  caveats=['Nominal local calibration and CAD; cables/camera bracket not fully modeled.',
                           'Current global and wrist images require review before physical acquisition.'])
    log = (a.output / 'moveit.log').open('x')
    process = node = None
    try:
        process = subprocess.Popen(['ros2', 'launch', 'd1_pick_place', 'planning.launch.py'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Task(cfg, simulated=True)
        valid = node.create_client(GetStateValidity, '/check_state_validity')
        if not valid.wait_for_service(timeout_sec=15):
            raise RuntimeError('No offline validity service')
        getter = node.create_client(GetPlanningScene, '/get_planning_scene')
        if not getter.wait_for_service(timeout_sec=5):
            raise RuntimeError('No offline scene service')
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = node.await_future(getter.call_async(req), 5).scene.allowed_collision_matrix

        def state(angles):
            s = RobotState()
            s.joint_state.name = NAMES + ['Joint7_1', 'Joint7_2']
            s.joint_state.position = mapping.to_model(angles).tolist() + [.02, -.02]
            return s

        scene = scene_for(cfg, np.array(pose['T_base_cube']), np.array(pose['T_base_table']), state(q), False, acm)
        for obj in scene.world.collision_objects:
            if obj.id == 'd1_task_cube':
                obj.primitives[0].dimensions = [.04, .04, .04] if a.setup_retreat else [.06, .06, .06]
        acm = scene.allowed_collision_matrix
        for finger in cfg['finger_links']:
            i, j = acm.entry_names.index('d1_task_cube'), acm.entry_names.index(finger)
            acm.entry_values[i].enabled[j] = acm.entry_values[j].enabled[i] = False
        apply = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not apply.wait_for_service(timeout_sec=5):
            raise RuntimeError('No offline scene apply service')
        req = ApplyPlanningScene.Request()
        req.scene = scene
        if not node.await_future(apply.call_async(req), 5).success:
            raise RuntimeError('Scene not applied')
        checks = []
        urdf = WS / 'src/d1_pick_place/config/d1.urdf'
        for elapsed, arm in all_refs:
            full = np.r_[arm, q[6]]
            req = GetStateValidity.Request()
            req.robot_state = state(full)
            req.group_name = ''  # Include fingers and all robot links.
            result = node.await_future(valid.call_async(req), 5)
            values = dict(zip(NAMES, mapping.to_model(full)))
            tcp = forward(urdf, values)
            clearance = cad_clearance(tcp, np.array(pose['T_base_table']), .02)
            minimum = min(v['minimum_table_plane_clearance_m'] for v in clearance['fingers'].values())
            camera = forward(urdf, values, 'Link6') @ np.array(calibration['T_end_camera'])
            hull = projected_hull(np.array(pose['T_base_cube']), camera,
                                  np.array(calibration['camera_matrix']), np.array(calibration['distortion']))
            in_frame = bool((hull.min(0) >= [8, 8]).all() and (hull.max(0) <= [632, 472]).all())
            check = dict(t=elapsed, valid=result.valid, finger_table_clearance_m=minimum, cube_in_frame=in_frame,
                         contacts=[dict(a=c.contact_body_1, b=c.contact_body_2, depth=c.depth) for c in result.contacts])
            checks.append(check)
            if not result.valid or minimum < .035 or not in_frame:
                raise ValueError('Unsafe/occluded sampling reference: ' + json.dumps(check))
            if a.setup_retreat and len(checks) > 1 and minimum < checks[-2]['finger_table_clearance_m'] - 1e-8:
                raise ValueError('Setup retreat does not monotonically increase finger/table clearance')
        if a.setup_retreat and checks[-1]['finger_table_clearance_m'] < .055:
            raise ValueError('Setup retreat would not provide sufficient final clearance')
        if a.setup_retreat:
            report['finger_clearance_monotonic_increase'] = True
        report.update(checked_reference_count=len(checks), min_finger_table_clearance_m=min(c['finger_table_clearance_m'] for c in checks))
        (a.output / 'collision_checks.json').write_text(json.dumps(checks, indent=2) + '\n')
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = NAMES
        for point in [q, target]:
            jp = JointTrajectoryPoint()
            jp.positions = mapping.to_model(point).tolist()
            trajectory.joint_trajectory.points.append(jp)
        apply_plan_timing(trajectory, bounded, mapping)
        report.update(completed=True, stages=[dict(name='unheld_view', sha256=trajectory_digest(trajectory),
            trajectory_cdr_base64=base64.b64encode(serialize_message(trajectory)).decode(),
            gripper_cad_position=None, duration_s=bounded['duration_s'])])
    except Exception as e:
        report['error'] = str(e)
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
                process.wait(timeout=3)
        log.close()
        (a.output / 'plan.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'stages'}), flush=True)
    return 0 if report['completed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())

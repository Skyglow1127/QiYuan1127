#!/usr/bin/env python3
"""Single 40 mm cube cycle with a reviewed path and automatic OpenCV evidence.

Planning never sends motion commands. One explicit approval binds target/path/
parameters; execution has no human gates, replanning or automatic enable.
Only an explicitly reviewed fixed-pose policy can reissue its approved endpoint.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
from grasp_timing import Trace, import_recognition, import_transport, summarize, load_rows, boot_id

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / 'ros2_ws'
PACKAGE = WS / 'src/d1_pick_place/d1_pick_place'
CONFIG = WS / 'src/d1_pick_place/config/first_pick.yaml'
PLANNER = WS / 'tools/plan_grasp_stage.py'
ARM = WS / 'tools/run_arm_stage.py'
GRIPPER = ROOT / 'tools/gripper_stage.py'
OBSERVER = ROOT / 'tools/observe_grasp.py'
sys.path.insert(0, str(ROOT.parents[1]))
from app.vision import verify_lift, verify_return
from app.execution_scope import FULL_CYCLE, PREGRASP, FIXED_PREGRASP, select_execution_stages
from d1_pick_place.sdk_executor import ARM_POSITION_TOLERANCE_DEG
from d1_pick_place.endpoint_control import ENDPOINT_RETRY_POLICY, checked_endpoint_policy
_STOP_FILE = None

ORDER = ['approach', 'close', 'lift', 'lower', 'open', 'retreat']
GRIPPER_STAGES = ('prepare_gripper', 'close', 'open')
LIMITS = dict(rate_hz=10., peak_velocity_deg_s=.5, peak_acceleration_deg_s2=1.,
              start_arm_tolerance_deg=ARM_POSITION_TOLERANCE_DEG,
              endpoint_tolerance_deg=ARM_POSITION_TOLERANCE_DEG, start_j6_tolerance=.5,
              max_lift_height_m=.10, gripper_single_segment_max_sdk=10.)
TARGET_RECHECK = dict(max_center_error_m=.005, max_yaw_error_deg_mod90=10.,
                      policy='fresh_live_recognition_before_first_motion')
ARM_EXECUTION_POLICY = 'frozen_approved_trajectory_v1'


def speed_arguments(spec):
    values = [spec['speed_deg_s'], spec['acceleration_deg_s2'], spec['gripper_speed_sdk_s'], spec['rate_hz']]
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) or x <= 0 for x in values):
        raise ValueError('Speed, acceleration and rate must be positive and finite')
    if spec['rate_hz'] > 10:
        raise ValueError('Command rate exceeds the supported 10 Hz transport cadence')
    return ['--speed-deg-s', spec['speed_deg_s'], '--acceleration-deg-s2', spec['acceleration_deg_s2'], '--rate-hz', spec['rate_hz']]

def approved_limits(spec):
    speed_arguments(spec)
    from d1_pick_place.sdk_executor import FEEDBACK_JOINT_TOLERANCE_DEG
    return dict(LIMITS, rate_hz=spec['rate_hz'], peak_velocity_deg_s=spec['speed_deg_s'],
                peak_acceleration_deg_s2=spec['acceleration_deg_s2'], gripper_speed_sdk_s=spec['gripper_speed_sdk_s'],
                feedback_joint_tolerance_deg=FEEDBACK_JOINT_TOLERANCE_DEG)

def check_stop():
    if _STOP_FILE is not None and _STOP_FILE.exists():
        raise KeyboardInterrupt('User requested cancellation; future commands stopped')


def stage_order(spec):
    if spec.get('fixed_ready'):
        return ['prepare_gripper', 'fixed_pregrasp']
    return ((['prepare_gripper'] if spec.get('include_prepare_gripper', False) else [])
            + (['pregrasp'] if spec.get('include_pregrasp', False) else []) + ORDER)


def read(path):
    return json.loads(Path(path).read_text())


def save(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_endpoint_policies(manifest):
    """Bind endpoint retries to the reviewed fixed stage and its exact source."""
    fixed = bool(manifest['spec'].get('fixed_ready'))
    top = checked_endpoint_policy(manifest.get('endpoint_policy'))
    if top is not None and not fixed:
        raise ValueError('Endpoint retry is restricted to fixed pregrasp')
    for stage in manifest['stages']:
        policy = checked_endpoint_policy(stage.get('endpoint_policy'))
        if stage['name'] in GRIPPER_STAGES:
            if policy is not None:
                raise ValueError('Endpoint retry cannot command the gripper')
            continue
        source = read(stage['plan_file'])
        source_policy = checked_endpoint_policy(source.get('endpoint_policy'))
        if policy != source_policy:
            raise ValueError('Approved endpoint policy differs from source plan')
        if policy is not None and (not fixed or stage['name'] != 'fixed_pregrasp'
                or stage.get('progress_policy') != 'feedback_paced'
                or source.get('progress_policy') != 'feedback_paced'):
            raise ValueError('Endpoint retry requires an approved feedback-paced fixed-pregrasp stage')
        if stage['name'] == 'fixed_pregrasp' and policy != top:
            raise ValueError('Fixed-pregrasp endpoint policy differs from reviewed manifest')


def validate_scheduling_policies(manifest):
    """The method's command clock must match the frozen source and stage."""
    expected = manifest['spec'].get('scheduling_policy', 'ack_then_period_v1')
    if expected not in ('ack_then_period_v1', 'ack_overlap_v1', 'shared_progress_v1', 'timed_group_v1'):
        raise ValueError('Unsupported scheduling policy')
    if expected in ('ack_overlap_v1', 'shared_progress_v1') and not manifest['spec'].get('fixed_ready'):
        raise ValueError('ACK overlap is restricted to fixed pregrasp')
    for stage in manifest['stages']:
        if stage['name'] in GRIPPER_STAGES:
            if stage.get('scheduling_policy', 'ack_then_period_v1') != 'ack_then_period_v1':
                raise ValueError('Arm scheduling policy cannot command the gripper')
            continue
        source = read(stage['plan_file'])
        if (stage.get('scheduling_policy', 'ack_then_period_v1') != expected
                or source.get('scheduling_policy', 'ack_then_period_v1') != expected):
            raise ValueError('Scheduling policy differs from reviewed source or manifest')
        if expected in ('ack_overlap_v1', 'shared_progress_v1') and (stage['name'] != 'fixed_pregrasp'
                or source.get('progress_policy') != 'feedback_paced'):
            raise ValueError('ACK overlap requires feedback-paced fixed pregrasp')


def stage_process_timeout(stage, duration):
    """Include reviewed endpoint seeking without changing other stage budgets."""
    extra_settle = 5 * len(stage.get('segments', [])) if stage['name'] == 'prepare_gripper' else 0
    policy = checked_endpoint_policy(stage.get('endpoint_policy'))
    if policy is not None and stage['name'] != 'fixed_pregrasp':
        raise ValueError('Endpoint retry budget is restricted to fixed pregrasp')
    traversal_budget=(4*duration+30 if stage.get('scheduling_policy')=='shared_progress_v1'
                      else duration)
    return traversal_budget + 45 + extra_settle + (policy['max_seek_s'] if policy else 0.)


def lift_height(spec):
    value = spec.get('lift_height_m')
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not 0 < value <= LIMITS['max_lift_height_m']):
        raise ValueError('Approved lift height must be finite and in (0, 0.10] m')
    return value


def recognition_contract(report_path, pose_path, mapping_path):
    """Bind a perception result to its actual consumed inputs and control mapping."""
    perception, pose = read(report_path), read(pose_path)
    if not perception.get('completed') or perception.get('pose_sha256') != digest(pose_path):
        raise ValueError('Recognition report does not bind this accepted pose')
    candidate = str(Path(mapping_path).resolve())
    if pose.get('candidate_sha256') != digest(candidate):
        raise ValueError('Perception candidate differs from planner mapping')
    if pose.get('source_kind') != perception.get('source_kind'):
        raise ValueError('Perception/pose source kind mismatch')
    dependencies = perception.get('dependencies')
    if not isinstance(dependencies, dict) or not dependencies:
        raise ValueError('Recognition dependency registry missing')
    verified = {}
    for path, expected in dependencies.items():
        resolved = str(Path(path).resolve())
        if digest(resolved) != expected:
            raise ValueError('Recognition dependency changed: '+resolved)
        verified[resolved] = expected
    table = str(Path(pose['source_table']).resolve())
    for path, expected in [(candidate, pose['candidate_sha256']),
                           (table, pose['source_table_sha256'])]:
        if verified.get(path) != expected:
            raise ValueError('Recognition registry does not bind candidate/table')
    observation = str(Path(pose['source_observation']).resolve())
    if verified.get(observation) != digest(observation):
        raise ValueError('Recognition registry does not bind source observation')
    for key in ['image_sha256', 'foreground_sha256']:
        if pose.get(key) not in verified.values():
            raise ValueError('Recognition registry does not bind '+key)
    refresh = dict(backend=perception['backend'], preprocess=perception['preprocessing'],
                   candidate=candidate, table_pose=table, endpoint=perception.get('endpoint'),
                   fpga_url=perception.get('fpga_url') or 'http://127.0.0.1:18862/preprocess')
    return perception, verified, refresh


def import_span_rows(trace, rows, prefix):
    """Import preparation/refresh spans without creating another trial start."""
    for row in rows:
        if row.get('boot_id') != trace.boot:
            raise ValueError('Preparation/refresh timing belongs to a different boot')
        if row.get('kind') != 'span':
            continue
        if not 0 <= row['start'] <= row['end'] <= trace.clock():
            raise ValueError('Invalid preparation/refresh span')
        trace.interval(row['category'], prefix+row['name'], row['start'], row['end'],
                       outcome=row.get('outcome', 'unknown'))


def capture_fresh_recognition(options):
    from prepare_accel_grasp import prepare as prepare_perception
    return prepare_perception(options)


def check_fresh_target(result, pose, manifest):
    """Accept the original approved target only; never silently retarget."""
    import numpy as np
    if (not result.get('completed') or result.get('source_kind') != 'live_camera'
            or result.get('robot_ready') is not True
            or result.get('execution_candidate_eligible') is not True
            or pose.get('geometric_fit_accepted') is not True):
        raise ValueError('Fresh recognition is not a ready live execution candidate')
    if manifest.get('target_recheck') != TARGET_RECHECK:
        raise ValueError('Missing or changed approved target recheck limits')
    approved = np.asarray(manifest['target']['T_base_cube'], float)
    fresh = np.asarray(pose['T_base_cube'], float)
    for matrix in [approved, fresh]:
        if (matrix.shape != (4,4) or not np.isfinite(matrix).all()
                or not np.allclose(matrix[3], [0,0,0,1], atol=1e-9)
                or not np.allclose(matrix[:3,:3].T@matrix[:3,:3], np.eye(3), atol=1e-5)
                or abs(np.linalg.det(matrix[:3,:3])-1) > 1e-5):
            raise ValueError('Invalid target transform')
    table = np.asarray(pose['T_base_table'], float)
    if table.shape != (4,4) or not np.allclose(table, manifest['target']['T_base_table'], atol=1e-9):
        raise ValueError('Fresh recognition changed the fixed table')
    center_error = float(np.linalg.norm(fresh[:3,3]-approved[:3,3]))
    relative = approved[:3,:3].T@fresh[:3,:3]
    yaw = math.degrees(math.atan2(relative[1,0], relative[0,0]))
    yaw_error = abs((yaw+45)%90-45)
    if center_error > TARGET_RECHECK['max_center_error_m']+1e-9:
        raise ValueError('Fresh cube moved beyond approved 5 mm target tolerance')
    if yaw_error > TARGET_RECHECK['max_yaw_error_deg_mod90']+1e-9:
        raise ValueError('Fresh cube yaw changed beyond approved 10 degree tolerance')
    # Both estimates use the same fixed plane; a changed normal is inconsistent.
    if not np.allclose(approved[:3,2], fresh[:3,2], atol=1e-6):
        raise ValueError('Fresh cube normal changed')
    return dict(center_error_m=center_error, yaw_error_deg_mod90=yaw_error,
                target_updated=False, limits=TARGET_RECHECK)


def recheck_target(manifest, folder, trace):
    """One fresh observation replaces first-stage observe(), with no commands."""
    config = manifest['recognition_refresh']
    output = Path(folder)/'target_recheck'
    options = SimpleNamespace(capture_live=True, observation=None, output=output,
        backend=config['backend'], preprocess=config['preprocess'],
        candidate=Path(config['candidate']), table_pose=Path(config['table_pose']),
        endpoint=config.get('endpoint'), fpga_url=config['fpga_url'])
    result = capture_fresh_recognition(options)
    # Preserve timing even when a failed/multiple/stale target stops execution.
    if (output/'timing.jsonl').exists():
        import_span_rows(trace, load_rows(output/'timing.jsonl'), 'target_recheck_')
    if not result.get('completed'):
        raise ValueError('Fresh target recognition failed: '+str(result.get('error')))
    pose_path = Path(result['pose'])
    result, _, _ = recognition_contract(output/'result.json', pose_path, config['candidate'])
    pose = read(pose_path)
    comparison = check_fresh_target(result, pose, manifest)
    observation = Path(pose['source_observation'])
    observed = read(observation)
    if observed.get('hardware_commands_sent') != 0:
        raise ValueError('Fresh observation sent hardware commands')
    save(output/'approved_target_comparison.json', comparison)
    return observed['sdk_angles'], observation, comparison


def vector(values, length):
    if len(values) != length or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values):
        raise ValueError(f'Expected {length} finite values')
    return values


def check_start(actual, reviewed, arm_tolerance=ARM_POSITION_TOLERANCE_DEG):
    vector(actual, 7); vector(reviewed, 7)
    errors=[]
    for i,(a,b) in enumerate(zip(actual,reviewed)):
        tolerance=LIMITS['start_j6_tolerance'] if i==6 else arm_tolerance
        if abs(a-b)>tolerance+1e-5:
            unit=' SDK' if i==6 else '°'
            errors.append(f'J{i} 实测 {a:.3f}{unit}，轨迹起点 {b:.3f}{unit}，差值 {abs(a-b):.3f}{unit}超过 {tolerance:g}{unit}')
    if errors:
        raise ValueError('Fresh state differs from approved stage start: '+'；'.join(errors))


def check_path(candidate, approved, envelope=.3):
    """Conservative ordered endpoint/edge check against the reviewed polyline.

Each new edge must fit one original segment; this rejects corner shortcuts even
when its endpoints lie on the approved path. The envelope is part of approval.
"""
    q, old = candidate['sdk'], approved['sdk']
    if len(q) < 2 or len(old) < 2:
        raise ValueError('Empty trajectory')
    for row in q + old:
        vector(row, 6)
    tol = envelope + 1e-5
    if max(abs(a-b) for a, b in zip(q[0], old[0])) > tol or max(abs(a-b) for a, b in zip(q[-1], old[-1])) > tol:
        raise ValueError('Replanned path endpoints outside approved envelope')
    edge = 0
    for a, b in zip(q[:-1], q[1:]):
        found = False
        for i in range(edge, len(old)-1):
            start, end = old[i], old[i+1]
            d = [y-x for x, y in zip(start, end)]
            norm = sum(x*x for x in d)
            def residual(p):
                u = 0. if norm == 0 else sum((x-y)*z for x, y, z in zip(p, start, d))/norm
                u = max(0., min(1., u))
                return max(abs(x-y-u*z) for x, y, z in zip(p, start, d)), u
            ra, ua = residual(a); rb, ub = residual(b)
            if max(ra, rb) <= tol and ub + 1e-6 >= ua:
                edge = i; found = True; break
        if not found:
            raise ValueError('Replanned edge leaves or reverses approved path envelope')
    for key in ('rate', 'max_velocity_deg_s', 'max_acceleration_deg_s2'):
        if candidate[key] != approved[key]:
            raise ValueError('Execution limits changed')
    if candidate.get('timing_profile', 'edge_quintic_v1') != approved.get('timing_profile', 'edge_quintic_v1'):
        raise ValueError('Approved timing profile changed')
    if candidate['duration_s'] > approved['duration_s'] + 5:
        raise ValueError('Replanned duration exceeds approved duration plus 5 seconds')



def check_geometry(name, fresh, approved, manifest, lift_attachment):
    """Bind relative motions to the actual FK start, placement to the original cube.

The offline collision planner still checks every fresh path. This check prevents
replanning from changing the approved task when endpoint feedback differs from
the nominal prediction within the approved continuation envelope.
    """
    start, target = fresh['start_tcp'], fresh['target_tcp']
    cube, table = manifest['target']['T_base_cube'], manifest['target']['T_base_table']
    def near(a, b):
        if len(a) != len(b) or any(not math.isfinite(x) or not math.isfinite(y) or abs(x-y) > 1e-7 for x, y in zip(a, b)):
            raise ValueError('Fresh Cartesian geometry differs from the approved task')
    def xyz(t): return [t[i][3] for i in range(3)]
    def rotation(t): return [t[i][j] for i in range(3) for j in range(3)]
    def product(a, b): return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
    normal = [table[i][2] for i in range(3)]
    near([x for row in fresh['cube'] for x in row], [x for row in cube for x in row])
    near([x for row in fresh['table'] for x in row], [x for row in table for x in row])
    if name in ['pregrasp', 'approach']:
        near(xyz(target), xyz(approved['target_tcp']))
        near(rotation(target), rotation(start if name == 'approach' and manifest['spec']['keep_start_orientation'] else approved['target_tcp']))
    elif name in ['lift', 'retreat']:
        distance = lift_height(manifest['spec']) if name == 'lift' else manifest['spec']['retreat_height_m']
        near(xyz(target), [x+distance*n for x, n in zip(xyz(start), normal)])
        near(rotation(target), rotation(start))
        if name == 'lift':
            near([x for row in product(start, fresh['attachment']) for x in row], [x for row in cube for x in row])
    elif name == 'lower':
        if lift_attachment is None:
            raise ValueError('Missing actual lift attachment')
        near([x for row in fresh['attachment'] for x in row], [x for row in lift_attachment for x in row])
        placed = product(target, lift_attachment)
        near(xyz(placed), [x+manifest['place_release_gap_m']*n for x, n in zip(xyz(cube), normal)])
        if manifest['spec']['keep_start_orientation']:
            near(rotation(target), rotation(start))
        else:
            near(rotation(placed), rotation(cube))

def run_command(argv, folder, timeout, allow_failure=False):
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID='79')
    with (folder / 'command.log').open('w') as stream:
        proc = subprocess.Popen([sys.executable, *map(str, argv)], stdout=stream, stderr=subprocess.STDOUT, env=env)
        try:
            deadline = time.monotonic() + timeout
            while proc.poll() is None:
                check_stop()
                if time.monotonic() > deadline:
                    raise TimeoutError('Stage process timeout')
                time.sleep(.05)
            code = proc.wait()
        except BaseException:
            # Existing children own cancellation semantics. Never invent a recovery target.
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill(); proc.wait()
            raise
    if code and not allow_failure:
        raise RuntimeError(f'{Path(argv[0]).name} failed ({code}); see {folder / "command.log"}')
    return code


def plan_arm(stage, angles, spec, folder, attachment=None):
    folder = Path(folder)
    output = folder / 'planned'
    command = [PLANNER, '--pose', spec['pose'], '--stage', stage,
               '--expected-angles', *angles, '--output', output, '--yaw-deg', spec['yaw_deg'],
               '--style', spec['style'], '--grasp-height-offset-m', spec['grasp_height_offset_m'],
               '--lift-height-m', lift_height(spec), '--retreat-height-m', spec['retreat_height_m']]
    command.extend(['--timing-profile', spec.get('timing_profile', 'collinear_bounded_v1')])
    command.extend(['--scheduling-policy', spec.get('scheduling_policy', 'ack_then_period_v1')])
    command.extend(speed_arguments(spec))
    if spec['keep_start_orientation']:
        command.append('--keep-start-orientation')
    if attachment is not None:
        command.extend(['--attachment-plan', attachment])
    run_command(command, folder / 'planner_call', 180)
    plan_file = output / 'plan.json'
    plan = read(plan_file)
    if not plan.get('completed'):
        raise RuntimeError('Offline planning did not complete')
    held = spec['close_target'] if stage in ['lift', 'lower'] else spec['open_target']
    preview = folder / 'preview'
    run_command([ARM, '--plan', plan_file, '--stage', stage, '--expected-angles', *angles,
                 '--gripper-target', held, '--output', preview], folder / 'preview_call', 30)
    bounded = read(preview / 'plan.json')['sdk_plan']
    return dict(name=stage, expected_angles=list(angles), gripper_target=held,
                progress_policy=plan.get('progress_policy','scheduled'),
                scheduling_policy=plan.get('scheduling_policy','ack_then_period_v1'),
                endpoint_policy=checked_endpoint_policy(plan.get('endpoint_policy')),
                plan_file=str(plan_file.resolve()), sdk_plan=bounded,
                sdk_measured_start=plan.get('sdk_measured_start',list(angles)),
                sdk_nominal_start=plan.get('sdk_nominal_start',list(angles)),
                start_reference_adjustments=plan.get('start_reference_adjustments',[]),
                target_tcp=plan['T_base_target_tcp'], start_tcp=plan['T_base_start_tcp'],
                attachment=plan['T_tcp_cube'], cube=plan['T_base_cube'], table=plan['T_base_table'])


def prepare(args):
    if getattr(args, 'accept_stable_close_shortfall', False):
        raise ValueError('Stable close shortfall acceptance is unsupported by this execution logger')
    from gripper_stage import plan as gripper_plan
    import yaml
    if getattr(args, 'continuation_policy', 'reviewed_path') != 'reviewed_path':
        raise ValueError('Only reviewed_path is supported: execution uses the approved trajectories without replanning')
    vector(args.expected_angles, 7)
    if args.close_target is None or args.open_target is None or args.pose is None:
        raise ValueError('Planning requires --pose, --close-target and --open-target')
    if not 0 < args.valid_for_s <= 7200 or not 0 < args.verification_timeout_s <= 600:
        raise ValueError('Approval lifetime must be <=7200 s; verification timeout <=600 s')
    if not .3 <= args.continuation_envelope_deg <= 1.:
        raise ValueError('Reviewed continuation path envelope must be in [0.3, 1.0] degrees')
    if not 0 < args.retreat_height_m <= .15:
        raise ValueError('Retreat must be in (0, 0.15] m')
    reviewed_lift = lift_height(dict(lift_height_m=args.lift_height_m))
    cfg = yaml.safe_load(CONFIG.read_text())
    pose = read(args.pose)
    if pose.get('cube_size_m', .04) != .04 or cfg['cube_size_m'] != .04:
        raise ValueError('Only the existing 40 mm cube is supported')
    recognition_report = getattr(args, 'recognition_report', None)
    perception = None; recognition_files = {}; refresh = None
    if recognition_report:
        perception, recognition_files, refresh = recognition_contract(
            recognition_report, args.pose, cfg['hardware_mapping_candidate'])
    started = time.time()
    args.output.mkdir(parents=True, exist_ok=False)
    preparation_trace = Trace(args.output/'preparation_timing.jsonl')
    preparation_trace.mark('preparation_start')
    spec = dict(pose=str(args.pose.resolve()), close_target=args.close_target, open_target=args.open_target,
                yaw_deg=args.yaw_deg, style=args.style, grasp_height_offset_m=args.grasp_height_offset_m,
                keep_start_orientation=args.keep_start_orientation, retreat_height_m=args.retreat_height_m,
                lift_height_m=reviewed_lift, speed_deg_s=args.speed_deg_s, acceleration_deg_s2=args.acceleration_deg_s2,
                rate_hz=args.rate_hz, gripper_speed_sdk_s=args.gripper_speed_sdk_s, candidate=cfg['hardware_mapping_candidate'])
    speed_arguments(spec)
    spec['timing_profile'] = getattr(args, 'timing_profile', 'collinear_bounded_v1')
    spec['include_pregrasp'] = getattr(args, 'include_pregrasp', False)
    spec['include_prepare_gripper'] = True
    spec['fixed_ready'] = getattr(args, 'fixed_ready', False)
    spec['scheduling_policy'] = getattr(args, 'scheduling_policy', 'ack_then_period_v1')
    if (spec['scheduling_policy'] not in ('ack_then_period_v1', 'ack_overlap_v1', 'shared_progress_v1', 'timed_group_v1')
            or (spec['scheduling_policy'] in ('ack_overlap_v1', 'shared_progress_v1') and not spec['fixed_ready'])):
        raise ValueError('ACK overlap requires a fixed-ready method')
    profile_path = ROOT.parents[1] / 'app/fixed_ready_pose.json'
    if spec['fixed_ready']:
        from d1_pick_place.sdk_executor import Mapping
        profile = read(profile_path)
        fixed_target = Mapping.from_candidate(spec['candidate']).to_sdk(profile['model_target_rad']).tolist()
    current = list(args.expected_angles); stages = []; attachment = None
    for number, name in enumerate(stage_order(spec)):
        folder = args.output / f'{number:02d}_{name}'
        if name in GRIPPER_STAGES:
            target = spec['close_target'] if name == 'close' else spec['open_target']
            with preparation_trace.span('planning', name+'_gripper_plan'):
                segment = gripper_plan(target, current, speed=spec['gripper_speed_sdk_s'],
                                       rate=spec['rate_hz'], segmented=name == 'prepare_gripper')
            stages.append(dict(name=name, expected_angles=current.copy(), gripper_target=target, gripper_plan=segment))
            current[6] = target
        else:
            if name == 'fixed_pregrasp' and max(abs(a-b) for a,b in zip(current[:6],fixed_target)) <= LIMITS['endpoint_tolerance_deg']:
                folder.mkdir(parents=True)
                source = folder/'already_at_fixed_target.json'
                policy = None if spec['scheduling_policy']=='timed_group_v1' else checked_endpoint_policy(ENDPOINT_RETRY_POLICY)
                save(source, dict(already_at_fixed_target=True, measured=current.copy(), fixed_target=fixed_target,
                                  scheduling_policy=spec['scheduling_policy'],
                                  progress_policy='scheduled' if spec['scheduling_policy']=='timed_group_v1' else 'feedback_paced', endpoint_policy=policy))
                item = dict(name=name, expected_angles=current.copy(), gripper_target=current[6],
                    plan_file=str(source.resolve()), already_at_fixed_target=True,
                    progress_policy='scheduled' if spec['scheduling_policy']=='timed_group_v1' else 'feedback_paced', endpoint_policy=policy,
                    scheduling_policy=spec['scheduling_policy'],
                    sdk_plan=dict(sdk=[current[:6].copy(),current[:6].copy()],duration_s=0.))
            else:
                with preparation_trace.span('planning', name+'_arm_plan_and_preview'):
                    item = plan_arm(name, current, spec, folder, attachment)
            stages.append(item); current[:6] = item['sdk_plan']['sdk'][-1]
            if name == 'lift':
                attachment = item['plan_file']
    preparation_trace.mark('preparation_complete')
    paths = [args.pose, CONFIG, cfg['hardware_mapping_candidate'], WS / 'src/d1_pick_place/config/d1.urdf',
             Path(__file__), PLANNER, ARM, GRIPPER, OBSERVER, ROOT / 'tools/handeye_joint_step.py',
             ROOT / 'tools/j4_smooth_test.py', ROOT / 'host_build/arm_control',
             ROOT / 'host_src/src/controller.cpp', ROOT / 'host_src/src/protocol.cpp',
             PACKAGE / 'sdk_executor.py', PACKAGE / 'hardware_bridge.py', PACKAGE / 'task.py',
             PACKAGE / 'configuration.py', PACKAGE / 'geometry.py', PACKAGE / 'kinematics.py']
    paths.extend([PACKAGE / 'path_timing.py', ROOT / 'tools/grasp_timing.py', ROOT.parents[1] / 'app/vision.py', ROOT.parents[1] / 'app/hsv.py', ROOT.parents[1] / 'app/execution_scope.py'])
    paths.append(ROOT.parents[1] / 'app/arm_diagnostics.py')
    paths.append(PACKAGE / 'adaptive_reference.py')
    if spec['scheduling_policy']=='timed_group_v1':
        paths.extend([PACKAGE/'timed_group_transport.py',PACKAGE/'sdk_timed_executor.py',
            ROOT/'host_build/d1_timed_bridge',ROOT/'host_src/timed_trajectory_bridge.cpp',
            ROOT.parents[1]/'execution_contract.json'])
    paths.append(preparation_trace.path)
    if spec['fixed_ready']:
        paths.append(profile_path)
        paths.append(ROOT.parents[1] / 'app/fixed_pose.py')
        paths.append(PACKAGE / 'endpoint_control.py')
        if spec['scheduling_policy'] in ('ack_overlap_v1', 'shared_progress_v1'):
            paths.append(ROOT.parents[1] / 'app/backend.py')
            paths.append(ROOT.parents[1] / 'app/method_runner.py')
        if spec['scheduling_policy']=='shared_progress_v1':
            paths.extend([PACKAGE / 'shared_progress.py', ROOT.parents[1] / 'app/closed_loop_runner.py',
                          ROOT.parents[1] / 'execution_contract.json'])
    if recognition_report:
        paths.append(recognition_report)
        paths.extend(recognition_files)
    paths.extend(item['plan_file'] for item in stages if 'plan_file' in item)
    manifest = dict(schema_version=1, created_at=started, expires_at=started+args.valid_for_s,
                    initial_angles=args.expected_angles, target=dict(cube_size_m=.04,
                    T_base_cube=pose['T_base_cube'], T_base_table=pose['T_base_table']),
                    spec=spec, limits=approved_limits(spec), place_release_gap_m=cfg['place_release_gap_m'],
                    verification_timeout_s=args.verification_timeout_s, stages=stages,
                    continuation_envelope_deg=args.continuation_envelope_deg,
                    continuation_policy=getattr(args, 'continuation_policy', 'reviewed_path'),
                    arm_execution_policy=ARM_EXECUTION_POLICY,
                    accept_stable_close_shortfall=args.accept_stable_close_shortfall,
                    files={str(Path(p).resolve()): digest(p) for p in paths},
                    physical_grasp_verification='automatic_opencv_two_hypothesis_v1',
                    limitations=['nominal CAD TCP/fingers; force and jaw width uncalibrated',
                    'camera freshness does not verify clearance or object pose',
                    'cancellation stops future commands without a same-position hold; physical stop is not certified'],
                    hardware_commands_sent=False)
    manifest['recognition_report'] = str(recognition_report.resolve()) if recognition_report else None
    manifest['preparation_timing'] = str(preparation_trace.path.resolve())
    manifest['replay_only'] = (perception.get('source_kind') != 'live_camera' if perception
                             else pose.get('source_kind') == 'saved_image')
    manifest['execution_candidate_eligible'] = (perception.get('execution_candidate_eligible') is True
                                               if perception else not manifest['replay_only'])
    manifest['recognition_refresh'] = refresh
    manifest['target_recheck'] = TARGET_RECHECK if perception else None
    if spec['fixed_ready']:
        manifest.update(operation=FIXED_PREGRASP, fixed_model_target_rad=profile['model_target_rad'],
                        fixed_sdk_target_deg=fixed_target, fixed_ready_profile=profile,
                        endpoint_policy=checked_endpoint_policy(stages[-1].get('endpoint_policy')),
                        replay_only=False,execution_candidate_eligible=True)
        manifest['target'] = dict(description='Gazebo 固定准备姿态（ready）；与红块位置无关',
            model_target_rad=profile['model_target_rad'], sdk_target_deg=fixed_target, source=profile['source'])
    save(args.output / 'manifest.json', manifest)
    return dict(manifest=str((args.output / 'manifest.json').resolve()),
                approval_sha256=digest(args.output / 'manifest.json'), expires_at=manifest['expires_at'],
                lift_height_m=spec['lift_height_m'],
                stages=[dict(name=s['name'], duration_s=s.get('sdk_plan', s.get('gripper_plan'))['duration_s']) for s in stages],
                replay_only=manifest['replay_only'],
                execution_candidate_eligible=manifest['execution_candidate_eligible'],
                target_recheck=manifest['target_recheck'],
                hardware_commands_sent=False)


def validate_manifest(path, approval=None, now=None):
    manifest = read(path)
    if manifest.get('accept_stable_close_shortfall'):
        raise ValueError('Stable close shortfall acceptance is unsupported by this execution logger')
    if manifest.get('physical_grasp_verification') != 'automatic_opencv_two_hypothesis_v1':
        raise ValueError('Manifest is not this application automatic visual protocol')
    if approval is not None and digest(path) != approval:
        raise ValueError('Approval SHA-256 does not match the exact manifest')
    if manifest['schema_version'] != 1 or manifest['limits'] != approved_limits(manifest['spec']):
        raise ValueError('Unsupported manifest or changed limits')
    lift_height(manifest['spec'])
    if manifest.get('continuation_policy', 'reviewed_path') != 'reviewed_path':
        raise ValueError('Only reviewed_path is supported; prepare and approve fixed trajectories again')
    if manifest.get('arm_execution_policy', ARM_EXECUTION_POLICY) != ARM_EXECUTION_POLICY:
        raise ValueError('Unsupported arm execution policy; prepare a new reviewed trajectory')
    if not .3 <= manifest['continuation_envelope_deg'] <= 1.:
        raise ValueError('Invalid approved continuation path envelope')
    if [s['name'] for s in manifest['stages']] != stage_order(manifest['spec']):
        raise ValueError('Unexpected stage order')
    if manifest['spec'].get('fixed_ready'):
        from d1_pick_place.sdk_executor import Mapping
        profile_path = ROOT.parents[1] / 'app/fixed_ready_pose.json'
        profile = read(profile_path)
        fixed_target = Mapping.from_candidate(manifest['spec']['candidate']).to_sdk(profile['model_target_rad']).tolist()
        stage = manifest['stages'][-1]
        tolerance = LIMITS['endpoint_tolerance_deg'] if stage.get('already_at_fixed_target') else .01
        if (manifest.get('operation') != FIXED_PREGRASP or manifest.get('recognition_report')
                or manifest.get('fixed_model_target_rad') != profile['model_target_rad']
                or manifest.get('fixed_sdk_target_deg') != fixed_target
                or manifest['files'].get(str(profile_path.resolve())) != digest(profile_path)
                or max(abs(a-b) for a,b in zip(stage['sdk_plan']['sdk'][-1],fixed_target)) > tolerance + 1e-7):
            raise ValueError('固定预抓取目标与 Gazebo ready 定义不一致，请重新准备')
    else:
        approved_lift = next(s for s in manifest['stages'] if s['name'] == 'lift')
        check_geometry('lift', approved_lift, approved_lift, manifest, None)
    if not manifest['created_at'] <= (time.time() if now is None else now) <= manifest['expires_at']:
        raise ValueError('Approval manifest is expired or future-dated')
    for source, expected in manifest['files'].items():
        if digest(source) != expected:
            raise ValueError(f'Reviewed file changed: {source}')
    for stage in manifest['stages']:
        if stage['name'] not in GRIPPER_STAGES:
            source = str(Path(stage['plan_file']).resolve())
            if manifest['files'].get(source) != digest(source):
                raise ValueError('Approved arm trajectory file is not bound by the manifest: '+source)
    validate_endpoint_policies(manifest)
    validate_scheduling_policies(manifest)
    return manifest


def observe(folder, allow_position_start=False):
    command = [OBSERVER, '--output', Path(folder) / 'observation']
    if allow_position_start:
        command.append('--allow-position-start')
    run_command(command, Path(folder) / 'observe_call', 30)
    source = Path(folder) / 'observation/wrist_observation.json'
    report = read(source)
    if report['hardware_commands_sent'] != 0:
        raise RuntimeError('Read-only observer sent commands')
    return report['sdk_angles'], source


def observed_position_mode_ready(path):
    """Only explicit fresh status evidence consumes an unused startup allowance."""
    report = read(path)
    return (report.get('status_fresh') is True
            and all(type(report.get('status', {}).get(key)) is int
                    and report['status'][key] == expected
                    for key, expected in [('power_status', 1), ('enable_status', 1), ('error_status', 0)]))


def position_commands_recorded(result, output):
    count = result.get('commands')
    if type(count) is int and count > 0:
        return True
    transport = Path(output) / 'transport.jsonl'
    if transport.is_file():
        for line in transport.read_text().splitlines():
            try:
                if json.loads(line).get('kind') == 'command':
                    return True
            except (ValueError, AttributeError):
                continue
    return False


def classify_stable_shortfall(folder, target):
    """Accept only evidence for a bounded position shortfall, never contact/force.

Requires the new common monotonic time fields so camera/feedback freshness can
be checked directly. Historical logs without that clock evidence are rejected.
    """
    from gripper_stage import check_angles
    folder = Path(folder); summary = read(folder / 'summary.json')
    def require(condition, reason):
        if not condition: raise ValueError('Close shortfall rejected: ' + reason)
    require(summary.get('error') == 'Endpoint error > 0.5 position units; stop next phase', 'not the precise endpoint error')
    require(summary.get('completed') is False and summary.get('position_reached') is False, 'unexpected completion state')
    require(summary.get('camera_error', 'missing') is None and not summary.get('cleanup_error'), 'camera/cleanup fault')
    require(summary.get('final_phase') == 'gripper_single_stage_settle', 'not after bounded segment')
    require(summary.get('final_status') == {'enable_status': 1, 'power_status': 1, 'error_status': 0}, 'invalid final status')
    require(summary.get('hardware_commands_sent') is True, 'no sent commands')
    require(summary.get('last_commanded_target_sdk_units') == target, 'final target not commanded')
    actual_plan = summary['actual_segment_plan']
    rate=actual_plan.get('rate_hz')
    require(actual_plan['target_sdk_units'] == target and isinstance(rate,(int,float))
            and not isinstance(rate,bool) and math.isfinite(rate) and 0<rate<=10, 'wrong target or rate')
    expected_count = round(actual_plan['duration_s'] * rate) + 1
    require(summary['commands'] == summary['receipts'] == expected_count, 'incomplete command receipts')
    require(math.isfinite(summary['non_target_max_drift_deg']) and summary['non_target_max_drift_deg'] <= .5+1e-6, 'non-target drift')
    baseline = summary['measured_start']; final = summary['final_angles']
    check_angles(baseline); check_angles(final)
    require(target < baseline[6] and .5+1e-5 < final[6]-target <= 3, 'not an eligible bounded closing shortfall')
    events_path = folder / 'motion/events.jsonl'; frames_path = folder / 'frames/frames.jsonl'
    events = [json.loads(line) for line in events_path.read_text().splitlines()]
    commands = []; acks = []; feedback = []; statuses = []; pending = False; last_received = -math.inf
    for row in events:
        stamp = row.get('host_monotonic')
        require(isinstance(stamp, (float, int)) and math.isfinite(stamp), 'missing common monotonic evidence')
        if row['kind'] == 'command':
            require(not pending and row.get('joint') == 6, 'overlapping or wrong-joint command')
            commands.append(row); pending = True
        if row['kind'] != 'output': continue
        line = row['line']
        require(not line.startswith(('[设备拒绝]', '[保护停止]', '[反馈中断]', '[接收超时]', '[未发送]', '[失败]', '[参数错误]', '[DDS写入失败]')), 'transport fault')
        if line.startswith('[ok] '):
            require(pending and line not in acks, 'unexpected or duplicate receipt')
            acks.append(line); pending = False
        if '[rx] ' not in line: continue
        received = row['session_origin_monotonic'] + row['received_t']
        require(math.isfinite(received) and received > last_received and -.001 <= stamp-received <= .6, 'invalid/reordered/stale receive timestamps')
        last_received = received
        message = json.loads(line.split('[rx] ', 1)[1]); data = message.get('data', {})
        if message.get('address') != 2: continue
        if message.get('funcode') == 1:
            q = [data[f'angle{i}'] for i in range(7)]; check_angles(q)
            if commands:
                require(max(abs(q[i]-baseline[i]) for i in range(6)) <= .5+1e-6, 'feedback non-target drift')
                require(abs(q[6]-commands[-1]['target_sdk_units']) <= 3, 'feedback tracking fault')
            feedback.append((received, q, row.get('phase')))
        if message.get('funcode') == 3:
            if commands:
                require(all(data.get(k) == v for k, v in [('enable_status', 1), ('power_status', 1), ('error_status', 0)]), 'abnormal feedback status')
            statuses.append((received, data))
    require(not pending and len(commands) == len(acks) == expected_count, 'missing logged commands/receipts')
    require(abs(commands[-1]['target_sdk_units']-target) <= .00051, 'final target absent from command log')
    require(all(commands[i+1]['target_sdk_units'] <= commands[i]['target_sdk_units']+1e-9 for i in range(len(commands)-1)), 'not a single closing segment')
    require(bool(feedback) and bool(statuses), 'missing angle/status feedback')
    end = events[-1]['host_monotonic']
    tail = [(stamp, q) for stamp, q, phase in feedback if end-1 <= stamp <= end and phase == 'gripper_single_stage_settle'
            and stamp > commands[-1]['host_monotonic']]
    require(len(tail) >= 5 and len({stamp for stamp, _ in tail}) == len(tail), 'fewer than five independent fresh terminal samples')
    require(end-tail[-1][0] <= .6 and end-statuses[-1][0] <= .6, 'stale final feedback/status')
    values = [q[6] for _, q in tail]
    require(max(values)-min(values) <= .1+1e-5, 'terminal J6 feedback is not stable')
    require(all(target <= value <= target+3 for value in values), 'terminal shortfall outside bound')
    require(max(abs(a-b) for a, b in zip(final, tail[-1][1])) <= 1e-5, 'summary differs from final feedback')
    frames = [json.loads(line) for line in frames_path.read_text().splitlines()]
    for role, serial in [('global', 'CC1365302K0'), ('wrist', 'CC1365301EY')]:
        rows = [r for r in frames if r['role'] == role]
        require(bool(rows), 'missing '+role+' camera evidence')
        require(all(r['serial'] == serial and 0 <= r['age_ms'] <= 250 for r in rows), 'unhealthy camera frame')
        require(all(b['index'] > a['index'] and b['host_monotonic'] > a['host_monotonic'] for a, b in zip(rows, rows[1:])), 'reordered camera frames')
        recent = [r for r in rows if 0 <= end-r['host_monotonic'] <= .8]
        require(bool(recent), 'stale '+role+' camera evidence')
    return dict(classification='stable_position_shortfall/contact_unverified', contact_detected=False,
                grasp_verified=False, exact_endpoint_completed=False, commanded_target_sdk_units=target,
                actual_sdk_units=final[6], terminal_samples=len(tail), terminal_span_sdk_units=max(values)-min(values),
                commands=expected_count, receipts=len(acks),
                evidence=[dict(path=str(p.resolve()), sha256=digest(p)) for p in [folder/'summary.json', events_path, frames_path]])


def execute_cycle(args):
    global _STOP_FILE
    _STOP_FILE = getattr(args, 'stop_file', None)
    check_stop()
    if not args.approval_sha256 or not args.approval_text:
        raise ValueError('--execute requires --approval-sha256 and --approval-text recording the user approval; a hash alone is not consent')
    manifest = validate_manifest(args.manifest, args.approval_sha256)
    scope = getattr(args, 'execution_scope', FULL_CYCLE)
    if bool(manifest['spec'].get('fixed_ready')) != (scope == FIXED_PREGRASP):
        raise ValueError('固定姿态计划与完整抓放的执行范围不能混用')
    selected_stages = select_execution_stages(manifest['stages'], scope)
    if manifest.get('replay_only') or manifest.get('execution_candidate_eligible', True) is not True:
        raise ValueError('Replay-only or ineligible perception manifest cannot execute')
    if manifest.get('recognition_report'):
        approved_perception = read(manifest['recognition_report'])
        if (approved_perception.get('source_kind') != 'live_camera'
                or approved_perception.get('execution_candidate_eligible') is not True
                or not manifest.get('recognition_refresh') or manifest.get('target_recheck') != TARGET_RECHECK):
            raise ValueError('Recognition manifest lacks approved live target recheck contract')
    # Claim before read-only preflight too: failures are never silently replayable.
    claim = Path(str(args.manifest) + '.consumed')
    with claim.open('x') as stream:
        stream.write(json.dumps(dict(approval_sha256=args.approval_sha256, output=str(args.output.resolve()),
                                     execution_scope=scope, claimed_at=time.time())))
    args.output.mkdir(parents=True, exist_ok=False)
    trace = Trace(args.output / 'timing.jsonl')
    trace.mark('execution_session_start')
    if manifest.get('recognition_report'):
        import_recognition(trace, read(manifest['recognition_report']))
    if manifest.get('preparation_timing'):
        import_span_rows(trace, load_rows(manifest['preparation_timing']), 'preparation_')
    wait_file = getattr(args, 'approval_wait_json', None)
    if wait_file:
        wait = read(wait_file)
        if wait.get('boot_id') != boot_id() or wait.get('approval_sha256') != args.approval_sha256:
            raise ValueError('Human wait must bind the current boot and approved manifest')
        if not 0 <= wait['start_monotonic'] <= wait['end_monotonic'] <= time.monotonic():
            raise ValueError('Invalid human approval wait')
        trace.interval('human_confirmation', 'user_plan_approval', wait['start_monotonic'], wait['end_monotonic'])
    report = dict(completed=False, trajectory_completed=False, physical_grasp_verified=False, physical_cycle_verified=False,
                  approval_sha256=args.approval_sha256, user_approval_text=args.approval_text, completed_stages=[], accepted_stages=[], stage_outcomes={}, visual_verifications={}, state='starting',
                  hardware_commands_may_have_been_sent=False)
    report.update(execution_scope=scope, requested_stages=[stage['name'] for stage in selected_stages])
    if scope in (PREGRASP, FIXED_PREGRASP):
        report.update(pregrasp_reached=False, grasp_result='not_attempted', cycle_result='not_attempted')
    shortfall_j6 = None
    lift_start_observation = None
    lift_execution_started_at = None
    report['arm_execution_policy'] = ARM_EXECUTION_POLICY
    position_start_pending = True
    with (args.output / 'events.jsonl').open('x') as events:
        def event(state, **extra):
            report['state'] = state
            events.write(json.dumps(dict(time=time.time(), state=state, **extra), allow_nan=False) + '\n'); events.flush()
            save(args.output / 'result.json', report)
        try:
            for number, approved in enumerate(selected_stages):
                check_stop()
                name = approved['name']; folder = args.output / f'{number:02d}_{name}'
                event('observing', stage=name)
                if number == 0 and manifest.get('recognition_report'):
                    event('rechecking_approved_target', stage=name)
                    current, observation, comparison = recheck_target(manifest, folder, trace)
                    report['initial_target_recheck'] = comparison
                else:
                    with trace.span('stage_wait', name+'_fresh_stationary_observation'):
                        current, observation = observe(folder, allow_position_start=position_start_pending)
                if observed_position_mode_ready(observation):
                    position_start_pending = False
                reviewed = approved['expected_angles'].copy()
                if name not in GRIPPER_STAGES:
                    # Compare against the actual frozen trajectory start, including
                    # the explicitly recorded bounded reference for noisy feedback.
                    reviewed[:6] = approved['sdk_plan']['sdk'][0]
                if shortfall_j6 is not None and name in ['lift', 'lower', 'open']:
                    reviewed[6] = shortfall_j6  # Evidence-backed endpoint, never a replacement command.
                check_start(current, reviewed, LIMITS['start_arm_tolerance_deg'])
                if number == 0:
                    check_start(current, manifest['initial_angles'])
                if name == 'fixed_pregrasp' and approved.get('already_at_fixed_target'):
                    validate_manifest(args.manifest, args.approval_sha256)
                    if max(abs(a-b) for a,b in zip(current[:6],manifest['fixed_sdk_target_deg'])) > LIMITS['endpoint_tolerance_deg'] + 1e-5:
                        raise ValueError('固定预抓取姿态实际反馈已变化，请重新准备')
                    result = dict(final_angles=current,completed=True,commands=0)
                    report['completed_stages'].append(name)
                    report['stage_outcomes'][name] = dict(exact_endpoint_completed=True,already_at_fixed_target=True)
                    event('stage_completed',stage=name,no_motion_required=True)
                    continue
                if name in GRIPPER_STAGES:
                    from gripper_stage import plan as gripper_plan
                    fresh = gripper_plan(approved['gripper_target'], current,
                        speed=manifest['spec']['gripper_speed_sdk_s'], rate=manifest['spec']['rate_hz'],
                        segmented=name == 'prepare_gripper')
                    if fresh['duration_s'] > approved['gripper_plan']['duration_s'] + 5:
                        raise ValueError('Gripper duration leaves approved bound')
                    command = [GRIPPER, '--target', approved['gripper_target'], '--expected-angles', *current,
                               '--output', folder / 'executed', '--execute', '--speed-sdk-s', manifest['spec']['gripper_speed_sdk_s'],
                               '--arm-speed-deg-s', manifest['spec']['speed_deg_s'], '--rate-hz', manifest['spec']['rate_hz']]
                    if name == 'prepare_gripper':
                        command.append('--segmented')
                    result_file = folder / 'executed/summary.json'; duration = fresh['duration_s']
                else:
                    # Geometry, speed and endpoint were collision-checked during
                    # preparation and approved together. Do not ask IK/OMPL for a
                    # different solution after that approval.
                    fresh = approved
                    plan_file = str(Path(approved['plan_file']).resolve())
                    plan_sha = manifest['files'][plan_file]
                    folder.mkdir(parents=True, exist_ok=True)
                    selection = dict(policy=ARM_EXECUTION_POLICY, approved_plan_file=plan_file,
                        approved_plan_sha256=plan_sha, measured_start=current,
                        approved_trajectory_start=approved['sdk_plan']['sdk'][0],
                        approved_trajectory_end=approved['sdk_plan']['sdk'][-1],
                        arm_start_tolerance_deg=LIMITS['start_arm_tolerance_deg'],
                        arm_start_difference_deg=[a-b for a,b in zip(current[:6],reviewed[:6])],
                        replanned=False)
                    save(folder/'approved_trajectory_selection.json', selection)
                    event('using_approved_trajectory', stage=name, plan_sha256=plan_sha)
                    if name == 'lift':
                        lift_start_observation = observation
                    command = [ARM, '--plan', plan_file, '--approved-plan-sha256', plan_sha,
                               '--candidate', manifest['spec']['candidate'], '--stage', name, '--expected-angles', *current,
                               '--gripper-target', approved['gripper_target'], '--output', folder / 'executed', '--execute']
                    result_file = folder / 'executed/result.json'; duration = fresh['sdk_plan']['duration_s']
                if _STOP_FILE is not None:
                    command.extend(['--stop-file', _STOP_FILE])
                if position_start_pending:
                    command.append('--allow-position-start')
                check_stop()
                # Recheck the approved files and lifetime immediately before each motion.
                validate_manifest(args.manifest, args.approval_sha256)
                if name == 'lift':
                    lift_execution_started_at = time.time()
                    report['lift_execution_started_at'] = lift_execution_started_at
                event('executing', stage=name, allow_position_start=position_start_pending)
                report['hardware_commands_may_have_been_sent'] = True
                allow_shortfall = name == 'close' and manifest['accept_stable_close_shortfall']
                try:
                    with trace.span('gripper_execution' if name in GRIPPER_STAGES else 'arm_execution', name):
                        timeout_stage = dict(fresh, name=name)
                        code = run_command(command, folder / 'execute_call', stage_process_timeout(timeout_stage, duration), allow_failure=allow_shortfall)
                finally:
                    import_transport(trace, folder / 'executed', name)
                result = read(result_file)
                if position_commands_recorded(result, result_file.parent):
                    position_start_pending = False
                report['position_start_pending'] = position_start_pending
                if not result.get('completed'):
                    if not allow_shortfall or code != 1:
                        raise RuntimeError('Stage did not complete: ' + str(result.get('error')))
                    evidence = classify_stable_shortfall(result_file.parent, approved['gripper_target'])
                    save(folder / 'stable_shortfall.json', evidence)
                    shortfall_j6 = evidence['actual_sdk_units']
                    report['accepted_stages'].append(name)
                    report['close_classification'] = evidence['classification']
                    report['stage_outcomes'][name] = dict(exact_endpoint_completed=False, accepted=True,
                        classification=evidence['classification'], evidence=str((folder / 'stable_shortfall.json').resolve()))
                    event('stable_close_shortfall_accepted_contact_unverified', stage=name, **evidence)
                else:
                    if code != 0:
                        raise RuntimeError('Nonzero process exit despite reported completion')
                    report['completed_stages'].append(name)
                    report['stage_outcomes'][name] = dict(exact_endpoint_completed=True, accepted=False)
                    if shortfall_j6 is not None and name in ['lift', 'lower']:
                        vector(result['final_angles'], 7); shortfall_j6 = result['final_angles'][6]
                    if name == 'open': shortfall_j6 = None
                    event('stage_completed', stage=name)
                    if name == 'retreat': trace.mark('retreat_complete')
                if name == 'lift':
                    with trace.span('verification', 'automatic_lift_visual_evidence'):
                        _, lifted = observe(folder / 'lifted')
                        review = verify_lift(manifest, lifted, folder / 'visual_result',
                            lift_plan_path=Path(approved['plan_file']),
                            lift_start_observation_path=lift_start_observation,
                            lift_execution_started_at=lift_execution_started_at)
                    report['visual_verifications']['lift'] = review
                    report['grasp_result'] = review['status']
                    report['physical_grasp_verified'] = review['status'] == 'success'
                    event('automatic_lift_result', stage=name, status=review['status'], reason=review.get('reason'))
                    if review['status'] != 'success':
                        raise RuntimeError('Lift result ' + review['status'] + ': ' + review.get('reason', 'insufficient visual evidence'))
            if scope in (PREGRASP, FIXED_PREGRASP):
                check_stop()
                report.update(completed=True, pregrasp_reached=True, final_angles=result.get('final_angles'))
                trace.mark('pregrasp_complete')
                event('pregrasp_completed', stage='pregrasp')
                return report
            report['trajectory_completed'] = True
            with trace.span('verification', 'automatic_return_visual_evidence'):
                _, final_observation = observe(args.output / f"{len(manifest['stages']):02d}_final_observation")
                review = verify_return(manifest, final_observation, args.output / 'return_visual_result')
            report['visual_verifications']['placed_released_retreat'] = review
            report.update(physical_cycle_verified=review['status'] == 'success',
                          cycle_result=review['status'])
            if review['status'] != 'success':
                raise RuntimeError('Return visual result ' + review['status'] + ': ' + review.get('reason', 'insufficient visual evidence'))
            report['completed'] = True
            event('cycle_completed', visual_result=review['status'])
        except BaseException as exc:
            report['error'] = str(exc) or type(exc).__name__
            report['cancelled'] = isinstance(exc, KeyboardInterrupt)
            report['physical_stop_verified'] = False
            event('stopped_no_recovery', error=report['error'])
        finally:
            trace.mark('execution_session_end')
            report['timing'] = summarize(load_rows(trace.path))
            report['timing']['approval_wait_explicitly_recorded'] = wait_file is not None
            save(args.output / 'timing_summary.json', report['timing'])
            save(args.output / 'result.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, help='Check a prepared manifest; with --execute run it once')
    parser.add_argument('--approval-sha256')
    parser.add_argument('--approval-text', help='Record the actual user approval for this manifest; the digest alone is not consent')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--execution-scope', choices=[FULL_CYCLE, PREGRASP, FIXED_PREGRASP], default=FULL_CYCLE,
                        help='pregrasp executes only the approved initial gripper adjustment and pregrasp path, then stops')
    parser.add_argument('--stop-file', type=Path)
    parser.add_argument('--speed-deg-s', type=float, default=.5)
    parser.add_argument('--acceleration-deg-s2', type=float, default=1.)
    parser.add_argument('--gripper-speed-sdk-s', type=float, default=.5)
    parser.add_argument('--rate-hz', type=float, default=10.)
    parser.add_argument('--recognition-report', type=Path, help='Immutable prepare_accel_grasp report bound to --pose')
    parser.add_argument('--approval-wait-json', type=Path, help='Explicit measured user-confirmation interval; never infer from all idle time')
    parser.add_argument('--timing-profile', choices=['edge_quintic_v1', 'collinear_bounded_v1'], default='collinear_bounded_v1')
    parser.add_argument('--scheduling-policy', choices=['ack_then_period_v1', 'ack_overlap_v1', 'shared_progress_v1', 'timed_group_v1'], default='ack_then_period_v1')
    parser.add_argument('--pose', type=Path)
    parser.add_argument('--expected-angles', type=float, nargs=7)
    parser.add_argument('--close-target', type=float)
    parser.add_argument('--open-target', type=float)
    parser.add_argument('--yaw-deg', type=float, default=0.)
    parser.add_argument('--style', choices=['top', 'side'], default='top')
    parser.add_argument('--grasp-height-offset-m', type=float, default=0.)
    parser.add_argument('--keep-start-orientation', action='store_true')
    parser.add_argument('--include-pregrasp', action='store_true',
                        help='Plan a collision-checked free-space entry above the cube before straight approach')
    parser.add_argument('--fixed-ready', action='store_true', help='Plan only the fixed Gazebo ready joint posture, independent of cube recognition')
    parser.add_argument('--lift-height-m', type=float, default=.03,
                        help='Reviewed lift along the table normal, in (0, 0.10] m; default 0.03 m')
    parser.add_argument('--retreat-height-m', type=float, default=.08)
    parser.add_argument('--accept-stable-close-shortfall', action='store_true',
                        help='Unsupported legacy option; rejected for this execution logger')
    parser.add_argument('--continuation-envelope-deg', type=float, default=1.,
                        help='Legacy manifest field, 0.3..1.0 deg; fixed trajectory starts require <=1 degree')
    parser.add_argument('--continuation-policy', choices=['reviewed_path'], default='reviewed_path',
                        help='Execute the exact reviewed arm trajectories; no execution-time replanning')
    parser.add_argument('--valid-for-s', type=float, default=3600.)
    parser.add_argument('--verification-timeout-s', type=float, default=300.)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    try:
        if args.execution_scope != FULL_CYCLE and not args.execute:
            raise ValueError('--execution-scope requires --execute and an already reviewed manifest')
        if args.execute:
            if args.manifest is None or args.output is None:
                raise ValueError('--execute requires --manifest and --output')
            report = execute_cycle(args)
        elif args.manifest:
            validate_manifest(args.manifest, args.approval_sha256)
            report = dict(valid=True, approval_sha256=digest(args.manifest), hardware_commands_sent=False)
        else:
            if args.output is None or args.expected_angles is None:
                raise ValueError('Planning requires --output and --expected-angles')
            report = prepare(args)
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0 if report.get('completed', True) else 1
    except (ValueError, OSError, RuntimeError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Run exactly one reviewed arm stage, or a prefix ending at a planner waypoint.

Default: save/print the plan without opening robot or camera connections.
--execute enables one stage only; never enables, homes, opens the gripper,
returns, or advances to another stage. Cancellation stops future commands;
it sends no same-position hold and does not certify a physical emergency stop.
"""
import argparse
import base64
import copy
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

WS = Path(__file__).resolve().parents[1]
CONTROL = WS.parent
sys.path.insert(0, str(CONTROL.parents[1]))
sys.path.insert(0, str(WS / 'src/d1_pick_place'))
sys.path.insert(0, str(CONTROL / 'tools'))
from app.arm_diagnostics import execution_error_fields
from d1_pick_place.sdk_executor import ARM_POSITION_TOLERANCE_DEG, FIXED_PROGRESS_MARGIN_DEG
from d1_pick_place.endpoint_control import checked_endpoint_policy
ENDPOINT_TOLERANCE_DEG = ARM_POSITION_TOLERANCE_DEG


def candidate_wait_policy(scheduling_policy='ack_then_period_v1'):
    """Describe feedback admission without changing the approved path/timing."""
    if scheduling_policy=='timed_group_v1':
        return dict(enabled=True,admission_scope='board_whole_arm',per_axis_wait=False,
            progress='board_monotonic_clock',transport='buffered_timed_group_no_per_axis_ACK',
            dispatch_lateness_limit_ms=20,arrival='three_valid_encoder_frames_at_endpoint')
    if scheduling_policy=='shared_progress_v1':
        return dict(enabled=True,margin_deg=FIXED_PROGRESS_MARGIN_DEG,
                    admission_scope='whole_reference_vector',per_axis_wait=False,
                    progress='one_common_source_phase_for_all_arm_axes',
                    transport='serial_per_axis_receipts_not_hardware_atomic',
                    tracking_reference='last_published_joint_targets')
    return dict(enabled=True, margin_deg=FIXED_PROGRESS_MARGIN_DEG,
                per_wait_timeout_s=2., total_wait_budget_s=30.,
                tracking_reference='last_published_joint_targets')


def check_approved_plan_digest(raw, expected):
    actual = hashlib.sha256(raw).hexdigest()
    if expected is not None and actual != expected:
        raise ValueError('Approved trajectory file changed before execution; prepare a new path')
    return actual


def source_endpoint_policy(source, stage):
    policy = checked_endpoint_policy(source.get('endpoint_policy'))
    if policy is not None and (stage != 'fixed_pregrasp'
                               or source.get('progress_policy') != 'feedback_paced'):
        raise ValueError('Endpoint retry requires an approved feedback-paced fixed-pregrasp source')
    return policy


def source_scheduling_policy(source, stage):
    policy = source.get('scheduling_policy', 'ack_then_period_v1')
    if policy not in ('ack_then_period_v1', 'ack_overlap_v1', 'shared_progress_v1', 'timed_group_v1'):
        raise ValueError('Unsupported source scheduling policy')
    if policy in ('ack_overlap_v1', 'shared_progress_v1') and (stage != 'fixed_pregrasp'
                                      or source.get('progress_policy') != 'feedback_paced'):
        raise ValueError('ACK overlap requires an approved feedback-paced fixed-pregrasp source')
    return policy


def execution_timing(source, args):
    """Use the recorded speed for preview and execution, rejecting silent edits."""
    recorded = source.get('execution_timing')
    timing = dict(rate=10., speed=.5, acceleration=1., respect_source_timing=True)
    if recorded is not None:
        if not isinstance(recorded, dict) or set(recorded) != set(timing):
            raise ValueError('Malformed source execution timing')
        timing.update(recorded)
    for argument, key in [('rate_hz', 'rate'), ('speed_deg_s', 'speed'),
                          ('acceleration_deg_s2', 'acceleration'),
                          ('respect_source_timing', 'respect_source_timing')]:
        requested = getattr(args, argument, None)
        if requested is None:
            continue
        if recorded is not None and requested != timing[key]:
            raise ValueError('Requested timing differs from source plan: '+argument)
        timing[key] = requested
    if any(isinstance(timing[key], bool) or not isinstance(timing[key], (int, float))
           or not math.isfinite(timing[key]) or timing[key] <= 0
           for key in ('rate', 'speed', 'acceleration')) or timing['rate'] > 10:
        raise ValueError('Speed/acceleration must be positive finite; rate must be in (0, 10] Hz')
    if not isinstance(timing['respect_source_timing'], bool):
        raise ValueError('Source timing policy must be a boolean')
    return timing


def bounded_prefix(full, maximum):
    """End at an existing zero-speed boundary; never cut a continuous run."""
    if maximum is not None and (not math.isfinite(maximum) or maximum <= 0):
        raise ValueError('--max-duration must be positive and finite')
    count = 0
    ticks = 0
    continuous = full.get('timing_profile') == 'collinear_bounded_v1'
    boundaries = [(r['count'], r['end_index']) for r in full['runs']] if continuous else [
        (edge_ticks, i+1) for i, edge_ticks in enumerate(full['ticks'])]
    run_count = 0
    for edge_ticks, end_index in boundaries:
        if maximum is not None and (ticks + edge_ticks) / full['rate'] > maximum + 1e-9:
            break
        ticks += edge_ticks
        count = end_index
        run_count += 1
    if not count:
        raise ValueError('Duration budget cannot contain one complete stopped path run/edge')
    result = copy.deepcopy(full)
    result['sdk'] = result['sdk'][:count + 1]
    result['ticks'] = result['ticks'][:count]
    result['duration_s'] = ticks / result['rate']
    if continuous:
        result['runs'] = result['runs'][:run_count]
        for key in ['waypoint_times_s', 'waypoint_velocities_deg_s', 'waypoint_accelerations_deg_s2']:
            result[key] = result[key][:count+1]
        result['continuous_internal_points'] = sum(r['end_index']-r['start_index']-1 for r in result['runs'])
        result['stopped_corners'] = max(0, run_count-1)
    result['active_joints'] = [i for i in range(6)
        if max(q[i] for q in result['sdk']) - min(q[i] for q in result['sdk']) > .001]
    return result, dict(truncated=count < len(full['ticks']),
                       original_point_count=len(full['sdk']),
                       start_point_index=0, target_point_index=count,
                       selected_point_indices=list(range(count + 1)),
                       complete_stage_duration_s=full['duration_s'],
                       max_duration_s=maximum)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True, help='plan_first_cube.py plan.json')
    parser.add_argument('--approved-plan-sha256', help='Exact approved plan bytes; checked before any hardware connection')
    parser.add_argument('--stage', required=True, help='One arm stage name, e.g. pregrasp')
    parser.add_argument('--expected-angles', type=float, nargs=7, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, help='Defaults to first_pick.yaml mapping candidate')
    parser.add_argument('--max-duration', type=float, help='Retimed path duration budget in seconds')
    parser.add_argument('--timing-profile', choices=['edge_quintic_v1', 'collinear_bounded_v1'],
                        help='Must match source plan; historical untagged plans use edge_quintic_v1')
    parser.add_argument('--speed-deg-s', type=float, help='Must match the speed recorded by the planner')
    parser.add_argument('--acceleration-deg-s2', type=float, help='Must match the planner acceleration')
    parser.add_argument('--rate-hz', type=float, help='Must match the planner command rate')
    source_timing = parser.add_mutually_exclusive_group()
    source_timing.add_argument('--respect-source-timing', dest='respect_source_timing', action='store_true')
    source_timing.add_argument('--retime-from-geometry', dest='respect_source_timing', action='store_false')
    parser.set_defaults(respect_source_timing=None)
    parser.add_argument('--stop-file', type=Path, help='Existence requests cancellation; never removed')
    parser.add_argument('--nic', default='enp99s0')
    parser.add_argument('--gripper-target', type=float,
                        help='Retain an existing J6 SDK command instead of its measured position; not jaw width')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--allow-position-start', action='store_true', help='Allow the approved first nonzero target to start position control once')
    args = parser.parse_args(argv)

    # ROS imports occur only after --help exits; none of these creates a ROS node.
    from rclpy.serialization import deserialize_message
    from moveit_msgs.msg import RobotTrajectory
    from d1_pick_place.configuration import load_config
    from d1_pick_place.hardware_bridge import trajectory_digest, trajectory_plan
    from d1_pick_place.sdk_executor import Mapping, CliTransport, Cancelled, LIMITS, HOST_BINARY, execute, healthy, feedback_limit_errors, joint_feedback_times

    try:
        errors=feedback_limit_errors(args.expected_angles)
        if errors:raise ValueError('; '.join(errors))
        if args.gripper_target is not None:
            if not math.isfinite(args.gripper_target) or not 0 <= args.gripper_target <= 65:
                raise ValueError('Gripper target must be finite and inside SDK [0,65]')
            if abs(args.gripper_target-args.expected_angles[6]) > 3:
                raise ValueError('Gripper target differs from reviewed feedback by >3 SDK units')
        source_bytes = args.plan.read_bytes()
        source_sha = check_approved_plan_digest(source_bytes, args.approved_plan_sha256)
        source = json.loads(source_bytes)
        endpoint_policy = source_endpoint_policy(source, args.stage)
        scheduling_policy = source_scheduling_policy(source, args.stage)
        position_delay_policy = source.get('position_delay_policy','archived_formula')
        progress_policy = source.get('progress_policy','scheduled')
        if (progress_policy not in ('scheduled','feedback_paced')
                or (progress_policy=='feedback_paced' and args.stage!='fixed_pregrasp')):
            raise ValueError('Feedback-paced progress requires an approved fixed-pregrasp source')
        if (position_delay_policy not in ('archived_formula','sdk_minimum','timed_segment_interval')
                or (position_delay_policy=='sdk_minimum' and args.stage!='fixed_pregrasp')
                or (position_delay_policy=='timed_segment_interval' and scheduling_policy!='timed_group_v1')):
            raise ValueError('SDK minimum delay is restricted to a reviewed fixed-pregrasp stage')
        if not source.get('completed'):
            raise ValueError('Source planning did not complete')
        matches = [stage for stage in source.get('stages', []) if stage.get('name') == args.stage]
        if len(matches) != 1:
            raise ValueError('Exactly one matching stage required')
        stage = matches[0]
        if stage.get('gripper_cad_position') is not None:
            raise ValueError('Gripper stage is not an arm trajectory')
        trajectory = deserialize_message(base64.b64decode(stage['trajectory_cdr_base64'], validate=True), RobotTrajectory)
        digest = trajectory_digest(trajectory)
        if digest != stage['sha256']:
            raise ValueError('Stored trajectory digest mismatch')
        candidate = args.candidate or Path(load_config(WS / 'src/d1_pick_place/config/first_pick.yaml')['hardware_mapping_candidate'])
        mapping = Mapping.from_candidate(candidate)
        timing_profile = source.get('timing_profile', 'edge_quintic_v1')
        if args.timing_profile is not None and args.timing_profile != timing_profile:
            raise ValueError('Timing profile differs from source plan; prepare and approve a matching plan')
        timing = execution_timing(source, args)
        full = trajectory_plan(trajectory, mapping, timing_profile, **timing)
        selected, selection = bounded_prefix(full, args.max_duration)
        if endpoint_policy is not None and selection['truncated']:
            raise ValueError('Endpoint retry requires the complete approved fixed-pregrasp endpoint')
        if any(abs(actual - expected) > (.3 if scheduling_policy=='timed_group_v1' else ARM_POSITION_TOLERANCE_DEG) + 1e-5
               for actual, expected in zip(selected['sdk'][0], args.expected_angles[:6])):
            raise ValueError('Reviewed start differs from trajectory start beyond this scheduling policy tolerance')
        plan_report = dict(stage=args.stage, source_plan=str(args.plan.resolve()),
                           source_plan_sha256=source_sha,
                           source_trajectory_sha256=digest, candidate=str(candidate.resolve()),
                           candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest(),
                           expected_angles=args.expected_angles, selection=selection, sdk_plan=selected,
                           execution_timing=timing,
                           position_delay_policy=position_delay_policy,
                           progress_policy=progress_policy,
                           scheduling_policy=scheduling_policy,
                           candidate_wait_policy=candidate_wait_policy(scheduling_policy),
                           endpoint_policy=endpoint_policy,
                           requested_gripper_target_sdk=args.gripper_target,
                           stop_file=None if args.stop_file is None else str(args.stop_file.resolve()),
                           automatic_next_stage=False, automatic_return=False, gripper_target_changed=False,
                           allow_position_start=args.allow_position_start,
                           endpoint_tolerance_deg=ENDPOINT_TOLERANCE_DEG,
                           feedback_filter=dict(scope='J0-J5',multiplier=3.,
                               max_speed_deg_s=timing['speed'],comparison='last_accepted_angle_and_time',
                               preserve_raw=True,refresh_time_on_reject=False),
                           note='Only changed arm targets are sent; J6 is monitored, never resent as a hold; no width/contact claim.')
        args.output.mkdir(parents=True, exist_ok=False)
        (args.output / 'plan.json').write_text(json.dumps(plan_report, indent=2) + '\n')
    except (ValueError, KeyError, OSError) as error:
        parser.error(str(error))

    print(json.dumps(dict(stage=args.stage, duration_s=selected['duration_s'],
        target_sdk=selected['sdk'][-1], selection=selection,
        plan_file=str(args.output/'plan.json'))), flush=True)
    report = dict(stage=args.stage, completed=False, executed=False, hardware_commands_sent=False,
                  endpoint_tolerance_deg=ENDPOINT_TOLERANCE_DEG,
                  endpoint_policy=endpoint_policy,
                  progress_policy=progress_policy,
                  scheduling_policy=scheduling_policy,
                  candidate_wait_policy=candidate_wait_policy(scheduling_policy),
                  selection=selection, progress=[], gripper_target_changed=False,
                  physical_stop_verified=False, hold_command_received=False,
                  execution_timing=timing,
                  requested_gripper_target_sdk=args.gripper_target)
    if not args.execute:
        report['plan_only'] = True
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
        return 0

    # Deliberately constructed only in the explicit execution branch.
    from handeye_joint_step import Cameras
    transport = cameras = None
    stopping = threading.Event()
    previous_handlers = {}
    def canceled():
        return stopping.is_set() or (args.stop_file is not None and args.stop_file.exists())
    def check_cancel():
        if canceled():
            raise Cancelled('Stop file or process cancellation requested')
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[sig] = signal.signal(sig, lambda *_: stopping.set())
        check_cancel()
        cameras = Cameras(args.output / 'frames')
        deadline = time.monotonic() + 5
        while not cameras.ready:
            check_cancel()
            if cameras.error or time.monotonic() > deadline:
                cameras.check()
            time.sleep(.05)
        cameras.check()
        check_cancel()
        if scheduling_policy=='timed_group_v1':
            from d1_pick_place.timed_group_transport import TimedGroupTransport
            transport=TimedGroupTransport(binary=HOST_BINARY.parent/'d1_timed_bridge',nic=args.nic,
                output=args.output/'transport.jsonl')
        else:
            transport = CliTransport(HOST_BINARY, args.nic, args.output / 'transport.jsonl',
                                     allow_position_start=args.allow_position_start,
                                     feedback_max_speed_deg_s=timing['speed'])
        transport.position_delay_policy=position_delay_policy
        deadline = time.monotonic() + 5
        while True:
            check_cancel()
            cameras.check()
            snapshot = transport.snapshot()
            try:
                measured = healthy(snapshot, transport.now(),allow_unstarted_position=args.allow_position_start or scheduling_policy=='timed_group_v1')
                if not snapshot.get('feedback_pending',False):break
                transport.wait(.01)
            except RuntimeError:
                if snapshot['fault'] or time.monotonic() > deadline:
                    raise
                transport.wait(.05)
        if any(abs(q - expected) > (.3 if scheduling_policy=='timed_group_v1' else .5 if i == 6 else ARM_POSITION_TOLERANCE_DEG) + 1e-5
               for i, (q, expected) in enumerate(zip(measured, args.expected_angles))):
            raise ValueError('Fresh feedback differs from reviewed pose; no motion')
        report['measured_start'] = measured.tolist()
        begin = transport.now()
        last_print = begin - 5
        last_sequence = -1
        def feedback(q):
            nonlocal last_print, last_sequence
            now = transport.now()
            current = transport.snapshot()
            if now - last_print >= 5 and current['q_seq'] != last_sequence:
                last_print = now
                last_sequence = current['q_seq']
                phase_state=getattr(transport,'shared_progress_state',{})
                phase=phase_state.get('source_elapsed_s')
                shared=scheduling_policy=='shared_progress_v1'
                timed_mode=scheduling_policy=='timed_group_v1'
                fraction=(min(1.,phase/selected['duration_s']) if isinstance(phase,(int,float))
                          else None) if shared else min(1.,(now-begin)/selected['duration_s'])
                item = dict(elapsed_s=round(now - begin, 3),
                            progress_fraction=None if timed_mode else fraction,
                            progress_basis=('board_state_in_transport_log' if timed_mode else
                                'commanded_source_phase' if shared else 'wall_time_estimate'),
                            source_elapsed_s=phase if shared else None,
                            sdk_feedback=q.tolist(), feedback_sequence=current['q_seq'],
                            feedback_age_s=now - current['q_time'], commands=transport.commands)
                report['progress'].append(item)
                print(json.dumps(dict(stage=args.stage, **item)), flush=True)
        report['executed'] = True
        report.update(execute(selected, transport, cancel=canceled,
                              external_check=cameras.check, feedback=feedback,
                              gripper_target=args.gripper_target,allow_position_start=args.allow_position_start,
                              endpoint_tolerance_deg=ENDPOINT_TOLERANCE_DEG,
                              endpoint_policy=endpoint_policy,
                              progress_policy=progress_policy,
                              scheduling_policy=scheduling_policy))
    except (Exception, KeyboardInterrupt) as error:
        report.update(execution_error_fields(error))
        if hasattr(error,'feedback_diagnostics'):
            report['feedback_failure_diagnostics']=error.feedback_diagnostics
            report.update({key:value for key,value in error.feedback_diagnostics.items()
                           if key.startswith(('feedback_gap_','feedback_recovery_','feedback_joint_'))
                           or key=='feedback_timeout_policy'})
        report.update(command_wait_pause_s=getattr(error,'command_wait_pause_s',0.),
                      hold_command_received=False, future_commands_stopped=True,
                      physical_stop_verified=False)
        for key in ('scheduling_policy', 'scheduling_overrun_s', 'command_group_schedule',
                    'shared_progress_metrics', 'phase_events', 'reference_epoch_monotonic_s'):
            if hasattr(error, key):
                report[key] = getattr(error, key)
    finally:
        if transport is not None:
            snapshot = transport.snapshot()
            report.update(commands=transport.commands, receipts=transport.acks,
                          command_count_semantics=('trajectory_messages' if scheduling_policy=='timed_group_v1' else 'joint_commands'),
                          hardware_commands_sent=transport.commands > 0,
                          final_angles=None if snapshot['q'] is None else snapshot['q'].tolist(),
                          final_status=snapshot['status'])
            report.update(feedback_rejected_count=snapshot.get('feedback_rejected_count',0),
                          feedback_pause_total_s=snapshot.get('feedback_pause_total_s',0.),
                          feedback_pending_at_end=snapshot.get('feedback_pending',False))
            now=transport.now()
            ages=now-joint_feedback_times(snapshot)
            report['joint_feedback_age_s']=[float(age) if math.isfinite(age) else None for age in ages]
            report['stale_feedback_joints']=[joint for joint,age in enumerate(ages) if age>.6]
            for name,key in [('raw','raw_q_time'),('valid','q_time')]:
                stamp=snapshot.get(key,-math.inf)
                report['final_'+name+'_feedback_age_s']=(max(0.,now-stamp)
                    if isinstance(stamp,(int,float)) and math.isfinite(stamp) else None)
            try:
                transport.close()
            except Exception as error:
                report.update(completed=False, transport_cleanup_error=str(error))
        if cameras is not None:
            try:
                cameras.close()
            except Exception as error:
                report.update(completed=False, camera_cleanup_error=str(error))
            report['camera_error'] = cameras.error
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        (args.output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('progress','selection')}, indent=2), flush=True)
    return 0 if report['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

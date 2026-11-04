#!/usr/bin/env python3
"""One J6 segment with user-selected speed and fresh feedback, no auto recovery.

SDK J6 position is not jaw width or force. Cancellation stops future commands,
without sending a same-position hold or claiming a physical emergency stop.
Default invocation only computes the segment; --execute is required for motion.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/d1_pick_place'))
from d1_pick_place.sdk_executor import (CliTransport, Cancelled, HOST_BINARY, LIMITS, healthy, FeedbackRecovery,
                                        FEEDBACK_JOINT_TOLERANCE_DEG, feedback_limit_errors,
                                        all_feedback_fresh, joint_feedback_times, joint_feedback_sequences)

RATE_HZ = 10.
PEAK_SPEED = .5
JOINT_LIMITS = LIMITS


def check_angles(angles):
    """Validate measured/reviewed feedback; no arm target is emitted here."""
    if len(angles) != 7 or not all(math.isfinite(x) for x in angles):
        raise ValueError('Seven finite SDK feedback values required')
    errors = feedback_limit_errors(angles)
    if errors:
        raise ValueError('; '.join(errors))
    # J6 is an SDK position, not an arm angle: preserve its exact old bounds.
    lo, hi = JOINT_LIMITS[6]
    if not lo <= angles[6] <= hi:
        raise ValueError(f'J6 outside existing command limits [{lo}, {hi}]')


def plan(target, expected, speed=.5, rate=10., segmented=False):
    check_angles(expected)
    if any(isinstance(value, bool) or not math.isfinite(value) or value <= 0
           for value in (speed, rate)) or rate > 10:
        raise ValueError('Speed must be positive finite; rate must be in (0, 10] Hz')
    if isinstance(target, bool) or not math.isfinite(target) or not 0 <= target <= 50:
        raise ValueError('First-grasp target must be within historical SDK range [0, 50]')
    if not 0 <= expected[6] <= 50:
        raise ValueError('First-grasp start must be within historical SDK range [0, 50]')
    delta = target - expected[6]
    if segmented:
        # Leave half a unit for the preceding segment's measured endpoint
        # tolerance, while every actually executed segment still checks <=10.
        count = max(1, math.ceil(abs(delta) / 9.5 - 1e-9))
        current = list(expected)
        segments = []
        for index in range(1, count + 1):
            endpoint = float(expected[6] + delta * index / count)
            segment = plan(endpoint, current, speed=speed, rate=rate)
            segments.append(segment)
            current[6] = endpoint
        return dict(joint=6, units='uncalibrated_sdk_position', segmented=True,
                    feedback_joint_tolerance_deg=FEEDBACK_JOINT_TOLERANCE_DEG,
                    reviewed_start=list(expected), target_sdk_units=target,
                    delta_sdk_units=delta, duration_s=sum(s['duration_s'] for s in segments),
                    rate_hz=rate, speed_sdk_s=speed, segments=segments,
                    single_segment_max_sdk=10., planned_segment_max_sdk=9.5,
                    existing_j6_command_limits=[0, 65], first_grasp_candidate_range=[0, 50],
                    automatic_return=False, automatic_open_on_failure=False,
                    hardware_commands_sent=False, contact_detected=False, grasp_verified=False,
                    note='Initial J6 adjustment after plan approval; SDK position is not jaw width or force.')
    if abs(delta) > 10 + 1e-9:
        raise ValueError('Single J6 segment must change by at most 10 SDK units')
    ticks = max(1, math.ceil(1.875 * abs(delta) / speed * rate - 1e-9))
    duration = ticks / rate
    return dict(joint=6, units='uncalibrated_sdk_position', reviewed_start=list(expected),
                feedback_joint_tolerance_deg=FEEDBACK_JOINT_TOLERANCE_DEG,
                target_sdk_units=target, delta_sdk_units=delta, duration_s=duration,
                rate_hz=rate, speed_sdk_s=speed, ticks=ticks,
                peak_reference_velocity_sdk_units_s=1.875 * abs(delta) / duration,
                existing_j6_command_limits=[0, 65], first_grasp_candidate_range=[0, 50],
                automatic_return=False, automatic_open_on_failure=False,
                hardware_commands_sent=False, contact_detected=False, grasp_verified=False,
                note='Position arrival only; no calibrated jaw width or contact-force feedback.')


def checked_start(measured, expected, target, speed=.5, rate=10.):
    check_angles(measured)
    check_angles(expected)
    if any(abs(a-b) > .3 + 1e-5 for a, b in zip(measured, expected)):
        raise ValueError('Feedback differs from reviewed start by >0.3; no motion')
    return plan(target, measured, speed=speed, rate=rate)


def endpoint_result(samples, target, now):
    recent = [(stamp, value) for stamp, value in samples if now - 1 <= stamp <= now]
    if len({stamp for stamp, _ in recent}) < 3:
        raise RuntimeError('Fewer than three fresh endpoint feedback samples')
    values = [value for _, value in recent]
    span = max(values) - min(values)
    error = max(abs(value-target) for value in values)
    if span > .5 + 1e-5 or error > .5 + 1e-5:
        raise RuntimeError('J6 endpoint not stable/within 0.5 SDK units; no further command')
    return dict(position_reached=True, feedback_count=len(values),
                feedback_span_sdk_units=span, max_error_sdk_units=error,
                final_sdk_units=values[-1], contact_detected=False, grasp_verified=False)


def feedback_pause_total(snapshot):
    """Completed rejected-feedback waits, measured by the feedback driver."""
    total = snapshot.get('feedback_pause_total_s', 0.)
    if isinstance(total, bool) or not math.isfinite(total) or total < 0:
        raise RuntimeError('Invalid feedback pause duration')
    return float(total)


class GripperTransport(CliTransport):
    """Use the same feedback/receipt channel, sending only J6 position commands."""
    def send_target(self, target, check):
        check()
        snapshot = self.snapshot()
        current = snapshot.get('q')
        rounded = round(float(target), 3)
        if ((current is not None and rounded == round(float(current[6]), 3))
                or rounded == getattr(self, '_last_j6_target', None)):
            return False
        while True:
            with self.lock:
                latest = self.snapshot()
                pending = latest.get('feedback_pending', False)
                fresh = all_feedback_fresh(latest, self.now())
                if not pending and fresh:
                    current = latest.get('q')
                    if ((current is not None and rounded == round(float(current[6]), 3))
                            or rounded == getattr(self, '_last_j6_target', None)):
                        return False
                    before = self.acks
                    pause_seen = feedback_pause_total(latest)
                    extra_pause_seen = float(getattr(self, 'feedback_recovery_extra_pause_s', 0.))
                    self.log.write(json.dumps(dict(t=self.now(), kind='command', joint=6,
                                                   target_sdk_units=rounded)) + '\n')
                    self.log.flush()
                    # Quantization, logging and the wire use the same value.
                    # The pending check shares the reader's lock with the write.
                    self.proc.stdin.write(f'j 6 {rounded:.3f}\n')
                    self.proc.stdin.flush()
                    self.commands += 1
                    self._last_j6_target = rounded
                    break
            check()
            self.wait(.01)
        until = self.now() + .6
        while True:
            check()
            snapshot = self.snapshot()
            total = feedback_pause_total(snapshot)
            until += max(0., total - pause_seen)
            pause_seen = total
            extra_pause = float(getattr(self, 'feedback_recovery_extra_pause_s', 0.))
            until += max(0., extra_pause - extra_pause_seen)
            extra_pause_seen = extra_pause
            with self.lock:
                if self.acks > before and not self.snapshot().get('feedback_pending', False):
                    return True
            if self.now() > until:
                raise TimeoutError('Receipt timeout; no next command')
            self.wait(.01)


def execute_segment(target, expected, transport, speed=.5, rate=10.,
                    cancel=lambda: False, external_check=lambda: None, report=None,
                    allow_position_start=False):
    """Pure transport interface for one segment; mockable without hardware."""
    report = {} if report is None else report
    report.update(completed=False, position_reached=False, contact_detected=False,
                  grasp_verified=False, physical_stop_verified=False,
                  hold_command_received=False, completed_segments=[],
                  non_target_max_drift_deg=0., final_phase='preflight')
    plan(target, expected, speed=speed, rate=rate)
    baseline = None
    reference = None
    begin = None
    pause_seen = 0.
    command_wait_pause_s = 0.
    recovery = FeedbackRecovery(transport, cancel=cancel, external_check=external_check,
        allow_unstarted_position=lambda: allow_position_start and transport.acks == 0)

    def total_pause(snapshot):
        return feedback_pause_total(snapshot) + recovery.extra_pause_s

    def check():
        nonlocal begin, pause_seen
        if baseline is None:
            # Preparation remains strict: a stale initial observation cannot
            # authorize the first motion or establish the reviewed baseline.
            while True:
                if cancel():
                    raise Cancelled('Stop file or process cancellation requested')
                external_check()
                snapshot = transport.snapshot()
                q = healthy(snapshot, transport.now(),
                            allow_unstarted_position=allow_position_start and transport.acks == 0)
                if not snapshot.get('feedback_pending', False):
                    break
                transport.wait(.01)
        else:
            q, snapshot = recovery.read()
        total = total_pause(snapshot)
        if begin is not None:
            begin += max(0., total - pause_seen)
        pause_seen = total
        check_angles(q)
        if baseline is not None:
            drift = float(np.max(np.abs(q[:6] - baseline[:6])))
            report['non_target_max_drift_deg'] = max(report['non_target_max_drift_deg'], drift)
            if drift > .5 + 1e-5:
                raise RuntimeError('Non-target joint drift > 0.5')
        if reference is not None and abs(float(q[6]) - reference) > 3. + 1e-5:
            raise RuntimeError('J6 tracking error > 3 SDK units')
        return q.copy(), snapshot

    try:
        # A single receipt/sample is insufficient for the starting pose.
        starts = []
        seen = None
        seen_joints = None
        deadline = transport.now() + 5.
        while len(starts) < 3:
            q, snapshot = check()
            checked_start(q, expected, target, speed=speed, rate=rate)
            sequences=joint_feedback_sequences(snapshot)
            if snapshot['q_seq'] != seen and (seen_joints is None or np.all(sequences>seen_joints)):
                starts.append(q)
                seen = snapshot['q_seq']
                seen_joints=sequences
            if transport.now() >= deadline:
                raise TimeoutError('Three fresh start feedback samples not received')
            if len(starts) < 3:
                transport.wait(.05)
        if float(np.max(np.ptp(np.asarray(starts), axis=0))) > .3 + 1e-5:
            raise RuntimeError('Robot not stationary during gripper preflight')
        baseline = starts[-1]
        actual = checked_start(baseline, expected, target, speed=speed, rate=rate)
        report.update(measured_start=baseline.tolist(), actual_segment_plan=actual)
        reference = float(baseline[6])
        report['last_commanded_target_sdk_units'] = None
        begin = motion_started = transport.now()
        report['final_phase'] = 'gripper_single_stage'
        if abs(actual['delta_sdk_units']) > 1e-9:
            # No t=0 command: do not send a deliberate same-position command.
            for index in range(1, actual['ticks'] + 1):
                while True:
                    check()
                    due = begin + index / rate
                    if transport.now() >= due:
                        break
                    transport.wait(min(.02, due - transport.now()))
                check()
                due = begin + index / rate
                if transport.now() - due > .5 / rate:
                    raise RuntimeError('Missed command schedule; no catch-up burst')
                u = index / actual['ticks']
                reference = float(baseline[6] + actual['delta_sdk_units'] *
                                  (10*u**3 - 15*u**4 + 6*u**5))
                if transport.send_target(reference, check) is not False:
                    report['last_commanded_target_sdk_units'] = round(reference, 3)
                    # Startup may wait for a new position-mode status/frame
                    # even after the raw receipt arrived. Preserve a complete
                    # next tick after that synchronous wait; never catch up.
                    extra = max(0., transport.now() - (begin + index / rate))
                    begin += extra
                    command_wait_pause_s += extra
                    report['command_wait_pause_s'] = command_wait_pause_s
        else:
            report['already_at_target'] = True
        report['final_phase'] = 'gripper_single_stage_settle'
        # Require new samples after the last command, including for zero move.
        settled_snapshot=transport.snapshot()
        seen = settled_snapshot['q_seq']
        seen_j6 = joint_feedback_sequences(settled_snapshot)[6]
        samples = deque(maxlen=1000)
        deadline = transport.now() + 5.
        settle_pause_seen = pause_seen
        while transport.now() < deadline:
            q, snapshot = check()
            total = total_pause(snapshot)
            deadline += max(0., total - settle_pause_seen)
            settle_pause_seen = total
            j6_sequence=joint_feedback_sequences(snapshot)[6]
            if snapshot['q_seq'] != seen and j6_sequence>seen_j6:
                seen = snapshot['q_seq']
                seen_j6=j6_sequence
                if abs(float(q[6]) - target) <= .5 + 1e-5:
                    samples.append((snapshot['q_time'], float(q[6])))
                else:
                    samples.clear()
                try:
                    endpoint = endpoint_result(samples, target, transport.now())
                except RuntimeError:
                    pass
                else:
                    report.update(endpoint=endpoint, completed=True, position_reached=True,
                                  completed_segments=['gripper_single_stage'], final_phase='completed',
                                  execution_duration_s=transport.now()-motion_started)
                    return report
            transport.wait(.02)
        raise TimeoutError('J6 endpoint not reached/stable; receipt was not completion')
    except BaseException as error:
        report.update(error=str(error) or type(error).__name__, future_commands_stopped=True,
                      hold_command_received=False, physical_stop_verified=False)
        diagnostics = getattr(error, 'feedback_diagnostics', None)
        if diagnostics is not None:
            report.update(diagnostics)
            report['feedback_failure_diagnostics'] = diagnostics
            report['feedback_diagnostics'] = diagnostics
        raise
    finally:
        report.update(recovery.metrics())
        report['command_wait_pause_s'] = command_wait_pause_s


def execute_segmented(target, expected, transport, speed=.5, rate=10.,
                      cancel=lambda: False, external_check=lambda: None, report=None,
                      allow_position_start=False):
    """Execute the reviewed initial J6 adjustment once, with verified segments.

    No retries, same-position holds or recovery targets are added on failure.
    Segment boundaries require measured completion before any next command.
    """
    report = {} if report is None else report
    reviewed = plan(target, expected, speed=speed, rate=rate, segmented=True)
    report.update(completed=False, position_reached=False, contact_detected=False,
                  grasp_verified=False, physical_stop_verified=False,
                  hold_command_received=False, completed_segments=[], segment_results=[],
                  segmented_plan=reviewed, non_target_max_drift_deg=0., final_phase='preflight')
    begin = transport.now()
    current = list(expected)

    def check_whole_adjustment():
        external_check()
        snapshot = transport.snapshot()
        q = snapshot.get('q')
        if q is not None:
            drift = max(abs(float(q[i])-expected[i]) for i in range(6))
            report['non_target_max_drift_deg'] = max(report['non_target_max_drift_deg'], drift)
            if drift > .5 + 1e-5:
                raise RuntimeError('Non-target joint drift > 0.5 across initial gripper adjustment')

    try:
        for index, segment in enumerate(reviewed['segments']):
            partial = {}
            report['segment_results'].append(partial)
            report['final_phase'] = f'prepare_gripper_segment_{index + 1}'
            execute_segment(segment['target_sdk_units'], current, transport,
                            speed=speed, rate=rate, cancel=cancel,
                            external_check=check_whole_adjustment, report=partial,
                            allow_position_start=allow_position_start)
            report['completed_segments'].append(f'prepare_gripper_segment_{index + 1}')
            report['last_commanded_target_sdk_units'] = partial.get('last_commanded_target_sdk_units')
            # The successful endpoint is measured, never assumed from a receipt.
            current = list(healthy(transport.snapshot(), transport.now(),
                                   allow_unstarted_position=allow_position_start and transport.acks == 0))
            check_angles(current)
        report.update(completed=True, position_reached=True, endpoint=partial['endpoint'],
                      measured_start=report['segment_results'][0]['measured_start'],
                      already_at_target=all(item.get('already_at_target', False)
                                            for item in report['segment_results']),
                      final_phase='completed', execution_duration_s=transport.now()-begin)
        return report
    except BaseException as error:
        report.update(error=str(error) or type(error).__name__, future_commands_stopped=True,
                      hold_command_received=False, physical_stop_verified=False,
                      execution_duration_s=transport.now()-begin)
        diagnostics = getattr(error, 'feedback_diagnostics', None)
        if diagnostics is not None:
            report.update(diagnostics)
            report['feedback_failure_diagnostics'] = diagnostics
            report['feedback_diagnostics'] = diagnostics
        raise
    finally:
        report['command_wait_pause_s']=sum(item.get('command_wait_pause_s',0.)
            for item in report.get('segment_results',[]))
        for key in ('feedback_gap_wait_s', 'feedback_gap_recoveries', 'feedback_recovery_extra_pause_s'):
            report[key] = sum(item.get(key, 0) for item in report.get('segment_results', []))
        report['feedback_joint_timeout_events']=[event for item in report.get('segment_results', [])
            for event in item.get('feedback_joint_timeout_events', [])]
        report['feedback_timeout_policy']='all_joints_stale'
        report['feedback_gap_timeout_s'] = 2.


def run(args, report):
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report.update(completed=False, position_reached=False, final_angles=None, final_status={},
                  commands=0, receipts=0, hardware_commands_sent=False,
                  physical_stop_verified=False, hold_command_received=False,
                  completed_segments=[], final_phase='preflight', camera_error=None)
    cameras = transport = None
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
        from handeye_joint_step import Cameras
        cameras = Cameras(args.output / 'frames')
        deadline = time.monotonic() + 5.
        while not cameras.ready:
            check_cancel()
            if cameras.error or time.monotonic() >= deadline:
                cameras.check()
            time.sleep(.05)
        cameras.check()
        check_cancel()
        allow_position_start = getattr(args, 'allow_position_start', False)
        transport = GripperTransport(HOST_BINARY, args.nic, args.output / 'transport.jsonl',
                                     allow_position_start=allow_position_start,
                                     feedback_max_speed_deg_s=getattr(args, 'arm_speed_deg_s', .5))
        deadline = time.monotonic() + 5.
        while True:
            check_cancel()
            cameras.check()
            snapshot = transport.snapshot()
            try:
                healthy(snapshot, transport.now(),
                        allow_unstarted_position=allow_position_start and transport.acks == 0)
                break
            except RuntimeError:
                if snapshot['fault'] or time.monotonic() >= deadline:
                    raise
                transport.wait(.05)
        execute = execute_segmented if getattr(args, 'segmented', False) else execute_segment
        execute(args.target, args.expected_angles, transport,
                speed=args.speed_sdk_s, rate=args.rate_hz, cancel=canceled,
                external_check=cameras.check, report=report,
                allow_position_start=allow_position_start)
    except (Exception, KeyboardInterrupt) as exc:
        report.update(error=str(exc) or type(exc).__name__, future_commands_stopped=True)
    finally:
        if transport is not None:
            snapshot = transport.snapshot()
            report.update(final_angles=None if snapshot['q'] is None else snapshot['q'].tolist(),
                          final_status=snapshot['status'], commands=transport.commands,
                          receipts=transport.acks, hardware_commands_sent=transport.commands > 0,
                          feedback_pending=snapshot.get('feedback_pending', False),
                          feedback_pause_total_s=feedback_pause_total(snapshot),
                          feedback_rejected_count=snapshot.get('feedback_rejected_count', 0))
            now=transport.now()
            failure_diagnostics = report.get('feedback_failure_diagnostics',
                                             report.get('feedback_diagnostics', {}))
            ages=now-joint_feedback_times(snapshot)
            report['joint_feedback_age_s']=failure_diagnostics.get('joint_feedback_age_s',
                [float(age) if math.isfinite(age) else None for age in ages])
            report['stale_feedback_joints']=failure_diagnostics.get('stale_feedback_joints',
                np.flatnonzero(ages>.6).tolist())
            for name,key in [('raw','raw_q_time'),('valid','q_time')]:
                output_key = 'final_'+name+'_feedback_age_s'
                if output_key in failure_diagnostics:
                    report[output_key] = failure_diagnostics[output_key]
                    continue
                stamp=snapshot.get(key,-math.inf)
                report[output_key]=(max(0.,now-stamp)
                    if isinstance(stamp,(int,float)) and math.isfinite(stamp) else None)
            try:
                transport.close()
            except Exception as exc:
                report.update(completed=False, cleanup_error=str(exc))
        if cameras is not None:
            try:
                cameras.close()
            except Exception as exc:
                report.update(completed=False, camera_cleanup_error=str(exc))
            report['camera_error'] = cameras.error
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        report['total_duration_s'] = time.monotonic() - started
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if report['completed'] else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', type=float, required=True, help='Absolute J6 SDK position, not mm')
    parser.add_argument('--expected-angles', type=float, nargs=7, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--nic', default='enp99s0')
    parser.add_argument('--speed-sdk-s', type=float, default=.5)
    parser.add_argument('--arm-speed-deg-s', type=float, default=.5,
                        help='Approved arm speed for J0-J5 feedback checks; J6 keeps SDK units')
    parser.add_argument('--rate-hz', type=float, default=10.)
    parser.add_argument('--stop-file', type=Path)
    parser.add_argument('--segmented', action='store_true',
                        help='Reviewed initial J6 adjustment using verified segments of at most 10 SDK units')
    parser.add_argument('--allow-position-start', action='store_true',
                        help='Allow the approved cycle first real position command to enter position mode once')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.arm_speed_deg_s) or args.arm_speed_deg_s <= 0:
            raise ValueError('Arm feedback speed must be positive and finite')
        report = plan(args.target, args.expected_angles, speed=args.speed_sdk_s, rate=args.rate_hz,
                      segmented=args.segmented)
    except ValueError as exc:
        parser.error(str(exc))
    if not args.execute:
        print(json.dumps(report, indent=2))
        return 0
    return run(args, report)


if __name__ == '__main__':
    raise SystemExit(main())

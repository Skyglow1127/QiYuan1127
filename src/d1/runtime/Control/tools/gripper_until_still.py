#!/usr/bin/env python3
"""J6-only 1-raw-unit closing steps until fresh position feedback stops changing.

Default is a plan only. This is NOT contact/force detection or grasp success.
Each step is 8 s quintic at 5 Hz. No enable, zero, retry, release, lift or return.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import statistics
import threading
import time

from handeye_joint_step import Cameras
from j4_smooth_test import JOINT_LIMITS, Session

ROOT = Path(__file__).resolve().parents[1]


class Cancelled(RuntimeError):
    pass


def classify_step(start, samples, before_sequence, motion_end, now):
    """Samples are (independent sequence, receive monotonic time, raw J6 value)."""
    new = [x for x in samples if x[0] > before_sequence]
    end = [x for x in new if motion_end <= x[1] <= now and now-x[1] <= 1.2]
    if len({x[0] for x in end}) < 5 or len({x[1] for x in end}) < 5:
        raise RuntimeError('Fewer than five new endpoint feedback samples; not a stall')
    if not math.isfinite(start) or any(not math.isfinite(x[2]) for x in new):
        raise RuntimeError('Non-finite position feedback; not a stall')
    values = [x[2] for x in end]
    span = max(values)-min(values)
    if span > .1 + 1e-5:
        raise RuntimeError('Endpoint span exceeds 0.1 raw; not stable')
    whole = [start] + [x[2] for x in new]
    whole_span = max(whole)-min(whole)
    maximum_change = max(abs(x-start) for x in whole)
    return dict(new_endpoint_samples=len(end), endpoint_span_raw=span,
                before_raw=start, after_raw=statistics.median(values),
                whole_step_span_raw=whole_span, maximum_change_raw=maximum_change,
                no_position_change=whole_span <= .1+1e-5 and maximum_change <= .1+1e-5)


def check_angles(q):
    if len(q) != 7 or any(not math.isfinite(v) or not lo <= v <= hi
                          for v, (lo, hi) in zip(q, JOINT_LIMITS)):
        raise RuntimeError('Seven finite in-limit actual positions required')


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--expected-angles', type=float, nargs=7, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stop-file', type=Path)
    p.add_argument('--nic', default='enp99s0')
    p.add_argument('--execute', action='store_true')
    args = p.parse_args(argv)
    try:
        check_angles(args.expected_angles)
    except RuntimeError as error:
        p.error(str(error))
    report = dict(completed=False, hardware_commands_sent=False, joint=6,
                  units='uncalibrated_sdk_position', expected_angles=args.expected_angles,
                  step_raw=-1., duration_s=8., rate_hz=5, peak_reference_speed_raw_s=.234375,
                  limits_raw=[0, 65], termination=None, steps=[], contact_detected=False,
                  grasp_verified=False, note='Unchanged fresh position feedback only; not contact or force evidence.')
    if not args.execute:
        print(json.dumps(report, indent=2))
        return 0
    args.output.mkdir(parents=True, exist_ok=False)
    session = cameras = None
    stopping = threading.Event()
    handlers = {}
    samples = deque(maxlen=5000)
    sequence = 0

    def cancel_check():
        if stopping.is_set() or (args.stop_file is not None and args.stop_file.exists()):
            raise Cancelled('Cancellation requested; no next closing command')

    def save():
        if session is not None:
            report.update(final_angles=session.monitor.angles, final_status=session.monitor.status,
                          commands=session.commands, receipts=session.acks,
                          hardware_commands_sent=session.commands > 0,
                          non_target_max_drift_deg=session.monitor.max_drift,
                          last_commanded_target_raw=session.monitor.target)
        (args.output / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')

    try:
        for sig in [signal.SIGINT, signal.SIGTERM]:
            handlers[sig] = signal.signal(sig, lambda *_: stopping.set())
        cancel_check()
        cameras = Cameras(args.output / 'frames')
        deadline = time.monotonic()+5
        while not cameras.ready:
            cancel_check()
            if cameras.error or time.monotonic() > deadline:
                cameras.check()
            time.sleep(.05)
        folder = args.output / 'motion'
        folder.mkdir()
        session = Session(ROOT / 'build/arm_control', args.nic, folder, 5, joint=6)
        original_check, original_observe = session.monitor.check, session.monitor.observe
        def check(now):
            original_check(now)
            check_angles(session.monitor.angles)
            cameras.check()
            cancel_check()
        def observe(message, now):
            nonlocal sequence
            original_observe(message, now)
            if message.get('address') == 2 and message.get('funcode') == 1:
                sequence += 1
                samples.append((sequence, now, session.monitor.angles[6]))
        session.monitor.check, session.monitor.observe = check, observe
        session.pump(3)
        check(time.monotonic())
        measured = list(session.monitor.angles)
        if any(abs(a-b) > (.5 if i == 6 else .3)+1e-5
               for i, (a, b) in enumerate(zip(measured, args.expected_angles))):
            raise RuntimeError('Current feedback differs from reviewed initial pose; no motion')
        session.monitor.baseline = measured
        session.monitor.target = measured[6]
        session.active = True
        save()
        while True:
            # Require fresh stable observations before each new downward target.
            seen = sequence
            deadline = time.monotonic()+3
            while True:
                session.pump(.05)
                check(time.monotonic())
                before = [x for x in samples if x[0] > seen][-5:]
                if len(before) >= 5 and max(x[2] for x in before)-min(x[2] for x in before) <= .1+1e-5:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('No five fresh stable pre-step frames; no next command')
            start = float(session.monitor.angles[6])
            if start <= .1+1e-5:
                report.update(completed=True, termination='lower_limit', lower_limit_feedback_raw=start)
                break
            target = max(0., start-1.)
            step = dict(index=len(report['steps'])+1, start_raw=start, target_raw=target)
            report['steps'].append(step)
            save()
            before_sequence = sequence
            begin = time.monotonic()
            endpoint_error = None
            try:
                session.segment('close_step_'+str(step['index']), start, target, 8)
            except RuntimeError as error:
                # All other errors, including feedback/status/camera faults,
                # abort immediately and can never count as unchanged feedback.
                if not str(error).startswith('Endpoint error > 0.5 position units;'):
                    raise
                endpoint_error = str(error)
            check(time.monotonic())
            result = classify_step(start, samples, before_sequence, begin+8, time.monotonic())
            step.update(result, endpoint_error=endpoint_error)
            print(json.dumps(step), flush=True)
            if result['no_position_change']:
                report.update(completed=True, termination='no_position_change')
                # Exactly one measured-position hold is permitted. Respect the
                # existing Session reference-step limit without relaxing it.
                hold = round(float(session.monitor.angles[6]), 3)
                difference = abs(hold-session.monitor.target)
                step['hold_target_delta_raw'] = difference
                if difference <= 1. and 0 <= hold <= 65:
                    check(time.monotonic())
                    session.send(hold)
                    step['measured_position_hold_received'] = True
                else:
                    step.update(measured_position_hold_received=False,
                                hold_skipped='Measured hold would exceed original one-unit reference-step limit')
                save()
                break
            if endpoint_error:
                raise RuntimeError(endpoint_error+' Movement occurred; this is not no-position-change termination')
            if target == 0.:
                report.update(completed=True, termination='lower_limit',
                              lower_limit_commanded=True, lower_limit_feedback_raw=result['after_raw'])
                save()
                break
            save()
    except (Exception, KeyboardInterrupt) as error:
        report.update(completed=False, error=str(error) or type(error).__name__,
                      termination='cancelled' if isinstance(error, (Cancelled, KeyboardInterrupt)) else 'fault')
    finally:
        save()
        if session is not None:
            try:
                session.close()
            except Exception as error:
                report.update(completed=False, cleanup_error=str(error))
        if cameras is not None:
            cameras.close()
            report['camera_error'] = cameras.error
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        save()
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Small SDK command/feedback mapping measurement; no claim of URDF calibration.

Only one arm joint per subprocess, no gripper commands. Uses the existing guarded
quintic runner. A 1-degree endpoint guard is distinct from the 0.3-degree accuracy
assessment. No compensation, enable, zero, power, release, automatic recovery.
"""
import argparse
import json
import math
import statistics
import time
from pathlib import Path
from j4_smooth_test import Session


def endpoints(folder, joint):
    samples = {}
    targets = {}
    for raw in (folder/'events.jsonl').read_text().splitlines():
        e = json.loads(raw)
        if e['kind'] == 'segment':
            targets[e['phase']+'_settle'] = e['end_deg']
        if e['kind'] != 'output' or not e['phase'].endswith('_settle') or '[rx] ' not in e['line']:
            continue
        d = json.loads(e['line'].split('[rx] ', 1)[1])
        if d.get('address') == 2 and d.get('funcode') == 1:
            samples.setdefault(e['phase'], []).append([d['data'][f'angle{i}'] for i in range(7)])
    result = []
    for phase, target in targets.items():
        tail = samples.get(phase, [])[-20:]
        if len(tail) < 10:
            result.append({'phase': phase, 'target_deg': target, 'usable': False, 'frames': len(tail)})
            continue
        median = [statistics.median(row[i] for row in tail) for i in range(7)]
        values = [row[joint] for row in tail]
        error = median[joint]-target
        result.append({'phase': phase, 'target_deg': target, 'usable': True, 'frames': len(tail),
                       'feedback_median_deg': median[joint], 'all_joints_median': median,
                       'feedback_span_deg': max(values)-min(values), 'signed_error_deg': error,
                       'within_0_3_deg': abs(error) <= .3+1e-5})
    return result


def check_home(angles):
    if (angles is None or len(angles) != 7 or not all(math.isfinite(q) for q in angles)
            or any(abs(q)>1.5 for q in angles[:6]) or abs(angles[6])>.5):
        raise RuntimeError('Expected measured zero vicinity: arms ±1.5°, J6 ±0.5 raw; no automatic homing')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--nic', default='enp99s0')
    parser.add_argument('--joints', type=int, nargs='+', default=list(range(6)))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if any(i not in range(6) for i in args.joints) or len(set(args.joints)) != len(args.joints):
        parser.error('Select distinct arm IDs 0..5 only')
    if not args.execute:
        print(f'Plan only J{args.joints}: +2→0→−2→0 deg, 8 s each leg + 3 s settle; 5 Hz. No hardware connection.')
        return 0
    args.output.mkdir(parents=True, exist_ok=False)
    summary = {'completed': False, 'requested_joints': args.joints, 'compensation': False,
               'rate_hz': 5, 'leg_seconds': 8, 'endpoint_guard_deg': 1, 'accuracy_threshold_deg': .3,
               'joint_mapping_validated': False, 'physical_axis_direction_validated': False,
               'zero_offset_calibrated': False, 'results': []}
    try:
        for joint in args.joints:
            output = args.output/f'j{joint}'
            output.mkdir()
            session = Session(Path(__file__).resolve().parents[1]/'build/arm_control', args.nic, output, 5, joint)
            item = {'joint': joint, 'completed': False}
            try:
                session.pump(3)
                session.monitor.check(time.monotonic())
                check_home(session.monitor.angles)
                item['start_angles'] = list(session.monitor.angles)
                session.monitor.baseline = list(session.monitor.angles)
                session.monitor.target = session.monitor.angles[joint]
                session.active = True
                start = session.monitor.target
                for phase, target in [('positive', 2), ('zero_from_positive', 0),
                                      ('negative', -2), ('zero_from_negative', 0)]:
                    session.segment(phase, start, target, 8)
                    start = target
                    phase_result = endpoints(output, joint)[-1]
                    if not phase_result['usable'] or phase_result['feedback_span_deg']>.3+1e-5:
                        raise RuntimeError('Endpoint lacks enough stable feedback; stop next phase')
                    print(f'J{joint} {phase}: median={phase_result["feedback_median_deg"]:.3f} '
                          f'error={phase_result["signed_error_deg"]:+.3f} accuracy_pass={phase_result["within_0_3_deg"]}', flush=True)
                item['completed'] = True
            except (Exception, KeyboardInterrupt) as exc:
                item['error'] = str(exc) or type(exc).__name__
                raise
            finally:
                item.update(final_angles=session.monitor.angles, commands=session.commands, receipts=session.acks,
                            non_target_max_drift=session.monitor.max_drift, completed_segments=session.completed_segments)
                session.close()
                item['endpoints'] = endpoints(output, joint)
                summary['results'].append(item)
                (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        summary['completed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        summary['error'] = str(exc) or type(exc).__name__
        print('STOP:', summary['error'], flush=True)
    finally:
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    return 0 if summary['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

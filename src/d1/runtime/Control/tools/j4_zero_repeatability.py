#!/usr/bin/env python3
"""Bounded J4 ±1 degree repeatability experiment; onboard feedback, not calibration."""
import argparse
import json
import math
import time
from pathlib import Path

from j4_smooth_test import Session
from joint_mapping_small_test import check_home, endpoints


class SmallSession(Session):
    def send(self, target):
        if not math.isfinite(target) or abs(target) > 1.000001:
            raise RuntimeError('Diagnostic target exceeds ±1 degree')
        return super().send(target)


def run(output, nic):
    output.mkdir(parents=True, exist_ok=False)
    result = dict(completed=False, joint=4, compensation=False,
                  physical_mapping_validated=False, mechanical_zero_calibrated=False,
                  rate_hz=5, leg_seconds=8, settle_seconds=10, cycles=3,
                  endpoint_guard_deg=1, accuracy_threshold_deg=.3, measurements=[])
    session = SmallSession(Path(__file__).resolve().parents[1]/'build/arm_control',
                           nic, output, 5, 4)
    try:
        session.pump(3)
        session.monitor.check(time.monotonic())
        check_home(session.monitor.angles)
        start = session.monitor.angles[4]
        if abs(start) > 1:
            raise RuntimeError('J4 start exceeds diagnostic ±1 degree region')
        result['initial_angles'] = list(session.monitor.angles)
        session.monitor.baseline = list(session.monitor.angles)
        session.monitor.target = start
        session.active = True
        stages = [('initial_zero', 0)]
        for cycle in range(1, 4):
            stages.extend((f'cycle{cycle}_{name}', target) for name, target in
                          [('positive', 1), ('zero_from_positive', 0),
                           ('negative', -1), ('zero_from_negative', 0)])
        for name, target in stages:
            item = dict(phase=name, target_deg=target)
            result['measurements'].append(item)
            session.segment(name, start, target, 8)
            item['at_3_seconds'] = endpoints(output, 4)[-1]
            # Passive dwell, not resending the target or retrying a failed leg.
            until = time.monotonic()+7
            while time.monotonic() < until:
                session.pump(min(.1, until-time.monotonic()))
                if abs(session.monitor.angles[4]-target) > 1+1e-5:
                    raise RuntimeError('Endpoint error exceeds 1 degree during passive dwell')
            late = endpoints(output, 4)[-1]
            item['at_10_seconds'] = late
            if not late['usable'] or late['feedback_span_deg'] > .3+1e-5:
                raise RuntimeError('Insufficient/still-changing endpoint feedback')
            print(f'{name}: 3s={item["at_3_seconds"]["feedback_median_deg"]:.3f} '
                  f'10s={late["feedback_median_deg"]:.3f} '
                  f'error={late["signed_error_deg"]:+.3f}', flush=True)
            start = target
            (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        result['completed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or type(exc).__name__
        print('STOP:', result['error'], flush=True)
    finally:
        result.update(final_angles=session.monitor.angles, commands=session.commands,
                      receipts=session.acks, max_non_target_drift=session.monitor.max_drift,
                      last_target=session.monitor.target, final_phase=session.phase)
        session.close()
        result['endpoints'] = endpoints(output, 4)
        (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    return 0 if result['completed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--nic', default='enp99s0')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        print('Plan only: J4 initial 0, then 3x(+1,0,-1,0); 8s legs, 10s passive settle, 5Hz. No DDS connection.')
    else:
        raise SystemExit(run(args.output, args.nic))

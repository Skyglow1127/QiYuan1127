#!/usr/bin/env python3
"""Save a stationary wrist RGB observation and fresh robot feedback; sends no command."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import sys
import time
import urllib.request
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/d1_pick_place'))
from d1_pick_place.sdk_executor import (CliTransport, healthy, HOST_BINARY,
    all_feedback_fresh, feedback_limit_errors, FEEDBACK_FRESHNESS_S)


def observation_feedback(snapshot, now, *, allow_position_start=False, allow_unpowered=False):
    """Read-only planning may observe power=0; motion still requires power=1."""
    if not allow_unpowered:
        return healthy(snapshot, now, allow_unstarted_position=allow_position_start)
    if snapshot['fault']:
        raise RuntimeError(snapshot['fault'])
    if not all_feedback_fresh(snapshot, now):
        raise RuntimeError('Angle feedback stale or pending')
    errors = feedback_limit_errors(snapshot['q'])
    if errors:
        raise RuntimeError('Actual joint outside limits: '+'; '.join(errors))
    for key, allowed in [('power_status', (0, 1)),
                         ('enable_status', (0, 1) if allow_position_start else (1,)),
                         ('error_status', (0,))]:
        if (type(snapshot['status'].get(key)) is not int
                or snapshot['status'].get(key) not in allowed
                or not 0 <= now-snapshot['status_time'].get(key, float('-inf')) <= FEEDBACK_FRESHNESS_S):
            raise RuntimeError('Invalid/stale '+key)
    return snapshot['q']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-position-start', action='store_true', help='Only read enable=0 as an unstarted position session; sends nothing')
    parser.add_argument('--allow-unpowered-observation', action='store_true',
                        help='Read-only method preparation may observe power=0; sends no power or motion commands')
    parser.add_argument('--arm-only-observation', action='store_true',
                        help='Fixed-pose method planning only: save fresh arm feedback without starting or reading cameras')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    transport = CliTransport(HOST_BINARY, 'enp99s0', args.output / 'readonly.jsonl')
    try:
        time.sleep(1)
        samples = []
        for _ in range(10):
            samples.append(observation_feedback(transport.snapshot(), transport.now(),
                allow_position_start=args.allow_position_start,
                allow_unpowered=args.allow_unpowered_observation).tolist())
            time.sleep(.1)
        if np.ptp(samples, axis=0).max() > .3 + 1e-5:
            raise RuntimeError('Robot not stationary during observation')
        if args.arm_only_observation:
            snapshot = transport.snapshot()
            angles = observation_feedback(snapshot, transport.now(),
                allow_position_start=args.allow_position_start,
                allow_unpowered=args.allow_unpowered_observation)
            report = dict(captured_at=datetime.datetime.now().astimezone().isoformat(),
                source_kind='arm_feedback_only', image=None, camera_observation_performed=False,
                sdk_angles=angles.tolist(), feedback_age_s=transport.now()-snapshot['q_time'],
                status=snapshot['status'], status_fresh=True,
                power_on_required=snapshot['status'].get('power_status') == 0,
                unpowered_observation_allowed=args.allow_unpowered_observation,
                stationary_span_deg=np.ptp(samples, axis=0).tolist(),
                hardware_commands_sent=transport.commands)
            (args.output / 'wrist_observation.json').write_text(json.dumps(report, indent=2))
            print(json.dumps(report))
            return
        for role, port in [('global', 18761), ('wrist', 18760)]:
            with urllib.request.urlopen(f'http://127.0.0.1:{port}/status', timeout=1) as response:
                status = json.load(response)
            serial = 'CC1365302K0' if role == 'global' else 'CC1365301EY'
            if status.get('error') or status.get('stopped') or status['device']['serial'] != serial:
                raise RuntimeError('Unexpected camera status')
            with urllib.request.urlopen(f'http://127.0.0.1:{port}/color.jpg', timeout=1) as response:
                raw, headers = response.read(), dict(response.headers)
            if not 0 <= float(headers['X-Board-Age-Ms']) <= 250:
                raise RuntimeError('Stale image')
            image = args.output / f'{role}.jpg'
            image.write_bytes(raw)
            snapshot = transport.snapshot()
            angles = observation_feedback(snapshot, transport.now(),
                allow_position_start=args.allow_position_start,
                allow_unpowered=args.allow_unpowered_observation)
            report = dict(captured_at=datetime.datetime.now().astimezone().isoformat(),
                image=str(image.resolve()), image_sha256=hashlib.sha256(raw).hexdigest(),
                headers=headers, camera_serial=serial, sdk_angles=angles.tolist(),
                feedback_age_s=transport.now()-snapshot['q_time'], status=snapshot['status'],
                power_on_required=snapshot['status'].get('power_status') == 0,
                unpowered_observation_allowed=args.allow_unpowered_observation,
                status_fresh=True,
                stationary_span_deg=np.ptp(samples, axis=0).tolist(), hardware_commands_sent=transport.commands)
            (args.output / f'{role}_observation.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report))
    finally:
        transport.close()


if __name__ == '__main__':
    main()

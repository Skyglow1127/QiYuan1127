#!/usr/bin/env python3
"""Camera HTTP-only soak: never constructs a Robot or opens a CAN/RPC client."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), '/home/aup/Lerobot/projects/piper/deployment',
               '/home/aup/Lerobot/projects/piper/official_adapter']


def summary(values):
    return {'count': len(values), 'median': statistics.median(values),
            'max': max(values)} if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--adapter', choices=('original', 'recovery'), required=True)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error('seconds must be within1..60')
    from lerobot_robot_piper_shared.devices import PiperHttpCameraConfig, PiperHttpCamera
    from lerobot_robot_piper_executor.backend import PiperBackend
    if args.adapter == 'recovery':
        from camera_recovery import VLACameraRecoveryBackend, VLABoundedHttpCamera
        backend_type, camera_type = VLACameraRecoveryBackend, VLABoundedHttpCamera
    else:
        backend_type, camera_type = PiperBackend, PiperHttpCamera
    config = json.loads((HERE/'pi05_observe_config.json').read_text())
    if config.get('execute') is not False:
        raise RuntimeError('Probe requires explicit execute=false')
    io = json.loads(Path(config['io_config']).read_text())
    # This object must never perform an RPC, including startup/cleanup.
    class NoRPC:
        def __getattr__(self, name):
            raise AssertionError('Camera-only probe attempted RPC: '+name)
    backend = backend_type(config, io, client=NoRPC())
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    try:
        backend.pool = ThreadPoolExecutor(max_workers=2)
        for role in ('global', 'wrist'):
            c = io['cameras'][role]
            camera = camera_type(PiperHttpCameraConfig(role=role, url=c['url'], serial=c['serial']))
            backend.cameras[role] = camera
            camera.connect()
        started = time.monotonic()
        next_tick = started
        with (args.output/'samples.jsonl').open('w') as stream:
            while time.monotonic()-started < args.seconds:
                before = time.monotonic()
                row = {'index': len(rows), 'elapsed_s': before-started}
                try:
                    images, source = backend._read_camera_pair()
                    now = time.monotonic()
                    metadata = {r: dict(c.last_metadata) for r,c in backend.cameras.items()}
                    stamps = [m['estimated_capture_monotonic'] for m in metadata.values()]
                    row.update(success=True, metadata=metadata,
                        pair_age_ms=(now-source)*1000,
                        pair_skew_ms=(max(stamps)-min(stamps))*1000,
                        images_shape={r: list(i.shape) for r,i in images.items()})
                except Exception as error:
                    row.update(success=False, error_type=type(error).__name__, error=str(error))
                row['acquisition_ms'] = (time.monotonic()-before)*1000
                row['pair_diagnostic'] = getattr(backend, 'last_camera_pair_timing', None)
                row['fetch_diagnostics'] = {r: getattr(c,'last_fetch_diagnostic',None)
                                            for r,c in backend.cameras.items()}
                rows.append(row)
                stream.write(json.dumps(row, ensure_ascii=False)+'\n')
                stream.flush()
                next_tick = max(next_tick+.05, time.monotonic())
                time.sleep(max(0., next_tick-time.monotonic()))
    finally:
        if backend.pool is not None:
            backend.pool.shutdown(wait=True, cancel_futures=True)
        for camera in backend.cameras.values():
            camera.disconnect()
    passed = [r for r in rows if r['success']]
    report = {'created_utc': datetime.now(timezone.utc).isoformat(),
              'adapter': args.adapter, 'duration_s': args.seconds,
              'attempts': len(rows), 'passed': len(passed), 'rejected': len(rows)-len(passed),
              'robot_commands_sent': 0, 'rpc_opened': False,
              'pair_age_ms': summary([r['pair_age_ms'] for r in passed]),
              'pair_skew_ms': summary([r['pair_skew_ms'] for r in passed]),
              'acquisition_ms': summary([r['acquisition_ms'] for r in rows]),
              'http_rtt_ms': {role: summary([r['metadata'][role]['http_rtt_ms'] for r in passed])
                              for role in ('global','wrist')},
              'scope': 'HTTP cameras only; not policy, CAN, future feedback or physical grasp'}
    (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

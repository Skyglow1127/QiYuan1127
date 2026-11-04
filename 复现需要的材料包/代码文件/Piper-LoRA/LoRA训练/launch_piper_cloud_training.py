#!/usr/bin/env python3
"""Launch a detached training job on the configured cloud instance.

Run on the cloud: python launch_piper_cloud_training.py smoke
After inspecting that result: python launch_piper_cloud_training.py train
Each invocation creates a new run directory and prints its PID and log path.
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('smoke', 'train'))
    parser.add_argument('--steps', type=int)
    parser.add_argument('--dataset-root', type=Path, default=Path(
        '/root/autodl-tmp/lerobot_act/datasets/rad_place_merged_v5'
    ))
    args = parser.parse_args()
    steps = args.steps if args.steps is not None else (100 if args.stage == 'smoke' else 20000)
    if steps < 1:
        parser.error('--steps must be positive')
    if not (args.dataset_root / 'meta/info.json').is_file():
        parser.error('Dataset root is missing meta/info.json')

    root = Path('/root/autodl-tmp/piper_pi05')
    source = json.loads((root / 'models/pi05_base/source_manifest.json').read_text())
    if not source.get('sha256_verified'):
        parser.error('Base weights have not passed transfer verification')
    now = datetime.now(ZoneInfo('Asia/Singapore'))
    run_id = f'pi05_{args.stage}_{now:%Y%m%d_%H%M%S}'
    run_dir = root / 'runs' / run_id
    log_path = root / 'logs' / f'{run_id}.log'
    if run_dir.exists():
        parser.error('This run directory already exists; retry with a fresh timestamp')
    with log_path.open('xb') as log:
        proc = subprocess.Popen(
            ['bash', str(root / 'bin/start_piper_cloud_training.sh'),
             str(args.dataset_root), str(run_dir), str(steps)],
            cwd=root, stdin=subprocess.DEVNULL, stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True,
        )
    receipt = {
        'stage': args.stage, 'steps': steps, 'pid': proc.pid,
        'process_group': proc.pid, 'started_at': now.isoformat(),
        'run_dir': str(run_dir), 'log_path': str(log_path),
        'dataset_root': str(args.dataset_root),
        'base_revision': source['revision'],
    }
    (root / 'logs' / f'latest_{args.stage}.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Launch a detached local training job using preloaded model assets.

Run a short check: python launch_piper_local_training.py smoke
Start training after reviewing it: python launch_piper_local_training.py train
Training writes models/PI05/piper_lora; --run-dir selects a different new path.
Each invocation prints its local PID, output directory, and log path.
"""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
from zoneinfo import ZoneInfo


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('smoke', 'train'))
    parser.add_argument('--steps', type=int)
    parser.add_argument('--root', type=Path, default=Path(
        os.environ.get('PIPER_PI05_ROOT', '/home/aup/Lerobot')))
    parser.add_argument('--dataset-root', type=Path)
    parser.add_argument('--base-model', type=Path)
    parser.add_argument('--tokenizer', type=Path)
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--python', type=Path)
    args = parser.parse_args()
    steps = args.steps if args.steps is not None else (100 if args.stage == 'smoke' else 20000)
    if steps < 1:
        parser.error('--steps must be positive')

    root = args.root.expanduser().resolve()
    dataset = (args.dataset_root or root / 'datasets/rad_place_merged_v5').expanduser().resolve()
    base = (args.base_model or Path(os.environ.get(
        'PI05_BASE_MODEL_PATH', str(root / 'models/PI05/pi05_base')))).expanduser().resolve()
    tokenizer = (args.tokenizer or Path(os.environ.get(
        'PI05_TOKENIZER_PATH', str(root / 'models/PI05/paligemma_tokenizer')))).expanduser().resolve()
    python = (args.python or Path(os.environ.get(
        'PIPER_PI05_PYTHON', str(root / 'envs/pi05-rocm-7.14.1/bin/python')))).expanduser().absolute()
    if not root.is_dir():
        parser.error(f'Local workspace does not exist: {root}')
    for filename in ('info.json', 'stats.json'):
        if not (dataset / 'meta' / filename).is_file():
            parser.error(f'Dataset is missing meta/{filename}: {dataset}')
    for filename in ('config.json', 'model.safetensors', 'policy_preprocessor.json', 'policy_postprocessor.json'):
        if not (base / filename).is_file():
            parser.error(f'Preloaded base model is missing {filename}: {base}')
    if not (tokenizer / 'tokenizer_config.json').is_file() or not any(
        (tokenizer / name).is_file() for name in ('tokenizer.json', 'tokenizer.model', 'spiece.model')
    ):
        parser.error(f'Preload the local PaliGemma tokenizer at: {tokenizer}')
    if not python.is_file() or not os.access(python, os.X_OK):
        parser.error(f'Local training Python is unavailable; use --python: {python}')

    now = datetime.now(ZoneInfo('Asia/Singapore'))
    run_id = f'pi05_{args.stage}_{now:%Y%m%d_%H%M%S_%f}'
    default_run = root / ('models/PI05/piper_lora' if args.stage == 'train' else f'runs/PI05/{run_id}')
    run_dir = (args.run_dir or default_run).expanduser().resolve()
    if run_dir.exists():
        parser.error(f'Output directory already exists; choose --run-dir: {run_dir}')
    entry = Path(__file__).resolve().parent / 'start_piper_local_training.sh'
    if not entry.is_file():
        parser.error(f'Missing companion entry point: {entry}')
    log_dir = root / 'logs/PI05'
    log_dir.mkdir(parents=True, exist_ok=True)
    run_dir.parent.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f'{run_id}.log'
    env = os.environ.copy()
    env.update(PIPER_PI05_ROOT=str(root), PI05_BASE_MODEL_PATH=str(base),
               PI05_TOKENIZER_PATH=str(tokenizer), PIPER_PI05_PYTHON=str(python))
    with log_path.open('xb') as log:
        proc = subprocess.Popen(
            ['bash', str(entry), str(dataset), str(run_dir), str(steps)],
            cwd=root, env=env, stdin=subprocess.DEVNULL, stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True,
        )
    receipt = {
        'stage': args.stage, 'steps': steps, 'pid': proc.pid,
        'process_group': proc.pid, 'started_at': now.isoformat(),
        'run_dir': str(run_dir), 'log_path': str(log_path),
        'dataset_root': str(dataset), 'base_model': str(base),
        'tokenizer': str(tokenizer), 'python': str(python),
    }
    (log_dir / f'latest_{args.stage}.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()

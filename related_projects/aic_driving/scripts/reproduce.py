#!/usr/bin/env python3
"""Run the real collect/train/calibrate/tune/evaluate/report pipeline in a fresh directory."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import zipfile

from aic_drive.runner import ROOT, dump_json, source_manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    p.add_argument('--full', action='store_true', help='90 collection + 270 tuning + 1800 main + 360 stress episodes')
    a = p.parse_args()
    out = Path(a.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / 'logs').mkdir()
    manifest = source_manifest()
    dump_json(out / 'source_manifest.json', manifest)
    with zipfile.ZipFile(out / 'source.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in sorted(set(manifest['files']) | {'requirements.lock', 'requirements-torch.lock', 'README.md'}):
            if (ROOT / name).exists():
                archive.write(ROOT / name, name)
        for path in (ROOT / 'tests').glob('test_*.py'):
            archive.write(path, str(path.relative_to(ROOT)))
    env = dict(os.environ)
    env.pop('PYTHONPATH', None)
    started = time.perf_counter()
    stages = []

    def run(name, script, *args):
        command = [sys.executable, str(ROOT / 'scripts' / script), *map(str, args)]
        print(json.dumps({'stage': name, 'status': 'running'}, ensure_ascii=False), flush=True)
        step_start = time.perf_counter()
        with (out / 'logs' / f'{name}.log').open('w') as log:
            process = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        stages.append({'name': name, 'command': command, 'returncode': process.returncode,
                       'wall_seconds': time.perf_counter() - step_start})
        dump_json(out / 'pipeline.json', {'mode': 'full' if a.full else 'quick', 'stages': stages,
                   'status': 'running' if process.returncode == 0 else 'failed'})
        if process.returncode:
            raise RuntimeError(f'{name} failed; inspect {out / "logs" / (name + ".log")}')
        print(json.dumps({'stage': name, 'status': 'completed', 'seconds': round(stages[-1]['wall_seconds'], 2)}), flush=True)

    data, model = out / 'data', out / 'model'
    run('collect', 'collect_data.py', '--output', data, '--episodes-per-scenario', 30 if a.full else 20)
    run('train', 'train_predictor.py', '--dataset', data / 'dataset.npz', '--output', model,
        '--epochs', 80 if a.full else 5, '--seed', 0)
    run('calibrate', 'calibrate.py', '--dataset', data / 'dataset.npz', '--model', model / 'model.pt',
        '--output', model / 'calibration.json')
    common = ['--model', model / 'model.pt', '--calibration', model / 'calibration.json',
              '--position-noise', .1, '--velocity-noise', .1, '--duration', 30]
    # The selection rule is fixed before opening final test episodes. All methods
    # use the same selected margin; adaptive methods do not use this parameter.
    candidates = [.5, 2., 4.] if a.full else [2.]
    scores = []
    for margin in candidates:
        name = f'tuning_margin_{margin:g}'
        run(name, 'evaluate.py', '--output', out / name, '--seeds', 5 if a.full else 1,
            '--seed-start', 30000, '--methods', 'ca_fixed', 'ml_fixed', '--fixed-margin', margin, *common)
        with (out / name / 'episodes.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        scores.append({'margin': margin, 'collisions': sum(int(r['collision']) for r in rows),
                       'mean_progress_rate': sum(float(r['progress_rate']) for r in rows) / len(rows),
                       'episodes': len(rows)})
    best = min(scores, key=lambda r: (r['collisions'], -r['mean_progress_rate'], abs(r['margin'] - 2)))
    dump_json(out / 'selection.json', {'rule': 'min pooled collisions, then max pooled progress, tie nearest 2m',
              'split': 'development tuning seeds 30000 onward only', 'candidates': scores,
              'selected_fixed_margin': best['margin'], 'applied_to_both_fixed_methods': True})
    run('main', 'evaluate.py', '--output', out / 'main', '--seeds', 50 if a.full else 1,
        '--seed-start', 100000, '--methods', 'ca_fixed', 'ca_adaptive', 'ml_fixed', 'ml_adaptive',
        '--fixed-margin', best['margin'], *common)
    run('stress', 'evaluate.py', '--output', out / 'stress', '--seeds', 10 if a.full else 1,
        '--seed-start', 200000, '--methods', 'ca_adaptive', 'ml_adaptive', 'age_guard', '--profile', 'holdout',
        '--delays', .2, .4, '--delay-modes', 'fixed', 'bursty', *common)
    run('report_main', 'make_report.py', '--input', out / 'main' / 'episodes.csv', '--output', out / 'report_main')
    run('report_stress', 'make_report.py', '--input', out / 'stress' / 'episodes.csv', '--output', out / 'report_stress')
    run('paired_main', 'compare_methods.py', '--input', out / 'main' / 'episodes.csv', '--output', out / 'paired_main')
    run('paired_stress', 'compare_methods.py', '--input', out / 'stress' / 'episodes.csv', '--output', out / 'paired_stress')
    run('report_prediction', 'prediction_report.py', '--model-dir', model, '--output', out / 'report_prediction')
    from aic_drive.visualization import render_replay, export_run_plot
    with (out / 'main' / 'episodes.csv').open() as stream:
        results = list(csv.DictReader(stream))
    (out / 'videos').mkdir()
    print(json.dumps({'stage': 'render_saved_episodes', 'status': 'running'}), flush=True)
    for scenario in ['follow', 'brake', 'cutin']:
        chosen = next(r for r in results if r['scenario'] == scenario and r['method'] == 'ml_adaptive'
                      and r['seed'] == '100000' and float(r['delay']) == .2)
        render_replay(chosen['run_dir'], out / 'videos' / f'{scenario}.mp4', fps=10)
        export_run_plot(chosen['run_dir'])
    print(json.dumps({'stage': 'render_saved_episodes', 'status': 'completed'}), flush=True)
    if source_manifest()['source_sha256'] != manifest['source_sha256']:
        raise RuntimeError('Source changed during execution; do not present this run as a frozen experiment')
    dump_json(out / 'pipeline.json', {'mode': 'full' if a.full else 'quick', 'stages': stages,
              'status': 'completed', 'wall_seconds': time.perf_counter() - started,
              'source_sha256': manifest['source_sha256']})
    print(str(out), flush=True)


if __name__ == '__main__':
    main()

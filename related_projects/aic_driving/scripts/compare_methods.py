#!/usr/bin/env python3
"""Paired progress differences with seed-cluster bootstrap intervals."""
import argparse
import csv
from collections import defaultdict
from pathlib import Path
import numpy as np
from aic_drive.runner import dump_json

KEYS = ('scenario','seed','delay','delay_mode','profile','position_noise','velocity_noise','budget_s')
CONTRASTS = [('ca_adaptive','ca_fixed'), ('ml_fixed','ca_fixed'),
             ('ml_adaptive','ca_adaptive'), ('ml_adaptive','ml_fixed'), ('age_guard','ca_adaptive')]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    with Path(a.input).open() as f:
        rows = list(csv.DictReader(f))
    if not rows or any(r['status'] != 'completed' for r in rows):
        raise ValueError('Only nonempty completed experiment tables are supported')
    indexed = defaultdict(dict)
    for row in rows:
        key = tuple(row[k] for k in KEYS)
        if key in indexed[row['method']]:
            raise ValueError('Duplicate method/condition rows; do not pool unrelated batches')
        indexed[row['method']][key] = row
    summaries = []
    for left, right in CONTRASTS:
        if left not in indexed or right not in indexed:
            continue
        if indexed[left].keys() != indexed[right].keys():
            raise ValueError(f'Incomplete pairing for {left} vs {right}')
        by_seed = defaultdict(list)
        collisions = [0, 0]
        for key in sorted(indexed[left]):
            l, r = indexed[left][key], indexed[right][key]
            by_seed[l['seed']].append(float(l['progress_rate']) - float(r['progress_rate']))
            collisions[0] += int(l['collision'])
            collisions[1] += int(r['collision'])
        differences = np.array([np.mean(by_seed[s]) for s in sorted(by_seed)])
        rng = np.random.default_rng(20261023)
        boot = rng.choice(differences, (2000, len(differences)), replace=True).mean(axis=1)
        ci = np.quantile(boot, [.025, .975]).tolist() if len(differences) >= 2 else None
        summaries.append({'left': left, 'right': right, 'paired_conditions': len(indexed[left]),
                          'seed_clusters': len(differences), 'mean_progress_difference_mps': float(differences.mean()),
                          'bootstrap95_ci_mps': ci, 'collision_counts_left_right': collisions,
                          'bootstrap_resamples': 2000})
    dump_json(out / 'paired.json', {'definition': 'left minus right; each seed block contains all scenarios and delays; seed clusters weighted equally',
                                  'caveat': 'Exploratory percentile intervals, no multiple-testing correction. Same-seed conditions are not counted as independent episodes.',
                                  'contrasts': summaries})
    text = ['# 配对比较', '', '按相同场景、种子、延迟、噪声和时间预算配对。差值为左方法减右方法，正数表示进度更高。',
            '同一种子的各场景及延迟归为一个块，再做 2,000 次 bootstrap。区间是探索性估计，未作多重检验校正。', '',
            '| 左 − 右 | 配对条件 | 独立种子块 | 进度差 m/s | 95% 区间 m/s | 碰撞数（左/右） |',
            '|---|---:|---:|---:|---|---|']
    for r in summaries:
        ci = r['bootstrap95_ci_mps']
        interval = f'[{ci[0]:.4f}, {ci[1]:.4f}]' if ci else '样本不足'
        text.append(f"| {r['left']} − {r['right']} | {r['paired_conditions']} | {r['seed_clusters']} | {r['mean_progress_difference_mps']:.4f} | {interval} | {r['collision_counts_left_right'][0]}/{r['collision_counts_left_right'][1]} |")
    text += ['', '零碰撞不能用来证明风险为零或某种方法更安全；各条件碰撞率的 Wilson 区间见批量报告。',
             '进度改进需结合 jerk、急减速和计算代价判断，不能只选一项指标。', '']
    (out / 'report.md').write_text('\n'.join(text))
    print(out / 'report.md')


if __name__ == '__main__':
    main()

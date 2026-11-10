#!/usr/bin/env python3
"""Report v2 paired control results without treating windows as independent trials."""
import argparse,csv,json
from collections import defaultdict
from pathlib import Path
import numpy as np
from aic_drive.runner import dump_json

def analyze(path):
    with Path(path).open() as f: rows=list(csv.DictReader(f))
    groups=defaultdict(list)
    for r in rows:
        if r['status']!='completed':raise ValueError('Incomplete episode')
        groups[r['method']].append(r)
    summary={m:{'episodes':len(rs),'collisions':sum(int(r['collision']) for r in rs),
        **{k:float(np.mean([float(r[k]) for r in rs])) for k in ['progress_m','jerk_rms','decision_p95_ms','hard_brake_cycles']},
        'diagnostics':{k:sum(json.loads(r['candidate_diagnostics'].replace("'",'"')).get(k,0) for r in rs)
                       for k in ['candidates','prediction_rejected','guard_rejected','guard_only_rejected','comfort_override']}}
        for m,rs in groups.items()}
    suffix="recovery" if "ca_recovery" in groups else "smooth"
    contrasts=[]
    for left,right in [(f'ca_{suffix}','ca_adaptive'),(f'ml_{suffix}','ml_adaptive'),(f'ml_{suffix}',f'ca_{suffix}')]:
        key=lambda r:tuple(r[k] for k in ['scenario','seed','delay','delay_mode','profile','position_noise','velocity_noise','budget_s'])
        a={key(r):r for r in groups[left]};b={key(r):r for r in groups[right]}
        assert a.keys()==b.keys()
        seeds=defaultdict(list)
        for k in sorted(a):seeds[a[k]['seed']].append([float(a[k][v])-float(b[k][v]) for v in ['progress_m','jerk_rms']])
        d=np.array([np.mean(v,axis=0) for v in seeds.values()]);rng=np.random.default_rng(20261023)
        boot=d[rng.integers(len(d),size=(2000,len(d)))].mean(axis=1)
        contrasts.append({'left':left,'right':right,'seed_clusters':len(d),'mean_difference_progress_m_jerk':d.mean(axis=0).tolist(),
            'bootstrap95_ci':np.quantile(boot,[.025,.975],axis=0).T.tolist()})
    checks={m:{'collision_no_increase':summary[m]['collisions']<=summary[ref]['collisions'],
        'progress_loss_at_most_1pct':summary[m]['progress_m']>=.99*summary[ref]['progress_m'],
        'jerk_reduction_at_least_30pct':summary[m]['jerk_rms']<=.7*summary[ref]['jerk_rms']}
        for m,ref in [(f'ca_{suffix}','ca_adaptive'),(f'ml_{suffix}','ml_adaptive')]}
    return {'summary':summary,'contrasts':contrasts,'predeclared_acceptance':checks}

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);a=p.parse_args();root=Path(a.root)
    data={split:analyze(root/split/'episodes.csv') for split in ['development','main','stress'] if (root/split/'completion.json').exists()}
    dump_json(root/'results.json',data)
    lines=['# 控制修订验证结果','','若方法名含recovery，负加速度恢复时允许每周期增加至多2m/s²，其余阶段保留0.4m/s²偏好。候选加速度从6档细化到0.25 m/s²间隔；优先从满足全部原有检查的候选中选取每周期动作变化不超过0.4 m/s²的动作。必要时越过舒适限制执行更强制动。解析与学习方法都获得相同改进。',
      '独立测试前预设：碰撞不增加、平均进度损失不超过1%、平均回合jerk RMS至少降低30%。这些是本次工程验收标准，不是道路安全或乘坐舒适性的认证。','']
    for split,res in data.items():
        lines += [f'## {split}','','| 方法 | 回合 | 碰撞 | 进度m | 回合jerk RMS均值 | 回合P95耗时均值ms |','|---|---:|---:|---:|---:|---:|']
        for m,r in res['summary'].items():lines.append(f"| {m} | {r['episodes']} | {r['collisions']} | {r['progress_m']:.3f} | {r['jerk_rms']:.3f} | {r['decision_p95_ms']:.3f} |")
        lines += ['','| 配对差（左减右） | 独立种子块 | 进度差m及95%区间 | jerk差及95%区间 |','|---|---:|---|---|']
        for r in res['contrasts']:
            d=r['mean_difference_progress_m_jerk'];ci=r['bootstrap95_ci']
            lines.append(f"| {r['left']} − {r['right']} | {r['seed_clusters']} | {d[0]:.4f} [{ci[0][0]:.4f},{ci[0][1]:.4f}] | {d[1]:.4f} [{ci[1][0]:.4f},{ci[1][1]:.4f}] |")
        lines+=['','验收：'+json.dumps(res['predeclared_acceptance'],ensure_ascii=False),'']
    lines+=['## 结论边界','','bootstrap按同一种子下的所有条件归块，2000次重采样；区间是未作多重校正的探索估计。仅使用一个旧版冻结网络，未据旧测试重新训练。新测试只有10或5个种子块。',
      '守护与预测拒绝计数分别记录，可重叠，不可直接相加；guard_only_rejected表示仅被制动检查拒绝的候选。它描述判定机制，不是关闭安全检查后的因果收益。',
      '新版解析方案是同样细化与平滑后的强对照；不能把两组共有的控制优化收益称为AI独有收益。完整日志、模型哈希、配置和源码快照保留在本目录。','']
    (root/'report.md').write_text('\n'.join(lines));print(json.dumps(data,ensure_ascii=False))
if __name__=='__main__':main()

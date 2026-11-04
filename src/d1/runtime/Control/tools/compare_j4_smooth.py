#!/usr/bin/env python3
"""Compare completed main sweeps; reads logs only, never controls hardware."""
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folders=[Path(p) for p in sys.argv[1:]]
if len(folders)!=2:
    raise SystemExit('Usage: compare_j4_smooth.py baseline_dir new_dir')
fig,axes=plt.subplots(2,1,figsize=(12,8),sharex=True)
comparison=[]
for root in folders:
    summary=json.loads((root/'summary.json').read_text())
    analysis=json.loads((root/'analysis.json').read_text())
    metrics=analysis['phases']['sweep']
    events=[json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
    segment=next(e for e in events if e['kind']=='segment' and e['phase']=='sweep')
    duration=segment['duration_s']
    if segment['start_deg']!=-90 or segment['end_deg']!=90 or duration<=0:
        raise SystemExit('Expected a -90 to +90 degree sweep with positive duration')
    sweep_commands=[e for e in events if e['kind']=='command' and e['phase']=='sweep']
    settled=[]
    for e in events:
        if e['phase']=='sweep_settle' and e['kind']=='output' and '[rx] ' in e['line']:
            msg=json.loads(e['line'].split('[rx] ',1)[1])
            if msg.get('address')==2 and msg.get('funcode')==1: settled.append(e)
    if (len(sweep_commands)!=round(duration*summary['rate_hz'])+1 or sweep_commands[-1]['target_deg']!=90
            or len(settled)<20 or sweep_commands[-1]['t']-sweep_commands[0]['t']<duration-.1):
        raise SystemExit(f'{root}: main sweep not complete; no full-sweep comparison')
    comparison.append(dict(rate_hz=summary['rate_hz'],duration_s=duration,source=str(root),metrics=metrics,
                           entire_sequence_completed=summary['completed'],main_sweep_completed=True,
                           sequence_error=summary.get('error')))
    points=[]
    for e in events:
        if e['phase']!='sweep' or e['kind']!='output' or '[rx] ' not in e['line']: continue
        m=json.loads(e['line'].split('[rx] ',1)[1])
        if m.get('address')==2 and m.get('funcode')==1:
            points.append((e['received_t']-segment['t'],m['data']['angle4']))
    t,q=np.array(points).T
    u=np.clip(t/duration,0,1)
    reference=-90+180*(10*u**3-15*u**4+6*u**5)
    label=f"{summary['rate_hz']} Hz / {duration:g} s"
    axes[0].plot(t/duration*100,q-reference,label=label,linewidth=1)
    grid=np.arange(t[0]+.25,t[-1]-.25,.1)
    speed=(np.interp(grid+.25,t,q)-np.interp(grid-.25,t,q))/.5
    line=axes[1].plot(grid/duration*100,speed,label=label,linewidth=1)[0]
    u=np.linspace(0,1,1201)
    axes[1].plot(u*100,180/duration*30*u*u*(1-u)**2,'--',color=line.get_color(),label=f'Planned ({duration:g} s)')
axes[0].set_ylabel('Encoder - curve (deg)')
axes[1].set_ylabel('Estimated velocity (deg/s)')
axes[1].set_xlabel('Main sweep progress (%)')
for ax in axes:
    ax.grid(alpha=.25); ax.legend()
fig.suptitle('J4 -90 to +90 degrees: compensation OFF; normalized sweep progress\nSDK encoder / host arrival timing; velocity estimated over 0.5 s')
fig.tight_layout()
fig.savefig(folders[1]/'comparison.png',dpi=150)
(folders[1]/'comparison.json').write_text(json.dumps(dict(sweeps=comparison,note='Single trials; feedback irregularity cannot alone prove mechanical vibration.'),indent=2)+'\n')
for row in comparison:
    r=row['metrics']
    print(json.dumps(dict(rate_hz=row['rate_hz'],duration_s=row['duration_s'],plateaus=len(r['plateau_ge_0_5s_at_reference_speed_ge_1']),
        velocity_rmse=r['velocity_rmse_when_reference_speed_ge_1_deg_s'],peak_velocity=r['measured_speed_max_deg_s'],
        tracking_rmse=r['tracking_rmse_deg'],endpoint_error=r['endpoint_error_deg']),indent=2))

#!/usr/bin/env python3
"""Offline analysis of J6 smoothness logs; raw units, no physical calibration."""
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(sys.argv[1])
events=[json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
summary=json.loads((root/'summary.json').read_text()) if (root/'summary.json').exists() else {}
segments={e['phase']:e for e in events if e['kind']=='segment'}
commands=[e for e in events if e['kind']=='command']
feedback=[]; transmitted=[]; receipts=set()
for e in events:
    if e['kind']!='output': continue
    if '[tx] ' in e['line']:
        m=json.loads(e['line'].split('[tx] ',1)[1])
        if m.get('funcode')!=1 or m['data']['id']!=6 or not 0<=m['data']['angle']<=65:
            raise ValueError('Unexpected command outside J6 [0,65]')
        transmitted.append(m['seq'])
    if '[rx] ' not in e['line']: continue
    m=json.loads(e['line'].split('[rx] ',1)[1])
    if m.get('address')==2 and m.get('funcode')==1:
        feedback.append((e['received_t'],e['phase'],m['data']['angle6']))
    if m.get('address')==3 and m.get('funcode')==1 and m['data'].get('recv_status')==1:
        receipts.add(m['seq'])
report=dict(joint=6,units='SDK raw units, not calibrated millimeters',
    measurement='Internal feedback with host receipt timestamps, not external jaw or force measurement',
    completed=summary.get('completed',False),transmitted=len(transmitted),
    matched_receive_acknowledgements=len(set(transmitted)&receipts),phases={})
report['requested_range_sdk_units']=summary.get('requested_range_sdk_units')
fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True)
ct=np.array([e['t'] for e in commands]); cq=np.array([e['target_sdk_units'] for e in commands])
ft=np.array([r[0] for r in feedback]); fq=np.array([r[2] for r in feedback])
axes[0].plot(ct,cq,label='Command',linewidth=1.4)
axes[0].plot(ft,fq,label='Feedback',linewidth=.9)
for phase,segment in segments.items():
    rows=[r for r in feedback if r[1]==phase]
    if len(rows)<10: continue
    t=np.array([r[0] for r in rows]); q=np.array([r[2] for r in rows])
    duration=segment['duration_s']; start=segment['start_sdk_units']; end=segment['end_sdk_units']
    u=np.clip((t-segment['t'])/duration,0,1)
    reference=start+(end-start)*(10*u**3-15*u**4+6*u**5)
    error=q-reference
    velocity_ref=(end-start)/duration*30*u*u*(1-u)**2
    grid=np.arange(t[0]+.25,t[-1]-.25,.1)
    if not len(grid): continue
    speed=(np.interp(grid+.25,t,q)-np.interp(grid-.25,t,q))/.5
    gu=np.clip((grid-segment['t'])/duration,0,1)
    gv=(end-start)/duration*30*gu**2*(1-gu)**2
    active=np.abs(gv)>=1
    line=axes[1].plot(grid,speed,label=phase,linewidth=.9)[0]
    axes[1].plot(t,velocity_ref,'--',color=line.get_color(),alpha=.6)
    axes[2].plot(t,error,label=phase,linewidth=.9)
    plateaus=[]; first=0
    for i in range(1,len(q)+1):
        if i<len(q) and abs(q[i]-q[first])<.05 and abs(velocity_ref[i])>=1:continue
        if i-1>first and t[i-1]-t[first]>=.5 and abs(velocity_ref[first])>=1:
            plateaus.append(dict(t=float(t[first]),duration_s=float(t[i-1]-t[first]),value=float(q[first])))
        first=i
    pc=[e for e in commands if e['phase']==phase]
    rec=dict(planned_duration_s=duration,commands=len(pc),feedback_frames=len(t),
        phase_completed=phase in summary.get('completed_segments',[]),
        feedback_interval_median_s=float(np.median(np.diff(t))),feedback_interval_max_s=float(max(np.diff(t))),
        reference_peak_speed_units_s=float(max(abs(velocity_ref))),
        estimated_peak_speed_units_s=float(max(abs(speed))),
        tracking_error_min_units=float(min(error)),tracking_error_max_units=float(max(error)),
        tracking_rmse_units=float(np.sqrt(np.mean(error**2))),plateaus_ge_0_5s=plateaus)
    if np.any(active):rec['velocity_rmse_units_s']=float(np.sqrt(np.mean((speed[active]-gv[active])**2)))
    if len(pc)>1:
        gaps=np.diff([e['t'] for e in pc]);rec['command_interval_min_s']=float(min(gaps));rec['command_interval_max_s']=float(max(gaps))
    settle=[r[2] for r in feedback if r[1]==phase+'_settle']
    if len(settle)>=20:
        rec['endpoint_median_units']=float(np.median(settle[-20:]))
        rec['endpoint_error_units']=rec['endpoint_median_units']-end
    report['phases'][phase]=rec
if summary.get('error') and commands and feedback:
    stop_time=commands[-1]['t']; held_error=summary['final_angles_sdk_units'][6]-summary['last_commanded_target_sdk_units']
    report['stop_check']=dict(error=summary['error'],held_feedback_minus_target_units=held_error,
        feedback_age_s=stop_time-feedback[-1][0])
    axes[2].scatter([stop_time],[held_error],color='red',marker='x',label='Stop check (held sample)')
axes[0].set_ylabel('J6 position (SDK units)')
axes[1].set_ylabel('Velocity (SDK units/s)')
axes[2].set_ylabel('Feedback - curve (SDK units)')
axes[2].set_xlabel('Elapsed time (s)')
axes[2].axhline(.5,color='gray',linestyle=':'); axes[2].axhline(-.5,color='gray',linestyle=':')
for ax in axes:ax.grid(alpha=.25);ax.legend(loc='upper left',ncol=3,fontsize=8)
state='COMPLETE' if summary.get('completed') else 'INCOMPLETE'
fig.suptitle(f'{state}: D1-T J6 staged range check and full-stroke round trip\nSDK feedback; 0.5 s velocity window; physical jaw width and force uncalibrated')
fig.tight_layout();fig.savefig(root/'motion_plot.png',dpi=150)
(root/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))

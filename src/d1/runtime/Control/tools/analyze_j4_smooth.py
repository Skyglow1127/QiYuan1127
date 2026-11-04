#!/usr/bin/env python3
"""Analyze timestamped encoder feedback. Never controls hardware."""
import json
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(sys.argv[1])
events=[json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
segments={e['phase']:e for e in events if e['kind']=='segment'}
commands=[e for e in events if e['kind']=='command']
feedback=[]
for e in events:
    if e['kind']!='output' or '[rx] ' not in e['line']: continue
    m=json.loads(e['line'].split('[rx] ',1)[1])
    if m.get('address')==2 and m.get('funcode')==1:
        feedback.append((e['received_t'],e['phase'],m['data']['angle4']))
report=dict(measurement='SDK encoder, host receipt timestamps; no external angle or high-frequency vibration measurement',
            velocity_method='0.5-second centered difference on interpolated encoder samples',phases={})
fig,axes=plt.subplots(3,1,figsize=(12,10),sharex=True)
ct=np.array([e['t'] for e in commands]); cq=np.array([e['target_deg'] for e in commands])
ft=np.array([e[0] for e in feedback]); fq=np.array([e[2] for e in feedback])
rate_hz=next(iter(segments.values())).get('rate_hz',5)
report['rate_hz']=rate_hz
axes[0].plot(ct,cq,label=f'Command ({rate_hz} Hz)',linewidth=1.5)
axes[0].plot(ft,fq,label='Encoder',linewidth=.9)
for phase,segment in segments.items():
    rows=[e for e in feedback if e[1]==phase]
    if len(rows)<10: continue
    t=np.array([e[0] for e in rows]); q=np.array([e[2] for e in rows])
    u=np.clip((t-segment['t'])/segment['duration_s'],0,1)
    start,end=segment['start_deg'],segment['end_deg']
    reference=start+(end-start)*(10*u**3-15*u**4+6*u**5)
    error=q-reference
    grid=np.arange(t[0]+.25,t[-1]-.25,.1)
    speed=(np.interp(grid+.25,t,q)-np.interp(grid-.25,t,q))/.5
    velocity_ref=(end-start)/segment['duration_s']*30*u*u*(1-u)**2
    grid_u=np.clip((grid-segment['t'])/segment['duration_s'],0,1)
    grid_velocity_ref=(end-start)/segment['duration_s']*30*grid_u**2*(1-grid_u)**2
    moving=np.abs(grid_velocity_ref)>=1
    axes[1].plot(grid,speed,label=phase+' encoder')
    axes[1].plot(t,velocity_ref,linestyle='--',color='black',alpha=.6)
    axes[2].plot(t,error,label=phase)
    # Plateaus only when planned speed >= 1 deg/s, excluding slow endpoints.
    plateaus=[]
    first=0
    for i in range(1,len(q)+1):
        if i<len(q) and abs(q[i]-q[first])<.05 and abs(velocity_ref[i])>=1: continue
        if i-1>first and t[i-1]-t[first]>=.5 and abs(velocity_ref[first])>=1:
            plateaus.append(dict(start_s=float(t[first]),duration_s=float(t[i-1]-t[first]),angle_deg=float(q[first])))
        first=i
    settle=[e[2] for e in feedback if e[1]==phase+'_settle']
    record=dict(planned_duration_s=segment['duration_s'],observed_feedback_span_s=float(t[-1]-t[0]),
                feedback_frames=len(t),feedback_interval_median_s=float(np.median(np.diff(t))),
                feedback_interval_max_s=float(np.max(np.diff(t))),reference_speed_max_deg_s=float(max(abs(velocity_ref))),
                measured_speed_max_deg_s=float(max(abs(speed))),tracking_error_min_deg=float(min(error)),
                tracking_error_max_deg=float(max(error)),tracking_rmse_deg=float(np.sqrt(np.mean(error**2))),
                plateau_ge_0_5s_at_reference_speed_ge_1=plateaus)
    if np.any(moving):
        record['velocity_rmse_when_reference_speed_ge_1_deg_s']=float(np.sqrt(np.mean((speed[moving]-grid_velocity_ref[moving])**2)))
    command_times=[e['t'] for e in commands if e['phase']==phase]
    record['commands_sent']=len(command_times)
    if len(command_times)>1:
        intervals=np.diff(command_times)
        record['command_interval_median_s']=float(np.median(intervals))
        record['command_interval_min_s']=float(min(intervals))
        record['command_interval_max_s']=float(max(intervals))
    if settle:
        record['endpoint_median_deg']=float(np.median(settle[-20:]))
        record['endpoint_error_deg']=record['endpoint_median_deg']-end
    report['phases'][phase]=record
axes[0].set_ylabel('J4 angle (deg)')
axes[1].set_ylabel('Speed (deg/s)')
axes[2].set_ylabel('Encoder - curve (deg)')
axes[2].set_xlabel('Elapsed time (s)')
axes[2].axhline(.3,color='gray',linestyle=':'); axes[2].axhline(-.3,color='gray',linestyle=':')
summary_path=root/'summary.json'
summary=json.loads(summary_path.read_text()) if summary_path.exists() else {}
if summary.get('error')=='J4 tracking error > 3 degrees' and commands and feedback:
    error=summary['final_angles'][4]-summary['last_commanded_target_deg']
    stop_time=commands[-1]['t']
    report['stop_check']=dict(latest_feedback_minus_last_command_deg=error,
        latest_feedback_age_at_last_command_s=stop_time-feedback[-1][0],
        note='Uses latest held feedback versus last issued target, unlike timestamp-aligned curve errors.')
    axes[2].scatter([stop_time],[error],color='red',marker='x',s=60,label='Stop check: latest held sample')
for ax in axes:
    ax.grid(alpha=.25); ax.legend(loc='upper left',ncol=3)
prefix='INCOMPLETE — ' if summary and not summary.get('completed') else ''
fig.suptitle(prefix+'D1-T J4 quintic sweep: compensation OFF; host-timed encoder measurements')
fig.tight_layout()
fig.savefig(root/'motion_plot.png',dpi=160)
(root/'analysis.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))

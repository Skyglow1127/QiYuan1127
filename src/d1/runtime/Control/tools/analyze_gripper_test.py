#!/usr/bin/env python3
"""Analyze J6 step-test logs offline. SDK units are not calibrated jaw width."""
import json
from pathlib import Path
import statistics
import sys

root=Path(sys.argv[1])
def messages(path,tag='[rx] '):
    return [json.loads(line.split(tag,1)[1]) for line in path.read_text().splitlines() if tag in line]
def angles(path):
    return [[m['data'][f'angle{i}'] for i in range(7)] for m in messages(path)
            if m.get('address')==2 and m.get('funcode')==1]
baseline_samples=angles(root/'preflight.log')
baseline=[statistics.median(a[i] for a in baseline_samples[-20:]) for i in range(7)]
rows=[]
for command in sorted(root.glob('*_command.log')):
    after=command.with_name(command.name.replace('_command.log','_after.log'))
    if not after.exists():continue
    sent=messages(command,'[tx] ')
    if len(sent)!=1 or sent[0]['funcode']!=1 or sent[0]['data']['id']!=6:
        raise ValueError(f'Unexpected command: {command}')
    samples=angles(after)
    if len(samples)<20:raise ValueError(f'Need >=20 settled frames: {after}')
    target=sent[0]['data']['angle']
    tail=[a[6] for a in samples[-20:]]
    actual=statistics.median(tail)
    all_samples=angles(command)+samples
    drift=max(abs(a[i]-baseline[i]) for a in all_samples for i in range(6))
    statuses=[m['data'] for m in messages(after) if m.get('address')==2 and m.get('funcode')==3]
    seq=sent[0]['seq']
    receipt=any(m.get('address')==3 and m.get('seq')==seq and m.get('funcode')==1 and m.get('data',{}).get('recv_status')==1 for m in messages(command))
    row=dict(command_log=command.name,after_log=after.name,target_sdk_units=target,
        start_sdk_units=angles(command)[0][6],actual_median_sdk_units=actual,
        signed_error_sdk_units=actual-target,settled_spread_sdk_units=max(tail)-min(tail),
        settled_frames=len(samples),non_target_max_drift_deg=drift,
        receive_ack=receipt,cli_position_pass='位置闭环通过' in command.read_text(),
        settled_within_0_5=abs(actual-target)<=.50001,
        status_ok=bool(statuses) and all(s==dict(enable_status=1,power_status=1,error_status=0) for s in statuses))
    row['continue_ok']=(row['cli_position_pass'] and row['settled_within_0_5'] and
        row['status_ok'] and drift<=.5 and row['settled_spread_sdk_units']<=.20001)
    rows.append(row)
camera_log=root/'camera_before.log'
camera_available=False if camera_log.exists() and 'No Orbbec device found' in camera_log.read_text() else None
report=dict(joint=6,units='SDK raw units, not independently calibrated mm',baseline_sdk_units=baseline,
    camera_available=camera_available,physical_opening_validated=False,force_validated=False,
    position_tolerance_sdk_units=.5,rows=rows)
if rows:
    report['summary']=dict(trials=len(rows),position_passes=sum(r['cli_position_pass'] and r['settled_within_0_5'] for r in rows),
        max_absolute_error_sdk_units=max(abs(r['signed_error_sdk_units']) for r in rows),
        non_target_max_drift_deg=max(r['non_target_max_drift_deg'] for r in rows),
        final_median_sdk_units=[statistics.median(a[i] for a in angles(root/rows[-1]['after_log'])[-20:]) for i in range(7)])
(root/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(rows[-1] if rows else report,indent=2))

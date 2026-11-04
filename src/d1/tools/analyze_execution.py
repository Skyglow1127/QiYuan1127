#!/usr/bin/env python3
"""Read-only host-timing analysis. No SDK, ROS, network or hardware imports.

Usage: python3 tools/analyze_execution.py --stage-dir /path/to/executed \
    --output /path/to/a/NEW/report-directory

The only interpolation of measured data is between neighbouring valid samples
within --max-feedback-gap-s. Host receipt is not encoder sampling or motor
application time. A rejected joint's retained merged value is never a new sample.
"""
import argparse
import bisect
import csv
import hashlib
import html
import json
import math
from pathlib import Path
import statistics


def number(x):
    return type(x) in (int, float) and math.isfinite(x)


def percentile(values, p):
    if not values:
        return None
    a = sorted(values)
    x = (len(a)-1)*p
    lo = int(x)
    return a[lo] + (a[min(lo+1, len(a)-1)]-a[lo])*(x-lo)


def summary(values):
    values = [x for x in values if number(x)]
    return dict(count=len(values), minimum=min(values) if values else None,
                median=statistics.median(values) if values else None,
                p95=percentile(values, .95), maximum=max(values) if values else None)


class Reference:
    """Evaluate only trajectory shapes explicitly stored in the saved SDK plan."""
    def __init__(self, plan):
        self.plan = plan.get('sdk_plan', plan)
        self.reason = None
        p = self.plan
        self.duration = p.get('duration_s')
        profile = p.get('timing_profile', 'edge_quintic_v1')
        if profile == 'collinear_bounded_v1' and p.get('runs'):
            self.mode = 'runs'
        elif profile == 'edge_quintic_v1' and p.get('sdk') and p.get('ticks') and number(p.get('rate')):
            self.mode = 'edges'
        else:
            self.mode = None
            self.reason = 'Saved plan lacks a supported complete reference representation; no endpoint-derived trajectory was created.'
        if not number(self.duration) or self.duration <= 0:
            self.mode = None
            self.reason = 'Saved reference duration is missing or invalid.'

    def at(self, t):
        if self.mode is None or t < -1e-8 or t > self.duration+1e-8:
            return None
        t = min(max(t, 0.), self.duration)
        p = self.plan
        if self.mode == 'edges':
            elapsed = 0.
            for a, b, ticks in zip(p['sdk'][:-1], p['sdk'][1:], p['ticks']):
                duration = ticks/p['rate']
                if t <= elapsed+duration+1e-9:
                    u = min(max((t-elapsed)/duration, 0.), 1.)
                    s = 10*u**3-15*u**4+6*u**5
                    return [aa+(bb-aa)*s for aa, bb in zip(a, b)]
                elapsed += duration
            return None
        for run in p['runs']:
            start = run.get('start_time_s')
            if not number(start) or not number(run.get('duration_s')):
                return None
            if start-1e-8 <= t <= start+run['duration_s']+1e-8:
                dt = min(max(t-start, 0.), run['duration_s'])
                d = run['distance']
                if d == 0:
                    return list(run['start'])
                if run['shape'] == 'quintic':
                    u = dt/run['duration_s']
                    s = d*(10*u**3-15*u**4+6*u**5)
                elif run['shape'] == 'cosine_cruise':
                    v, ramp, cruise, stretch = [run[k] for k in ('peak_speed','ramp_s','cruise_s','stretch')]
                    x = min(max(dt/stretch, 0.), 2*ramp+cruise)
                    if x >= 2*ramp+cruise:
                        s = d
                    elif x < ramp or x > ramp+cruise:
                        decel = x > ramp+cruise
                        r = 2*ramp+cruise-x if decel else x
                        s = .5*v*(r-ramp/math.pi*math.sin(math.pi*r/ramp))
                        if decel:
                            s = d-s
                    else:
                        s = .5*v*ramp+v*(x-ramp)
                elif run['shape'] == 'hold':
                    s = 0.
                else:
                    return None
                return [a+(b-a)*s/d for a,b in zip(run['start'],run['end'])]
        return None


def linear_abs_cdf(a, b, bound):
    """Fraction of an affine error segment whose absolute value <= bound."""
    if abs(a-b) < 1e-14:
        return 1. if abs(a) <= bound else 0.
    lo, hi = sorted((a,b))
    return max(0., min(hi,bound)-max(lo,-bound))/(hi-lo)


def weighted_error(samples, reference, epoch, joint, gap_limit):
    if reference.mode is None or epoch is None or joint >= 6:
        return None, []
    rows = []
    for s in samples:
        elapsed = s['received_monotonic_s']-epoch
        q = reference.at(elapsed)
        if q is not None and joint < len(q):
            rows.append(dict(**s, original_elapsed_s=elapsed,
                             reference_deg=q[joint], error_deg=s['angle']-q[joint]))
    # Intervals spanning missing data are excluded, not filled with held values.
    segments = []
    for a,b in zip(rows,rows[1:]):
        dt = b['received_monotonic_s']-a['received_monotonic_s']
        if 0 < dt <= gap_limit+1e-9:
            segments.append((dt,a['error_deg'],b['error_deg']))
    duration = sum(s[0] for s in segments)
    if not segments:
        return dict(samples=len(rows), covered_s=0., rmse_deg=None,
                    absolute_error_p95_deg=None, maximum_observed_absolute_error_deg=
                    max((abs(r['error_deg']) for r in rows), default=None)), rows
    integral = sum(dt*(a*a+a*b+b*b)/3 for dt,a,b in segments)
    upper = max(max(abs(a),abs(b)) for _,a,b in segments)
    lo,hi = 0.,upper
    for _ in range(60):
        mid = (lo+hi)/2
        cdf = sum(dt*linear_abs_cdf(a,b,mid) for dt,a,b in segments)/duration
        if cdf < .95:
            lo = mid
        else:
            hi = mid
    return dict(samples=len(rows), covered_s=duration,
                coverage_of_original_duration=duration/reference.duration,
                rmse_deg=math.sqrt(integral/duration), absolute_error_p95_deg=hi,
                maximum_observed_absolute_error_deg=max(abs(r['error_deg']) for r in rows),
                integration='piecewise-linear error between valid host-received samples; long gaps excluded'), rows


def parse_transport(rows):
    commands, tx, feedback = [], {}, [[] for _ in range(7)]
    rejected = [0]*7
    ages = [[] for _ in range(7)]
    warnings = []
    filter_enabled = None
    raw_feedback = []
    for source_line,r in enumerate(rows,1):
        t = r.get('t')
        if r.get('kind') == 'feedback_filter':
            filter_enabled = r.get('enabled')
        if r.get('kind') == 'command' and number(t) and isinstance(r.get('joint'),int):
            commands.append(dict(source_line=source_line, joint=r['joint'],
                submitted_monotonic_s=t,target_sdk_units=r.get('target_sdk_units'),
                reference_id=r.get('reference_id'),seq=None,tx_monotonic_s=None,
                ack_monotonic_s=None,ack_elapsed_s=None,ack_status=None))
        line = r.get('line','')
        if line.startswith(('[feedback-accepted] ','[feedback-rejected] ')):
            event = json.loads(line.split(' ',1)[1])
            stamp = event.get('received_monotonic_s')
            rejected_joints = event.get('rejected_joints')
            if rejected_joints is None:
                rejected_joints = [event['joint']] if line.startswith('[feedback-rejected] ') and isinstance(event.get('joint'),int) else []
            for j in set(rejected_joints):
                if 0 <= j < 7:
                    rejected[j] += 1
            q, jt = event.get('merged_angles'), event.get('joint_times_s')
            if not isinstance(q,list) or not number(stamp):
                continue
            accepted = event.get('accepted_joints')
            if accepted is None:
                # A merged vector without per-joint provenance is not a frame.
                if jt is None:
                    warnings.append('Ignored filtered merged vector without per-joint receipt provenance.')
                    continue
                accepted = [j for j in range(min(7,len(q))) if j not in rejected_joints]
            for j in accepted:
                if not isinstance(j,int) or not 0 <= j < min(7,len(q)) or j in rejected_joints:
                    continue
                received = jt[j] if isinstance(jt,list) and j < len(jt) else stamp
                if number(received) and number(q[j]) and received <= stamp+.01:
                    feedback[j].append(dict(source_line=source_line,joint=j,
                        received_monotonic_s=received,event_monotonic_s=t,
                        angle=q[j],basis='per_joint_host_receipt'))
                    if number(t) and t >= received:
                        ages[j].append(t-received)
            continue
        if line.startswith(('[tx] ','[rx] ')) and number(t):
            direction,payload = line.split(' ',1)
            m = json.loads(payload)
            data,seq = m.get('data',{}),m.get('seq')
            if direction == '[tx]' and m.get('address') == 1 and m.get('funcode') == 1:
                j = data.get('id')
                matching = [c for c in commands if c['seq'] is None and c['joint']==j and
                            c['submitted_monotonic_s'] <= t and number(c['target_sdk_units']) and
                            number(data.get('angle')) and abs(c['target_sdk_units']-data['angle']) < 1e-6]
                if matching:
                    c = matching[-1]
                    c.update(seq=seq,tx_monotonic_s=t)
                    tx[seq] = c
            if direction == '[rx]' and m.get('address') == 3 and m.get('funcode') == 1 and seq in tx:
                c=tx.pop(seq)
                if t >= c['tx_monotonic_s']:
                    c.update(ack_monotonic_s=t,ack_elapsed_s=t-c['tx_monotonic_s'],
                             ack_status=data.get('recv_status'))
            if direction == '[rx]' and m.get('address') == 2 and m.get('funcode') == 1:
                raw_feedback.append((t,data,source_line))
    if filter_enabled is False and not any(feedback):
        for t,data,source_line in raw_feedback:
            for j in range(7):
                if number(data.get(f'angle{j}')):
                    feedback[j].append(dict(source_line=source_line,joint=j,
                        received_monotonic_s=t,event_monotonic_s=t,angle=data[f'angle{j}'],
                        basis='unfiltered_host_log_receipt'))
    for j in range(7):
        # First observed sample at a joint's receipt timestamp wins; retained
        # merged samples neither refresh age nor add sample count.
        feedback[j] = sorted({s['received_monotonic_s']:s for s in reversed(feedback[j])}.values(),
                             key=lambda s:s['received_monotonic_s'])
    return commands,feedback,rejected,ages,sorted(set(warnings))


def static_residual(reference, groups, commands, feedback, gap_limit):
    """Feedback stability after all final active-axis targets were submitted.

    This certifies host submission/receipt and logged feedback only. It cannot
    identify a hardware defect or certify physical stillness.
    """
    reason='Final reference group and all final active-axis targets were not demonstrably submitted and positively acknowledged.'
    if reference.mode is None or not groups:
        return None,reason
    final=reference.at(reference.duration)
    active=reference.plan.get('active_joints')
    if not isinstance(active,list):
        return None,'Saved active-joint list missing; no final-target completeness inferred.'
    active=[j for j in active if isinstance(j,int) and 0<=j<6]
    final_groups=[g for g in groups if g['completed'] and number(g['call_completed_monotonic_s']) and
                  g['source_elapsed_s'] >= reference.duration-1e-7]
    if not active or not final_groups:
        return None,reason
    state={};complete_at=None
    for c in sorted(commands,key=lambda c:c['submitted_monotonic_s']):
        if c['joint'] not in active:
            continue
        state[c['joint']]=c
        full=all(j in state and number(state[j]['target_sdk_units']) and
                 abs(state[j]['target_sdk_units']-final[j])<=.00051 and
                 state[j]['ack_status']==1 and number(state[j]['ack_monotonic_s']) for j in active)
        if not full:
            complete_at=None
        elif complete_at is None:
            complete_at=max(state[j]['ack_monotonic_s'] for j in active)
    if complete_at is None:
        return None,reason
    complete_at=max(complete_at,final_groups[0]['call_completed_monotonic_s'])
    ends=[feedback[j][-1]['received_monotonic_s'] for j in active if feedback[j]]
    if len(ends)!=len(active):
        return None,'Final active-axis feedback is missing.'
    end=min(ends)
    start=max(complete_at+.4,end-2.)
    if end-start<1.:
        return None,'Less than 1 s of feedback remains after the final group plus 0.4 s settling exclusion.'
    axes=[]
    for j in active:
        ss=[s for s in feedback[j] if start<=s['received_monotonic_s']<=end]
        if len(ss)<3 or ss[-1]['received_monotonic_s']-ss[0]['received_monotonic_s']<1.:
            return None,'At least one final active axis lacks 1 s and 3 accepted samples in the stability window.'
        intervals=[b['received_monotonic_s']-a['received_monotonic_s'] for a,b in zip(ss,ss[1:])]
        if max(intervals)>gap_limit+1e-9:
            return None,'Accepted feedback has a gap exceeding the analysis bound in the stability window.'
        angles=[s['angle'] for s in ss];ptp=max(angles)-min(angles)
        if ptp>.20001:
            return None,'Accepted feedback is not stable within the declared 0.2 degree peak-to-peak analysis threshold.'
        axes.append(dict(joint=f'J{j}',target_deg=final[j],median_received_deg=statistics.median(angles),
            median_residual_deg=statistics.median(angles)-final[j],peak_to_peak_deg=ptp,
            samples=len(ss),sample_span_s=ss[-1]['received_monotonic_s']-ss[0]['received_monotonic_s']))
    return dict(basis='host final-target submission and positive receipt ACK; stable accepted feedback, not motor application or physical stillness',
        final_target_submission_complete_monotonic_s=complete_at,window_s=[start,end],
        settle_exclusion_s=.4,required_observed_span_s=1.,peak_to_peak_threshold_deg=.2,
        axes=axes,hardware_defect_proven=False,
        implication='Continued upper-layer timing changes alone are not evidence that this persistent residual will be corrected.'),None


def analyze(stage_dir, gap_limit=.3):
    stage_dir = Path(stage_dir).resolve()
    paths = {name:stage_dir/name for name in ('plan.json','result.json','transport.jsonl')}
    for p in paths.values():
        if not p.is_file():
            raise ValueError(f'Missing required input: {p}')
    contents={name:p.read_bytes() for name,p in paths.items()}
    inputs = {name:dict(path=str(p),sha256=hashlib.sha256(contents[name]).hexdigest(),bytes=len(contents[name]))
              for name,p in paths.items()}
    plan,result = [json.loads(contents[n]) for n in ('plan.json','result.json')]
    rows = [json.loads(line) for line in contents['transport.jsonl'].splitlines() if line.strip()]
    commands,feedback,rejected,ages,warnings = parse_transport(rows)
    command_ages=[[] for _ in range(7)]
    for c in commands:
        j=c['joint']
        if not 0 <= j < 7:
            continue
        stamps=[s['received_monotonic_s'] for s in feedback[j]]
        index=bisect.bisect_right(stamps,c['submitted_monotonic_s'])-1
        # Do not use a receipt whose acceptance event became visible only
        # after this command. Offline access must not create future knowledge.
        while index>=0 and (not number(feedback[j][index].get('event_monotonic_s')) or
                            feedback[j][index]['event_monotonic_s']>c['submitted_monotonic_s']):
            index-=1
        age=c['submitted_monotonic_s']-stamps[index] if index>=0 else None
        c['same_joint_feedback_receipt_age_s']=age
        if age is not None:
            command_ages[j].append(age)
    reference = Reference(plan)
    groups=[]
    for index,g in enumerate(result.get('command_group_schedule',[])):
        phase,start,end = [g.get(k) for k in ('source_elapsed_s','group_started_monotonic_s','group_completed_monotonic_s')]
        if not number(phase) or not number(start):
            warnings.append('A group without source phase/call time was excluded from timing metrics.')
            continue
        groups.append(dict(index=index,reference_id=g.get('reference_id'),source_elapsed_s=phase,
            call_started_monotonic_s=start,call_completed_monotonic_s=end,
            completed=g.get('completed') is True,commands=g.get('commands'),
            basis=g.get('group_start_basis','transport_send_call')))
    epoch = result.get('reference_epoch_monotonic_s')
    if not number(epoch):
        epoch = next((r.get('original_reference_epoch_monotonic_s') for r in rows
                      if number(r.get('original_reference_epoch_monotonic_s'))),None)
    epoch_basis = 'explicit_original_reference_epoch'
    if not number(epoch):
        epoch = groups[0]['call_started_monotonic_s']-groups[0]['source_elapsed_s'] if groups else None
        epoch_basis = 'legacy_inferred_from_first_group_call_and_phase' if groups else 'unavailable'
        if groups:
            warnings.append('Legacy reference epoch is inferred from the first send-call start minus source phase; initial dispatch latency is unknown.')
    for i,g in enumerate(groups):
        first=groups[0]
        g['relative_cumulative_drift_s'] = ((g['call_started_monotonic_s']-first['call_started_monotonic_s'])-
                                           (g['source_elapsed_s']-first['source_elapsed_s']))
        g['local_stretch'] = g['period_error_s'] = g['actual_interval_s'] = g['reference_interval_s'] = None
        if i:
            prev=groups[i-1]
            actual=g['call_started_monotonic_s']-prev['call_started_monotonic_s']
            source=g['source_elapsed_s']-prev['source_elapsed_s']
            g.update(actual_interval_s=actual,reference_interval_s=source,period_error_s=actual-source,
                     local_stretch=actual/source if source > 1e-10 else None)
        end=g['call_completed_monotonic_s']
        submitted=[c for c in commands if number(end) and 'group_index' not in c and
                   g['call_started_monotonic_s'] <= c['submitted_monotonic_s'] < end and
                   (g['reference_id'] is None or c['reference_id'] is None or g['reference_id']==c['reference_id'])]
        # No synthetic command for skipped/unchanged axes.
        g['submitted_active_joints']=sorted(set(c['joint'] for c in submitted))
        g['submitted_count']=len(submitted)
        g['submission_span_s'] = max(c['submitted_monotonic_s'] for c in submitted)-min(c['submitted_monotonic_s'] for c in submitted) if submitted else None
        g['call_duration_s']=end-g['call_started_monotonic_s'] if number(end) else None
        for c in submitted:
            c['group_index']=g['index']
    group_metrics=dict(group_count=len(groups),
        reference_phase_span_s=groups[-1]['source_elapsed_s']-groups[0]['source_elapsed_s'] if groups else None,
        actual_call_start_span_s=groups[-1]['call_started_monotonic_s']-groups[0]['call_started_monotonic_s'] if groups else None,
        relative_cumulative_drift_s=groups[-1]['relative_cumulative_drift_s'] if groups else None,
        period_error_s=summary([g['period_error_s'] for g in groups]),
        local_stretch=summary([g['local_stretch'] for g in groups]),
        submitted_joint_span_s=summary([g['submission_span_s'] for g in groups]),
        six_axis_submission_span_s=summary([g['submission_span_s'] for g in groups if g['submitted_active_joints']==list(range(6))]),
        call_duration_s=summary([g['call_duration_s'] for g in groups]))
    window_start=epoch
    window_end=epoch+reference.duration if epoch is not None and number(reference.duration) else None
    joints=[];error_rows=[]
    for j,samples in enumerate(feedback):
        times=[s['received_monotonic_s'] for s in samples]
        intervals=[b-a for a,b in zip(times,times[1:])]
        valid_intervals=[dt for dt in intervals if dt <= gap_limit+1e-9]
        observed_span=times[-1]-times[0] if len(times)>1 else None
        err,errors=weighted_error(samples,reference,epoch,j,gap_limit)
        error_rows.extend(errors)
        joints.append(dict(joint=f'J{j}',units='degree' if j<6 else 'uncalibrated_SDK_units',
            accepted_unique_samples=len(samples),rejected_events=rejected[j],
            interval_s=summary(intervals),event_receipt_age_s=summary(ages[j]),
            receipt_age_at_same_joint_submission_s=summary(command_ages[j]),
            observed_span_s=observed_span,
            effective_logged_hz=(len(times)-1)/observed_span if observed_span else None,
            typical_logged_hz=1/statistics.median(intervals) if intervals else None,
            bounded_interval_covered_s=sum(valid_intervals),
            bounded_coverage_of_observed_span=sum(valid_intervals)/observed_span if observed_span else None,
            original_timeline_tracking=err))
    raw_times=[r['t'] for r in rows if number(r.get('t'))]
    residual,residual_reason=static_residual(reference,groups,commands,feedback,gap_limit)
    unresolved=result.get('error_code') in ('Endpoint convergence stalled','Accepted commands but no measured joint progress')
    metrics=dict(schema='d1-host-execution-metrics-v1',stage_dir=str(stage_dir),inputs=inputs,
        analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        completed=result.get('completed') is True,trajectory_completed=result.get('trajectory_completed'),
        source_error=result.get('error'),reference_epoch_monotonic_s=epoch,reference_epoch_basis=epoch_basis,
        original_reference_duration_s=reference.duration,reference_representation=reference.mode,
        reference_unavailable_reason=reference.reason,max_feedback_gap_s=gap_limit,
        observation_window_s=[min(raw_times),max(raw_times)] if raw_times else None,
        original_reference_window_s=[window_start,window_end],group_timing=group_metrics,
        command_count=len(commands),ack_pair_count=sum(c['ack_elapsed_s'] is not None for c in commands),
        ack_elapsed_s=summary([c['ack_elapsed_s'] for c in commands]),joints=joints,
        lower_layer_execution_unresolved=unresolved,hardware_defect_proven=False,
        static_residual_evidence=residual,static_residual_unavailable_reason=residual_reason,
        lower_layer_assessment='Existing receipt ACK plus nonprogress/static residual requires downstream execution verification. Continued upper-layer timing changes alone cannot be assumed to repair it.' if unresolved else None,
        recorded_wait_costs=dict(feedback_pacing_wait_s=(result.get('execution_diagnostics') or {}).get('feedback_pacing_wait_s'),
            feedback_gap_wait_s=result.get('feedback_gap_wait_s'),
            scheduling_overrun_s=result.get('scheduling_overrun_s'),
            command_wait_pause_s=result.get('command_wait_pause_s'),
            note='Recorded categories can overlap; do not sum them as total duration.'),
        warnings=sorted(set(warnings)),
        semantics=[
            'All times use host monotonic receipt/submission. Motor application and encoder sampling times, and their delays, are unknown.',
            'Group start is entry to transport.send, possibly before internal waits; group skew measures only commands actually submitted.',
            'ACK is paired by DDS sequence to the transmitted single-joint command. It acknowledges receipt, not movement completion.',
            'Filtered merged joint values retain per-joint receipt provenance. Rejected/retained joints are not new simultaneous samples.',
            'Time-weighted tracking errors compare host-received joint position to the original reference time, with no fitted time shift or post-abort endpoint extension.',
            'No motor synchronization, calibrated physical vibration, jerk, causal hardware diagnosis, or completed motion is inferred.',
            'Coverage excludes adjacent accepted-sample intervals longer than max_feedback_gap_s; this is an analysis parameter, not a vendor limit.'
        ])
    return metrics,dict(groups=groups,commands=commands,feedback=[s for ss in feedback for s in ss],tracking=error_rows)


def write_csv(path,rows):
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('x',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=fields or ['no_data'])
        writer.writeheader()
        for row in rows:
            writer.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(list,dict)) else v for k,v in row.items()})


def svg_plot(series,xkey,ykey,title):
    points=[(r[xkey],r[ykey]) for r in series if number(r.get(xkey)) and number(r.get(ykey))]
    if not points:
        return '<p>'+html.escape(title)+': 无可用数据</p>'
    xmin,xmax=min(p[0] for p in points),max(p[0] for p in points)
    ymin,ymax=min(p[1] for p in points),max(p[1] for p in points)
    if xmax==xmin:xmax=xmin+1
    if ymax==ymin:ymax=ymin+1
    coords=' '.join(f'{55+660*(x-xmin)/(xmax-xmin):.2f},{220-170*(y-ymin)/(ymax-ymin):.2f}' for x,y in points)
    return f'<figure><figcaption>{html.escape(title)}</figcaption><svg viewBox="0 0 760 260" role="img"><path d="M55 40V220H720" fill="none" stroke="#888"/><polyline points="{coords}" fill="none" stroke="#1864ab" stroke-width="1.8"/><text x="55" y="245">x: {xmin:.3f} — {xmax:.3f}</text><text x="55" y="25">y: {ymin:.4f} — {ymax:.4f}</text></svg></figure>'


def write_report(output,metrics,tables):
    output=Path(output).resolve()
    stage=Path(metrics['stage_dir'])
    if output==stage or stage in output.parents:
        raise ValueError('Output must not be inside the original evidence stage directory.')
    output.mkdir(parents=True,exist_ok=False)
    (output/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    for name,rows in tables.items():write_csv(output/(name+'.csv'),rows)
    rows=''.join('<tr><td>'+html.escape(j['joint'])+'</td><td>'+str(j['accepted_unique_samples'])+'</td><td>'+str(j['rejected_events'])+'</td><td>'+html.escape(str(j['interval_s']['median']))+'</td><td>'+html.escape(str(j['original_timeline_tracking']))+'</td></tr>' for j in metrics['joints'])
    plots=''.join(svg_plot(tables['groups'],'source_elapsed_s',key,title) for key,title in [
        ('relative_cumulative_drift_s','累计提交调用漂移（x 原参考秒，y 秒）'),
        ('period_error_s','相邻调用周期偏差（x 原参考秒，y 秒）'),
        ('submission_span_s','同组已提交关节跨度（x 原参考秒，y 秒）')])
    body=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>D1 主机执行时序量化</title>
<style>body{{font:16px/1.65 system-ui;max-width:1100px;margin:32px auto;padding:0 24px;color:#172b4d}}table{{border-collapse:collapse;font-size:13px}}td,th{{border:1px solid #ccd;padding:8px;vertical-align:top;overflow-wrap:anywhere}}pre{{white-space:pre-wrap;background:#f4f6f8;padding:16px}}svg{{width:100%;max-height:300px}}figure{{margin:24px 0}}a{{color:#1864ab}}</style>
<h1>D1 主机执行时序量化</h1><p>运动完成：{metrics['completed']}。分析使用已有证据，未连接设备。</p>
<p>原始证据：{html.escape(metrics['stage_dir'])}</p><p>参考时钟：{html.escape(metrics['reference_epoch_basis'])}；反馈最大插值间隔：{metrics['max_feedback_gap_s']} s。</p>
<p><a href="metrics.json">完整指标与输入 SHA256</a> · <a href="groups.csv">调用组 CSV</a> · <a href="commands.csv">指令与 ACK CSV</a> · <a href="feedback.csv">逐轴有效反馈 CSV</a> · <a href="tracking.csv">原时间轴误差 CSV</a></p>
<h2>时间关系</h2><pre>{html.escape(json.dumps(metrics['group_timing'],ensure_ascii=False,indent=2))}</pre>{plots}
<h2>反馈与原时间轴误差</h2><table><tr><th>轴</th><th>有效样本</th><th>拒绝次数</th><th>中位间隔 s</th><th>时间加权跟踪指标</th></tr>{rows}</table>
<h2>下游执行与稳定残差</h2><p>下游执行仍待确认：{metrics['lower_layer_execution_unresolved']}；已证明硬件缺陷：{metrics['hardware_defect_proven']}。</p><p>继续改上层时序，不能证明能够修复当前静态残差或无进展。需要核实下游指令采用、位置执行与反馈。</p><pre>{html.escape(json.dumps(metrics['static_residual_evidence'],ensure_ascii=False,indent=2))}</pre><p>{html.escape(str(metrics['static_residual_unavailable_reason']))}</p>
<h2>指标语义与限制</h2><ul>{''.join('<li>'+html.escape(s)+'</li>' for s in metrics['semantics'])}</ul>
<h2>缺失证据</h2><ul>{''.join('<li>'+html.escape(s)+'</li>' for s in metrics['warnings'])}</ul>
<p>原始失败信息：{html.escape(str(metrics['source_error']))}</p></html>'''
    (output/'report.html').write_text(body)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True,help='Must be a new directory outside input stage.')
    parser.add_argument('--max-feedback-gap-s',type=float,default=.3,help='Analysis coverage/interpolation bound; not a hardware capability claim.')
    args=parser.parse_args()
    if not math.isfinite(args.max_feedback_gap_s) or args.max_feedback_gap_s<=0:
        parser.error('--max-feedback-gap-s must be positive and finite')
    try:
        metrics,tables=analyze(args.stage_dir,args.max_feedback_gap_s)
        write_report(args.output,metrics,tables)
    except (ValueError,OSError,KeyError,TypeError) as exc:
        parser.exit(2,f'Analysis failed: {exc}\n')
    print(json.dumps(dict(output=str(args.output.resolve()),completed=metrics['completed'],
                         groups=metrics['group_timing']['group_count'],commands=metrics['command_count']),ensure_ascii=False))


if __name__=='__main__':main()

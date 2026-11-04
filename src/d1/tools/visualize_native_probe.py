#!/usr/bin/env python3
"""Offline native J5 probe report; never imports or starts robot code.

python3 tools/visualize_native_probe.py --run-dir /completed/probe/run \
    --output /a/new/report/directory

Only saved JSON/JSONL are read. Output is report.html plus report.json. A new
directory outside the evidence run is required. Read summary.completed as
experiment completion, not physical accuracy or target arrival.
"""
import argparse
import hashlib
import html
import json
import math
from pathlib import Path
import re
import statistics

DELAY_FACTOR=1.8*(math.sqrt(3)-1)
PHASES=['positive','return_from_positive','negative','return_from_negative','cover_A','cover_B','final_return']
POSITION=re.compile(r'^j\s+5\s+([-+\d.eE]+)\s+(\d+)\s*$')


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


def load(run):
    """Read/hash the exact same bytes. Missing optional logs remain explicit."""
    contents={};inputs={}
    for name in ('summary.json','probe.jsonl','transport.jsonl','power_session.jsonl','review.json'):
        path=run/name
        if path.is_file():
            raw=path.read_bytes()
            inputs[name]=dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw))
            contents[name]=([json.loads(line) for line in raw.splitlines() if line.strip()]
                            if name.endswith('.jsonl') else json.loads(raw))
    if 'summary.json' not in contents:
        raise ValueError('summary.json is required; no completed experiment is inferred from a live directory.')
    return contents,inputs


def build(run_dir):
    run=Path(run_dir).resolve();data,inputs=load(run)
    result=data['summary.json'];events=data.get('probe.jsonl',[])
    transport=data.get('transport.jsonl',[]);review=data.get('review.json',{})
    commands=[];skips=[];holds={};raw_q=[];probe_q=[];acks={};faults=[]
    for row in events:
        kind,t=row.get('kind'),row.get('t')
        if kind=='native_command' and row.get('position') is True:
            match=POSITION.fullmatch(row.get('command',''))
            if match and finite(t):
                commands.append(dict(phase=row.get('phase'),target_deg=float(match[1]),
                    requested_delay_ms=int(match[2]),host_submission_monotonic_s=t,
                    cli_tx_monotonic_s=None,receive_ack_monotonic_s=None,receive_ack_status=None,
                    cli_confirmation_monotonic_s=None,sequence=None,command=row['command']))
        elif kind=='native_receipt':
            pending=[c for c in commands if c['command']==row.get('command') and
                     c['cli_confirmation_monotonic_s'] is None and finite(t) and c['host_submission_monotonic_s']<=t]
            if pending:pending[-1]['cli_confirmation_monotonic_s']=t
        elif kind=='skipped_zero_distance':
            skips.append(dict(phase=row.get('phase'),target_deg=row.get('target_deg'),
                host_submission_monotonic_s=None,skip_event_monotonic_s=t,
                outcome='skipped_zero_distance',measured_deg=row.get('measured_deg')))
        elif kind=='cover_B_skipped':
            skips.append(dict(phase='cover_B',target_deg=None,host_submission_monotonic_s=None,
                skip_event_monotonic_s=t,outcome='skipped_cover_condition',
                movement_deg=row.get('movement_deg'),remaining_to_A_deg=row.get('remaining_to_A_deg')))
        elif kind=='hold_result':
            hold=row.get('result',{});holds[hold.get('phase',row.get('phase'))]=hold
        elif kind=='sample' and isinstance(row.get('q'),list) and len(row['q'])>5 and finite(row['q'][5]) and finite(t):
            probe_q.append(dict(t=t,q5_deg=row['q'][5],phase=row.get('phase'),source='probe_checked_SDK_state'))
    for row in transport:
        t,line=row.get('t'),row.get('line','')
        if not finite(t):continue
        if line.startswith(('[tx] ','[rx] ')):
            message=json.loads(line.split(' ',1)[1]);d=message.get('data',{});seq=message.get('seq')
            if line.startswith('[tx] ') and message.get('address')==1 and message.get('funcode')==1 and d.get('id')==5:
                pending=[c for c in commands if c['cli_tx_monotonic_s'] is None and
                         c['host_submission_monotonic_s']<=t and finite(d.get('angle')) and abs(c['target_deg']-d['angle'])<.00051]
                if pending:
                    c=pending[0];c.update(cli_tx_monotonic_s=t,sequence=seq,
                        tx_requested_delay_ms=d.get('delay_ms'))
                    acks[seq]=c
            elif line.startswith('[rx] ') and message.get('address')==3 and message.get('funcode')==1 and seq in acks:
                c=acks.pop(seq)
                c.update(receive_ack_monotonic_s=t,receive_ack_status=d.get('recv_status'))
            elif line.startswith('[rx] ') and message.get('address')==2 and message.get('funcode')==1 and finite(d.get('angle5')):
                raw_q.append(dict(t=t,q5_deg=d['angle5'],source='raw_SDK_feedback_host_log_receipt'))
        if line.startswith(('[参数错误]','[未发送]','[设备拒绝]','[失败]','[接收超时]','[DDS写入失败]','[执行/状态未确认]')):
            faults.append(dict(t=t,line=line))
    for index,c in enumerate(commands):
        following=commands[index+1]['host_submission_monotonic_s'] if index+1<len(commands) else math.inf
        matching_faults=[f for f in faults if c['host_submission_monotonic_s']<=f['t']<following]
        c['related_cli_faults']=matching_faults
        c['device_formula_duration_estimate_ms']=int(DELAY_FACTOR*c['requested_delay_ms'])
        c['device_formula_estimate_basis']='Approximate current-node conversion, assuming no speed/acceleration clamp; not measured duration.'
        if c['cli_tx_monotonic_s'] is None:
            c['outcome']='host_attempt_cli_not_sent' if any(f['line'].startswith(('[参数错误]','[未发送]')) for f in matching_faults) else 'host_attempt_no_tx_evidence'
        elif c['receive_ack_status']==1:
            c['outcome']='cli_published_receive_ack_confirmed'
        elif any(f['line'].startswith('[DDS写入失败]') for f in matching_faults):
            c['outcome']='cli_tx_recorded_publish_failed'
        elif c['receive_ack_status'] is not None:
            c['outcome']='cli_published_receive_rejected'
        else:
            c['outcome']='cli_published_receive_unknown'
        c['host_to_cli_tx_s']=(c['cli_tx_monotonic_s']-c['host_submission_monotonic_s']
                               if c['cli_tx_monotonic_s'] is not None else None)
        c['tx_to_receive_ack_s']=(c['receive_ack_monotonic_s']-c['cli_tx_monotonic_s']
                                  if c['receive_ack_monotonic_s'] is not None and c['cli_tx_monotonic_s'] is not None else None)
        c['hold_result']=holds.get(c['phase'])
        c['phase_failure']=result.get('error') if result.get('completed') is not True and result.get('final_phase')==c['phase'] else None
    planned=[]
    targets=review.get('targets_deg')
    if isinstance(targets,list) and len(targets)==len(PHASES):
        for phase,target in zip(PHASES,targets):
            matches=[c for c in commands+skips if c['phase']==phase]
            if matches:
                for c in matches:
                    if c.get('target_deg') is None:c['target_deg']=target
            else:
                planned.append(dict(phase=phase,target_deg=target,outcome='not_attempted',host_submission_monotonic_s=None))
    all_commands=commands+skips+planned
    all_commands.sort(key=lambda c:PHASES.index(c['phase']) if c.get('phase') in PHASES else -1)
    samples=raw_q or probe_q
    dedup={s['t']:s for s in reversed(samples)}
    samples=sorted(dedup.values(),key=lambda s:s['t'])
    times=[s['t'] for s in samples]
    gaps=[b-a for a,b in zip(times,times[1:])]
    power=[]
    for row in data.get('power_session.jsonl',[]):
        line=row.get('line','')
        if line.startswith(('[tx] ','[rx] ','[ok] ','[参数错误]','[未发送]','[失败]','[设备拒绝]')):
            power.append(dict(t=row.get('t'),line=line))
    all_times=times+[c[k] for c in commands for k in ('host_submission_monotonic_s','cli_tx_monotonic_s','receive_ack_monotonic_s') if finite(c.get(k))]
    return dict(schema='d1_native_j5_offline_report_v1',run_dir=str(run),inputs=inputs,
        analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        experiment_completed=result.get('completed') is True,error=result.get('error'),
        experiment_summary=result,physical_accuracy_verified=False,
        current_delay_conversion_factor=DELAY_FACTOR,
        current_delay_conversion_semantics='Requested SDK delay and current-node formula estimate are separate. Neither is observed motor duration.',
        q5_samples=samples,checked_probe_samples=probe_q,target_events=all_commands,
        power_session_events=power,cli_faults=faults,
        observation=dict(samples=len(samples),typical_interval_s=statistics.median(gaps) if gaps else None,
            typical_logged_hz=1/statistics.median(gaps) if gaps and statistics.median(gaps)>0 else None,
            maximum_gap_s=max(gaps) if gaps else None,
            host_time_origin=min(all_times) if all_times else None),
        missing_inputs=[n for n in ('probe.jsonl','transport.jsonl','power_session.jsonl','review.json') if n not in data],
        limitations=[
            '图中 q5 是 SDK 编码器反馈；编码器读数接近目标不等于独立物理精度已验证。',
            '目标阶梯以实际 CLI [tx] 的主机日志时刻画出，表示发布尝试；只有 host submission 而无 tx 的尝试和已明确写入失败分别标记，不作为生效目标。电机采用时刻未知。',
            '接收 ACK 表示协议接收；它不代表电机开始执行、完成运动或到达目标。',
            '机载约 100 ms 等待后逐轴查询，典型约 9 Hz。主机收到时间不是编码器采样时间，未观测到某中间姿态不能证明底层覆盖/排队语义。',
            '查询失败时可能保留旧缓存值；重复反馈、新鲜收到时间或健康状态本身都不能排除机载缓存风险。',
            '约 1.31769 × delay 是现役控制公式在未触发速度/加速度钳制时的推算，经过整数截断；不是实测电机执行时长。',
            '实验流程完成与精度通过分别呈现；失败、未尝试、CLI 未发送、零距离跳过分别保留。',
            '图中超过 0.6 s 的反馈间隔断线显示；没有计算 jerk 或推断高频机械振动。'
        ])


def chart(report):
    samples=report['q5_samples'];commands=report['target_events'];origin=report['observation']['host_time_origin']
    if origin is None:return '<p>没有反馈或命令时标，无法绘制。</p>'
    failed_outcomes={'cli_tx_recorded_publish_failed','cli_published_receive_rejected'}
    real=[c for c in commands if finite(c.get('cli_tx_monotonic_s')) and c['outcome'] not in failed_outcomes]
    times=[s['t'] for s in samples]+[c['cli_tx_monotonic_s'] for c in real]
    attempts=[c for c in commands if finite(c.get('host_submission_monotonic_s')) and
              (c.get('cli_tx_monotonic_s') is None or c['outcome'] in failed_outcomes)]
    times += [c['host_submission_monotonic_s'] for c in attempts]
    end=max(times) if times else origin+1
    values=[s['q5_deg'] for s in samples]+[c['target_deg'] for c in commands if finite(c.get('target_deg'))]
    if not values:return '<p>没有有效 q5 或目标角度。</p>'
    lo,hi=min(values)-.3,max(values)+.3
    span=max(end-origin,.1)
    x=lambda t:70+900*(t-origin)/span
    y=lambda q:330-250*(q-lo)/(hi-lo)
    parts=['<svg viewBox="0 0 1010 400" role="img" aria-label="J5反馈与实际发布目标"><rect x="70" y="80" width="900" height="250" fill="#fafbfc"/>']
    for i in range(6):
        q=lo+(hi-lo)*i/5;yy=y(q)
        parts.append(f'<path d="M70 {yy:.2f}H970" stroke="#dfe5ec"/><text x="12" y="{yy+5:.2f}">{q:.2f}°</text>')
    for i in range(7):
        t=origin+span*i/6;xx=x(t)
        parts.append(f'<text x="{xx:.2f}" y="355" text-anchor="middle">{t-origin:.2f}s</text>')
    segments=[];current=[];previous=None
    for s in samples:
        if previous is not None and s['t']-previous>.6:
            segments.append(current);current=[]
        current.append(s);previous=s['t']
    if current:segments.append(current)
    for segment in segments:
        coords=' '.join(f'{x(s["t"]):.2f},{y(s["q5_deg"]):.2f}' for s in segment)
        parts.append(f'<polyline points="{coords}" stroke="#1864ab" fill="none" stroke-width="2"/>')
    for s in samples:
        parts.append(f'<circle cx="{x(s["t"]):.2f}" cy="{y(s["q5_deg"]):.2f}" r="2" fill="#1864ab"><title>SDK q5 {s["q5_deg"]:.4f}°，主机相对时间 {s["t"]-origin:.4f}s</title></circle>')
    for index,c in enumerate(real):
        start=c['cli_tx_monotonic_s'];stop=real[index+1]['cli_tx_monotonic_s'] if index+1<len(real) else end
        xx,yy=x(start),y(c['target_deg'])
        parts.append(f'<path d="M{xx:.2f} {yy:.2f}H{x(stop):.2f}" stroke="#d9480f" stroke-width="2" stroke-dasharray="7 4"/><circle cx="{xx:.2f}" cy="{yy:.2f}" r="4" fill="#d9480f"><title>{html.escape(c["phase"])}：CLI [tx]目标 {c["target_deg"]}°；不是电机采用时刻</title></circle>')
        if finite(c.get('receive_ack_monotonic_s')):
            ax=x(c['receive_ack_monotonic_s'])
            parts.append(f'<path d="M{ax:.2f} 320v10" stroke="#7048e8" stroke-width="2"><title>接收ACK；不等于运动开始或到位</title></path>')
    for c in attempts:
        xx,yy=x(c['host_submission_monotonic_s']),y(c['target_deg'])
        label='CLI写入失败／接收被拒绝' if c['outcome'] in failed_outcomes else '仅主机尝试，缺少CLI发布证据'
        parts.append(f'<path d="M{xx-5:.2f} {yy+5:.2f}L{xx:.2f} {yy-5:.2f}L{xx+5:.2f} {yy+5:.2f}Z" fill="#f59f00"><title>{html.escape(c["phase"])}：{label}</title></path>')
    parts.append('<text x="70" y="382">蓝：SDK反馈　橙虚线：CLI [tx]目标　黄三角：未发布／失败　紫短线：接收ACK</text></svg>')
    return ''.join(parts)


LABELS=dict(not_attempted='未尝试／未执行到此阶段',skipped_zero_distance='零距离规则跳过',
    skipped_cover_condition='覆盖切换条件不满足，跳过',host_attempt_cli_not_sent='主机尝试；CLI明确未发送',
    host_attempt_no_tx_evidence='主机尝试；无CLI发布证据',cli_published_receive_ack_confirmed='CLI已发布；接收ACK确认',
    cli_published_receive_rejected='CLI [tx]已记录；接收被拒绝',cli_published_receive_unknown='CLI [tx]已记录；接收未知',
    cli_tx_recorded_publish_failed='CLI [tx]已记录；DDS写入失败')


def write(output,report):
    output=Path(output).resolve();run=Path(report['run_dir'])
    if output==run or run in output.parents:raise ValueError('Output must be outside the original evidence run.')
    output.mkdir(parents=True,exist_ok=False)
    (output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    origin=report['observation']['host_time_origin']
    def relative(t):return f'{t-origin:.4f}s' if finite(t) and origin is not None else '—'
    rows=[]
    for c in report['target_events']:
        hold=c.get('hold_result') or {}
        outcome=LABELS.get(c['outcome'],c['outcome'])+('；该阶段流程失败' if c.get('phase_failure') else '')
        cells=[c.get('phase'),c.get('target_deg'),outcome,
               relative(c.get('host_submission_monotonic_s')),relative(c.get('cli_tx_monotonic_s')),
               relative(c.get('receive_ack_monotonic_s')),c.get('requested_delay_ms'),
               c.get('device_formula_duration_estimate_ms'),hold.get('encoder_signed_residual_deg'),
               hold.get('encoder_peak_to_peak_deg'),hold.get('task_1deg_pass'),hold.get('assessment_0_3deg_pass')]
        rows.append('<tr>'+''.join('<td>'+html.escape('—' if v is None else str(v))+'</td>' for v in cells)+'</tr>')
    result=report['experiment_summary']
    body=f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>原生J5执行诊断</title>
<style>body{{font:16px/1.6 system-ui;max-width:1180px;margin:30px auto;padding:0 22px;color:#18324d}}svg{{width:100%}}text{{font:13px system-ui}}table{{border-collapse:collapse;font-size:13px}}th,td{{border:1px solid #ccd5df;padding:7px;vertical-align:top}}pre{{white-space:pre-wrap;background:#f3f6f9;padding:15px}}.scroll{{overflow-x:auto}}.error{{color:#a93226}}</style>
<h1>原生 J5 执行诊断</h1><p>实验流程完成：<b>{report['experiment_completed']}</b>；独立物理精度已验证：<b>False</b>。流程完成不等于精度通过。</p>
<p class="error">失败信息：{html.escape(str(report['error'] or '无已记录错误'))}</p><p>所有保持段 1°要求：{result.get('all_holds_task_1deg_pass','未知')}；所有保持段 0.3°评估：{result.get('all_holds_assessment_0_3deg_pass','未知')}。</p>
<p>输入：{html.escape(report['run_dir'])}。<a href="report.json">完整数据、事件与输入 SHA256</a>。</p>
<h2>编码器反馈与命令时间</h2><p>目标线从实际 CLI 发布日志时刻开始，仅表示发布目标；电机采用时刻未知。没有实际发布证据的目标只画黄三角。悬停数据点可见数值。</p>{chart(report)}
<p>样本：{report['observation']['samples']}；典型主机记录频率：{report['observation']['typical_logged_hz']} Hz；最大间隔：{report['observation']['maximum_gap_s']} s。</p>
<h2>目标逐项状态</h2><p>请求 delay 与现役公式估算时长分列；约 1.31769 × delay 不等于实测完成时长。</p><div class="scroll"><table><tr>{''.join('<th>'+s+'</th>' for s in ['阶段','目标°','提交／接收状态','主机尝试','CLI发布','接收ACK','请求delay ms','现役公式估算 ms','保持段编码器残差°','保持段波动°','1°要求','0.3°评估'])}</tr>{''.join(rows)}</table></div>
<h2>上电及CLI失败证据</h2><pre>{html.escape(json.dumps(dict(power_delivery=result.get('power_delivery'),power_session=report['power_session_events'],cli_faults=report['cli_faults']),ensure_ascii=False,indent=2))}</pre>
<h2>读数与判断边界</h2><ul>{''.join('<li>'+html.escape(s)+'</li>' for s in report['limitations'])}</ul><p>缺少的可选输入：{html.escape(', '.join(report['missing_inputs']) or '无')}。</p></html>'''
    (output/'report.html').write_text(body)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True,help='New directory outside evidence run')
    a=p.parse_args()
    try:
        report=build(a.run_dir);write(a.output,report)
    except (ValueError,OSError,KeyError,TypeError) as exc:p.exit(2,f'Offline report failed: {exc}\n')
    print(json.dumps(dict(output=str(a.output.resolve()),experiment_completed=report['experiment_completed'],
                         q5_samples=report['observation']['samples']),ensure_ascii=False))


if __name__=='__main__':main()

"""Export received feedback separately from command attempts and visual outcomes."""
import csv
from datetime import datetime, timezone, timedelta
import html
import json
import math
from pathlib import Path

ZONE = timezone(timedelta(hours=8))


def _constant(value):
    raise ValueError('Nonfinite JSON value: ' + value)


def _json(text):
    return json.loads(text, parse_constant=_constant)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _count(diagnostics, key):
    if diagnostics is not None:
        diagnostics[key] = diagnostics.get(key, 0) + 1


def load(path, default=None):
    try:
        return _json(Path(path).read_text())
    except (OSError, ValueError):
        return default


def rows(path, diagnostics=None):
    """Yield JSON objects only; truncated and malformed lines remain auditable."""
    try:
        with Path(path).open(errors='replace') as stream:
            for line in stream:
                if not line.strip():
                    continue
                try:
                    value = _json(line)
                    if not isinstance(value, dict):
                        raise ValueError('JSONL row is not an object')
                    yield value
                except (ValueError, TypeError):
                    _count(diagnostics, 'malformed_jsonl_rows')
    except OSError:
        _count(diagnostics, 'unreadable_jsonl_files')


def write_json(path, data):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def _stage(path, folder):
    for parent in path.parents:
        if parent == folder:
            break
        prefix, separator, _ = parent.name.partition('_')
        if separator and prefix.isdigit():
            return parent.name
    return 'unknown'


def feedback_marker_sample(event, now=None):
    """Read safe merged values; a marker's raw message may contain rejected axes."""
    if not isinstance(event,dict):raise ValueError('Feedback marker is not an object')
    stamp=event['received_monotonic_s']
    if not _finite(stamp) or stamp<0:
        raise ValueError('Invalid accepted feedback timestamp')
    if 'merged_angles' in event:
        values=event['merged_angles']
        if not isinstance(values,list) or len(values)!=7 or not all(_finite(v) for v in values):
            raise ValueError('Merged feedback requires seven finite angles')
        sample=dict(t=stamp,q=list(map(float,values)),feedback_value_source='merged_valid_axes')
        for key in ('joint_times_s','joint_sequences','joint_pending','accepted_joints',
                    'rejected_joints','joint_rejections'):
            if key in event:sample[key]=event[key]
        times=event.get('joint_times_s')
        if isinstance(times,list) and len(times)==7:
            basis=stamp if now is None else now
            sample['joint_feedback_age_s']=[max(0.,basis-t) if _finite(t) and t>=0 else None for t in times]
            sample['stale_feedback_joints']=[i for i,age in enumerate(sample['joint_feedback_age_s'])
                                              if age is None or age>.6]
        pending=event.get('joint_pending')
        if isinstance(pending,list) and len(pending)==7:
            sample['pending_feedback_joints']=[i for i,value in enumerate(pending) if value is True]
        sample['feedback_warning']=bool(sample.get('stale_feedback_joints') or sample.get('pending_feedback_joints'))
        return sample
    message=event['message']
    if not isinstance(message,dict):raise ValueError('Accepted feedback message is not an object')
    if message.get('address')!=2 or message.get('funcode')!=1:
        raise ValueError('Invalid accepted feedback message')
    values=[message['data'][f'angle{i}'] for i in range(7)]
    if not all(_finite(value) for value in values):
        raise ValueError('joint feedback requires seven finite numbers')
    return dict(t=stamp,q=list(map(float,values)),feedback_value_source='accepted_frame')


def feedback_records(folder, diagnostics=None):
    """Command rows are attempts, never joint samples or delivery confirmations."""
    folder = Path(folder)
    paths = sorted(set(folder.rglob('transport.jsonl')) | set(folder.rglob('readonly.jsonl')))
    for path in paths:
        if not path.resolve().is_relative_to(folder.resolve()):
            _count(diagnostics, 'outside_execution_log_files')
            continue
        stage = _stage(path, folder)
        filtered = False
        for row in rows(path, diagnostics):
            stamp = row.get('t')
            if not _finite(stamp):
                _count(diagnostics, 'invalid_transport_timestamps')
                continue
            origin = 'synthetic' if row.get('synthetic') is True else 'robot_rx'
            line = row.get('line')
            if row.get('kind') == 'feedback_filter':
                filtered = row.get('enabled') is True
                continue
            accepted = isinstance(line, str) and line.startswith('[feedback-accepted] ')
            rejected = isinstance(line, str) and line.startswith('[feedback-rejected] ')
            if rejected:
                filtered = True
                _count(diagnostics, 'rejected_angle_frames')
            if row.get('kind') == 'output' and isinstance(line, str) and (
                    accepted or rejected or ('[rx] ' in line and not filtered)):
                try:
                    if accepted or rejected:
                        filtered = True
                        event = _json(line.split(' ', 1)[1])
                        if not isinstance(event,dict):raise ValueError('Feedback marker is not an object')
                        rejected_joints=event.get('rejected_joints')
                        has_rejected_joints=isinstance(rejected_joints,list) and bool(rejected_joints)
                        if has_rejected_joints:
                            _count(diagnostics,'frames_with_rejected_joints')
                            if not rejected:_count(diagnostics,'rejected_angle_frames')
                        if rejected and 'merged_angles' not in event:continue
                        sample=feedback_marker_sample(event)
                        if has_rejected_joints and event.get('accepted_joints'):
                            _count(diagnostics,'partially_rejected_angle_frames')
                    else:
                        message = _json(line.split('[rx] ', 1)[1])
                        if not isinstance(message, dict):raise ValueError('RX is not an object')
                        if message.get('address')!=2 or message.get('funcode')!=1:continue
                        values=[message['data'][f'angle{i}'] for i in range(7)]
                        if not all(_finite(value) for value in values):raise ValueError('joint feedback requires seven finite numbers')
                        sample=dict(t=stamp,q=list(map(float,values)))
                    yield dict(**sample,stage=stage,source=str(path),kind='feedback',data_origin=origin)
                except (KeyError, ValueError, TypeError):
                    _count(diagnostics, 'malformed_joint_feedback_rows')
            elif row.get('kind') == 'command':
                sdk, target, joint = row.get('sdk'), row.get('target_sdk_units'), row.get('joint')
                if isinstance(sdk, list) and len(sdk) == 7 and all(_finite(value) for value in sdk):
                    target, joint = sdk, 'all'
                    units = 'J0-J5:deg;J6:SDK_position'
                elif isinstance(joint, int) and not isinstance(joint, bool) and 0 <= joint <= 6 and _finite(target):
                    units = 'SDK_position' if joint == 6 else 'deg'
                else:
                    _count(diagnostics, 'malformed_command_rows')
                    continue
                yield dict(t=stamp, stage=stage, source=str(path), kind='command', joint=joint,
                           target=target, target_units=units,
                           data_origin='synthetic' if origin == 'synthetic' else 'command_attempt')


def _iso(timestamp):
    try:
        return datetime.fromtimestamp(timestamp, ZONE).isoformat()
    except (OverflowError, OSError, ValueError, TypeError):
        return ''


def _stage_timings(path, end_unix, diagnostics):
    timings = {}
    current = None
    last_stamp = None

    def finish(stage, stamp, outcome=None):
        item = timings[stage]
        item['end_unix'] = max(item['start_unix'], stamp)
        if item.get('execution_start_unix') is not None and item.get('execution_end_unix') is None:
            item['execution_end_unix'] = item['end_unix']
        if outcome is not None:
            item['outcome'] = outcome

    for event in rows(path, diagnostics):
        state, stage, stamp = event.get('state'), event.get('stage'), event.get('time')
        if not _finite(stamp) or (last_stamp is not None and stamp < last_stamp):
            _count(diagnostics, 'invalid_or_reordered_stage_events')
            continue
        last_stamp = stamp
        if stage is not None and (not isinstance(stage, str) or not stage):
            _count(diagnostics, 'invalid_or_reordered_stage_events')
            continue
        if state in ('observing', 'planning', 'executing') and stage:
            if stage not in timings:
                if current is not None and timings[current]['end_unix'] is None:
                    finish(current, stamp, 'interrupted')
                timings[stage] = dict(start_unix=stamp, end_unix=None, execution_start_unix=None,
                                      execution_end_unix=None, outcome='in_progress')
            current = stage
            if state == 'executing' and timings[stage]['execution_start_unix'] is None:
                timings[stage]['execution_start_unix'] = stamp
        elif state in ('stage_completed', 'stable_close_shortfall_accepted_contact_unverified') and stage in timings:
            finish(stage, stamp, 'completed' if state == 'stage_completed' else 'accepted_contact_unverified')
        elif state == 'automatic_lift_result' and stage in timings:
            outcome = event.get('status')
            finish(stage, stamp, {'success': 'visual_success', 'failure': 'visual_failure',
                                  'unknown': 'visual_unknown'}.get(outcome, 'visual_unknown'))
        elif state == 'stopped_no_recovery' and current is not None:
            outcome = timings[current]['outcome']
            if outcome in ('completed', 'accepted_contact_unverified'):
                outcome = 'completed_then_session_stopped'
            elif not outcome.startswith('visual_'):
                outcome = 'stopped'
            finish(current, stamp, outcome)
    for name, item in timings.items():
        if item['end_unix'] is None:
            # The caller measures the session end with monotonic time. A crash or
            # cancellation may leave no terminal event; that remains interrupted.
            finish(name, max(item['start_unix'], end_unix, last_stamp or end_unix), 'interrupted')
        item['elapsed_s'] = max(0.0, item['end_unix'] - item['start_unix'])
        start, end = item['execution_start_unix'], item['execution_end_unix']
        item['execution_s'] = max(0.0, end - start) if start is not None and end is not None else 0.0
        item['started_at'] = _iso(item['start_unix'])
        item['ended_at'] = _iso(item['end_unix'])
    return timings


def _preparation_timings(path, diagnostics):
    """Read measured perception spans without treating them as robot motion."""
    items = []
    marks = {}
    for row in rows(path, diagnostics):
        if row.get('kind') == 'mark':
            if row.get('name') in ('recognition_start', 'perception_complete') and _finite(row.get('monotonic')):
                marks[row['name']] = row['monotonic']
            continue
        if row.get('kind') != 'span':
            continue
        name, category = row.get('name'), row.get('category')
        start, end, duration = row.get('start'), row.get('end'), row.get('duration_s')
        if (not isinstance(name, str) or not name or not isinstance(category, str) or
                not all(_finite(value) for value in (start, end, duration)) or
                end < start or duration < 0 or not math.isclose(duration, end - start, abs_tol=1e-4)):
            _count(diagnostics, 'invalid_preparation_timing_rows')
            continue
        items.append(dict(name=name, category=category, duration_s=duration,
                          start_monotonic=start, end_monotonic=end,
                          outcome=str(row.get('outcome', 'unknown'))))
    start, end = marks.get('recognition_start'), marks.get('perception_complete')
    if start is not None and end is not None and end >= start:
        elapsed = end - start
    elif items:
        # Cover the recorded intervals rather than summing potentially nested spans.
        elapsed = max(item['end_monotonic'] for item in items) - min(item['start_monotonic'] for item in items)
    else:
        elapsed = None
    return items, elapsed


def _visual_outcomes(result):
    details = result.get('details')
    details = details if isinstance(details, dict) else {}
    visual = details.get('visual_verifications')
    visual = visual if isinstance(visual, dict) else {}
    lift = visual.get('lift')
    lift = lift if isinstance(lift, dict) else {}
    returned = visual.get('placed_released_retreat')
    returned = returned if isinstance(returned, dict) else {}
    stated = result.get('grasp_result', details.get('grasp_result', 'unknown'))
    visual_status = lift.get('status')
    if result.get('mode') == 'demo':
        grasp = stated if stated in ('simulated_success', 'not_attempted') else 'unknown'
    elif visual_status in ('success', 'failure', 'unknown'):
        grasp = visual_status
    elif stated in ('failure', 'unknown', 'not_attempted'):
        grasp = stated
    else:
        grasp = 'unknown'
    if stated != grasp:
        result['reported_grasp_result'] = stated
        result['grasp_result_note'] = 'Resolved from explicit visual evidence; process completion is not physical grasp evidence.'
    result.update(grasp_result=grasp,
                  grasp_verified_by_visual_evidence=result.get('mode') != 'demo' and visual_status == 'success',
                  return_visible_at_target=result.get('mode') != 'demo' and returned.get('status') == 'success',
                  jaw_release_measured=False,
                  grasp_result_reason=lift.get('reason', details.get('error', result.get('error', ''))))


def execution_command_evidence(execution, command_rows, diagnostics, details):
    """Only explicit zero counts for every entered executor establish no motion commands."""
    execution=Path(execution);root=execution.resolve();reports=[];unknown=[];seen=set()
    details=details if isinstance(details,dict) else {}
    positive=bool(command_rows) or details.get('hardware_commands_sent') is True or any(
        _finite(details.get(key)) and details[key]>0 for key in ('commands','robot_commands_sent'))
    for stage in sorted(execution.glob('[0-9][0-9]_*')):
        if not ((stage/'executed').exists() or (stage/'execute_call').exists()): continue
        name=stage.name.split('_',1)[1];seen.add(name)
        if not stage.resolve().is_relative_to(root):
            unknown.append(stage.name);continue
        paths=[stage/'executed'/filename for filename in ('summary.json','result.json')
               if (stage/'executed'/filename).exists()]
        if not paths:
            unknown.append(stage.name);continue
        for path in paths:
            report=load(path,{}) if path.resolve().is_relative_to(root) else {}
            count=report.get('commands') if isinstance(report,dict) else None
            if type(count) is not int or count<0:
                unknown.append(stage.name);continue
            reports.append(dict(stage=name,path=str(path),commands=count))
            if count>0 or report.get('hardware_commands_sent') is True or report.get('receipts',0) not in (0,None):
                positive=True
    event_diagnostics={};events=execution/'events.jsonl'
    entered=set()
    if events.exists():
        if not events.resolve().is_relative_to(root):
            unknown.append('events.jsonl')
        else:
            for event in rows(events,event_diagnostics):
                if event.get('state') not in ('executing','stage_completed'): continue
                if isinstance(event.get('stage'),str) and event['stage']:
                    entered.add(event['stage'])
                else:
                    unknown.append('unidentified_execution_stage')
    completed=details.get('completed_stages')
    if isinstance(completed,list): entered.update(name for name in completed if isinstance(name,str))
    unknown.extend(sorted(entered-seen))
    invalid=any(value for key,value in diagnostics.items() if key in (
        'malformed_jsonl_rows','malformed_command_rows','invalid_transport_timestamps',
        'unreadable_jsonl_files','outside_execution_log_files')) or bool(event_diagnostics)
    # The coordinator marks potential motion before entering an executor. A
    # failed read-only target recheck before that boundary can prove no motion,
    # even though no executor report exists yet.
    recheck_only=False
    if details.get('hardware_commands_may_have_been_sent') is False and not entered and not seen:
        for path in execution.glob('[0-9][0-9]_*/target_recheck/result.json'):
            if not path.resolve().is_relative_to(root):continue
            report=load(path,{})
            if (isinstance(report,dict) and report.get('completed') is False
                    and type(report.get('robot_commands_sent')) is int and report['robot_commands_sent']==0):
                recheck_only=True
    proven=(bool(reports) or recheck_only) and not positive and not unknown and not invalid
    return dict(no_commands_proven=proven,stage_reports=reports,unresolved_stages=sorted(set(unknown)),
                readonly_target_recheck_only=recheck_only,
                scope='executor command counts or explicit read-only target recheck before any executor; cross-checked against send attempts')


def export_report(run_dir, result, clock):
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    diagnostics = {}
    samples = sorted(feedback_records(run_dir / 'execution', diagnostics), key=lambda row: row['t'])
    start_mono, start_unix = clock['monotonic'], clock['unix']
    feedback = [row for row in samples if row['kind'] == 'feedback']
    commands = [row for row in samples if row['kind'] == 'command']
    with (run_dir / 'joint_positions.csv').open('w', newline='', encoding='utf-8-sig') as stream:
        out = csv.writer(stream)
        out.writerow(['time_iso_utc8', 'elapsed_s', 'stage', *[f'J{i}_deg' for i in range(6)],
                      'J6_sdk_position', 'source', 'data_origin', 'feedback_value_source',
                      'joint_times_s_json','joint_sequences_json','joint_feedback_age_s_json',
                      'joint_pending_json','pending_feedback_joints_json',
                      'stale_feedback_joints_json','accepted_joints_json',
                      'rejected_joints_json','joint_rejections_json'])
        for row in feedback:
            elapsed = row['t'] - start_mono
            out.writerow([_iso(start_unix + elapsed), f'{elapsed:.6f}', row['stage'], *row['q'],
                          row['source'], row['data_origin'],row.get('feedback_value_source','raw_frame'),
                          *[json.dumps(row.get(key)) for key in ('joint_times_s','joint_sequences',
                              'joint_feedback_age_s','joint_pending','pending_feedback_joints',
                              'stale_feedback_joints','accepted_joints','rejected_joints','joint_rejections')]])
    with (run_dir / 'commands.csv').open('w', newline='', encoding='utf-8-sig') as stream:
        out = csv.writer(stream)
        out.writerow(['time_iso_utc8', 'elapsed_s', 'stage', 'joint', 'target_sdk_json', 'target_units',
                      'source', 'data_origin', 'delivery_status'])
        for row in commands:
            elapsed = row['t'] - start_mono
            out.writerow([_iso(start_unix + elapsed), f'{elapsed:.6f}', row['stage'], row['joint'],
                          json.dumps(row['target']), row['target_units'], row['source'], row['data_origin'],
                          'attempt_recorded_delivery_not_established'])
    duration = result.get('total_time_s')
    if not _finite(duration) or duration < 0:
        duration = None
        result['total_time_s'] = None
    end_unix = start_unix + (duration or 0.0)
    event_path = run_dir / 'execution/events.jsonl'
    timings = _stage_timings(event_path, end_unix, diagnostics) if event_path.exists() else {}
    preparation_path = run_dir / 'perception/timing.jsonl'
    preparation_timings, perception_time = (_preparation_timings(preparation_path, diagnostics)
                                            if preparation_path.exists() else ([], None))
    execution_started = bool(samples or timings or (run_dir / 'execution_clock.json').exists())
    preparation_time = result.get('preparation_time_s')
    if not _finite(preparation_time) or preparation_time < 0:
        preparation_time = duration if not execution_started else None
    result.update(stage_times={name: item['elapsed_s'] for name, item in timings.items()}, stage_timings=timings,
                  stage_timing_scope='observing/planning through stage completion and recorded verification; execution_s is separate',
                  preparation_timings=preparation_timings, preparation_time_s=preparation_time,
                  perception_time_s=perception_time,
                  preparation_timing_scope='preparation_time_s includes the whole measured preparation; perception_time_s and preparation_timings cover recorded perception only',
                  feedback_samples=len(feedback), actual_feedback_samples=sum(row['data_origin'] == 'robot_rx' for row in feedback),
                  simulated_feedback_samples=sum(row['data_origin'] == 'synthetic' for row in feedback),
                  command_count=len(commands), command_count_scope='logged send attempts, not delivery acknowledgements',
                  joint_log=str(run_dir / 'joint_positions.csv'), command_log=str(run_dir / 'commands.csv'),
                  log_diagnostics=diagnostics, session_started_at=_iso(start_unix),
                  session_ended_at=_iso(end_unix) if duration is not None else None,
                  session_timing_scope='execution' if execution_started else 'preparation',
                  execution_started_at=_iso(start_unix) if execution_started else None,
                  execution_ended_at=_iso(end_unix) if execution_started and duration is not None else None)
    command_evidence=execution_command_evidence(run_dir/'execution',commands,diagnostics,result.get('details'))
    result['execution_command_evidence']=command_evidence
    details=result.get('details') if isinstance(result.get('details'),dict) else {}
    if (result.get('mode')!='demo' and result.get('status') in ('failed','cancelled','unconfirmed')
            and command_evidence['no_commands_proven'] and not details.get('visual_verifications')
            and details.get('trajectory_completed') is not True
            and not (_finite(result.get('robot_commands_sent')) and result['robot_commands_sent']>0)):
        result.update(grasp_result='not_attempted',cycle_result='not_attempted',robot_commands_sent=0,
                      grasp_result_note='已核实本次停止前未下发运动指令；未执行抓取。')
    _visual_outcomes(result)
    write_json(run_dir / 'result.json', result)
    chart = ''
    if feedback:
        stride = max(1, math.ceil(len(feedback) / 1000))
        selected = feedback[::stride]
        lo = min(row['t'] for row in feedback)
        span = max(1e-3, max(row['t'] for row in feedback) - lo)
        colors = ['#2563eb', '#059669', '#dc2626', '#9333ea', '#d97706', '#0891b2', '#64748b']
        chart = '<svg viewBox="0 0 950 310" role="img" aria-label="记录的关节位置曲线">'
        for i, color in enumerate(colors):
            values = [row['q'][i] for row in selected]
            mn, mx = min(values), max(values)
            rng = max(.2, mx - mn)
            points = ' '.join(f"{90+(row['t']-lo)/span*830:.1f},{32+i*39+20-(row['q'][i]-mn)/rng*24:.1f}" for row in selected)
            unit = 'deg' if i < 6 else 'SDK'
            chart += f'<text x="5" y="{35+i*39}" fill="{color}">J{i} {mn:.1f}..{mx:.1f} {unit}</text><polyline fill="none" stroke="{color}" points="{points}"/>'
        chart += '</svg><p>各轴分别缩放；J0–J5 为度，J6 为原始 SDK 位置量。曲线使用各轴最后有效反馈，原始坏值保留在 transport 日志。CSV 同时记录逐轴有效时间、反馈年龄、待恢复状态及实际/模拟来源；合成位置不表示所有轴同时更新。</p>'
    status = {'completed': '流程完成', 'pregrasp_completed': '已到预抓取点', 'failed': '执行未完成', 'blocked': '准备未通过',
              'cancelled': '已停止后续指令', 'demo_completed': '演示完成'}.get(result.get('status'), result.get('status', '未知流程状态'))
    if result.get('status')=='pregrasp_completed' and result.get('execution_scope')=='fixed_pregrasp':
        status='已到固定预抓取姿态'
    grasp = {'success': '抬升随动已获视觉支持', 'failure': '未抓起（视觉证据）', 'unknown': '无法确认',
             'not_attempted': '未执行抓取', 'simulated_success': '模拟成功（无实机）'}[result['grasp_result']]
    cycle={'success':'成功','failure':'失败','failed':'失败','unknown':'未确认','unconfirmed':'未确认',
           'not_attempted':'未执行','not_completed':'未完成','simulated_success':'模拟成功（无实机）'}.get(result.get('cycle_result'),'未确认')
    seconds = f'{duration:.2f} 秒' if duration is not None else '未知'
    escaped = html.escape(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    timing_rows = ''.join(f'<tr><td>{html.escape(name)}</td><td>{item["elapsed_s"]:.3f}</td><td>{item["execution_s"]:.3f}</td><td>{html.escape(item["outcome"])}</td></tr>' for name, item in timings.items())
    timing_table = ('<h2>抓放阶段耗时</h2><table><tr><th>阶段</th><th>总耗时（秒）</th><th>执行（秒）</th><th>结束状态</th></tr>' + timing_rows + '</table>') if timings else '<p>抓放阶段未开始，无阶段执行耗时。</p>'
    preparation_labels = {
        'read_only_feedback_discovery_and_stationarity': '实机反馈读取与静止检查',
        'wrist_http_status_and_image': '腕部图像采集',
        'saved_image_and_observation_read': '保存图像与反馈读取',
        'foreground_http_inference': 'OpenCV 前景识别',
        'local_opencv_foreground': 'OpenCV 前景识别',
        'fixed_plane_silhouette_fit_and_coordinate_transform': '目标几何拟合与坐标转换',
    }
    preparation_table = ''
    if preparation_timings:
        preparation_rows = ''.join(
            f'<tr><td>{html.escape(preparation_labels.get(item["name"], item["name"]))}</td>'
            f'<td>{html.escape(item["category"])}</td><td>{item["duration_s"]:.3f}</td>'
            f'<td>{html.escape(item["outcome"])}</td></tr>' for item in preparation_timings)
        preparation_table = ('<h2>准备分项耗时</h2><p>以下为实际记录的采集、识别等准备步骤，不计为抓放执行。</p>'
                             '<table><tr><th>步骤</th><th>分类</th><th>耗时（秒）</th><th>结束状态</th></tr>' +
                             preparation_rows + '</table>')
    data_note = ('这是模拟数据，没有访问设备。' if result.get('mode') == 'demo' else
                 f'实际反馈 {result["actual_feedback_samples"]} 条，模拟反馈 {result["simulated_feedback_samples"]} 条。命令是单独保存的发送尝试，不代表送达或到位。')
    page = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>D1 抓放结果</title>
<style>body{{font:16px system-ui;max-width:1050px;margin:40px auto;padding:0 24px;background:#f6f8fc;color:#172033}}pre{{background:white;padding:20px;white-space:pre-wrap;border-radius:10px}}svg,table{{width:100%;background:white}}td,th{{text-align:left;padding:8px}}a{{color:#2563eb}}</style>
<h1>{html.escape(str(status))}</h1><p>抓取结果（视觉）：{html.escape(grasp)}；完整抓放结果：{html.escape(cycle)}；总耗时 {seconds}</p>
<p>{html.escape(data_note)}</p><p>流程完成、关节到位和视觉证据分别记录；当前程序不测量夹持力或松爪接触状态。</p>
<p><a href="joint_positions.csv">关节位置 CSV</a> · <a href="commands.csv">命令 CSV</a> · <a href="result.json">结果 JSON</a></p>{chart}{preparation_table}{timing_table}<pre>{escaped}</pre></html>'''
    (run_dir / 'report.html').write_text(page)
    return result

#!/usr/bin/env python3
"""Read-only comparison of physical grasp attempts; missing evidence stays null.

Usage: --trial original=/path/trial --trial software=/path/trial
       --trial hardware=/path/trial --output /new/report_directory
Or --inputs inputs.json, an array of {group, path, protocol?} entries.
Optional trial.json records mode, physical_attempt_started, trial_id and protocol.
Optional user outcome.json records verified_success, verifier, missed_grasp,
slip, abnormal_stop, placement_error_mm and measurement_method.
This command does not contact cameras, inference services or robot hardware.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

GROUPS = ('original', 'software', 'hardware')
PROTOCOL_KEYS = ('protocol_id', 'scene_id', 'object', 'placement_protocol_id',
                 'lift_height_m', 'gripper', 'speed')
CATEGORIES = ('acquisition', 'recognition', 'pose', 'planning', 'communication',
              'arm_execution', 'gripper_execution', 'stage_wait',
              'human_confirmation', 'verification', 'startup', 'serialization')


def read_optional(path):
    if not path.exists():
        return {}
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f'Expected an object: {path}')
    return value


def number(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0 else None


def boolean(value):
    return value if isinstance(value, bool) else None


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def percentile(values, q):
    values = sorted(values)
    if not values: return None
    position = (len(values)-1)*q
    low = int(position); high = min(low+1, len(values)-1)
    return values[low]+(values[high]-values[low])*(position-low)


def metric(values):
    present = [v for v in values if v is not None]
    return dict(measured=len(present), missing=len(values)-len(present),
        mean=statistics.mean(present) if present else None,
        p50=percentile(present,.5), p95=percentile(present,.95),
        minimum=min(present) if present else None, maximum=max(present) if present else None)


def protocol_identity(protocol):
    missing = [key for key in PROTOCOL_KEYS if key not in protocol or protocol[key] in (None, '', {}, [])]
    canonical = {key:protocol.get(key) for key in PROTOCOL_KEYS}
    return canonical, missing, None if missing else fingerprint(canonical)


def timing_values(timing, notes):
    categories = {key:None for key in CATEGORIES}
    result = dict(total_s=None, explicit_human_wait_s=None,
        excluding_explicit_human_wait_s=None, categories_s=categories,
        complete_interval=False, timing_definition=timing.get('definition'))
    if timing.get('schema') != 'd1-grasp-timing-v1':
        notes.append('Unified recognition_start→retreat_complete timing missing; other windows are not substituted.')
        return result
    for key in CATEGORIES:
        cell = timing.get('categories',{}).get(key,{})
        categories[key] = number(cell.get('union_s')) if cell.get('measured') is True else None
    total = number(timing.get('wall_cycle_s'))
    if timing.get('complete_interval') is not True or total is None:
        notes.append('Incomplete cycle interval retained; failed attempt remains in denominator.')
        return result
    result.update(total_s=total, complete_interval=True)
    human = number(timing.get('explicit_human_wait_s'))
    if human is not None and human <= total:
        result.update(explicit_human_wait_s=human, excluding_explicit_human_wait_s=total-human)
    else:
        notes.append('Explicit human wait absent or invalid; adjusted duration remains null.')
    claimed = number(timing.get('cycle_excluding_explicit_human_wait_s'))
    if claimed is not None and result['excluding_explicit_human_wait_s'] is not None and abs(claimed-result['excluding_explicit_human_wait_s'])>1e-6:
        notes.append('Adjusted-duration claim conflicts with measured total/human wait; recomputed from those fields.')
    return result


def stage_values(folder, notes):
    path = folder/'timing.jsonl'
    if not path.exists(): return {}
    from grasp_timing import union_seconds
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    if len({row.get('boot_id') for row in rows}) > 1:
        notes.append('Stage trace contains different boot IDs; per-stage times withheld.')
        return {}
    spans = {}
    for row in rows:
        if row.get('kind') != 'span': continue
        start, end = row.get('start'), row.get('end')
        if not isinstance(start,(int,float)) or not isinstance(end,(int,float)):
            notes.append('Invalid stage span omitted.'); continue
        key = str(row.get('category','unknown'))+'/'+str(row.get('name','unnamed'))
        spans.setdefault(key,[]).append((start,end))
    result = {}
    for key, intervals in spans.items():
        try: result[key] = union_seconds(intervals)
        except ValueError: notes.append(f'Invalid intervals for {key}; duration withheld.')
    return result


def load_trial(entry):
    if entry.get('group') not in GROUPS:
        raise ValueError('group must be original, software or hardware')
    source = Path(entry['path']).expanduser().resolve()
    folder = source if source.is_dir() else source.parent
    result_path = folder/'result.json' if source.is_dir() else source
    historical = False
    if not result_path.exists() and (folder/'project_status.json').exists():
        result_path = folder/'project_status.json'
        historical = True
    result = read_optional(result_path)
    historical = historical or result.get('schema','').startswith('d1-first-grasp-project-status')
    trial = read_optional(folder/'trial.json')
    outcome_path = folder/'outcome.json'
    outcome = read_optional(outcome_path)
    timing = read_optional(folder/'timing_summary.json') or result.get('timing',{})
    if not isinstance(timing,dict): timing={}
    notes=[]
    mode = trial.get('mode',entry.get('mode'))
    replay = mode in ('replay','offline','simulation') or result.get('source_kind') in ('saved_image','replay')
    physical = None
    if historical:
        notes.append('Historical composite with retries; individual grasp-attempt denominator cannot be reconstructed from this summary.')
    elif replay:
        physical=False
    elif trial.get('physical_attempt_started') is True or result.get('physical_grasp_executed') is True or result.get('hardware_commands_may_have_been_sent') is True:
        physical=True
    elif trial.get('physical_attempt_started') is False or result.get('physical_grasp_executed') is False or result.get('hardware_commands_may_have_been_sent') is False:
        physical=False
    elif isinstance(result.get('robot_commands_sent'),int) and result['robot_commands_sent']==0:
        physical=False
    else:
        notes.append('Physical attempt status unknown; record retained outside the known physical denominator.')
    protocol = entry.get('protocol', trial.get('protocol', result.get('protocol',{}))) or {}
    canonical, missing, cohort = protocol_identity(protocol)
    if missing: notes.append('Protocol incomplete: '+', '.join(missing))
    stopped = result.get('state')=='stopped_no_recovery' or bool(result.get('error'))
    flags = {key:boolean(outcome.get(key)) for key in ('missed_grasp','slip','abnormal_stop')}
    if stopped: flags['abnormal_stop']=True
    success = None
    if outcome.get('verified_success') is not None:
        if boolean(outcome.get('verified_success')) is False:
            success=False
        elif outcome.get('verified_success') is True and outcome.get('verifier'):
            success=True
        else:
            notes.append('Success annotation lacks verifier; not accepted as verified success.')
    elif result.get('physical_cycle_verified') is True and result.get('completed') is True:
        success=True
    elif stopped or result.get('physical_cycle_verified') is False:
        success=False if stopped else None
    if any(value is True for value in flags.values()) or stopped:
        if success is True: notes.append('Success conflicts with failure/stop evidence; counted as unsuccessful.')
        success=False
    if physical is not True: success=None
    placement = outcome.get('placement',{})
    error_mm = number(outcome.get('placement_error_mm',placement.get('error_mm')))
    method = outcome.get('measurement_method',placement.get('measurement_method'))
    if error_mm is None:
        error_mm=number(result.get('placement_error_mm'))
        method=method or result.get('placement_measurement_method')
    if error_mm is not None and not method:
        notes.append('Placement error value has no measurement_method; retained as unqualified evidence, not comparable metric.')
    metrics=timing_values(timing,notes)
    historical_window=None
    if historical:
        historic_timing=read_optional(folder/'timing_analysis.json')
        historical_window=number(historic_timing.get('interval',{}).get('wall_clock_s'))
    files=[path for path in (result_path,folder/'trial.json',outcome_path,folder/'timing_summary.json',folder/'timing.jsonl',folder/'timing_analysis.json') if path.exists()]
    return dict(trial_id=trial.get('trial_id',entry.get('trial_id',folder.name)),group=entry['group'],
        source=str(source),result_file=str(result_path.resolve()),source_files_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in files},
        record_kind='historical_composite' if historical else 'replay' if replay else 'physical_attempt' if physical else 'preflight_or_unknown',
        physical_attempt=physical,verified_success=success,outcome_annotations_present=bool(outcome),
        **flags,placement_error_mm=error_mm,placement_measurement_method=method,
        placement_metric_qualified=error_mm is not None and bool(method),
        protocol=canonical,protocol_missing=missing,cohort_id=cohort,
        metrics=metrics,stage_times_s=stage_values(folder,notes),
        stage_time_scope='all available named spans; may include final verification after retreat and overlap other categories',
        historical_reported_cycle_success=result.get('grasp_place_cycle_completed') if historical else None,
        historical_recorded_window_s=historical_window,notes=notes)


def summarize_group(rows):
    attempts=[row for row in rows if row['physical_attempt'] is True]
    unknown_physical=sum(row['physical_attempt'] is None for row in rows)
    stage_keys=sorted({key for row in attempts for key in row['stage_times_s']})
    placement_methods=sorted({row['placement_measurement_method'] for row in attempts if row['placement_metric_qualified']})
    return dict(records=len(rows),physical_attempts=None if unknown_physical else len(attempts),
        known_physical_attempts=len(attempts),
        verified_successes=sum(row['verified_success'] is True for row in attempts),
        known_unsuccessful=sum(row['verified_success'] is False for row in attempts),
        success_unknown=sum(row['verified_success'] is None for row in attempts),
        verified_success_rate=sum(row['verified_success'] is True for row in attempts)/len(attempts) if attempts and not unknown_physical else None,
        excluded_replay=sum(row['record_kind']=='replay' for row in rows),
        physical_status_unknown=unknown_physical,
        historical_composite_records=sum(row['record_kind']=='historical_composite' for row in rows),
        anomaly_counts={key:dict(true=sum(row[key] is True for row in attempts),
            false=sum(row[key] is False for row in attempts),unknown=sum(row[key] is None for row in attempts)) for key in ('missed_grasp','slip','abnormal_stop')},
        total_s=metric([row['metrics']['total_s'] for row in attempts]),
        excluding_explicit_human_wait_s=metric([row['metrics']['excluding_explicit_human_wait_s'] for row in attempts]),
        categories_s={key:metric([row['metrics']['categories_s'][key] for row in attempts]) for key in CATEGORIES},
        stages_s={key:metric([row['stage_times_s'].get(key) for row in attempts]) for key in stage_keys},
        placement_error_mm_by_method={method:metric([row['placement_error_mm'] if row['placement_measurement_method']==method and row['placement_metric_qualified'] else None for row in attempts]) for method in placement_methods},
        placement_unknown=sum(not row['placement_metric_qualified'] for row in attempts),
        cohort_ids=sorted({row['cohort_id'] for row in attempts if row['cohort_id']}),
        incomplete_protocol_attempts=sum(bool(row['protocol_missing']) for row in attempts))


def compare(entries):
    rows=[load_trial(entry) for entry in entries]
    identities=[(row['group'],row['result_file']) for row in rows]
    if len(set(identities))!=len(identities): raise ValueError('Duplicate source in one group would inflate denominator')
    # A physical attempt may not be relabelled into two experimental groups.
    physical_sources=[row['result_file'] for row in rows if row['physical_attempt'] is True]
    if len(set(physical_sources))!=len(physical_sources): raise ValueError('Same physical attempt assigned to multiple groups')
    groups={group:summarize_group([row for row in rows if row['group']==group]) for group in GROUPS}
    comparisons=[]
    for i,first in enumerate(GROUPS):
        for second in GROUPS[i+1:]:
            a,b=groups[first],groups[second]
            reasons=[]
            if not a['physical_attempts'] or not b['physical_attempts']:reasons.append('missing_physical_attempts')
            if a['physical_status_unknown'] or b['physical_status_unknown']:reasons.append('physical_attempt_denominator_unknown')
            if a['incomplete_protocol_attempts'] or b['incomplete_protocol_attempts']:reasons.append('incomplete_protocol')
            if len(a['cohort_ids'])!=1 or len(b['cohort_ids'])!=1 or a['cohort_ids']!=b['cohort_ids']:reasons.append('scene_object_placement_height_gripper_speed_or_protocol_mismatch')
            if a['total_s']['measured']==0 or b['total_s']['measured']==0:reasons.append('missing_comparable_cycle_timing')
            if a['total_s']['missing'] or b['total_s']['missing']:reasons.append('missing_cycle_time_for_some_attempts')
            comparable=not reasons
            ratio=a['total_s']['p50']/b['total_s']['p50'] if comparable and b['total_s']['p50']>0 else None
            comparisons.append(dict(first=first,second=second,comparable=comparable,reasons=reasons,
                median_total_speedup_first_over_second=ratio,
                note='Descriptive paired-protocol comparison only; missing failed-cycle times suppress speedup; sample counts do not imply statistical significance.'))
    return dict(schema='d1-grasp-trial-comparison-v1',read_only=True,robot_commands_sent=0,
        primary_interval='recognition_start to feedback-confirmed retreat_complete',
        denominator='All known physical attempts, including failure, stop, and missing outcome; replay and historical composites listed separately.',
        human_wait_rule='Subtract only explicitly measured human confirmation wait; no estimated residual subtraction.',
        timing_overlap_note='Stage/category intervals can overlap; never sum them to claim cycle duration.',
        groups=groups,comparisons=comparisons,trials=rows,
        caveats=['Placement methods are separated; image/model estimates are not independent physical measurements.',
                 'A historical composite with many retries does not supply a fair single-cycle baseline.',
                 'Missing fields remain null, not zero or successful.'])


def write_report(report, output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    (output/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    fields=['group','trial_id','record_kind','physical_attempt','verified_success','missed_grasp','slip','abnormal_stop','total_s','explicit_human_wait_s','excluding_explicit_human_wait_s','placement_error_mm','placement_measurement_method','cohort_id','protocol_missing','categories_s','stage_times_s','notes']
    with (output/'comparison.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for row in report['trials']:
            flat={key:row.get(key) for key in fields};flat.update({key:row['metrics'][key] for key in ('total_s','explicit_human_wait_s','excluding_explicit_human_wait_s')})
            flat['categories_s']=row['metrics']['categories_s']
            for key in ('protocol_missing','categories_s','stage_times_s','notes'):flat[key]=json.dumps(flat[key],ensure_ascii=False,sort_keys=True)
            flat={key:'null' if value is None else value for key,value in flat.items()}
            writer.writerow(flat)
    lines=['抓放对比（只读汇总）','统一区间：开始识别 → 反馈确认退离完成；人工等待仅扣明确记录。',
        '缺失为 unknown；失败仍在实机 attempts 分母；回放不进入实机分母。','',
        '组别 | 实机次数 | 核实成功 | 成败未知 | 总耗时p50(s) | 明确扣人工后p50(s) | 漏抓/滑落/异常停止']
    def show(value):return 'unknown' if value is None else f'{value:.3f}'
    for name,row in report['groups'].items():
        flags='/'.join(str(row['anomaly_counts'][key]['true'])+'(未知'+str(row['anomaly_counts'][key]['unknown'])+')' for key in ('missed_grasp','slip','abnormal_stop'))
        attempt_text='unknown' if row['physical_attempts'] is None else str(row['physical_attempts'])
        lines.append(f"{name} | {attempt_text} | {row['verified_successes']} | {row['success_unknown']} | {show(row['total_s']['p50'])} | {show(row['excluding_explicit_human_wait_s']['p50'])} | {flags}")
        lines.append(f"  回放排除{row['excluded_replay']}；历史复合记录{row['historical_composite_records']}；实机状态未知{row['physical_status_unknown']}；放置误差缺测{row['placement_unknown']}。")
    lines.append('')
    for row in report['comparisons']:
        lines.append(f"{row['first']} vs {row['second']}："+('可按相同协议描述性比较' if row['comparable'] else '不可比：'+', '.join(row['reasons'])))
    for row in report['trials']:
        if row['historical_recorded_window_s'] is not None:
            lines.append(f"历史记录 {row['trial_id']} 报告功能完成={row['historical_reported_cycle_success']}；{row['historical_recorded_window_s']:.3f}s 只是旧记录窗口，统一总耗时和单次attempt数unknown。")
    lines.append('分阶段和测量方法详见 JSON；类别时间有重叠，不直接相加。')
    (output/'comparison.txt').write_text('\n'.join(lines)+'\n')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',action='append',default=[],metavar='GROUP=PATH')
    parser.add_argument('--inputs',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    entries=json.loads(args.inputs.read_text()) if args.inputs else []
    if not isinstance(entries,list):parser.error('--inputs must contain an array')
    for value in args.trial:
        group,separator,path=value.partition('=')
        if not separator:parser.error('--trial requires GROUP=PATH')
        entries.append(dict(group=group,path=path))
    if not entries:parser.error('At least one input record required')
    report=compare(entries);write_report(report,args.output)
    print(json.dumps({'output':str(args.output.resolve()),'physical_attempts':{k:v['physical_attempts'] for k,v in report['groups'].items()},'robot_commands_sent':0}))


if __name__=='__main__':main()

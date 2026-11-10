"""Explain saved continuous-shadow diagnostics against recorded teacher targets.

Reads existing predictions only: no policy inference, training or hardware I/O.
Future teacher commands are labels used here solely as retrospective controls.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

import numpy as np

from .contracts import LIMITS
from .continuous_shadow import connection,SPEED,EPS

BOARD_SOURCE=Path('/home/aup/D1/Control/logs/gripper_recovery_20261023_201607/controller_patch/marm_controller_node.cpp')
HOST_PROTOCOL=Path('/home/aup/D1/Control/src/protocol.hpp')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())


def entry_statistics(records,key):
    if not records:return dict(n=0,required_speed_exceeded=0,required_speed_median_deg_s=None,
        required_speed_p95_deg_s=None,required_speed_max_deg_s=None,arm_gap_mean_absolute_deg=None,
        arm_gap_max_absolute_deg=None,per_joint_mean_absolute_gap_deg=None)
    values=np.asarray([r[key]['required_speed_deg_s'] for r in records],float)
    gaps=np.asarray([r[key]['arm_absolute_gap_deg'] for r in records],float)
    return dict(n=len(values),required_speed_exceeded=int(sum(values>SPEED+EPS)),
        required_speed_median_deg_s=float(np.median(values)),required_speed_p95_deg_s=float(np.quantile(values,.95)),
        required_speed_max_deg_s=float(values.max()),arm_gap_mean_absolute_deg=float(gaps.mean()),
        arm_gap_max_absolute_deg=float(gaps.max()),per_joint_mean_absolute_gap_deg=gaps.mean(0).tolist())


def contextual_row(record,example):
    if record['observation_id']!=example['id'] or record['split']!=example['split']:
        raise ValueError('example_record_identity_mismatch')
    identity=dict(observation_id=example['id'],split=example['split'])
    if record['raw_policy_action'] is None:
        return dict(**identity,available=False,unavailable_reason='no_saved_policy_prediction')
    if len(example['actions'])<2 or len(example['is_pad'])<2 or example['is_pad'][1] or len(example['actual_target_times'])<2:
        return dict(**identity,available=False,unavailable_reason='teacher_index1_missing_or_padded')
    observation=record['historical_observation'];candidate=record['review']['candidate']
    if observation['q']!=example['q']:raise ValueError('observation_q_mismatch')
    def measure(target,target_time):
        point=dict(time=target_time,q=target)
        c=connection([observation,point])
        return dict(target=target,target_time=target_time,observation_q_time=observation['time'],
            arm_absolute_gap_deg=abs(np.asarray(target[:6])-np.asarray(observation['q'][:6])).tolist(),
            required_speed_deg_s=c['last_speed_deg_s'],required_speed_pass=c['speed_pass'],
            acceleration_known=False)
    teachers=example['actions'];actual_time=example['actual_target_times'][1]
    previous=record['previous_recorded_commands'][-1] if record['previous_recorded_commands'] else None
    return dict(**identity,available=True,
        policy_index1=measure(candidate['q'],candidate['time']),
        teacher_index1_same_candidate_time=measure(teachers[1],candidate['time']),
        teacher_index1_actual_recorded_time=measure(teachers[1],actual_time),
        teacher_target_index0_minus_observed_q_deg=(np.asarray(teachers[0][:6])-np.asarray(example['q'][:6])).tolist(),
        previous_actually_sent_teacher_target_minus_observed_q_deg=None if previous is None else
            (np.asarray(previous['q'][:6])-np.asarray(example['q'][:6])).tolist(),
        teacher_target_role='retrospective comparison label only; never passed to policy inference',
        teacher_index1_source=dict(path=example['action_source_log'],line=example['action_source_lines'][1],time=actual_time))


def summarize(dataset,review_dir,output):
    dataset=Path(dataset).resolve();review_dir=Path(review_dir).resolve();output=Path(output).resolve()
    manifest=read(dataset/'manifest.json');protocol=read(review_dir/'protocol.json');summary=read(review_dir/'summary.json')
    if sha(dataset/'manifest.json')!=protocol['dataset_manifest_sha256']:raise ValueError('dataset_manifest_mismatch')
    if sha(manifest['examples_file'])!=manifest['examples_sha256']:raise ValueError('dataset_examples_changed')
    if sha(review_dir/'predictions.jsonl')!=summary['predictions_sha256']:raise ValueError('review_predictions_changed')
    examples={r['id']:r for r in map(json.loads,Path(manifest['examples_file']).read_text().splitlines())}
    records=[json.loads(line) for line in (review_dir/'predictions.jsonl').read_text().splitlines()]
    controls=[contextual_row(r,examples[r['observation_id']]) for r in records]
    by_split={}
    for split in sorted({r['split'] for r in records}):
        subset=[r for r in records if r['split']==split];all_labels=[r for r in controls if r['split']==split]
        labels=[r for r in all_labels if r['available']]
        gaps=np.abs(np.asarray([r['teacher_target_index0_minus_observed_q_deg'] for r in labels]))
        prior=[r['previous_actually_sent_teacher_target_minus_observed_q_deg'] for r in labels
               if r['previous_actually_sent_teacher_target_minus_observed_q_deg'] is not None]
        prior=np.abs(np.asarray(prior))
        by_split[split]=dict(observations=len(subset),diagnostic_pass=sum(r['status']=='DIAGNOSTIC_PASS_ONLY' for r in subset),
            teacher_control_n_available=len(labels),teacher_control_n_unavailable=len(all_labels)-len(labels),
            teacher_control_unavailable_reasons=dict(Counter(r['unavailable_reason'] for r in all_labels if not r['available'])),
            rejection_counts=dict(Counter(reason for r in subset for reason in r['rejection_reasons'])),
            entry_controls={key:entry_statistics(labels,key) for key in
                ('policy_index1','teacher_index1_same_candidate_time','teacher_index1_actual_recorded_time')},
            teacher_current_target_gap=dict(mean_absolute_deg=float(gaps.mean()),maximum_absolute_deg=float(gaps.max()))
                if len(labels) else dict(mean_absolute_deg=None,maximum_absolute_deg=None),
            teacher_previous_sent_target_gap=dict(n=len(prior),mean_absolute_deg=float(prior.mean()),maximum_absolute_deg=float(prior.max()))
                if len(prior) else dict(n=0))
    board=BOARD_SOURCE.read_text()
    def bounds(name):
        match=re.search(r'float\s+'+name+r'\[7\]\s*=\s*\{([^}]+)\}',board)
        if not match:raise ValueError('saved_board_bounds_not_found')
        values=[float(v.strip()) for v in match.group(1).split(',')]
        if len(values)!=7:raise ValueError('invalid_saved_board_bounds')
        return values
    lo,hi=bounds('minangle'),bounds('maxangle')
    intersection=[[max(a,lo[i]),min(b,hi[i])] for i,(a,b) in enumerate(LIMITS)]
    all_commands=np.asarray([q for r in records if r['raw_policy_action'] is not None for q in r['raw_policy_action']['commands']])
    selected=np.asarray([r['review']['candidate']['q'] for r in records if r['raw_policy_action'] is not None])
    board_scope=dict(policy_contract_limits=LIMITS,saved_board_limits=list(map(list,zip(lo,hi))),
        offline_intersection=intersection,original_review_used_intersection=False,old_contract_changed=False,
        full_controller_range_verification=False,source_scope='Saved 2026-10-23 source; no current hardware or firmware read was performed',
        raw_chunk_j6_range_sdk=[float(all_commands[:,6].min()),float(all_commands[:,6].max())],
        selected_index1_j6_range_sdk=[float(selected[:,6].min()),float(selected[:,6].max())],
        raw_chunk_positions_outside_saved_intersection=int(np.sum(np.any((all_commands<np.asarray(intersection)[:,0])|
            (all_commands>np.asarray(intersection)[:,1]),axis=1))))
    sources=[Path(__file__),Path(__file__).with_name('continuous_shadow.py'),Path(__file__).with_name('shadow.py'),
        Path(__file__).with_name('contracts.py'),BOARD_SOURCE,HOST_PROTOCOL,review_dir/'protocol.json',
        review_dir/'summary.json',review_dir/'predictions.jsonl',dataset/'manifest.json',Path(manifest['examples_file'])]
    result=dict(schema='d1-act-continuous-shadow-interpretive-controls-v1',by_split=by_split,limits=board_scope,
        interpretation=['Required-displacement rejection is an interface diagnostic under the fixed next-tick schedule, not a measured ACT physical trajectory or proof of unsafe skill.',
            'The same rule also rejects many teacher targets because commanded targets differ from actual recorded q. This comparison exposes that confound; it does not waive the entry gate.',
            'Teacher index-1 targets are future recorded labels used only after inference for this comparison, never model inputs or policy-generated feedback.',
            'All raw index-0 targets expire structurally: they are timestamped at inference trigger and completion has positive latency. This is not evidence of inadequate compute.',
            'Initial acceleration remains unknown without prior velocity; quantized historical feedback derivatives are not independently measured physical acceleration.',
            'Policy J6 contract is [0,65]; saved host/board intersection is [0,50]. Original review is not full onboard-range validation.'],
        source_sha256={str(p.resolve()):sha(p) for p in sources},new_inference=False,new_training=False,robot_commands=0,hardware_accessed=False)
    output.mkdir(parents=True,exist_ok=False)
    (output/'context_rows.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in controls))
    (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    lines=['ACT 连续历史观察接口审查补充结果（2026-10-24）','',
        '纯离线：本补充只读取已保存预测，没有重新推理、训练、发送命令或访问硬件。',
        '固定候选调度：原始 index 1、原定下一 tick；不平移动作时间，不裁剪，不追赶。',
        'index 0 全部过期是“目标时刻等于推理触发时刻＋正推理耗时”的结构性结果，不表示算力不足。','']
    def number(value):return '不可用' if value is None else f'{value:.4f}'
    for split,data in by_split.items():
        policy=data['entry_controls']['policy_index1'];teacher=data['entry_controls']['teacher_index1_same_candidate_time']
        lines.extend([f'{split}: {data["observations"]} 条；数值诊断通过 {data["diagnostic_pass"]}。',
            f'教师下一目标对照可用 {data["teacher_control_n_available"]} 条，不可用 {data["teacher_control_n_unavailable"]} 条；以下比例仅使用可用分母。',
            f'相同 q/候选时刻的首步 required-displacement 速度超过 0.5°/s：ACT {policy["required_speed_exceeded"]}/{policy["n"]}，教师下一目标 {teacher["required_speed_exceeded"]}/{teacher["n"]}。',
            f'对应速度中位数：ACT {number(policy["required_speed_median_deg_s"])}°/s，教师 {number(teacher["required_speed_median_deg_s"])}°/s。',
            f'教师当前目标与观察 q 的六轴平均绝对差 {number(data["teacher_current_target_gap"]["mean_absolute_deg"])}°，最大 {number(data["teacher_current_target_gap"]["maximum_absolute_deg"])}°。',''])
    lines.extend(['当前直接接入调度假设未通过；命令—反馈跟踪差是共同影响，不能据此认定实物危险或 ACT 能力失败。接口拒绝仍保留，没有据此放宽边界。',
        '过去教师目标用于因果起点诊断；未来教师目标仅在本补充中作事后对照，从未输入 ACT。后续记录 q 来自老师轨迹，不是模型执行反馈。',
        f'旧 policies/contracts.py 的 J6 限位为 [0,65]，已保存机载目标为 [-20,50]，交集 [0,50]；未改旧 contract。候选全部原始动作块 J6 范围 {board_scope["raw_chunk_j6_range_sdk"]}，选取 index 1 范围 {board_scope["selected_index1_j6_range_sdk"]}。',
        '本轮只是对已保存范围作补充数值比较，不是实时固件核验、碰撞验证或完整实机允许性判断。',
        '未知初始速度/加速度保留不可判定；量化反馈差分不是独立实测物理加速度。',
        '详细分组计数、逐条对照及输入/源码 SHA 见本目录 summary.json 与 context_rows.jsonl。'])
    (output/'中文结果.txt').write_text('\n'.join(lines)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',required=True);p.add_argument('--review',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();r=summarize(a.dataset,a.review,a.output)
    print(json.dumps(dict(status='SAVED_PREDICTIONS_CONTEXT_COMPLETE',output=a.output,limits=r['limits']),ensure_ascii=False))


if __name__=='__main__':main()

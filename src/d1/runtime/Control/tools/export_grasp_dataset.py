#!/usr/bin/env python3
"""Offline causal export of separately recorded SDK commands and actual states.

Images stay in place. Stages remain separate segments, never a fabricated
continuous episode. Missing clock alignment is retained as trainable=false.
"""
import argparse
from bisect import bisect_left
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

LIMITS = [(-135, 135), (-90, 90), (-90, 90), (-135, 135), (-90, 90), (-135, 135), (0, 65)]
SERIALS = {'global': 'CC1365302K0', 'wrist': 'CC1365301EY'}
UNITS = ['degree'] * 6 + ['uncalibrated_sdk_raw']
CYCLE_ORDER = ['approach', 'close', 'lift', 'lower', 'open', 'retreat']


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def rows(path):
    with path.open() as stream:
        for line, text in enumerate(stream, 1):
            yield line, json.loads(text)


def before(records, keys, command_key):
    index = bisect_left(keys, command_key) - 1
    return records[index] if index >= 0 else None


def provenance(path, cache):
    path = path.resolve()
    key = str(path)
    if key not in cache:
        cache[key] = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(path=key, sha256=cache[key])


def labels_for(stage, result, result_path, project, project_path):
    visual = None
    evidence = []
    for item in project.get('historical_lift_attempts', []):
        if item.get('stage') == stage and item.get('object_lifted') is False:
            visual = False
            evidence.append(dict(path=str(project_path), field='historical_lift_attempts', stage=stage))
    lift_source = project.get('lift_measurement', {}).get('source_result')
    if lift_source and Path(lift_source).parent.name == stage:
        visual = project.get('physical_lift_verified')
        evidence.append(dict(path=str(project_path), field='physical_lift_verified/lift_measurement.source_result'))
    member = any(item.get('stage') == stage and item.get('completed') is True
                 for item in project.get('stage_chain', []))
    cycle = project.get('grasp_place_cycle_completed') is True
    if member:
        evidence.append(dict(path=str(project_path), field='stage_chain/grasp_place_cycle_completed'))
    robot = result.get('completed') is True
    return dict(robot_execution_completed=robot, robot_result_evidence=str(result_path) if result_path else None,
                visual_stage_success=visual, completed_cycle_member=member and cycle,
                visual_evidence=evidence, visual_label_is_per_command=False,
                demonstration_success=robot and visual is not False and (visual is True or (member and cycle)))


def run_file(run, name):
    """Evidence must belong to this run, not a previous successful run."""
    path = Path(name)
    path = (path if path.is_absolute() else run/path).resolve()
    if not path.is_relative_to(run.resolve()):
        raise ValueError('Evidence escapes this run')
    return path


def bound_review(run, result, prefix, flag, hashes):
    request_path, gate_path = run/(prefix+'_request.json'), run/(prefix+'.json')
    proof = dict(verified=False, request=str(request_path), review=str(gate_path))
    try:
        request, gate = json.loads(request_path.read_text()), json.loads(gate_path.read_text())
        provenance(request_path, hashes); provenance(gate_path, hashes)
        approval = result.get('approval_sha256')
        if not approval or request.get('approval_sha256') != approval or gate.get('approval_sha256') != approval:
            raise ValueError('Approval binding mismatch')
        if result.get(flag) is not True or gate.get(flag) is not True or gate.get('verifier') not in ['human', 'supervising_agent']:
            raise ValueError('Physical review not explicitly confirmed')
        observation = run_file(run, request['observation'])
        observation_sha = provenance(observation, hashes)['sha256']
        if request.get('observation_sha256') != observation_sha or gate.get('observation_sha256') != observation_sha:
            raise ValueError('Observation binding mismatch')
        images = {}
        for role in SERIALS:
            image_path = run_file(run, observation.parent/(role+'.jpg'))
            if image_path.stat().st_size == 0:
                raise ValueError('Empty physical review image')
            images[role] = provenance(image_path, hashes)
        expected = {role: item['sha256'] for role, item in images.items()}
        if request.get('image_sha256') != expected or gate.get('image_sha256') != expected:
            raise ValueError('Image binding mismatch')
        proof.update(verified=True, verifier=gate['verifier'], observation=str(observation),
                     observation_sha256=observation_sha, images=images)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        proof['invalid_reason'] = str(exc)
    return proof


def fsm_context(run, hashes):
    result_path = run/'result.json'
    result = json.loads(result_path.read_text()) if result_path.exists() else {}
    for path in [result_path, run/'events.jsonl']:
        if path.exists(): provenance(path, hashes)
    return dict(result=result, result_path=str(result_path),
                lift_review=bound_review(run, result, 'verification', 'physical_grasp_verified', hashes),
                cycle_review=bound_review(run, result, 'cycle_verification', 'physical_cycle_verified', hashes))


def fsm_labels(folder, run, semantic_stage, context, hashes):
    result_path = next((p for p in [folder/'result.json', folder/'summary.json'] if p.exists()), None)
    result = json.loads(result_path.read_text()) if result_path else {}
    top = context['result']
    robot = result.get('completed') is True
    accepted = robot and semantic_stage in top.get('completed_stages', [])
    shortfall = dict(accepted=False)
    if semantic_stage == 'close' and not robot:
        path = folder.parent/'stable_shortfall.json'
        try:
            item = json.loads(path.read_text()); provenance(path, hashes)
            classification = 'stable_position_shortfall/contact_unverified'
            if (item.get('classification') != classification or top.get('close_classification') != classification
                    or item.get('contact_detected') is not False or item.get('grasp_verified') is not False
                    or semantic_stage not in top.get('accepted_stages', [])):
                raise ValueError('Shortfall acceptance is not explicitly recorded')
            evidence = item.get('evidence', [])
            if not isinstance(evidence, list) or not evidence:
                raise ValueError('Shortfall acceptance lacks file evidence')
            checked = []
            for source in evidence:
                check = provenance(run_file(run, source['path']), hashes)
                if check['sha256'] != source['sha256']:
                    raise ValueError('Shortfall evidence SHA mismatch')
                checked.append(check)
            shortfall.update(accepted=True, path=str(path), classification=classification, evidence=checked,
                             contact_detected=False, grasp_verified=False)
            accepted = True
        except (OSError, ValueError, KeyError, TypeError) as exc:
            shortfall['invalid_reason'] = str(exc)
    cycle = (context['cycle_review']['verified'] and context['lift_review']['verified']
             and top.get('completed') is True and top.get('trajectory_completed') is True)
    physical = cycle or (semantic_stage in ['approach', 'close', 'lift'] and context['lift_review']['verified'])
    return dict(robot_execution_completed=robot, robot_result_evidence=str(result_path) if result_path else None,
                stage_execution_accepted=accepted, shortfall_acceptance=shortfall,
                visual_stage_success=True if physical else None, completed_cycle_member=cycle and accepted,
                physical_cycle_verified=cycle, visual_label_is_per_command=False,
                visual_evidence=[context['lift_review'], context['cycle_review']],
                demonstration_success=accepted and physical)


def export_stage(folder, output, run, project, project_path, hashes,
                 feedback_max_age=.6, camera_max_age=.25, semantic_stage=None, segment_name=None, fsm=None):
    semantic_stage = semantic_stage or folder.name
    segment_name = segment_name or folder.name
    absolute = (folder / 'transport.jsonl').exists()
    source = folder / ('transport.jsonl' if absolute else 'motion/events.jsonl')
    result_path = next((p for p in [folder / 'result.json', folder / 'summary.json'] if p.exists()), None)
    result = json.loads(result_path.read_text()) if result_path else {}
    origin = result.get('session_origin_monotonic')
    origin = origin if finite(origin) else None
    clock_aligned = absolute or origin is not None
    provenance(source, hashes)
    if result_path:
        provenance(result_path, hashes)
    stage_labels = (fsm_labels(folder, run, semantic_stage, fsm, hashes) if fsm is not None else
                    labels_for(folder.name, result, result_path, project, project_path))
    observations, statuses, commands = [], [], []
    status_state = {}
    previous_time = -math.inf
    for line, item in rows(source):
        t = item.get('t')
        if not finite(t) or t < previous_time:
            raise ValueError(f'{source}:{line}: non-finite/reordered event time')
        previous_time = t
        if not absolute and 'host_monotonic' in item:
            declared_origin = item.get('session_origin_monotonic')
            host_time = item['host_monotonic']
            if not finite(declared_origin) or not finite(host_time) or abs(host_time-t-declared_origin) > 1e-6:
                raise ValueError(f'{source}:{line}: inconsistent explicit monotonic clock fields')
            if origin is not None and abs(origin-declared_origin) > 1e-6:
                raise ValueError(f'{source}:{line}: session origin changed within segment')
            origin = declared_origin
        key = (t, line)
        if item.get('kind') == 'command':
            values = item.get('sdk')
            if values is None:
                values = [None] * 7
                joint = item.get('joint')
                if type(joint) is int and 0 <= joint <= 6:
                    values[joint] = item.get('target_sdk_units', item.get('target_deg'))
            commands.append((key, list(values), line))
        text = item.get('line', '')
        if '[rx] ' not in text:
            continue
        message = json.loads(text.split('[rx] ', 1)[1])
        if message.get('address') != 2:
            continue
        data = message.get('data', {})
        received = item.get('received_t', t)
        if not finite(received) or received > t:
            raise ValueError(f'{source}:{line}: invalid receipt time')
        if message.get('funcode') == 1:
            q = [data.get(f'angle{i}') for i in range(7)]
            observations.append(dict(key=key, received_t=received, q=q, source_line=line))
        if message.get('funcode') == 3:
            for name in ['power_status', 'enable_status', 'error_status']:
                if name in data:
                    status_state[name] = dict(value=data[name], received_t=received, source_line=line)
            statuses.append(dict(key=key, values=dict(status_state)))
    obs_keys = [x['key'] for x in observations]
    status_keys = [x['key'] for x in statuses]
    clock_aligned = absolute or origin is not None
    images = {role: [] for role in SERIALS}
    frame_path = folder / 'frames/frames.jsonl'
    if frame_path.exists():
        provenance(frame_path, hashes)
        last_index = {}
        for line, item in rows(frame_path):
            role = item.get('role')
            if role not in images or not finite(item.get('host_monotonic')):
                continue
            index = item.get('index')
            ordered = isinstance(index, int) and index > last_index.get(role, -1)
            if isinstance(index, int):
                last_index[role] = index
            image_path = (frame_path.parent / item['file']).resolve()
            images[role].append(dict(**item, source_line=line, image_path=image_path,
                                     frame_index_order_valid=ordered))
        for role in images:
            images[role].sort(key=lambda x: x['host_monotonic'])
    image_keys = {role: [x['host_monotonic'] for x in items] for role, items in images.items()}
    counts = Counter()
    output_path = output / (segment_name + '.jsonl')
    with output_path.open('x') as stream:
        for index, (key, target, command_line) in enumerate(commands):
            t = key[0]
            absolute_t = t if absolute else t + origin if origin is not None else None
            obs = before(observations, obs_keys, key)
            status = before(statuses, status_keys, key)
            q = obs['q'] if obs else [None] * 7
            valid_q = [finite(v) and lo <= v <= hi for v, (lo, hi) in zip(q, LIMITS)]
            q_age = t - obs['received_t'] if obs else None
            masks = dict(feedback_present=obs is not None, feedback_values_valid=all(valid_q),
                         feedback_fresh=q_age is not None and 0 <= q_age <= feedback_max_age,
                         command_valid=len(target) == 7 and any(v is not None for v in target)
                           and all(v is None or (finite(v) and lo <= v <= hi) for v, (lo, hi) in zip(target, LIMITS)),
                         camera_clock_aligned=clock_aligned)
            status_observation = {}
            for name, required in [('power_status', 1), ('enable_status', 1), ('error_status', 0)]:
                value = (status or {}).get('values', {}).get(name)
                age = t - value['received_t'] if value else None
                status_observation[name] = dict(value=value['value'] if value else None, age_s=age,
                    source_line=value['source_line'] if value else None)
                masks[name+'_valid'] = value is not None and value['value'] == required and 0 <= age <= feedback_max_age
            camera_observation = {}
            for role in SERIALS:
                frame = None
                if absolute_t is not None:
                    # Arrival must precede the command; old exposure timestamps
                    # do not make a frame arriving later causally available.
                    frame = before(images[role], image_keys[role], absolute_t)
                entry = dict(path=None, sha256=None, available_age_s=None, capture_age_estimate_s=None)
                valid_frame = False
                if frame is not None:
                    available_age = absolute_t - frame['host_monotonic']
                    board_age = frame.get('age_ms')
                    total_age = available_age + board_age / 1000 if finite(board_age) and board_age >= 0 else None
                    exists = frame['image_path'].is_file() and frame['image_path'].stat().st_size > 0
                    entry.update(path=str(frame['image_path']), available_t=frame['host_monotonic'],
                        available_age_s=available_age, capture_age_estimate_s=total_age,
                        board_age_at_fetch_ms=board_age, frame_index=frame.get('index'),
                        serial=frame.get('serial'), source_log=str(frame_path), source_line=frame['source_line'])
                    if exists:
                        entry['sha256'] = provenance(frame['image_path'], hashes)['sha256']
                    valid_frame = exists and frame.get('serial') == SERIALS[role] and frame['frame_index_order_valid'] \
                        and total_age is not None and 0 <= total_age <= camera_max_age
                masks[role+'_image_valid'] = valid_frame
                camera_observation[role] = entry
            masks['demonstration_success_label'] = stage_labels['demonstration_success']
            reasons = [name for name, valid in masks.items() if not valid]
            counts['samples'] += 1
            counts['trainable' if not reasons else 'not_trainable'] += 1
            for reason in reasons:
                counts['invalid:'+reason] += 1
            sample = dict(schema='d1-causal-command-state-v1', segment_id=run.name+'/'+segment_name,
                stage=semantic_stage,
                sample_index=index, segment_start=index == 0, segment_end=index == len(commands)-1,
                continuous_across_segments=False, command_t=t, command_host_monotonic=absolute_t,
                clock='host_monotonic' if absolute else 'session_relative_monotonic',
                action=dict(sdk_target=[v if finite(v) else None for v in target],
                    commanded_joint_mask=[v is not None for v in target], units=UNITS,
                    source_log=str(source), source_line=command_line),
                observation=dict(q=[v if finite(v) else None for v in q], units=UNITS,
                    q_valid_mask=valid_q, q_age_s=q_age, q_available_t=obs['key'][0] if obs else None,
                    q_received_t=obs['received_t'] if obs else None, q_source_line=obs['source_line'] if obs else None,
                    status=status_observation, cameras=camera_observation),
                masks=masks, trainable=not reasons, invalid_reasons=reasons, labels=stage_labels)
            stream.write(json.dumps(sample, allow_nan=False) + '\n')
    return dict(stage=semantic_stage, segment_id=run.name+'/'+segment_name,
                samples_file=str(output_path), source_log=str(source),
                source_clock='host_monotonic' if absolute else 'session_relative_monotonic',
                camera_clock_aligned=clock_aligned, source_origin_monotonic=origin,
                result_path=str(result_path) if result_path else None, labels=stage_labels,
                counts=dict(counts), complete_episode=False,
                first_command_host_monotonic=(commands[0][0][0]+(0 if absolute else origin) if commands and clock_aligned else None),
                last_command_host_monotonic=(commands[-1][0][0]+(0 if absolute else origin) if commands and clock_aligned else None))


def export_run(run, output, stages=None):
    run, output = Path(run).resolve(), Path(output).resolve()
    def logged(path):
        return (path/'transport.jsonl').exists() or (path/'motion/events.jsonl').exists()
    discovered = []
    for folder in sorted(p for p in run.iterdir() if p.is_dir()):
        if logged(folder):
            discovered.append(dict(folder=folder, stage=folder.name, segment_name=folder.name, fsm=False))
        elif logged(folder/'executed'):
            prefix, separator, name = folder.name.partition('_')
            if separator and prefix.isdigit() and name in ['pregrasp'] + CYCLE_ORDER:
                discovered.append(dict(folder=folder/'executed', stage=name, segment_name=folder.name, fsm=True))
    selected = [x for x in discovered if stages is None or x['stage'] in stages or x['segment_name'] in stages]
    if stages is not None and any(not any(name in [x['stage'], x['segment_name']] for x in selected) for name in stages):
        raise ValueError('Unknown requested stage')
    if not selected:
        raise ValueError('Each selected stage must contain transport.jsonl or motion/events.jsonl')
    if len({x['segment_name'] for x in selected}) != len(selected):
        raise ValueError('Duplicate stage names')
    output.mkdir(parents=True, exist_ok=False)
    hashes = {}
    project_path = run / 'project_status.json'
    project = json.loads(project_path.read_text()) if project_path.exists() else {}
    if project_path.exists():
        provenance(project_path, hashes)
    provenance(Path(__file__), hashes)
    context = fsm_context(run, hashes) if any(x['fsm'] for x in selected) else None
    segments = [export_stage(x['folder'], output, run, project, project_path, hashes,
                    semantic_stage=x['stage'], segment_name=x['segment_name'], fsm=context if x['fsm'] else None)
                for x in selected]
    recorded = (context['result'].get('completed_stages', []) + context['result'].get('accepted_stages', [])) if context else []
    required_order = (['pregrasp'] if 'pregrasp' in recorded else []) + CYCLE_ORDER
    episode_checks = dict(single_fsm_root=bool(context) and all(x['fsm'] for x in selected),
        all_ordered_stages=[x['stage'] for x in selected] == required_order and len(selected) == len(discovered),
        clocks_complete=all(x['camera_clock_aligned'] and x['counts'].get('samples', 0) > 0 for x in segments),
        stage_execution_accepted=all(x['labels'].get('stage_execution_accepted') is True for x in segments),
        trajectory_completed=bool(context) and context['result'].get('trajectory_completed') is True,
        run_completed=bool(context) and context['result'].get('completed') is True,
        end_to_end_physical_verified=bool(context) and context['lift_review']['verified'] and context['cycle_review']['verified'])
    episode_checks['command_time_order'] = episode_checks['clocks_complete'] and all(
        left['last_command_host_monotonic'] < right['first_command_host_monotonic']
        for left, right in zip(segments[:-1], segments[1:]))
    complete_episode = all(episode_checks.values())
    totals = Counter()
    for segment in segments:
        totals.update(segment['counts'])
    manifest = dict(schema='d1-offline-segmented-dataset-v1', run=str(run), segments=segments,
        counts=dict(totals), source_sha256=hashes, training_performed=False, hardware_accessed=False,
        continuous_complete_episode=False, copied_images=False,
        complete_episode=complete_episode, complete_episode_checks=episode_checks,
        episode_evidence=context, all_robot_endpoints_completed=all(x['labels']['robot_execution_completed'] for x in segments),
        thresholds=dict(feedback_max_age_s=.6, image_capture_age_estimate_max_s=.25),
        semantics=dict(action='Actual SDK target command, never replaced by actual feedback.',
            observation='Latest available earlier feedback; invalid latest feedback is retained and masked, not silently replaced.',
            partial_action='Single-joint commands leave other targets null; feedback is never invented as their action.',
            image_timing='Latest earlier host receipt; age includes reported board age. No synchronized exposure guarantee.',
            unknown_relative_clock='No timestamp origin inferred from mtime or first camera frame; camera masks false.',
            labels='Robot arrival and visual/cycle outcome are distinct, provenance-linked retrospective labels, not input observations.',
            fsm_labels='FSM physical labels require run-bound review JSON, observation and both image SHA matches; completed alone is insufficient. Accepted close shortfall never changes its original failed endpoint label.',
            complete_episode='One root with all six cycle stages and any recorded pregrasp, ordered and clock-aligned, with accepted execution and two bound physical reviews. This flag does not merge stage gaps into uninterrupted control or assert all samples pass quality gates.',
            trainable='Passed listed causal/data-quality/success-label gates; not proof of learned policy or validated image decoder.',
            independent_segments='Temporal gaps and separate stage stops remain separate segments; no fabricated cross-stage transitions.'))
    (output/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    (output/'README.txt').write_text('分段示范离线导出，未训练模型，未访问设备。\n命令与实际反馈分别保存；图像使用原路径及SHA256，没有复制。\n仅关联命令之前可用的反馈和图像。trainable=false样本保留供审查，不能直接当成功训练样本。\n各阶段独立，未伪装为无间断控制。complete_episode仅在同次FSM六阶段及已记录的预抓取阶段、时钟、接受结果及首尾物理图片核验齐全时为true。\n相对日志缺少绝对单调时钟原点时，不猜图像对齐。原始轨迹结果与物理结果独立保存。\n详见manifest.json的标签证据、阈值、来源哈希与统计。\n')
    return manifest


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--stages', nargs='+')
    args = p.parse_args(argv)
    result = export_run(args.run, args.output, args.stages)
    print(json.dumps(dict(output=str(args.output.resolve()), counts=result['counts'],
                         segments=[dict(stage=x['stage'], counts=x['counts']) for x in result['segments']]),
                     ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

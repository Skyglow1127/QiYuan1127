"""Build offline sequence supervision from hash-bound, previously executed plans.

No model, ACT checkpoint, camera, transport or robot executor is instantiated.
This prepares weak supervision, not a trained predictor or grasp/collision labels.
Run: python -m scripts.preaction.prepare_sequence_data --inventory ... --output ...
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict, OrderedDict
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .core import PARAM_SCALE, box_of, crop, crop_spec, red_mask
from .prepare_cycle_pairs import (EXPORTER, command_reference, load_module,
                                  plan_command_mask, planned_references)
from .sequence_contract import DELTA_SCALE, FEATURE_COLUMNS, Q_SCALE, UNITS

SCHEMA = 'preaction-sequence-dataset-v1'
STEPS, RATE, STRIDE = 10, 5., 10
SERIALS = {'wrist': 'CC1365301EY', 'global': 'CC1365302K0'}
SERIAL = SERIALS['wrist']  # Backwards-compatible synthetic wrist fixture constant.
REQUIRED = ('feedback_present', 'feedback_values_valid', 'feedback_fresh',
            'command_valid', 'camera_clock_aligned', 'power_status_valid',
            'enable_status_valid', 'error_status_valid')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    def invalid(value):
        raise ValueError('nonfinite_json:' + value)
    return json.loads(Path(path).read_text(), parse_constant=invalid)


def jsonlines(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def finite(x):
    return type(x) in (int, float) and math.isfinite(x)


def save_json(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def verify_inventory(path):
    """Recheck the approved source list, never discover additional log folders."""
    inventory = read_json(path)
    if inventory.get('schema') != 'preaction-sequence-source-inventory-v1':
        raise ValueError('inventory_schema')
    for item in inventory.get('manifests', []):
        if sha(item['path']) != item['sha256']:
            raise ValueError('source_manifest_hash_mismatch')
    selected, groups, seen = [], {}, set()
    for source in inventory['sources']:
        for name, digest in source['source_paths_sha256'].items():
            if sha(name) != digest:
                raise ValueError('source_hash_mismatch:' + name)
        split = source['suggested_split']
        if split == 'excluded':
            continue
        if split not in ('train', 'val', 'development_validation'):
            raise ValueError('unsupported_inventory_split')
        split = 'development_validation' if split == 'val' else split
        if set(source.get('original_splits', [])) & {'val', 'test'}:
            raise ValueError('prior_holdout_must_remain_excluded')
        group = source['group']
        if group in groups and groups[group] != split:
            raise ValueError('group_crosses_split')
        groups[group] = split
        folder = Path(source['path']).resolve()
        if folder in seen:
            raise ValueError('duplicate_source')
        seen.add(folder)
        required_paths = [folder / name for name in
                          ('plan.json', 'transport.jsonl', 'result.json', 'frames/frames.jsonl')]
        if any(str(p) not in source['source_paths_sha256'] for p in required_paths):
            raise ValueError('missing_required_source_hash')
        plan = read_json(folder / 'plan.json')
        prior = Path(plan['source_plan']).resolve()
        candidate = Path(plan['candidate']).resolve()
        if any(str(p) not in source['source_paths_sha256'] for p in (prior, candidate)):
            raise ValueError('missing_plan_chain_hash')
        if (sha(prior) != plan['source_plan_sha256']
                or sha(candidate) != plan['candidate_sha256']):
            raise ValueError('plan_source_chain_mismatch')
        stages = [s for s in read_json(prior)['stages'] if s.get('name') == plan['stage']]
        if len(stages) != 1 or stages[0].get('sha256') != plan['source_trajectory_sha256']:
            raise ValueError('source_trajectory_declaration_mismatch')
        if plan['sdk_plan']['rate'] != RATE:
            raise ValueError('expected_5hz_plan')
        result = read_json(folder / 'result.json')
        if result.get('hardware_commands_sent') is not True or result.get('commands', 0) <= 0:
            raise ValueError('no_executed_commands')
        selected.append((source, plan))
    if not {'train', 'development_validation'} <= set(groups.values()):
        raise ValueError('train_and_development_validation_required')
    return inventory, selected


def verify_protocol(path, inventory_path):
    protocol = read_json(path)
    if protocol.get('schema') != 'preaction-sequence-pilot-protocol-v1':
        raise ValueError('protocol_schema')
    if (Path(protocol['inventory_path']).resolve() != Path(inventory_path).resolve()
            or protocol['inventory_sha256'] != sha(inventory_path)):
        raise ValueError('protocol_inventory_mismatch')
    if protocol.get('data', {}).get('camera_role') not in SERIALS:
        raise ValueError('unsupported_camera_role')
    expected = dict(chunk_size=10, rate_hz=5., anchor_stride=10,
                    min_valid_labels=5, feedback_max_age_s=.6, image_max_age_s=.25,
                    command_schedule_tolerance_s=.1, label_time_tolerance_s=.12,
                    label_step_zero=False)
    if any(protocol.get('data', {}).get(k) != v for k, v in expected.items()):
        raise ValueError('unsupported_frozen_data_protocol')
    return protocol


def encode_plan(q, raw_targets, joint_mask, offsets):
    """Same 17 columns/scales as sequence_contract, with raw None preserved elsewhere."""
    if len(q) != 7 or not all(finite(x) for x in q):
        raise ValueError('invalid_current_q')
    if len(raw_targets) != STEPS or len(offsets) != STEPS:
        raise ValueError('expected_ten_plan_steps')
    if len(joint_mask) != 7 or any(type(x) is not bool for x in joint_mask):
        raise ValueError('invalid_joint_mask')
    if any(not finite(t) for t in offsets) or any(b <= a for a, b in zip(offsets, offsets[1:])):
        raise ValueError('invalid_target_offsets')
    features = []
    for target, offset in zip(raw_targets, offsets):
        if len(target) != 7 or [v is not None for v in target] != joint_mask:
            raise ValueError('raw_target_mask_mismatch')
        if any(v is not None and not finite(v) for v in target):
            raise ValueError('invalid_planned_target')
        delta = [(target[j]-q[j])/DELTA_SCALE[j] if joint_mask[j] else 0. for j in range(7)]
        features.append(delta + list(map(float, joint_mask)) + [offset, 1., float(offset > 0)])
    q_features = np.asarray(q, np.float32) / np.asarray(Q_SCALE, np.float32)
    features = np.asarray(features, np.float32)
    if not np.isfinite(features).all() or not np.isfinite(q_features).all():
        raise ValueError('feature_float32_overflow')
    return q_features, features


def load_frames(folder, role='wrist'):
    if role not in SERIALS:
        raise ValueError('unsupported_camera_role')
    frames, rejected = [], Counter()
    path = folder / 'frames/frames.jsonl'
    last_index, last_t = -1, -math.inf
    for line, frame in enumerate(jsonlines(path), 1):
        if frame.get('role') != role:
            continue
        stamp, age, index = frame.get('host_monotonic'), frame.get('age_ms'), frame.get('index')
        if (frame.get('serial') != SERIALS[role] or not finite(stamp) or not finite(age)
                or not 0 <= age <= 250 or type(index) is not int
                or index <= last_index or stamp <= last_t):
            rejected['frame_identity_age_or_order'] += 1
            continue
        last_index, last_t = index, stamp
        image_path = (path.parent / frame['file']).resolve()
        if not image_path.is_relative_to(path.parent.resolve()):
            raise ValueError('frame_path_escapes_source')
        if not image_path.is_file():
            rejected['missing_frame'] += 1
            continue
        frames.append(dict(path=str(image_path), sha256=sha(image_path),
                           available_t=stamp, capture_time_estimate=stamp-age/1000,
                           board_age_at_fetch_ms=age, frame_index=index,
                           role=role, serial=SERIALS[role], source_log=str(path), source_line=line))
    frames.sort(key=lambda x: x['capture_time_estimate'])
    return frames, rejected


def image_data(frame, cache):
    key = frame['path'], frame['sha256']
    if key not in cache:
        encoded = Path(frame['path']).read_bytes()
        if hashlib.sha256(encoded).hexdigest() != frame['sha256']:
            raise ValueError('image_hash_mismatch')
        bgr = cv2.imdecode(np.frombuffer(encoded, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None or bgr.shape != (480, 640, 3):
            raise ValueError('invalid_original_rgb_shape')
        mask = red_mask(bgr)
        cache[key] = (bgr, mask, box_of(mask))
        if len(cache) > 48:
            cache.pop(next(iter(cache)))
    return cache[key]


def boundary_cut(box, spec=None):
    if box[0] <= 0 or box[1] <= 0 or box[2] >= 640 or box[3] >= 480:
        return True
    return spec is not None and (box[0] <= spec[0] or box[1] <= spec[1]
                                or box[2] >= spec[0]+spec[2] or box[3] >= spec[1]+spec[2])


def validate_window(rows, refs, i):
    start = rows[i]['command_host_monotonic']
    for k, row in enumerate(rows[i:i+STEPS]):
        bad = [name for name in REQUIRED if row['masks'].get(name) is not True]
        if bad:
            raise ValueError('window_' + bad[0])
        observed = row['observation']['q_available_t']
        if not finite(observed) or not 0 < row['command_host_monotonic']-observed <= .6:
            raise ValueError('window_noncausal_or_stale_q')
        offset = refs[i+k][0]-refs[i][0]
        if abs((row['command_host_monotonic']-start)-offset) > .1:
            raise ValueError('window_command_schedule')
    return start


def label_time_rejection(frame, wanted, start, next_command, used, last_capture):
    """A label must advance the actual observed image sequence, not only its index."""
    if frame is None or abs(frame['capture_time_estimate']-wanted) > .12:
        return 'label_outside_time_tolerance'
    capture = frame['capture_time_estimate']
    if capture <= start:
        return 'label_not_future'
    if capture <= last_capture:
        return 'label_nonmonotonic'
    if next_command is None or capture >= next_command:
        return 'label_reaches_next_command_or_boundary'
    if frame['sha256'] in used:
        return 'label_repeated_or_current_image'
    return None


def make_sample(source, plan, rows, refs, mask, i, frames, cache, label_rejections, role='wrist'):
    if role not in SERIALS:
        raise ValueError('unsupported_camera_role')
    start = validate_window(rows, refs, i)
    row = rows[i]
    current = row['observation']['cameras'][role]
    frame_by_path = {frame['path']: frame for frame in frames}
    if row['masks'].get(role+'_image_valid') is not True or current.get('path') not in frame_by_path:
        raise ValueError('current_image_invalid')
    current_frame = frame_by_path[current['path']]
    if (current.get('serial') != SERIALS[role]
            or any(f.get('role') != role or f.get('serial') != SERIALS[role] for f in frames)):
        raise ValueError('mixed_or_wrong_camera_identity')
    if current.get('sha256') != current_frame['sha256']:
        raise ValueError('image_hash_mismatch')
    age = start-current_frame['available_t']+current_frame['board_age_at_fetch_ms']/1000
    if not current_frame['available_t'] < start or not 0 <= age <= .25:
        raise ValueError('current_capture_stale_or_noncausal')
    if abs(current['capture_age_estimate_s']-age) > 1e-6:
        raise ValueError('current_capture_age_inconsistent')
    bgr, current_mask, box = image_data(current_frame, cache)
    if boundary_cut(box):
        raise ValueError('current_target_boundary_cut')
    spec = crop_spec(box)
    rgb = cv2.cvtColor(crop(bgr, spec), cv2.COLOR_BGR2RGB)
    local_mask = crop(current_mask, spec, nearest=True)
    rgb_mask = np.concatenate((rgb.transpose(2, 0, 1).astype(np.float32)/255,
                               local_mask[None]), axis=0).astype(np.float32)
    offsets = [float(refs[i+k][0]-refs[i][0]) for k in range(STEPS)]
    grip = None if plan_command_mask(plan) is not None else plan['requested_gripper_target_sdk']
    raw_targets = [command_reference(refs[i+k][1], grip, mask if not all(mask) else None)
                   for k in range(STEPS)]
    q = row['observation']['q']
    q_features, sequence_features = encode_plan(q, raw_targets, mask, offsets)
    target = np.zeros((STEPS, 4), np.float32)
    label_mask = np.zeros(STEPS, np.bool_)
    future_rgb = np.zeros((STEPS, 3, 96, 96), np.uint8)
    future_mask = np.zeros((STEPS, 96, 96), np.uint8)
    future_box = np.zeros((STEPS, 4), np.float32)
    label_offsets = np.zeros(STEPS, np.float64)
    label_details = [dict(valid=False, reason='step_zero_has_no_future_label')]
    used = {current_frame['sha256']}
    last_capture = start
    times = [f['capture_time_estimate'] for f in frames]
    for k in range(1, STEPS):
        wanted = start+offsets[k]
        # Select the nearest temporally admissible, previously unused image.
        # Visibility is evaluated afterwards; do not pick a nicer-looking label.
        lo, hi = bisect_left(times, wanted-.12), bisect_left(times, wanted+.12+1e-10)
        candidates = frames[lo:hi]
        next_command = rows[i+k+1]['command_host_monotonic'] if i+k+1 < len(rows) else None
        admissible = [f for f in candidates if last_capture < f['capture_time_estimate']
                      and next_command is not None and f['capture_time_estimate'] < next_command
                      and f['sha256'] not in used]
        frame = min(admissible or candidates,
                    key=lambda f: abs(f['capture_time_estimate']-wanted), default=None)
        reason = label_time_rejection(frame, wanted, start, next_command, used, last_capture)
        if reason is None:
            try:
                fbgr, fmask, fbox = image_data(frame, cache)
                if boundary_cut(fbox, spec):
                    reason = 'label_target_boundary_cut'
            except ValueError as error:
                if str(error) == 'image_hash_mismatch':
                    raise
                reason = 'label_no_usable_red_target_or_rgb'
        if reason:
            label_rejections[reason] += 1
            label_details.append(dict(valid=False, reason=reason, candidate=frame))
            continue
        used.add(frame['sha256'])
        last_capture = frame['capture_time_estimate']
        center = (box[:2]+box[2:])/2
        fcenter = (fbox[:2]+fbox[2:])/2
        target[k] = np.r_[(fcenter-center)/spec[2],
                          np.log((fbox[2:]-fbox[:2])/(box[2:]-box[:2]))]*PARAM_SCALE
        future_rgb[k] = cv2.cvtColor(crop(fbgr, spec), cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
        future_mask[k] = crop(fmask, spec, nearest=True).astype(np.uint8)
        future_box[k], label_mask[k] = fbox, True
        label_offsets[k] = frame['capture_time_estimate']-start
        label_details.append(dict(valid=True, image=frame, actual_horizon_s=label_offsets[k]))
    if label_mask.sum() < 5:
        raise ValueError('fewer_than_five_labels')
    split = 'train' if source['suggested_split'] == 'train' else 'val'
    sample_id = source['segment_id'] + ':' + str(i)
    arrays = dict(rgb_mask=rgb_mask, q_features=q_features, sequence_features=sequence_features,
                  motion_target=target, label_mask=label_mask, future_rgb=future_rgb,
                  future_mask=future_mask, current_box=box.astype(np.float32), future_box=future_box,
                  crop_specs=np.asarray(spec, np.int32),
                  centers=(((box[:2]+box[2:])/2-np.asarray(spec[:2]))/spec[2]).astype(np.float32),
                  split=split, ids=sample_id, group=source['group'], stage=plan['stage'],
                  target_offsets=np.asarray(offsets, np.float64), label_offsets=label_offsets)
    detail = dict(id=sample_id, split=split, evaluation_role=source['suggested_split'],
                  group=source['group'], stage=plan['stage'], camera_role=role, camera_serial=SERIALS[role],
                  source=str(Path(source['path']).resolve()),
                  command_index=i, prediction_started_at=start, prediction_completed_at=start,
                  completion_clock='offline_zero_latency_convention_not_measured_inference',
                  clock='host_monotonic', q_current=q, q_observed_at=row['observation']['q_available_t'],
                  q_source_line=row['observation']['q_source_line'],
                  q_source_log=row['action']['source_log'],
                  q_source_sha256=source['source_paths_sha256'][row['action']['source_log']],
                  plan_source=str(Path(source['path']).resolve()/'plan.json'),
                  plan_sha256=source['source_paths_sha256'][str(Path(source['path']).resolve()/'plan.json')],
                  preexecution_source_plan=plan['source_plan'], source_plan_sha256=plan['source_plan_sha256'],
                  command_source_log=row['action']['source_log'],
                  command_source_sha256=source['source_paths_sha256'][row['action']['source_log']],
                  source_paths_sha256=source['source_paths_sha256'],
                  current_image={**current_frame, 'capture_age_estimate_s': age},
                  raw_planned_targets=raw_targets, commanded_joint_mask=[mask[:] for _ in range(STEPS)],
                  valid_steps=[True]*STEPS, future_mask=[False]+[True]*9,
                  target_times=[start+t for t in offsets], target_offsets=offsets,
                  actual_command_times=[r['command_host_monotonic'] for r in rows[i:i+STEPS]],
                  command_source_lines=[r['action']['source_line'] for r in rows[i:i+STEPS]],
                  units=UNITS, labels=label_details, current_box=box.tolist(), crop_spec=spec,
                  feature_source='saved_preexecution_plan_not_future_command_log',
                  label_source='HSV_largest_red_component_weak_supervision',
                  inactive_axis_semantics='None raw targets and zero masked features mean no update, not hold or zero physical motion.')
    return arrays, detail


def build_dataset(inventory_path, output, protocol_path):
    inventory_path, output = Path(inventory_path).resolve(), Path(output).resolve()
    protocol_path = Path(protocol_path).resolve()
    if output.exists():
        raise FileExistsError('output_must_be_new')
    initial_inventory_sha, initial_protocol_sha = sha(inventory_path), sha(protocol_path)
    protocol = verify_protocol(protocol_path, inventory_path)
    role = protocol['data']['camera_role']
    inventory, selected = verify_inventory(inventory_path)
    output.mkdir(parents=True, exist_ok=False)
    causal_dir = output/'causal'
    causal_dir.mkdir()
    exporter = load_module('sequence_offline_causal_exporter', EXPORTER)
    arrays, details, reports = [], [], []
    all_rejections, all_label_rejections = Counter(), Counter()
    candidate_counts, retained_counts = Counter(), Counter()
    for number, (source, plan) in enumerate(selected):
        folder = Path(source['path']).resolve()
        segment = exporter.export_stage(folder, causal_dir, folder.parent, {},
                                        folder.parent/'project_status.json', {},
                                        segment_name=f'{number:02d}_source',
                                        semantic_stage=plan['stage'], feedback_max_age=.6, camera_max_age=.25)
        rows = jsonlines(segment['samples_file'])
        refs = planned_references(plan['sdk_plan'])
        partial = plan_command_mask(plan)
        mask = partial if partial is not None else [True]*7
        grip = None if partial is not None else plan.get('requested_gripper_target_sdk')
        if partial is None and not finite(grip):
            raise ValueError('missing_preplanned_gripper_target')
        prefix = 0
        for row, (_, ref) in zip(rows, refs):
            expected = command_reference(ref, grip, partial)
            actual = row['action']['sdk_target']
            if (row['action']['commanded_joint_mask'] != mask or len(actual) != 7
                    or any((a is None) != (b is None) or
                           (a is not None and (not finite(a) or abs(a-b) > 1e-4))
                           for a, b in zip(actual, expected))):
                break
            prefix += 1
        if (prefix != source['verified_executed_plan_prefix_commands']
                or len(rows) != source['command_count']):
            raise ValueError('inventory_prefix_or_command_count_mismatch')
        rows = rows[:prefix]  # No labels/anchors can use an unverified executed tail.
        frames, frame_rejections = load_frames(folder, role=role)
        rejection, label_rejection = Counter(), Counter()
        count = 0
        cache = OrderedDict()
        anchors = range(0, max(0, prefix-STEPS+1), STRIDE)
        canonical_split = 'train' if source['suggested_split'] == 'train' else 'val'
        candidate_counts[canonical_split] += len(anchors)
        for i in anchors:
            try:
                data, detail = make_sample(source, plan, rows, refs, mask, i, frames, cache, label_rejection, role=role)
            except ValueError as error:
                if str(error) == 'image_hash_mismatch':
                    raise
                rejection[str(error)] += 1
                continue
            arrays.append(data)
            details.append(detail)
            count += 1
            retained_counts[canonical_split] += 1
        all_rejections.update(rejection)
        all_label_rejections.update(label_rejection)
        reports.append(dict(source=source['path'], group=source['group'],
                            split=source['suggested_split'], windows=count,
                            candidate_windows=len(anchors),
                            verified_prefix_commands=prefix, logged_commands=source['command_count'],
                            rejected_windows=dict(rejection), rejected_labels=dict(label_rejection),
                            rejected_frame_records=dict(frame_rejections),
                            causal_export=segment['samples_file'], causal_sha256=sha(segment['samples_file'])))
    diagnostics = dict(schema='preaction-sequence-build-diagnostics-v1',
                       inventory_path=str(inventory_path), inventory_sha256=initial_inventory_sha,
                       protocol_path=str(protocol_path), protocol_sha256=initial_protocol_sha,
                       camera_role=role, camera_serial=SERIALS[role],
                       sources=reports, candidate_windows_by_split=dict(candidate_counts),
                       retained_windows_by_split=dict(retained_counts),
                       rejected_windows=dict(all_rejections), rejected_labels=dict(all_label_rejections),
                       training_performed=False, hardware_accessed=False, robot_commands=0)
    if not arrays or {a['split'] for a in arrays} != {'train', 'val'}:
        diagnostics.update(status='FAILED', error='no_eligible_windows_in_both_train_and_development_validation')
        save_json(output/'build_diagnostics.json', diagnostics)
        raise ValueError('no_eligible_windows_in_both_train_and_development_validation')
    diagnostics['status'] = 'ELIGIBLE_WINDOWS_IN_BOTH_SPLITS'
    save_json(output/'build_diagnostics.json', diagnostics)
    packed = {key: np.asarray([a[key] for a in arrays]) for key in arrays[0]}
    # Exact image identity leakage check includes inputs and accepted labels.
    image_splits = defaultdict(set)
    for detail in details:
        image_splits[detail['current_image']['sha256']].add(detail['split'])
        for label in detail['labels']:
            if label['valid']:
                image_splits[label['image']['sha256']].add(detail['split'])
    if any(len(splits) > 1 for splits in image_splits.values()):
        raise ValueError('image_hash_crosses_split')
    # Refuse mutable raw evidence or a changed protocol during a long build.
    if sha(inventory_path) != initial_inventory_sha or sha(protocol_path) != initial_protocol_sha:
        raise ValueError('inventory_or_protocol_changed_during_build')
    verify_protocol(protocol_path, inventory_path)
    verify_inventory(inventory_path)
    with (output/'samples.jsonl').open('x') as stream:
        for detail in details:
            stream.write(json.dumps(detail, ensure_ascii=False, allow_nan=False)+'\n')
    np.savez_compressed(output/'dataset.npz', **packed)
    split_counts = Counter(packed['split'].tolist())
    group_hashes = {group: hashlib.sha256(json.dumps(
        sorted((s['path'], sorted(s['source_paths_sha256'].items()))
               for s, _ in selected if s['group'] == group), separators=(',', ':')).encode()).hexdigest()
                   for group in {s['group'] for s, _ in selected}}
    manifest = dict(schema=SCHEMA, status='OFFLINE_WEAK_SUPERVISION_PREPARED_NOT_TRAINED',
                    inventory_path=str(inventory_path), inventory_sha256=sha(inventory_path),
                    protocol_path=str(protocol_path), protocol_sha256=sha(protocol_path),
                    camera_role=role, camera_serial=SERIALS[role],
                    counts=dict(windows=len(arrays), split=dict(split_counts),
                                labels=int(packed['label_mask'].sum()), independent_test=0),
                    sources=reports, group_hashes=group_hashes,
                    excluded_sources=[s['path'] for s in inventory['sources'] if s['suggested_split']=='excluded'],
                    rejected_windows=dict(all_rejections), rejected_labels=dict(all_label_rejections),
                    steps=STEPS, rate_hz=RATE, anchor_stride=STRIDE, feature_columns=FEATURE_COLUMNS,
                    q_scale=Q_SCALE, delta_scale=DELTA_SCALE, motion_scale=PARAM_SCALE.tolist(),
                    units=UNITS, npz_fields={k: dict(shape=list(v.shape), dtype=str(v.dtype)) for k, v in packed.items()},
                    dataset_sha256=sha(output/'dataset.npz'), samples_sha256=sha(output/'samples.jsonl'),
                    dataset_path=str(output/'dataset.npz'), samples_path=str(output/'samples.jsonl'),
                    code_sha256={str(Path(__file__).resolve()): sha(__file__), str(EXPORTER): sha(EXPORTER)},
                    label_scope='HSV weak segmentation/boxes and recorded RGB, same current-frame ROI for all future labels; no success/contact/collision labels.',
                    clock_limits=dict(q_age_s=.6, current_capture_age_s=.25, board_frame_age_s=.25,
                                      plan_execution_timing_tolerance_s=.1, label_capture_tolerance_s=.12,
                                      label_step_zero=False, minimum_labels_per_window=5),
                    completion_convention='started_at == completed_at, historical offline training convention, not measured inference latency',
                    preexecution_evidence=inventory.get('preexecution_evidence'),
                    evaluation_scope='development_validation from a separate date; no independent test; dense windows within a source are correlated',
                    training_performed=False, hardware_accessed=False, robot_commands=0)
    save_json(output/'manifest.json', manifest)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inventory', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = build_dataset(args.inventory, args.output, args.protocol)
    print(json.dumps(dict(output=str(args.output.resolve()), counts=result['counts'],
                          dataset_sha256=result['dataset_sha256']), ensure_ascii=False))


if __name__ == '__main__':
    main()

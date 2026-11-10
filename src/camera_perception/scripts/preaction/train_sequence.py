"""CPU training for the frozen first PREACTION sequence experiment.

Use ``python -m scripts.preaction.train_sequence --data DATA --protocol JSON
--legacy WEIGHTS --output NEW_DIR --mode sequence|endpoint``. The old endpoint
head is never loaded. Both conditions use the same architecture, initialization,
data order and budget. Current images/state are inputs; matched future images
are masked supervision only. This module does not generate or execute actions.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import re
import time

import cv2
import numpy as np
import torch
from torch.nn import functional as F

from .core import warp_tensor
from .sequence_model import PreActionSequence, load_sequence_checkpoint, save_sequence_checkpoint


DATA_SCHEMA = 'preaction-sequence-dataset-v1'
PROTOCOL_SCHEMA = 'preaction-sequence-pilot-protocol-v1'
MODES = ('sequence', 'endpoint')
FIXED_TRAINING = dict(steps=300, batch_size=32, seed=42, eval_every=20,
                      learning_rate=.001, weight_decay=.0001, gradient_clip=5.,
                      threads=4, device='cpu', optimizer='AdamW',
                      loss=dict(parameter=1., rgb=.15, silhouette=.25))
ARRAY_SPEC = {
    'rgb_mask': ((4, 96, 96), np.dtype('float32')),
    'q_features': ((7,), np.dtype('float32')),
    'sequence_features': ((10, 17), np.dtype('float32')),
    'motion_target': ((10, 4), np.dtype('float32')),
    'label_mask': ((10,), np.dtype('bool')),
    'future_rgb': ((10, 3, 96, 96), np.dtype('uint8')),
    'future_mask': ((10, 96, 96), np.dtype('uint8')),
    'centers': ((2,), np.dtype('float32')),
    'current_box': ((4,), None), 'future_box': ((10, 4), None),
    'crop_specs': ((3,), None),
    'target_offsets': ((10,), np.dtype('float64')),
    'label_offsets': ((10,), np.dtype('float64')),
    'split': ((), 'unicode'), 'ids': ((), 'unicode'),
    'group': ((), 'unicode'), 'stage': ((), 'unicode'),
}


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _invalid_json_constant(value):
    raise ValueError('nonfinite_json_constant:' + value)


def _read_json(path):
    return json.loads(Path(path).read_text(), parse_constant=_invalid_json_constant)


def _hash(value, name):
    if not isinstance(value, str) or re.fullmatch('[0-9a-fA-F]{64}', value) is None:
        raise ValueError(name + ': expected SHA256')
    return value.lower()


def _binary(array, name):
    if not np.isin(array, [0, 1]).all():
        raise ValueError(name + ': expected binary values')


def load_dataset(data_dir):
    """Load hash-bound arrays and samples, returning ``(arrays, manifest)``.

    Images remain uint8 in RAM; training converts only a selected batch. This
    loader verifies shapes, numerical masks, sample joins and session splits.
    It does not reinterpret teacher padding or infer missing command targets.
    Source/clock reconstruction is the dataset builder's separately recorded
    responsibility. Only train/development-val data are accepted for this run.
    """
    root = Path(data_dir).resolve()
    manifest = _read_json(root / 'manifest.json')
    if manifest.get('schema') != DATA_SCHEMA:
        raise ValueError('unsupported_sequence_dataset_schema')
    paths = {'dataset': root / 'dataset.npz', 'samples': root / 'samples.jsonl'}
    for name, path in paths.items():
        declared_path = manifest.get(name + '_path')
        if declared_path is not None and Path(declared_path).resolve() != path:
            raise ValueError(name + '_path_mismatch')
        if sha(path) != _hash(manifest.get(name + '_sha256'), name + '_sha256'):
            raise ValueError(name + '_hash_mismatch')
    with np.load(paths['dataset'], allow_pickle=False) as stored:
        if set(stored.files) != set(ARRAY_SPEC):
            raise ValueError('dataset_array_fields_mismatch')
        arrays = {key: stored[key] for key in ARRAY_SPEC}
    count = len(arrays['ids'])
    if count < 2:
        raise ValueError('nonempty_train_and_validation_required')
    for name, (tail, dtype) in ARRAY_SPEC.items():
        array = arrays[name]
        if array.shape != (count, *tail):
            raise ValueError(name + '_shape_mismatch')
        if dtype == 'unicode':
            if array.dtype.kind != 'U' or any(not x.strip() for x in array.tolist()):
                raise ValueError(name + '_unicode_values_required')
        else:
            if dtype is not None and array.dtype != dtype:
                raise ValueError(name + '_dtype_mismatch')
            if array.dtype.kind not in 'fibu' or not np.isfinite(array).all():
                raise ValueError(name + '_finite_numeric_values_required')
    if len(set(arrays['ids'].tolist())) != count:
        raise ValueError('duplicate_dataset_ids')
    if set(arrays['split'].tolist()) != {'train', 'val'}:
        raise ValueError('only_nonempty_train_and_development_val_supported')
    group_splits = {}
    for group, split in zip(arrays['group'], arrays['split']):
        if group in group_splits and group_splits[group] != split:
            raise ValueError('group_crosses_split:' + group)
        group_splits[group] = split
    samples = []
    with paths['samples'].open() as source:
        for line in source:
            if line.strip():
                samples.append(json.loads(line, parse_constant=_invalid_json_constant))
    if len(samples) != count:
        raise ValueError('samples_count_mismatch')
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            raise ValueError('sample_mapping_required')
        for key, array_key in [('id', 'ids'), ('split', 'split'), ('group', 'group'), ('stage', 'stage')]:
            if sample.get(key) != arrays[array_key][index]:
                raise ValueError('sample_array_join_mismatch:' + key)
    seq = arrays['sequence_features']
    _binary(arrays['rgb_mask'][:, 3], 'current_mask')
    _binary(arrays['future_mask'], 'future_mask')
    _binary(seq[:, :, 7:14], 'commanded_joint_mask')
    _binary(seq[:, :, 15:17], 'valid_future_flags')
    if np.any((arrays['rgb_mask'][:, :3] < 0) | (arrays['rgb_mask'][:, :3] > 1)):
        raise ValueError('current_rgb_outside_0_1')
    if not np.array_equal(seq[:, :, 14], arrays['target_offsets'].astype(np.float32)):
        raise ValueError('target_offset_feature_mismatch')
    if np.any(np.diff(arrays['target_offsets'], axis=1) <= 0):
        raise ValueError('target_offsets_not_increasing')
    valid = seq[:, :, 15].astype(bool)
    future = seq[:, :, 16].astype(bool)
    axes = seq[:, :, 7:14].astype(bool)
    if np.any(seq[:, :, :7][~(valid[:, :, None] & axes)] != 0):
        raise ValueError('invalid_or_uncommanded_delta_not_zero')
    supervised = arrays['label_mask'] & valid & future
    if np.any(supervised.sum(axis=1) == 0):
        raise ValueError('sample_has_no_supervised_future_steps')
    if np.any(arrays['label_offsets'][supervised] <= 0):
        raise ValueError('supervised_label_is_not_future')
    if np.any(arrays['current_box'][:, 2:] <= arrays['current_box'][:, :2]):
        raise ValueError('invalid_current_box')
    if np.any(arrays['future_box'][supervised, 2:] <= arrays['future_box'][supervised, :2]):
        raise ValueError('invalid_supervised_future_box')
    if np.any(arrays['crop_specs'][:, 2] <= 0):
        raise ValueError('invalid_crop_size')
    counts = manifest.get('counts', {})
    expected_splits = {s: int(np.sum(arrays['split'] == s)) for s in ('train', 'val')}
    if (counts.get('windows') != count or counts.get('split') != expected_splits
            or counts.get('labels') != int(arrays['label_mask'].sum())
            or counts.get('independent_test') != 0):
        raise ValueError('manifest_counts_mismatch')
    return arrays, manifest


def endpoint_features(sequence_features):
    """Repeat last valid delta/axis mask; preserve time/valid/future columns.

    The capacity-matched endpoint condition has no intermediate action values.
    All-padding items use zero delta/mask. The caller's tensor is not modified.
    """
    if (not isinstance(sequence_features, torch.Tensor) or sequence_features.ndim != 3
            or sequence_features.shape[1:] != (10, 17)):
        raise ValueError('endpoint_features requires [B,10,17]')
    valid = sequence_features[:, :, 15]
    if not bool(((valid == 0) | (valid == 1)).all()):
        raise ValueError('endpoint valid flags must be binary')
    indices = torch.arange(10, device=sequence_features.device)[None].expand_as(valid)
    last = torch.where(valid.bool(), indices, -1).max(dim=1).values
    endpoint = sequence_features[torch.arange(len(last), device=last.device), last.clamp_min(0), :14]
    endpoint = torch.where((last >= 0)[:, None], endpoint, torch.zeros_like(endpoint))
    result = sequence_features.clone()
    result[:, :, :14] = endpoint[:, None]
    return result


def conditioning_features(sequence_features, mode):
    """Apply the declared conditioning mode without mutating source features."""
    if mode == 'sequence':
        return sequence_features
    if mode == 'endpoint':
        return endpoint_features(sequence_features)
    raise ValueError('unsupported_conditioning_mode')


def sequence_objective(model, rgb_mask, q_features, seq_features, motion_target,
                       label_mask, future_rgb_float01, future_mask_float01, centers,
                       mode='sequence'):
    """Return ``(loss, prediction[B,10,4], components)``.

    Supervision uses label_mask AND input-valid AND future-at-completion. Each
    valid step contributes equally: SmoothL1 averaged over 4 parameters, RGB L1
    normalized by that step's mask-union area, and silhouette L1 over its ROI.
    Components contain scalar tensors parameter/rgb/silhouette and an integer
    valid_steps. Missing labels, including their placeholder values, never enter
    any loss. The current ROI is the warp source for every horizon.
    """
    batch = rgb_mask.shape[0]
    expected = ((motion_target, (batch, 10, 4), 'motion_target'),
                (label_mask, (batch, 10), 'label_mask'),
                (future_rgb_float01, (batch, 10, 3, 96, 96), 'future_rgb'),
                (future_mask_float01, (batch, 10, 96, 96), 'future_mask'),
                (centers, (batch, 2), 'centers'))
    for tensor, shape, name in expected:
        if not isinstance(tensor, torch.Tensor) or tensor.shape != shape:
            raise ValueError(name + '_shape_mismatch')
        if tensor.device != rgb_mask.device:
            raise ValueError(name + '_device_mismatch')
        if name == 'label_mask':
            if tensor.dtype != torch.bool:
                raise ValueError('label_mask_must_be_bool')
        elif tensor.dtype != rgb_mask.dtype:
            raise ValueError(name + '_dtype_mismatch')
    result = model(rgb_mask, q_features, conditioning_features(seq_features, mode))
    prediction = result['motion']
    mask = label_mask & result['valid_steps'] & result['future_mask']
    n_valid = int(mask.sum().item())
    if n_valid == 0:
        raise ValueError('no_supervised_future_steps')
    rows, steps = torch.where(mask)
    pred = prediction[rows, steps]
    target = motion_target[rows, steps]
    future_rgb = future_rgb_float01[rows, steps]
    future_mask = future_mask_float01[rows, steps]
    selected_centers = centers[rows]
    for values in (pred, target, future_rgb, future_mask, selected_centers):
        if not bool(torch.isfinite(values).all()):
            raise ValueError('nonfinite_supervised_values')
    if (not bool(((future_mask == 0) | (future_mask == 1)).all())
            or bool(((future_rgb < 0) | (future_rgb > 1)).any())):
        raise ValueError('invalid_supervised_rgb_or_mask_range')
    parameter = F.smooth_l1_loss(pred, target, reduction='none').mean(dim=1).mean()
    current = rgb_mask[rows]
    warped = warp_tensor(current, selected_centers, pred)
    union = torch.maximum(current[:, 3], future_mask)
    rgb_per_step = ((warped[:, :3] - future_rgb).abs() * union[:, None]).sum(dim=(1, 2, 3))
    rgb_per_step = rgb_per_step / (union.sum(dim=(1, 2)) * 3 + 1e-6)
    rgb = rgb_per_step.mean()
    silhouette = (warped[:, 3] - future_mask).abs().mean(dim=(1, 2)).mean()
    loss = parameter + .15 * rgb + .25 * silhouette
    return loss, prediction, dict(parameter=parameter, rgb=rgb, silhouette=silhouette,
                                  valid_steps=n_valid)


def batch_tensors(arrays, indices):
    """Materialize CPU float32 future images for this batch only."""
    return (torch.from_numpy(arrays['rgb_mask'][indices]),
            torch.from_numpy(arrays['q_features'][indices]),
            torch.from_numpy(arrays['sequence_features'][indices]),
            torch.from_numpy(arrays['motion_target'][indices]),
            torch.from_numpy(arrays['label_mask'][indices]),
            torch.from_numpy(arrays['future_rgb'][indices]).float().div_(255.),
            torch.from_numpy(arrays['future_mask'][indices]).float(),
            torch.from_numpy(arrays['centers'][indices]))


def evaluate_objective(model, arrays, indices, *, mode, batch_size=32):
    """Aggregate batch means by supervised step count, not batch/window count."""
    model.eval()
    total = 0
    sums = dict(objective=0., parameter=0., rgb=0., silhouette=0.)
    with torch.inference_mode():
        for lo in range(0, len(indices), batch_size):
            loss, _, parts = sequence_objective(model, *batch_tensors(arrays, indices[lo:lo + batch_size]), mode=mode)
            count = parts['valid_steps']
            total += count
            sums['objective'] += float(loss) * count
            for key in ('parameter', 'rgb', 'silhouette'):
                sums[key] += float(parts[key]) * count
    if total == 0:
        raise ValueError('empty_evaluation')
    return {**{k: value / total for k, value in sums.items()}, 'valid_steps': total,
            'windows': len(indices)}


def _state_sha(state):
    digest = hashlib.sha256()
    for name, tensor in sorted(state.items()):
        array = tensor.detach().cpu().contiguous().numpy()
        digest.update(name.encode()); digest.update(str(array.shape).encode())
        digest.update(str(array.dtype).encode()); digest.update(array.tobytes())
    return digest.hexdigest()


def _validate_protocol(protocol, legacy):
    if protocol.get('schema') != PROTOCOL_SCHEMA:
        raise ValueError('unsupported_training_protocol')
    settings = protocol.get('training', {})
    for key, expected in FIXED_TRAINING.items():
        if settings.get(key) != expected or (type(expected) is int and type(settings.get(key)) is not int):
            raise ValueError('frozen_training_parameter_mismatch:' + key)
    if sha(legacy) != _hash(protocol.get('legacy_checkpoint_sha256'), 'legacy_checkpoint_sha256'):
        raise ValueError('legacy_checkpoint_hash_mismatch')
    if set(protocol.get('modes', [])) != set(MODES):
        raise ValueError('protocol_conditioning_modes_mismatch')
    return dict(settings)


def load_trained_sequence(checkpoint, *, conditioning_mode, data_hash=None,
                          protocol_hash=None, legacy_hash=None):
    """Load a trained candidate only after explicit conditioning/binding checks.

    Evaluation must still call conditioning_features (or sequence_objective)
    with the same mode. Loading weights alone cannot apply input conditioning.
    """
    if conditioning_mode not in MODES:
        raise ValueError('unsupported_conditioning_mode')
    model = load_sequence_checkpoint(checkpoint)
    metadata = model.training_metadata
    if (metadata.get('conditioning_mode') != conditioning_mode
            or model.training_status != 'trained_candidate_unvalidated'
            or metadata.get('training_performed') is not True):
        raise ValueError('checkpoint_conditioning_mode_or_training_status_mismatch')
    if type(metadata.get('best_step')) is not int or metadata['best_step'] <= 0:
        raise ValueError('checkpoint_requires_a_trained_best_step')
    if not model.legacy_vision_loaded:
        raise ValueError('checkpoint_missing_loaded_legacy_vision')
    for key, expected in [('data_manifest_sha256', data_hash), ('protocol_sha256', protocol_hash),
                          ('legacy_checkpoint_sha256', legacy_hash)]:
        actual = _hash(metadata.get(key), key)
        if expected is not None and actual != _hash(expected, key + '_expected'):
            raise ValueError('checkpoint_binding_mismatch:' + key)
    return model


def train(data_dir, protocol_path, legacy_path, output, *, mode):
    """Run exactly the frozen protocol; the CLI exposes no budget overrides."""
    if mode not in MODES:
        raise ValueError('unsupported_conditioning_mode')
    root = Path(data_dir).resolve()
    protocol_path, legacy_path, out = (Path(p).resolve() for p in (protocol_path, legacy_path, output))
    if out.exists():
        raise FileExistsError(out)
    protocol = _read_json(protocol_path)
    settings = _validate_protocol(protocol, legacy_path)
    arrays, manifest = load_dataset(root)
    data_hash, protocol_hash, legacy_hash = sha(root / 'manifest.json'), sha(protocol_path), sha(legacy_path)
    if manifest.get('protocol_sha256') != protocol_hash:
        raise ValueError('dataset_protocol_hash_mismatch')
    torch.set_num_threads(settings['threads']); cv2.setNumThreads(1)
    torch.manual_seed(settings['seed']); np.random.seed(settings['seed']); random.seed(settings['seed'])
    torch.use_deterministic_algorithms(True)
    model = PreActionSequence().cpu()
    model.load_legacy_vision(torch.load(legacy_path, weights_only=True, map_location='cpu'))
    torch.nn.init.zeros_(model.motion_head.weight); torch.nn.init.zeros_(model.motion_head.bias)
    initial_hash = _state_sha(model.state_dict())
    vision_hash = _state_sha(model.vision.state_dict())
    train_indices = np.flatnonzero(arrays['split'] == 'train')
    val_indices = np.flatnonzero(arrays['split'] == 'val')
    rng = np.random.default_rng(settings['seed'])
    out.mkdir(parents=True, exist_ok=False)
    config = dict(schema='preaction-sequence-training-config-v1', conditioning_mode=mode,
                  data_dir=str(root), data_manifest_sha256=data_hash,
                  dataset_sha256=manifest['dataset_sha256'], samples_sha256=manifest['samples_sha256'],
                  protocol_path=str(protocol_path), protocol_sha256=protocol_hash,
                  legacy_checkpoint=str(legacy_path), legacy_checkpoint_sha256=legacy_hash,
                  training=settings, hidden_dim=model.hidden_dim, initial_state_sha256=initial_hash,
                  frozen_vision_sha256=vision_hash, motion_head_initialization='zero_weight_and_bias',
                  model_parameters=sum(p.numel() for p in model.parameters()),
                  trainable_parameters=sum(p.numel() for p in model.parameters() if p.requires_grad),
                  train_windows=len(train_indices), val_windows=len(val_indices), test_windows=0,
                  evaluation_status='development_validation', trained_device='cpu',
                  source_sha256={str(p): sha(p) for p in (Path(__file__), Path(__file__).with_name('sequence_model.py'), Path(__file__).with_name('core.py'))},
                  old_endpoint_head_loaded=False, robot_commands=0, hardware_accessed=False,
                  live_eligible=False, promotion=False)
    (out / 'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    initial = evaluate_objective(model, arrays, val_indices, mode=mode, batch_size=settings['batch_size'])
    baseline = dict(step=0, trained=False, validation=initial,
                    prediction='zero motion; same current-image persistence for every valid horizon',
                    initial_state_sha256=initial_hash)
    (out / 'initial_validation.json').write_text(json.dumps(baseline, indent=2, allow_nan=False) + '\n')
    history = [baseline]
    (out / 'learning_curve.json').write_text(json.dumps(history, indent=2, allow_nan=False) + '\n')
    optimizer = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad],
                                 lr=settings['learning_rate'], weight_decay=settings['weight_decay'])
    best_value, best_step, best_state, best_validation = math.inf, None, None, None
    started = time.perf_counter()
    for step in range(1, settings['steps'] + 1):
        model.train()
        ix = rng.choice(train_indices, size=min(settings['batch_size'], len(train_indices)), replace=False)
        loss, _, parts = sequence_objective(model, *batch_tensors(arrays, ix), mode=mode)
        if not bool(torch.isfinite(loss)):
            raise ValueError('nonfinite_training_loss')
        optimizer.zero_grad(set_to_none=True); loss.backward()
        norm = torch.nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad], settings['gradient_clip'])
        if not bool(torch.isfinite(norm)):
            raise ValueError('nonfinite_training_gradient')
        optimizer.step()
        if step % settings['eval_every'] == 0 or step == settings['steps']:
            validation = evaluate_objective(model, arrays, val_indices, mode=mode, batch_size=settings['batch_size'])
            record = dict(step=step, trained=True, train_batch_objective=float(loss.detach()),
                          train_batch_components={k: (v if isinstance(v, int) else float(v.detach())) for k, v in parts.items()},
                          validation=validation)
            history.append(record)
            (out / 'learning_curve.json').write_text(json.dumps(history, indent=2, allow_nan=False) + '\n')
            if validation['objective'] < best_value:
                best_value, best_step, best_validation = validation['objective'], step, validation
                best_state = {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}
            print(json.dumps(record, allow_nan=False), flush=True)
    if best_state is None or best_step is None or best_step <= 0:
        raise ValueError('no_trained_checkpoint_selected')
    model.load_state_dict(best_state); model.eval()
    if _state_sha(model.vision.state_dict()) != vision_hash:
        raise ValueError('frozen_visual_encoder_changed')
    metadata = dict(training_performed=True, conditioning_mode=mode, best_step=best_step,
                    steps_run=settings['steps'], data_manifest_sha256=data_hash,
                    dataset_sha256=manifest['dataset_sha256'], samples_sha256=manifest['samples_sha256'],
                    protocol_sha256=protocol_hash, legacy_checkpoint_sha256=legacy_hash,
                    initial_state_sha256=initial_hash, frozen_vision_sha256=vision_hash,
                    validation_objective=best_value, evaluation_status='development_validation',
                    independent_test=False, selection='minimum objective among trained validation points',
                    optimizer='AdamW', device='cpu', robot_commands=0, live_eligible=False)
    save_sequence_checkpoint(model, out / 'best.pt', training_metadata=metadata)
    reloaded = load_trained_sequence(out / 'best.pt', conditioning_mode=mode, data_hash=data_hash,
                                    protocol_hash=protocol_hash, legacy_hash=legacy_hash)
    reload_validation = evaluate_objective(reloaded, arrays, val_indices, mode=mode, batch_size=settings['batch_size'])
    if reload_validation != best_validation:
        raise ValueError('checkpoint_reload_validation_mismatch')
    summary = dict(schema='preaction-sequence-training-result-v1', status='TRAINED_CANDIDATE_OFFLINE',
                   conditioning_mode=mode, best_step=best_step, steps_run=settings['steps'],
                   initial_validation=initial, best_validation=best_validation,
                   improved_over_persistence_objective=best_value < initial['objective'],
                   validation_after_reload=reload_validation, checkpoint_sha256=sha(out / 'best.pt'),
                   data_manifest_sha256=data_hash, protocol_sha256=protocol_hash,
                   legacy_checkpoint_sha256=legacy_hash, initial_state_sha256=initial_hash,
                   evaluation_status='development_validation', independent_test=False,
                   elapsed_s=time.perf_counter() - started, robot_commands=0,
                   hardware_accessed=False, live_eligible=False, promotion=False)
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', required=True, type=Path)
    parser.add_argument('--protocol', required=True, type=Path)
    parser.add_argument('--legacy', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--mode', required=True, choices=MODES)
    args = parser.parse_args()
    print(json.dumps(train(args.data, args.protocol, args.legacy, args.output, mode=args.mode), allow_nan=False))


if __name__ == '__main__':
    main()

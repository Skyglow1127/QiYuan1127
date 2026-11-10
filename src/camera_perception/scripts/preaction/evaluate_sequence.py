"""Offline, matched-label evaluation of the first sequence forecast candidates.

Scores use HSV weak labels on visible eligible frames. They are not independent
grasp outcomes, calibrated geometry errors, or evidence of live readiness.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch

from .core import PARAM_SCALE, warp_tensor
from .train_sequence import conditioning_features, load_dataset, load_trained_sequence


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def boxes_from_motion(current_box, crop_specs, predictions):
    u = predictions / PARAM_SCALE
    center = (current_box[:, :2] + current_box[:, 2:])[:, None] / 2
    center = center + u[:, :, :2] * crop_specs[:, None, 2:3]
    size = (current_box[:, 2:] - current_box[:, :2])[:, None]
    size = size * np.exp(np.clip(u[:, :, 2:], -.7, .7))
    return np.concatenate((center - size / 2, center + size / 2), axis=-1)


def metric_arrays(data, predictions, batch_size=32):
    """Return per-window/per-step metrics; masking occurs only in aggregation."""
    if predictions.shape != data['motion_target'].shape or not np.isfinite(predictions).all():
        raise ValueError('prediction_shape_or_finiteness')
    predicted = boxes_from_motion(data['current_box'], data['crop_specs'], predictions)
    target = data['future_box']
    center_error = np.linalg.norm((predicted[:, :, :2] + predicted[:, :, 2:]
                                  - target[:, :, :2] - target[:, :, 2:]) / 2, axis=-1)
    intersection = np.maximum(0, np.minimum(predicted[:, :, 2:], target[:, :, 2:])
                              - np.maximum(predicted[:, :, :2], target[:, :, :2])).prod(axis=-1)
    area = np.maximum(0, predicted[:, :, 2:] - predicted[:, :, :2]).prod(axis=-1)
    target_area = np.maximum(0, target[:, :, 2:] - target[:, :, :2]).prod(axis=-1)
    bbox_iou = intersection / np.maximum(area + target_area - intersection, 1e-9)
    rgb_l1 = np.zeros_like(center_error)
    mask_iou = np.zeros_like(center_error)
    for start in range(0, len(predictions), batch_size):
        end = min(start + batch_size, len(predictions)); count = end - start
        x = torch.from_numpy(data['rgb_mask'][start:end]).repeat_interleave(10, dim=0)
        centers = torch.from_numpy(data['centers'][start:end]).repeat_interleave(10, dim=0)
        with torch.no_grad():
            warped = warp_tensor(x, centers, torch.from_numpy(predictions[start:end].reshape(-1, 4)))
        future_rgb = torch.from_numpy(data['future_rgb'][start:end].reshape(-1, 3, 96, 96)).float() / 255
        future_mask = torch.from_numpy(data['future_mask'][start:end].reshape(-1, 1, 96, 96)).float()
        region = torch.maximum(x[:, 3:4], future_mask)
        error = ((warped[:, :3] - future_rgb).abs() * region).sum(dim=(1, 2, 3))
        error /= (region.sum(dim=(1, 2, 3)) * 3).clamp_min(1)
        a = warped[:, 3:4] >= .5; b = future_mask >= .5
        overlap = (a & b).sum(dim=(1, 2, 3)) / (a | b).sum(dim=(1, 2, 3)).clamp_min(1)
        rgb_l1[start:end] = error.numpy().reshape(count, 10)
        mask_iou[start:end] = overlap.numpy().reshape(count, 10)
    return dict(center_error_px=center_error, bbox_iou=bbox_iou,
                roi_rgb_l1=rgb_l1, roi_mask_iou=mask_iou,
                scaled_parameter_mae=np.abs(predictions - data['motion_target']).mean(axis=-1),
                render_scale_clipped=np.any(np.abs(predictions[:, :, 2:] / PARAM_SCALE[2:]) > .7, axis=-1))


def summarize(metrics, mask):
    mask = np.asarray(mask, dtype=bool)
    count = int(mask.sum())
    result = dict(labeled_steps=count, windows=int(mask.any(axis=1).sum()))
    for key, values in metrics.items():
        selected = values[mask]
        if count and not np.isfinite(selected).all():
            raise ValueError('nonfinite_labeled_metric:' + key)
        result[key] = float(selected.mean()) if count else None
    if count:
        result['center_error_p95_px'] = float(np.percentile(metrics['center_error_px'][mask], 95))
    return result


def predict(model, data, mode, *, ablation=None, batch_size=32):
    parts = []
    for start in range(0, len(data['ids']), batch_size):
        sl = slice(start, start + batch_size)
        seq = torch.from_numpy(data['sequence_features'][sl]).clone()
        if ablation == 'zero_action_delta':
            seq[:, :, :7] = 0
        elif ablation == 'reverse_middle_action_order_fixed_endpoints':
            seq[:, 1:9, :14] = seq[:, 1:9, :14].flip(1)
        elif ablation is not None:
            raise ValueError('unknown_ablation')
        with torch.no_grad():
            result = model(torch.from_numpy(data['rgb_mask'][sl]),
                           torch.from_numpy(data['q_features'][sl]),
                           conditioning_features(seq, mode))
        parts.append(result['motion'].numpy())
    return np.concatenate(parts)


def plot_results(out, data, predictions, reports):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(10, 3.8), constrained_layout=True)
    for name in ('persistence', 'endpoint', 'sequence'):
        rows = reports[name]['by_horizon']['val']
        x = [r['nominal_seconds'] for r in rows]
        axes[0].plot(x, [r['center_error_px'] for r in rows], marker='o', label=name)
        axes[1].plot(x, [r['bbox_iou'] for r in rows], marker='o', label=name)
    axes[0].set_ylabel('Object center error (image pixels)')
    axes[1].set_ylabel('Weak target box IoU')
    for axis in axes:
        axis.set_xlabel('Scheduled horizon (seconds)'); axis.grid(alpha=.25); axis.legend()
    fig.suptitle('Development validation: one historical lift session; HSV weak labels')
    fig.savefig(out / 'validation_by_horizon.png', dpi=160); plt.close(fig)

    # Fixed first/middle/last validation rows; no selection based on model score.
    valid = np.flatnonzero((data['split'] == 'val') & data['label_mask'][:, 9])
    if not len(valid):
        return
    chosen = valid[np.unique(np.linspace(0, len(valid) - 1, min(3, len(valid))).astype(int))]
    titles = ['Current observation', 'Persistence', 'Endpoint', 'Full sequence', 'Actual future']
    fig, axes = plt.subplots(len(chosen), 5, figsize=(12, 2.8 * len(chosen)), squeeze=False)
    records = []
    for row, idx in enumerate(chosen):
        x = torch.from_numpy(data['rgb_mask'][idx:idx+1])
        center = torch.from_numpy(data['centers'][idx:idx+1])
        panels = [data['rgb_mask'][idx, :3].transpose(1, 2, 0)]
        for name in ('persistence', 'endpoint', 'sequence'):
            with torch.no_grad():
                warped = warp_tensor(x, center, torch.from_numpy(predictions[name][idx:idx+1, 9]))
            panels.append(np.clip(warped[0, :3].numpy().transpose(1, 2, 0), 0, 1))
        panels.append(data['future_rgb'][idx, 9].transpose(1, 2, 0) / 255)
        for col, panel in enumerate(panels):
            axes[row, col].imshow(panel); axes[row, col].set_xticks([]); axes[row, col].set_yticks([])
            if row == 0:
                axes[row, col].set_title(titles[col])
        axes[row, 0].set_ylabel(f'Window {idx}\n{data["label_offsets"][idx, 9]:.3f}s')
        records.append(dict(id=str(data['ids'][idx]), index=int(idx), step=9,
                            actual_label_offset_s=float(data['label_offsets'][idx, 9])))
    fig.suptitle('Fixed 1.8s examples; predicted panels warp current pixels only')
    fig.tight_layout(); fig.savefig(out / 'validation_examples.png', dpi=160); plt.close(fig)
    (out / 'example_selection.json').write_text(json.dumps(records, indent=2) + '\n')


def evaluate(data_dir, protocol_path, sequence_path, endpoint_path, output):
    data, manifest = load_dataset(data_dir)
    protocol = json.loads(Path(protocol_path).read_text())
    protocol_hash = sha(protocol_path)
    if manifest.get('protocol_sha256') != protocol_hash:
        raise ValueError('dataset_protocol_mismatch')
    out = Path(output).resolve(); out.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(protocol['training']['threads'])
    models = {mode: load_trained_sequence(path, conditioning_mode=mode,
                  data_hash=sha(Path(data_dir) / 'manifest.json'), protocol_hash=protocol_hash,
                  legacy_hash=protocol['legacy_checkpoint_sha256'])
              for mode, path in (('sequence', sequence_path), ('endpoint', endpoint_path))}
    predictions = {'persistence': np.zeros_like(data['motion_target'])}
    for name, model in models.items():
        predictions[name] = predict(model, data, name)
    for ablation in protocol['ablations']:
        predictions['sequence_' + ablation] = predict(models['sequence'], data, 'sequence', ablation=ablation)
    label_mask = data['label_mask'] & data['sequence_features'][:, :, 15].astype(bool) & data['sequence_features'][:, :, 16].astype(bool)
    reports = {}
    for name, pred in predictions.items():
        metrics = metric_arrays(data, pred)
        reports[name] = dict(by_split={}, by_group={}, by_stage={}, by_horizon={})
        for kind, field in (('by_split', 'split'), ('by_group', 'group'), ('by_stage', 'stage')):
            for value in np.unique(data[field]):
                mask = label_mask & (data[field] == value)[:, None]
                reports[name][kind][str(value)] = summarize(metrics, mask)
        for split in np.unique(data['split']):
            rows = []
            for k in range(1, 10):
                mask = np.zeros_like(label_mask)
                mask[:, k] = label_mask[:, k] & (data['split'] == split)
                item = summarize(metrics, mask)
                item.update(step=k, nominal_seconds=k / 5,
                            actual_label_seconds_mean=float(data['label_offsets'][mask].mean()) if mask.any() else None)
                rows.append(item)
            reports[name]['by_horizon'][str(split)] = rows
    valid_rows = np.flatnonzero(data['split'] == 'val')[:30]
    timings = []
    for repeat, idx in enumerate(np.resize(valid_rows, len(valid_rows) + 5)):
        x = torch.from_numpy(data['rgb_mask'][idx:idx+1]); q = torch.from_numpy(data['q_features'][idx:idx+1])
        seq = torch.from_numpy(data['sequence_features'][idx:idx+1])
        start = time.perf_counter()
        with torch.no_grad():
            models['sequence'](x, q, seq)
        elapsed = (time.perf_counter() - start) * 1000
        if repeat >= 5:
            timings.append(elapsed)
    np.savez_compressed(out / 'predictions.npz', **predictions, ids=data['ids'], label_mask=label_mask)
    plot_results(out, data, predictions, reports)
    result = dict(schema='preaction-sequence-evaluation-v1', status='OFFLINE_DEVELOPMENT_COMPARISON_COMPLETE',
                  reports=reports, split_counts={str(k): int((data['split'] == k).sum()) for k in np.unique(data['split'])},
                  validation_groups=np.unique(data['group'][data['split'] == 'val']).tolist(),
                  independent_test=False, validation_used_for_checkpoint_selection=True,
                  score_scope='Eligible visible frames; current-frame HSV ROI and future HSV weak targets. Not grasp outcomes.',
                  scale_render_rule='log-scale clamped to +/-0.7, same renderer and metrics for all models',
                  timing=dict(samples=len(timings), median_ms=float(np.median(timings)), p95_ms=float(np.percentile(timings, 95)),
                              scope='CPU B=1 forward only, inputs already prepared; excludes camera, decode, planning, communication, motor execution',
                              threads=protocol['training']['threads']),
                  source_sha256=dict(dataset=manifest['dataset_sha256'], data_manifest=sha(Path(data_dir) / 'manifest.json'), protocol=protocol_hash,
                    sequence=sha(sequence_path), endpoint=sha(endpoint_path)),
                  predictions_sha256=sha(out / 'predictions.npz'), robot_commands=0, hardware_accessed=False,
                  live_eligible=False)
    (out / 'evaluation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--sequence', type=Path, required=True)
    parser.add_argument('--endpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args.data, args.protocol, args.sequence, args.endpoint, args.output)
    print(json.dumps(dict(status=result['status'], validation={name: r['by_split']['val'] for name, r in result['reports'].items()},
                         timing=result['timing']), ensure_ascii=False))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Actual VPR3 packed DMA tests against frozen official pi05 float fixtures.

PS timings measure single-thread OpenCV on the same board. Frozen torch values
test image/tensor error only: no torch inference or policy-action equivalence.
"""
import argparse
import hashlib
import json
from pathlib import Path
from time import perf_counter_ns

import cv2
import numpy as np
from geometry import geometry, cpu, validate_pl
from transport import PL


def stats(values):
    return {'samples': len(values), 'p50_ms': float(np.median(values) / 1e6),
            'p95_ms': float(np.percentile(values, 95) / 1e6)}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fixture_cases(directory):
    manifest = json.loads((directory / 'manifest.json').read_text())
    records = manifest['records']
    if manifest.get('case_count') != 14 or len(records) != 14 or len(manifest.get('cases', [])) != 14:
        raise ValueError('VPR3 regression requires exactly 14 official fixtures')
    if not manifest.get('official_reference'):
        raise ValueError('Official pi05 reference provenance missing')
    cases = []
    names = set()
    for record, declared in zip(records, manifest['cases']):
        name = record['name']
        if not name or Path(name).name != name or name in names:
            raise ValueError('Invalid or duplicate fixture name')
        names.add(name)
        meta = record['geometry']
        g = geometry(meta['iw'], meta['ih'], meta['width'], meta['height'],
                     pad=meta['pad'], rounding=meta['rounding'])
        validate_pl(g)
        if declared != [name, g.iw, g.ih, g.rw, g.rh, g.width, g.height, g.pad]:
            raise ValueError('Fixture cases/records geometry mismatch: ' + name)
        for key in ('iw', 'ih', 'width', 'height', 'rw', 'rh', 'left', 'top', 'right', 'bottom', 'pad', 'rounding'):
            if getattr(g, key) != meta[key]:
                raise ValueError('Fixture geometry differs from runtime: ' + name)
        arrays = []
        for suffix, hash_key, dtype, shape in (
            ('input.rgb', 'input_sha256', np.uint8, (g.ih, g.iw, 3)),
            ('expected.rgb', 'expected_sha256', np.uint8, (g.height, g.width, 3)),
            ('expected.float32.rgb', 'float_sha256', np.dtype('<f4'), (g.height, g.width, 3)),
        ):
            path = directory / (name + '.' + suffix)
            if path.is_symlink() or digest(path) != record[hash_key]:
                raise ValueError('Fixture SHA256 mismatch: ' + str(path))
            arrays.append(np.fromfile(path, dtype=dtype).reshape(shape))
        image, expected, floating = arrays
        if not np.isfinite(floating).all() or floating.min() < 0 or floating.max() > 1:
            raise ValueError('Invalid official float fixture: ' + name)
        quantized = np.rint(floating * 255).clip(0, 255).astype(np.uint8)
        if not np.array_equal(quantized, expected):
            raise ValueError('Official float/quantized reference mismatch: ' + name)
        cases.append((name, image, g, expected, floating, record.get('origin')))
    return manifest, cases


def byte_error(actual, expected):
    delta = np.abs(actual.astype(np.int16) - expected.astype(np.int16))
    return {'max': int(delta.max()), 'mae': float(delta.mean()),
            'mismatched_bytes': int(np.count_nonzero(delta))}


def main(args):
    if not 1 <= args.repeats <= 1000:
        raise ValueError('repeats must be 1..1000')
    cv2.setNumThreads(1)
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'schema': 'vpr3-hardware-v1', 'status': 'RUNNING',
              'algorithm': 'Vitis Vision 2025.2 bilinear + pi05 floor geometry + packed RGB888',
              'frames': [], 'bypass': [], 'opencv': cv2.__version__, 'cpu_threads': 1,
              'rng_seed': 20261031, 'required_official_cases': 14,
              'required_packed_tail_cases': 3,
              'ps_baseline': 'board OpenCV uint8 resize/border; not torch or model inference',
              'model_action_regression': 'NOT_RUN',
              'quality_gates': {'quantized_max_bytes': 2, 'quantized_mae_bytes': 0.5,
                                'float_max_unit': 2.5001 / 255, 'float_mae_unit': 1 / 255,
                                'padding_exact': True},
              'timing_scope': 'same board input RGB in RAM -> owned output RGB; includes all input packing, coherent copy, DMA reset/submit/wait and output copy; excludes loading fixtures, JPEG decode, network and policy inference'}
    transport = None
    try:
        manifest, cases = fixture_cases(args.vectors)
        report['official_reference'] = manifest['official_reference']
        report['torch_version_of_frozen_reference'] = manifest.get('torch_version')
        report['fixture_manifest_sha256'] = digest(args.vectors / 'manifest.json')
        rng = np.random.default_rng(20261031)
        for width in (17, 18, 19):
            image = rng.integers(0, 256, (25, 33, 3), np.uint8)
            g = geometry(33, 25, width, 17)
            cases.append(('packed_tail_' + str((width * 17 * 3) % 4), image, g,
                          None, None, 'deterministic synthetic RGB; OpenCV tail-packet reference'))
        for path in args.jpeg:
            bgr = cv2.imread(str(path))
            if bgr is None:
                raise ValueError('Unable to decode JPEG: ' + str(path))
            image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            g = geometry(image.shape[1], image.shape[0])
            validate_pl(g)
            cases.append(('jpeg_' + path.stem, image, g, None, None, str(path)))
        transport = PL()
        for iw, ih in ((33, 25), (640, 480), (1920, 1080)):
            words = rng.integers(0, 2**32, (ih, iw), np.uint32)
            traces = []
            case = {'input': [iw, ih], 'frames': 0, 'compared_bytes': 0,
                    'max_error': 0, 'traces': traces}
            report['bypass'].append(case)
            for _ in range(args.repeats):
                actual, trace = transport.bypass(words)
                traces.append(trace)
                if not np.array_equal(actual, words):
                    raise RuntimeError('Raw RGBX32 bypass mismatch')
                case['frames'] += 1
                case['compared_bytes'] += words.nbytes
        for name, image, g, official_u8, official_float, origin in cases:
            reference, _ = cpu(image, g)
            traces, software, software_cpu, errors = [], [], [], []
            row = {'sample': name, 'origin': origin, 'geometry': g.metadata(),
                   'input_rgb_sha256': hashlib.sha256(image.tobytes()).hexdigest(),
                   'reference_kind': 'frozen_official_pi05_float' if official_float is not None else 'opencv_only',
                   'repeats': args.repeats, 'completed_repeats': 0, 'error': errors,
                   'traces': traces, 'ps_traces_ns': software, 'ps_cpu_traces_ns': software_cpu}
            report['frames'].append(row)
            for _ in range(3):
                transport.run(image, g)
                cpu(image, g)
            for trial in range(args.repeats):
                if trial % 2 == 0:
                    begin = perf_counter_ns()
                    _, sw_trace = cpu(image, g)
                    software.append(perf_counter_ns() - begin)
                    out, trace = transport.run(image, g)
                else:
                    out, trace = transport.run(image, g)
                    begin = perf_counter_ns()
                    _, sw_trace = cpu(image, g)
                    software.append(perf_counter_ns() - begin)
                traces.append(trace)
                software_cpu.append(sw_trace['cpu_time_ns'])
                error = {'opencv_u8': byte_error(out, reference)}
                mask = np.ones((g.height, g.width), bool)
                mask[g.top:g.top+g.rh, g.left:g.left+g.rw] = False
                error['padding_exact'] = bool(np.all(out[mask] == g.pad))
                errors.append(error)
                if official_float is not None:
                    error['official_quantized_u8'] = byte_error(out, official_u8)
                    delta = np.abs(out.astype(np.float32) / 255 - official_float)
                    error['official_float_unit'] = {'max': float(delta.max()), 'mae': float(delta.mean())}
                    error['normalized_minus1_plus1'] = {'max': 2 * float(delta.max()), 'mae': 2 * float(delta.mean())}
                    row['opencv_vs_official_quantized_u8'] = byte_error(reference, official_u8)
                    if delta.max() > 2.5001 / 255 or delta.mean() > 1 / 255:
                        raise RuntimeError('Official pi05 float image/tensor quality gate failed: ' + name)
                chosen = error.get('official_quantized_u8', error['opencv_u8'])
                if chosen['max'] > 2 or chosen['mae'] > 0.5:
                    raise RuntimeError('Quantized bilinear quality gate failed: ' + name)
                if not error['padding_exact']:
                    raise RuntimeError('Padding mismatch: ' + name)
                row['completed_repeats'] += 1
            out.tofile(args.output / (name + '.rgb'))
            row.update(pl=stats([t['preprocess_ns'] for t in traces]), ps=stats(software),
                       ps_cpu_time=stats(software_cpu),
                       pl_cpu_time=stats([t['cpu_time_ns'] for t in traces]),
                       ps_over_pl_speedup=float(np.median(software) / np.median([t['preprocess_ns'] for t in traces])),
                       sha256=hashlib.sha256(out.tobytes()).hexdigest())
        report['status'] = 'VPR3_HARDWARE_PASS'
        print('VPR3_HARDWARE_PASS', len(report['frames']), 'cases', flush=True)
    except BaseException as error:
        report.update(status='FAIL', error=str(error))
        raise
    finally:
        try:
            if transport:
                transport.close()
        finally:
            (args.output / 'hardware_results.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vectors', type=Path, required=True)
    parser.add_argument('--jpeg', type=Path, action='append', default=[])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=20)
    main(parser.parse_args())

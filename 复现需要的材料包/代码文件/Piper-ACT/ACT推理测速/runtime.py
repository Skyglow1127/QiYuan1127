#!/usr/bin/env python3
"""Recorded ACT inference benchmark, local CLI or loopback HTTP server.

No robot, camera or actuator code. Both cameras retain their 480 x 640 inputs.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, HTTPServer
import importlib.metadata
import io
import json
from pathlib import Path
import socket
import time

import numpy as np
import torch

IMAGE_KEYS = ('observation.images.global', 'observation.images.wrist')
INPUT_KEYS = ('observation.state', *IMAGE_KEYS)


def stats(values):
    array = np.asarray(values, dtype=np.float64)
    return {'count': len(values), 'median_seconds': float(np.median(array)),
            'min_seconds': float(np.min(array)), 'max_seconds': float(np.max(array)),
            'p95_seconds': float(np.percentile(array, 95))}


def read_npz(payload):
    with np.load(io.BytesIO(payload), allow_pickle=False) as archive:
        mode = str(archive['input_mode'].item()) if 'input_mode' in archive else 'original_float_npz'
        if mode not in ('original_float_npz', 'fullres_uint8_npz'):
            raise ValueError(f'Unknown encoding: {mode}')
        sample = {key: archive[key].copy() for key in INPUT_KEYS}
        source = {key: archive[key].item() for key in
                  ('source_episode', 'source_frame', 'source_relative_time_s',
                   'source_timestamp_kind', 'episode_index', 'frame_index') if key in archive}
    for key, value in sample.items():
        expected = (3, 480, 640) if key in IMAGE_KEYS else (7,)
        if value.shape != expected:
            raise ValueError(f'Unexpected shape {key}: {value.shape}')
        if key in IMAGE_KEYS and mode == 'fullres_uint8_npz':
            if value.dtype != np.uint8:
                raise ValueError(f'Expected uint8: {key}')
            sample[key] = value.astype(np.float32) / np.float32(255)
        elif value.dtype != np.float32:
            raise ValueError(f'Expected float32: {key}, got {value.dtype}')
        if not np.isfinite(sample[key]).all():
            raise ValueError(f'Nonfinite input: {key}')
    return {key: torch.from_numpy(value) for key, value in sample.items()}, mode, source


class Runtime:
    def __init__(self, model_path, device, threads):
        from safetensors.torch import load_file
        # Import before config parsing, to register the ACT type with Draccus.
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.configs import PreTrainedConfig
        from lerobot.policies import make_pre_post_processors

        if importlib.metadata.version('lerobot') != '0.6.1':
            raise RuntimeError('This benchmark requires lerobot==0.6.1.')
        if device == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('Requested GPU is unavailable.')
        torch.set_num_threads(threads)
        torch.set_float32_matmul_precision('highest')
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cudnn.benchmark = False
        self.device = device
        load_start = time.perf_counter()
        cfg = PreTrainedConfig.from_pretrained(str(model_path), local_files_only=True)
        if cfg.type != 'act' or cfg.chunk_size != 100 or cfg.use_peft:
            raise RuntimeError('Expected the final non-PEFT ACT checkpoint with chunk_size=100.')
        cfg.device, cfg.use_amp, cfg.push_to_hub = device, False, False
        # The saved checkpoint contains the trained backbone. Downloading an
        # ImageNet initialization would be unnecessary and less reproducible.
        cfg.pretrained_backbone_weights = None
        self.cfg = cfg
        self.policy = ACTPolicy(cfg)
        saved = load_file(str(model_path / 'model.safetensors'), device='cpu')
        initialized = self.policy.state_dict()
        if set(saved) != set(initialized):
            raise RuntimeError(f'Saved keys differ: missing={sorted(set(initialized)-set(saved))}, '
                               f'unexpected={sorted(set(saved)-set(initialized))}')
        for key, tensor in saved.items():
            if tensor.shape != initialized[key].shape or tensor.dtype != initialized[key].dtype:
                raise RuntimeError(f'Checkpoint shape/dtype mismatch: {key}')
            if tensor.is_floating_point() and not torch.isfinite(tensor).all().item():
                raise RuntimeError(f'Nonfinite checkpoint tensor: {key}')
        self.policy.load_state_dict(saved, strict=True)
        # Exact value audit once on CPU, outside all prediction timers.
        for key, tensor in self.policy.state_dict().items():
            if not torch.equal(tensor, saved[key]):
                raise RuntimeError(f'Loaded tensor differs from checkpoint: {key}')
        verified = len(saved)
        del saved, initialized
        self.policy.to(device=device, dtype=torch.float32).eval().requires_grad_(False)
        self.pre, self.post = make_pre_post_processors(
            policy_cfg=cfg, pretrained_path=str(model_path),
            preprocessor_overrides={'device_processor': {'device': device},
                                    'normalizer_processor': {'device': device}},
            postprocessor_overrides={'device_processor': {'device': 'cpu'},
                                     'unnormalizer_processor': {'device': device}})
        self.sync()
        self.metadata = {
            'ready_utc': datetime.now(timezone.utc).isoformat(), 'robot_io': False,
            'model': str(model_path.resolve()), 'device': device,
            'accelerator': torch.cuda.get_device_name() if device == 'cuda' else None,
            'threads': threads, 'batch_size': 1, 'action_shape': [100, 7],
            'camera_shapes': {key: [3, 480, 640] for key in IMAGE_KEYS},
            'dtype': 'float32', 'amp': False, 'tf32': False,
            'weights_none_for_backbone_init': True,
            'strict_checkpoint_keys_shapes_dtypes_values_verified': verified,
            'load_seconds': time.perf_counter() - load_start,
            'versions': {key: importlib.metadata.version(key)
                         for key in ('torch', 'torchvision', 'lerobot', 'numpy')},
            'source_capture_wall_time': None,
            'timing_note': 'Model timer is synchronized predict_action_chunk, not cached '
                           'select_action. Total includes preprocessor, full model, postprocessor '
                           'CPU transfer and finite/shape checks. Disk read, input NPZ decode and '
                           'action JSON serialization are excluded from that total.'}

    def sync(self):
        if self.device == 'cuda':
            torch.cuda.synchronize()

    def infer(self, sample):
        self.policy.reset()
        self.sync()
        total_start = time.perf_counter()
        with torch.inference_mode():
            batch = self.pre(sample)
            self.sync()
            model_start = time.perf_counter()
            prediction = self.policy.predict_action_chunk(batch)
            self.sync()
            model_end = time.perf_counter()
            actions = self.post(prediction).detach().cpu()
            self.sync()
        post_end = time.perf_counter()
        if tuple(actions.shape) != (1, 100, 7) or not torch.isfinite(actions).all().item():
            raise RuntimeError(f'Invalid ACT actions: {tuple(actions.shape)}')
        validated = time.perf_counter()
        return actions[0], {'preprocess': model_start - total_start,
                            'model': model_end - model_start,
                            'postprocess': post_end - model_end,
                            'validate': validated - post_end,
                            'pre_model_post_validate': validated - total_start}


def local(args, runtime):
    read_start = time.perf_counter()
    payload = args.input.read_bytes()
    sample, mode, source = read_npz(payload)
    input_read_decode = time.perf_counter() - read_start
    rows = []
    count = 1 + args.warmup + args.samples
    for index in range(count):
        phase = 'first' if index == 0 else ('warmup' if index <= args.warmup else 'measured')
        actions, measured = runtime.infer(sample)
        rows.append({'index': index, 'phase': phase, 'seconds': measured})
        print(json.dumps(rows[-1]), flush=True)
    warm = [row for row in rows if row['phase'] == 'measured']
    report = {'metadata': runtime.metadata, 'input': str(args.input.resolve()),
              'input_read_decode_seconds': input_read_decode, 'input_mode': mode,
              'source': source, 'source_capture_wall_time': None,
              'first_prediction': rows[0], 'warmup_count': args.warmup,
              'warm_summary': {key: stats([row['seconds'][key] for row in warm])
                               for key in warm[0]['seconds']},
              'results': rows, 'last_action_shape': list(actions.shape),
              'last_actions': actions.tolist(), 'robot_io': False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')


def server(args, runtime):
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix('.metadata.json').write_text(json.dumps(runtime.metadata, indent=2) + '\n')
    counter = 0
    mode_counters = {}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def reply(self, code, result):
            payload = json.dumps(result, ensure_ascii=False, separators=(',', ':')).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            self.wfile.flush()

        def do_GET(self):
            if self.path == '/health':
                self.reply(200, {'ready': True, 'metadata': runtime.metadata})
            else:
                self.reply(404, {'error': 'unknown endpoint'})

        def do_POST(self):
            nonlocal counter
            began = time.perf_counter()
            try:
                if self.path != '/infer':
                    self.reply(404, {'error': 'unknown endpoint'})
                    return
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length < 16 * 1024 * 1024:
                    raise ValueError('Expected nonempty NPZ below 16 MiB.')
                payload = self.rfile.read(length)
                body_read = time.perf_counter()
                if len(payload) != length:
                    raise ValueError('Truncated body.')
                sample, mode, source = read_npz(payload)
                unpacked = time.perf_counter()
                actions, measured = runtime.infer(sample)
                as_list = actions.tolist()
                converted = time.perf_counter()
                result = {'request_id': counter, 'cold_prediction': counter == 0,
                          'mode_request_index': mode_counters.get(mode, 0),
                          'input_mode': mode, 'source': source,
                          'source_capture_wall_time': None, 'robot_io': False,
                          'request_bytes': length, 'action_shape': [100, 7],
                          'actions': as_list, 'server_seconds': {
                              **measured, 'read_body': body_read - began,
                              'decode_validate_input': unpacked - body_read,
                              'before_response_serialization': converted - began}}
                with args.output.open('a') as stream:
                    stream.write(json.dumps(result) + '\n')
                counter += 1
                mode_counters[mode] = mode_counters.get(mode, 0) + 1
                self.reply(200, result)
            except Exception as error:
                self.reply(500, {'error': f'{type(error).__name__}: {error}'})

    class NoDelayHTTPServer(HTTPServer):
        def get_request(self):
            connection, address = super().get_request()
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            connection.settimeout(180)
            return connection, address

    print(json.dumps({'ready': True, 'port': args.port, 'metadata': runtime.metadata}), flush=True)
    NoDelayHTTPServer(('127.0.0.1', args.port), Handler).serve_forever()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('local', 'server'))
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--device', choices=('cpu', 'cuda'), required=True)
    parser.add_argument('--threads', type=int, default=8)
    parser.add_argument('--input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--warmup', type=int, default=5)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--port', type=int, default=18806)
    args = parser.parse_args()
    if args.threads < 1 or args.warmup < 0 or args.samples < 1:
        parser.error('Positive threads/samples and nonnegative warmup required.')
    if args.mode == 'local' and args.input is None:
        parser.error('local requires --input')
    runtime = Runtime(args.model.resolve(), args.device, args.threads)
    (local if args.mode == 'local' else server)(args, runtime)


if __name__ == '__main__':
    main()

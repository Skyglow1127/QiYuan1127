#!/usr/bin/env python3
"""Recorded-input PI05+LoRA benchmark server. No robot/camera/control imports.

Bind only to 127.0.0.1 for requests from this computer. Requires the inspected
LeRobot 0.6.1 runtime and pi05_low_memory.py beside this script.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, HTTPServer
import importlib.metadata
import io
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--adapter', type=Path, required=True)
    parser.add_argument('--tokenizer', type=Path, required=True)
    parser.add_argument('--port', type=int, default=18805)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    import numpy as np
    import torch
    from peft import PeftConfig, PeftModel, get_peft_model_state_dict
    from safetensors.torch import load_file
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies import make_pre_post_processors
    # Import before config deserialization so Draccus registers the PI05 type.
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy  # noqa: F401
    from pi05_low_memory import load_base_streaming

    if importlib.metadata.version('lerobot') != '0.6.1':
        raise RuntimeError('Requires the inspected lerobot==0.6.1 runtime.')
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA GPU is not available.')
    torch.set_num_threads(8)
    cfg = PreTrainedConfig.from_pretrained(
        str(args.adapter), local_files_only=True,
        cli_overrides=['--device=cuda', '--dtype=bfloat16',
                       '--gradient_checkpointing=false', '--compile_model=false',
                       '--push_to_hub=false'])
    if cfg.chunk_size != 50 or cfg.num_inference_steps != 10:
        raise RuntimeError('This comparison requires chunk_size=50 and num_inference_steps=10.')
    cfg.pretrained_path = str(args.base.resolve())
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg, pretrained_path=str(args.adapter),
        preprocessor_overrides={
            'tokenizer_processor': {'tokenizer_name': str(args.tokenizer.resolve())},
            'device_processor': {'device': 'cuda'}},
        postprocessor_overrides={'device_processor': {'device': 'cpu'}})
    load_start = time.perf_counter()
    base, base_load = load_base_streaming(args.base, cfg)
    peft_cfg = PeftConfig.from_pretrained(str(args.adapter), local_files_only=True)
    peft_cfg.base_model_name_or_path = str(args.base.resolve())
    policy = PeftModel.from_pretrained(base, str(args.adapter), config=peft_cfg,
                                       local_files_only=True, is_trainable=False)
    policy.eval().requires_grad_(False)
    saved = load_file(str(args.adapter / 'adapter_model.safetensors'), device='cpu')
    actual = get_peft_model_state_dict(policy, adapter_name='default')
    if set(saved) != set(actual):
        raise RuntimeError('Loaded LoRA keys differ from saved keys.')
    for key, value in saved.items():
        loaded = actual[key].detach().cpu()
        if loaded.shape != value.shape or not torch.equal(value.to(loaded.dtype), loaded):
            raise RuntimeError(f'Loaded LoRA tensor differs: {key}')
    adapter_tensors = len(saved)
    del saved, actual
    torch.cuda.synchronize()
    metadata = {
        'ready_utc': datetime.now(timezone.utc).isoformat(), 'robot_io': False,
        'gpu': torch.cuda.get_device_name(), 'batch_size': 1, 'chunk_size': 50,
        'num_inference_steps': 10, 'compile_model': False,
        'load_seconds': time.perf_counter() - load_start,
        'base': str(args.base.resolve()), 'adapter': str(args.adapter.resolve()),
        'tokenizer': str(args.tokenizer.resolve()), 'adapter_tensors_verified': adapter_tensors,
        'base_loading': base_load,
        'versions': {name: importlib.metadata.version(name)
                     for name in ('torch', 'lerobot', 'transformers', 'peft', 'accelerate')}}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.with_suffix('.metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    counter = 0
    mode_counters = {}

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def reply(self, code, obj):
            body = json.dumps(obj, ensure_ascii=False, separators=(',', ':')).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/health':
                self.reply(200, {'ready': True, 'metadata': metadata})
            else:
                self.reply(404, {'error': 'unknown endpoint'})

        def do_POST(self):
            nonlocal counter
            began = time.perf_counter()
            received_unix_ns = time.time_ns()
            try:
                if self.path != '/infer':
                    self.reply(404, {'error': 'unknown endpoint'})
                    return
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length < 16 * 1024 * 1024:
                    raise ValueError('Expected a nonempty NPZ request below 16 MiB.')
                payload = self.rfile.read(length)
                body_read_end = time.perf_counter()
                if len(payload) != length:
                    raise ValueError('Truncated request body.')
                with np.load(io.BytesIO(payload), allow_pickle=False) as archive:
                    input_mode = str(archive['input_mode'].item()) if 'input_mode' in archive else 'original_float_npz'
                    if input_mode not in ('original_float_npz', 'resized_uint8_npz'):
                        raise ValueError(f'Unknown input mode: {input_mode}')
                    sample = {key: torch.from_numpy(archive[key].copy()) for key in cfg.input_features}
                    sample['task'] = str(archive['task'].item())
                    source = {key: archive[key].item() for key in
                              ('source_episode', 'source_frame', 'source_relative_time_s',
                               'source_timestamp_kind') if key in archive}
                for key in cfg.input_features:
                    image = key.startswith('observation.images.')
                    expected_shape = (3, 224, 224) if image and input_mode == 'resized_uint8_npz' else tuple(cfg.input_features[key].shape)
                    if tuple(sample[key].shape) != expected_shape:
                        raise ValueError(f'Unexpected shape: {key}={tuple(sample[key].shape)}')
                    if image and input_mode == 'resized_uint8_npz':
                        if sample[key].dtype != torch.uint8:
                            raise ValueError(f'Compact image must be uint8: {key}')
                        sample[key] = sample[key].to(torch.float32) / 255.0
                    elif image and sample[key].dtype != torch.float32:
                        raise ValueError(f'Original image must be float32: {key}')
                    if not torch.isfinite(sample[key]).all().item():
                        raise ValueError(f'Nonfinite input: {key}')
                unpack_end = time.perf_counter()
                torch.cuda.synchronize()
                preprocess_start = time.perf_counter()
                batch = preprocessor(sample)
                torch.cuda.synchronize()
                preprocess_end = time.perf_counter()
                policy.reset()
                torch.manual_seed(counter)
                torch.cuda.reset_peak_memory_stats()
                torch.cuda.synchronize()
                model_start = time.perf_counter()
                with torch.inference_mode():
                    prediction = policy.predict_action_chunk(batch)
                torch.cuda.synchronize()
                model_end = time.perf_counter()
                with torch.inference_mode():
                    actions = postprocessor(prediction).detach().cpu()
                torch.cuda.synchronize()
                postprocess_end = time.perf_counter()
                if tuple(actions.shape) != (1, 50, 7) or not torch.isfinite(actions).all().item():
                    raise RuntimeError(f'Invalid returned actions: shape={tuple(actions.shape)}')
                actions_json = actions[0].tolist()
                conversion_end = time.perf_counter()
                result = {
                    'request_id': counter, 'cold_prediction': counter == 0,
                    'input_mode': input_mode,
                    'mode_request_index': mode_counters.get(input_mode, 0),
                    'first_request_for_mode': mode_counters.get(input_mode, 0) == 0,
                    'robot_io': False, 'source': source,
                    'source_capture_wall_time': None,
                    'client_sent_unix_ns': self.headers.get('X-Client-Sent-Unix-Ns'),
                    'server_received_unix_ns': received_unix_ns,
                    'request_bytes': length, 'batch_size': 1, 'action_shape': [50, 7],
                    'task': sample['task'], 'actions': actions_json,
                    'units': ['degree'] * 6 + ['metre'],
                    'representation': 'absolute joint targets and gripper width',
                    'server_seconds': {
                        'read_request_body': body_read_end - began,
                        'unpack_validate': unpack_end - body_read_end,
                        'preprocess': preprocess_end - preprocess_start,
                        'model': model_end - model_start,
                        'postprocess': postprocess_end - model_end,
                        'actions_to_list': conversion_end - postprocess_end,
                        'processing_excluding_body_read':
                            (unpack_end - body_read_end) + (preprocess_end - preprocess_start)
                            + (model_end - model_start) + (postprocess_end - model_end)
                            + (conversion_end - postprocess_end),
                        'before_response_serialization': conversion_end - began},
                    'gpu_peak_allocated_bytes': torch.cuda.max_memory_allocated()}
                with args.report.open('a') as stream:
                    stream.write(json.dumps(result, ensure_ascii=False) + '\n')
                counter += 1
                mode_counters[input_mode] = mode_counters.get(input_mode, 0) + 1
                self.reply(200, result)
            except Exception as error:
                self.reply(500, {'error': f'{type(error).__name__}: {error}'})

    print(json.dumps({'ready': True, 'port': args.port, 'metadata': metadata}), flush=True)
    HTTPServer(('127.0.0.1', args.port), Handler).serve_forever()


if __name__ == '__main__':
    main()

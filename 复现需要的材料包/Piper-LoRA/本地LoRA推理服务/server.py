#!/usr/bin/env python3
"""Strict PI05+LoRA inference service. Localhost only; never controls hardware.

Only compact RGB CHW uint8 224x224 inputs are accepted. The caller performs
LeRobot's official resize-with-padding; no JPEG or image geometry change here.
All observation age decisions remain with the local robot-side monotonic clock.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.metadata
import io
import json
import os
from pathlib import Path
import socket
import threading
import time
import uuid
import zipfile

PROTOCOL = 'pi05-local-grasp/v1'
INPUT_KEYS = {'protocol_version', 'request_id', 'task', 'observation.state',
              'observation.images.global', 'observation.images.wrist'}
UNITS = ['degree'] * 6 + ['metre']
REPRESENTATION = 'absolute joint targets and gripper width'


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def file_identity(path):
    info = Path(path).stat()
    return {'path': str(Path(path).resolve()), 'inode': info.st_ino,
            'device': info.st_dev, 'size': info.st_size,
            'mtime_ns': info.st_mtime_ns, 'ctime_ns': info.st_ctime_ns}


def verify_weights(path, expected_hash, expected_size, cache_path=None):
    """Cache only an actually computed digest; any file metadata change rehashes."""
    identity = file_identity(path)
    if identity['size'] != expected_size:
        raise RuntimeError(f'Unexpected model file size: {path}')
    cached = None
    if cache_path and cache_path.exists():
        try:
            cached = json.loads(cache_path.read_text())
        except (OSError, ValueError):
            pass
    reused = bool(cached and cached.get('file') == identity
                  and cached.get('sha256') == expected_hash)
    digest = cached['sha256'] if reused else sha256_file(path)
    if digest != expected_hash or file_identity(path) != identity:
        raise RuntimeError(f'Model SHA256 mismatch or file changed during verification: {path}')
    if cache_path and not reused:
        temporary = cache_path.with_suffix('.tmp')
        temporary.write_text(json.dumps({'file': identity, 'sha256': digest}) + '\n')
        temporary.replace(cache_path)
    return {'sha256': digest, 'bytes': identity['size'], 'cache_reused': reused,
            'file_identity': identity}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--startup-id', required=True)
    args = parser.parse_args()
    startup_id = str(uuid.UUID(args.startup_id))
    settings = json.loads(args.config.read_text())
    if settings['protocol_version'] != PROTOCOL:
        raise RuntimeError('Unsupported configured protocol.')
    config_root = args.config.resolve().parent
    deployment = Path(settings.get('runtime_dir', 'runtime')).expanduser()
    if not deployment.is_absolute():
        deployment = config_root / deployment
    deployment = deployment.resolve()
    deployment.mkdir(parents=True, exist_ok=True)
    for key in ('HF_HUB_OFFLINE', 'TRANSFORMERS_OFFLINE', 'HF_DATASETS_OFFLINE',
                'HF_HUB_DISABLE_TELEMETRY', 'PYTHONNOUSERSITE'):
        os.environ[key] = '1'
    os.environ['TOKENIZERS_PARALLELISM'] = 'false'
    os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '4')
    os.environ.setdefault('MAX_JOBS', '4')
    base_dir, adapter_dir, tokenizer_dir = (Path(settings[key]).expanduser().resolve()
                                           for key in ('base', 'adapter', 'tokenizer'))
    base_identity = verify_weights(
        base_dir / 'model.safetensors', settings['expected_base_sha256'],
        settings['expected_base_bytes'], deployment / 'base_hash_cache.json')
    adapter_identity = verify_weights(
        adapter_dir / 'adapter_model.safetensors', settings['expected_adapter_sha256'],
        settings['expected_adapter_bytes'])

    import numpy as np
    import torch
    from peft import PeftConfig, PeftModel, get_peft_model_state_dict
    from safetensors.torch import load_file
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies import make_pre_post_processors
    # Registers the PI05 config before deserialization (LeRobot 0.6.1).
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy  # noqa: F401
    from pi05_low_memory import load_base_streaming

    if importlib.metadata.version('lerobot') != '0.6.1':
        raise RuntimeError('This deployment requires inspected LeRobot 0.6.1.')
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA GPU is unavailable.')
    torch.set_num_threads(8)
    torch.set_num_interop_threads(4)
    cfg = PreTrainedConfig.from_pretrained(
        str(adapter_dir), local_files_only=True,
        cli_overrides=['--device=cuda', '--dtype=bfloat16',
                       '--gradient_checkpointing=false', '--compile_model=false',
                       '--push_to_hub=false'])
    if cfg.chunk_size != 50 or cfg.num_inference_steps != 10:
        raise RuntimeError('Expected chunk_size=50, num_inference_steps=10.')
    if set(cfg.input_features) != {'observation.state', 'observation.images.global',
                                  'observation.images.wrist'}:
        raise RuntimeError('Unexpected model observation features.')
    cfg.pretrained_path = str(base_dir)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg, pretrained_path=str(adapter_dir),
        preprocessor_overrides={
            'tokenizer_processor': {'tokenizer_name': str(tokenizer_dir)},
            'device_processor': {'device': 'cuda'}},
        postprocessor_overrides={'device_processor': {'device': 'cpu'}})
    load_start = time.perf_counter()
    base, load_proof = load_base_streaming(base_dir, cfg)
    peft_cfg = PeftConfig.from_pretrained(str(adapter_dir), local_files_only=True)
    peft_cfg.base_model_name_or_path = str(base_dir)
    policy = PeftModel.from_pretrained(base, str(adapter_dir), config=peft_cfg,
                                      local_files_only=True, is_trainable=False)
    policy.eval().requires_grad_(False)
    saved = load_file(str(adapter_dir / 'adapter_model.safetensors'), device='cpu')
    loaded = get_peft_model_state_dict(policy, adapter_name='default')
    if len(saved) != 80 or set(saved) != set(loaded):
        raise RuntimeError('Unexpected or missing LoRA keys.')
    for key, value in saved.items():
        actual = loaded[key].detach().cpu()
        if actual.shape != value.shape or not torch.equal(value.to(actual.dtype), actual):
            raise RuntimeError(f'LoRA verification failed: {key}')
    del saved, loaded
    if file_identity(base_dir / 'model.safetensors') != base_identity['file_identity']:
        raise RuntimeError('Base file changed while loading.')
    if file_identity(adapter_dir / 'adapter_model.safetensors') != adapter_identity['file_identity']:
        raise RuntimeError('Adapter file changed while loading.')
    compile_inference = settings.get('compile_inference', False)
    if type(compile_inference) is not bool:
        raise RuntimeError('compile_inference must be an explicit boolean.')
    if compile_inference:
        # Compile the selected official inference routine. Warmup must finish
        # before readiness; strict loading and parameter dtypes stay unchanged.
        torch.set_float32_matmul_precision('high')
        torch._dynamo.config.suppress_errors = False
        compile_target = settings.get('compile_target', 'sample_actions')
        core = policy.get_base_model().model
        if compile_target == 'denoise_step':
            compile_mode = 'reduce-overhead'
            core.denoise_step = torch.compile(core.denoise_step, mode=compile_mode)
        elif compile_target == 'sample_actions':
            compile_mode = cfg.compile_mode
            core.sample_actions = torch.compile(core.sample_actions, mode=compile_mode)
        else:
            raise RuntimeError('Unsupported compile_target; no fallback attempted.')
    torch.cuda.synchronize()
    loading_seconds = time.perf_counter() - load_start

    def predict(sample):
        torch.cuda.synchronize()
        begin = time.perf_counter()
        batch = preprocessor(sample)
        torch.cuda.synchronize()
        pre_end = time.perf_counter()
        policy.reset()
        with torch.inference_mode():
            prediction = policy.predict_action_chunk(batch)
        torch.cuda.synchronize()
        model_end = time.perf_counter()
        with torch.inference_mode():
            actions = postprocessor(prediction).detach().cpu()
        torch.cuda.synchronize()
        post_end = time.perf_counter()
        if tuple(actions.shape) != (1, 50, 7) or not torch.isfinite(actions).all().item():
            raise RuntimeError('Model produced invalid actions.')
        return actions[0].tolist(), {'preprocess': pre_end - begin,
                                    'model': model_end - pre_end,
                                    'postprocess': post_end - model_end}

    warmup_start = time.perf_counter()
    for _ in range(int(settings.get('warmup_predictions', 2))):
        predict({'task': settings['task'], 'observation.state': torch.zeros(7),
                 'observation.images.global': torch.zeros(3, 224, 224),
                 'observation.images.wrist': torch.zeros(3, 224, 224)})
    if int(settings.get('warmup_predictions', 2)) < 1:
        raise RuntimeError('At least one startup warmup is required.')
    manifest = {}
    for prefix, folder in [('adapter', adapter_dir), ('tokenizer', tokenizer_dir)]:
        for path in sorted(folder.iterdir()):
            if path.is_file() and path.suffix in ('.json', '.safetensors', '.model'):
                manifest[f'{prefix}/{path.name}'] = sha256_file(path)
    metadata = {
        'model_id': settings['model_id'], 'pid': os.getpid(), 'startup_id': startup_id,
        'base_sha256': base_identity['sha256'], 'adapter_sha256': adapter_identity['sha256'],
        'base_verification': base_identity, 'adapter_verification': adapter_identity,
        'pipeline_files_sha256': manifest,
        'ready_utc': datetime.now(timezone.utc).isoformat(), 'robot_io': False,
        'device': torch.cuda.get_device_name(), 'chunk_size': 50,
        'action_shape': [50, 7], 'units': UNITS, 'representation': REPRESENTATION,
        'num_inference_steps': 10, 'batch_size': 1, 'compile_model': compile_inference,
        'compile_mode': compile_mode if compile_inference else None,
        'compile_target': compile_target if compile_inference else None,
        'float32_matmul_precision': torch.get_float32_matmul_precision(),
        'adapter_tensors_verified': 80, 'base_loading': load_proof,
        'loading_seconds': loading_seconds,
        'warmup_seconds': time.perf_counter() - warmup_start,
        'warmup_predictions': int(settings.get('warmup_predictions', 2)),
        'input_image_format': 'RGB CHW uint8 [3,224,224], official resize_with_pad_torch',
        'freshness_clock': 'caller local monotonic only; server timestamps are not observation timestamps',
        'versions': {name: importlib.metadata.version(name) for name in
                     ('torch', 'lerobot', 'transformers', 'peft', 'accelerate')}}
    (deployment / 'server_metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    infer_lock = threading.Lock()
    seen_ids = set()
    seen_order = deque()
    log_lock = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def setup(self):
            super().setup()
            # Keep the warmed connection alive across robot construction and
            # the guarded startup hold before the first live observation.
            self.connection.settimeout(30.0)
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def reply(self, status, body):
            payload = json.dumps(body, ensure_ascii=False, allow_nan=False,
                                 separators=(',', ':')).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            keep_alive = status == 200
            self.send_header('Connection', 'keep-alive' if keep_alive else 'close')
            self.end_headers()
            self.wfile.write(payload)
            self.close_connection = not keep_alive

        def do_GET(self):
            if self.path == '/health':
                self.reply(200, {'ready': True, 'warmed_up': True,
                                 'protocol_version': PROTOCOL, 'metadata': metadata})
            else:
                self.reply(404, {'error': 'unknown endpoint'})

        def do_POST(self):
            began = time.perf_counter()
            request_id = None
            acquired = False
            try:
                if self.path != '/infer':
                    self.reply(404, {'error': 'unknown endpoint'})
                    return
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/x-npz':
                    raise ValueError('Content-Type must be application/x-npz.')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 512 * 1024:
                    raise ValueError('Expected nonempty compact NPZ below 512 KiB.')
                expected_hash = self.headers.get('X-Input-SHA256', '')
                if len(expected_hash) != 64 or any(c not in '0123456789abcdef' for c in expected_hash):
                    raise ValueError('Expected lowercase hexadecimal X-Input-SHA256.')
                request_id = self.headers.get('X-Request-ID', '')
                if str(uuid.UUID(request_id)) != request_id:
                    raise ValueError('Expected canonical UUID X-Request-ID.')
                payload = self.rfile.read(length)
                if len(payload) != length:
                    raise ValueError('Truncated request body.')
                digest = hashlib.sha256(payload).hexdigest()
                if digest != expected_hash:
                    raise ValueError('Request body SHA256 mismatch.')
                with zipfile.ZipFile(io.BytesIO(payload)) as archive:
                    if set(archive.namelist()) != {name + '.npy' for name in INPUT_KEYS}:
                        raise ValueError('Unexpected or missing NPZ fields.')
                    if len(archive.infolist()) != len(INPUT_KEYS) or sum(
                            item.file_size for item in archive.infolist()) > 400 * 1024:
                        raise ValueError('Invalid decompressed NPZ size.')
                with np.load(io.BytesIO(payload), allow_pickle=False) as archive:
                    scalar = {}
                    for name in ('protocol_version', 'request_id', 'task'):
                        item = archive[name]
                        if item.shape != () or item.dtype.kind != 'U':
                            raise ValueError(f'Expected scalar Unicode string: {name}')
                        scalar[name] = str(item.item())
                    if scalar['protocol_version'] != PROTOCOL or scalar['request_id'] != request_id:
                        raise ValueError('Protocol or request UUID mismatch.')
                    if scalar['task'] != settings['task']:
                        raise ValueError('Task must match the trained red-cube task exactly.')
                    state = archive['observation.state']
                    if state.dtype != np.float32 or state.shape != (7,) or not np.isfinite(state).all():
                        raise ValueError('State must be finite float32[7] in degrees/metres.')
                    sample = {'task': scalar['task'], 'observation.state': torch.from_numpy(state.copy())}
                    for key in ('observation.images.global', 'observation.images.wrist'):
                        value = archive[key]
                        if value.dtype != np.uint8 or value.shape != (3, 224, 224):
                            raise ValueError(f'{key} must be RGB uint8[3,224,224].')
                        sample[key] = torch.from_numpy(value.copy()).to(torch.float32) / 255.0
                decoded = time.perf_counter()
                acquired = infer_lock.acquire(blocking=False)
                if not acquired:
                    self.reply(429, {'error': 'inference busy; do not queue stale observations',
                                     'request_id': request_id, 'protocol_version': PROTOCOL})
                    return
                if request_id in seen_ids:
                    self.reply(409, {'error': 'duplicate request UUID', 'request_id': request_id})
                    return
                seen_ids.add(request_id)
                seen_order.append(request_id)
                if len(seen_order) > 1024:
                    seen_ids.remove(seen_order.popleft())
                actions, timing = predict(sample)
                finished = time.perf_counter()
                timing['decode'] = decoded - began
                timing['total'] = finished - began
                response = {'protocol_version': PROTOCOL, 'request_id': request_id,
                            'input_sha256': digest, 'model_id': settings['model_id'],
                            'actions': actions, 'action_shape': [50, 7],
                            'units': UNITS, 'representation': REPRESENTATION,
                            'server_timing_s': timing, 'robot_io': False,
                            'startup_id': startup_id}
                with log_lock, (deployment / 'requests.jsonl').open('a') as stream:
                    stream.write(json.dumps({key: value for key, value in response.items()
                                             if key != 'actions'}) + '\n')
                self.reply(200, response)
            except (ValueError, KeyError, zipfile.BadZipFile, EOFError) as error:
                self.reply(400, {'error': f'{type(error).__name__}: {error}',
                                 'request_id': request_id, 'protocol_version': PROTOCOL})
            except (BrokenPipeError, ConnectionResetError, socket.timeout):
                self.close_connection = True
            except Exception as error:
                print(f'Inference failed: {type(error).__name__}: {error}', flush=True)
                self.reply(500, {'error': f'{type(error).__name__}: {error}',
                                 'request_id': request_id, 'protocol_version': PROTOCOL})
            finally:
                if acquired:
                    infer_lock.release()

    print(json.dumps({'ready': True, 'protocol_version': PROTOCOL,
                      'startup_id': startup_id, 'port': settings['local_port']}), flush=True)
    server = ThreadingHTTPServer(('127.0.0.1', int(settings['local_port'])), Handler)
    server.daemon_threads = True
    server.serve_forever(poll_interval=0.2)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Benchmark the saved Piper PI0.5 LoRA on one recorded observation."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import sys
import time

import numpy as np
import psutil
import torch
from peft import PeftConfig, PeftModel, get_peft_model_state_dict
from safetensors.torch import load_file
from lerobot.configs import PreTrainedConfig
from lerobot.policies import make_pre_post_processors

OUTPUTS = Path('/home/aup/Documents/Codex/v5/wo-y/outputs')
sys.path.insert(0, str(OUTPUTS))
from pi05_low_memory import load_base_streaming

BASE = Path('/home/aup/Lerobot/models/PI05/pi05_base')
BACKUP = OUTPUTS / 'piper_pi05_cloud_backup_v5'
ADAPTER = BACKUP / 'runs/pi05_train_v5_r06/checkpoints/020000/pretrained_model'
TOKENIZER = BACKUP / 'models/paligemma_tokenizer'
OBSERVATION = OUTPUTS / 'pi05_actual_inference_v5/recorded_observation.npz'


def save_json(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    os.replace(temp, path)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def p95(values):
    values = sorted(values)
    index = 0.95 * (len(values) - 1)
    lo, hi = math.floor(index), math.ceil(index)
    return values[lo] + (values[hi] - values[lo]) * (index - lo)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--warmup', type=int, default=3)
    parser.add_argument('--repeats', type=int, default=10)
    args = parser.parse_args()
    if args.warmup < 1 or args.repeats < 2:
        parser.error('warmup must be >= 1 and repeats >= 2')
    output = args.output
    assert output.is_dir()
    report = {
        'success': False,
        'scope': 'One recorded observation; preprocessed input in memory through postprocessed 50x7 actions; GPU synchronized.',
        'robot_io': False,
        'model': str(BASE),
        'adapter': str(ADAPTER),
        'adapter_weights_sha256': sha256_file(ADAPTER / 'adapter_model.safetensors'),
        'tokenizer': str(TOKENIZER),
        'observation': str(OBSERVATION),
        'observation_sha256': sha256_file(OBSERVATION),
        'kernel': platform.release(),
        'device': torch.cuda.get_device_name(0),
        'torch': torch.__version__,
        'hip': torch.version.hip,
        'batch_size': 1,
        'warmup_each': args.warmup,
        'samples_each': args.repeats,
        'gpu_allocator_fraction': 0.85,
        'system_available_before_load_bytes': psutil.virtual_memory().available,
        'variants': {},
    }
    save_json(output / 'report.partial.json', report)

    torch.set_num_threads(8)
    torch.cuda.set_per_process_memory_fraction(0.85)
    torch.cuda.reset_peak_memory_stats()
    cfg = PreTrainedConfig.from_pretrained(
        str(ADAPTER), local_files_only=True,
        cli_overrides=['--device=cuda', '--dtype=bfloat16',
                       '--gradient_checkpointing=false', '--compile_model=false',
                       '--push_to_hub=false'],
    )
    assert cfg.chunk_size == 50 and cfg.num_inference_steps == 10
    cfg.pretrained_path = str(BASE)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg, pretrained_path=str(ADAPTER),
        preprocessor_overrides={
            'tokenizer_processor': {'tokenizer_name': str(TOKENIZER)},
            'device_processor': {'device': 'cuda'},
        },
        postprocessor_overrides={'device_processor': {'device': 'cpu'}},
    )
    with np.load(OBSERVATION, allow_pickle=False) as archive:
        observation = {key: torch.from_numpy(archive[key].copy()) for key in cfg.input_features}
        observation['task'] = str(archive['task'].item())
    report['task'] = observation['task']
    batch = preprocessor(observation)
    started = time.monotonic()
    print('正在将基础模型和 LoRA 加载到本机 GPU……', flush=True)
    base, loading = load_base_streaming(BASE, cfg)
    assert not loading['missing_keys'] and not loading['unexpected_keys'] and not loading['shape_mismatches']
    peft_cfg = PeftConfig.from_pretrained(str(ADAPTER), local_files_only=True)
    peft_cfg.base_model_name_or_path = str(BASE)
    policy = PeftModel.from_pretrained(base, str(ADAPTER), config=peft_cfg,
                                       local_files_only=True, is_trainable=False)
    policy.eval().requires_grad_(False)
    saved = load_file(str(ADAPTER / 'adapter_model.safetensors'), device='cpu')
    loaded = get_peft_model_state_dict(policy, adapter_name='default')
    assert set(saved) == set(loaded)
    for name in saved:
        actual = loaded[name].detach().cpu()
        assert saved[name].shape == actual.shape and torch.equal(saved[name].to(actual.dtype), actual), name
    report['verified_adapter_tensors'] = len(saved)
    del saved, loaded
    torch.cuda.synchronize()
    report['loading_seconds'] = time.monotonic() - started
    print(f'模型加载并完成 LoRA 校验，用时 {report["loading_seconds"]:.2f} 秒。', flush=True)
    save_json(output / 'report.partial.json', report)

    # The same noise seed is used for each variant in each round. Reversing the
    # order on alternating rounds reduces ordering bias from thermal/load drift.
    variants = [10, 5, 3, 2, 1]
    timings = {steps: [] for steps in variants}
    predictions = {steps: [] for steps in variants}
    for index in range(args.warmup + args.repeats):
        order = variants if index % 2 == 0 else list(reversed(variants))
        phase = 'warmup' if index < args.warmup else 'measured'
        sample_index = index - args.warmup
        for steps in order:
            policy.reset()
            torch.manual_seed(20000 + index)
            torch.cuda.synchronize()
            start = time.perf_counter()
            with torch.inference_mode():
                actions = postprocessor(policy.predict_action_chunk(batch, num_steps=steps)).detach().cpu()
            torch.cuda.synchronize()
            elapsed = time.perf_counter() - start
            if tuple(actions.shape) != (1, 50, 7) or not torch.isfinite(actions).all().item():
                raise RuntimeError(f'Invalid output for steps={steps}, round={index}')
            if phase == 'measured':
                timings[steps].append(elapsed)
                predictions[steps].append(actions[0].numpy().copy())
            display_round = index + 1 if phase == 'warmup' else sample_index + 1
            phase_label = '预热' if phase == 'warmup' else '正式测速'
            print(f'{phase_label}第 {display_round} 次：流匹配 {steps} 步，用时 {elapsed:.3f} 秒。', flush=True)
        if phase == 'measured':
            report['completed_measured_rounds'] = sample_index + 1
            report['partial_timings_seconds'] = {str(k): v for k, v in timings.items()}
            save_json(output / 'report.partial.json', report)

    base_actions = np.stack(predictions[10])
    np.savez_compressed(output / 'predicted_actions_by_step.npz',
                        **{f'steps_{k}': np.stack(v) for k, v in predictions.items()})
    for steps in variants:
        times = timings[steps]
        diff = np.abs(np.stack(predictions[steps]) - base_actions)
        report['variants'][str(steps)] = {
            'steps': steps,
            'times_seconds': times,
            'median_seconds': statistics.median(times),
            'mean_seconds': statistics.mean(times),
            'p95_seconds': p95(times),
            'max_seconds': max(times),
            'minimum_seconds': min(times),
            'action_shape': [50, 7],
            'joint_mean_abs_difference_deg_vs_10_steps': float(diff[:, :, :6].mean()),
            'joint_max_abs_difference_deg_vs_10_steps': float(diff[:, :, :6].max()),
            'gripper_mean_abs_difference_m_vs_10_steps': float(diff[:, :, 6].mean()),
            'gripper_max_abs_difference_m_vs_10_steps': float(diff[:, :, 6].max()),
        }
    report.pop('partial_timings_seconds', None)
    report['gpu_peak_allocated_bytes'] = torch.cuda.max_memory_allocated()
    report['system_available_after_bytes'] = psutil.virtual_memory().available
    report['success'] = True
    save_json(output / 'report.json', report)
    lines = [
        '# π0.5 + LoRA 020000 本机推理测速结果',
        '',
        f'计算设备：{report["device"]}；PyTorch {report["torch"]}；ROCm HIP {report["hip"]}。',
        f'基座：`{BASE}`；LoRA：`{ADAPTER}`。',
        f'使用同一条录制观察，每种设置预热 {args.warmup} 次、正式计时 {args.repeats} 次；每轮在不同设置下使用同一随机种子。',
        '计时范围为内存中已完成预处理的输入到输出经反归一化的完整 50×7 动作块，包含 GPU 同步及输出传回；不包含相机采集和输入预处理。',
        '',
        '| 流匹配步数 | 中位时间（秒） | P95（秒） | 最大时间（秒） | 相对 10 步的关节平均绝对差（度） | 相对 10 步的关节最大绝对差（度） |',
        '| ---: | ---: | ---: | ---: | ---: | ---: |',
    ]
    for steps in variants:
        item = report['variants'][str(steps)]
        lines.append('| %d | %.4f | %.4f | %.4f | %.4f | %.4f |' % (
            steps, item['median_seconds'], item['p95_seconds'], item['max_seconds'],
            item['joint_mean_abs_difference_deg_vs_10_steps'],
            item['joint_max_abs_difference_deg_vs_10_steps']))
    lines += ['', '较少的流匹配步数会改变动作数值；此离线对照本身不提供实机任务成功率。',
              f'完整参数、逐次时间和夹爪差异见 `{output / "report.json"}`。']
    (output / '技术文档可用数据.md').write_text('\n'.join(lines) + '\n')
    print('逐次结果：', output / 'report.json', flush=True)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Check or run the trained PI05 LoRA on one recorded frame, without robot I/O."""
from __future__ import annotations

import argparse
import contextlib
from datetime import datetime
import importlib.metadata
import io
import json
from pathlib import Path
import subprocess
import time
from zoneinfo import ZoneInfo


HERE = Path(__file__).resolve().parent
MODELS = Path('/home/aup/Lerobot/models/PI05')
ADAPTER = MODELS / 'piper_lora/checkpoints/020000/pretrained_model'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=Path('/home/aup/Lerobot/models/PI05/pi05_base'))
    parser.add_argument('--adapter', type=Path, default=ADAPTER)
    parser.add_argument('--tokenizer', type=Path, default=MODELS / 'paligemma_tokenizer')
    parser.add_argument('--dataset', type=Path, default=Path('/home/aup/Lerobot/datasets/rad_place_merged_v5'))
    parser.add_argument('--data-python', type=Path, default=Path('/home/aup/Lerobot/envs/piper-cpu/bin/python'),
                        help='Existing dataset-reader environment; kept separate from ROCm Torch.')
    parser.add_argument('--episode', type=int, default=46)
    parser.add_argument('--frame', type=int, default=0, help='Frame index within the selected episode.')
    parser.add_argument('--device', choices=['cpu', 'cuda'], default='cuda')
    parser.add_argument('--load-mode', choices=['streaming', 'standard'], default='streaming',
                        help='Streaming preserves mixed precision without full FP32 CPU initialization.')
    parser.add_argument('--repeats', type=int, default=2,
                        help='Number of recorded-frame predictions, including a first cold prediction.')
    parser.add_argument('--output-dir', type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--preflight', action='store_true', help='Check files and preprocessing; the default.')
    mode.add_argument('--run', action='store_true', help='Also load the full model and predict one action chunk.')
    args = parser.parse_args()
    if args.episode < 0 or args.frame < 0 or args.repeats < 1:
        parser.error('episode/frame must be nonnegative and repeats must be positive')
    timestamp = datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S')
    output = args.output_dir or HERE / f'pi05_offline_{timestamp}'
    output.mkdir(parents=True, exist_ok=False)
    report = {'mode': 'run' if args.run else 'preflight', 'passed': False,
              'model_inference_executed': False, 'robot_io': False,
              'episode': args.episode, 'frame': args.frame, 'output_directory': str(output)}
    report_path = output / 'report.json'
    try:
        import torch
        import psutil
        from peft import PeftConfig, PeftModel, get_peft_model_state_dict
        from lerobot.configs import PreTrainedConfig
        from lerobot.policies import make_pre_post_processors
        from lerobot.policies.pi05.modeling_pi05 import PI05Policy

        torch.set_num_threads(8)
        versions = {name: importlib.metadata.version(name) for name in
                    ('torch', 'lerobot', 'transformers', 'peft', 'accelerate')}
        report['packages'] = versions
        if versions['lerobot'] != '0.6.1':
            raise RuntimeError('This entry point is checked against lerobot==0.6.1.')
        for folder, names in (
            (args.base, ['config.json', 'model.safetensors']),
            (args.adapter, ['config.json', 'adapter_config.json', 'adapter_model.safetensors',
                            'policy_preprocessor.json', 'policy_postprocessor.json']),
            (args.tokenizer, ['tokenizer.json', 'tokenizer_config.json']),
            (args.dataset, ['meta/info.json', 'meta/stats.json']),
        ):
            for name in names:
                if not (folder / name).is_file():
                    raise FileNotFoundError(folder / name)
        for name in ('policy_preprocessor.json', 'policy_postprocessor.json'):
            for step in json.loads((args.adapter / name).read_text())['steps']:
                if step.get('state_file') and not (args.adapter / step['state_file']).is_file():
                    raise FileNotFoundError(args.adapter / step['state_file'])
        # Keep the saved local model intact; replace paths only at runtime.
        cfg = PreTrainedConfig.from_pretrained(
            str(args.adapter), local_files_only=True,
            cli_overrides=[f'--device={args.device}', '--dtype=bfloat16',
                           '--gradient_checkpointing=false', '--compile_model=false',
                           '--push_to_hub=false'],
        )
        cfg.pretrained_path = str(args.base.resolve())
        processor_device = args.device if args.run else 'cpu'
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=cfg, pretrained_path=str(args.adapter),
            preprocessor_overrides={
                'tokenizer_processor': {'tokenizer_name': str(args.tokenizer.resolve())},
                'device_processor': {'device': processor_device},
            },
            postprocessor_overrides={'device_processor': {'device': 'cpu'}},
        )
        # The existing CPU environment already has the dataset/video extras.
        # Export one frame instead of mixing its packages with the ROCm environment.
        frame_path = output / 'recorded_observation.npz'
        subprocess.run([str(args.data_python), str(HERE / 'export_pi05_recorded_input.py'),
                        '--dataset', str(args.dataset), '--episode', str(args.episode),
                        '--frame', str(args.frame), '--output', str(frame_path)], check=True)
        import numpy as np
        with np.load(frame_path, allow_pickle=False) as archive:
            sample = {key: torch.from_numpy(archive[key].copy()) for key in cfg.input_features}
            sample['task'] = str(archive['task'].item())
        observation = {key: sample[key] for key in cfg.input_features}
        observation['task'] = sample['task']
        for key, tensor in observation.items():
            if torch.is_tensor(tensor) and not torch.isfinite(tensor).all().item():
                raise ValueError(f'Nonfinite input: {key}')
        batch = preprocessor(observation)
        report.update(
            base=str(args.base.resolve()), adapter=str(args.adapter.resolve()),
            tokenizer=str(args.tokenizer.resolve()), task=sample['task'],
            input_shapes={key: list(sample[key].shape) for key in cfg.input_features},
            processed_shapes={key: list(value.shape) for key, value in batch.items()
                              if torch.is_tensor(value)},
            available_system_memory_bytes=psutil.virtual_memory().available,
            memory_note='The standard loader builds FP32 CPU parameters before casting. '
                        'Full-load peak and latency on the local shared-memory AMD GPU '
                        'have not been measured. Preflight does not allocate the model.',
        )
        if not args.run:
            report['passed'] = True
            print('Preflight passed: saved processors and one recorded observation work.')
            print('Full model loading and inference were not executed.')
            return

        if args.device == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('GPU computation device is unavailable.')
        print(f'Loading the full base and LoRA with {args.load_mode} loading.', flush=True)
        report['load_mode'] = args.load_mode
        started = time.monotonic()
        if args.device == 'cuda':
            torch.cuda.set_per_process_memory_fraction(0.85)
            report['gpu_allocator_fraction_limit'] = 0.85
            torch.cuda.reset_peak_memory_stats()
        if args.load_mode == 'streaming':
            from pi05_low_memory import load_base_streaming
            base, loading = load_base_streaming(args.base, cfg)
            (output / 'base_load.json').write_text(json.dumps(loading, indent=2) + '\n')
            report['base_loading'] = loading
        else:
            captured = io.StringIO()
            with contextlib.redirect_stdout(captured):
                base = PI05Policy.from_pretrained(str(args.base), config=cfg,
                                                 local_files_only=True, strict=True)
            load_log = captured.getvalue()
            (output / 'base_load.log').write_text(load_log)
            if 'All keys loaded successfully!' not in load_log.splitlines():
                raise RuntimeError('Base loading was not confirmed; see base_load.log.')
        peft_cfg = PeftConfig.from_pretrained(str(args.adapter), local_files_only=True)
        peft_cfg.base_model_name_or_path = str(args.base.resolve())
        policy = PeftModel.from_pretrained(base, str(args.adapter), config=peft_cfg,
                                           local_files_only=True, is_trainable=False)
        policy.eval().requires_grad_(False)
        from safetensors.torch import load_file
        saved_adapter = load_file(str(args.adapter / 'adapter_model.safetensors'), device='cpu')
        loaded_adapter = get_peft_model_state_dict(policy, adapter_name='default')
        if set(saved_adapter) != set(loaded_adapter):
            raise RuntimeError('Loaded LoRA keys do not exactly match the saved adapter.')
        for name, saved in saved_adapter.items():
            loaded = loaded_adapter[name].detach().cpu()
            if saved.shape != loaded.shape or not torch.equal(saved.to(loaded.dtype), loaded):
                raise RuntimeError(f'Loaded LoRA differs from the saved tensor: {name}')
        report['adapter_tensors_verified'] = len(saved_adapter)
        del saved_adapter, loaded_adapter
        if args.device == 'cuda':
            torch.cuda.synchronize()
            report['gpu_peak_allocated_bytes_during_load'] = torch.cuda.max_memory_allocated()
            torch.cuda.reset_peak_memory_stats()
        report['load_seconds'] = time.monotonic() - started
        timings = []
        actions = None
        for repeat in range(args.repeats):
            policy.reset()
            torch.manual_seed(repeat)
            if args.device == 'cuda':
                torch.cuda.synchronize()
            started = time.monotonic()
            with torch.inference_mode():
                current = postprocessor(policy.predict_action_chunk(batch)).detach().cpu()
            if args.device == 'cuda':
                torch.cuda.synchronize()
            timings.append(time.monotonic() - started)
            if tuple(current.shape) != (1, cfg.chunk_size, 7):
                raise RuntimeError(f'Unexpected action shape: {tuple(current.shape)}')
            if not torch.isfinite(current).all().item():
                raise RuntimeError('Predicted actions contain nonfinite values.')
            if actions is None:
                actions = current
            print(f'Prediction {repeat + 1}/{args.repeats}: {timings[-1]:.3f} seconds', flush=True)
        if args.device == 'cuda':
            report['gpu_peak_allocated_bytes'] = torch.cuda.max_memory_allocated()
        report['prediction_seconds_each'] = timings
        report['predict_seconds'] = timings[0]
        report['warm_predict_seconds'] = timings[1:] if len(timings) > 1 else []
        action_output = {'names': cfg.action_feature_names,
                         'units': ['degree'] * 6 + ['metre'],
                         'representation': 'absolute joint targets and gripper width',
                         'actions': actions[0].tolist()}
        (output / 'predicted_actions.json').write_text(json.dumps(action_output, indent=2) + '\n')
        report.update(passed=True, model_inference_executed=True,
                      action_shape=list(actions.shape),
                      memory_note='Measured offline model/processor latency and GPU allocation in this run. '
                                  'Live camera, feedback and controller latency are not included.',
                      note='Recorded-frame inference only; this does not measure real task success.')
        print('Offline prediction passed. Actions were saved to', output / 'predicted_actions.json')
    except Exception as error:
        report['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print('Report:', report_path, flush=True)


if __name__ == '__main__':
    main()

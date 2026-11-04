#!/usr/bin/env python3
"""A separate official ACT fine-tune; never deploy or edit the parent model.

Default --check validates the saved plan, parent, data and official CLI config.
Only --run starts one bounded job in the new output directory.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys


HERE = Path(__file__).resolve().parent


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def command(plan):
    args = [sys.executable, '-m', 'lerobot.scripts.lerobot_train',
        '--policy.path='+plan['parent_checkpoint'],
        '--dataset.repo_id='+plan['dataset_repo_id'],
        '--dataset.root='+plan['dataset_root'],
        '--dataset.video_backend=torchcodec', '--dataset.eval_split=0.0',
        '--policy.device='+plan['device'], '--policy.use_amp=false',
        '--policy.push_to_hub=false', '--policy.chunk_size=100', '--policy.n_action_steps=100',
        '--policy.optimizer_lr='+str(plan['optimizer_lr']),
        '--policy.optimizer_lr_backbone='+str(plan['optimizer_lr_backbone']),
        '--batch_size='+str(plan['batch_size']), '--num_workers=0',
        '--persistent_workers=false', '--steps='+str(plan['steps']),
        '--seed='+str(plan['seed']), '--resume=false', '--use_policy_training_preset=true',
        '--eval_steps=0', '--env_eval_freq=0', '--save_freq='+str(plan['save_freq']),
        '--log_freq='+str(plan['log_freq']), '--wandb.enable=false',
        '--save_checkpoint_to_hub=false', '--job_name='+plan['name'],
        '--output_dir='+plan['output_dir']]
    return args


def validate_plan(plan):
    root = Path(plan['local_root']).resolve()
    parent = Path(plan['parent_checkpoint']).resolve()
    output = Path(plan['output_dir']).resolve()
    if (not output.is_relative_to(root/'outputs') or output == root/'outputs'
            or not output.name.startswith('act_rad_place_refine_')
            or output == parent or output in parent.parents or parent in output.parents):
        raise ValueError('New output must be separate from the parent and under local outputs')
    if output.exists():
        raise FileExistsError('New output already exists; refusing overwrite or automatic resume')
    if (plan['automatic_deploy'] is not False or plan['dataset_changes'] is not False
            or plan['framework_source_changes'] is not False or plan['resume_optimizer'] is not False):
        raise ValueError('Plan must preserve deployment, dataset and framework; weights-only initialization')
    if (plan['steps'] != 10000 or plan['batch_size'] != 8
            or plan['optimizer_lr'] != 5e-6 or plan['optimizer_lr_backbone'] != 2e-6
            or plan['chunk_size'] != 100 or plan['n_action_steps'] != 100
            or plan['use_amp'] is not False or plan['device'] not in ('cpu', 'cuda')):
        raise ValueError('Unexpected parameters for the bounded second-round plan')


def official_config_check(plan):
    # Parse using the installed official entry point's configuration classes,
    # without constructing a dataset, model, optimizer or training loop.
    from lerobot.configs import parser
    from lerobot.configs.train import TrainPipelineConfig
    from lerobot.policies.act.configuration_act import ACTConfig  # register the saved policy type

    @parser.wrap()
    def check(cfg: TrainPipelineConfig):
        cfg.validate()
        assert cfg.policy.type == 'act'
        assert Path(cfg.policy.pretrained_path).resolve() == Path(plan['parent_checkpoint']).resolve()
        assert cfg.optimizer.lr == plan['optimizer_lr']
        assert cfg.policy.optimizer_lr_backbone == plan['optimizer_lr_backbone']
        assert cfg.resume is False and cfg.steps == plan['steps']
        assert cfg.policy.chunk_size == 100 and cfg.policy.n_action_steps == 100
        assert cfg.policy.temporal_ensemble_coeff is None
        return cfg.to_dict()

    previous_argv = sys.argv
    try:
        sys.argv = ['lerobot-train', *command(plan)[3:]]
        return check()
    finally:
        sys.argv = previous_argv


def preflight(plan):
    validate_plan(plan)
    assert importlib.metadata.version('lerobot') == '0.6.1'
    parent = Path(plan['parent_checkpoint'])
    assert digest(parent/'model.safetensors') == plan['parent_weights_sha256']
    config = json.loads((parent/'config.json').read_text())
    assert config['type'] == 'act' and config['chunk_size'] == 100
    assert config['input_features']['observation.state']['shape'] == [7]
    assert config['output_features']['action']['shape'] == [7]
    for camera in ('global', 'wrist'):
        assert config['input_features']['observation.images.'+camera]['shape'] == [3,480,640]
    data = Path(plan['dataset_root'])
    info = json.loads((data/'meta/info.json').read_text())
    assert info['total_episodes'] == 52 and info['total_frames'] == 20488 and info['fps'] == 20
    checked_files = 0
    for line in (HERE/'dataset.sha256').read_text().splitlines():
        expected, name = line.split(maxsplit=1)
        path = (data/name.lstrip('*')).resolve()
        assert path.is_relative_to(data.resolve())
        assert digest(path) == expected, name
        checked_files += 1
    import torch
    from safetensors import safe_open
    free = total = None
    if plan['device'] == 'cuda':
        assert torch.cuda.is_available(), 'Local CUDA unavailable; select the CPU device'
        free, total = torch.cuda.mem_get_info()
        assert free > 8*1024**3, 'Require at least8GiB free local GPU memory; no process will be stopped'
    tensors = 0
    with safe_open(parent/'model.safetensors', framework='pt', device='cpu') as model:
        for key in model.keys():
            assert torch.isfinite(model.get_tensor(key)).all(), key
            tensors += 1
    return {'parent_sha256':plan['parent_weights_sha256'], 'dataset_files_verified':checked_files,
        'model_tensors_finite':tensors, 'device':plan['device'],
        'gpu':torch.cuda.get_device_name() if plan['device'] == 'cuda' else None,
        'gpu_free_bytes':free, 'gpu_total_bytes':total, 'torch':torch.__version__,
        'official_config':official_config_check(plan), 'command_argv':command(plan),
        'parent_files_sha256':{p.name:digest(p) for p in parent.iterdir() if p.is_file()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--run', action='store_true')
    args = parser.parse_args()
    plan = json.loads((HERE/'plan.json').read_text())
    saved_root = Path(plan['local_root']).expanduser().resolve()
    local_root = Path(os.environ.get('PIPER_ACT_LOCAL_ROOT', str(saved_root))).expanduser().resolve()
    for key in ('local_root', 'local_job_dir', 'parent_checkpoint',
                'parent_local_checkpoint', 'dataset_root', 'output_dir'):
        plan[key] = str(local_root / Path(plan[key]).relative_to(saved_root))
    plan['device'] = os.environ.get('PIPER_ACT_DEVICE', plan['device'])
    report = preflight(plan)
    save(HERE/'preflight.json', report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('official_config','parent_files_sha256')}, ensure_ascii=False), flush=True)
    if not args.run:
        return
    status_path = HERE/'status.json'
    if status_path.exists():
        raise FileExistsError('This job has already been attempted; refusing duplicate training')
    status = {'state':'starting', 'started_at':datetime.now(timezone.utc).isoformat(),
        'output_dir':plan['output_dir'], 'parent_checkpoint':plan['parent_checkpoint'],
        'parent_sha256':plan['parent_weights_sha256'], 'automatic_deploy':False}
    save(status_path,status)
    log = HERE/'train.log'
    with log.open('x') as stream:
        process = subprocess.Popen(command(plan), stdout=stream, stderr=subprocess.STDOUT, cwd=HERE)
        status.update(state='running', pid=process.pid, log=str(log))
        save(status_path,status)
        code = process.wait()
    status.update(exit_code=code, finished_at=datetime.now(timezone.utc).isoformat())
    parent = Path(plan['parent_checkpoint'])
    unchanged = all(digest(parent/name) == expected for name,expected in report['parent_files_sha256'].items())
    status['parent_files_unchanged'] = unchanged
    final = Path(plan['output_dir'])/'checkpoints/last/pretrained_model'
    if code == 0 and (final/'model.safetensors').exists() and unchanged:
        try:
            step = json.loads((final.parent/'training_state/training_step.json').read_text())['step']
            assert step == plan['steps'], 'Final checkpoint step does not match the requested steps'
            import torch
            from safetensors import safe_open
            tensors = 0
            with safe_open(final/'model.safetensors', framework='pt', device='cpu') as model:
                for key in model.keys():
                    assert torch.isfinite(model.get_tensor(key)).all(), key
                    tensors += 1
            status.update(state='complete', final_checkpoint=str(final.resolve()),
                final_model_sha256=digest(final/'model.safetensors'),
                final_step=step, final_model_tensors_finite=tensors)
        except Exception as exc:
            status.update(state='failed', final_validation_error=str(exc))
    else:
        status['state'] = 'failed'
    save(status_path,status)
    print(json.dumps(status,ensure_ascii=False), flush=True)
    if status['state'] != 'complete':
        raise SystemExit(code or 1)


if __name__ == '__main__':
    main()

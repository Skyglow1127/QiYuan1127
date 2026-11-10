"""Offline SmolVLA/D1 integration and training smoke test. No hardware imports."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import resource
import time

import numpy as np
from PIL import Image
import torch
from lerobot.configs import FeatureType, PolicyFeature
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.smolvla.processor_smolvla import make_smolvla_pre_post_processors
from lerobot.policies.factory import make_pre_post_processors

HERE = Path(__file__).resolve().parent
ART = HERE / 'artifacts'
MODEL_ROOT = Path('/home/aup/模型库/策略模型/SmolVLA')
CHECKPOINT = MODEL_ROOT / 'd1_lift_smoke_20261025'

def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2))

def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def load_input():
    data = json.loads((ART / 'smoke_samples.json').read_text())
    stats = json.loads((ART / 'data_stats.json').read_text())
    tensor_stats = {
        feature: {key: torch.tensor(stats[name][key], dtype=torch.float32)
                  for key in ('mean', 'std', 'min', 'max')}
        for feature, name in [('observation.state', 'state'), ('action', 'action')]
    }
    overrides = []
    for feature, name in [('observation.state', 'state'), ('action', 'action')]:
        # A constant training coordinate carries no learned scale. Use one native
        # SDK unit rather than amplifying a harmless sensor offset by 1e6.
        for axis, raw_std in enumerate(stats[name]['raw_std']):
            if raw_std < 1e-6:
                tensor_stats[feature]['std'][axis] = 1.0
                overrides.append({'feature': feature, 'axis': axis, 'processor_std': 1.0,
                                  'reason': 'constant_training_axis_unit_scale_no_validation_fit'})
    write_json(ART / 'normalization_overrides.json', overrides)
    return data['samples'], tensor_stats

def make_batch(row, pre):
    observation = {
        'observation.state': torch.tensor(row['q'], dtype=torch.float32),
        'action': torch.tensor(row['actions'], dtype=torch.float32).unsqueeze(0),
        # Technical annotation of the recorded stage, not original language supervision.
        'task': 'Lift the grasped red cube.',
    }
    for cam in ('global', 'wrist'):
        im = np.asarray(Image.open(row['cameras'][cam]['path']).convert('RGB')).copy()
        observation[f'observation.images.{cam}'] = torch.from_numpy(im).permute(2, 0, 1).float() / 255
    batch = pre(observation)
    batch['action_is_pad'] = torch.tensor(row['is_pad'], dtype=torch.bool).unsqueeze(0)
    return batch

def fixed_noise(config):
    return torch.randn((1, config.chunk_size, config.max_action_dim),
                       generator=torch.Generator().manual_seed(37))

def infer(policy, batch, post):
    policy.eval()
    t = time.perf_counter()
    with torch.inference_mode():
        normalized = policy.predict_action_chunk(batch, noise=fixed_noise(policy.config))
        decoded = post(normalized.clone())
    elapsed = time.perf_counter() - t
    assert tuple(normalized.shape) == (1, 10, 7), normalized.shape
    assert torch.isfinite(normalized).all() and torch.isfinite(decoded).all()
    return normalized.cpu().numpy(), decoded.cpu().numpy(), elapsed

def configure():
    cfg = SmolVLAConfig.from_pretrained(MODEL_ROOT / 'base', local_files_only=True)
    cfg.device = 'cpu'
    cfg.load_vlm_weights = False
    cfg.vlm_model_name = str(MODEL_ROOT / 'vlm_processor')
    cfg.push_to_hub = False
    cfg.input_features = {
        'observation.state': PolicyFeature(type=FeatureType.STATE, shape=(7,)),
        **{f'observation.images.{cam}': PolicyFeature(type=FeatureType.VISUAL, shape=(3, 512, 512))
           for cam in ('global', 'wrist')},
    }
    cfg.output_features = {'action': PolicyFeature(type=FeatureType.ACTION, shape=(7,))}
    cfg.chunk_size = cfg.n_action_steps = 10
    cfg.num_steps = 2  # Deliberately reduced for software smoke; pretrained default is 10.
    cfg.freeze_vision_encoder = True
    cfg.train_expert_only = True
    cfg.train_state_proj = True
    return cfg

def train_smoke(steps):
    if CHECKPOINT.exists():
        raise FileExistsError(f'Refusing to overwrite {CHECKPOINT}')
    rows, stats = load_input()
    train_rows = [r for r in rows if r['split'] == 'train']
    val_row = next(r for r in rows if r['split'] == 'val')
    cfg = configure()
    print('Loading full pretrained SmolVLA weights strictly on CPU', flush=True)
    t = time.perf_counter()
    policy = SmolVLAPolicy.from_pretrained(MODEL_ROOT / 'base', config=cfg,
                                          local_files_only=True, strict=True)
    pre, post = make_smolvla_pre_post_processors(cfg, dataset_stats=stats)
    batch0 = make_batch(train_rows[0], pre)
    assert tuple(batch0['observation.state'].shape) == (1, 7)
    assert tuple(batch0['action'].shape) == (1, 10, 7)
    # Normalization must roundtrip D1 values, not use the base SO100 statistics.
    roundtrip = post(batch0['action'].clone())
    assert torch.allclose(roundtrip, torch.tensor(train_rows[0]['actions']).unsqueeze(0), atol=1e-4)
    state_roundtrip = batch0['observation.state'] * stats['observation.state']['std'] + stats['observation.state']['mean']
    assert torch.allclose(state_roundtrip, torch.tensor(train_rows[0]['q']).unsqueeze(0), atol=1e-4)
    report = {
        'status': 'RUNNING', 'scope': 'offline_software_and_short_lift_training_only',
        'robot_commands_sent': 0, 'camera_accessed': False, 'npu_used': False,
        'device': 'cpu', 'cpu_threads': torch.get_num_threads(),
        'versions': {p: importlib.metadata.version(p) for p in
                     ('lerobot', 'torch', 'torchvision', 'transformers', 'huggingface-hub')},
        'load_s': time.perf_counter() - t,
        'total_parameters': sum(p.numel() for p in policy.parameters()),
        'trainable_parameters': sum(p.numel() for p in policy.parameters() if p.requires_grad),
        'chunk_size': 10, 'action_dimensions': 7, 'sampling_steps': cfg.num_steps,
        'pretrained_default_sampling_steps': 10, 'state_action_roundtrip_passed': True,
        'input_sample_ids': [r['id'] for r in rows],
        'steps': [], 'complete_grasp_training': False,
        'claim_limit': 'No physical grasp, language grounding, generalization, or NPU performance claim.',
    }
    write_json(ART / 'train_results.json', report)
    before, _, inference_s = infer(policy, batch0, post)
    report['initial_inference_s'] = inference_s
    report['output_shape'] = list(before.shape)
    print(f'Inference passed: {before.shape}, {inference_s:.3f}s', flush=True)
    optimizer = torch.optim.AdamW([p for p in policy.parameters() if p.requires_grad],
                                 lr=1e-4, betas=(0.9, 0.95), eps=1e-8, weight_decay=1e-10)
    probe = policy.model.action_out_proj.weight
    probe_before = probe.detach().clone()
    noise = fixed_noise(cfg)
    flow_time = torch.tensor([0.5])
    policy.eval()
    with torch.no_grad():
        initial_loss, _ = policy(batch0, noise=noise, time=flow_time)
    report['fixed_input_loss_before'] = float(initial_loss)
    for step in range(steps):
        batch = make_batch(train_rows[step % len(train_rows)], pre)
        policy.train()
        optimizer.zero_grad(set_to_none=True)
        start = time.perf_counter()
        loss, metrics = policy(batch, noise=noise, time=flow_time)
        if not torch.isfinite(loss):
            raise RuntimeError('Non-finite loss')
        loss.backward()
        grads = [p.grad for p in policy.parameters() if p.requires_grad and p.grad is not None]
        if not grads or not all(torch.isfinite(g).all().item() for g in grads):
            raise RuntimeError('Missing or non-finite gradients')
        grad_norm = torch.nn.utils.clip_grad_norm_(
            [p for p in policy.parameters() if p.requires_grad], 10.0)
        if not torch.isfinite(grad_norm) or float(grad_norm) <= 0:
            raise RuntimeError('Zero or non-finite gradient norm')
        optimizer.step()
        del grads
        item = {'step': step + 1, 'sample_id': train_rows[step % len(train_rows)]['id'],
                'loss': float(loss.detach()), 'grad_norm_before_clip': float(grad_norm),
                'seconds': time.perf_counter() - start}
        report['steps'].append(item)
        write_json(ART / 'train_results.json', report)
        print(json.dumps(item), flush=True)
    report['action_head_max_parameter_change'] = float((probe - probe_before).abs().max().detach())
    assert report['action_head_max_parameter_change'] > 0
    optimizer.zero_grad(set_to_none=True)
    del optimizer
    policy.eval()
    with torch.no_grad():
        final_loss, _ = policy(batch0, noise=noise, time=flow_time)
        val_loss, _ = policy(make_batch(val_row, pre), noise=noise, time=flow_time)
    report['fixed_input_loss_after'] = float(final_loss)
    report['one_validation_window_loss'] = float(val_loss)
    normalized, decoded, secs = infer(policy, batch0, post)
    report['post_training_inference_s'] = secs
    np.savez(ART / 'expected_reload_outputs.npz', normalized=normalized, decoded=decoded)
    CHECKPOINT.mkdir(parents=True)
    policy.save_pretrained(CHECKPOINT)
    pre.save_pretrained(CHECKPOINT)
    post.save_pretrained(CHECKPOINT)
    write_json(CHECKPOINT / 'USAGE_RESTRICTIONS.json', {
        'purpose': 'Offline software smoke test only; short training on two lift fragments.',
        'physical_execution_validated': False, 'full_grasp_policy': False,
        'complete_demonstrations': 0, 'normalization': 'D1 lift training split only',
        'action_units': ['sdk_degree'] * 6 + ['uncalibrated_gripper_sdk_raw'],
        'original_dataset': '/home/aup/Camera/output/act_lift_pilot_20261024/data/examples.jsonl',
        'language_labels': 'Fixed stage label added for interface testing; no language grounding evaluation',
    })
    report['checkpoint'] = str(CHECKPOINT)
    report['checkpoint_sha256'] = sha256(CHECKPOINT / 'model.safetensors')
    report['peak_rss_mib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    report['status'] = 'TRAIN_SAVE_PASSED_RELOAD_PENDING'
    write_json(ART / 'train_results.json', report)
    print('Training and save passed; separate-process reload is still required.', flush=True)

def reload_smoke():
    rows, stats = load_input()
    row = next(r for r in rows if r['split'] == 'train')
    cfg = SmolVLAConfig.from_pretrained(CHECKPOINT, local_files_only=True)
    policy = SmolVLAPolicy.from_pretrained(CHECKPOINT, config=cfg, local_files_only=True, strict=True)
    pre, post = make_pre_post_processors(cfg, pretrained_path=CHECKPOINT)
    batch = make_batch(row, pre)
    state_roundtrip = batch['observation.state'] * stats['observation.state']['std'] + stats['observation.state']['mean']
    assert torch.allclose(state_roundtrip, torch.tensor(row['q']).unsqueeze(0), atol=1e-4)
    normalized, decoded, secs = infer(policy, batch, post)
    expected = np.load(ART / 'expected_reload_outputs.npz', allow_pickle=False)
    errors = {}
    for name, actual in [('normalized', normalized), ('decoded', decoded)]:
        errors[name] = float(np.max(np.abs(actual - expected[name])))
        np.testing.assert_allclose(actual, expected[name], rtol=1e-5, atol=1e-5)
    report = {'status': 'PASSED', 'separate_process_reload': True, 'max_absolute_errors': errors,
              'state_roundtrip_passed': True,
              'inference_s': secs, 'output_shape': list(decoded.shape), 'robot_commands_sent': 0,
              'npu_used': False, 'checkpoint_sha256': sha256(CHECKPOINT / 'model.safetensors'),
              'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024}
    write_json(ART / 'reload_results.json', report)
    print(json.dumps(report, indent=2), flush=True)

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--phase', choices=('train', 'reload'), required=True)
    p.add_argument('--steps', type=int, default=3)
    args = p.parse_args()
    if not 1 <= args.steps <= 10:
        p.error('This smoke script permits 1–10 steps; it is not the full training launcher.')
    torch.manual_seed(37)
    torch.set_num_threads(4)
    torch.set_num_interop_threads(1)
    ART.mkdir(exist_ok=True)
    if args.phase == 'train':
        train_smoke(args.steps)
    else:
        reload_smoke()

if __name__ == '__main__':
    main()

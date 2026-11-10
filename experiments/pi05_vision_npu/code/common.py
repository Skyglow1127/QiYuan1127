"""Read-only adapters for the installed, unmodified LeRobot PI05/PEFT stack."""
from pathlib import Path
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
OLD = Path('/home/aup/Documents/Codex/2026-10-30/wo-y/outputs')
BASE = Path('/home/aup/Lerobot/models/PI05/pi05_base')
ADAPTER = OLD / 'piper_pi05_cloud_backup_20261030/runs/pi05_train_20261030_091725/checkpoints/020000/pretrained_model'
TOKENIZER = OLD / 'piper_pi05_cloud_backup_20261030/models/paligemma_tokenizer'
RECORDED = ROOT / 'results/original_gpu_baseline/recorded_observation.npz'

def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')

def load_policy(recorded=RECORDED):
    import numpy as np
    import torch
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies import make_pre_post_processors
    from peft import PeftConfig, PeftModel, get_peft_model_state_dict
    from safetensors.torch import load_file
    from pi05_low_memory import load_base_streaming
    torch.set_num_threads(8)
    torch.cuda.set_per_process_memory_fraction(0.85)
    cfg = PreTrainedConfig.from_pretrained(str(ADAPTER), local_files_only=True,
        cli_overrides=['--device=cuda', '--dtype=bfloat16', '--gradient_checkpointing=false',
                       '--compile_model=false', '--push_to_hub=false'])
    cfg.pretrained_path = str(BASE)
    pre, post = make_pre_post_processors(policy_cfg=cfg, pretrained_path=str(ADAPTER),
        preprocessor_overrides={'tokenizer_processor': {'tokenizer_name': str(TOKENIZER)},
                                'device_processor': {'device': 'cuda'}},
        postprocessor_overrides={'device_processor': {'device': 'cpu'}})
    with np.load(recorded, allow_pickle=False) as z:
        sample = {k: torch.from_numpy(z[k].copy()) for k in cfg.input_features}
        sample['task'] = str(z['task'].item())
    batch = pre(sample)
    started = time.perf_counter()
    base, loading = load_base_streaming(BASE, cfg)
    pc = PeftConfig.from_pretrained(str(ADAPTER), local_files_only=True)
    pc.base_model_name_or_path = str(BASE)
    policy = PeftModel.from_pretrained(base, str(ADAPTER), config=pc,
                                     local_files_only=True, is_trainable=False)
    policy.eval().requires_grad_(False)
    saved = load_file(str(ADAPTER / 'adapter_model.safetensors'), device='cpu')
    loaded = get_peft_model_state_dict(policy, adapter_name='default')
    assert set(saved) == set(loaded), 'LoRA keys differ'
    for k, v in saved.items():
        assert torch.equal(v.to(loaded[k].dtype), loaded[k].detach().cpu()), k
    loading['adapter_tensors_verified'] = len(saved)
    loading['base_plus_adapter_load_seconds'] = time.perf_counter() - started
    return policy, base, cfg, batch, post, sample, loading

def stats(reference, actual):
    import numpy as np
    a, b = np.asarray(reference, dtype=np.float64), np.asarray(actual, dtype=np.float64)
    d = b - a
    return dict(shape=list(b.shape), reference_dtype=str(np.asarray(reference).dtype),
        actual_dtype=str(np.asarray(actual).dtype), all_finite=bool(np.isfinite(b).all()),
        max_abs=float(np.abs(d).max()), mean_abs=float(np.abs(d).mean()),
        rmse=float(np.sqrt(np.mean(d*d))),
        cosine=float(np.dot(a.ravel(), b.ravel()) / (np.linalg.norm(a)*np.linalg.norm(b))))

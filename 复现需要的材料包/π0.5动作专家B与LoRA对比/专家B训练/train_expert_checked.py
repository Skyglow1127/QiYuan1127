#!/usr/bin/env python3
"""Audited batch-8 expert-only entry around the unchanged LeRobot 0.6.1 train loop.

No PEFT, no robot imports, no changes to installed packages. Hooks are scoped to
this process. The companion loader preserves the inspected official mixed
dtypes, strictly maps all checkpoint tensors and reconstructs six buffers.
"""
from __future__ import annotations

from collections import defaultdict
from functools import wraps
from importlib.metadata import version
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
import types

BASE_SHA256 = "0eb11ca9587678c1d2ef8cf32807c29f8ce53a2bfdfc1aa4a4c96f16fca59b0f"
EXPERT_PREFIX = "model.paligemma_with_expert.gemma_expert."
VLM_PREFIX = "model.paligemma_with_expert.paligemma."
PROJECTIONS = ("action_in_proj", "action_out_proj", "time_mlp_in", "time_mlp_out")
STATE = {}


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(name, value):
    path = STATE["audit_dir"] / name
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, default=str) + "\n")
    temporary.replace(path)


def group_name(name):
    if name.startswith(VLM_PREFIX):
        return "vlm"
    if name.startswith(EXPERT_PREFIX):
        return "gemma_expert"
    for projection in PROJECTIONS:
        if name.startswith(f"model.{projection}."):
            return projection
    raise RuntimeError(f"Unrecognized PI05 parameter: {name}")


def tensor_hash(value):
    import torch
    value = value.detach().contiguous().cpu()
    return hashlib.sha256(value.view(torch.uint8).numpy().tobytes()).hexdigest()


def module_hashes(policy):
    hashes = defaultdict(hashlib.sha256)
    for name, parameter in policy.named_parameters():
        digest = hashes[group_name(name)]
        digest.update(name.encode())
        digest.update(str(parameter.dtype).encode())
        digest.update(str(tuple(parameter.shape)).encode())
        digest.update(bytes.fromhex(tensor_hash(parameter)))
    return {name: digest.hexdigest() for name, digest in hashes.items()}


def expert_bfloat16_hashes(policy):
    import torch
    return {name: tensor_hash(parameter) for name, parameter in policy.named_parameters()
            if name.startswith(EXPERT_PREFIX + "model.") and parameter.dtype == torch.bfloat16}


def checkpoint_space_guard(target, required):
    target = Path(target)
    while not target.exists():
        target = target.parent
    free = shutil.disk_usage(target).free
    if free < required + 512 * 1024**2:
        raise RuntimeError(f"Insufficient checkpoint space on {target}: "
                           f"free={free}, required={required} + 512 MiB reserve")
    return free


def optimizer_state_audit(policy, optimizer):
    import torch
    by_id = {id(parameter): (name, parameter) for name, parameter in policy.named_parameters()}
    names, numel, size = [], 0, 0
    for parameter, state in optimizer.state.items():
        if id(parameter) not in by_id or not parameter.requires_grad:
            raise RuntimeError("Optimizer state contains unknown/frozen parameter")
        names.append(by_id[id(parameter)][0])
        numel += parameter.numel()
        size += sum(value.numel() * value.element_size() for value in state.values()
                    if torch.is_tensor(value))
    return {"parameter_names_with_state": names, "parameters_with_state_numel": numel,
            "state_tensor_bytes": size, "last_gradient_none_trainable_names": STATE.get("last_none_gradient_names")}


def parameter_audit(policy, optimizer=None):
    names = dict(policy.named_parameters())
    records, totals = [], defaultdict(lambda: {"tensors": 0, "numel": 0, "bytes": 0})
    for name, parameter in names.items():
        group = group_name(name)
        expected_trainable = group != "vlm"
        if parameter.requires_grad != expected_trainable:
            raise RuntimeError(f"Incorrect requires_grad for {name}: {parameter.requires_grad}")
        if "lora_" in name.lower() or "peft" in name.lower():
            raise RuntimeError(f"PEFT parameter found: {name}")
        record = {"name": name, "group": group, "numel": parameter.numel(),
                  "dtype": str(parameter.dtype), "shape": list(parameter.shape),
                  "requires_grad": parameter.requires_grad}
        records.append(record)
        for category in (group, "trainable" if expected_trainable else "frozen", "all"):
            totals[category]["tensors"] += 1
            totals[category]["numel"] += parameter.numel()
            totals[category]["bytes"] += parameter.numel() * parameter.element_size()
    if len(names) != 813 or totals["trainable"]["numel"] != 693_422_112:
        raise RuntimeError(f"Unexpected inspected PI05 parameter structure: {dict(totals)}")
    if hasattr(policy, "peft_config") or type(policy).__name__ != "PI05Policy":
        raise RuntimeError(f"Expected unwrapped PI05Policy, got {type(policy)}")
    result = {"parameters": records, "totals": dict(totals), "peft_present": False}
    if optimizer is not None:
        by_id = {id(parameter): name for name, parameter in names.items()}
        actual = [id(parameter) for group in optimizer.param_groups for parameter in group["params"]]
        expected = {id(parameter) for parameter in names.values() if parameter.requires_grad}
        if len(actual) != len(set(actual)) or set(actual) != expected:
            raise RuntimeError("Optimizer parameter set differs from exact expert/projection set")
        result["optimizer_parameter_names"] = [by_id[key] for key in actual]
        result["optimizer_excludes_frozen_vlm"] = True
    return result


def validate_config(cfg):
    if cfg.peft is not None or cfg.policy.use_peft:
        raise RuntimeError("B requires peft=null AND policy.use_peft=false")
    expected = {"train_expert_only": True, "freeze_vision_encoder": True,
                "use_relative_actions": False, "chunk_size": 50,
                "n_action_steps": 10, "num_inference_steps": 10, "n_obs_steps": 1,
                "gradient_checkpointing": True, "compile_model": False,
                "dtype": "bfloat16"}
    for key, value in expected.items():
        if getattr(cfg.policy, key) != value:
            raise RuntimeError(f"Unexpected {key}: {getattr(cfg.policy, key)} != {value}")
    if cfg.batch_size != 8 or cfg.seed != 1000 or cfg.dataset.eval_split != 0.1:
        raise RuntimeError("B main comparison requires batch=8, seed=1000, eval_split=0.1")
    if not cfg.use_policy_training_preset:
        raise RuntimeError("Use the inspected PI05 optimizer/scheduler preset")
    if cfg.env is not None or cfg.sample_weighting is not None:
        raise RuntimeError("This entry is offline imitation only")
    if os.environ.get("WORLD_SIZE", "1") != "1":
        raise RuntimeError("This audited entry currently supports one GPU/process")
    root = Path(os.environ["PI05_EXPERIMENT_ROOT"]).resolve(strict=True)
    output = Path(cfg.output_dir).resolve()
    if not output.is_relative_to(root) or output == root:
        raise RuntimeError(f"Output must be a new child of isolated B experiment root: {root}")
    if cfg.resume:
        checkpoint = Path(cfg.checkpoint_path).resolve(strict=True)
        if not checkpoint.is_relative_to(root):
            raise RuntimeError("Resume checkpoint must belong to isolated model B root")
        STATE["step"] = json.loads((checkpoint / "training_state/training_step.json").read_text())["step"]
    else:
        expected_base = Path(os.environ["PI05_BASE_MODEL_PATH"]).resolve(strict=True)
        if Path(cfg.policy.pretrained_path).resolve(strict=True) != expected_base:
            raise RuntimeError("Fresh B must start at original base, not A or smoke weights")
        STATE["step"] = 0
    STATE["start_step"] = STATE["step"]
    STATE["cfg"] = cfg
    STATE["audit_dir"] = output / "audit"
    STATE["audit_dir"].mkdir(parents=True, exist_ok=True)
    write_json("resolved_configuration.json", cfg.to_dict())


def install_hooks():
    import torch
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy
    from lerobot.processor import TokenizerProcessorStep
    from lerobot.scripts import lerobot_train as training
    from pi05_low_memory import load_base_streaming

    original_datasets = training.make_train_eval_datasets

    @wraps(original_datasets)
    def datasets(cfg):
        validate_config(cfg)
        train, evaluation = original_datasets(cfg)
        if evaluation is None:
            raise RuntimeError("Missing held-out validation episodes")
        train_ids, eval_ids = list(train.episodes), list(evaluation.episodes)
        if train_ids != list(range(46)) or eval_ids != list(range(46, 52)):
            raise RuntimeError(f"A/B split mismatch: train={train_ids}, eval={eval_ids}")
        if train.num_frames != 18_231 or evaluation.num_frames != 2_257:
            raise RuntimeError("A/B frame count mismatch")
        root = Path(cfg.dataset.root)
        write_json("dataset_split.json", {"train_episode_ids": train_ids,
                   "validation_episode_ids": eval_ids, "train_frames": train.num_frames,
                   "validation_frames": evaluation.num_frames,
                   "dataset_root": str(root),
                   "meta_sha256": {name: sha256_file(root / "meta" / name)
                                   for name in ("info.json", "stats.json")}})
        return train, evaluation

    training.make_train_eval_datasets = datasets

    def strict_loader(cls, pretrained_name_or_path, *, config=None, **kwargs):
        if config is None:
            raise RuntimeError("Expected explicit dataset-derived PI05 config")
        source = Path(pretrained_name_or_path).resolve(strict=True)
        cfg = STATE["cfg"]
        if config.use_peft or (source / "adapter_config.json").exists():
            raise RuntimeError("B cannot load an adapter checkpoint")
        if not cfg.resume:
            digest = sha256_file(source / "model.safetensors")
            if digest != BASE_SHA256:
                raise RuntimeError(f"Original PI05 base SHA256 mismatch: {digest}")
        else:
            digest = sha256_file(source / "model.safetensors")
        policy, summary = load_base_streaming(source, config)
        # The inference loader freezes all parameters. Restore the official
        # train_expert_only rule, then audit exact names, never a heuristic regex.
        policy.requires_grad_(True)
        policy.model.paligemma_with_expert._set_requires_grad()
        policy.train()
        summary.update({"source": str(source), "source_model_sha256": digest,
                        "fresh_original_base": not cfg.resume})
        write_json("strict_load.json", summary)
        write_json("parameter_scope.json", parameter_audit(policy))
        STATE["policy"] = policy
        return policy

    PI05Policy.from_pretrained = classmethod(strict_loader)

    original_tokenizer_init = TokenizerProcessorStep.__post_init__

    @wraps(original_tokenizer_init)
    def tokenizer_init(self):
        if self.tokenizer is None and self.tokenizer_name == "google/paligemma-3b-pt-224":
            self.tokenizer_name = str(Path(os.environ["PI05_TOKENIZER_PATH"]).resolve(strict=True))
        original_tokenizer_init(self)

    TokenizerProcessorStep.__post_init__ = tokenizer_init
    original_optimizer = training.make_optimizer_and_scheduler

    @wraps(original_optimizer)
    def optimizer_factory(cfg, policy):
        # Official PI05 get_optim_params returns ALL parameters. Restrict this
        # instance so frozen VLM tensors never enter AdamW parameter groups.
        policy.get_optim_params = types.MethodType(
            lambda self: (parameter for parameter in self.parameters() if parameter.requires_grad), policy)
        optimizer, scheduler = original_optimizer(cfg, policy)
        write_json("optimizer_scope.json", parameter_audit(policy, optimizer))
        config = cfg.scheduler
        effective_decay = min(cfg.steps, config.num_decay_steps)
        effective_warmup = int(config.num_warmup_steps * cfg.steps / config.num_decay_steps) \
            if cfg.steps < config.num_decay_steps else config.num_warmup_steps
        write_json("scheduler_actual.json", {"implementation": type(config).__name__,
                   "configured": {"warmup_steps": config.num_warmup_steps,
                                  "decay_steps": config.num_decay_steps,
                                  "peak_lr": config.peak_lr, "decay_lr": config.decay_lr},
                   "effective_warmup_steps": effective_warmup,
                   "effective_decay_steps": effective_decay,
                   "initial_optimizer_lr": optimizer.param_groups[0]["lr"],
                   "note": "Unchanged official scheduler automatically scales shorter runs."})

        def gradient_guard(optim, args, kwargs):
            finite = []
            none_names = []
            for name, parameter in policy.named_parameters():
                if not parameter.requires_grad and parameter.grad is not None:
                    raise RuntimeError(f"Frozen VLM gradient found: {name}")
                if parameter.grad is not None:
                    finite.append(torch.isfinite(parameter.grad).all())
                elif parameter.requires_grad:
                    none_names.append(name)
            if not finite or not torch.stack(finite).all().item():
                raise RuntimeError("Nonfinite or missing trainable gradients; stopped before optimizer.step")
            STATE["last_none_gradient_names"] = none_names

        optimizer.register_step_pre_hook(gradient_guard)
        original_forward = policy.forward

        @wraps(original_forward)
        def forward_checked(*args, **kwargs):
            loss, details = original_forward(*args, **kwargs)
            if not torch.isfinite(loss).all().item():
                raise RuntimeError("Nonfinite loss; stopped before backward/optimizer")
            if not policy.training:
                bucket = STATE.setdefault("eval_losses", {})
                bucket.setdefault(STATE["step"], []).append(float(loss.item()))
            return loss, details

        policy.forward = forward_checked
        return optimizer, scheduler

    training.make_optimizer_and_scheduler = optimizer_factory
    original_update = training.update_policy

    @wraps(original_update)
    def update(*args, **kwargs):
        accelerator = kwargs.get("accelerator")
        policy = accelerator.unwrap_model(args[1])
        first = "initial_hashes" not in STATE
        if first:
            STATE["initial_hashes"] = module_hashes(policy)
            STATE["initial_expert_bfloat16_hashes"] = expert_bfloat16_hashes(policy)
            write_json("weights_before_first_update.json", STATE["initial_hashes"])
        started = time.perf_counter()
        lr_used = float(args[3].param_groups[0]["lr"])
        result = original_update(*args, **kwargs)
        STATE["step"] += 1
        metrics = result[0]
        # AverageMeter tracks val separately from the logging-window average.
        loss = float(metrics.loss.val)
        grad = float(metrics.grad_norm.val)
        if not math.isfinite(loss) or not math.isfinite(grad):
            raise RuntimeError("Nonfinite scalar update metrics")
        step_info = {"step": STATE["step"], "loss": loss, "grad_norm": grad,
                     "lr_used_for_this_optimizer_step": lr_used,
                     "lr_after_scheduler_step": float(args[3].param_groups[0]["lr"]),
                     "update_seconds": time.perf_counter() - started,
                     "cuda_peak_allocated_bytes": torch.cuda.max_memory_allocated(),
                     "cuda_peak_reserved_bytes": torch.cuda.max_memory_reserved()}
        with (STATE["audit_dir"] / "actual_steps.jsonl").open("a") as stream:
            stream.write(json.dumps(step_info) + "\n")
        if first:
            after = module_hashes(policy)
            changed = {name: after[name] != digest for name, digest in STATE["initial_hashes"].items()}
            audit = {"before": STATE["initial_hashes"], "after": after, "changed": changed,
                     "gradient_none_trainable_names": STATE.get("last_none_gradient_names"),
                     "optimizer_state": optimizer_state_audit(policy, args[3]),
                     "step": step_info}
            write_json("first_optimizer_update.json", audit)
            if changed["vlm"]:
                raise RuntimeError("Complete frozen VLM changed during first update")
            print("AUDIT PASSED: first-step finite gradients and complete frozen VLM unchanged; "
                  "original-weight update audit continues to checkpoint boundary.", flush=True)
        return result

    training.update_policy = update
    original_save = training.save_checkpoint

    @wraps(original_save)
    def save(*args, **kwargs):
        checkpoint = Path(kwargs["checkpoint_dir"])
        policy = kwargs["policy"]
        cfg = kwargs["cfg"]
        if not checkpoint.resolve().is_relative_to(Path(cfg.output_dir).resolve() / "checkpoints"):
            raise RuntimeError("Checkpoint is outside isolated B run")
        before = module_hashes(policy)
        if before["vlm"] != STATE["initial_hashes"]["vlm"]:
            raise RuntimeError("Complete frozen VLM hash changed before checkpoint")
        changed_modules = {name: digest != STATE["initial_hashes"][name]
                           for name, digest in before.items()}
        if STATE["start_step"] == 0 and kwargs["step"] >= 100 \
                and (not changed_modules["gemma_expert"] or not all(changed_modules[p] for p in PROJECTIONS)):
            raise RuntimeError(f"Original expert/projection groups did not update by boundary: {changed_modules}")
        actual_bf16 = expert_bfloat16_hashes(policy)
        changed_bf16 = [name for name, digest in actual_bf16.items()
                        if digest != STATE["initial_expert_bfloat16_hashes"][name]]
        write_json(f"expert_bfloat16_updates_{kwargs['step']:06d}.json", {
            "step": kwargs["step"], "compared_tensors": len(actual_bf16),
            "changed_tensors": changed_bf16, "original_expert_bfloat16_updated": bool(changed_bf16)})
        if STATE["start_step"] == 0 and kwargs["step"] >= 100 and not changed_bf16:
            raise RuntimeError("No original BF16 expert layer weight changed by checkpoint boundary")
        estimated = sum(p.numel() * p.element_size() for p in policy.parameters())
        estimated += sum(value.numel() * value.element_size()
                         for state in kwargs["optimizer"].state.values()
                         for value in state.values() if torch.is_tensor(value))
        free_before = checkpoint_space_guard(checkpoint.parent, estimated)
        original_save(*args, **kwargs)
        from safetensors import safe_open
        path = checkpoint / "pretrained_model/model.safetensors"
        parameters = dict(policy.named_parameters())
        with safe_open(str(path), framework="pt", device="cpu") as saved:
            if set(saved.keys()) != set(parameters):
                raise RuntimeError("Saved full-model keys differ from live policy")
            for name, parameter in parameters.items():
                value = saved.get_tensor(name)
                if value.dtype != parameter.dtype or tuple(value.shape) != tuple(parameter.shape) \
                        or tensor_hash(value) != tensor_hash(parameter):
                    raise RuntimeError(f"Saved tensor does not reload exactly: {name}")
        artifacts = {str(p.relative_to(checkpoint)): {"bytes": p.stat().st_size,
                     "sha256": sha256_file(p)} for p in checkpoint.rglob("*") if p.is_file()}
        receipt = {"checkpoint": str(checkpoint), "step": kwargs["step"],
                   "exact_cpu_tensor_reload_verified": True,
                   "complete_frozen_vlm_sha256_unchanged": True,
                   "module_hashes": before, "artifacts": artifacts,
                   "changed_modules_since_run_start": changed_modules,
                   "optimizer_state": optimizer_state_audit(policy, kwargs["optimizer"]),
                   "free_bytes_before_save": free_before,
                   "total_checkpoint_bytes": sum(p["bytes"] for p in artifacts.values())}
        write_json(f"checkpoint_{kwargs['step']:06d}.json", receipt)
        eval_losses = STATE.get("eval_losses", {}).pop(kwargs["step"], [])
        best_path = os.environ.get("PI05_BEST_MODEL_DIR")
        if eval_losses:
            eval_loss = sum(eval_losses) / len(eval_losses)
            write_json(f"validation_{kwargs['step']:06d}.json", {
                "step": kwargs["step"], "eval_loss": eval_loss,
                "batches": len(eval_losses), "same_official_batch_mean": True})
            if best_path:
                best = Path(best_path).resolve()
                previous_record = best.parent / "best_model_receipt.json"
                previous = json.loads(previous_record.read_text()) if previous_record.exists() else None
                if best.name != "best_model" or (best.exists() and previous is None):
                    raise RuntimeError("Best destination must be a new best_model or previously receipted B best model")
                if previous and not Path(previous["source_checkpoint"]).resolve().is_relative_to(
                        Path(os.environ["PI05_EXPERIMENT_ROOT"]).resolve()):
                    raise RuntimeError("Existing best receipt does not belong to this B experiment")
                if previous is None or eval_loss < previous["eval_loss"]:
                    model_dir = checkpoint / "pretrained_model"
                    required = sum(p.stat().st_size for p in model_dir.rglob("*") if p.is_file())
                    checkpoint_space_guard(best.parent, required)
                    stage = best.with_name(best.name + ".staging")
                    old = best.with_name(best.name + ".previous")
                    if stage.exists() or old.exists():
                        raise RuntimeError("Unresolved B best-model staging directory; inspect before replacing")
                    stage.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copytree(model_dir, stage)
                    for name, asset in artifacts.items():
                        relative = Path(name)
                        if relative.parts[0] == "pretrained_model":
                            copied = stage.joinpath(*relative.parts[1:])
                            if sha256_file(copied) != asset["sha256"]:
                                raise RuntimeError(f"Best-model copy SHA mismatch: {name}")
                    if best.exists():
                        best.rename(old)
                    try:
                        stage.rename(best)
                    except BaseException:
                        if old.exists() and not best.exists():
                            old.rename(best)
                        raise
                    best_receipt = {"model_dir": str(best), "step": kwargs["step"],
                                    "eval_loss": eval_loss, "source_checkpoint": str(checkpoint),
                                    "total_bytes": required, "sha256_verified": True,
                                    "source_model_sha256": artifacts["pretrained_model/model.safetensors"]["sha256"]}
                    temporary = previous_record.with_suffix(".tmp")
                    temporary.write_text(json.dumps(best_receipt, indent=2) + "\n")
                    temporary.replace(previous_record)
                    if old.exists():
                        shutil.rmtree(old)
        # Delete only checkpoints created and verified by this B entry. Existing
        # model A/base paths can never pass the path and receipt checks above.
        keep = int(os.environ.get("PI05_KEEP_CHECKPOINTS", "1"))
        if keep < 1:
            raise RuntimeError("PI05_KEEP_CHECKPOINTS must be >=1")
        verified = [p for p in checkpoint.parent.iterdir() if p.is_dir() and not p.is_symlink()
                    and p.name.isdigit() and (STATE["audit_dir"] / f"checkpoint_{int(p.name):06d}.json").exists()]
        removed = []
        for obsolete in sorted(verified, key=lambda p: int(p.name), reverse=True)[keep:]:
            if obsolete != checkpoint:
                shutil.rmtree(obsolete)
                removed.append(str(obsolete))
        write_json("checkpoint_retention.json", {"keep_latest": keep, "removed": removed,
                   "current_verified_checkpoint": str(checkpoint),
                   "audit_receipts_preserved": True})
        print(f"AUDIT PASSED: complete checkpoint {kwargs['step']} reloads exactly; "
              f"{receipt['total_checkpoint_bytes'] / 2**30:.2f} GiB including resume state.", flush=True)

    training.save_checkpoint = save


def main():
    if version("lerobot") != "0.6.1":
        raise RuntimeError("This audited entry requires lerobot==0.6.1")
    for key in ("PI05_EXPERIMENT_ROOT", "PI05_BASE_MODEL_PATH", "PI05_TOKENIZER_PATH"):
        if not os.environ.get(key):
            raise RuntimeError(f"Missing required environment variable: {key}")
    install_hooks()
    from lerobot.scripts.lerobot_train import main as train_main
    train_main()


if __name__ == "__main__":
    main()

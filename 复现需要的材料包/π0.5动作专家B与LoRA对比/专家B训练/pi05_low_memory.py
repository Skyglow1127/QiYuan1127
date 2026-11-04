"""Strict, tensor-by-tensor PI05 base loading without FP32 CPU initialization.

Targets the inspected LeRobot 0.6.1 architecture. Original package and backups
are not modified. Unknown state or nonpersistent buffers fail closed.
"""
from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
import time


@dataclass(frozen=True)
class Source:
    key: str

    def clone(self):
        # The official key fixer clones lm_head into the input embedding.
        # Materialize these as two independent device tensors during loading.
        return Source(self.key)


def build_plan(base_dir: Path, config):
    import torch
    from accelerate import init_empty_weights
    from safetensors import safe_open
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy

    previous_device = config.device
    try:
        config.device = 'meta'
        with init_empty_weights():
            model = PI05Policy(config)
    finally:
        config.device = previous_device
    model.requires_grad_(False)
    expected = model.state_dict()
    with safe_open(str(base_dir / 'model.safetensors'), framework='pt', device='cpu') as saved:
        source_shapes = {key: tuple(saved.get_slice(key).get_shape()) for key in saved.keys()}
    original_markers = {key: Source(key) for key in source_shapes}
    # Fresh official base has 812 tensors and needs the official key fixer.
    # A full B checkpoint already has the exact 813 native keys: preserve each
    # stored tensor directly rather than cloning lm_head over a saved embedding.
    native_checkpoint = set(source_shapes) == set(expected)
    fixed = original_markers if native_checkpoint else model._fix_pytorch_state_dict_keys(original_markers, config)
    mapping = {}
    for key, marker in fixed.items():
        destination = key if key.startswith('model.') else 'model.' + key
        if destination in mapping:
            raise RuntimeError(f'Checkpoint key collision: {destination}')
        mapping[destination] = marker.key
    missing, unexpected = set(expected) - set(mapping), set(mapping) - set(expected)
    mismatched = {key: [source_shapes[source], tuple(expected[key].shape)]
                  for key, source in mapping.items() if key in expected
                  and source_shapes[source] != tuple(expected[key].shape)}
    if missing or unexpected or mismatched:
        raise RuntimeError(f'Invalid base state: missing={sorted(missing)}, '
                           f'unexpected={sorted(unexpected)}, shape_mismatch={mismatched}')
    aliases = defaultdict(list)
    for key, parameter in model.named_parameters(remove_duplicate=False):
        aliases[id(parameter)].append(key)
    if any(len(keys) != 1 for keys in aliases.values()):
        raise RuntimeError('Unexpected tied parameters; this loader requires the inspected untied architecture.')
    buffer_specs = {}
    for name, buffer in model.named_buffers(remove_duplicate=False):
        parent, leaf = name.rsplit('.', 1)
        module = model.get_submodule(parent)
        if not buffer.is_meta:
            raise RuntimeError(f'Unexpected non-meta initialization buffer: {name}')
        if leaf == 'position_ids' and parent.endswith('vision_model.embeddings'):
            if buffer.ndim != 2 or buffer.shape[0] != 1 or buffer.dtype != torch.int64:
                raise RuntimeError(f'Unexpected vision positions: {name}')
            kind = 'vision_positions'
        elif leaf == 'embed_scale' and parent.endswith('language_model.embed_tokens'):
            if buffer.ndim != 0 or not hasattr(module, 'scalar_embed_scale'):
                raise RuntimeError(f'Unexpected embedding scale: {name}')
            kind = 'embedding_scale'
        elif leaf in ('inv_freq', 'original_inv_freq') and parent.endswith('rotary_emb'):
            if not hasattr(module, 'config') or buffer.ndim != 1:
                raise RuntimeError(f'Unexpected rotary buffer: {name}')
            kind = 'rotary'
        else:
            raise RuntimeError(f'Unknown nonpersistent buffer: {name}')
        buffer_specs[name] = {'shape': tuple(buffer.shape), 'dtype': buffer.dtype, 'kind': kind}
    if len(buffer_specs) != 6:
        raise RuntimeError(f'Expected six nonpersistent buffers, found {len(buffer_specs)}.')
    specs = {key: {'shape': tuple(value.shape), 'dtype': value.dtype}
             for key, value in expected.items()}
    byte_counts = defaultdict(int)
    for value in expected.values():
        byte_counts[str(value.dtype)] += value.numel() * value.element_size()
    summary = {'source_tensors': len(source_shapes), 'destination_tensors': len(specs),
               'native_complete_checkpoint': native_checkpoint,
               'parameter_count': sum(p.numel() for p in model.parameters()),
               'parameter_bytes': sum(byte_counts.values()), 'bytes_by_dtype': dict(byte_counts),
               'nonpersistent_buffers': {name: {'shape': list(spec['shape']),
                                               'dtype': str(spec['dtype']), 'kind': spec['kind']}
                                         for name, spec in buffer_specs.items()},
               'missing_keys': [], 'unexpected_keys': [], 'shape_mismatches': {}}
    return model, mapping, specs, buffer_specs, summary


def load_base_streaming(base_dir: Path, config):
    import torch
    from accelerate.utils import set_module_tensor_to_device
    from safetensors import safe_open

    started = time.monotonic()
    target_device = torch.device(config.device)
    if target_device.type != 'cuda':
        raise ValueError('The streaming path is validated for a cuda API device (NVIDIA or ROCm).')
    if not torch.cuda.is_available():
        raise RuntimeError('No usable GPU device is available.')
    model, mapping, specs, buffers, summary = build_plan(Path(base_dir), config)
    free_bytes, _ = torch.cuda.mem_get_info(target_device)
    if summary['parameter_bytes'] >= free_bytes:
        raise RuntimeError('GPU free memory is smaller than the base parameters alone.')
    source_targets = defaultdict(list)
    for destination, source in mapping.items():
        source_targets[source].append(destination)
    print(f"Streaming {summary['source_tensors']} saved tensors into "
          f"{summary['destination_tensors']} parameters; "
          f"{summary['parameter_bytes'] / 2**30:.2f} GiB of mixed-precision parameters.", flush=True)
    with torch.no_grad(), safe_open(str(Path(base_dir) / 'model.safetensors'),
                                    framework='pt', device='cpu') as saved:
        for index, (source, destinations) in enumerate(source_targets.items(), 1):
            raw = saved.get_tensor(source)
            for name in destinations:
                # Casting on device avoids an additional full CPU BF16 tensor.
                value = raw.to(device=target_device, dtype=specs[name]['dtype'])
                set_module_tensor_to_device(model, name, target_device,
                                            value=value, clear_cache=False)
                del value
            del raw
            if index % 64 == 0 or index == len(source_targets):
                print(f'Base tensors loaded: {index}/{len(source_targets)}', flush=True)

    rotary_references = {}
    for name, spec in buffers.items():
        parent, leaf = name.rsplit('.', 1)
        module = model.get_submodule(parent)
        if spec['kind'] == 'vision_positions':
            value = torch.arange(spec['shape'][1], dtype=torch.int64).expand(spec['shape'])
        elif spec['kind'] == 'embedding_scale':
            value = torch.tensor(module.scalar_embed_scale, dtype=torch.float32)
        else:
            if parent not in rotary_references:
                rotary_references[parent] = type(module)(module.config, device='cpu')
            value = getattr(rotary_references[parent], leaf)
        if tuple(value.shape) != spec['shape']:
            raise RuntimeError(f'Reconstructed buffer shape mismatch: {name}')
        value = value.to(device=target_device, dtype=spec['dtype'])
        set_module_tensor_to_device(model, name, target_device,
                                    value=value, clear_cache=False)
    del rotary_references

    actual = dict(model.named_parameters())
    for name, spec in specs.items():
        value = actual[name]
        if value.is_meta or value.device.type != target_device.type or value.dtype != spec['dtype']:
            raise RuntimeError(f'Parameter device/dtype was not restored: {name}')
    for name, value in model.named_buffers(remove_duplicate=False):
        if value.is_meta or value.device.type != target_device.type or value.dtype != buffers[name]['dtype']:
            raise RuntimeError(f'Buffer device/dtype was not restored: {name}')
    head = actual['model.paligemma_with_expert.paligemma.lm_head.weight']
    embedding = actual['model.paligemma_with_expert.paligemma.model.language_model.embed_tokens.weight']
    if head.data_ptr() == embedding.data_ptr():
        raise RuntimeError('The input embedding unexpectedly shares storage with lm_head.')
    torch.cuda.synchronize(target_device)
    summary['load_seconds'] = time.monotonic() - started
    summary['gpu_allocated_bytes_after_base'] = torch.cuda.memory_allocated(target_device)
    summary['all_parameters_and_buffers_materialized'] = True
    print('Strict streaming base loading passed.', flush=True)
    return model, summary

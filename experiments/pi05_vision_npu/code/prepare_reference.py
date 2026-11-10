"""Capture the original GPU vision tensors, preprocessing, weights config and noise."""
import time
import numpy as np
import torch
from common import ROOT, load_policy, write_json, stats

out = ROOT / 'results/reference'
out.mkdir(exist_ok=True)
policy, base, cfg, batch, post, sample, loading = load_policy()
with torch.inference_mode():
    images, masks = base._preprocess_images(batch)
    vlm = base.model.paligemma_with_expert
    visual = vlm.paligemma.model.vision_tower.config
    write_json(out / 'vision_config.json', visual.to_dict())
    gpu_features = [vlm.embed_image(x) for x in images]
    pixels = torch.cat(images).contiguous().cpu().numpy()
    features = torch.cat(gpu_features).contiguous().cpu().numpy()
    np.save(out / 'pixels.npy', pixels)
    np.save(out / 'gpu_features.npy', features)
    torch.manual_seed(0)
    noise = base.model.sample_noise((1, cfg.chunk_size, cfg.max_action_dim), 'cuda')
    np.save(out / 'noise.npy', noise.cpu().numpy())
    torch.cuda.synchronize()
    t = time.perf_counter()
    actions = post(policy.predict_action_chunk(batch, noise=noise.clone())).detach().cpu().numpy()
    torch.cuda.synchronize()
    seconds = time.perf_counter() - t
    np.save(out / 'gpu_actions.npy', actions[0])
    old = __import__('json').loads((ROOT / 'results/original_gpu_baseline/predicted_actions.json').read_text())
    report = dict(loading=loading, camera_order=list(cfg.image_features),
        raw_shapes={k: list(v.shape) for k,v in sample.items() if torch.is_tensor(v)},
        processed_shapes={k:list(v.shape) for k,v in batch.items() if torch.is_tensor(v)},
        pixels_shape=list(pixels.shape), pixels_dtype=str(pixels.dtype),
        pixels_range=[float(pixels.min()),float(pixels.max())],
        image_masks=[m.cpu().tolist() for m in masks],
        feature_shape=list(features.shape), feature_dtype=str(features.dtype),
        no_additional_feature_scaling=True, action_names=cfg.action_feature_names,
        units=['degree']*6+['metre'], action_shape=list(actions.shape),
        original_baseline_comparison=stats(np.array(old['actions']),actions[0]),
        prediction_seconds=seconds)
    write_json(out / 'reference.json', report)
    print(__import__('json').dumps(report,indent=2), flush=True)

#!/usr/bin/env python3
"""Preview a local ACT checkpoint on recorded frames, without any robot interface.

This is a deployment precheck using the unmodified official LeRobot model,
dataset reader and processors. It does not assess real-world task success.
Joint values stay in degrees; gripper width stays in metres.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import html
import importlib.metadata
import json
import math
import os
from pathlib import Path
import statistics
import time
from zoneinfo import ZoneInfo


ACTION_NAMES = [f"joint_{i}.deg" for i in range(1, 7)] + ["gripper.m"]
CAMERAS = ["observation.images.global", "observation.images.wrist"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True,
                        help="Local checkpoint pretrained_model directory.")
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--episodes", default="0,8,26,51")
    parser.add_argument("--samples-per-episode", type=int, default=3)
    parser.add_argument("--threads", type=int, default=8)
    args = parser.parse_args()
    try:
        args.episodes = [int(value.strip()) for value in args.episodes.split(",")]
    except ValueError:
        parser.error("--episodes must be comma-separated integer episode indices")
    if not args.episodes or any(value < 0 for value in args.episodes):
        parser.error("--episodes must contain nonnegative indices")
    if len(set(args.episodes)) != len(args.episodes):
        parser.error("--episodes must not contain duplicates")
    if args.samples_per_episode < 1 or args.threads < 1:
        parser.error("--samples-per-episode and --threads must be positive")
    return args


def read_json(path: Path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def ensure_checkpoint_files(checkpoint: Path) -> None:
    for filename in ("config.json", "model.safetensors",
                     "policy_preprocessor.json", "policy_postprocessor.json"):
        if not (checkpoint / filename).is_file():
            raise FileNotFoundError(checkpoint / filename)
    for filename in ("policy_preprocessor.json", "policy_postprocessor.json"):
        for step in read_json(checkpoint / filename).get("steps", []):
            state_file = step.get("state_file")
            if state_file:
                state_path = (checkpoint / state_file).resolve()
                if not state_path.is_relative_to(checkpoint):
                    raise ValueError(f"Processor state is outside the checkpoint: {state_file}")
                if not state_path.is_file():
                    raise FileNotFoundError(state_path)


def json_safe(value):
    """Represent failed numerical results as null, with separate finite flags."""
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def observed_bounds(stats, np):
    action_stats = stats.get("action", {})
    try:
        lower = np.asarray(action_stats["min"], dtype=np.float64).reshape(-1)
        upper = np.asarray(action_stats["max"], dtype=np.float64).reshape(-1)
    except (KeyError, TypeError, ValueError):
        return None
    if (lower.shape != (7,) or upper.shape != (7,)
            or not np.isfinite(lower).all() or not np.isfinite(upper).all()
            or (lower > upper).any()):
        return None
    return lower, upper


def range_report(values, bounds, np):
    if bounds is None:
        return {"checked": False, "reason": "Dataset action min/max unavailable."}
    lower, upper = bounds
    finite = np.isfinite(values)
    below = finite & (values < lower - 1e-6)
    above = finite & (values > upper + 1e-6)
    violation = below | above
    return {
        "checked": True,
        "reference": "training_dataset_observed_action_range",
        "any_outside": bool(violation.any()),
        "outside_count_per_dimension": violation.sum(axis=0).astype(int).tolist(),
        "outside_indices": np.argwhere(violation).astype(int).tolist(),
        "nonfinite_count_per_dimension": (~finite).sum(axis=0).astype(int).tolist(),
        "clamped": False,
    }


def save_camera_pair(sample, output: Path, caption: str, np) -> dict:
    from PIL import Image, ImageDraw

    tiles = []
    image_checks = {}
    for key in CAMERAS:
        tensor = sample[key].detach().cpu()
        array = tensor.numpy()
        if array.ndim != 3 or array.shape[0] != 3:
            raise ValueError(f"Expected RGB CHW image for {key}, got {array.shape}")
        image_checks[key] = {
            "shape": list(array.shape), "dtype": str(array.dtype),
            "finite": bool(np.isfinite(array).all()),
            "min": float(np.min(array)), "max": float(np.max(array)),
        }
        # Pixel display conversion only; model inputs/actions are never clipped.
        pixels = np.nan_to_num(array, nan=0.0, posinf=1.0, neginf=0.0)
        pixels = np.rint(np.clip(pixels, 0, 1) * 255).astype(np.uint8)
        image = Image.fromarray(pixels.transpose(1, 2, 0), mode="RGB")
        tile = Image.new("RGB", (image.width, image.height + 28), "#18212b")
        tile.paste(image, (0, 28))
        ImageDraw.Draw(tile).text((10, 8), key, fill="white")
        tiles.append(tile)
    canvas = Image.new("RGB", (sum(tile.width for tile in tiles),
                               max(tile.height for tile in tiles) + 28), "#101820")
    ImageDraw.Draw(canvas).text((10, 8), caption, fill="white")
    x = 0
    for tile in tiles:
        canvas.paste(tile, (x, 28))
        x += tile.width
    canvas.save(output)
    return image_checks


def render_html(report: dict) -> str:
    def esc(value):
        return html.escape(str(value), quote=True)

    def number(value):
        return "—" if value is None else f"{value:.6f}"

    def result_table(sample):
        rows = []
        for index, name in enumerate(ACTION_NAMES):
            unit = "deg" if index < 6 else "m"
            rows.append(
                "<tr><th>" + esc(name) + "</th><td>" + unit + "</td>"
                + "".join("<td>" + number(sample[key][index]) + "</td>"
                          for key in ("raw_state", "gt_action", "predicted_action",
                                      "absolute_error", "chunk_mae_per_dimension"))
                + "</tr>"
            )
        return ("<table><thead><tr><th>维度</th><th>单位</th><th>当前状态</th>"
                "<th>示范动作</th><th>预测首动作</th><th>首动作绝对误差</th>"
                "<th>有效未来片段 MAE</th></tr></thead><tbody>"
                + "".join(rows) + "</tbody></table>")

    sections = []
    for sample in report["samples"]:
        status = "有限" if sample["finite"]["predicted_chunk"] else "存在非有限数值"
        bounds = sample["training_action_range"]
        range_text = ("有输出超出训练集观测范围" if bounds.get("any_outside")
                      else "输出位于训练集观测范围内") if bounds["checked"] else "未检查训练范围"
        chunks = json.dumps({key: sample[key] for key in
                             ("predicted_chunk", "target_chunk", "target_padding_mask")},
                            ensure_ascii=False, indent=2, allow_nan=False)
        sections.append(
            f'<section><h2>E{sample["episode_index"]} · 帧 {sample["frame_index"]}</h2>'
            f'<p>片段时间 {sample["timestamp_s"]:.3f} s；模型推理 '
            f'{sample["model_latency_ms"]:.1f} ms；含前后处理 '
            f'{sample["pipeline_latency_ms"]:.1f} ms。预测：{esc(status)}；{esc(range_text)}。'
            f'有效目标 {sample["valid_target_steps"]}/{report["model"]["chunk_size"]} 步。</p>'
            f'<img src="{esc(sample["image"])}" alt="双相机记录图像">'
            + result_table(sample)
            + '<details><summary>完整预测片段、目标片段及填充掩码</summary><pre>'
            + esc(chunks) + '</pre></details></section>'
        )
    summary_rows = "".join(
        f'<tr><th>{esc(name)}</th><td>{"deg" if i < 6 else "m"}</td>'
        f'<td>{number(report["summary"]["first_action_mae_per_dimension"][i])}</td></tr>'
        for i, name in enumerate(ACTION_NAMES)
    )
    latency = report["summary"]["model_latency_ms"]
    return """<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ACT 训练集离线预览</title><style>
body{font:16px/1.6 system-ui,sans-serif;max-width:1320px;margin:30px auto;padding:0 20px;color:#18212b;background:#f5f7fa}
header,section{background:white;padding:24px;margin:20px 0;border:1px solid #d9e0e8;border-radius:10px}
h1{margin:0 0 12px}h2{margin:0 0 8px}p{overflow-wrap:anywhere}.notice{background:#fff2c7;padding:14px;border-radius:6px}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums;font-size:14px}
th,td{padding:8px;border:1px solid #d9e0e8;text-align:right}th:first-child{text-align:left}thead{background:#edf2f8}
img{display:block;width:100%;height:auto;margin:18px 0}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}
details{margin-top:16px}code{overflow-wrap:anywhere}a{color:#1659a8}.scroll{overflow:auto}
</style><header><h1>ACT 训练集离线预览</h1>
<p class="notice">仅把已录制的双相机图像及关节状态输入模型，比较预测动作与同一训练示范。
本报告不连接机器人、不发送动作，不代表独立测试集成绩或真实抓放成功率。
每个样本重新预测完整动作片段；没有真实闭环反馈。关节使用 deg，夹爪使用 m。</p>
""" + (
        f'<p>生成时间：{esc(report["created_at"])}<br>checkpoint：<code>'
        f'{esc(report["checkpoint"])}</code><br>数据集：<code>{esc(report["dataset_root"])}</code></p>'
        f'<p>官方 LeRobot {esc(report["versions"]["lerobot"])}；CPU '
        f'{report["threads"]} 线程；{len(report["samples"])} 个样本；一次预热不计入推理耗时。'
        f'模型推理均值 {latency["mean"]:.1f} ms，中位数 {latency["median"]:.1f} ms。'
        f'<a href="report.json">完整 JSON</a></p>'
        '<p>范围检查只使用训练集 action 最小值／最大值，未验证机械限位。预测动作未裁剪。'
        '未来片段的填充帧不计入 MAE；非有限数值在 JSON 中用 null 表示。</p>'
        '<h2>抽样首动作 MAE（逐维，不混合 deg 与 m）</h2>'
        '<table><thead><tr><th>维度</th><th>单位</th><th>MAE</th></tr></thead><tbody>'
        + summary_rows + '</tbody></table></header>' + "".join(sections) + '</html>'
    )


def main() -> None:
    args = parse_args()
    checkpoint = args.checkpoint.expanduser().resolve()
    dataset_root = args.dataset_root.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    ensure_checkpoint_files(checkpoint)
    if output.is_relative_to(dataset_root) or output.is_relative_to(checkpoint):
        raise ValueError("Output must be outside the input dataset and checkpoint directories.")
    if (output / "report.json").exists() or (output / "report.html").exists():
        raise FileExistsError("Choose a new output directory; existing reports are not overwritten.")
    output.mkdir(parents=True, exist_ok=True)
    (output / "images").mkdir(exist_ok=True)

    # Keep library caches with the report; never download or alter source data.
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_DATASETS_OFFLINE"] = "1"
    os.environ["HF_HUB_DISABLE_TELEMETRY"] = "1"
    os.environ["HF_DATASETS_CACHE"] = str(output / "cache" / "datasets")
    os.environ["OMP_NUM_THREADS"] = str(args.threads)
    os.environ["MKL_NUM_THREADS"] = str(args.threads)
    os.environ["OPENBLAS_NUM_THREADS"] = "1"

    import numpy as np
    import pyarrow.dataset as pads
    import torch
    from safetensors.torch import load_model
    from lerobot.configs import PreTrainedConfig
    from lerobot.datasets import LeRobotDataset
    from lerobot.policies import make_policy, make_pre_post_processors

    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    torch.manual_seed(0)
    info = read_json(dataset_root / "meta" / "info.json")
    for key in ("action", "observation.state"):
        feature = info["features"][key]
        if feature["shape"] != [7] or feature.get("names") != ACTION_NAMES:
            raise ValueError(f"Unexpected {key} ordering/units; expected {ACTION_NAMES}")

    cfg = PreTrainedConfig.from_pretrained(
        checkpoint, local_files_only=True,
        cli_overrides=["--device=cpu", "--use_amp=false",
                       "--pretrained_backbone_weights=null"],
    )
    if cfg.type != "act" or cfg.use_peft or cfg.n_obs_steps != 1:
        raise ValueError("This preview requires a regular ACT checkpoint with n_obs_steps=1.")
    if set(cfg.image_features) != set(CAMERAS):
        raise ValueError(f"Expected the recorded global/wrist cameras, got {list(cfg.image_features)}")
    if cfg.output_features["action"].shape != (7,):
        raise ValueError(f"Checkpoint action dimension is not 7: {cfg.output_features['action'].shape}")

    # Read numeric targets only. This does not decode videos for future steps.
    columns = ["index", "episode_index", "frame_index", "timestamp", "observation.state", "action"]
    arrow_data = pads.dataset(dataset_root / "data", format="parquet")
    rows = arrow_data.to_table(
        columns=columns, filter=pads.field("episode_index").isin(args.episodes)
    ).to_pylist()
    episodes = {episode: [] for episode in args.episodes}
    for row in rows:
        episodes[int(row["episode_index"])].append(row)
    for episode, records in episodes.items():
        records.sort(key=lambda row: row["index"])
        if not records:
            raise ValueError(f"Episode {episode} is absent from the local dataset.")
        if len({row["frame_index"] for row in records}) != len(records):
            raise ValueError(f"Duplicate frame indices in episode {episode}")

    dataset = LeRobotDataset(
        repo_id=f"local/{dataset_root.name}", root=dataset_root,
        episodes=args.episodes, video_backend="torchcodec", return_uint8=False,
    )
    index_map = {int(value): index for index, value in enumerate(
        dataset.hf_dataset.data.column("index").to_pylist())}
    bounds = observed_bounds(dataset.meta.stats, np)

    # make_policy has no strict argument. Build the official architecture once,
    # then use its official safetensors loader with strict=True, avoiding a
    # permissive load followed by a redundant second weights read.
    cfg.pretrained_path = None
    policy = make_policy(cfg=cfg, ds_meta=dataset.meta)
    missing, unexpected = load_model(
        policy, str(checkpoint / "model.safetensors"), strict=True, device="cpu"
    )
    if missing or unexpected:
        raise RuntimeError(f"Strict load mismatch: missing={missing}, unexpected={unexpected}")
    cfg.pretrained_path = checkpoint
    policy.eval()
    policy.requires_grad_(False)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg, pretrained_path=str(checkpoint),
        preprocessor_overrides={
            "device_processor": {"device": "cpu", "float_dtype": "float32"},
            "normalizer_processor": {"device": "cpu"},
        },
        postprocessor_overrides={
            "device_processor": {"device": "cpu", "float_dtype": "float32"},
            "unnormalizer_processor": {"device": "cpu"},
        },
    )
    print("Strict ACT weights and checkpoint processors loaded on CPU.", flush=True)

    selections = []
    for episode in args.episodes:
        records = episodes[episode]
        offsets = np.unique(np.rint(np.linspace(
            0, len(records) - 1, min(args.samples_per_episode, len(records))
        )).astype(int))
        selections.extend((episode, int(offset)) for offset in offsets)

    def reset_inference():
        policy.reset()
        preprocessor.reset()
        postprocessor.reset()

    def predict(sample):
        obs = {key: sample[key].clone() for key in cfg.input_features if key in sample}
        if "action" in obs:
            raise AssertionError("Ground-truth actions must never be model inputs.")
        started = time.perf_counter()
        batch = preprocessor(obs)
        model_started = time.perf_counter()
        with torch.inference_mode():
            normalized = policy.predict_action_chunk(batch)
            model_seconds = time.perf_counter() - model_started
            denormalized = postprocessor(normalized).detach().cpu()
        pipeline_seconds = time.perf_counter() - started
        if tuple(denormalized.shape) != (1, cfg.chunk_size, 7):
            raise ValueError(f"Unexpected predicted chunk shape: {tuple(denormalized.shape)}")
        return denormalized.squeeze(0).numpy(), model_seconds, pipeline_seconds

    first_episode, first_offset = selections[0]
    first_row = episodes[first_episode][first_offset]
    decoded_at = time.perf_counter()
    first_sample = dataset[index_map[int(first_row["index"])]]
    first_decode_seconds = time.perf_counter() - decoded_at
    reset_inference()
    warmup_started = time.perf_counter()
    predict(first_sample)
    warmup_seconds = time.perf_counter() - warmup_started
    reset_inference()

    report = {
        "created_at": datetime.now(ZoneInfo("Asia/Singapore")).isoformat(),
        "purpose": "training_dataset_offline_fit_preview_not_robot_execution",
        "hardware_connected": False, "actions_sent": False,
        "checkpoint": str(checkpoint), "dataset_root": str(dataset_root),
        "threads": args.threads, "episodes": args.episodes,
        "samples_per_episode_requested": args.samples_per_episode,
        "versions": {name: importlib.metadata.version(name)
                     for name in ("lerobot", "torch", "torchvision", "torchcodec")},
        "model": {"type": "act", "device": "cpu", "use_amp": False,
                  "chunk_size": cfg.chunk_size, "n_action_steps": cfg.n_action_steps,
                  "parameter_count": sum(parameter.numel() for parameter in policy.parameters()),
                  "strict_weights_load": True, "missing_keys": [], "unexpected_keys": [],
                  "weights_bytes": (checkpoint / "model.safetensors").stat().st_size,
                  "checkpoint_normalization_used": True},
        "action_names": ACTION_NAMES, "units": ["deg"] * 6 + ["m"],
        "sampling": "evenly_spaced_including_first_and_last_frame_per_episode",
        "warmup_runs": 1, "warmup_seconds": warmup_seconds,
        "latency_scope": "CPU model forward; pipeline adds official pre/postprocessing; excludes video decoding",
        "training_action_min": bounds[0].tolist() if bounds else None,
        "training_action_max": bounds[1].tolist() if bounds else None,
        "mechanical_limits": {"checked": False,
                              "reason": "Only recorded training action min/max are available; no mechanical-limit claim."},
        "nonfinite_json_encoding": "null, accompanied by explicit finite flags",
        "samples": [],
    }

    previous_episode = None
    for sample_number, (episode, offset) in enumerate(selections):
        if episode != previous_episode:
            reset_inference()
            previous_episode = episode
        records = episodes[episode]
        row = records[offset]
        if sample_number == 0:
            sample, decode_seconds = first_sample, first_decode_seconds
        else:
            decoded_at = time.perf_counter()
            sample = dataset[index_map[int(row["index"])]]
            decode_seconds = time.perf_counter() - decoded_at
        if int(sample["episode_index"].item()) != episode:
            raise ValueError("Dataset index/episode mismatch.")
        raw_state = sample["observation.state"].detach().cpu().numpy().astype(np.float64)
        gt = np.asarray(row["action"], dtype=np.float64)
        if raw_state.shape != (7,) or gt.shape != (7,):
            raise ValueError("Expected seven state/action dimensions.")
        if not np.allclose(sample["action"].numpy(), gt, equal_nan=True):
            raise ValueError("Dataset reader and parquet ground-truth action disagree.")
        predicted, model_seconds, pipeline_seconds = predict(sample)
        target_positions = np.arange(offset, offset + cfg.chunk_size)
        padding_mask = target_positions >= len(records)
        target_positions = np.minimum(target_positions, len(records) - 1)
        target = np.asarray([records[int(index)]["action"] for index in target_positions], dtype=np.float64)
        valid = ~padding_mask
        first_error = np.abs(predicted[0] - gt)
        chunk_mae = np.abs(predicted[valid] - target[valid]).mean(axis=0)
        image_name = f"images/E{episode:03d}_F{int(row['frame_index']):05d}.png"
        image_checks = save_camera_pair(
            sample, output / image_name,
            f"Episode {episode} / frame {int(row['frame_index'])} / recorded RGB", np
        )
        sample_report = {
            "episode_index": episode, "frame_index": int(row["frame_index"]),
            "dataset_index": int(row["index"]), "episode_offset": offset,
            "timestamp_s": float(row["timestamp"]), "task": sample.get("task"),
            "raw_state": raw_state.tolist(), "gt_action": gt.tolist(),
            "predicted_action": predicted[0].tolist(), "absolute_error": first_error.tolist(),
            "predicted_chunk": predicted.tolist(), "target_chunk": target.tolist(),
            "target_padding_mask": padding_mask.tolist(),
            "valid_target_steps": int(valid.sum()),
            "chunk_mae_per_dimension": chunk_mae.tolist(),
            "model_latency_ms": model_seconds * 1000,
            "pipeline_latency_ms": pipeline_seconds * 1000,
            "decode_latency_ms": decode_seconds * 1000,
            "finite": {"raw_state": bool(np.isfinite(raw_state).all()),
                       "gt_action": bool(np.isfinite(gt).all()),
                       "target_chunk": bool(np.isfinite(target).all()),
                       "predicted_action": bool(np.isfinite(predicted[0]).all()),
                       "predicted_chunk": bool(np.isfinite(predicted).all())},
            "training_action_range": range_report(predicted, bounds, np),
            "image": image_name, "image_checks": image_checks,
        }
        report["samples"].append(sample_report)
        print(f"E{episode} frame {row['frame_index']}: model {model_seconds:.3f}s; "
              f"finite={sample_report['finite']['predicted_chunk']}", flush=True)

    latencies = [sample["model_latency_ms"] for sample in report["samples"]]
    errors = np.asarray([sample["absolute_error"] for sample in report["samples"]])
    per_dimension_mae = errors.mean(axis=0)
    report["summary"] = {
        "sample_count": len(report["samples"]),
        "all_predictions_finite": all(sample["finite"]["predicted_chunk"] for sample in report["samples"]),
        "first_action_mae_per_dimension": per_dimension_mae.tolist(),
        "first_action_joint_mae_deg": float(per_dimension_mae[:6].mean()),
        "first_action_gripper_mae_m": float(per_dimension_mae[6]),
        "model_latency_ms": {"mean": statistics.mean(latencies),
                             "median": statistics.median(latencies),
                             "min": min(latencies), "max": max(latencies)},
        "samples_outside_training_action_range": sum(
            bool(sample["training_action_range"].get("any_outside")) for sample in report["samples"]),
    }
    safe_report = json_safe(report)
    (output / "report.json").write_text(
        json.dumps(safe_report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    (output / "report.html").write_text(render_html(safe_report), encoding="utf-8")
    print(f"Offline preview written: {output / 'report.html'}", flush=True)


if __name__ == "__main__":
    main()

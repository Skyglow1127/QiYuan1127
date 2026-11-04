#!/usr/bin/env python3
"""One passive ACT prediction from fresh PiPER feedback and HTTP camera images.

This program has no CAN transmit path and never constructs a Robot/Teleoperator.
Predictions are a preview, not an executable trajectory or confirmed limits.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import html
import importlib.metadata
import json
import math
import os
from pathlib import Path
import time
from zoneinfo import ZoneInfo

from preview_act_offline import (
    ACTION_NAMES, CAMERAS, ensure_checkpoint_files, json_safe, read_json,
    save_camera_pair,
)


def state_only_snapshot(bus, timeout_s=8.0):
    """Require live, complete feedback; missing leader commands are irrelevant."""
    deadline = time.monotonic() + timeout_s
    last_issues = []
    while True:
        snapshot = bus.snapshot()
        now = time.monotonic()
        issues = []
        if snapshot.get("error"):
            issues.append(snapshot["error"])
        if not snapshot.get("link", {}).get("ready"):
            issues.append("CAN must remain UP + LISTEN-ONLY + 1 Mbps")
        parts = snapshot.get("parts", {})
        for key in ["status_0"] + [f"state_{i}" for i in range(4)]:
            part = parts.get(key)
            if not part:
                issues.append(f"Missing {key}")
            elif not 0 <= (now-part["receive_monotonic"])*1000 < 100:
                issues.append(f"Stale {key}")
        state = snapshot.get("state")
        if (state is None or len(state) != 7
                or any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in state)):
            issues.append("Missing or nonfinite 7D state")
        status = parts.get("status_0", {})
        if status and (status.get("arm_status") != 0 or status.get("err_code") != 0):
            issues.append("Arm reports a status/error fault")
        grip = parts.get("state_3", {})
        if grip and (grip.get("mode") != "width" or grip.get("fault_bits")):
            issues.append("Gripper mode is not width or it reports a fault")
        if not issues:
            return {
                "sampled_monotonic": now,
                "state": list(state),
                "parts": {key: parts[key] for key in ["status_0"] + [f"state_{i}" for i in range(4)]},
                "status": status,
                "gripper": grip,
                "source_ages_ms": {key: (now-parts[key]["receive_monotonic"])*1000
                                   for key in ["status_0"] + [f"state_{i}" for i in range(4)]},
                "transport": snapshot.get("transport"),
                "link": snapshot.get("link"),
                "generation": snapshot.get("generation"),
                "state_only_gate": "fresh state/status <100ms; complete finite 7D; no reported faults; width mode",
                "leader_action_required": False,
                "leader_action_used_as_model_input": False,
            }
        last_issues = issues
        if now >= deadline:
            raise RuntimeError("Passive feedback gate failed: " + "; ".join(last_issues))
        time.sleep(.005)


def capture_observation(bus, cameras):
    """Pair RGB frames, then sample CAN state and recheck image ages."""
    state_only_snapshot(bus)
    deadline = time.monotonic() + 1.0
    last_pair = None
    with ThreadPoolExecutor(max_workers=2) as executor:
        for attempt in range(1, 6):
            futures = {name: executor.submit(camera.async_read, timeout_ms=200)
                       for name, camera in cameras.items()}
            images, failures = {}, []
            for name, future in futures.items():
                try:
                    images[name] = future.result()
                except TimeoutError as error:
                    failures.append(f"{name}: {error}")
            if failures:
                last_pair = {"attempt": attempt, "errors": failures}
                if time.monotonic() >= deadline:
                    break
                continue
            feedback = state_only_snapshot(bus, timeout_s=.15)
            now = time.monotonic()
            metadata = {name: dict(camera.last_metadata) for name, camera in cameras.items()}
            ages = {name: (now-item["estimated_capture_monotonic"])*1000
                    for name, item in metadata.items()}
            times = [item["estimated_capture_monotonic"] for item in metadata.values()]
            skew = (max(times)-min(times))*1000
            last_pair = {"attempt": attempt, "checked_monotonic": now,
                         "ages_ms": ages, "skew_ms": skew}
            if all(0 <= age <= 150 for age in ages.values()) and skew <= 60:
                for name, age in ages.items():
                    metadata[name]["sample_age_ms"] = age
                return images, metadata, feedback, last_pair
            if now >= deadline:
                break
    raise RuntimeError("Passive camera freshness/pair gate failed: " + json.dumps(last_pair))


def render_html(report):
    esc = lambda x: html.escape(str(x), quote=True)
    if report.get("status") != "predicted":
        return '<!doctype html><meta charset="utf-8"><h1>NO_COMMANDS_SENT</h1><pre>' + esc(json.dumps(report, ensure_ascii=False, indent=2)) + '</pre>'
    current = report["observation"]["state"]
    first = report["prediction"]["first_action"]
    delta = report["prediction"]["first_action_delta_from_start"]
    final = report["prediction"]["last_action"]
    max_delta = report["prediction"]["max_abs_chunk_delta_from_start"]
    rows = []
    for i, name in enumerate(ACTION_NAMES):
        rows.append('<tr><th>' + esc(name) + '</th><td>' + ('deg' if i < 6 else 'm')
                    + '</td>' + ''.join('<td>' + (f'{v:.6f}' if v is not None else 'nonfinite') + '</td>'
                                       for v in (current[i], first[i], delta[i], final[i], max_delta[i])) + '</tr>')
    return '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ACT 实时被动预测预览</title><style>body{font:16px/1.6 system-ui;max-width:1280px;margin:30px auto;padding:0 20px;color:#18212b}table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}th,td{border:1px solid #ccd5df;padding:8px;text-align:right}th:first-child{text-align:left}img{width:100%;height:auto}.note{background:#fff1c8;padding:16px}pre{white-space:pre-wrap;overflow-wrap:anywhere}</style>
<h1>ACT 实时被动预测 · NO_COMMANDS_SENT</h1><p class="note">仅监听机械臂反馈并读取双相机，使用真实当前状态预测一个100步动作片段。没有发送CAN、使能、标零、复位或切换模式。预测未裁剪；机械限位、碰撞路径、夹爪零位与实机成功率尚未确认。100步仅对应训练时20Hz的5秒名义片段，并非已批准的执行路径。</p>''' + (
        '<p>时间：' + esc(report["created_at"]) + '<br>模型：' + esc(report["checkpoint"])
        + '<br>CPU模型耗时：' + f'{report["prediction"]["model_latency_ms"]:.1f} ms'
        + '；前后处理合计：' + f'{report["prediction"]["pipeline_latency_ms"]:.1f} ms'
        + '（仅一次冷启动预测，没有预热）。<a href="report.json">完整JSON及100×7预测</a></p>'
        + '<img src="images/camera_pair.png" alt="输入ACT的全局与腕部RGB图像">'
        + '<table><thead><tr><th>维度</th><th>单位</th><th>当前state</th><th>预测首action</th><th>首action−当前state</th><th>第100步action</th><th>片段最大绝对偏移</th></tr></thead><tbody>'
        + ''.join(rows) + '</tbody></table><p>相机配对时差：'
        + f'{report["camera_pair"]["skew_ms"]:.2f} ms；state在相机读完后采样，推理后另存新鲜反馈。输入只有state与两张RGB图，没有action。</p>'
        + '<details><summary>推理后的新鲜反馈</summary><pre>'
        + esc(json.dumps(report["postprediction_feedback"], ensure_ascii=False, indent=2)) + '</pre></details></html>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--io-config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--threads", type=int, default=8)
    args = parser.parse_args()
    checkpoint, config_path, output = [p.expanduser().resolve() for p in
                                        (args.checkpoint, args.io_config, args.output_dir)]
    ensure_checkpoint_files(checkpoint)
    config = read_json(config_path)
    if not 1 <= args.threads <= 64:
        parser.error("--threads must be within 1..64")
    if config.get("transport", {}).get("kind") != "ssh":
        raise ValueError("This preview requires the configured passive KR260 SSH source")
    if output.exists():
        raise FileExistsError(f"Use a new output directory: {output}")
    if output.is_relative_to(checkpoint):
        raise ValueError("Output must be outside checkpoint")
    output.mkdir(parents=True)
    (output / "images").mkdir()
    for name, value in {"HF_HUB_OFFLINE": "1", "HF_DATASETS_OFFLINE": "1",
                        "HF_HUB_DISABLE_TELEMETRY": "1", "OMP_NUM_THREADS": str(args.threads),
                        "MKL_NUM_THREADS": str(args.threads), "OPENBLAS_NUM_THREADS": "1"}.items():
        os.environ[name] = value
    report = {"created_at": datetime.now(ZoneInfo("Asia/Singapore")).isoformat(),
              "purpose": "one_passive_live_ACT_chunk_preview", "status": "initializing",
              "NO_COMMANDS_SENT": True, "robot_commands_sent": 0, "actions_sent": False,
              "mode_changed": False, "enable_zero_reset_called": False,
              "checkpoint": str(checkpoint), "io_config": str(config_path),
              "threads": args.threads, "device": "cpu", "dtype": "float32",
              "mechanical_limits_confirmed": False, "actions_clamped": False,
              "hardware_control_interface_created": False}
    bus, cameras = None, {}
    try:
        import numpy as np
        import torch
        from safetensors.torch import load_model
        from lerobot.configs import PreTrainedConfig
        from lerobot.policies import make_pre_post_processors
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot_robot_piper_shared.devices import PiperHttpCamera, PiperHttpCameraConfig
        from lerobot_robot_piper_shared.transport import PassiveCAN

        torch.set_num_threads(args.threads)
        torch.set_num_interop_threads(1)
        cfg = PreTrainedConfig.from_pretrained(
            checkpoint, local_files_only=True,
            cli_overrides=["--device=cpu", "--use_amp=false", "--pretrained_backbone_weights=null"])
        if (cfg.type != "act" or cfg.use_peft or cfg.n_obs_steps != 1 or cfg.chunk_size != 100
                or set(cfg.input_features) != {"observation.state", *CAMERAS}
                or tuple(cfg.input_features["observation.state"].shape) != (7,)
                or tuple(cfg.output_features["action"].shape) != (7,)):
            raise ValueError("Expected ACT with only 7D state + global/wrist RGB input, 100x7 output")
        cfg.pretrained_path = None
        policy = ACTPolicy(cfg)
        missing, unexpected = load_model(policy, str(checkpoint / "model.safetensors"),
                                         strict=True, device="cpu")
        if missing or unexpected:
            raise RuntimeError(f"Strict checkpoint load mismatch: {missing}, {unexpected}")
        policy.eval().requires_grad_(False)
        cfg.pretrained_path = checkpoint
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=cfg, pretrained_path=str(checkpoint),
            preprocessor_overrides={"device_processor": {"device": "cpu", "float_dtype": "float32"},
                                    "normalizer_processor": {"device": "cpu"}},
            postprocessor_overrides={"device_processor": {"device": "cpu", "float_dtype": "float32"},
                                     "unnormalizer_processor": {"device": "cpu"}})
        report["model"] = {"strict_load": True, "missing_keys": [], "unexpected_keys": [],
                           "chunk_size": cfg.chunk_size, "n_obs_steps": cfg.n_obs_steps,
                           "input_keys": list(cfg.input_features), "warmup_runs": 0,
                           "prediction_calls": 0}
        report["versions"] = {key: importlib.metadata.version(key)
                              for key in ("lerobot", "torch", "torchvision", "safetensors")}
        print("Strict ACT + checkpoint processors loaded on CPU; opening receive-only observations.", flush=True)
        for role in ("global", "wrist"):
            camera_config = config["cameras"][role]
            feature_shape = tuple(cfg.input_features[f"observation.images.{role}"].shape)
            if feature_shape != (3, 480, 640):
                raise ValueError(f"Unexpected {role} model shape: {feature_shape}")
            camera = PiperHttpCamera(PiperHttpCameraConfig(
                role=role, url=camera_config["url"], serial=camera_config["serial"],
                width=640, height=480, max_age_ms=150))
            camera.connect(warmup=False)
            cameras[role] = camera
        bus = PassiveCAN(config)
        images, metadata, observation, pair = capture_observation(bus, cameras)
        report.update(observation=observation, camera_metadata=metadata, camera_pair=pair)
        sample = {"observation.state": torch.tensor(observation["state"], dtype=torch.float32)}
        for role, image in images.items():
            sample[f"observation.images.{role}"] = torch.from_numpy(image.copy()).permute(2, 0, 1).float()/255
        if set(sample) != set(cfg.input_features) or "action" in sample:
            raise AssertionError("Model input must contain observations only")
        policy.reset()
        preprocessor.reset()
        postprocessor.reset()
        started = time.perf_counter()
        report["observation_age_at_prediction_start_ms"] = (
            time.monotonic()-observation["sampled_monotonic"])*1000
        batch = preprocessor(sample)
        model_started = time.perf_counter()
        with torch.inference_mode():
            normalized = policy.predict_action_chunk(batch)
            model_seconds = time.perf_counter()-model_started
            denormalized = postprocessor(normalized).detach().cpu()
        pipeline_seconds = time.perf_counter()-started
        report["model"]["prediction_calls"] = 1
        postprediction = state_only_snapshot(bus, timeout_s=.3)
        if tuple(denormalized.shape) != (1, 100, 7):
            raise ValueError(f"Unexpected chunk shape: {tuple(denormalized.shape)}")
        chunk = denormalized.squeeze(0).numpy()
        state = np.asarray(observation["state"], dtype=np.float64)
        delta = chunk-state
        report["prediction"] = {
            "names": ACTION_NAMES, "units": ["deg"]*6+["m"], "shape": [100, 7],
            "finite": bool(np.isfinite(chunk).all()), "chunk": chunk.tolist(),
            "first_action": chunk[0].tolist(), "last_action": chunk[-1].tolist(),
            "first_action_delta_from_start": delta[0].tolist(),
            "chunk_delta_from_start": delta.tolist(),
            "max_abs_chunk_delta_from_start": np.max(np.abs(delta), axis=0).tolist(),
            "model_latency_ms": model_seconds*1000, "pipeline_latency_ms": pipeline_seconds*1000,
            "latency_scope": "one cold prediction; no warmup; not a steady-state benchmark",
            "nominal_training_fps": config["fps"], "executed_steps": 0,
        }
        report["postprediction_feedback"] = postprediction
        report["postprediction_state_delta"] = (np.asarray(postprediction["state"])-state).tolist()
        # Save the presentation image after the fresh postprediction feedback;
        # PNG compression must not delay inference on the sampled observation.
        report["image_checks"] = save_camera_pair(sample, output / "images/camera_pair.png",
                                                   "PASSIVE LIVE ACT INPUT - NO COMMANDS SENT", np)
        report["status"] = "predicted"
        print(f"NO_COMMANDS_SENT: predicted 100x7 on CPU, model {model_seconds*1000:.1f} ms", flush=True)
    except Exception as error:
        report["status"] = "failed"
        report["error"] = {"type": type(error).__name__, "message": str(error)}
        raise
    finally:
        if bus is not None:
            bus.close()
        for camera in cameras.values():
            camera.disconnect()
        report["finished_at"] = datetime.now(ZoneInfo("Asia/Singapore")).isoformat()
        clean = json_safe(report)
        (output / "report.json").write_text(json.dumps(clean, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
        (output / "report.html").write_text(render_html(clean), encoding="utf-8")


if __name__ == "__main__":
    main()

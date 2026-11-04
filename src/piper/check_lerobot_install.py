#!/usr/bin/env python3
"""不连接硬件的安装验证：模块、ACT 合成输入、双相机视频数据读写、CLI。

此脚本不下载模型权重、不连接 CAN、不连接相机、不发送机器人指令。
用法：先 source lerobot_env.sh，再 python check_lerobot_install.py。
"""

import argparse
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("logs") / "lerobot_install")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    import av
    import numpy as np
    import torch
    import torchvision
    import torchcodec
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

    packages = ["lerobot", "torch", "torchvision", "torchcodec", "transformers", "datasets", "av", "python-can"]
    report = {
        "python": sys.executable,
        "versions": {name: importlib.metadata.version(name) for name in packages},
        "cuda_available": torch.cuda.is_available(),
        "smolvla": "config and policy imports passed; pretrained weights not downloaded",
        "robot_commands_sent": 0,
    }
    print("[OK] LeRobot / ACT / SmolVLA / 视频模块导入", flush=True)

    torch.set_num_threads(2)
    torch.manual_seed(42)
    config = ACTConfig(
        device="cpu", chunk_size=4, n_action_steps=2,
        input_features={
            "observation.state": PolicyFeature(FeatureType.STATE, (7,)),
            "observation.images.global": PolicyFeature(FeatureType.VISUAL, (3, 64, 64)),
            "observation.images.wrist": PolicyFeature(FeatureType.VISUAL, (3, 64, 64)),
        },
        output_features={"action": PolicyFeature(FeatureType.ACTION, (7,))},
        pretrained_backbone_weights=None,
        dim_model=64, n_heads=4, dim_feedforward=128,
        n_encoder_layers=1, n_decoder_layers=1, use_vae=False,
    )
    policy = ACTPolicy(config)
    batch = {
        "observation.state": torch.zeros(1, 7),
        "observation.images.global": torch.rand(1, 3, 64, 64),
        "observation.images.wrist": torch.rand(1, 3, 64, 64),
        "action": torch.zeros(1, 4, 7),
        "action_is_pad": torch.zeros(1, 4, dtype=torch.bool),
    }
    policy.train()
    loss, _ = policy(batch)
    assert torch.isfinite(loss)
    loss.backward()
    policy.eval()
    with torch.no_grad():
        action = policy.select_action(batch)
    assert action.shape == (1, 7) and torch.isfinite(action).all()
    report["act_synthetic_smoke"] = {"loss": float(loss.detach()), "action_shape": list(action.shape), "backward_pass": True}
    print("[OK] ACT 双图像、7 维状态的前向、反向与动作预测", flush=True)

    with tempfile.TemporaryDirectory(prefix="synthetic_dataset_", dir=args.output) as temp:
        root = Path(temp) / "dataset"
        features = {
            "observation.state": {"dtype": "float32", "shape": (7,), "names": [f"q{i}" for i in range(7)]},
            "action": {"dtype": "float32", "shape": (7,), "names": [f"target{i}" for i in range(7)]},
            "observation.images.global": {"dtype": "video", "shape": (64, 64, 3), "names": ["height", "width", "channels"]},
            "observation.images.wrist": {"dtype": "video", "shape": (64, 64, 3), "names": ["height", "width", "channels"]},
        }
        dataset = LeRobotDataset.create(
            repo_id="local/installation_smoke", fps=10, features=features,
            root=root, robot_type="synthetic_piper_7dof", use_videos=True,
            video_backend="torchcodec", image_writer_threads=1, encoder_threads=1,
            rgb_encoder=RGBEncoderConfig(vcodec="h264", crf=23, preset="fast"),
        )
        for frame in range(8):
            image = np.zeros((64, 64, 3), dtype=np.uint8)
            image[:, :, 0] = 40 + frame * 20
            image[:, :, 1] = 100
            dataset.add_frame({
                "observation.state": np.full(7, frame, np.float32),
                "action": np.full(7, frame + 0.5, np.float32),
                "observation.images.global": image,
                "observation.images.wrist": np.flip(image, axis=2).copy(),
                "task": "installation test only",
            })
        dataset.save_episode(parallel_encoding=False)
        dataset.finalize()
        loaded = LeRobotDataset("local/installation_smoke", root=root, video_backend="torchcodec")
        assert len(loaded) == 8 and loaded.meta.total_episodes == 1
        item = loaded[3]
        assert item["action"].shape == (7,)
        assert torch.allclose(item["observation.state"], torch.full((7,), 3.0))
        for key in ["observation.images.global", "observation.images.wrist"]:
            assert item[key].shape == (3, 64, 64) and torch.isfinite(item[key]).all()
        report["dataset_roundtrip"] = {"frames": len(loaded), "episodes": 1, "rgb_cameras": 2, "video_backend": "torchcodec", "codec": "h264"}
    print("[OK] 双相机 MP4＋关节/动作 Parquet 保存、重新加载及解码", flush=True)

    report["cli"] = {}
    bin_dir = Path(sys.executable).parent
    for command in ["lerobot-info", "lerobot-train", "lerobot-record", "lerobot-rollout", "lerobot-teleoperate"]:
        argv = [str(bin_dir / command)]
        if command != "lerobot-info":
            argv.append("--help")
        result = subprocess.run(argv, capture_output=True, text=True, timeout=60)
        (args.output / (command + ".txt")).write_text(result.stdout + result.stderr)
        report["cli"][command] = result.returncode
        if result.returncode:
            raise RuntimeError(f"{command} 入口验证失败；详情见 {args.output}")
        print(f"[OK] {command}", flush=True)
    report["success"] = True
    (args.output / "verification.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"[完成] {args.output / 'verification.json'}")


if __name__ == "__main__":
    main()

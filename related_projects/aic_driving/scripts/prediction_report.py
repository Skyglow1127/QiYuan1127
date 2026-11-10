#!/usr/bin/env python3
"""Report frozen paired prediction errors with an episode-cluster bootstrap."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def mean_metrics(errors):
    errors = np.asarray(errors, dtype=np.float64)
    return {"mean_displacement_error_m": float(np.linalg.norm(errors, axis=1).mean()),
            "mae_xy_m": np.abs(errors).mean(axis=0).tolist(),
            "rmse_xy_m": np.sqrt(np.mean(errors ** 2, axis=0)).tolist()}


def paired_cluster_summary(analytic, learned, episode_ids, seed, repetitions):
    """Average matched sample errors within episode, then resample episodes."""
    analytic_distance = np.linalg.norm(analytic, axis=1)
    learned_distance = np.linalg.norm(learned, axis=1)
    episodes = np.unique(episode_ids)
    episode_rows = []
    for episode in episodes:
        mask = episode_ids == episode
        baseline = float(analytic_distance[mask].mean())
        candidate = float(learned_distance[mask].mean())
        episode_rows.append({"episode_id": str(episode), "samples": int(mask.sum()),
                             "analytic_m": baseline, "learned_m": candidate,
                             "difference_learned_minus_analytic_m": candidate - baseline})
    differences = np.asarray([row["difference_learned_minus_analytic_m"] for row in episode_rows])
    rng = np.random.default_rng(seed)
    resampled_indices = rng.integers(0, len(episodes), size=(repetitions, len(episodes)))
    resampled_means = differences[resampled_indices].mean(axis=1)
    interval = np.quantile(resampled_means, [0.025, 0.975])
    return {
        "sample_count": int(len(analytic)), "episode_count": int(len(episodes)),
        "sample_weighted": {"analytic": mean_metrics(analytic), "learned": mean_metrics(learned)},
        "episode_equal_weight": {
            "analytic_mean_displacement_error_m": float(np.mean([r["analytic_m"] for r in episode_rows])),
            "learned_mean_displacement_error_m": float(np.mean([r["learned_m"] for r in episode_rows])),
            "difference_learned_minus_analytic_m": float(differences.mean()),
            "bootstrap_95_percentile_ci_m": interval.tolist(),
        },
        "episodes": episode_rows,
    }


def generate_report(model_dir, output, seed=20261023, repetitions=1000):
    model_dir, output = Path(model_dir), Path(output)
    required = ["heldout_errors.json", "heldout_errors.npz", "calibration.json", "metadata.json"]
    for name in required:
        if not (model_dir / name).is_file():
            raise FileNotFoundError(model_dir / name)
    saved = json.loads((model_dir / "heldout_errors.json").read_text())
    metadata = json.loads((model_dir / "metadata.json").read_text())
    calibration = json.loads((model_dir / "calibration.json").read_text())
    with np.load(model_dir / "heldout_errors.npz", allow_pickle=False) as archive:
        analytic = archive["analytic_errors"].astype(np.float64)
        learned = archive["learned_errors"].astype(np.float64)
        episodes = archive["episode_ids"].astype(str)
        horizons = np.round(archive["horizons"].astype(np.float64), 6)
        np.testing.assert_allclose(analytic, archive["targets"], rtol=1e-6, atol=1e-6)
        np.testing.assert_allclose(learned, archive["targets"] - archive["residual_predictions"],
                                   rtol=1e-6, atol=1e-6)
    if analytic.shape != learned.shape or analytic.shape != (len(episodes), 2):
        raise ValueError("Unpaired or malformed prediction-error arrays")
    if not np.isfinite(analytic).all() or not np.isfinite(learned).all():
        raise ValueError("Non-finite prediction errors")
    if set(episodes) != set(metadata["split_summary"]["test"]["episodes"]):
        raise ValueError("Heldout episodes differ from the recorded test partition")
    if set(episodes) & set(calibration["calibration_episode_ids"]):
        raise ValueError("Calibration episodes overlap the test set")
    if calibration["fit_split"] != "calibration":
        raise ValueError("Calibration is not from the independent partition")
    scenario_by_episode = {row["episode_id"]: row["scenario"]
                           for row in metadata["dataset_metadata"]["episodes"]}
    scenarios = np.array([scenario_by_episode[episode] for episode in episodes])
    for method, errors in (("analytic", analytic), ("learned", learned)):
        np.testing.assert_allclose(mean_metrics(errors)["mean_displacement_error_m"],
                                   saved[method]["mean_displacement_error_m"], rtol=1e-5)
    groups = {"overall": paired_cluster_summary(analytic, learned, episodes, seed, repetitions)}
    for scenario in sorted(set(scenarios)):
        mask = scenarios == scenario
        groups[scenario] = paired_cluster_summary(analytic[mask], learned[mask], episodes[mask], seed, repetitions)
    per_horizon = []
    for horizon in np.unique(horizons):
        mask = horizons == horizon
        per_horizon.append({"horizon_seconds": float(horizon), "samples": int(mask.sum()),
                            "episodes": int(len(np.unique(episodes[mask]))),
                            "analytic": mean_metrics(analytic[mask]), "learned": mean_metrics(learned[mask])})
    summary = {
        "model_dir": str(model_dir.resolve()),
        "source_sha256": {name: file_hash(model_dir / name) for name in required},
        "report_script_sha256": file_hash(__file__), "groups": groups, "per_horizon": per_horizon,
        "bootstrap": {"repetitions": repetitions, "seed": seed, "unit": "whole episode",
                      "method": "paired sample errors -> per-episode means -> episode resampling",
                      "ci": "percentile 2.5%-97.5%", "difference": "learned minus analytic; negative is better",
                      "overall_clusters": groups["overall"]["episode_count"]},
        "calibration": {"fit_episode_count": len(calibration["calibration_episode_ids"]),
                        "quantile": calibration["quantile"], "buckets": calibration["buckets"],
                        "analytic": calibration["analytic"]["heldout_coverage"],
                        "learned": calibration["learned"]["heldout_coverage"]},
        "limitations": [
            "Prediction errors are frozen raw residual-network forecasts; online fallback can use CA",
            "Offline sample prediction error is not a closed-loop collision or progress result",
            "The pilot test partition has new seeds under the development scenario profile",
            "Episode bootstrap has few clusters and describes only this sampled simulation distribution",
            "Horizon curves mix available episodes and observation delays, not controlled causal horizon effects",
            "Calibration uses coordinate-wise empirical 95% error quantiles, not joint safety guarantees",
        ],
    }
    output.mkdir(parents=True, exist_ok=False)
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8), constrained_layout=True)
    h = [row["horizon_seconds"] for row in per_horizon]
    for axis, title, component in ((axes[0, 0], "Mean Euclidean position error", None),
                                   (axes[0, 1], "Longitudinal mean absolute error", 0),
                                   (axes[1, 0], "Lateral mean absolute error", 1)):
        for method, color, label in (("analytic", "#32689c", "CA"), ("learned", "#d17a27", "Learned residual")):
            y = [row[method]["mean_displacement_error_m"] if component is None else row[method]["mae_xy_m"][component]
                 for row in per_horizon]
            axis.plot(h, y, "o-", color=color, label=label, markersize=4)
        axis.set(xlabel="Horizon since latest observation (s)", ylabel="Error (m)", title=title)
        axis.grid(alpha=0.2)
        axis.legend()
    names = list(groups)
    points = [groups[name]["episode_equal_weight"]["difference_learned_minus_analytic_m"] for name in names]
    bounds = np.array([groups[name]["episode_equal_weight"]["bootstrap_95_percentile_ci_m"] for name in names])
    axes[1, 1].errorbar(points, range(len(names)),
                       xerr=np.vstack((np.asarray(points) - bounds[:, 0], bounds[:, 1] - points)),
                       fmt="o", capsize=4, color="#287d65")
    axes[1, 1].axvline(0, color="gray", linestyle="--", linewidth=1)
    axes[1, 1].set_yticks(range(len(names)), [f"{name} (n={groups[name]['episode_count']})" for name in names])
    axes[1, 1].set(xlabel="Learned minus CA episode mean (m); negative is better",
                   title=f"Paired episode bootstrap: 95% CI, {repetitions} resamples")
    axes[1, 1].grid(axis="x", alpha=0.2)
    fig.suptitle(f"Frozen pilot prediction test: {len(episodes):,} samples, {len(np.unique(episodes))} episodes", fontsize=15)
    fig.savefig(output / "prediction_errors.png", dpi=180)
    plt.close(fig)
    lines = ["# 冻结模型预测误差报告", "",
             f"模型：`{model_dir}`。测试共 {len(episodes)} 个配对样本、{len(np.unique(episodes))} 个独立回合。",
             "样本先在各回合内求平均，再对整回合均值进行配对 bootstrap；没有把相邻窗口当成独立回合。",
             f"采用 {repetitions} 次有放回重采样，固定随机种子 {seed}，95% 百分位区间。差值为学习模型减 CA，负数表示误差下降。", "",
             "## 总体与分场景", "",
             "| 范围 | 样本/回合数 | 样本均值 CA→学习 (m) | 回合等权 CA→学习 (m) | 回合均值差及95%CI (m) |",
             "|---|---:|---:|---:|---:|"]
    for name, group in groups.items():
        weighted, equal = group["sample_weighted"], group["episode_equal_weight"]
        lo, hi = equal["bootstrap_95_percentile_ci_m"]
        lines.append(f"| {name} | {group['sample_count']}/{group['episode_count']} | "
                     f"{weighted['analytic']['mean_displacement_error_m']:.4f} → {weighted['learned']['mean_displacement_error_m']:.4f} | "
                     f"{equal['analytic_mean_displacement_error_m']:.4f} → {equal['learned_mean_displacement_error_m']:.4f} | "
                     f"{equal['difference_learned_minus_analytic_m']:+.4f} [{lo:+.4f}, {hi:+.4f}] |")
    lines.extend(["", "样本均值按窗口数加权，回合均值让各回合权重相同；两者分母和统计对象不同。每场景仅5个测试回合时，区间稳定性仍有限。", "",
                  "![预测误差和回合区间](prediction_errors.png)", "", "## 按有效预测时距", "",
                  "时距包含观测年龄与决策未来偏移。不同点可能包含不同回合和延迟组合，曲线用于描述，不能解释为时距的独立因果效应。", "",
                  "| 时距(s) | 样本/回合 | CA位置误差(m) | 学习位置误差(m) | 差值(m) |", "|---:|---:|---:|---:|---:|"])
    for row in per_horizon:
        ca, learned_value = row["analytic"]["mean_displacement_error_m"], row["learned"]["mean_displacement_error_m"]
        lines.append(f"| {row['horizon_seconds']:.2f} | {row['samples']}/{row['episodes']} | {ca:.4f} | {learned_value:.4f} | {learned_value-ca:+.4f} |")
    lines.extend(["", "## 独立误差标定", "",
                  f"标定使用独立的 {len(calibration['calibration_episode_ids'])} 个回合；每个时距桶分别计算 x/y 绝对误差的经验95%分位数。未用测试覆盖率修改裕度。", "",
                  "| 方法 | 测试x覆盖率 | 测试y覆盖率 | 同时覆盖x和y |", "|---|---:|---:|---:|"])
    for method in ("analytic", "learned"):
        coverage = calibration[method]["heldout_coverage"]
        x, y = coverage["coordinate_coverage"]
        lines.append(f"| {method} | {x:.2%} | {y:.2%} | {coverage['joint_coverage']:.2%} |")
    lines.extend(["", "逐坐标95%经验分位数不等于联合95%覆盖，也不等于碰撞安全概率或形式化保证。", "",
                  "## 结论边界", "",
                  "- 这是冻结网络的原始残差预测评估；在线短历史或时距越界时会退回CA，实际控制结果应单独评测。",
                  "- 离线预测误差下降不证明闭环碰撞率下降、进度改善或真实道路安全。",
                  "- 当前测试采用同一 development 场景分布下的新种子，不能称作未见参数范围或真实交通泛化。",
                  "- 0.1秒等误差上升的切片完整保留；报告不据这些测试结果选择模型或调整超参数。",
                  "- 模型、输入证据及报告脚本的哈希记录在 summary.json；可逐回合核对原始配对误差。", ""])
    (output / "report.md").write_text("\n".join(lines))
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", default="models/pilot_v1")
    parser.add_argument("--output", required=True)
    parser.add_argument("--seed", type=int, default=20261023)
    parser.add_argument("--bootstrap-repetitions", type=int, default=1000)
    args = parser.parse_args()
    if args.bootstrap_repetitions < 1:
        parser.error("bootstrap-repetitions must be positive")
    result = generate_report(args.model_dir, args.output, args.seed, args.bootstrap_repetitions)
    print(json.dumps({"output": str(Path(args.output).resolve()),
                      "overall": result["groups"]["overall"]["episode_equal_weight"]}, indent=2))


if __name__ == "__main__":
    main()

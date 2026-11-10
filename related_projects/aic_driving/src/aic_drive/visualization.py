"""Offline, evidence-only replay and descriptive evaluation reports.

Plots consume saved records. This module never runs the simulator or changes raw
records, and deliberately labels all findings as structured-state simulation.
"""
from __future__ import annotations

import csv
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean, stdev
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np


_DISCLAIMER = "STRUCTURED-STATE SIMULATION | Development evaluation"


def _finite(value: Any, fallback: float = math.nan) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return fallback
    return number if math.isfinite(number) else fallback


def _observation_age(decision: dict[str, Any]) -> float:
    """Treat an absent observation as unavailable, preserving timestamp zero."""
    if decision.get("latest_observation_time") is None:
        return math.nan
    return _finite(decision.get("max_observation_age"))


def _read_run(run_dir: str | Path) -> tuple[Path, dict, list[dict]]:
    run_dir = Path(run_dir)
    with (run_dir / "trajectory.jsonl").open(encoding="utf-8") as stream:
        records = [json.loads(line) for line in stream if line.strip()]
    if not records:
        raise ValueError(f"Empty trajectory: {run_dir / 'trajectory.jsonl'}")
    last_time = -math.inf
    for index, record in enumerate(records):
        time = _finite(record.get("time"))
        ego = record.get("ego", {})
        if not math.isfinite(time) or time < last_time:
            raise ValueError(f"Non-monotonic or invalid time in trajectory row {index + 1}")
        if any(not math.isfinite(_finite(ego.get(key))) for key in ("x", "y", "speed")):
            raise ValueError(f"Invalid ego state in trajectory row {index + 1}")
        last_time = time
    config_path = run_dir / "config.json"
    config = json.loads(config_path.read_text(encoding="utf-8")) if config_path.exists() else {}
    return run_dir, config, records


def _series(records: list[dict]) -> dict[str, np.ndarray]:
    return {
        "time": np.array([record["time"] for record in records], dtype=float),
        "speed": np.array([record["ego"]["speed"] for record in records], dtype=float),
        "acceleration": np.array([
            _finite(record.get("decision", {}).get("acceleration")) for record in records
        ]),
        "age": np.array([
            _observation_age(record.get("decision", {})) for record in records
        ]),
    }


def _plot_series(axis, times, values, label: str, color: str = "#1368ad") -> None:
    axis.plot(times, values, color=color, linewidth=1.6)
    axis.set_xlabel("Simulation time (s)")
    axis.set_ylabel(label)
    axis.grid(alpha=0.25)
    if len(times) == 1:
        axis.set_xlim(times[0] - 0.05, times[0] + 0.05)


def export_run_plot(run_dir: str | Path) -> Path:
    """Save run_plot.png with positions, speed, acceleration and observation age."""
    run_dir, config, records = _read_run(run_dir)
    series = _series(records)
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    try:
        positions = axes[0, 0]
        positions.plot([r["ego"]["x"] for r in records], [r["ego"]["y"] for r in records],
                       label="Ego", color="#1368ad", linewidth=2)
        target_paths = defaultdict(list)
        for row in records:
            for target in row.get("targets", []):
                target_paths[str(target["track_id"])].append((target["x"], target["y"]))
        for track_id, points in sorted(target_paths.items()):
            xy = np.asarray(points)
            positions.plot(xy[:, 0], xy[:, 1], label=f"Target {track_id}", alpha=0.7)
        positions.axhline(2, linestyle="--", color="#999999", linewidth=0.7)
        positions.set(xlabel="World x (m)", ylabel="World y (m)", title="Reference-state paths")
        positions.legend(fontsize=8, loc="best")
        positions.grid(alpha=0.2)
        _plot_series(axes[0, 1], series["time"], series["speed"], "Ego speed (m/s)")
        _plot_series(axes[1, 0], series["time"], series["acceleration"], "Commanded acceleration (m/s²)")
        _plot_series(axes[1, 1], series["time"], series["age"], "Max observation age (s)", "#c47400")
        crash_times = [row["time"] for row in records if row.get("crashed", False)]
        if crash_times:
            for axis in (axes[0, 1], axes[1, 0], axes[1, 1]):
                axis.axvline(crash_times[0], color="#bf2525", linestyle="--", label="Collision")
            axes[0, 1].legend(fontsize=8)
        fig.suptitle(f"{_DISCLAIMER}\nScenario: {config.get('scenario', 'unspecified')} | "
                     "NaN age indicates unavailable observations", fontsize=11)
        output = run_dir / "run_plot.png"
        fig.savefig(output, dpi=150)
        return output
    finally:
        plt.close(fig)


def render_replay(run_dir: str | Path, output: str | Path | None = None, fps: int = 10) -> Path:
    """Render an MP4 from saved states; road positions are simulator reference states.

    Time-based frame selection preserves simulated duration even if source sampling
    frequency differs from the requested FPS. It holds the last available sample.
    """
    import imageio.v2 as imageio

    if not isinstance(fps, int) or fps <= 0:
        raise ValueError("fps must be a positive integer")
    run_dir, config, records = _read_run(run_dir)
    output = Path(output) if output is not None else run_dir / "replay.mp4"
    if output.suffix.lower() != ".mp4":
        raise ValueError("Replay output must have an .mp4 extension")
    output.parent.mkdir(parents=True, exist_ok=True)
    series = _series(records)
    times = series["time"]
    frame_times = np.arange(times[0], times[-1] + 1e-8, 1.0 / fps)
    if not len(frame_times) or frame_times[-1] < times[-1] - 1e-8:
        frame_times = np.append(frame_times, times[-1])
    indices = np.searchsorted(times, frame_times, side="right") - 1
    fig = plt.figure(figsize=(12, 6.75), dpi=80, facecolor="white")
    grid = fig.add_gridspec(3, 1, height_ratios=[1.45, 1, 1], hspace=0.62,
                          left=0.09, right=0.98, top=0.84, bottom=0.08)
    road, speed_axis, acceleration_axis = [fig.add_subplot(grid[i]) for i in range(3)]
    try:
        fig.suptitle(f"{_DISCLAIMER}\nOffline replay | Scenario: {config.get('scenario', 'unspecified')}",
                     y=0.98, fontsize=11)
        road.set(xlim=(-30, 70), ylim=(-3, 7), xlabel="Longitudinal distance from ego (m)",
                 ylabel="Lateral (m)")
        road.set_facecolor("#e9edf0")
        road.set_aspect("equal", adjustable="box")
        for line_y in (-2, 2, 6):
            road.axhline(line_y, color="#737b83", linewidth=1,
                         linestyle="--" if line_y == 2 else "-")
        road.set_yticks([0, 4], ["Lane 0", "Lane 1"])
        road.text(0.01, 0.02, "Vehicle positions: simulator reference states", transform=road.transAxes,
                  fontsize=7, color="#39424a")
        hud = fig.text(0.09, 0.86, "", fontsize=9, family="monospace")
        decision_text = fig.text(0.09, 0.815, "", fontsize=8, color="#444444")
        ego_patch = Rectangle((0, 0), 5, 2, facecolor="#1368ad", edgecolor="white", linewidth=1.5)
        road.add_patch(ego_patch)
        ego_label = road.text(0, 0, "EGO", fontsize=7, ha="center", va="center", color="white")
        ids = sorted({str(t["track_id"]) for row in records for t in row.get("targets", [])})
        target_artists = {}
        for track_id in ids:
            patch = Rectangle((0, 0), 5, 2, facecolor="#6c7a89", edgecolor="white", linewidth=1)
            road.add_patch(patch)
            label = road.text(0, 0, f"ID {track_id}", fontsize=7, ha="center", va="center",
                              color="white", clip_on=True)
            target_artists[track_id] = patch, label
        _plot_series(speed_axis, times, series["speed"], "Speed (m/s)")
        _plot_series(acceleration_axis, times, series["acceleration"], "Command a (m/s²)")
        speed_axis.set_xlabel("")
        cursors = [axis.axvline(times[0], color="#c47400", linewidth=1.5)
                   for axis in (speed_axis, acceleration_axis)]
        with imageio.get_writer(str(output), format="FFMPEG", mode="I", fps=fps,
                                codec="libx264", quality=7, macro_block_size=2) as writer:
            for index in indices:
                record = records[int(index)]
                ego, decision = record["ego"], record.get("decision", {})
                length, width = float(ego.get("length", 5)), float(ego.get("width", 2))
                ego_patch.set_bounds(-length / 2, ego["y"] - width / 2, length, width)
                ego_patch.set_facecolor("#bf2525" if record.get("crashed") else "#1368ad")
                ego_label.set_position((0, ego["y"]))
                for patch, label in target_artists.values():
                    patch.set_visible(False)
                    label.set_visible(False)
                for target in record.get("targets", []):
                    patch, label = target_artists[str(target["track_id"])]
                    relative_x = float(target["x"]) - float(ego["x"])
                    length, width = float(target.get("length", 5)), float(target.get("width", 2))
                    patch.set_bounds(relative_x - length / 2, float(target["y"]) - width / 2,
                                     length, width)
                    label.set_position((relative_x, target["y"]))
                    patch.set_visible(True)
                    label.set_visible(True)
                age = _observation_age(decision)
                age_label = f"{age:.2f}s" if math.isfinite(age) else "unavailable"
                status = "COLLISION" if record.get("crashed") else (
                    "EMERGENCY" if decision.get("emergency") else "NORMAL")
                if decision.get("degraded"):
                    status += " | DEGRADED"
                hud.set_text(f"t={record['time']:5.2f}s   speed={ego['speed']:5.2f}m/s   "
                             f"a={_finite(decision.get('acceleration')):5.2f}m/s²   "
                             f"age={age_label}   {status}")
                reason = str(decision.get("reason", "unspecified"))
                # English-only annotation avoids platform-specific missing CJK glyphs.
                reason = reason.encode("ascii", errors="replace").decode("ascii")
                decision_text.set_text(f"Decision: {reason[:115]}")
                for cursor in cursors:
                    cursor.set_xdata([record["time"], record["time"]])
                fig.canvas.draw()
                frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
                writer.append_data(frame)
        return output
    finally:
        plt.close(fig)


def wilson_interval(successes: int, total: int, z: float = 1.959963984540054) -> tuple[float, float]:
    """Two-sided 95% Wilson interval by default, including 0/N and N/N."""
    if total <= 0 or successes < 0 or successes > total:
        raise ValueError("Wilson interval requires 0 <= successes <= total and total > 0")
    proportion = successes / total
    denominator = 1 + z * z / total
    center = (proportion + z * z / (2 * total)) / denominator
    half_width = z * math.sqrt(proportion * (1 - proportion) / total + z * z / (4 * total**2)) / denominator
    lower = 0.0 if successes == 0 else max(0.0, center - half_width)
    upper = 1.0 if successes == total else min(1.0, center + half_width)
    return lower, upper


def _read_episodes(path: Path) -> list[dict]:
    required = {"scenario", "seed", "method", "delay", "collision", "progress_m",
                "decision_p95_ms", "emergency_count", "duration_s"}
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing episode columns: {', '.join(sorted(missing))}")
        episodes = []
        for row_number, raw in enumerate(reader, start=2):
            row = dict(raw)
            collision = str(row["collision"]).strip().lower()
            if collision not in {"0", "1", "0.0", "1.0", "true", "false"}:
                raise ValueError(f"Invalid collision value at CSV line {row_number}")
            row["collision"] = int(collision in {"1", "1.0", "true"})
            for key in ("delay", "progress_m", "decision_p95_ms", "emergency_count", "duration_s"):
                value = _finite(row.get(key))
                if not math.isfinite(value):
                    raise ValueError(f"Invalid {key} at CSV line {row_number}")
                row[key] = value
            if any(row[key] < 0 for key in ("delay", "decision_p95_ms", "emergency_count", "duration_s")):
                raise ValueError(f"Negative time/count at CSV line {row_number}")
            if not row["emergency_count"].is_integer():
                raise ValueError(f"Non-integer emergency_count at CSV line {row_number}")
            for key in ("progress_rate", "budget_s", "position_noise", "velocity_noise"):
                value = _finite(row.get(key))
                if row.get(key) not in (None, "") and not math.isfinite(value):
                    raise ValueError(f"Invalid {key} at CSV line {row_number}")
                row[key] = value if math.isfinite(value) else None
            if row["budget_s"] is not None:
                if row["budget_s"] <= 0:
                    raise ValueError(f"Non-positive budget_s at CSV line {row_number}")
                if row["progress_rate"] is not None and not math.isclose(
                    row["progress_rate"], row["progress_m"] / row["budget_s"], rel_tol=1e-5, abs_tol=1e-6
                ):
                    raise ValueError(f"progress_rate does not use budget_s at CSV line {row_number}")
            if any(row[key] is not None and row[key] < 0 for key in ("position_noise", "velocity_noise")):
                raise ValueError(f"Negative observation noise at CSV line {row_number}")
            row["profile"] = row.get("profile") or "unspecified"
            row["delay_mode"] = row.get("delay_mode") or "unspecified"
            episodes.append(row)
    if not episodes:
        raise ValueError("No episode rows; refusing to produce an empty evaluation report")
    return episodes


def _slug(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_.-]+", "_", value).strip("_") or "unspecified"


def _aggregate(episodes: list[dict]) -> list[dict]:
    grouped = defaultdict(list)
    for row in episodes:
        grouped[(row["scenario"], row["profile"], row["delay_mode"], row["position_noise"],
                 row["velocity_noise"], row["budget_s"], row["delay"], row["method"])].append(row)
    groups = []
    for (scenario, profile, mode, position_noise, velocity_noise, budget, delay, method), rows in sorted(
        grouped.items(), key=lambda item: str(item[0])
    ):
        collisions = sum(row["collision"] for row in rows)
        lower, upper = wilson_interval(collisions, len(rows))
        progress = [row["progress_m"] for row in rows]
        rates = [row["progress_rate"] for row in rows if row["progress_rate"] is not None]
        groups.append({
            "scenario": scenario, "profile": profile, "delay_mode": mode, "delay": delay,
            "position_noise": position_noise, "velocity_noise": velocity_noise, "budget_s": budget,
            "method": method, "episodes": len(rows), "collisions": collisions,
            "collision_rate": collisions / len(rows),
            "collision_wilson95_low": lower, "collision_wilson95_high": upper,
            "mean_progress_m": mean(progress),
            "std_progress_m": stdev(progress) if len(progress) > 1 else 0.0,
            "mean_progress_rate_mps": mean(rates) if len(rates) == len(rows) else None,
            "mean_episode_decision_p95_ms": mean(row["decision_p95_ms"] for row in rows),
            "emergency_count": int(sum(row["emergency_count"] for row in rows)),
            "mean_actual_duration_s": mean(row["duration_s"] for row in rows),
        })
    return groups


def _report_plot(groups: list[dict], path: Path, title: str) -> None:
    fig, (collision_axis, progress_axis) = plt.subplots(1, 2, figsize=(12, 4.6), constrained_layout=True)
    try:
        for method in sorted({group["method"] for group in groups}):
            rows = sorted((g for g in groups if g["method"] == method), key=lambda row: row["delay"])
            delays = [row["delay"] for row in rows]
            proportions = np.array([row["collision_rate"] for row in rows])
            lower = proportions - np.array([row["collision_wilson95_low"] for row in rows])
            upper = np.array([row["collision_wilson95_high"] for row in rows]) - proportions
            collision_axis.errorbar(delays, proportions, yerr=np.maximum([lower, upper], 0),
                                    fmt="o-", capsize=4, label=method, linewidth=1.3)
            progress_axis.errorbar(delays, [row["mean_progress_m"] for row in rows],
                                   yerr=[row["std_progress_m"] for row in rows],
                                   fmt="o-", capsize=4, label=method, linewidth=1.3)
        collision_axis.set(ylabel="Collision episode proportion", ylim=(-0.03, 1.03),
                           title="Collision rate: 95% Wilson interval")
        progress_axis.set(ylabel="Progress over configured budget (m)",
                          title="Mean progress: error bars show sample SD")
        for axis in (collision_axis, progress_axis):
            axis.set_xlabel("Configured observation delay (s)")
            axis.grid(alpha=0.25)
            axis.legend(fontsize=8)
        fig.suptitle(f"{_DISCLAIMER}\n{title}", fontsize=11)
        fig.savefig(path, dpi=150)
    finally:
        plt.close(fig)


def make_report(input_csv: str | Path, output_dir: str | Path) -> Path:
    """Build descriptive charts, summary.json and report.md using actual CSV values."""
    input_csv, output_dir = Path(input_csv), Path(output_dir)
    episodes = _read_episodes(input_csv)
    output_dir.mkdir(parents=True, exist_ok=True)
    groups = _aggregate(episodes)
    warnings = []
    pairing_keys = [(r["scenario"], r["seed"], r["method"], r["delay"], r["delay_mode"], r["profile"],
                     r["position_noise"], r["velocity_noise"], r["budget_s"])
                    for r in episodes]
    duplicate_rows = sum(count - 1 for count in Counter(pairing_keys).values() if count > 1)
    if duplicate_rows:
        warnings.append(f"发现 {duplicate_rows} 条重复条件与种子的记录；汇总保留原始行，可能影响独立样本解释。")
    method_keys = defaultdict(set)
    for row in episodes:
        method_keys[row["method"]].add((row["scenario"], row["seed"], row["delay"],
                                       row["delay_mode"], row["profile"], row["position_noise"],
                                       row["velocity_noise"], row["budget_s"]))
    if len({frozenset(keys) for keys in method_keys.values()}) > 1:
        warnings.append("方法间的场景/种子/延迟覆盖不完全一致；本报告不作成对优劣结论。")
    if any(row["progress_rate"] is None for row in episodes):
        warnings.append("部分记录缺少固定预算进度速率；未用实际运行时长替代预算作分母。")
    budgets = {row["budget_s"] for row in episodes if row["budget_s"] is not None}
    if len(budgets) > 1:
        warnings.append("输入包含不同固定时间预算，已分别分组；累计进度不能直接跨预算比较。")
    if any(row["budget_s"] is None for row in episodes):
        warnings.append("部分或全部 CSV 记录缺少 budget_s；本报告无法独立核验这些回合的固定时间预算。")
    if any(row["position_noise"] is None or row["velocity_noise"] is None for row in episodes):
        warnings.append("部分或全部记录缺少观测噪声配置；图表标为 unspecified，不假定无噪声。")
    source_versions = {row.get("source_sha256") for row in episodes if row.get("source_sha256")}
    if len(source_versions) > 1:
        warnings.append("记录来自多个源代码版本；请复核算法与场景是否保持可比较。")
    facets = defaultdict(list)
    for group in groups:
        facets[(group["scenario"], group["profile"], group["delay_mode"], group["position_noise"],
                group["velocity_noise"], group["budget_s"])].append(group)
    chart_paths = []
    for index, ((scenario, profile, mode, noise_x, noise_v, budget), values) in enumerate(
        sorted(facets.items(), key=lambda item: str(item[0])), start=1
    ):
        path = output_dir / f"comparison_{index:02d}_{_slug(scenario)}_{_slug(profile)}_{_slug(mode)}.png"
        nx = "unspecified" if noise_x is None else f"{noise_x:g}m"
        nv = "unspecified" if noise_v is None else f"{noise_v:g}m/s"
        duration = "unspecified" if budget is None else f"{budget:g}s"
        _report_plot(values, path, f"Scenario: {scenario} | Profile: {profile} | Delay mode: {mode}\n"
                     f"Noise std: position {nx}, velocity {nv} | Fixed budget: {duration}")
        chart_paths.append(path)
    summary = {
        "evaluation_type": "development_evaluation",
        "simulation_type": "structured_state_simulation",
        "source_csv": str(input_csv.resolve()),
        "episode_count": len(episodes), "collision_count": sum(r["collision"] for r in episodes),
        "methods": sorted(method_keys), "groups": groups, "warnings": warnings,
        "charts": [p.name for p in chart_paths],
        "interpretation": [
            "Descriptive results from supplied simulation records; no real-road safety claim.",
            "Collision intervals are 95% Wilson binomial intervals, not absolute safety guarantees.",
            "Progress bars show sample SD, not confidence intervals.",
            "Mean episode decision P95 is not the pooled P95 of all controller calls.",
            "No competition placement or national-stage performance is promised.",
        ],
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                                             encoding="utf-8")
    with (output_dir / "groups.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(groups[0]))
        writer.writeheader()
        writer.writerows(groups)
    lines = [
        "# 驾驶仿真开发性评测简报", "",
        "**本报告是开发性评测，基于结构化状态仿真；不代表实车验证，也不承诺国赛成绩。**", "",
        f"数据来源：`{input_csv.resolve()}`。共 {len(episodes)} 个回合，"
        f"{len(method_keys)} 种方法，合计 {summary['collision_count']} 个碰撞回合。"
        "合计值混合了不同条件，不能直接用作方法排名。", "",
        "## 分组结果", "",
        "场景、数据配置、延迟模式、观测噪声与时间预算分别分组。碰撞按发生碰撞的回合数统计；"
        "括号内为 95% Wilson 区间。进度是在配置的完整预算内累计，"
        "提前终止后不继续增加；速率以固定预算作分母。", "",
        "| 场景 | 配置（位置/速度噪声标准差；预算） | 延迟模式 | 延迟/s | 方法 | 碰撞回合 | 碰撞率与区间 | 平均进度/m | 预算进度速率/m·s⁻¹ | 平均回合P95/ms | 紧急制动决策次数 |",
        "|---|---|---|---:|---|---:|---|---:|---:|---:|---:|",
    ]
    for g in groups:
        rate = "未记录" if g["mean_progress_rate_mps"] is None else f"{g['mean_progress_rate_mps']:.2f}"
        labels = [str(g[key]).replace("|", "\\|") for key in ("scenario", "profile", "delay_mode", "method")]
        noise_x = "未记录" if g["position_noise"] is None else f"{g['position_noise']:g}m"
        noise_v = "未记录" if g["velocity_noise"] is None else f"{g['velocity_noise']:g}m/s"
        budget = "未记录" if g["budget_s"] is None else f"{g['budget_s']:g}s"
        lines.append(f"| {labels[0]} | {labels[1]}（{noise_x}/{noise_v}；{budget}） | {labels[2]} | {g['delay']:.3f} | {labels[3]} | "
                     f"{g['collisions']}/{g['episodes']} | {g['collision_rate']:.1%} "
                     f"[{g['collision_wilson95_low']:.1%}, {g['collision_wilson95_high']:.1%}] | "
                     f"{g['mean_progress_m']:.2f} | {rate} | {g['mean_episode_decision_p95_ms']:.3f} | {g['emergency_count']} |")
    lines += ["", "## 对比图", ""]
    for path in chart_paths:
        lines += [f"![分组碰撞率和行驶进度]({path.name})", ""]
    lines += ["## 怎样解读", "",
              "- 安全与效率需要一起看：碰撞少但几乎停车，不等于方案全面更好。",
              "- `0/N` 只表示这些样本没有碰撞；Wilson 区间仍有非零上界。",
              "- 进度误差线表示样本标准差。碰撞区间采用二项近似，各回合相关性会影响解释。",
              "- 表中的 P95 是各回合 P95 的平均值，不是把全部决策耗时合并后计算的 P95。",
              "- 本简报不自动宣称创新或显著优势；须结合配对实验、失败回放及独立测试分析。", ""]
    if all(row["collision"] == 0 for row in episodes):
        lines += ["本批所有方法均未出现碰撞，不能从中建立碰撞率优势；应优先比较行驶进度、"
                  "平稳性和计算开销，并检查场景覆盖。不要人为挑选数据制造优势。", ""]
    if warnings:
        lines += ["## 数据检查提示", ""] + [f"- {warning}" for warning in warnings] + [""]
    lines += ["可复查文件：`summary.json`、`groups.csv` 以及来源 CSV 的各回合 `run_dir` 原始日志。", ""]
    report_path = output_dir / "report.md"
    report_path.write_text("\n".join(lines), encoding="utf-8")
    return report_path

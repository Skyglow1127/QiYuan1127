#!/usr/bin/env python3
"""Offline 5 Hz ZOH counterfactual versus an existing software C2 replay.

Example:
  /usr/bin/python3 measure/compare_smoothness_offline.py \
    --input-dir measure/replay_episode41_final \
    --output-dir measure/smoothness_episode41_20261102

Reads saved arrays only, without model, camera, video, hardware or PL access.
Neither curve represents the current physical deployment or measured f_exec.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
LABELS = ("zoh_5hz_counterfactual", "software_c2_candidate")
UPDATE_HZ = 5
SOURCE_TOLERANCE_S = 1e-5


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def metric_module():
    spec = importlib.util.spec_from_file_location("robot_rt_smoothness_metrics", ROOT/"measure/smoothness_metrics.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def strict_zoh(time, source_time, actions, *, source_fps=20):
    """Anchor nominal updates to raw source[0]; never select a future source."""
    import numpy as np
    time, source_time, actions = map(np.asarray, (time, source_time, actions))
    if source_fps != 20 or source_time.ndim != 1 or len(source_time) < 2 or actions.shape != (len(source_time), 7):
        raise ValueError("expected saved 20 Hz source times and Nx7 projected actions")
    if time.ndim != 1 or not len(time) or any(not np.all(np.isfinite(x)) for x in (time, source_time, actions)):
        raise ValueError("ZOH arrays must contain finite time and seven-axis values")
    anchor = float(source_time[0])
    if np.any(np.diff(source_time) <= 0) or np.any(np.diff(time) <= 0) or time[0] < anchor:
        raise ValueError("strictly increasing times at or after source anchor required")
    ideal_source = anchor+np.arange(len(source_time))/source_fps
    if np.any(abs(source_time-ideal_source) > SOURCE_TOLERANCE_S):
        raise ValueError("source timestamps disagree with their recorded 20 Hz frame order")
    # Generate one extra boundary then retain only those actually observed;
    # this avoids a rounded floor losing a boundary without ever adding epsilon.
    boundaries = anchor+np.arange(int(math.floor((time[-1]-anchor)*UPDATE_HZ))+2)/UPDATE_HZ
    boundaries = boundaries[boundaries <= time[-1]]
    selected = np.searchsorted(source_time, boundaries, side="right")-1
    bins = np.searchsorted(boundaries, time, side="right")-1
    if np.any(selected < 0) or np.any(bins < 0):
        raise ValueError("ZOH has no available source at an observation time")
    observed = np.searchsorted(time, boundaries, side="left")
    ideal_indices = np.arange(len(boundaries))*4  # 20 Hz / 5 Hz, exact integer.
    prior = (ideal_indices < len(source_time)) & (selected < ideal_indices)
    lag = boundaries-source_time[selected]
    grid_delay = time[observed]-boundaries
    if np.any(lag < 0) or np.any(source_time[selected[bins]] > time):
        raise ValueError("internal causal ZOH selection failure")
    schedule = [{"nominal_boundary_s": float(b), "source_frame_index": int(index),
                 "source_timestamp_s": float(source_time[index]), "source_lag_s": float(delay),
                 "first_evaluation_time_s": float(time[obs]),
                 "evaluation_delay_s": float(grid), "conservative_prior_frame": bool(previous)}
                for b, index, delay, obs, grid, previous in zip(boundaries, selected, lag, observed, grid_delay, prior)]
    return actions[selected[bins]].copy(), {
        "nominal_update_hz": UPDATE_HZ, "nominal_update_period_s": 1/UPDATE_HZ,
        "source_anchor_s": anchor, "source_selection": "last raw timestamp <= nominal boundary, no epsilon",
        "source_timestamp_grid_tolerance_s": SOURCE_TOLERANCE_S,
        "maximum_source_timestamp_quantization_s": float(np.max(abs(source_time-ideal_source))),
        "conservative_prior_frame_boundary_count": int(np.sum(prior)),
        "maximum_selected_source_lag_s": float(np.max(lag)),
        "maximum_evaluation_grid_delay_s": float(np.max(grid_delay)),
        "nominal_schedule": schedule}


def load_inputs(directory):
    import numpy as np
    directory = Path(directory)
    report = json.loads((directory/"results.json").read_text())
    required = ("time", "recorded_source_time", "recorded_projected_actions", "recorded_linear", "c2")
    with np.load(directory/"replay_arrays.npz", allow_pickle=False) as archive:
        arrays = {key: np.asarray(archive[key], dtype=float) for key in required}
    names, units = report["action_names"], report["action_units"]
    if len(names) != 7 or units != ["deg"]*6+["m"] or report["fps_dataset"] != 20:
        raise ValueError("saved replay must declare seven action names, six deg axes, gripper m and 20 Hz source")
    rate = report["scenario_output_hz"]
    if isinstance(rate, bool) or not isinstance(rate, (int, float)) or not math.isfinite(rate) or rate <= 0:
        raise ValueError("scenario output rate must be positive and finite")
    time = arrays["time"]
    if time.ndim != 1 or len(time) < 13:
        raise ValueError("saved uniform evaluation grid needs at least 13 samples")
    if any(not np.all(np.isfinite(value)) for value in arrays.values()):
        raise ValueError("saved arrays contain nonfinite values")
    if any(arrays[key].shape != (len(time), 7) for key in ("recorded_linear", "c2")):
        raise ValueError("saved candidate and reference must share the evaluation grid")
    tolerance = min(1e-9, 1/rate*1e-6)
    if np.any(np.diff(time) <= 0) or np.any(abs(np.diff(time)-1/rate) > tolerance):
        raise ValueError("saved evaluation grid must remain uniform; no resampling")
    zoh, schedule = strict_zoh(time, arrays["recorded_source_time"], arrays["recorded_projected_actions"])
    reference = np.column_stack([np.interp(time, arrays["recorded_source_time"], arrays["recorded_projected_actions"][:, axis]) for axis in range(7)])
    if not np.allclose(reference, arrays["recorded_linear"], rtol=0, atol=1e-9):
        raise ValueError("saved recorded_linear is not the same projected source reference")
    if report.get("ticks", len(time)) != len(time) or report.get("episode_frames", len(arrays["recorded_source_time"])) != len(arrays["recorded_source_time"]):
        raise ValueError("saved metadata sample counts disagree with arrays")
    config = {"schema": "robot_rt.smoothness.v1", "input_role": "offline_replay",
              "units": units, "sample_period_s": 1/rate, "uniform_tolerance_s": tolerance,
              "filter": metric_module().FILTER, "velocity_jump_threshold": [1.0]*6+[0.001], "settling": None}
    return report, arrays, zoh, schedule, config


def write_csv(path, time, command, reference, axes):
    with Path(path).open("x", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s"]+[f"{prefix}_{name}" for prefix in ("cmd", "ref") for name in axes])
        for t, q, ref in zip(time, command, reference):
            writer.writerow([float(t), *map(float, q), *map(float, ref)])


def plots(directory, report, arrays, zoh, results):
    import numpy as np
    if "MPLCONFIGDIR" not in os.environ:
        os.environ["MPLCONFIGDIR"] = tempfile.mkdtemp(prefix="robot_rt_compare_mpl_")
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    relative = arrays["time"]-arrays["recorded_source_time"][0]
    fig, axes = plt.subplots(7, 1, figsize=(11, 13), sharex=True, constrained_layout=True)
    for axis, ax in enumerate(axes):
        for label, values in (("5 Hz ZOH counterfactual", zoh), ("Software C2 reference (candidate)", arrays["c2"]), ("Recorded linear reference", arrays["recorded_linear"])):
            ax.plot(relative, values[:, axis], label=label, linewidth=.8)
        ax.set_ylabel(report["action_names"][axis]+"\n"+report["action_units"][axis]);ax.grid(alpha=.25)
    axes[0].legend(fontsize=8, ncol=3);axes[-1].set_xlabel("Time from recorded source anchor (s)")
    fig.suptitle("Offline 5 Hz ZOH vs Software C2 reference")
    fig.savefig(directory/"position.png", dpi=120);plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), gridspec_kw={"width_ratios": [3, 1]}, constrained_layout=True)
    for index, (label, result) in enumerate(zip(("5 Hz ZOH", "Software C2 reference"), results)):
        values = [axis["jerk_rms"] for axis in result["axes"]]
        axes[0].bar(np.arange(6)+(index-.5)*.36, values[:6], width=.36, label=label)
        axes[1].bar(index, values[6], label=label)
    axis_units = ("Jerk RMS (deg/s³)", "Gripper jerk RMS (m/s³)")
    scale_labels = []
    for axis, indices, unit in zip(axes, (range(6), (6,)), axis_units):
        selected = np.asarray([[result["axes"][i]["jerk_rms"] for i in indices] for result in results])
        if np.all(selected > 0):
            axis.set_yscale("log")
            axis.set_ylabel(unit+"; log scale")
            scale_labels.append("log scale")
        else:
            # A genuine zero cannot be represented on a logarithmic axis.
            # Keep this panel linear and label exact zeros without substituting
            # epsilon or changing the numeric table/metric JSON.
            axis.set_ylabel(unit+"; linear scale (contains zero)")
            axis.set_ylim(bottom=0)
            for bar in axis.patches:
                if bar.get_height() == 0:
                    axis.annotate("0", (bar.get_x()+bar.get_width()/2, 0),
                                  xytext=(0, 4), textcoords="offset points", ha="center", fontsize=8)
            scale_labels.append("linear scale for zero values")
        axis.grid(axis="y", which="both", alpha=.2)
    axes[0].set_xticks(np.arange(6), [f"J{i}" for i in range(1, 7)]);axes[0].legend(fontsize=8)
    axes[1].set_xticks([0, 1], ["5 Hz ZOH", "Software C2"], rotation=15)
    scale_note = "log scale" if all(label == "log scale" for label in scale_labels) else "; ".join(dict.fromkeys(scale_labels))
    fig.suptitle("Offline 5 Hz ZOH vs Software C2 reference — "+scale_note)
    fig.savefig(directory/"jerk_rms.png", dpi=130);plt.close(fig)


def run_comparison(input_dir, output_dir, *, make_plots=True):
    input_dir, output_dir = Path(input_dir), Path(output_dir)
    if output_dir.exists():
        raise ValueError("output directory exists; existing evidence is never overwritten")
    report, arrays, zoh, schedule, config = load_inputs(input_dir)
    metrics = metric_module()
    metrics.validate_config(config)
    output_dir.mkdir(parents=True, exist_ok=False)
    config_path = output_dir/"locked_config.json"
    config_path.write_text(json.dumps(config, indent=2, allow_nan=False)+"\n")
    results = []
    for label, command in zip(LABELS, (zoh, arrays["c2"])):
        csv_path, result_path = output_dir/(label+".csv"), output_dir/(label+"_metrics.json")
        write_csv(csv_path, arrays["time"], command, arrays["recorded_linear"], metrics.AXES)
        metrics.main(["--csv", str(csv_path), "--config", str(config_path), "--output", str(result_path)])
        results.append(json.loads(result_path.read_text()))
    if results[0]["config_sha256"] != results[1]["config_sha256"]:
        raise ValueError("comparison configuration hashes differ")
    manifest = {"schema": "robot_rt.offline_smoothness_comparison.v1", "provenance": "recorded_action_counterfactual",
        "labels": list(LABELS), "current_physical_deployment_baseline": False, "pl_output": False,
        "hardware_unverified": True, "physical_f_exec_hz": None, "episode_index": report.get("episode_index"),
        "evaluation_grid_hz": report["scenario_output_hz"], "evaluation_samples": len(arrays["time"]),
        "feedback_available": False, "config_sha256": digest(config_path), "zoh_schedule": schedule,
        "recorded_ingress_projection": report.get("recorded_ingress_projection"),
        "source_sha256": {str((input_dir/name).resolve()): digest(input_dir/name) for name in ("replay_arrays.npz", "results.json")},
        "script_sha256": {str(path.relative_to(ROOT)): digest(path) for path in (Path(__file__).resolve(), ROOT/"measure/smoothness_metrics.py", ROOT/"measure/replay_chunks.py")},
        "historical_raw_source_sha256": report.get("sha256", {}),
        "limitations": ["Both are offline software curves using recorded actions, not ACT predictions, PL output or the current robot baseline.",
            "Nominal 5 Hz update boundaries and the evaluation grid differ; first sampled observation of an update is quantized to the next grid point.",
            "Raw source timestamps are never shifted or rounded. Slightly late float32 source timestamps conservatively select the prior frame; this extra source lag is not a hardware or frequency effect.",
            "C2 shaping may trade smoothness for reference lag and contact timing changes; no physical acceptance is inferred.",
            "Feedback tracking and physical settling cannot be calculated; reference deviation is reported separately."]}
    (output_dir/"comparison.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    lines = ["# Offline smoothness counterfactual", "", "Software curves only: no current physical baseline, PL output or measured f_exec.", "",
        f"Nominal ZOH schedule: 5 Hz / 0.2 s from source anchor. Common evaluation grid: {report['scenario_output_hz']:g} Hz; same saved C2 and projected recorded_linear reference.",
        f"Conservative prior-frame boundaries due to raw timestamp rounding: {schedule['conservative_prior_frame_boundary_count']}; maximum selected-source lag {schedule['maximum_selected_source_lag_s']:.9g} s; maximum first-observation grid delay {schedule['maximum_evaluation_grid_delay_s']:.9g} s. These are separate effects, not hardware timing findings.",
        "Feedback tracking and settling are unavailable. Both metric JSONs share locked_config.json and its exact SHA256.", "",
        "| Axis | Unit | 5 Hz ZOH jerk RMS | Software C2 jerk RMS | ZOH reference P99 | C2 reference P99 |", "|---|---|---:|---:|---:|---:|"]
    for index, name in enumerate(report["action_names"]):
        a, b = results[0]["axes"][index], results[1]["axes"][index]
        lines.append(f"| {name} | {report['action_units'][index]} | {a['jerk_rms']:.7g} | {b['jerk_rms']:.7g} | {a['reference_deviation_p99']:.7g} | {b['reference_deviation_p99']:.7g} |")
    lines += ["", "The binomial analysis filter is shared. These values differ from the earlier B report's filter; old arrays and reports remain unchanged.", "", "![Software positions](position.png)", "", "![Common-filter jerk RMS](jerk_rms.png)", ""]
    (output_dir/"report.md").write_text("\n".join(lines))
    if make_plots:
        plots(output_dir, report, arrays, zoh, results)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input-dir", type=Path, required=True, help="Existing saved replay arrays and results, read only")
    parser.add_argument("--output-dir", type=Path, required=True, help="New evidence directory; never overwrite")
    args = parser.parse_args(argv)
    try:
        result = run_comparison(args.input_dir, args.output_dir)
        print(json.dumps({"output": str(args.output_dir.resolve()), "labels": result["labels"], "physical_f_exec_hz": None}))
        return 0
    except (ValueError, TypeError, KeyError, OSError, ImportError) as exc:
        parser.exit(2, f"offline comparison: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Offline recorded-action replay; no robot imports or physical frequency claim.

Example (all bounds are explicitly supplied design scenarios, not measured):
  /usr/bin/python3 measure/replay_chunks.py --contract spec/act_contract.json \
    --dataset-python /path/to/existing/python --episode 41 --f-exec 120 \
    --limits-json reviewed_design_test.json --output-dir measure/replay_example
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"sw"))


def digest(path):
    with Path(path).open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def load_episode(dataset_root, episode, interpreter):
    """Use an existing Arrow environment in a bounded, hardware-free subprocess."""
    code = '''import json,sys
from pathlib import Path
import pyarrow.dataset as ds
root=Path(sys.argv[1]); episode=int(sys.argv[2])
table=ds.dataset(root/'data',format='parquet').to_table(columns=['action','timestamp','frame_index','__filename'],filter=ds.field('episode_index')==episode)
print(json.dumps({key:table[key].to_pylist() for key in table.column_names},allow_nan=False))
'''
    completed = subprocess.run([interpreter, "-c", code, str(dataset_root), str(episode)], capture_output=True, text=True, timeout=60)
    if completed.returncode:
        raise ValueError("existing dataset Python failed: "+completed.stderr[-1500:])
    return json.loads(completed.stdout)


def derivative_metrics(q, fs, cutoff):
    import numpy as np
    from scipy import signal
    if not math.isfinite(cutoff) or cutoff <= 0 or cutoff >= fs*.4:
        raise ValueError("common analysis cutoff must be positive and below 0.4 * scenario sample rate")
    half = max(4, int(math.ceil(2*fs/cutoff)))
    if len(q) <= 2*half+8:
        raise ValueError("recorded episode too short for supplied common analysis cutoff")
    kernel = signal.firwin(2*half+1, cutoff, fs=fs, window="hann")
    filtered = np.column_stack([signal.convolve(q[:, axis], kernel, mode="same") for axis in range(q.shape[1])])
    v = np.gradient(filtered, 1/fs, axis=0)
    a = np.gradient(v, 1/fs, axis=0)
    j = np.gradient(a, 1/fs, axis=0)
    keep = slice(half+3, -half-3)
    return {"velocity_rms": np.sqrt(np.mean(v[keep]**2, axis=0)).tolist(),
            "acceleration_rms": np.sqrt(np.mean(a[keep]**2, axis=0)).tolist(),
            "jerk_rms": np.sqrt(np.mean(j[keep]**2, axis=0)).tolist(),
            "jerk_max_abs": np.max(abs(j[keep]), axis=0).tolist()}, j


def run_replay(contract, scenario, episode, rate, *, project_recorded_inputs=False):
    import numpy as np
    from rt_adapter.scheduler import ChunkScheduler, TimedChunk
    from rt_adapter.trajectory import C2Tracker, MotionBounds, exact_tick_times
    from rt_adapter.degradation import DegradationController
    if scenario.get("provenance") != "design_test_only":
        raise ValueError("limits JSON must explicitly declare provenance=design_test_only")
    bounds = MotionBounds.from_dict(scenario["motion_bounds"])
    fps, chunk_size, consume = float(contract["fps_dataset"]), int(contract["chunk_size"]), int(contract["runtime_n_action_steps"])
    names, units = contract["action_names"], contract["action_units"]
    if len(names) != len(bounds.lower) or len(units) != len(names) or not 0 < consume <= chunk_size:
        raise ValueError("contract/bounds dimension or consumption mismatch")
    order = np.argsort(episode["frame_index"])
    source_time = np.asarray(episode["timestamp"], dtype=float)[order]
    actions = np.asarray(episode["action"], dtype=float)[order]
    if len(actions) < 3 or actions.ndim != 2 or actions.shape[1] != len(names) or not np.all(np.isfinite(actions)) or np.any(np.diff(source_time) <= 0):
        raise ValueError("invalid recorded episode arrays")
    if not np.allclose(np.diff(source_time), 1/fps, atol=1e-5, rtol=0):
        raise ValueError("recorded episode timestamps disagree with actual dataset fps")
    raw_actions = actions.copy()
    out_of_bounds = (actions < bounds.lower) | (actions > bounds.upper)
    if np.any(out_of_bounds):
        if not project_recorded_inputs:
            raise ValueError("recorded actions outside explicit scenario bounds; no implicit clipping. Review raw values; --project-recorded-inputs explicitly projects the SAME inputs for ALL baselines.")
        actions = np.clip(actions, bounds.lower, bounds.upper)
    projection = {"explicit_cli_opt_in": bool(project_recorded_inputs), "projected_frames_per_axis": np.sum(out_of_bounds, axis=0).tolist(),
        "maximum_abs_correction_per_axis": np.max(abs(actions-raw_actions), axis=0).tolist(),
        "applied_equally_to": ["every_consume_window_one_step", "linear_model_time", "c2_reference", "ps_baseline_same_algorithm"]}
    times = exact_tick_times(float(source_time[0]), float(source_time[-1]-source_time[0]), rate)
    output_times = times+1/rate  # advance() returns the state at the interval END.
    reference = np.column_stack([np.interp(output_times, source_time, actions[:, axis]) for axis in range(len(names))])
    slots = np.minimum(((times-source_time[0])*fps+1e-8).astype(int), len(actions)-1)
    coarse = actions[np.minimum((slots//consume)*consume, len(actions)-1)]
    tracker = C2Tracker(actions[0], bounds, float(scenario["response_time_s"]))
    ps_tracker = C2Tracker(actions[0], bounds, float(scenario["response_time_s"]))
    scheduler = ChunkScheduler(consume_steps=consume, source_max_age_s=float(scenario["source_max_age_s"]), slot_lateness_s=float(scenario["slot_lateness_s"]), bounds=bounds)
    degrade = DegradationController(tracker, consume_period_s=consume/fps, source_max_age_s=float(scenario["source_max_age_s"]), stop_velocity_tolerance=1e-6)
    shaped, ps, velocities, accelerations, jerks, events, source_records = [], [], [], [], [], [], []
    next_chunk = 0
    last_source = None
    for time in times:
        while next_chunk < len(actions) and source_time[next_chunk] <= time+1e-9:
            chunk = TimedChunk(next_chunk//consume, float(source_time[next_chunk]), float(source_time[next_chunk]), fps, actions[next_chunk:next_chunk+chunk_size])
            try:
                accepted = scheduler.accept(chunk, now=float(time), current_target=tracker.q)
                source_records.append({"sequence": chunk.sequence, "observation_at": chunk.observation_at, "produced_at": chunk.produced_at, **accepted})
                last_source = chunk.observation_at
            except ValueError as exc:
                events.append({"time": float(time), "kind": "rejected_chunk", "reason": str(exc)})
            next_chunk += consume
        sample = scheduler.reference_at(now=float(time))
        decision = degrade.evaluate(now=float(time), observation_at=last_source, block_available=sample is not None)
        if decision.state == "FAULT":
            raise ValueError(f"offline replay fault at {time:.9g}: {decision.reason}; normal target output is latched off")
        if decision.state != "RUN":
            state = degrade.brake_tick(1/rate)
            ps_state = ps_tracker.advance(ps_tracker.q, 1/rate, braking=True)
            events.append({"time": float(time), "kind": decision.state, "reason": decision.reason})
        else:
            if sample is None:
                raise ValueError("RUN decision requires a fresh scheduled sample")
            state = tracker.advance(sample.target, 1/rate)
            ps_state = ps_tracker.advance(sample.target, 1/rate)
        shaped.append(state["q"]); ps.append(ps_state["q"])
        velocities.append(state["v"]); accelerations.append(state["a"]); jerks.append(state["j"])
    shaped, ps, velocities, accelerations, jerks = map(np.asarray, (shaped, ps, velocities, accelerations, jerks))
    metrics, derivative_arrays = {}, {}
    for label, q in (("every_consume_window_one_step", coarse), ("linear_model_time", reference), ("c2_reference", shaped), ("ps_baseline_same_algorithm", ps)):
        metrics[label], derivative_arrays[label] = derivative_metrics(q, rate, float(scenario["analysis_cutoff_hz"]))
        metrics[label]["recorded_reference_error_rms"] = np.sqrt(np.mean((q-reference)**2, axis=0)).tolist()
    physical_bounds_pass = bool(np.all(abs(velocities) <= bounds.velocity+1e-8) and np.all(abs(accelerations) <= bounds.acceleration+1e-8) and np.all(abs(jerks) <= bounds.jerk+1e-8) and np.all(shaped >= bounds.lower-1e-8) and np.all(shaped <= bounds.upper+1e-8))
    report = {"provenance": "design_test_only", "physical_f_exec_hz": None, "scenario_output_hz": rate,
        "description": "Recorded demonstration actions, not ACT predictions or physical motion. C2 reference is the same algorithm in PS baseline; no PL or timing benefit is claimed.",
        "fps_dataset": fps, "chunk_size": chunk_size, "consume_steps": consume, "action_names": names, "action_units": units,
        "supplied_design_scenario": scenario,
        "recorded_ingress_projection": projection,
        "episode_frames": len(actions), "ticks": len(times), "duration_s": float(times[-1]-times[0]),
        "common_analysis_filter_cutoff_hz": scenario["analysis_cutoff_hz"], "metrics": metrics,
        "continuous_time_analytic_limiter_bounds_pass": physical_bounds_pass, "ps_same_algorithm_max_abs_difference": float(np.max(abs(shaped-ps))),
        "analytic_c2_max_abs": {"velocity": np.max(abs(velocities), axis=0).tolist(), "acceleration": np.max(abs(accelerations), axis=0).tolist(), "jerk": np.max(abs(jerks), axis=0).tolist()},
        "events": events, "source_records": source_records, "limitations": ["Motion bounds and response time are supplied offline design scenarios, not validated hardware limits.", "No physical f_exec, receiver acceptance, model inference latency or robot smoothness was measured.", "C2 shaping introduces tracking lag and can alter task/contact timing; it is not enabled in deployment.", "A state with no feasible bounded jerk faults; software does not claim an instantaneous safe physical stop."]}
    arrays = {"time": output_times, "recorded_linear": reference, "coarse": coarse, "c2": shaped, "ps_baseline": ps, "velocity": velocities, "acceleration": accelerations, "jerk": jerks,
        "recorded_source_time": source_time, "recorded_raw_actions": raw_actions, "recorded_projected_actions": actions}
    return report, arrays, derivative_arrays


def plots(output, report, arrays, derivative_arrays):
    if "MPLCONFIGDIR" not in os.environ: os.environ["MPLCONFIGDIR"] = tempfile.mkdtemp(prefix="robot_rt_b_mpl_")
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    import numpy as np
    count = len(report["action_names"])
    time = arrays["time"]-arrays["time"][0]
    for kind in ("position", "jerk"):
        fig, axes = plt.subplots(count, 1, figsize=(11, 2.1*count), sharex=True, constrained_layout=True)
        for axis, ax in enumerate(np.atleast_1d(axes)):
            if kind == "position":
                traces = [("one step per consume window", arrays["coarse"]), ("recorded linear", arrays["recorded_linear"]), ("C2 / identical PS baseline", arrays["c2"])]
            else:
                traces = [("one step per consume window", derivative_arrays["every_consume_window_one_step"]), ("recorded linear", derivative_arrays["linear_model_time"]), ("C2 / identical PS baseline", derivative_arrays["c2_reference"])]
            for label, values in traces:
                if kind == "jerk":
                    edge = max(4, int(math.ceil(2*report["scenario_output_hz"]/report["common_analysis_filter_cutoff_hz"])))+3
                    ax.plot(time[edge:-edge], values[edge:-edge, axis], label=label, linewidth=.9)
                else:
                    ax.plot(time, values[:, axis], label=label, linewidth=.9)
            unit = report["action_units"][axis]+("/s³" if kind == "jerk" else "")
            ax.set_ylabel(report["action_names"][axis]+"\n"+unit)
            ax.grid(alpha=.25)
        axes[0].legend(fontsize="small", ncol=3)
        axes[-1].set_xlabel("Recorded episode time (s)")
        fig.suptitle(f"OFFLINE DESIGN TEST ONLY — {kind}; scenario {report['scenario_output_hz']:g} Hz, physical f_exec unknown")
        fig.savefig(output/f"{kind}.png", dpi=130)
        fig.savefig(output/f"{kind}.svg")
        plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--contract", required=True, type=Path)
    parser.add_argument("--dataset-python", required=True, help="Existing local Python containing pyarrow; never installs dependencies")
    parser.add_argument("--episode", required=True, type=int)
    parser.add_argument("--f-exec", required=True, type=float, help="Explicit OFFLINE scenario rate only; physical execution frequency remains unknown")
    parser.add_argument("--limits-json", required=True, type=Path, help="Explicit design_test_only bounds/timing/filter JSON; no physical defaults")
    parser.add_argument("--project-recorded-inputs", action="store_true", help="Explicitly project out-of-bounds recorded labels into supplied bounds, identically for EVERY baseline; raw values/hashes and correction counts are preserved")
    parser.add_argument("--output-dir", required=True, type=Path, help="New path; existing evidence is not overwritten")
    args = parser.parse_args(argv)
    if args.output_dir.exists(): parser.error("output path exists; choose a new evidence directory")
    try:
        if not math.isfinite(args.f_exec) or args.f_exec <= 0 or args.episode < 0: raise ValueError("positive explicit output rate and nonnegative episode required")
        contract = json.loads(args.contract.read_text())
        scenario = json.loads(args.limits_json.read_text())
        dataset_root = Path(contract["dataset_root"])
        dataset_info = dataset_root/"meta"/"info.json"
        info = json.loads(dataset_info.read_text())
        if info["fps"] != contract["fps_dataset"] or info["features"]["action"]["names"] != contract["action_names"]:
            raise ValueError("contract disagrees with actual dataset metadata")
        episode = load_episode(dataset_root, args.episode, args.dataset_python)
        report, arrays, derivatives = run_replay(contract, scenario, episode, args.f_exec, project_recorded_inputs=args.project_recorded_inputs)
        report["episode_index"] = args.episode
        report["invocation"] = [sys.executable, str(Path(__file__).resolve()), *(sys.argv[1:] if argv is None else argv)]
        files = [args.contract, args.limits_json, dataset_info, *set(episode["__filename"])]
        report["sha256"] = {str(Path(path).resolve()): digest(path) for path in files}
        report["implementation_sha256"] = {str(path.relative_to(ROOT)): digest(path) for path in [Path(__file__), *sorted((ROOT/"sw"/"rt_adapter").glob("*.py"))]}
        args.output_dir.mkdir(parents=True, exist_ok=False)
        import numpy as np
        np.savez_compressed(args.output_dir/"replay_arrays.npz", **arrays)
        plots(args.output_dir, report, arrays, derivatives)
        (args.output_dir/"results.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
        lines = ["# Offline recorded-action trajectory replay", "", "**Design scenario only. Physical f_exec remains unknown. No motion or model inference occurred.**", "", f"Episode {args.episode}; dataset {report['fps_dataset']:g} Hz; explicit scenario {args.f_exec:g} Hz; consume {report['consume_steps']} of chunk {report['chunk_size']}.", "", "PS baseline uses exactly the same C2 algorithm. This replay does not claim a PL timing benefit. The coarse comparison emits one recorded target per consume window; it is not evidence of the live controller's behavior.", "", f"Recorded ingress projection explicitly requested: {args.project_recorded_inputs}. Per-axis corrected frame counts: {report['recorded_ingress_projection']['projected_frames_per_axis']}; maximum corrections: {report['recorded_ingress_projection']['maximum_abs_correction_per_axis']}. Every baseline receives exactly the same resulting input; raw files remain unchanged.", "", "| Axis | Unit | Coarse jerk RMS | C2 jerk RMS | C2 tracking RMS |", "|---|---|---:|---:|---:|"]
        for i, name in enumerate(report["action_names"]):
            lines.append(f"| {name} | {report['action_units'][i]} | {report['metrics']['every_consume_window_one_step']['jerk_rms'][i]:.6g} | {report['metrics']['c2_reference']['jerk_rms'][i]:.6g} | {report['metrics']['c2_reference']['recorded_reference_error_rms'][i]:.6g} |")
        lines += ["", f"Analytic supplied-bound checks: {report['continuous_time_analytic_limiter_bounds_pass']}. Identical PS baseline max difference: {report['ps_same_algorithm_max_abs_difference']:.6g}. Rejected/fallback events: {len(report['events'])}.", "", "All metric traces use the same physical analysis lowpass before derivatives; the limiter's analytic jerk bound is also reported separately. Tracking lag and changed contact timing remain limitations.", "", "![Recorded position](position.png)", "", "![Common-filter jerk comparison](jerk.png)", "", "Invocation, raw input hashes, source timestamps, supplied constraints and every event are in results.json. Arrays are in replay_arrays.npz.", ""]
        (args.output_dir/"report.md").write_text("\n".join(lines))
        print(json.dumps({"report": str((args.output_dir/"report.md").resolve()), "provenance": "design_test_only", "physical_f_exec_hz": None}))
        return 0
    except (ValueError, KeyError, OSError, RuntimeError, subprocess.SubprocessError) as exc:
        parser.exit(2, f"offline replay: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())

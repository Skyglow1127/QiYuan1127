"""Synthetic mathematical fixtures only; no hardware or physical acceptance data."""
import copy
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("smoothness_metrics", ROOT/"measure/smoothness_metrics.py")
metrics = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(metrics)


def configuration(role="offline_replay", dt=0.01):
    return {"schema": "robot_rt.smoothness.v1", "input_role": role,
        "units": ["deg"]*6+["m"], "sample_period_s": dt, "uniform_tolerance_s": 1e-9,
        "filter": copy.deepcopy(metrics.FILTER), "velocity_jump_threshold": [1]*6+[0.001],
        "settling": None}


def samples(count=101, dt=0.01):
    time = np.arange(count)*dt
    return {"time": time, "command": np.zeros((count, 7)), "feedback": None, "reference": None}


def write_csv(path, data):
    prefixes = ["cmd"]+(["fb"] if data["feedback"] is not None else [])+(["ref"] if data["reference"] is not None else [])
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s"]+[f"{prefix}_{name}" for prefix in prefixes for name in metrics.AXES])
        for index, time in enumerate(data["time"]):
            vectors = [data["command"]]+([data["feedback"]] if data["feedback"] is not None else [])+([data["reference"]] if data["reference"] is not None else [])
            writer.writerow([time]+[value for vector in vectors for value in vector[index]])


class SmoothnessMetricsTests(unittest.TestCase):
    def test_cubic_third_derivative_has_correct_physical_units(self):
        data = samples()
        coefficients = np.asarray([1, -2, 3, -4, 5, -6, 0.01])
        data["command"] = data["time"][:, None]**3*coefficients
        result = metrics.analyze(data, configuration())
        np.testing.assert_allclose([axis["jerk_rms"] for axis in result["axes"]], 6*abs(coefficients), rtol=1e-9, atol=1e-9)
        self.assertEqual(result["axes"][0]["jerk_unit"], "deg/s^3")
        self.assertEqual(result["axes"][-1]["jerk_unit"], "m/s^3")
        self.assertEqual(result["analysis_sample_count"], 91)
        self.assertAlmostEqual(result["analysis_duration_s"], 0.9)

    def test_velocity_jump_counts_adjacent_pairs_above_frozen_threshold(self):
        data = samples()
        data["command"] = np.tile((data["time"]**2)[:, None], (1, 7))
        config = configuration()
        # q=t^2 gives v=2t plus a filter-dependent constant: delta v=0.02.
        config["velocity_jump_threshold"] = [0.019]*7
        result = metrics.analyze(data, config)
        self.assertEqual([axis["velocity_jump_count"] for axis in result["axes"]], [90]*7)
        config["velocity_jump_threshold"] = [0.021]*7
        self.assertEqual([axis["velocity_jump_count"] for axis in metrics.analyze(data, config)["axes"]], [0]*7)

    def test_raw_tracking_p99_is_separate_from_recorded_reference_deviation(self):
        data = samples()
        error = np.tile(data["time"][:, None], (1, 7))
        data["feedback"], data["reference"] = error, 2*error
        result = metrics.analyze(data, configuration("recorded_commands_feedback"))
        for axis in result["axes"]:
            self.assertAlmostEqual(axis["tracking_error_p99"], 0.941)
            self.assertAlmostEqual(axis["reference_deviation_p99"], 1.882)
            self.assertIsNone(axis["tracking_error_reason"])

    def test_missing_feedback_is_null_and_reference_cannot_impersonate_it(self):
        data = samples()
        data["reference"] = np.ones((101, 7))
        result = metrics.analyze(data, configuration())
        for axis in result["axes"]:
            self.assertIsNone(axis["tracking_error_p99"])
            self.assertEqual(axis["tracking_error_reason"], "feedback_columns_absent")
            self.assertEqual(axis["reference_deviation_p99"], 1)
        data["feedback"] = data["reference"]
        with self.assertRaisesRegex(ValueError, "input_role"):
            metrics.analyze(data, configuration())

    def test_short_nonuniform_duplicate_and_nonfinite_time_rejected(self):
        with self.assertRaisesRegex(ValueError, "13 samples"):
            metrics.analyze(samples(12), configuration())
        for mutation in (lambda t: t.__setitem__(50, t[50]+1e-5),
                         lambda t: t.__setitem__(50, t[49]),
                         lambda t: t.__setitem__(50, np.nan)):
            data = samples()
            mutation(data["time"])
            with self.assertRaises(ValueError): metrics.analyze(data, configuration())

    def test_large_finite_jerk_uses_scaled_rms_but_derived_overflow_rejects(self):
        data = samples()
        data["command"] = np.tile((data["time"]**3*1e160)[:, None], (1, 7))
        result = metrics.analyze(data, configuration())
        np.testing.assert_allclose([axis["jerk_rms"] for axis in result["axes"]], [6e160]*7, rtol=1e-9)
        data["command"][:50] = 1e308
        data["command"][50:] = -1e308
        self.assertTrue(np.all(np.isfinite(data["command"])))
        with self.assertRaisesRegex(ValueError, "nonfinite filtered/derived"):
            metrics.analyze(data, configuration())

    def test_units_dimensions_and_filter_contract_rejected(self):
        config = configuration()
        config["units"][-1] = "deg"
        with self.assertRaisesRegex(ValueError, "units"):
            metrics.analyze(samples(), config)
        data = samples()
        data["command"] = np.zeros((101, 6))
        with self.assertRaisesRegex(ValueError, "Nx7"):
            metrics.analyze(data, configuration())
        config = configuration()
        config["filter"]["coefficients"] = [1, 1, 1, 1, 1]
        with self.assertRaisesRegex(ValueError, "binomial"):
            metrics.analyze(samples(), config)
        config = configuration()
        config["velocity_jump_threshold"][-1] = 0
        with self.assertRaisesRegex(ValueError, "positive"):
            metrics.analyze(samples(), config)

    def test_csv_optional_columns_must_be_whole_groups(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"partial.csv"
            path.write_text(",".join(["time_s"]+[f"cmd_{name}" for name in metrics.AXES]+["fb_j1"])+"\n")
            with self.assertRaisesRegex(ValueError, "complete seven-axis"):
                metrics.load_csv(path)

    def test_settling_needs_constant_command_and_sampled_half_second(self):
        config = configuration("recorded_commands_feedback", dt=0.1)
        config["settling"] = {"goal": [0]*7, "window_start_s": 0, "window_end_s": 2,
            "stable_s": 0.5, "joint_tolerance_deg": 0.5, "gripper_tolerance_m": 0.001,
            "command_epsilon": list(metrics.COMMAND_EPSILON)}
        data = samples(21, dt=0.1)
        data["feedback"] = np.zeros((21, 7))
        data["feedback"][:8, 0] = 1
        data["feedback"][3:5, 0] = 0  # A brief crossing alone cannot settle.
        data["feedback"][:10, 6] = 0.01
        result = metrics.analyze(data, config)
        self.assertAlmostEqual(result["axes"][0]["settling"]["arrival_time_s"], 0.8)
        self.assertAlmostEqual(result["axes"][0]["settling"]["confirmed_time_s"], 1.3)
        self.assertAlmostEqual(result["axes"][6]["settling"]["arrival_time_s"], 1.0)
        data["command"][7, 0] = 2e-9
        result = metrics.analyze(data, config)
        self.assertEqual(result["axes"][0]["settling"]["reason"], "command_not_constant_at_declared_goal_in_window")

    def test_settling_missing_feedback_window_and_no_settle_are_distinct(self):
        config = configuration("recorded_commands_feedback", dt=0.1)
        config["settling"] = {"goal": [0]*7, "window_start_s": 0, "window_end_s": 2,
            "stable_s": 0.5, "joint_tolerance_deg": 0.5, "gripper_tolerance_m": 0.001,
            "command_epsilon": list(metrics.COMMAND_EPSILON)}
        data = samples(21, dt=0.1)
        self.assertEqual(metrics.analyze(data, config)["axes"][0]["settling"]["reason"], "feedback_columns_absent")
        data["feedback"] = np.ones((21, 7))
        self.assertEqual(metrics.analyze(data, config)["axes"][0]["settling"]["status"], "not_settled_in_observation_window")
        config["settling"]["window_end_s"] = 3
        self.assertEqual(metrics.analyze(data, config)["axes"][0]["settling"]["reason"], "incomplete_declared_observation_window")

    def test_cli_locks_raw_hashes_reports_offline_and_never_overwrites(self):
        script = ROOT/"measure/smoothness_metrics.py"
        self.assertEqual(subprocess.run([sys.executable, str(script), "--help"], capture_output=True).returncode, 0)
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            csv_path, config_path, output = directory/"fixture.csv", directory/"locked.json", directory/"result.json"
            write_csv(csv_path, samples())
            config_path.write_text(json.dumps(configuration()))
            command = [sys.executable, str(script), "--csv", str(csv_path), "--config", str(config_path), "--output", str(output)]
            completed = subprocess.run(command, text=True, capture_output=True)
            self.assertEqual(completed.returncode, 0, completed.stderr)
            result = json.loads(output.read_text())
            self.assertEqual(result["config_sha256"], hashlib.sha256(config_path.read_bytes()).hexdigest())
            self.assertEqual(result["input_csv_sha256"], hashlib.sha256(csv_path.read_bytes()).hexdigest())
            self.assertEqual(result["provenance"], "offline_analysis")
            self.assertTrue(result["hardware_unverified"])
            self.assertIsNone(result["physical_f_exec_hz"])
            self.assertEqual(result["numpy_version"], np.__version__)
            self.assertEqual(result["python_version"], sys.version.split()[0])
            before = output.read_bytes()
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)
            self.assertEqual(output.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()

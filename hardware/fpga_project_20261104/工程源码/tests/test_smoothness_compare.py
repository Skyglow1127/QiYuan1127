"""Small recorded-array-shaped fixtures; no policy, images or hardware imports."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("compare_smoothness_offline", ROOT/"measure/compare_smoothness_offline.py")
compare = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(compare)


def fixture(directory):
    directory.mkdir()
    source = np.arange(41)/20
    actions = np.arange(41)[:, None]*np.asarray([1, 2, 3, -1, -2, -3, 0.001])
    time = np.arange(1, 201)/100
    reference = np.column_stack([np.interp(time, source, actions[:, axis]) for axis in range(7)])
    arrays = {"time": time, "recorded_source_time": source, "recorded_projected_actions": actions,
              "recorded_linear": reference, "c2": reference.copy(), "coarse": np.zeros_like(reference)}
    np.savez(directory/"replay_arrays.npz", **arrays)
    report = {"action_names": [f"joint_{i}.deg" for i in range(1, 7)]+["gripper.m"],
              "action_units": ["deg"]*6+["m"], "fps_dataset": 20,
              "scenario_output_hz": 100, "consume_steps": 30, "ticks": 200, "episode_frames": 41}
    (directory/"results.json").write_text(json.dumps(report))
    return arrays


class SmoothnessCompareTests(unittest.TestCase):
    def test_true_five_hz_uses_point_two_second_source_boundaries(self):
        source = np.arange(41)/20
        actions = np.repeat(np.arange(41)[:, None], 7, axis=1)
        time = np.asarray([0.01, 0.1999999, 0.2, 0.3999999, 0.4, 0.6, 1.0])
        zoh, schedule = compare.strict_zoh(time, source, actions)
        np.testing.assert_array_equal(zoh[:, 0], [0, 0, 4, 4, 8, 12, 20])
        self.assertEqual(schedule["nominal_update_hz"], 5)
        np.testing.assert_allclose(np.diff([x["nominal_boundary_s"] for x in schedule["nominal_schedule"]]), .2, rtol=0, atol=1e-15)
        self.assertEqual(zoh[4, 0], 8)  # A 30/20=1.5s consume baseline would still be0.

    def test_float_source_late_at_boundary_never_selects_a_future_frame(self):
        source = np.arange(21)/20
        source[4] = np.nextafter(0.2, np.inf)
        actions = np.repeat(np.arange(21)[:, None], 7, axis=1)
        time = np.asarray([.19, .2, .21, .39999, .4, .41])
        zoh, schedule = compare.strict_zoh(time, source, actions)
        np.testing.assert_array_equal(zoh[:, 0], [0, 3, 3, 3, 8, 8])
        self.assertEqual(schedule["conservative_prior_frame_boundary_count"], 1)
        self.assertAlmostEqual(schedule["maximum_selected_source_lag_s"], .05)
        for update in schedule["nominal_schedule"]:
            self.assertLessEqual(update["source_timestamp_s"], update["nominal_boundary_s"])
            self.assertGreaterEqual(update["first_evaluation_time_s"], update["nominal_boundary_s"])

    def test_saved_float32_twenty_hz_quantization_is_reported_separately(self):
        source = (np.arange(201)/20).astype(np.float32).astype(float)
        actions = np.repeat(np.arange(len(source))[:, None], 7, axis=1)
        _, schedule = compare.strict_zoh(np.arange(1, 1001)/100, source, actions)
        boundaries = np.arange(51)/5
        expected = int(np.sum(source[np.arange(51)*4] > boundaries))
        self.assertEqual(schedule["conservative_prior_frame_boundary_count"], expected)
        self.assertGreater(expected, 0)
        self.assertGreater(schedule["maximum_source_timestamp_quantization_s"], 0)
        self.assertLess(schedule["maximum_selected_source_lag_s"], .050001)

    def test_same_config_same_reference_null_feedback_and_readonly_inputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary);source = root/"input";arrays = fixture(source);output = root/"new"
            before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()}
            with contextlib.redirect_stdout(io.StringIO()):
                manifest = compare.run_comparison(source, output, make_plots=False)
            self.assertFalse(manifest["current_physical_deployment_baseline"])
            self.assertFalse(manifest["pl_output"])
            results = [json.loads((output/(name+"_metrics.json")).read_text()) for name in compare.LABELS]
            self.assertEqual(results[0]["config_sha256"], results[1]["config_sha256"])
            metrics = compare.metric_module()
            for label, result in zip(compare.LABELS, results):
                data = metrics.load_csv(output/(label+".csv"))
                np.testing.assert_array_equal(data["time"], arrays["time"])
                np.testing.assert_array_equal(data["reference"], arrays["recorded_linear"])
                self.assertIsNone(data["feedback"])
                self.assertIsNone(result["configuration"]["settling"])
                self.assertTrue(all(axis["tracking_error_p99"] is None for axis in result["axes"]))
                self.assertTrue(all(axis["settling"]["arrival_time_s"] is None for axis in result["axes"]))
            self.assertEqual(before, {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()})

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary);source = root/"input";fixture(source);output = root/"output";output.mkdir()
            marker = output/"marker";marker.write_text("original")
            with self.assertRaisesRegex(ValueError, "never overwritten"):
                compare.run_comparison(source, output, make_plots=False)
            self.assertEqual(marker.read_text(), "original")
            self.assertEqual(list(output.iterdir()), [marker])

    def test_nonuniform_or_different_reference_rejected_before_output_creation(self):
        for defect in ("grid", "reference"):
            with self.subTest(defect=defect), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary);source = root/"input";arrays = fixture(source);output = root/"output"
                if defect == "grid": arrays["time"][20] += 1e-4
                else: arrays["recorded_linear"][20, 0] += .1
                np.savez(source/"replay_arrays.npz", **arrays)
                with self.assertRaises(ValueError): compare.run_comparison(source, output, make_plots=False)
                self.assertFalse(output.exists())

    def test_cli_help_is_available_without_accessing_any_input(self):
        result = subprocess.run([sys.executable, str(ROOT/"measure/compare_smoothness_offline.py"), "--help"], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0)
        self.assertIn("--input-dir", result.stdout)
        self.assertIn("--output-dir", result.stdout)


if __name__ == "__main__":
    unittest.main()

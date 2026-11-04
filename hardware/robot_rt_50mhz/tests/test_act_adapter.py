"""Actual saved processor tests; entirely offline, no policy weights or robot I/O."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sw"))
from rt_adapter.act import ObservationBuilder, ActionPostprocessor, ACTQueueAdapter, load_processors


class ACTAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = json.loads((ROOT / "spec/act_contract.json").read_text())
        cls.builder = ObservationBuilder(cls.contract["checkpoint"])
        cls.post = ActionPostprocessor(cls.contract["checkpoint"])
        cls.state = np.asarray([0, 80, -60, 0, 40, -30, .03], dtype=np.float32)
        cls.images = {"global": np.zeros((480, 640, 3), dtype=np.uint8),
                      "wrist": np.full((480, 640, 3), 127, dtype=np.uint8)}

    def test_raw_builder_rgb_order_scale_and_no_resize(self):
        images = {role: value.copy() for role, value in self.images.items()}
        images["global"][0, 0] = [255, 128, 0]
        raw = self.builder.raw_observation(images, self.state)
        self.assertEqual(raw["observation.images.global"].shape, (3, 480, 640))
        torch.testing.assert_close(raw["observation.images.global"][:, 0, 0], torch.tensor([1, 128/255, 0]))
        self.assertEqual(raw["observation.state"].dtype, torch.float32)

    def test_saved_preprocessor_applied_exactly_once(self):
        _, official, _ = load_processors(self.contract["checkpoint"])
        independent = {"observation.state": torch.tensor(self.state.copy()),
            "observation.images.global": torch.zeros(3, 480, 640),
            "observation.images.wrist": torch.full((3, 480, 640), 127/255)}
        expected = official(independent)
        actual = self.builder.build(self.images, self.state)
        for key in independent:
            torch.testing.assert_close(actual[key], expected[key], atol=0, rtol=0)

    def test_bad_observations_rejected(self):
        with self.assertRaises(ValueError):
            self.builder.build({"global": self.images["global"]}, self.state)
        with self.assertRaises(ValueError):
            self.builder.build({**self.images, "global": self.images["global"].astype(float)}, self.state)
        with self.assertRaises(ValueError):
            self.builder.build(self.images, [0]*6)
        with self.assertRaises(ValueError):
            self.builder.build(self.images, [float("nan")]*7)

    def test_postprocessor_matches_saved_official_unnormalizer(self):
        _, _, official = load_processors(self.contract["checkpoint"])
        normalized = torch.arange(21, dtype=torch.float32).reshape(1, 3, 7)/20
        actual = self.post.denormalize(normalized)
        expected = official(normalized.clone())
        torch.testing.assert_close(actual, expected, atol=0, rtol=0)
        self.assertFalse(torch.equal(actual, self.post.denormalize(actual)))

    def test_bounds_reject_default_and_projection_has_flags(self):
        physical = torch.tensor([1000., 80, -60, 0, 40, -30, .03])
        with self.assertRaises(ValueError):
            self.post.project(physical)
        projected = self.post.project(physical, clip=True)
        self.assertEqual(projected.values[0], self.contract["deployment_bounds"][0][1])
        self.assertEqual(projected.clipped_mask.tolist(), [True, False, False, False, False, False, False])
        with self.assertRaises(ValueError):
            self.post.process(torch.full((1, 7), 1e5))
        self.assertTrue(torch.isfinite(self.post.denormalize(torch.full((1, 7), 1e5))).all())

    def test_q16_mixed_units_roundtrip_and_finite_checks(self):
        physical = torch.tensor(self.state, dtype=torch.float64)
        encoded = self.post.to_q16(physical)
        self.assertEqual(encoded.dtype, torch.int32)
        self.assertLessEqual(float(abs(encoded.double()/65536-physical).max()), .5/65536+1e-12)
        with self.assertRaises(ValueError):
            self.post.to_q16(torch.full((7,), float("inf")))

    def test_q16_nongrid_endpoints_stay_inside_physical_bounds(self):
        bounds = [[-.100001, .100001]] * 6 + [[.001, .1]]
        post = ActionPostprocessor(self.contract["checkpoint"], bounds=bounds)
        intervals = torch.tensor(bounds, dtype=torch.float64)
        for physical in (intervals[:, 0], intervals[:, 1], intervals.mean(dim=1)):
            decoded = post.to_q16(physical).double() / 65536
            self.assertTrue((decoded >= intervals[:, 0]).all())
            self.assertTrue((decoded <= intervals[:, 1]).all())
            self.assertLess(float(abs(decoded-physical).max()), 1/65536)
        # The reviewed 100 mm endpoint must never encode as 100.0061 mm.
        physical = torch.tensor(self.state, dtype=torch.float64)
        physical[-1] = .1
        encoded = self.post.to_q16(physical)
        self.assertEqual(int(encoded[-1]), 6553)
        self.assertLessEqual(float(encoded[-1].double()/65536), .1)

    def test_q16_unrepresentable_interval_and_int32_overflow_rejected(self):
        bounds = [[-1, 1]] * 6 + [[.000001, .000002]]
        post = ActionPostprocessor(self.contract["checkpoint"], bounds=bounds)
        with self.assertRaisesRegex(ValueError, "no representable"):
            post.to_q16(torch.tensor([0]*6+[.0000015], dtype=torch.float64))
        overflow = ActionPostprocessor(self.contract["checkpoint"], bounds=[[-1e6, 1e6]]*7)
        with self.assertRaisesRegex(ValueError, "overflow"):
            overflow.to_q16(torch.full((7,), 100000., dtype=torch.float64))

    def test_actual_contract_runtime_saved_distinction(self):
        self.assertEqual(self.contract["fps_dataset"], 20)
        self.assertEqual(self.contract["chunk_size"], 100)
        self.assertEqual(self.contract["saved_n_action_steps"], 100)
        self.assertEqual(self.contract["runtime_n_action_steps"], 30)
        self.assertEqual(self.contract["action_units"], ["deg"]*6+["m"])
        self.assertIsNone(self.contract["f_exec_hz"])

    def test_queue_consumes30_then_uses_next_observation_without_time_wait(self):
        class Policy:
            config = SimpleNamespace(chunk_size=100, temporal_ensemble_coeff=None)
            calls = 0
            def predict_action_chunk(self, observation):
                self.calls += 1
                result = torch.zeros((1, 100, 7))
                result[0, :, 0] = torch.arange(100)*.001+self.calls
                return result
            def reset(self):
                self.calls = 0
        policy = Policy()
        queue = ACTQueueAdapter(policy, self.builder, self.post, 30, enforce_bounds=False)
        sequence = [queue.select_action(self.images, self.state) for _ in range(31)]
        self.assertEqual(policy.calls, 2)
        self.assertGreater(float(sequence[29][0, 0]), float(sequence[0][0, 0]))
        self.assertGreater(float(sequence[30][0, 0]), float(sequence[29][0, 0]))
        queue.reset()
        self.assertEqual(len(queue.queue), 0)
        with self.assertRaises(ValueError):
            ACTQueueAdapter(policy, self.builder, self.post, 101)


if __name__ == "__main__":
    unittest.main()

"""Numerical/measurement gates on in-memory tensors; no policy weights or I/O."""
from collections import deque
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
with mock.patch.object(sys, "path", [str(ROOT / "measure"), str(ROOT / "sw"), *sys.path]):
    import act_runtime
    import check_action_alignment as alignment
    import benchmark_act as bench
    import rt_adapter.act  # The production queue under test; no torch/model loaded here.


class Policy:
    def __init__(self):
        self.config = types.SimpleNamespace(chunk_size=4, n_action_steps=2, temporal_ensemble_coeff=None)
        self.queue = deque()
    def reset(self):
        self.queue.clear()
    def predict_action_chunk(self, batch):
        return torch.arange(28, dtype=torch.float32).reshape(1, 4, 7) / 100 + batch["state"]
    def select_action(self, batch):
        if not self.queue:
            self.queue.extend(self.predict_action_chunk(batch)[:, :2].transpose(0, 1))
        return self.queue.popleft()


class Builder:
    def raw_observation(self, images, state):
        return {"state": torch.as_tensor(state)}
    build = raw_observation
    def reset(self):
        pass


class Post:
    bounds = [[-100, 100]] * 7
    @staticmethod
    def normalized_to_physical(value):
        return value * 2 + 0.01
    @staticmethod
    def to_q16(value):
        return torch.round(value.to(torch.float64) * 65536).to(torch.int32)
    def reset(self):
        pass


def runtime():
    policy = Policy()
    return dict(torch=torch, policy=policy, cfg=policy.config, builder=Builder(), postprocessor=Post(),
                official_pre=lambda batch: batch, official_post=Post.normalized_to_physical, device="cpu",
                records=[dict(images={}, state=np.zeros(7, dtype=np.float32))])


class ActValidationTests(unittest.TestCase):
    def test_independent_queue_refill_and_single_unnormalization_match(self):
        result, arrays = alignment.check(runtime(), chunks=3)
        self.assertTrue(result["passed"])
        self.assertEqual(result["steps"], 6)
        self.assertEqual(result["maximum_absolute_difference"], 0)
        self.assertEqual(arrays["official_physical"].shape, (6, 7))
        self.assertLessEqual(result["q16_maximum_roundtrip_error"], 1 / 65536)

    def test_double_unnormalization_cannot_pass_numeric_alignment(self):
        value = runtime()
        value["postprocessor"].normalized_to_physical = lambda item: Post.normalized_to_physical(Post.normalized_to_physical(item))
        result, _ = alignment.check(value, chunks=2)
        self.assertFalse(result["passed"])
        self.assertGreater(result["maximum_absolute_difference"], 1e-5)

    def test_rollout_guard_detects_executor_and_never_persists_args(self):
        listing = ("PID ELAPSED %CPU COMMAND COMMAND\n"
                   "421 12 0.1 python /venv/bin/python /project/camera_server.py --secret camera-only\n"
                   "422 20 2.3 python /venv/bin/python /project/rollout_act_piper.py --secret do-not-persist\n"
                   "423 10 1.2 python /venv/bin/python /project/launch_act.py --profile contact_repair\n")
        with mock.patch.object(act_runtime.subprocess, "run", return_value=types.SimpleNamespace(stdout=listing)):
            result = act_runtime.process_inventory()
            self.assertEqual(len(result["active_rollouts"]), 2)
            self.assertEqual(result["active_rollouts"][0]["script_basename"], "rollout_act_piper.py")
            self.assertNotIn("secret", repr(result))
            self.assertNotIn("do-not-persist", repr(result))
            with self.assertRaisesRegex(RuntimeError, "Resource blocked"):
                act_runtime.require_idle_host()

    def test_short_benchmark_is_never_reported_as_ten_minutes(self):
        import contextlib
        fake_telemetry = dict(rss_bytes=100, gpu_allocated_bytes=None)
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(bench, "require_idle_host"), \
                mock.patch.object(bench, "telemetry", return_value=fake_telemetry), \
                contextlib.redirect_stdout(io.StringIO()):
            result = bench.benchmark(runtime(), Path(temporary), duration=.02, warmup=1)
            self.assertTrue(result["complete"])
            self.assertGreaterEqual(result["actual_measured_duration_s"], .02)
            self.assertFalse(result["ten_minute_steady_run_completed"])
            self.assertTrue((Path(temporary) / "timings.csv").is_file())
            self.assertTrue((Path(temporary) / "telemetry.jsonl").is_file())

    def test_benchmark_resource_conflict_is_incomplete_with_raw_samples(self):
        import contextlib
        with tempfile.TemporaryDirectory() as temporary, \
                mock.patch.object(bench, "require_idle_host", side_effect=RuntimeError("Resource blocked")), \
                contextlib.redirect_stdout(io.StringIO()):
            result = bench.benchmark(runtime(), Path(temporary), duration=.02, warmup=1)
            self.assertFalse(result["complete"])
            self.assertFalse(result["ten_minute_steady_run_completed"])
            self.assertIn("Resource blocked", result["abort_reason"])
            self.assertGreater(result["forward"]["count"], 0)


if __name__ == "__main__":
    unittest.main()

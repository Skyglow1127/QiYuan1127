"""Regression tests for evaluation units and shared label eligibility."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.preaction.evaluate_sequence import boxes_from_motion, summarize


class SequenceEvaluationTests(unittest.TestCase):
    def test_zero_motion_preserves_box_for_every_step(self):
        box = np.array([[20., 30., 60., 90.]], np.float32)
        out = boxes_from_motion(box, np.array([[0, 0, 160]]), np.zeros((1, 10, 4), np.float32))
        np.testing.assert_allclose(out, np.repeat(box[:, None], 10, axis=1))

    def test_scaled_translation_uses_full_image_pixels(self):
        box = np.array([[20., 30., 60., 90.]], np.float32)
        motion = np.zeros((1, 10, 4), np.float32); motion[:, :, 0] = 1
        out = boxes_from_motion(box, np.array([[0, 0, 160]]), motion)
        np.testing.assert_allclose(out[0, 0], [36, 30, 76, 90])

    def test_masked_label_does_not_change_aggregate_or_denominator(self):
        metrics = {'center_error_px': np.array([[1., 3., 1e8], [0., 0., 0.]])}
        mask = np.array([[True, True, False], [False, False, False]])
        result = summarize(metrics, mask)
        self.assertEqual(result['labeled_steps'], 2); self.assertEqual(result['windows'], 1)
        self.assertEqual(result['center_error_px'], 2.)
        self.assertIsNone(summarize(metrics, np.zeros_like(mask))['center_error_px'])


if __name__ == '__main__':
    unittest.main()

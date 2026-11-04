"""Pure image/phase tests; no network, robot, files or global worker threads."""
import copy
from dataclasses import FrozenInstanceError
import unittest

import cv2
import numpy as np

from visual_gripper_gate import GateState, red_geometry, shape_gripper, validate_config


CONFIG = {
    "center_x_range": [.40, .60], "center_y_min": .70, "bottom_y_min": .95,
    "area_fraction_range": [.03, .20], "second_largest_ratio_max": .25,
    "opening_seen_m": .035, "closing_delta_min_m": .001, "near_frames": 2,
    "max_joint_error_deg": 3.,
    "grasp_joint_bounds_deg": [[-27, -14], [96, 113], [-71, -46], [-4, 13], [35, 67], [-34, -12]],
    "reset_opening_m": .055, "reset_j1_max_deg": -10.,
}
JOINTS = (-20., 101., -58., 2., 51., -25.)


def image(*, x=270, y=350, width=100, height=110, color=(255, 0, 0)):
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[y:y+height, x:x+width] = color
    return rgb


class VisualGripperTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(CONFIG)
        self.near = red_geometry(image(), self.config)
        self.obs = (*JOINTS, .06)
        self.prev = (*JOINTS, .06)

    def call(self, width=.02, phase=None, geometry=None, frame_id=1, joints=JOINTS,
             observed=None, previous=None):
        return shape_gripper((*joints, width), observed or self.obs, previous or self.prev,
                             phase or GateState(), self.near if geometry is None else geometry,
                             frame_id, self.config)

    def test_config_exact_keys_finite_ranges_and_independent_copy(self):
        normalized = validate_config(self.config)
        self.config["grasp_joint_bounds_deg"][0][0] = -90
        self.assertEqual(normalized["grasp_joint_bounds_deg"][0], (-27, -14))
        for change in ({"typo": 1}, {"center_y_min": float("nan")},
                       {"max_joint_error_deg": float("inf")}, {"near_frames": True},
                       {"near_frames": 1}, {"center_x_range": [0.7, .6]},
                       {"area_fraction_range": [-.1, .2]}, {"reset_opening_m": .02},
                       {"max_joint_error_deg": 3.01}, {"grasp_joint_bounds_deg": [[0, 1]]}):
            cfg = copy.deepcopy(CONFIG); cfg.update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_config(cfg)
        missing = copy.deepcopy(CONFIG); del missing["center_y_min"]
        with self.assertRaises(ValueError):
            validate_config(missing)

    def test_geometry_near_bbox_centroid_and_red_hue_wrap(self):
        self.assertTrue(self.near["near"])
        self.assertEqual(self.near["bbox_xywh"], (270, 350, 100, 110))
        self.assertEqual(self.near["centroid_xy"], (319.5, 404.5))
        self.assertEqual(self.near["area_px"], 11000)
        self.assertFalse(self.near["contact_or_alignment_proven"])
        wrapped = cv2.cvtColor(np.full((480, 640, 3), (175, 255, 255), dtype=np.uint8), cv2.COLOR_HSV2RGB)
        rgb = np.zeros_like(wrapped); rgb[350:460, 270:370] = wrapped[350:460, 270:370]
        self.assertTrue(red_geometry(rgb, self.config)["near"])

    def test_missing_invalid_and_no_red_images_block_closing(self):
        for rgb in (None, np.zeros((480, 640, 3), dtype=np.float32),
                    np.zeros((640, 480, 3), dtype=np.uint8),
                    np.zeros((480, 640, 4), dtype=np.uint8),
                    np.full((480, 640, 3), np.nan), np.zeros((480, 640, 3), dtype=np.uint8)):
            with self.subTest(shape=getattr(rgb, "shape", None)):
                geometry = red_geometry(rgb, self.config)
                self.assertFalse(geometry["near"])
                candidate, pending, diag = self.call(geometry=geometry)
                self.assertTrue(diag["held"]); self.assertEqual(candidate[6], .06)
                self.assertFalse(pending.closing_started)

    def test_far_right_too_small_and_ambiguous_red_do_not_authorize(self):
        images = [image(y=150), image(x=480), image(width=10, height=10)]
        ambiguous = image(); ambiguous[350:460, 50:100] = (255, 0, 0); images.append(ambiguous)
        for rgb in images:
            geo = red_geometry(rgb, self.config)
            self.assertFalse(geo["near"])
            _, pending, diag = self.call(geometry=geo)
            self.assertTrue(diag["held"]); self.assertFalse(pending.closing_started)
        self.assertTrue(red_geometry(ambiguous, self.config)["ambiguous"])
        boundary = image(); boundary[350:460, 50:75] = (255, 0, 0)
        boundary_geometry = red_geometry(boundary, self.config)
        self.assertEqual(boundary_geometry["second_to_largest_area_ratio"], .25)
        self.assertFalse(boundary_geometry["ambiguous"])
        self.assertTrue(boundary_geometry["near"])

    def test_initial_small_opening_and_regular_opening_are_unfiltered(self):
        initial = (0., 0., 0., 0., 0., 0., .0001)
        action = (1., 2., -3., 4., 5., 6., .0046)
        candidate, pending, diag = shape_gripper(action, initial, initial, GateState(), None, None, self.config)
        self.assertEqual(candidate, action); self.assertEqual(diag["decision"], "initialization_passthrough")
        self.assertFalse(pending.opening_seen)
        candidate, _, diag = self.call(width=.07, geometry={})
        self.assertEqual(candidate[6], .07); self.assertFalse(diag["held"])

    def test_small_cumulative_closures_are_held_at_max_previous_and_actual(self):
        phase = GateState(opening_seen=True)
        previous = self.prev
        for index in range(12):
            action = (*JOINTS, .06 - (index + 1) * .0002)
            candidate, phase, diag = shape_gripper(action, self.obs, previous, phase, {}, index, self.config)
            self.assertEqual(candidate[6], .06); self.assertTrue(diag["held"])
            previous = candidate
        higher_observed = (*JOINTS, .063)
        candidate, _, _ = self.call(width=.061, phase=phase, geometry={}, observed=higher_observed)
        self.assertEqual(candidate[6], .063)

    def test_opening_intent_latches_before_actual_reaches35mm_known_run2(self):
        # Recorded run2: slot18 raw~55mm, actual~20mm; next chunk raw19.67mm.
        # Existing physical cap can send only~23mm on the opening frame, but
        # that successful opening still commits the gate's opening intent.
        actual = (*JOINTS, .02)
        candidate, pending, diag = self.call(width=.055, observed=actual, previous=actual,
                                            geometry={}, frame_id=18)
        self.assertEqual(candidate[6], .055)
        self.assertTrue(diag["opening_intent"]); self.assertTrue(pending.opening_seen)
        last_accepted = (*JOINTS, .023)
        candidate, nextphase, diag = self.call(width=.01967, phase=pending, observed=actual,
                                              previous=last_accepted, geometry={}, frame_id=20)
        self.assertEqual(candidate[6], .023); self.assertTrue(diag["held"])
        self.assertFalse(nextphase.closing_started)
        candidate, pending, diag = self.call(width=.01967, observed=actual,
                                            previous=(*JOINTS, .04), geometry={})
        self.assertTrue(pending.opening_seen); self.assertTrue(diag["held"])
        self.assertEqual(candidate[6], .04)

    def test_requires_two_unique_near_frames_and_does_not_mutate_input(self):
        original = GateState()
        candidate, pending, diag = self.call(phase=original, frame_id=("generation", 10))
        self.assertTrue(diag["held"]); self.assertEqual(pending.near_streak, 1)
        self.assertEqual(original, GateState())
        with self.assertRaises(FrozenInstanceError):
            original.opening_seen = True
        _, duplicate, diag = self.call(phase=pending, frame_id=("generation", 10))
        self.assertTrue(diag["held"]); self.assertEqual(duplicate.near_streak, 1)
        candidate, latched, diag = self.call(phase=duplicate, frame_id=("generation", 11))
        self.assertEqual(candidate[6], .02); self.assertTrue(latched.closing_started)
        self.assertEqual(diag["decision"], "closure_latched")
        self.assertEqual(pending.near_streak, 1)

    def test_new_far_or_missing_frame_resets_near_streak(self):
        _, phase, _ = self.call(frame_id=10)
        far = red_geometry(image(y=150), self.config)
        _, phase, _ = self.call(phase=phase, frame_id=11, geometry=far)
        self.assertEqual(phase.near_streak, 0)
        _, phase, diag = self.call(phase=phase, frame_id=12)
        self.assertTrue(diag["held"]); self.assertEqual(phase.near_streak, 1)
        _, phase, _ = self.call(phase=phase, frame_id=None, geometry={})
        self.assertEqual(phase.near_streak, 0)
        _, phase, diag = self.call(phase=phase, frame_id=None)
        self.assertTrue(diag["held"]); self.assertEqual(phase.near_streak, 0)
        _, phase, diag = self.call(phase=GateState(True, False, 2, 12), frame_id=None)
        self.assertTrue(diag["held"]); self.assertEqual(phase.near_streak, 0)

    def test_actual_grasp_and_joint_agreement_and_delta_are_all_required(self):
        phase = GateState(True, False, 2, 8)
        for joints, obs in (((-16., *JOINTS[1:]), self.obs),
                            (JOINTS, (-10., *self.obs[1:]))):
            candidate, pending, diag = self.call(phase=phase, frame_id=9, joints=joints, observed=obs)
            self.assertTrue(diag["held"]); self.assertFalse(pending.closing_started)
            self.assertEqual(candidate[6], .06)
        _, pending, diag = self.call(width=.0595, phase=phase, frame_id=9)
        self.assertTrue(diag["held"]); self.assertFalse(pending.closing_started)
        candidate, pending, _ = self.call(phase=phase, frame_id=9, joints=(-17., *JOINTS[1:]))
        self.assertEqual(candidate[6], .02); self.assertTrue(pending.closing_started)

    def test_latch_allows_carry_and_release_then_reviewed_reopening_resets(self):
        phase = GateState(True, True, 2, 5)
        carry_joints = (30., 122., -91., 3., 66., -55.)
        obs = (*carry_joints, .025)
        candidate, pending, diag = self.call(width=.025, phase=phase, geometry={}, frame_id=6,
                                            joints=carry_joints, observed=obs, previous=obs)
        self.assertEqual(candidate, obs); self.assertTrue(pending.closing_started)
        candidate, pending, _ = self.call(width=.07, phase=pending, geometry={}, frame_id=7,
                                         joints=carry_joints, observed=obs, previous=obs)
        self.assertEqual(candidate[6], .07); self.assertTrue(pending.closing_started)
        retreat = (-20., 90., -58., 2., 51., -25., .06)
        candidate, pending, diag = self.call(width=.06, phase=pending, frame_id=8,
                                            joints=retreat[:6], observed=retreat, previous=retreat)
        self.assertTrue(diag["reset"]); self.assertFalse(pending.closing_started)
        self.assertEqual(pending.near_streak, 0); self.assertTrue(pending.opening_seen)
        _, nextphase, diag = self.call(phase=pending, frame_id=9)
        self.assertTrue(diag["held"]); self.assertFalse(nextphase.closing_started)

    def test_six_joints_always_preserved_and_inputs_unchanged(self):
        action = np.array((*JOINTS, .02), dtype=np.float64)
        obs = np.array(self.obs); prev = np.array(self.prev)
        old = (action.copy(), obs.copy(), prev.copy(), copy.deepcopy(self.near), copy.deepcopy(self.config))
        candidate, _, _ = shape_gripper(action, obs, prev, GateState(), self.near, 1, self.config)
        self.assertEqual(candidate[:6], tuple(action[:6]))
        for before, after in zip(old[:3], (action, obs, prev)):
            np.testing.assert_array_equal(before, after)
        self.assertEqual(old[3], self.near); self.assertEqual(old[4], self.config)

    def test_invalid_vectors_phase_and_geometry_cannot_latch(self):
        for vector in ((*JOINTS, float("nan")), (*JOINTS, float("inf")), [0] * 6,
                       [True] * 7, ["0"] * 7):
            with self.assertRaises(ValueError):
                shape_gripper(vector, self.obs, self.prev, GateState(), self.near, 1, self.config)
        for phase in (None, GateState(False, True), GateState(True, False, -1), GateState(True, False, 3)):
            with self.assertRaises(ValueError):
                self.call(phase=phase if phase is not None else {"invalid": 1})
        for identity in ([], {}, True, -1, (None, 1)):
            with self.assertRaises(ValueError):
                self.call(frame_id=identity)
        forged = dict(self.near); forged["center_y_fraction"] = float("nan")
        _, pending, diag = self.call(phase=GateState(True, False, 2, 1), geometry=forged, frame_id=2)
        self.assertTrue(diag["held"]); self.assertFalse(pending.closing_started)


if __name__ == "__main__":
    unittest.main(verbosity=2)

"""Offline tests for approach-only gripper shaping; no hardware connection."""

from copy import deepcopy
from dataclasses import FrozenInstanceError
import math
import unittest

from lerobot_robot_piper_executor.approach_gripper import (
    ApproachGripperState, shape_approach_gripper, validate_approach_gripper)
from lerobot_robot_piper_executor.guard import ACTION_KEYS, GuardError


CONFIG = {
    "opening_intent_m": .035,
    "min_close_delta_m": .001,
    "retry_open_m": .055,
    "retry_joint1_max_deg": -10.,
    "grasp_joint_bounds_deg": [[-27., -14.], [96., 113.], [-71., -46.],
                               [-4., 13.], [35., 67.], [-34., -12.]],
}
BOUNDS = ((-60., 40.), (0., 135.), (-120., 0.), (-45., 45.),
          (-60., 70.), (-80., 60.), (0., .1))
INSIDE = (-20., 103., -60., 4., 50., -25., .055)
FAR = (-20., 80., -45., 4., 50., -25., .055)


class ApproachGripperTests(unittest.TestCase):
    def shape(self, action, *, observed=FAR, previous=FAR,
              phase=ApproachGripperState(), config=CONFIG):
        return shape_approach_gripper(action, observed_state=observed,
            previous_target=previous, phase=phase, config=config)

    def test_valid_config_is_copied_and_state_is_frozen(self):
        config = deepcopy(CONFIG)
        result = validate_approach_gripper(config, bounds=BOUNDS)
        self.assertEqual(result["grasp_joint_bounds_deg"], tuple(map(tuple,
            CONFIG["grasp_joint_bounds_deg"])))
        result["opening_intent_m"] = .04
        self.assertEqual(config, CONFIG)
        with self.assertRaises(FrozenInstanceError):
            ApproachGripperState().opening_seen = True

    def test_startup_at_zero_width_does_not_block_initial_closed_proposal(self):
        state = (0.,) * 7
        action = (-1., 2., -2., 0., 0., 1., .006)
        target, phase, held = self.shape(action, observed=state, previous=state)
        self.assertEqual(target, action)
        self.assertEqual(phase, ApproachGripperState())
        self.assertFalse(held)

    def test_far_early_close_holds_previous_ack_gripper_only(self):
        action = tuple(float(i) for i in range(6)) + (.020,)
        target, phase, held = self.shape(action)
        self.assertEqual(target, action[:6] + (FAR[6],))
        self.assertEqual(phase, ApproachGripperState(True, False))
        self.assertTrue(held)

    def test_opening_can_be_seen_in_actual_or_increasing_model_intent(self):
        for actual, raw, expected in ((.035, .02, True), (.01, .035, True),
                                     (.0345, .035, False), (.034, .035, False)):
            with self.subTest(actual=actual, raw=raw):
                state = FAR[:6] + (actual,)
                _, phase, _ = self.shape(FAR[:6] + (raw,), observed=state)
                self.assertEqual(phase.opening_seen, expected)

    def test_opening_seen_persists_across_calls_without_mutating_old_phase(self):
        old = ApproachGripperState(True, False)
        target, phase, held = self.shape(FAR[:6] + (.02,),
            observed=FAR[:6] + (.01,), phase=old)
        self.assertTrue(held)
        self.assertEqual(target[6], FAR[6])
        self.assertTrue(phase.opening_seen)
        self.assertEqual(old, ApproachGripperState(True, False))

    def test_window_allows_significant_closing_and_marks_started(self):
        action = INSIDE[:6] + (.02,)
        target, phase, held = self.shape(action, observed=INSIDE,
            phase=ApproachGripperState(True, False))
        self.assertEqual(target, action)
        self.assertEqual(phase, ApproachGripperState(True, True))
        self.assertFalse(held)

    def test_every_joint_window_is_inclusive_and_checked_against_actual(self):
        action = INSIDE[:6] + (.02,)
        for index, (low, high) in enumerate(CONFIG["grasp_joint_bounds_deg"]):
            for edge in (low, high):
                actual = list(INSIDE)
                actual[index] = edge
                _, phase, held = self.shape(action, observed=actual)
                self.assertTrue(phase.closing_started)
                self.assertFalse(held)
            for outside in (math.nextafter(low, -math.inf),
                            math.nextafter(high, math.inf)):
                actual = list(INSIDE)
                actual[index] = outside
                target, phase, held = self.shape(action, observed=actual)
                self.assertTrue(held)
                self.assertFalse(phase.closing_started)
                self.assertEqual(target[:6], action[:6])

    def test_small_closing_is_held_outside_window_but_does_not_start_inside(self):
        for grip in (FAR[6] - .001, FAR[6] - .0005, FAR[6], .07):
            for actual in (FAR, INSIDE):
                with self.subTest(grip=grip, actual=actual):
                    action = actual[:6] + (grip,)
                    target, phase, held = self.shape(action, observed=actual)
                    expected_hold = actual == FAR and grip < FAR[6]
                    self.assertEqual(held, expected_hold)
                    self.assertEqual(target, action[:6] +
                                     (FAR[6] if expected_hold else grip,))
                    self.assertFalse(phase.closing_started)

    def test_continuous_submillimetre_and_subcan_closing_cannot_accumulate(self):
        for decrement in (.0009, .0000004):
            with self.subTest(decrement=decrement):
                previous = FAR
                phase = ApproachGripperState()
                for _ in range(40):
                    action = FAR[:6] + (previous[6] - decrement,)
                    target, phase, held = self.shape(action,
                        observed=previous, previous=previous, phase=phase)
                    self.assertTrue(held)
                    self.assertEqual(target, FAR)
                    self.assertEqual(phase, ApproachGripperState(True, False))
                    # Acknowledge the returned candidate for the next call.
                    previous = target

    def test_after_grasp_placement_and_return_closing_pass_through(self):
        phase = ApproachGripperState(True, True)
        for actual in ((30., 125., -90., 0., 68., -55., .06),
                       (0., 20., -10., 0., 0., 0., .04)):
            action = actual[:6] + (.02,)
            target, next_phase, held = self.shape(action, observed=actual,
                previous=actual, phase=phase)
            self.assertEqual(target, action)
            self.assertEqual(next_phase, phase)
            self.assertFalse(held)

    def test_retry_opening_resets_and_next_early_closing_is_held(self):
        action = FAR[:6] + (.055,)
        target, phase, held = self.shape(action,
            phase=ApproachGripperState(True, True))
        self.assertEqual(target, action)
        self.assertEqual(phase, ApproachGripperState(True, False))
        self.assertFalse(held)
        target, phase, held = self.shape(FAR[:6] + (.02,),
            previous=action, phase=phase)
        self.assertTrue(held)
        self.assertEqual(target[6], .055)

    def test_retry_reset_requires_all_three_conditions(self):
        phase = ApproachGripperState(True, True)
        cases = [(FAR, math.nextafter(.055, -math.inf)),
                 ((-9.,) + FAR[1:], .055), (INSIDE, .055)]
        for actual, grip in cases:
            with self.subTest(actual=actual, grip=grip):
                _, next_phase, _ = self.shape(actual[:6] + (grip,),
                    observed=actual, phase=phase)
                self.assertTrue(next_phase.closing_started)

    def test_mapping_order_input_and_caller_objects_are_unchanged(self):
        action = dict(reversed(tuple(zip(ACTION_KEYS, FAR[:6] + (.02,)))))
        observed, previous, config = list(FAR), list(FAR), deepcopy(CONFIG)
        before = deepcopy((action, observed, previous, config))
        target, _, held = self.shape(action, observed=observed,
            previous=previous, config=config)
        self.assertTrue(held)
        self.assertEqual(target[:6], FAR[:6])
        self.assertEqual((action, observed, previous, config), before)

    def test_phase_proposal_does_not_mutate_committed_state_after_failed_send(self):
        committed = ApproachGripperState(True, False)
        _, proposed, _ = self.shape(INSIDE[:6] + (.02,),
            observed=INSIDE, phase=committed)
        self.assertTrue(proposed.closing_started)
        self.assertFalse(committed.closing_started)
        # A failed caller send discards proposed; the old phase still holds.
        _, _, held = self.shape(FAR[:6] + (.02,), phase=committed)
        self.assertTrue(held)

    def test_config_rejects_missing_unknown_keys_wrong_shapes_and_nonfinite(self):
        bad = [None, [], {}, {**CONFIG, "extra": 1}]
        for key in CONFIG:
            config = deepcopy(CONFIG)
            del config[key]
            bad.append(config)
        for key in set(CONFIG) - {"grasp_joint_bounds_deg"}:
            for value in (True, False, "0.1", None, math.nan, math.inf, -math.inf,
                          10 ** 400):
                bad.append({**CONFIG, key: value})
        for value in ([], CONFIG["grasp_joint_bounds_deg"][:5], [[0, 1, 2]] * 6,
                      [[1, 1]] * 6, [[2, 1]] * 6, [[0, math.nan]] * 6,
                      [[0, True]] * 6):
            bad.append({**CONFIG, "grasp_joint_bounds_deg": value})
        for config in bad:
            with self.subTest(config=config), self.assertRaises(GuardError):
                validate_approach_gripper(config, bounds=BOUNDS)

    def test_config_rejects_unordered_thresholds_and_windows_outside_guard(self):
        changes = ({"opening_intent_m": -.001}, {"retry_open_m": .035},
                   {"retry_open_m": .101}, {"min_close_delta_m": 0},
                   {"min_close_delta_m": .055}, {"retry_joint1_max_deg": -61})
        for changeset in changes:
            with self.subTest(changes=changeset), self.assertRaises(GuardError):
                validate_approach_gripper({**CONFIG, **changeset}, bounds=BOUNDS)
        for index in range(6):
            config = deepcopy(CONFIG)
            config["grasp_joint_bounds_deg"][index][0] = BOUNDS[index][0] - 1
            with self.subTest(index=index), self.assertRaises(GuardError):
                validate_approach_gripper(config, bounds=BOUNDS)
        for bounds in (BOUNDS[:6], [[0, math.inf]] * 7, [[0, 0]] * 7):
            with self.subTest(bounds=bounds), self.assertRaises(GuardError):
                validate_approach_gripper(CONFIG, bounds=bounds)

    def test_shaping_rejects_invalid_vectors_phase_and_config(self):
        bad_vectors = [None, [], [0.] * 6, [0.] * 8, {"wrong": 1}]
        for index in range(7):
            for value in (math.nan, math.inf, -math.inf):
                item = list(FAR)
                item[index] = value
                bad_vectors.append(item)
        for bad in bad_vectors:
            for name in ("action", "observed_state", "previous_target"):
                kwargs = dict(action=FAR, observed_state=FAR, previous_target=FAR,
                              phase=ApproachGripperState(), config=CONFIG)
                kwargs[name] = bad
                with self.subTest(name=name, bad=bad), self.assertRaises(GuardError):
                    shape_approach_gripper(**kwargs)
        for phase in (None, {}, ApproachGripperState(1, False),
                      ApproachGripperState(False, "yes")):
            with self.subTest(phase=phase), self.assertRaises(GuardError):
                self.shape(FAR, phase=phase)
        with self.assertRaises(GuardError):
            self.shape(FAR, config={**CONFIG, "extra": 1})


if __name__ == "__main__":
    unittest.main()

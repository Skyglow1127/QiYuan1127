"""Synthetic projection and original-guard tests; no hardware connection."""

from dataclasses import replace
import math
import struct
import sys
import unittest
from unittest.mock import patch

from lerobot_robot_piper_executor.guard import (
    ACTION_KEYS, ActionGuard, GuardError, GuardLimits, Readiness)
from lerobot_robot_piper_executor.policy_bounds import (
    j5_upper_target_bound, project_j5_upper_prediction, saturate_j5_upper_prediction)
from lerobot_robot_piper_executor.sdk_codec import encode_target


def limits():
    return GuardLimits(bounds=((-60., 30.), (0., 120.), (-120., 0.),
                              (-45., 45.), (-60., 70.), (-80., 60.), (0., .1)),
        max_step=(6.,) * 6 + (.008,),
        max_target_speed=(45.,) * 6 + (.05,),
        max_tracking_error=(8.,) * 6 + (.015,),
        max_observation_age_s=.4, max_command_age_s=.4,
        max_command_gap_s=.75)


def ready():
    return Readiness(**{name: "SYNTHETIC OFFLINE TEST ONLY"
        for name in Readiness.__dataclass_fields__})


class PolicyBoundsTests(unittest.TestCase):
    def setUp(self):
        self.limits = limits()
        self.action = (-10., 30., -30., 0., 70., 0., .02)

    def project(self, j5=70., **kwargs):
        action = self.action[:4] + (j5,) + self.action[5:]
        return project_j5_upper_prediction(action, limits=self.limits,
            tolerance_deg=kwargs.get("tolerance_deg", 1.))

    def guard(self, j5=69.5):
        state = self.action[:4] + (j5,) + self.action[5:]
        guard = ActionGuard(self.limits)
        guard.arm(state, sampled_at=1., now=1., readiness=ready())
        return guard, state

    def test_exact_upper_bound_is_unchanged_and_not_counted(self):
        target, corrections = self.project()
        self.assertEqual(target, self.action)
        self.assertEqual(corrections, ())

    def test_small_upper_overshoot_is_projected_and_recorded_exactly(self):
        for raw in (math.nextafter(70., math.inf), 70.545265, 71.):
            with self.subTest(raw=raw):
                target, corrections = self.project(raw)
                self.assertEqual(target, self.action)
                self.assertEqual(corrections, ({"key": "joint_5.deg",
                    "original": raw, "projected": 70.},))

    def test_larger_overshoot_has_no_floating_point_grace(self):
        for raw in (math.nextafter(71., math.inf), 71.001, 72.):
            with self.subTest(raw=raw), self.assertRaisesRegex(GuardError, "joint_5.deg.*outside"):
                self.project(raw)

    def test_zero_tolerance_disables_projection(self):
        self.assertEqual(self.project(tolerance_deg=0.)[1], ())
        with self.assertRaises(GuardError):
            self.project(math.nextafter(70., math.inf), tolerance_deg=0.)

    def test_invalid_tolerances_are_rejected(self):
        for tolerance in (-.1, 1.000001, math.nan, math.inf, -math.inf,
                          True, False, "1", None):
            with self.subTest(tolerance=tolerance), self.assertRaises(GuardError):
                self.project(70.5, tolerance_deg=tolerance)

    def test_invalid_action_fields_dimensions_and_values_are_rejected(self):
        invalid = [self.action[:6], self.action + (1.,), {"wrong": 70.}]
        for value in (math.nan, math.inf, -math.inf):
            invalid.append(self.action[:4] + (value,) + self.action[5:])
            invalid.append((value,) + self.action[1:])
        for action in invalid:
            with self.subTest(action=action), self.assertRaises(GuardError):
                project_j5_upper_prediction(action, limits=self.limits, tolerance_deg=1.)

    def test_mapping_input_preserves_the_canonical_dimension_order(self):
        action = dict(reversed(tuple(zip(ACTION_KEYS,
            self.action[:4] + (70.5,) + self.action[5:], strict=True))))
        target, corrections = project_j5_upper_prediction(action,
            limits=self.limits, tolerance_deg=1.)
        self.assertEqual(target, self.action)
        self.assertEqual(corrections[0]["key"], "joint_5.deg")

    def test_other_axes_and_gripper_never_gain_projection_permission(self):
        for index, (low, high) in enumerate(self.limits.bounds):
            if index == 4:
                continue
            for value in (math.nextafter(low, -math.inf), math.nextafter(high, math.inf)):
                action = list(self.action)
                action[4] = 70.5
                action[index] = value
                with self.subTest(index=index, value=value), self.assertRaises(GuardError):
                    project_j5_upper_prediction(action, limits=self.limits, tolerance_deg=1.)

    def test_j5_lower_bound_remains_strict(self):
        with self.assertRaisesRegex(GuardError, "joint_5.deg.*outside"):
            self.project(math.nextafter(-60., -math.inf))
        self.assertEqual(self.project(-60.)[1], ())

    def test_actual_sdk_encoding_keeps_j5_at_seventy_thousand_millidegrees(self):
        target, _ = self.project(70.545265)
        with patch.object(sys, "path", ["/home/aup/piper SDK/pyAgxArm", *sys.path]):
            frames = encode_target(target, gripper_force_n=1.)
        frame = next(frame for frame in frames if frame.arbitration_id == 0x157)
        self.assertEqual(struct.unpack(">ii", frame.data), (70000, 0))
        self.assertEqual(70000 * .001, target[4])

    def test_projection_does_not_relax_original_step_or_tracking_checks(self):
        target, _ = self.project(70.545265)
        guard, state = self.guard(j5=63.)
        with self.assertRaisesRegex(GuardError, "target step too large"):
            guard.prepare(target, observed_state=state,
                sampled_at=1.2, proposal_at=1.2, now=1.2)
        guard, state = self.guard()
        state = state[:4] + (60.,) + state[5:]
        with self.assertRaisesRegex(GuardError, "too far from current feedback"):
            guard.prepare(target, observed_state=state,
                sampled_at=1.05, proposal_at=1.05, now=1.05)

    def test_actual_feedback_above_seventy_is_still_a_hard_error(self):
        target, _ = self.project(70.545265)
        guard, state = self.guard()
        state = state[:4] + (math.nextafter(70., math.inf),) + state[5:]
        with self.assertRaisesRegex(GuardError, "joint_5.deg.*outside.*no clipping performed"):
            guard.prepare(target, observed_state=state,
                sampled_at=1.05, proposal_at=1.05, now=1.05)


class J5TrainingSaturationTests(unittest.TestCase):
    def setUp(self):
        self.limits = limits()
        self.action = (-10., 30., -30., 0., 70., 0., .02)

    def test_linear_head_finite_upper_outputs_saturate_at_fixed_training_plateau(self):
        for raw in (math.nextafter(70., math.inf), 71.315, 72., 1000., 1e300):
            with self.subTest(raw=raw):
                action = self.action[:4] + (raw,) + self.action[5:]
                target, corrections = saturate_j5_upper_prediction(action, limits=self.limits)
                self.assertEqual(target, self.action)
                self.assertEqual(corrections, ({'key': 'joint_5.deg', 'original': raw, 'projected': 70.},))
        self.assertEqual(saturate_j5_upper_prediction(self.action, limits=self.limits), (self.action, ()))

    def test_projection_respects_an_even_lower_guard_upper_bound(self):
        bounds = list(self.limits.bounds)
        bounds[4] = (-60., 65.)
        conservative = replace(self.limits, bounds=tuple(bounds))
        action = self.action[:4] + (65.,) + self.action[5:]
        self.assertEqual(saturate_j5_upper_prediction(action, limits=conservative), (action, ()))
        for raw in (65.001, 70., 71.315):
            with self.subTest(raw=raw):
                prediction = self.action[:4] + (raw,) + self.action[5:]
                target, corrections = saturate_j5_upper_prediction(prediction, limits=conservative)
                self.assertEqual(target[4], 65.)
                self.assertEqual(corrections, ({'key': 'joint_5.deg', 'original': raw, 'projected': 65.},))
        self.assertEqual(conservative.bounds[4], (-60., 65.))

    def test_nonfinite_malformed_other_axes_and_j5_lower_bound_still_reject(self):
        for index in range(7):
            for value in (math.nan, math.inf, -math.inf):
                action = list(self.action)
                action[index] = value
                with self.subTest(index=index, value=value), self.assertRaises(GuardError):
                    saturate_j5_upper_prediction(action, limits=self.limits)
        for index, (low, high) in enumerate(self.limits.bounds):
            values = (math.nextafter(low, -math.inf),) if index == 4 else (
                math.nextafter(low, -math.inf), math.nextafter(high, math.inf))
            for value in values:
                action = list(self.action)
                action[4] = 71.315
                action[index] = value
                with self.subTest(index=index, value=value), self.assertRaises(GuardError):
                    saturate_j5_upper_prediction(action, limits=self.limits)
        for invalid in (self.action[:6], self.action + (1.,), {'wrong': 71.315}):
            with self.subTest(invalid=invalid), self.assertRaises(GuardError):
                saturate_j5_upper_prediction(invalid, limits=self.limits)

    def test_real_codec_and_original_feedback_guard_remain_strict(self):
        action = self.action[:4] + (71.315,) + self.action[5:]
        target, _ = saturate_j5_upper_prediction(action, limits=self.limits)
        with patch.object(sys, 'path', ['/home/aup/piper SDK/pyAgxArm', *sys.path]):
            frames = encode_target(target, gripper_force_n=1.)
        self.assertEqual(struct.unpack('>ii', frames[2].data), (70000, 0))
        for actual in (70.001, math.nan):
            guard = ActionGuard(self.limits)
            state = self.action[:4] + (69.,) + self.action[5:]
            guard.arm(state, sampled_at=1., now=1., readiness=ready())
            actual_state = state[:4] + (actual,) + state[5:]
            with self.subTest(actual=actual), self.assertRaises(GuardError):
                guard.prepare(target, observed_state=actual_state,
                    sampled_at=1.05, proposal_at=1.05, now=1.05)
            self.assertIs(guard.limits, self.limits)


class J5UpperTargetMarginTests(unittest.TestCase):
    def setUp(self):
        self.limits = limits()
        self.action = (-10., 30., -30., 0., 70., 0., .02)

    def test_two_degree_margin_projects_only_above_sixty_eight(self):
        for raw, expected in ((67.9, 67.9), (68., 68.),
                              (68.001, 68.), (70., 68.), (72.88, 68.)):
            with self.subTest(raw=raw):
                action = self.action[:4] + (raw,) + self.action[5:]
                target, corrections = saturate_j5_upper_prediction(
                    action, limits=self.limits, upper_margin_deg=2.)
                self.assertEqual(target, self.action[:4] + (expected,) + self.action[5:])
                self.assertEqual(corrections, () if raw <= 68. else (
                    {'key': 'joint_5.deg', 'original': raw, 'projected': 68.},))
        self.assertEqual(self.limits.bounds[4], (-60., 70.))

    def test_effective_bound_respects_training_and_guard_ceilings(self):
        self.assertEqual(j5_upper_target_bound(self.limits), 70.)
        self.assertEqual(j5_upper_target_bound(self.limits, 0), 70.)
        self.assertEqual(j5_upper_target_bound(self.limits, 2), 68.)
        self.assertEqual(j5_upper_target_bound(self.limits, 5.), 65.)
        for high, expected in ((65., 63.), (75., 68.)):
            bounds = list(self.limits.bounds)
            bounds[4] = (-60., high)
            configured = replace(self.limits, bounds=tuple(bounds))
            with self.subTest(guard_upper=high):
                self.assertEqual(j5_upper_target_bound(configured, 2.), expected)

    def test_invalid_margins_are_rejected_by_helper_and_transform(self):
        for margin in (-.001, 5.001, True, False, '2', None, [], {},
                       2+0j, math.nan, math.inf, -math.inf, 10**1000):
            with self.subTest(margin=margin):
                with self.assertRaises(GuardError):
                    j5_upper_target_bound(self.limits, margin)
                with self.assertRaises(GuardError):
                    saturate_j5_upper_prediction(
                        self.action, limits=self.limits, upper_margin_deg=margin)

    def test_margin_may_reach_but_not_cross_guard_lower_bound(self):
        bounds = list(self.limits.bounds)
        bounds[4] = (68., 70.)
        at_lower = replace(self.limits, bounds=tuple(bounds))
        self.assertEqual(j5_upper_target_bound(at_lower, 2.), 68.)
        target, corrections = saturate_j5_upper_prediction(
            self.action, limits=at_lower, upper_margin_deg=2.)
        self.assertEqual(target[4], 68.)
        self.assertEqual(corrections[0]['projected'], 68.)
        bounds[4] = (69., 70.)
        below_lower = replace(self.limits, bounds=tuple(bounds))
        with self.assertRaisesRegex(GuardError, 'below the guard lower bound'):
            j5_upper_target_bound(below_lower, 2.)
        with self.assertRaises(GuardError):
            saturate_j5_upper_prediction(
                self.action, limits=below_lower, upper_margin_deg=2.)
        bounds[4] = (71., 75.)
        above_training = replace(self.limits, bounds=tuple(bounds))
        with self.assertRaises(GuardError):
            j5_upper_target_bound(above_training, 0.)

    def test_margin_does_not_project_nonfinite_lower_or_other_axis_violations(self):
        invalid = [self.action[:4] + (math.nextafter(-60., -math.inf),) + self.action[5:]]
        for index in range(7):
            for value in (math.nan, math.inf, -math.inf):
                action = list(self.action)
                action[index] = value
                invalid.append(action)
            if index != 4:
                for value in (math.nextafter(self.limits.bounds[index][0], -math.inf),
                              math.nextafter(self.limits.bounds[index][1], math.inf)):
                    action = list(self.action)
                    action[index] = value
                    invalid.append(action)
        for action in invalid:
            with self.subTest(action=action), self.assertRaises(GuardError):
                saturate_j5_upper_prediction(
                    action, limits=self.limits, upper_margin_deg=2.)

    def test_codec_target_is_inward_but_observed_feedback_limit_is_unchanged(self):
        action = self.action[:4] + (72.88,) + self.action[5:]
        target, _ = saturate_j5_upper_prediction(
            action, limits=self.limits, upper_margin_deg=2.)
        with patch.object(sys, 'path', ['/home/aup/piper SDK/pyAgxArm', *sys.path]):
            frames = encode_target(target, gripper_force_n=1.)
        self.assertEqual(struct.unpack('>ii', frames[2].data), (68000, 0))
        guard = ActionGuard(self.limits)
        state = self.action[:4] + (69.,) + self.action[5:]
        guard.arm(state, sampled_at=1., now=1., readiness=ready())
        outside = state[:4] + (70.072,) + state[5:]
        with self.assertRaisesRegex(GuardError, 'joint_5.deg.*outside.*no clipping performed'):
            guard.prepare(target, observed_state=outside,
                sampled_at=1.05, proposal_at=1.05, now=1.05)
        self.assertEqual(guard.limits.bounds[4], (-60., 70.))


if __name__ == "__main__":
    unittest.main()

"""Synthetic, offline target-shaping tests; no robot or transport connection."""

from dataclasses import replace
import math
import unittest

from lerobot_robot_piper_executor.guard import (
    ACTION_KEYS, ActionGuard, GuardError, GuardLimits, Readiness)
from lerobot_robot_piper_executor.target_rate import shape_target_rate, target_rate_wait_s


def limits():
    return GuardLimits(bounds=((-100., 100.),) * 6 + ((0., .1),),
        max_step=(6.,) * 6 + (.008,),
        max_target_speed=(45.,) * 6 + (.05,),
        max_tracking_error=(8.,) * 6 + (.015,),
        max_observation_age_s=.4, max_command_age_s=.4,
        max_command_gap_s=.75)


def ready():
    return Readiness(**{name: "SYNTHETIC OFFLINE TEST"
        for name in Readiness.__dataclass_fields__})


class TargetRateTests(unittest.TestCase):
    def setUp(self):
        self.limits = limits()
        self.previous = (0.,) * 6 + (.0559,)
        self.caps = dict(zip(ACTION_KEYS, (2.,) * 6 + (.003,), strict=True))

    def shape(self, desired, **kwargs):
        args = dict(state=self.previous, previous_target=self.previous,
            elapsed_s=.05, limits=self.limits, relative_cap=self.caps)
        args.update(kwargs)
        return shape_target_rate(desired, **args)

    def guard(self, state=None, limits_override=None):
        guard = ActionGuard(limits_override or self.limits)
        guard.arm(self.previous if state is None else state,
            sampled_at=1., now=1., readiness=ready())
        return guard

    def test_gripper_open_and_close_are_limited_and_can_representable(self):
        for desired_width, direction in ((.1, 1), (0., -1)):
            with self.subTest(direction=direction):
                target = self.shape(self.previous[:6] + (desired_width,))
                difference = target[6] - self.previous[6]
                self.assertGreater(direction * difference, 0)
                self.assertLessEqual(abs(difference), .05 * .05 * .9 - 1e-6)
                self.assertEqual(target[6], round(target[6] * 1_000_000) * 1e-6)
                self.guard().prepare(target, observed_state=self.previous,
                    sampled_at=1.05, proposal_at=1.05, now=1.05)

    def test_one_micrometre_margin_avoids_rounding_at_speed_boundary(self):
        # At full speed .0559 -> .05825 in .047s is a floating-point boundary;
        # shaping keeps both the fractional margin and a full micrometre.
        target = self.shape(self.previous[:6] + (.05825,), elapsed_s=.047)
        self.assertLessEqual(target[6] - self.previous[6], .05 * .047 * .9 - 1e-6)
        encoded = round(target[6] * 1_000_000)
        self.assertEqual(target[6], encoded * 1e-6)
        self.guard().prepare(target, observed_state=self.previous,
            sampled_at=1.047, proposal_at=1.047, now=1.047)

    def test_joints_are_limited_in_both_directions(self):
        target = self.shape((40., -40., 0., 0., 0., 0., self.previous[6]))
        self.assertGreater(target[0], 0)
        self.assertLess(target[1], 0)
        for value in target[:2]:
            self.assertLessEqual(abs(value), 45. * .05 * .9 - .001)
            self.assertEqual(value, round(value * 1000) * .001)

    def test_feedback_cap_remains_an_independent_interval(self):
        caps = (1.,) * 6 + (.0005,)
        target = self.shape((50.,) * 6 + (.1,), relative_cap=caps)
        for value in target[:6]:
            self.assertLessEqual(value, 1.)
        self.assertLessEqual(abs(target[6] - self.previous[6]), .0005)

    def test_disjoint_intervals_fail_instead_of_breaking_feedback_cap(self):
        state = (10.,) + self.previous[1:]
        with self.assertRaisesRegex(GuardError, "joint_1.deg.*do not intersect"):
            self.shape(state, state=state)
        state = self.previous[:6] + (.08,)
        with self.assertRaisesRegex(GuardError, "gripper.m.*do not intersect"):
            self.shape(state, state=state)

    def test_zero_budget_holds_an_exact_decoded_grid_point(self):
        previous = (123 * .001,) * 6 + (59100 * 1e-6,)
        for elapsed in (0., 1e-9):
            with self.subTest(elapsed=elapsed):
                self.assertEqual(self.shape((30.,) * 6 + (.1,),
                    previous_target=previous, state=previous, elapsed_s=elapsed), previous)

    def test_zero_budget_cannot_round_an_unrepresentable_previous_target(self):
        previous = (0.,) * 6 + (.0591005,)
        with self.assertRaisesRegex(GuardError, "no CAN-representable target"):
            self.shape(previous, previous_target=previous, state=previous, elapsed_s=0.)

    def test_narrow_nonempty_interval_without_grid_point_fails(self):
        previous = (0.,) * 6 + (.0591005,)
        caps = (2.,) * 6 + (1e-8,)
        with self.assertRaisesRegex(GuardError, "no CAN-representable target"):
            self.shape(previous, state=previous, previous_target=previous, relative_cap=caps)

    def test_invalid_elapsed_and_fraction(self):
        for elapsed in (-1., math.nan, math.inf, "bad", None, True):
            with self.subTest(elapsed=elapsed), self.assertRaises(GuardError):
                self.shape(self.previous, elapsed_s=elapsed)
        for fraction in (0., -.1, .951, 1., math.nan, math.inf, "bad", None, True):
            with self.subTest(fraction=fraction), self.assertRaises(GuardError):
                self.shape(self.previous, speed_fraction=fraction)
        self.shape(self.previous, speed_fraction=.95)
        with self.assertRaisesRegex(GuardError, "non-finite target-rate budget"):
            self.shape(self.previous, elapsed_s=1e308)

    def test_invalid_vectors_and_caps(self):
        for name, value in (("desired", self.previous[:6]),
                            ("state", (math.nan,) + self.previous[1:]),
                            ("previous_target", {"wrong": 1.})):
            with self.subTest(name=name), self.assertRaises(GuardError):
                if name == "desired":
                    self.shape(value)
                else:
                    self.shape(self.previous, **{name: value})
        for cap in (None, 0., -1., math.inf, True, {"wrong": 1.},
                    (2.,) * 6, (2.,) * 6 + (0.,)):
            with self.subTest(cap=cap), self.assertRaises(GuardError):
                self.shape(self.previous, relative_cap=cap)

    def test_scalar_and_mapping_inputs(self):
        previous = dict(zip(ACTION_KEYS, self.previous, strict=True))
        self.assertEqual(self.shape(previous, state=previous,
            previous_target=previous, relative_cap=.003), self.previous)

    def test_step_and_tracking_still_reach_the_original_guard(self):
        state = (8.,) + self.previous[1:]
        target = self.shape(state, state=state, elapsed_s=.2)
        self.assertEqual(target[0], 8.)
        with self.assertRaisesRegex(GuardError, "target step too large"):
            self.guard().prepare(target, observed_state=state,
                sampled_at=1.2, proposal_at=1.2, now=1.2)
        target = self.shape((10.,) + self.previous[1:], elapsed_s=.3,
            relative_cap=(20.,) * 6 + (.003,))
        tracking_limits = replace(self.limits, max_step=(20.,) * 6 + (.008,))
        with self.assertRaisesRegex(GuardError, "too far from current feedback"):
            self.guard(limits_override=tracking_limits).prepare(target,
                observed_state=self.previous, sampled_at=1.3, proposal_at=1.3, now=1.3)

    def test_raw_absolute_bounds_are_rejected_before_shaping(self):
        guard = self.guard()
        raw = (101.,) + self.previous[1:]
        with self.assertRaisesRegex(GuardError, "outside.*no clipping performed"):
            # This is the required order in the adapter. A rate projection must
            # never hide an out-of-bounds raw policy prediction.
            desired = guard._bounded(raw)
            self.shape(desired)

    def test_guard_also_rejects_shaped_output_outside_absolute_bounds(self):
        previous = (99.,) + self.previous[1:]
        target = self.shape((101.,) + previous[1:], state=previous,
            previous_target=previous, elapsed_s=.2, relative_cap=(5.,) * 6 + (.003,))
        self.assertEqual(target[0], 101.)
        with self.assertRaisesRegex(GuardError, "outside.*no clipping performed"):
            self.guard(state=previous).prepare(target, observed_state=previous,
                sampled_at=1.2, proposal_at=1.2, now=1.2)

    def test_continuous_quantized_targets_pass_host_and_receiver_guards(self):
        host, receiver = self.guard(), self.guard()
        previous = self.previous
        host_commit = 1.
        for index in range(120):
            created = 1.05 + index * .05
            desired = ((50., -50., 20., -20., 10., -10., .1)
                if index % 2 else (-50., 50., -20., 20., -10., 10., 0.))
            target = self.shape(desired, state=previous, previous_target=previous,
                elapsed_s=created - host_commit)
            command = host.prepare(target, observed_state=previous,
                sampled_at=created, proposal_at=created, now=created)
            # Causal ordering: remote acceptance precedes the host's successful
            # RPC completion. Vary its latency while preserving that ordering.
            received = created + (.0004 if index % 3 else .001)
            remote_command = receiver.prepare(target, observed_state=previous,
                sampled_at=received, proposal_at=received, now=received)
            receiver.commit(remote_command, now=received + .0001)
            host_commit = created + (.002 if index % 5 else .004)
            host.commit(command, now=host_commit)
            self.assertEqual(host.last_target, receiver.last_target)
            self.assertEqual(target, tuple(round(value * scale) * quantum
                for value, scale, quantum in zip(target,
                    (1000,) * 6 + (1_000_000,), (.001,) * 6 + (1e-6,), strict=True)))
            previous = target


class TargetRateWaitTests(unittest.TestCase):
    def setUp(self):
        self.limits = limits()
        self.state = (0.,) * 6 + (.0586,)
        self.previous = (0.,) * 6 + (.063817,)
        self.caps = (2.,) * 6 + (.003,)

    def wait(self, **kwargs):
        args = dict(state=self.state, previous_target=self.previous,
            elapsed_s=.047, limits=self.limits, relative_cap=self.caps)
        args.update(kwargs)
        return target_rate_wait_s(**args)

    def test_reported_gripper_conflict_needs_only_a_few_milliseconds(self):
        desired = (0.,) * 6 + (.062490586,)
        with self.assertRaisesRegex(GuardError, "do not intersect"):
            shape_target_rate(desired, state=self.state, previous_target=self.previous,
                elapsed_s=.047, limits=self.limits, relative_cap=self.caps)
        wait = self.wait()
        self.assertAlmostEqual(wait, .002311111111111, places=12)
        elapsed = .047 + wait
        target = shape_target_rate(desired, state=self.state,
            previous_target=self.previous, elapsed_s=elapsed,
            limits=self.limits, relative_cap=self.caps)
        self.assertLessEqual(abs(target[6] - self.state[6]), .003)
        guard = ActionGuard(self.limits)
        guard.arm(self.previous, sampled_at=1., now=1., readiness=ready())
        command = guard.prepare(target, observed_state=self.state,
            sampled_at=1.047, proposal_at=1.047, now=1. + elapsed)
        guard.commit(command, now=1. + elapsed)

    def test_enough_elapsed_or_representable_hold_needs_no_wait(self):
        self.assertEqual(self.wait(elapsed_s=.05), 0.)
        self.assertEqual(self.wait(state=self.previous, elapsed_s=0.), 0.)

    def test_non_grid_previous_point_can_wait_until_it_can_be_quantized(self):
        previous = self.state[:6] + (.0586005,)
        wait = self.wait(previous_target=previous, elapsed_s=0.)
        self.assertGreater(wait, 0.)
        target = shape_target_rate(previous, state=self.state,
            previous_target=previous, elapsed_s=wait,
            limits=self.limits, relative_cap=self.caps)
        self.assertEqual(target[6], round(target[6] * 1_000_000) * 1e-6)

    def test_previous_at_feedback_cap_edge_reserves_an_interior_grid_point(self):
        state = (0.,) * 6 + (.05,)
        previous = (2.,) + state[1:]
        wait = self.wait(state=state, previous_target=previous, elapsed_s=0.)
        self.assertGreaterEqual(wait, 2 * .001 / (45. * .9))

    def test_large_gap_is_returned_for_the_caller_to_reject(self):
        previous = (90.,) + self.previous[1:]
        self.assertGreater(self.wait(previous_target=previous), .05)

    def test_maximum_wait_across_all_seven_axes(self):
        waits = []
        for axis in range(7):
            quantum = .001 if axis < 6 else 1e-6
            speed = self.limits.max_target_speed[axis]
            state = [0.] * 6 + [.05]
            previous = state.copy()
            # Different required durations, with the maximum on a non-first
            # joint, show that neither the gripper nor J1 alone controls it.
            required = (.08, .09, .1, .13, .11, .12, .07)[axis]
            previous[axis] += self.caps[axis] + speed * .9 * required
            wait = self.wait(state=state, previous_target=previous)
            self.assertGreaterEqual(wait + .047, required + 2 * quantum / (speed * .9))
            waits.append(wait)
        state = (0.,) * 6 + (.05,)
        previous = tuple(current + cap + speed * .9 * required
            for current, cap, speed, required in zip(state, self.caps,
                self.limits.max_target_speed, (.08, .09, .1, .13, .11, .12, .07), strict=True))
        self.assertAlmostEqual(self.wait(state=state, previous_target=previous), max(waits))

    def test_feedback_cap_without_any_grid_point_cannot_be_fixed_by_waiting(self):
        state = self.state[:6] + (.0586005,)
        with self.assertRaisesRegex(GuardError, "no CAN-representable target"):
            self.wait(state=state, relative_cap=(2.,) * 6 + (1e-8,))

    def test_invalid_inputs_keep_the_same_strict_validation(self):
        for name, values in (
                ("elapsed_s", (-1., math.nan, math.inf, None, True)),
                ("speed_fraction", (0., .951, 1., math.nan, True)),
                ("relative_cap", (None, 0., (2.,) * 6)),
                ("state", ((0.,) * 6, (math.nan,) * 7)),
                ("previous_target", ((0.,) * 8, {"wrong": 1.}))):
            for value in values:
                with self.subTest(name=name, value=value), self.assertRaises(GuardError):
                    self.wait(**{name: value})
        with self.assertRaisesRegex(GuardError, "non-finite target-rate budget"):
            self.wait(elapsed_s=1e308)


class TargetRateBoundTests(unittest.TestCase):
    def setUp(self):
        hard = list(limits().bounds)
        hard[4] = (-60., 70.)
        self.limits = replace(limits(), bounds=tuple(hard))
        self.bounds = list(self.limits.bounds)
        self.bounds[4] = (-60., 68.)
        self.previous = (0., 0., 0., 0., 70., 0., .05)
        self.desired = self.previous[:4] + (68.,) + self.previous[5:]
        self.args = dict(state=self.previous, previous_target=self.previous,
            elapsed_s=.047, limits=self.limits, relative_cap=(2.,) * 6 + (.003,),
            target_bounds=self.bounds)

    def shape(self, desired=None, **kwargs):
        args = {**self.args, **kwargs}
        return shape_target_rate(self.desired if desired is None else desired, **args)

    def wait(self, **kwargs):
        return target_rate_wait_s(**{**self.args, **kwargs})

    def test_desired_clamp_alone_is_insufficient_but_final_bound_cannot_be_undone(self):
        # A raw saturation to 68 is pushed up again by the previous-target
        # rate interval unless the final intersection also contains the bound.
        legacy = self.shape(target_bounds=None)
        self.assertGreater(legacy[4], 68.)
        with self.assertRaisesRegex(GuardError, 'do not intersect'):
            self.shape()
        wait = self.wait()
        self.assertGreater(wait, 0.)
        self.assertLess(wait, .05)
        elapsed = .047 + wait
        target = self.shape(elapsed_s=elapsed)
        self.assertEqual(target[4], 68.)
        self.assertLessEqual(abs(target[4] - self.previous[4]), 45. * elapsed * .9 - .001)
        self.assertEqual(self.bounds[4], (-60., 68.))
        guard = ActionGuard(self.limits)
        guard.arm(self.previous, sampled_at=1., now=1., readiness=ready())
        command = guard.prepare(target, observed_state=self.previous,
            sampled_at=1.047, proposal_at=1.047, now=1. + elapsed)
        self.assertEqual(command.expires_at, 1.447)
        self.assertEqual(guard.limits.bounds[4], (-60., 70.))

    def test_can_rounding_stays_inside_fractional_soft_upper_bound(self):
        bounds = list(self.bounds)
        bounds[4] = (-60., 68.0006)
        previous = self.previous[:4] + (68.001,) + self.previous[5:]
        desired = self.desired[:4] + (68.0006,) + self.desired[5:]
        target = self.shape(desired, state=previous, previous_target=previous,
            elapsed_s=.05, target_bounds=bounds)
        self.assertEqual(target[4], 68.)
        self.assertLessEqual(target[4], bounds[4][1])
        self.assertEqual(target[4], round(target[4] * 1000) * .001)

    def test_exact_grid_point_target_interval_can_be_reached_from_above(self):
        bounds = list(self.bounds)
        bounds[4] = (68., 68.)
        wait = self.wait(target_bounds=bounds)
        self.assertEqual(self.shape(elapsed_s=.047 + wait, target_bounds=bounds)[4], 68.)
        self.assertEqual(self.wait(elapsed_s=.05, target_bounds=bounds), 0.)

    def test_feedback_cap_and_soft_bounds_without_intersection_cannot_be_fixed_by_time(self):
        state = self.previous[:4] + (69.,) + self.previous[5:]
        caps = (2.,) * 4 + (.5, 2., .003)
        with self.assertRaisesRegex(GuardError, 'feedback cap and target bounds do not intersect'):
            self.wait(state=state, relative_cap=caps)
        with self.assertRaisesRegex(GuardError, 'do not intersect'):
            self.shape(state=state, relative_cap=caps, elapsed_s=.3)

    def test_bound_interval_with_no_can_point_cannot_be_fixed_by_time(self):
        bounds = list(self.bounds)
        bounds[4] = (68.0001, 68.0002)
        for operation in (self.shape, self.wait):
            with self.subTest(operation=operation.__name__), self.assertRaisesRegex(GuardError, 'no CAN-representable target'):
                operation(target_bounds=bounds, elapsed_s=.3)

    def test_invalid_target_bounds_are_rejected_by_both_helpers(self):
        invalid = [True, 68., 'bounds', {}, self.bounds[:6], self.bounds + [(0., 1.)]]
        for pair in ((68.,), (-60., 68., 69.), (-60., math.nan),
                     (-60., math.inf), (68., 67.), (-61., 68.), (-60., 70.001),
                     (False, 68.), (-60., '68')):
            bounds = list(self.bounds)
            bounds[4] = pair
            invalid.append(bounds)
        for bounds in invalid:
            for operation in (self.shape, self.wait):
                with self.subTest(bounds=bounds, operation=operation.__name__), self.assertRaises(GuardError):
                    operation(target_bounds=bounds)

    def test_none_preserves_legacy_results_exactly(self):
        args = {key: value for key, value in self.args.items() if key != 'target_bounds'}
        self.assertEqual(self.shape(target_bounds=None), shape_target_rate(self.desired, **args))
        self.assertEqual(self.wait(target_bounds=None), target_rate_wait_s(**args))


if __name__ == "__main__":
    unittest.main()

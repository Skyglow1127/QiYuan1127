"""Offline quantized joint acceleration shaping; no hardware or transport."""

import math
import unittest

from lerobot_robot_piper_executor.guard import GuardError
from lerobot_robot_piper_executor.joint_acceleration import (
    limit_joint_acceleration, validate_joint_acceleration,
)
from lerobot_robot_piper_executor.target_rate import shape_target_rate
from test_target_rate import limits


class JointAccelerationTests(unittest.TestCase):
    def setUp(self):
        self.limits = limits()
        self.previous = (0.,) * 6 + (50000 * 1e-6,)
        self.caps = (2.,) * 6 + (.003,)
        self.bounds = self.limits.bounds

    def shape(self, target, **overrides):
        args = dict(state=self.previous, previous_target=self.previous,
            previous_velocity=(0.,) * 6, previous_interval_s=.05,
            elapsed_s=.05, rate_elapsed_s=.05, limits=self.limits,
            relative_cap=self.caps, speed_fraction=.9, target_bounds=self.bounds,
            max_acceleration_deg_s2=160.)
        args.update(overrides)
        return limit_joint_acceleration(target, **args)

    def assert_grid(self, target):
        self.assertEqual(target, tuple(round(value * scale) * quantum
            for value, scale, quantum in zip(target,
                (1000,) * 6 + (1_000_000,), (.001,) * 6 + (1e-6,), strict=True)))

    def test_limit_is_symmetric_and_gripper_remains_original_projection(self):
        target = (20., -20., 20., -20., 20., -20., .1)
        actual, relaxed = self.shape(target)
        baseline = shape_target_rate(target, state=self.previous,
            previous_target=self.previous, elapsed_s=.05, limits=self.limits,
            relative_cap=self.caps, speed_fraction=.9, target_bounds=self.bounds)
        self.assertEqual(relaxed, ())
        self.assertGreater(actual[0], 0.)
        self.assertLess(actual[1], 0.)
        self.assertLessEqual(max(abs(v) for v in actual[:6]), .4)
        self.assertEqual(actual[6], baseline[6])
        self.assert_grid(actual)

    def test_nonuniform_intervals_use_average_velocity_midpoint_distance(self):
        result, relaxed = self.shape((10.,) * 6 + (.05,),
            previous_velocity=(5.,) * 6, previous_interval_s=.03,
            elapsed_s=.07, rate_elapsed_s=.04, max_acceleration_deg_s2=100.)
        self.assertEqual(relaxed, ())
        # dv <= 100*(.03+.07)/2 = 5; displacement <= .07*(5+5).
        self.assertLessEqual(result[0], .07 * (5. + 100. * (.03+.07)/2.))
        self.assertGreaterEqual(result[0], .699)
        self.assert_grid(result)

    def test_reversal_brakes_before_changing_direction(self):
        result, relaxed = self.shape((-10.,) * 6 + (.05,),
            previous_velocity=(10.,) * 6, max_acceleration_deg_s2=100.)
        self.assertEqual(relaxed, ())
        # Prior positive 10 deg/s can fall only to positive 5 deg/s this step.
        self.assertGreaterEqual(result[0], .25)
        self.assertLessEqual(result[0], .251)

    def test_rate_interval_uses_commit_elapsed_not_shaping_elapsed(self):
        result, relaxed = self.shape((10.,) * 6 + (.1,), elapsed_s=.2,
            previous_interval_s=.05, rate_elapsed_s=.005)
        self.assertEqual(relaxed, ())
        self.assertLessEqual(result[0], 45. * .005 * .9 - .001)
        self.assertLessEqual(result[6] - .05, .05 * .005 * .9 - 1e-6)

    def test_decoded_float_endpoint_retains_strict_velocity_budget(self):
        previous = (99.,) + self.previous[1:]
        result, relaxed = self.shape((100.,) + self.previous[1:],
            state=previous, previous_target=previous)
        self.assertEqual(relaxed, ())
        # 99.4 decodes above its ideal displacement. An inward grid point is
        # required rather than widening the acceleration interval by epsilon.
        self.assertLessEqual(abs((result[0]-previous[0])/.05), 160.*.05)
        self.assert_grid(result)

    def test_missing_history_preserves_old_hard_projection_exactly(self):
        target = (10., -10., 3., -3., 8., -8., .1)
        expected = shape_target_rate(target, state=self.previous,
            previous_target=self.previous, elapsed_s=.05, limits=self.limits,
            relative_cap=self.caps, speed_fraction=.9, target_bounds=self.bounds)
        for kwargs in ({"previous_velocity": None}, {"previous_interval_s": None},
                {"previous_velocity": None, "previous_interval_s": None, "elapsed_s": 0.}):
            with self.subTest(kwargs=kwargs):
                self.assertEqual(self.shape(target, **kwargs), (expected, ()))

    def test_feedback_conflict_relaxes_only_one_soft_axis(self):
        state = (3.,) + self.previous[1:]
        result, relaxed = self.shape((3.,) + self.previous[1:], state=state)
        self.assertEqual(relaxed, ("joint_1.deg",))
        # Hard feedback requires >= 1, while acceleration permits <= .4.
        self.assertGreaterEqual(result[0], 1.)
        self.assertLessEqual(abs(result[0] - state[0]), 2.)
        self.assertEqual(result[1:], self.previous[1:])
        self.assert_grid(result)

    def test_soft_interval_with_no_can_point_relaxes_without_widening(self):
        result, relaxed = self.shape(self.previous,
            previous_velocity=(.01,) + (0.,) * 5,
            max_acceleration_deg_s2=1e-6)
        # J1 interval is centred on .0005 and much narrower than one quantum.
        self.assertEqual(relaxed, ("joint_1.deg",))
        self.assertEqual(result, self.previous)

    def test_optional_target_bounds_are_still_hard(self):
        bounds = ((.3, .35),) + self.bounds[1:]
        result, relaxed = self.shape((10.,) + self.previous[1:], target_bounds=bounds)
        self.assertEqual(relaxed, ())
        self.assertGreaterEqual(result[0], .3)
        self.assertLessEqual(result[0], .35)
        # Here a positive prior velocity makes acceleration demand >= .75.
        result, relaxed = self.shape((10.,) + self.previous[1:], target_bounds=bounds,
            previous_velocity=(20.,) + (0.,) * 5, max_acceleration_deg_s2=100.)
        self.assertEqual(relaxed, ("joint_1.deg",))
        self.assertGreaterEqual(result[0], .3)
        self.assertLessEqual(result[0], .35)

    def test_original_hard_interval_and_unrepresentable_grid_still_fail(self):
        with self.assertRaisesRegex(GuardError, "do not intersect"):
            self.shape(self.previous, state=(10.,) + self.previous[1:])
        previous = (0.,) * 6 + (.0500005,)
        with self.assertRaisesRegex(GuardError, "no CAN-representable"):
            self.shape(previous, state=previous, previous_target=previous,
                relative_cap=(2.,) * 6 + (1e-8,))

    def test_gripper_hard_conflict_does_not_become_soft_relaxation(self):
        with self.assertRaisesRegex(GuardError, "gripper.m.*do not intersect"):
            self.shape(self.previous, state=self.previous[:6] + (.08,))

    def test_finite_acceleration_and_history_validation(self):
        self.assertEqual(validate_joint_acceleration(1000), 1000.)
        self.assertEqual(validate_joint_acceleration(.001), .001)
        for value in (0., -1., 1000.01, math.nan, math.inf, -math.inf, True,
                False, "160", None):
            with self.subTest(value=value), self.assertRaises(GuardError):
                validate_joint_acceleration(value)
        for name in ("elapsed_s", "previous_interval_s"):
            for value in (-.1, math.nan, math.inf, True, "0.05"):
                with self.subTest(name=name, value=value), self.assertRaises(GuardError):
                    self.shape(self.previous, **{name: value})
        for name in ("elapsed_s", "previous_interval_s"):
            with self.subTest(name=name), self.assertRaises(GuardError):
                self.shape(self.previous, **{name: 0.})
        for value in ((0.,) * 5, (0.,) * 7, (math.nan,) + (0.,) * 5,
                (True,) + (0.,) * 5, {"joint_1.deg": 0.}, "000000"):
            with self.subTest(value=value), self.assertRaises(GuardError):
                self.shape(self.previous, previous_velocity=value)

    def test_nonfinite_computed_acceleration_budget_rejected(self):
        with self.assertRaisesRegex(GuardError, "Non-finite joint acceleration budget"):
            self.shape(self.previous, elapsed_s=1e308, previous_interval_s=1e308)

    def test_sequential_quantized_reversals_obey_limit_and_hard_caps(self):
        target = self.previous
        velocity = None
        previous_interval = None
        for index in range(160):
            dt = (.035, .055, .045, .065)[index % 4]
            desired = ((10., -10., 8., -8., 5., -5., .1)
                if index % 20 < 10 else (-10., 10., -8., 8., -5., 5., 0.))
            result, relaxed = self.shape(desired, state=target, previous_target=target,
                previous_velocity=velocity, previous_interval_s=previous_interval,
                elapsed_s=dt, rate_elapsed_s=dt-.002)
            self.assertEqual(relaxed, ())
            new_velocity = tuple((new-old)/dt for new, old in zip(result[:6], target[:6]))
            if velocity is not None:
                for new, old in zip(new_velocity, velocity):
                    self.assertLessEqual(abs(new-old), 160. * (dt+previous_interval)/2. + 1e-12)
            for axis, (new, old) in enumerate(zip(result, target)):
                self.assertLessEqual(abs(new-old), self.caps[axis])
                self.assertLessEqual(abs(new-old), self.limits.max_target_speed[axis]*(dt-.002)*.9)
            self.assert_grid(result)
            target, velocity, previous_interval = result, new_velocity, dt


if __name__ == "__main__":
    unittest.main()

"""Bounded release coordination tests; no policy or hardware transports."""

from copy import deepcopy
from dataclasses import replace
import math
import unittest

from lerobot_robot_piper_executor.guard import ACTION_KEYS, GuardError
from lerobot_robot_piper_executor.release_settle import (
    RELEASE_SETTLE, ReleaseSettleState, shape_release_settle, validate_release_settle,
)


BOUNDS = ((-60., 40.), (0., 135.), (-120., 0.), (-45., 45.),
          (-60., 70.), (-80., 60.), (0., .1))
PLACE = (31., 125., -90., 0., 65., -55., .044)
ANCHOR = (31.1, 125.1, -90.1, .1, 65.1, -55.1, .045)
ACTION = (31.5, 125.5, -89.5, .5, 65.5, -54.5, .073)
GRASP = (-21., 102., -54., 6., 45., -25., .044)


class ReleaseSettleTests(unittest.TestCase):
    def setUp(self):
        self.config = deepcopy(RELEASE_SETTLE)

    def shape(self, history, now, *, action=ACTION, actual=PLACE,
              previous=ANCHOR, sampled_at=None, config=None):
        return shape_release_settle(action, observed_state=actual,
            previous_target=previous, sampled_at=now if sampled_at is None else sampled_at,
            now=now, history=history, config=self.config if config is None else config)

    def armed(self):
        action = GRASP[:6] + (.02,)
        _, state, decision = self.shape(ReleaseSettleState(), 0., action=action, actual=GRASP)
        self.assertEqual(decision, 'armed')
        return state

    def started(self):
        state = self.armed()
        for now in (1., 1.05, 1.10, 1.16):
            result, state, decision = self.shape(state, now)
        self.assertEqual(decision, 'started')
        return result, state

    def test_requires_prior_grasp_side_closing_and_full_sample_coverage(self):
        state = ReleaseSettleState()
        for now in (1., 1.05, 1.10, 1.16):
            result, state, decision = self.shape(state, now)
        self.assertEqual(result, ACTION)
        self.assertEqual(decision, 'idle')
        state = self.armed()
        for now in (1., 1.05, 1.10):
            result, state, decision = self.shape(state, now)
            self.assertEqual(result, ACTION)
            self.assertEqual(decision, 'waiting_for_stability')
        result, state, decision = self.shape(state, 1.16)
        self.assertEqual(decision, 'started')
        self.assertEqual(result[:6], ANCHOR[:6])
        self.assertEqual(result[6], ACTION[6])
        self.assertFalse(state.armed)

    def test_current_act_gripper_is_never_frozen_or_generated(self):
        _, state = self.started()
        for now, width in ((1.21, .072), (1.26, .068), (1.31, .066)):
            result, state, decision = self.shape(state, now, action=ACTION[:6] + (width,))
            self.assertEqual(decision, 'holding')
            self.assertEqual(result[:6], ANCHOR[:6])
            self.assertEqual(result[6], width)

    def test_relative_five_mm_feedback_releases_without_absolute_sixty_mm(self):
        _, state = self.started()
        result, state, decision = self.shape(state, 1.21,
            actual=PLACE[:6] + (PLACE[6] + self.config['release_gain_m'],))
        self.assertEqual(decision, 'released_relative')
        self.assertLess(PLACE[6] + self.config['release_gain_m'], .060)
        self.assertEqual(result, ACTION)
        self.assertFalse(state.active)
        self.assertFalse(state.armed)

    def test_timeout_automatically_releases_and_cannot_repeat_for_new_chunks(self):
        _, state = self.started()
        for now in (1.21, 1.26, 1.31, 1.36, 1.41, 1.46):
            _, state, _ = self.shape(state, now)
        result, state, decision = self.shape(state, 1.16 + self.config['max_hold_s'])
        self.assertEqual(decision, 'released_timeout')
        self.assertEqual(result, ACTION)
        for now in (1.56, 1.61, 1.66, 1.71, 1.76):
            result, state, decision = self.shape(state, now)
            self.assertEqual(result, ACTION)
            self.assertFalse(state.active)

    def test_withdrawn_opening_then_reopened_cannot_retrigger_until_new_grasp(self):
        _, state = self.started()
        closed = ACTION[:6] + (.02,)
        result, state, decision = self.shape(state, 1.21, action=closed)
        self.assertEqual(decision, 'cancelled_opening_withdrawn')
        self.assertEqual(result, closed)
        for now in (1.26, 1.31, 1.36, 1.42):
            result, state, decision = self.shape(state, now)
            self.assertEqual(result, ACTION)
            self.assertFalse(state.active)
        _, state, decision = self.shape(state, 2., action=GRASP[:6] + (.02,), actual=GRASP)
        self.assertEqual(decision, 'armed')
        for now in (3., 3.05, 3.10, 3.16):
            _, state, decision = self.shape(state, now)
        self.assertEqual(decision, 'started')

    def test_leaving_window_or_large_requested_joint_motion_cancels_immediately(self):
        for case in ('window', 'trajectory'):
            _, state = self.started()
            kwargs = ({'actual': (24.,) + PLACE[1:]} if case == 'window'
                else {'action': (ANCHOR[0] + 5.01,) + ACTION[1:]})
            result, proposed, decision = self.shape(state, 1.21, **kwargs)
            self.assertEqual(decision, 'cancelled_outside_window' if case == 'window'
                else 'cancelled_joint_departure')
            self.assertEqual(result, kwargs.get('action', ACTION))
            self.assertFalse(proposed.active)

    def test_slow_feedback_span_is_not_claimed_as_static_or_contact(self):
        for span, expected in ((.74, 'started'), (.76, 'waiting_for_stability')):
            state = self.armed()
            for now, joint in ((1., PLACE[0]-span), (1.05, PLACE[0]),
                               (1.10, PLACE[0]), (1.16, PLACE[0])):
                _, state, decision = self.shape(state, now, actual=(joint,) + PLACE[1:])
            self.assertEqual(decision, expected)

    def test_duplicate_future_and_stale_samples_never_manufacture_stability(self):
        state = self.armed()
        _, state, _ = self.shape(state, 1.)
        for now, sample in ((1.05, 1.), (1.10, 1.11), (1.16, 1.)):
            result, state, decision = self.shape(state, now, sampled_at=sample)
            self.assertEqual(result, ACTION)
            self.assertEqual(decision, 'feedback_not_new_or_fresh')
            self.assertFalse(state.active)
        for now in (1.21, 1.26, 1.31):
            _, state, decision = self.shape(state, now)
            self.assertEqual(decision, 'waiting_for_stability')

    def test_single_old_endpoint_and_current_sample_cannot_bridge_large_gap(self):
        state = self.armed()
        for now in (1., 1.06, 1.22, 1.27, 1.32):
            result, state, decision = self.shape(state, now)
            self.assertEqual(result, ACTION)
            self.assertEqual(decision, 'waiting_for_stability')
        _, state, decision = self.shape(state, 1.38)
        self.assertEqual(decision, 'started')

    def test_cleared_buffer_does_not_forget_last_sample_or_monotonic_clock(self):
        state = self.armed()
        _, state, _ = self.shape(state, 1.)
        _, state, decision = self.shape(state, 1.05, sampled_at=1.)
        self.assertEqual(state.samples, ())
        self.assertEqual(state.last_sample_at, 1.)
        # Repeating again after buffer clear is still not a new sample.
        _, state, decision = self.shape(state, 1.06, sampled_at=1.)
        self.assertEqual(decision, 'feedback_not_new_or_fresh')
        self.assertEqual(state.samples, ())
        _, state, decision = self.shape(state, 1.04, sampled_at=1.04)
        self.assertEqual(decision, 'feedback_not_new_or_fresh')
        self.assertEqual(state.last_now, 1.06)
        for now in (1.07, 1.12, 1.17):
            _, state, decision = self.shape(state, now)
            self.assertEqual(decision, 'waiting_for_stability')

    def test_active_hold_exits_on_untrustworthy_samples_without_new_stop(self):
        for now, sample in ((1.21, 1.16), (1.21, 1.22), (1.31, 1.20), (1.40, 1.40)):
            _, state = self.started()
            result, proposed, decision = self.shape(state, now, sampled_at=sample)
            self.assertTrue(decision.startswith('cancelled_'))
            self.assertEqual(result, ACTION)
            self.assertFalse(proposed.active)

    def test_only_ack_commits_proposed_sample_and_latched_reference(self):
        state = self.armed()
        for now in (1., 1.05, 1.10):
            _, state, _ = self.shape(state, now)
        original = state
        first, proposed, decision = self.shape(state, 1.16)
        second, repeated, again = self.shape(state, 1.16)
        self.assertEqual(state, original)
        self.assertEqual((first, proposed, decision), (second, repeated, again))
        self.assertFalse(state.active)
        self.assertTrue(proposed.active)
        self.assertEqual(proposed.anchor, ANCHOR[:6])

    def test_scalar_mapping_can_include_images_without_mutating_inputs(self):
        state = self.armed()
        actual = {**dict(zip(ACTION_KEYS, PLACE)), 'wrist': object(), 'global': object()}
        action = dict(zip(ACTION_KEYS, ACTION))
        before = action.copy()
        result, _, _ = self.shape(state, 1., action=action, actual=actual)
        self.assertEqual(result, ACTION)
        self.assertEqual(action, before)

    def test_opening_requires_raw_intent_relative_gap_and_minimum_actual_width(self):
        cases = ({'action': ACTION[:6] + (.054,)},
            {'action': ACTION[:6] + (PLACE[6]+.005,)},
            {'actual': PLACE[:6] + (.034,)})
        for kwargs in cases:
            state = self.armed()
            for now in (1., 1.05, 1.10, 1.16):
                result, state, _ = self.shape(state, now, **kwargs)
                self.assertEqual(result, kwargs.get('action', ACTION))
                self.assertFalse(state.active)

    def test_configuration_and_finite_input_validation(self):
        validate_release_settle(self.config, bounds=BOUNDS)
        for key in ('max_hold_s', 'stable_span_deg', 'stable_window_s',
                    'trigger_open_m', 'release_gain_m', 'max_sample_gap_s'):
            for value in (math.nan, math.inf, True, '0.1'):
                cfg = {**self.config, key: value}
                with self.subTest(key=key, value=value), self.assertRaises(GuardError):
                    validate_release_settle(cfg, bounds=BOUNDS)
        for cfg in ({}, {**self.config, 'unknown': 1},
                    {**self.config, 'max_hold_s': 2.},
                    {**self.config, 'max_feedback_age_s': .101},
                    {**self.config, 'release_gain_m': .007}):
            with self.subTest(config=cfg), self.assertRaises(GuardError):
                validate_release_settle(cfg, bounds=BOUNDS)
        outside = deepcopy(self.config)
        outside['placement_joint_bounds_deg'][0] = [25., 41.]
        with self.assertRaises(GuardError):
            validate_release_settle(outside, bounds=BOUNDS)
        for invalid in (None, BOUNDS[:6]):
            with self.assertRaises(GuardError):
                validate_release_settle(self.config, bounds=invalid)
        for kwargs in ({'now': math.nan}, {'sampled_at': math.inf},
                       {'action': (math.nan,) + ACTION[1:]},
                       {'actual': PLACE[:6] + (True,)}):
            options = {'now': 1., **kwargs}
            now = options.pop('now')
            with self.subTest(kwargs=kwargs), self.assertRaises(GuardError):
                self.shape(ReleaseSettleState(), now, **options)
        with self.assertRaises(GuardError):
            self.shape(replace(ReleaseSettleState(), active=True), 1.)
        for samples in (None, [], ('bad',), ((1.,),)):
            with self.subTest(samples=samples), self.assertRaises(GuardError):
                self.shape(replace(ReleaseSettleState(), samples=samples), 1.)


if __name__ == '__main__':
    unittest.main()

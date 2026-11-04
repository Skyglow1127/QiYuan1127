"""Offline, model-neutral source-time and continuous-bound fault tests."""
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"sw"))
from rt_adapter.scheduler import ChunkScheduler, TimedChunk
from rt_adapter.ensemble import Ensembler
from rt_adapter.trajectory import C2Tracker, MotionBounds, QuinticSegment, TrajectoryLimitError, exact_tick_times
from rt_adapter.degradation import DegradationController


def bounds(dim=1):
    return MotionBounds(*[np.repeat(value, dim) for value in (-100, 100, 10, 20, 100, 5)])


def scheduler(**kwargs):
    return ChunkScheduler(consume_steps=30, source_max_age_s=2, slot_lateness_s=.01, bounds=bounds(), **kwargs)


class ChunkTests(unittest.TestCase):
    def test_chunk_copies_inputs_and_preserves_original_times(self):
        q = np.zeros((100, 1)); q[3] = .1
        chunk = TimedChunk(1, 10, 10.15, 20, q)
        q[3] = 100
        self.assertEqual(chunk.targets[3, 0], .1)
        self.assertAlmostEqual(chunk.slot_at(3), 10.15)
        self.assertFalse(chunk.targets.flags.writeable)

    def test_stale_slots_discarded_without_retimestamp(self):
        sch = scheduler()
        result = sch.accept(TimedChunk(0, 10, 10.19, 20, np.zeros((100, 1))), now=10.19, current_target=[0])
        self.assertEqual(result["first_valid_slot"], 4)
        self.assertIsNone(sch.reference_at(now=10.19))
        sample = sch.pop_due(now=10.2)
        self.assertEqual(sample.slot_index, 4)
        self.assertEqual(sample.observation_at, 10)
        self.assertAlmostEqual(sample.scheduled_at, 10.2)

    def test_entire_expired_prefix_cannot_be_reused(self):
        with self.assertRaisesRegex(ValueError, "all slots"):
            scheduler().accept(TimedChunk(0, 0, 1.6, 20, np.zeros((100, 1))), now=1.6, current_target=[0])

    def test_source_expiration_checked_again_at_output(self):
        sch = ChunkScheduler(consume_steps=100, source_max_age_s=.2, slot_lateness_s=.01, bounds=bounds())
        sch.accept(TimedChunk(0, 0, 0, 20, np.zeros((100, 1))), now=0, current_target=[0])
        self.assertIsNotNone(sch.reference_at(now=.15))
        self.assertIsNone(sch.reference_at(now=.25))

    def test_replayed_sequence_future_and_boundary_jump_rejected(self):
        sch = scheduler()
        chunk = TimedChunk(1, 0, 0, 20, np.zeros((100, 1)))
        sch.accept(chunk, now=0, current_target=[0])
        with self.assertRaisesRegex(ValueError, "sequence"):
            sch.accept(chunk, now=.1, current_target=[0])
        with self.assertRaisesRegex(ValueError, "future"):
            scheduler().accept(TimedChunk(0, 0, .2, 20, np.zeros((100, 1))), now=.1, current_target=[0])
        with self.assertRaisesRegex(TrajectoryLimitError, "DQ_MAX"):
            scheduler().accept(TimedChunk(0, 0, 0, 20, np.full((100, 1), 6)), now=0, current_target=[0])

    def test_rolling_window_and_explicit_profile_extension(self):
        for extension, last in ((0, 29), (20, 49)):
            sch = ChunkScheduler(consume_steps=30, source_max_age_s=4, slot_lateness_s=.01, bounds=bounds())
            sch.accept(TimedChunk(0, 0, 0, 20, np.zeros((100, 1))), now=0, current_target=[0], additional_steps=extension)
            self.assertIsNotNone(sch.reference_at(now=last/20))
            self.assertIsNone(sch.reference_at(now=(last+1)/20))
        with self.assertRaises(ValueError):
            scheduler().accept(TimedChunk(0, 0, 0, 20, np.zeros((100, 1))), now=0, current_target=[0], additional_steps=21)

    def test_none_ensemble_exact_and_first_order_causal(self):
        target = np.array([1., 2.])
        np.testing.assert_array_equal(Ensembler().apply(target, scheduled_at=0, now=0), target)
        blend = Ensembler("first_order", alpha=.25)
        blend.apply([0], scheduled_at=0, now=0)
        np.testing.assert_array_equal(blend.apply([4], scheduled_at=.1, now=.1), [1])
        with self.assertRaises(ValueError): blend.apply([4], scheduled_at=.3, now=.2)

    def test_noninteger_rate_has_no_cumulative_phase_drift(self):
        rate = 137.5
        ticks = exact_tick_times(123, 100, rate)
        self.assertEqual(len(ticks), 13750)
        np.testing.assert_array_equal(ticks, 123+np.arange(13750)/rate)

    def test_quintic_end_states_c2_and_analytic_limits(self):
        segment = QuinticSegment(0, 2, [0], [0], [0], [1], [0], [0])
        for t, q in ((0, 0), (2, 1)):
            self.assertAlmostEqual(float(segment.evaluate(t)[0]), q)
            self.assertAlmostEqual(float(segment.evaluate(t, 1)[0]), 0)
            self.assertAlmostEqual(float(segment.evaluate(t, 2)[0]), 0)
        self.assertTrue(segment.validate(bounds()))
        with self.assertRaises(TrajectoryLimitError):
            QuinticSegment(0, .01, [0], [0], [0], [1], [0], [0]).validate(bounds())

    def test_c2_limiter_respects_analytic_continuous_constraints(self):
        tracker = C2Tracker([0], bounds(), 1)
        dt = .01
        for _ in range(400):
            q0, v0, a0 = tracker.q.copy(), tracker.v.copy(), tracker.a.copy()
            state = tracker.advance([2], dt)
            t = np.linspace(0, dt, 21)[:, None]
            q = q0+v0*t+.5*a0*t*t+state["j"]*t**3/6
            v = v0+a0*t+.5*state["j"]*t*t
            a = a0+state["j"]*t
            self.assertLessEqual(float(np.max(abs(v))), 10+1e-9)
            self.assertLessEqual(float(np.max(abs(a))), 20+1e-9)
            self.assertLessEqual(float(np.max(abs(state["j"]))), 100+1e-9)
            self.assertLessEqual(float(np.max(abs(q-q0))), 5+1e-9)
            np.testing.assert_allclose(q[-1], state["q"], atol=1e-12)
        self.assertLess(abs(tracker.q[0]-2), .01)

    def test_impossible_stop_near_limit_faults_instead_of_fake_hold(self):
        b = MotionBounds([-1], [1], [10], [20], [100], [.1])
        tracker = C2Tracker([.999], b, .4, velocity=[10], acceleration=[20])
        with self.assertRaises(TrajectoryLimitError): tracker.advance([1], .01, braking=True)

    def test_ps_and_future_pl_reference_share_exact_same_algorithm(self):
        first, second = C2Tracker([0], bounds(), .4), C2Tracker([0], bounds(), .4)
        for t in np.arange(0, 2, .01):
            q = [np.sin(t)]
            np.testing.assert_array_equal(first.advance(q, .01)["q"], second.advance(q, .01)["q"])

    def test_slow_inference_recommends_no_automatic_preprocess_change(self):
        controller = DegradationController(C2Tracker([0], bounds(), .4), consume_period_s=1.5, source_max_age_s=2, stop_velocity_tolerance=1e-5)
        decision = controller.inference_health(.8)
        self.assertEqual(decision.reason, "slow_inference")
        self.assertTrue(decision.recommendations)
        self.assertFalse(hasattr(controller, "resolution"))

    def test_starvation_brakes_bounded_then_hold_without_old_chunk(self):
        tracker = C2Tracker([0], bounds(), .4, velocity=[1])
        controller = DegradationController(tracker, consume_period_s=1.5, source_max_age_s=2, stop_velocity_tolerance=1e-5)
        self.assertEqual(controller.evaluate(now=0).state, "RAMP")
        for t in np.arange(0, 3, .01):
            state = controller.brake_tick(.01)
            self.assertLessEqual(abs(state["j"][0]), 100+1e-9)
        self.assertEqual(controller.evaluate(now=3.1).state, "HOLD")
        self.assertEqual(controller.evaluate(now=3.2, observation_at=0, block_available=True).reason, "stale_observation_stop_new_blocks")

    def test_bounds_and_ik_rejections_do_not_send_bad_block(self):
        controller = DegradationController(C2Tracker([0], bounds(), .4), consume_period_s=1.5, source_max_age_s=2, stop_velocity_tolerance=1e-5)
        for options in ({"bounds_ok": False}, {"ik_ok": False}):
            decision = controller.evaluate(now=0, observation_at=0, block_available=True, **options)
            self.assertEqual(decision.reason, "reject_block_bounds_or_ik")

    def test_braking_deadline_never_fakes_instantaneous_hold(self):
        tracker = C2Tracker([0], bounds(), .4, velocity=[1])
        controller = DegradationController(tracker, consume_period_s=1.5, source_max_age_s=2, stop_velocity_tolerance=1e-5)
        controller.evaluate(now=0)
        decision = controller.evaluate(now=3.1)
        self.assertEqual(decision.state, "FAULT")
        self.assertEqual(controller.evaluate(now=3.2, observation_at=3.2, block_available=True).state, "FAULT")

    def test_offline_replay_requires_explicit_design_provenance(self):
        spec = importlib.util.spec_from_file_location("replay_chunks", ROOT/"measure"/"replay_chunks.py")
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        with self.assertRaisesRegex(ValueError, "design_test_only"):
            module.run_replay({}, {}, {}, 137.5)

    def test_recorded_projection_is_explicit_and_equal_in_all_baselines(self):
        spec = importlib.util.spec_from_file_location("replay_projection", ROOT/"measure"/"replay_chunks.py")
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        contract = {"fps_dataset": 20, "chunk_size": 100, "runtime_n_action_steps": 30, "action_names": ["test"], "action_units": ["deg"]}
        limits = {"lower": [0], "upper": [1], "velocity": [10], "acceleration": [20], "jerk": [100], "dq_max": [1]}
        scenario = {"provenance": "design_test_only", "motion_bounds": limits, "response_time_s": 1, "source_max_age_s": 2, "slot_lateness_s": .05, "analysis_cutoff_hz": 5}
        episode = {"frame_index": np.arange(60), "timestamp": np.arange(60)/20, "action": np.full((60, 1), -.1)}
        with self.assertRaisesRegex(ValueError, "no implicit clipping"):
            module.run_replay(contract, scenario, episode, 100)
        report, arrays, _ = module.run_replay(contract, scenario, episode, 100, project_recorded_inputs=True)
        self.assertEqual(report["recorded_ingress_projection"]["projected_frames_per_axis"], [60])
        self.assertEqual(report["recorded_ingress_projection"]["maximum_abs_correction_per_axis"], [.1])
        np.testing.assert_array_equal(arrays["recorded_raw_actions"], episode["action"])
        for name in ("coarse", "recorded_linear", "c2", "ps_baseline"):
            self.assertTrue(np.all(arrays[name] == 0))
        self.assertIsNone(report["physical_f_exec_hz"])

    def test_replay_caller_does_not_emit_target_after_latched_fault(self):
        spec = importlib.util.spec_from_file_location("replay_fault", ROOT/"measure"/"replay_chunks.py")
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        contract = {"fps_dataset": 20, "chunk_size": 100, "runtime_n_action_steps": 30, "action_names": ["test"], "action_units": ["deg"]}
        limits = {"lower": [0], "upper": [1], "velocity": [10], "acceleration": [20], "jerk": [100], "dq_max": [1]}
        scenario = {"provenance": "design_test_only", "motion_bounds": limits, "response_time_s": 1, "source_max_age_s": 2, "slot_lateness_s": .05, "analysis_cutoff_hz": 5}
        episode = {"frame_index": np.arange(60), "timestamp": np.arange(60)/20, "action": np.zeros((60, 1))}
        from rt_adapter.degradation import DegradationDecision
        with patch("rt_adapter.degradation.DegradationController.evaluate", return_value=DegradationDecision("FAULT", "test_latched_fault")), patch.object(C2Tracker, "advance", side_effect=AssertionError("must not emit after fault")):
            with self.assertRaisesRegex(ValueError, "normal target output is latched off"):
                module.run_replay(contract, scenario, episode, 100)

    def test_nonfinite_future_and_reversed_evaluation_are_rejected(self):
        for value in (float("nan"), float("inf")):
            with self.assertRaises(ValueError): Ensembler().apply([1], scheduled_at=value, now=0)
            with self.assertRaises(ValueError): scheduler().reference_at(now=value)
        sch = scheduler()
        sch.accept(TimedChunk(0, 0, 0, 20, np.zeros((100, 1))), now=.1, current_target=[0])
        with self.assertRaises(ValueError): sch.reference_at(now=.05)
        control = DegradationController(C2Tracker([0], bounds(), .4), consume_period_s=1.5, source_max_age_s=2, stop_velocity_tolerance=1e-5)
        with self.assertRaises(ValueError): control.evaluate(now=0, observation_at=.1, block_available=True)
        with self.assertRaises(ValueError): control.evaluate(now=0, observation_at=float("nan"))
        control.evaluate(now=1)
        with self.assertRaises(ValueError): control.evaluate(now=.5)
        with self.assertRaises(ValueError): control.evaluate(now=1, block_available=True)

    def test_outward_boundary_state_has_no_finite_safe_jerk(self):
        b = MotionBounds([-1], [1], [2], [10], [100], [5])
        for q, v, a in ((1, 0, 1), (1, .1, 0), (-1, 0, -1), (0, 2, 1), (0, -2, -1)):
            tracker = C2Tracker([q], b, 1, velocity=[v], acceleration=[a])
            with self.assertRaisesRegex(TrajectoryLimitError, "outward"):
                tracker.advance([0], .1)
        segment = QuinticSegment(0, 2, [0], [0], [0], [1], [0], [0])
        with self.assertRaises(ValueError): segment.evaluate(float("nan"))


if __name__ == "__main__": unittest.main()

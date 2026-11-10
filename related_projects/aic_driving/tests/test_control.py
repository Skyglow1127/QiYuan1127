import numpy as np
import pytest

from aic_drive.control import SafetyController
from aic_drive.prediction import AnalyticPredictor, longitudinal_motion
from aic_drive.types import EgoState, TrackSample, TrackState


def sample(t, x, vx, y=0.0, vy=0.0, track_id=1):
    return TrackSample(t, TrackState(track_id, x, y, vx, vy))


def test_cv_queries_absolute_time_and_single_sample_ca_fallback():
    samples = (sample(10.0, 20.0, 5.0, y=4.0, vy=-1.0),)
    predicted = AnalyticPredictor("ca").predict(samples, np.array([10.2, 11.0]))
    np.testing.assert_allclose(predicted, [[21, 3.8, 5, -1], [25, 3, 5, -1]])


def test_ca_uses_observed_velocity_change():
    history = (sample(0.0, 0, 2), sample(0.5, 1.25, 3))
    result = AnalyticPredictor("ca").predict(history, np.array([1.0, 1.5]))
    np.testing.assert_allclose(result[:, [0, 2]], [[3.0, 4.0], [5.25, 5.0]])


def test_low_speed_braking_stops_at_exact_distance_and_never_reverses():
    x, v = longitudinal_motion(10.0, 1.0, -6.0, np.array([0, 0.1, 1 / 6, 1, 2]))
    assert np.all(v >= 0)
    assert np.all(np.diff(x) >= 0)
    np.testing.assert_allclose(x[-3:], 10 + 1 / 12)
    np.testing.assert_allclose(v[-3:], 0)
    history = (sample(0.0, 0, 2), sample(0.2, 0.28, 0.8))
    prediction = AnalyticPredictor("ca").predict(history, np.array([0.2, 1.0, 2.0]))
    assert np.all(prediction[:, 2] >= 0)
    assert prediction[-1, 0] == pytest.approx(0.28 + 0.8 ** 2 / 12)


def test_braking_guard_handles_unobserved_lead_braking():
    # At now=0.4 a CV estimate would still place the lead 30m ahead.
    # It may instead have been braking since its last observed state.
    history = (sample(-0.1, 27.5, 25), sample(0.0, 30, 25))
    ego = EgoState(0.4, 10, 0, 25)
    guarded = SafetyController().decide(ego, {1: history})
    unguarded = SafetyController(guard=False).decide(ego, {1: history})
    assert guarded.acceleration < unguarded.acceleration
    assert guarded.acceleration <= -2


def test_normal_distant_following_makes_progress_from_public_inputs_only():
    ego = EgoState(1.0, 0, 0, 20)
    history = (sample(0.8, 76, 20), sample(0.9, 78, 20))
    result = SafetyController().decide(ego, {1: history})
    assert result.acceleration > 0
    assert result.candidate_count > 0
    assert result.prediction_ms >= 0
    assert not result.degraded


def test_stale_and_missing_observations_are_explicit_degradation():
    ego = EgoState(1.0, 0, 0, 5)
    result = SafetyController(max_age=0.8).decide(ego, {1: (sample(0, 100, 10),)})
    assert result.degraded and result.emergency
    assert result.reason == "stale_observation"
    assert result.acceleration == -6
    assert result.max_observation_age == 1
    missing = SafetyController().decide(ego, {})
    assert missing.reason == "no_observation" and missing.degraded


def test_cutin_future_lateral_occupancy_is_checked():
    ego = EgoState(0.1, 0, 0, 20)
    incoming = (sample(0, 18, 12, y=4.2, vy=-2), sample(0.1, 19.2, 12, y=4, vy=-2))
    parallel = (sample(0, 18, 12, y=4, vy=0), sample(0.1, 19.2, 12, y=4, vy=0))
    controller = SafetyController()
    assert controller.decide(ego, {1: incoming}).acceleration < controller.decide(ego, {1: parallel}).acceleration


def test_future_observations_are_rejected():
    with pytest.raises(ValueError, match="Future"):
        SafetyController().decide(EgoState(0, 0, 0, 10), {1: (sample(1, 100, 10),)})


def test_predictor_receives_age_plus_entire_planning_horizon():
    class RecordingPredictor(AnalyticPredictor):
        def predict(self, samples, query_times):
            self.queries = query_times.copy()
            return super().predict(samples, query_times)

    predictor = RecordingPredictor()
    history = (sample(0.5, 90, 20), sample(0.6, 92, 20))
    SafetyController(predictor).decide(EgoState(1, 0, 0, 20), {1: history})
    assert predictor.queries[0] == 1.0
    assert predictor.queries[-1] == 3.0
    assert predictor.queries[-1] - history[-1].captured_at == pytest.approx(2.4)


def test_uncertainty_is_positive_and_marked_uncalibrated():
    predictor = AnalyticPredictor()
    widths = predictor.uncertainty(np.array([0.0, 0.4, 2.4]))
    assert widths.shape == (3, 2) and np.all(widths > 0)
    assert "not_calibrated" in predictor.uncertainty_kind


def test_smooth_candidates_prefer_small_action_changes_when_safe():
    ego = EgoState(1, 0, 0, 20, last_acceleration=0)
    history = (sample(.8, 176, 20), sample(.9, 178, 20))
    c = SafetyController(smooth=True)
    d = c.decide(ego, {1: history})
    assert 0 < d.acceleration <= .4
    assert c.last_diagnostics['comfort_override'] == 0
    assert c.last_diagnostics['candidates'] == 33


def test_smoothness_cannot_prevent_required_emergency_braking():
    ego = EgoState(.4, 10, 0, 25, last_acceleration=2)
    history = (sample(-.1, 27.5, 25), sample(0, 30, 25))
    c = SafetyController(smooth=True)
    d = c.decide(ego, {1: history})
    assert d.acceleration < ego.last_acceleration - .4
    assert c.last_diagnostics['guard_rejected'] > 0
    assert d.acceleration >= -6


def test_recovery_releases_braking_faster_but_checks_every_candidate():
    ego = EgoState(1, 0, 0, 20, last_acceleration=-4)
    history = (sample(.8, 176, 20), sample(.9, 178, 20))
    slow = SafetyController(smooth=True).decide(ego, {1: history})
    c = SafetyController(smooth=True, recovery_step=2)
    recovery = c.decide(ego, {1: history})
    assert slow.acceleration <= -3.6
    assert recovery.acceleration == -2
    assert c.last_diagnostics['candidates'] == 33
    dangerous = {1: (sample(-.1,27.5,25), sample(0,30,25))}
    assert c.decide(EgoState(.4,10,0,25,last_acceleration=2), dangerous).acceleration < 1.6

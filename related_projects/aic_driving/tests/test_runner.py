import hashlib
import json

import numpy as np
import pytest

from aic_drive.control import SafetyController
from aic_drive.prediction import AnalyticPredictor
from aic_drive.runner import interval_kinematics, make_controller, run_episode
from aic_drive.types import EpisodeConfig


def test_cv_method_does_not_silently_load_ca_calibration():
    controller = make_controller("cv_fixed", calibration_path="unused-ca-calibration.json")
    assert isinstance(controller.predictor, AnalyticPredictor)
    assert controller.predictor.kind == "cv"
    assert controller.margin_mode == "fixed"


def test_duration_must_end_on_control_boundary():
    with pytest.raises(ValueError, match="integer multiple"):
        EpisodeConfig(duration=0.15)
    assert EpisodeConfig(duration=0.3).duration == 0.3


@pytest.mark.parametrize("simulation_hz,control_hz", [(50, 5), (100, 10), (100, 20)])
def test_mvp_rejects_other_simulation_or_control_frequencies(simulation_hz, control_hz):
    with pytest.raises(ValueError, match="fixed 50 Hz"):
        EpisodeConfig(simulation_hz=simulation_hz, control_hz=control_hz)


def test_partial_terminal_interval_uses_actual_time():
    dt, acceleration, jerk_dt, jerk = interval_kinematics(
        [0.0, 0.1, 0.14], [10.0, 9.8, 9.56])
    np.testing.assert_allclose(dt, [0.1, 0.04])
    np.testing.assert_allclose(acceleration, [-2.0, -6.0])
    np.testing.assert_allclose(jerk_dt, [0.07])
    np.testing.assert_allclose(jerk, [-4 / 0.07])


def test_last_action_and_actual_controller_are_recorded(tmp_path):
    controller = SafetyController(target_speed=22.0, fixed_margin=3.0,
                                  max_age=0.6, guard=False)
    output = tmp_path / "terminal-interval"
    result = run_episode(EpisodeConfig(duration=0.1, delay=0.2),
                         output_dir=output, controller=controller)
    # The sole action is the initial missing-observation braking interval. The
    # previous runner omitted it entirely because only the initial speed existed.
    assert result["acceleration_interval_count"] == 1
    assert result["hard_brake_cycles"] == 1
    assert result["hard_brake_duration_s"] == pytest.approx(0.1)
    assert result["missing_observation_count"] == 1
    assert result["stale_observation_count"] == 0
    assert result["emergency_reason_counts"] == {"no_observation": 1}
    assert result["reason_counts"] == {"no_observation": 1}
    assert result["timing_warmup_decisions"] == 0
    config = json.loads((output / "config.json").read_text())
    effective = config["effective_controller"]
    assert effective["target_speed"] == 22
    assert effective["fixed_margin"] == 3
    assert effective["max_age"] == 0.6
    assert effective["guard"] is False
    digest = hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()
    assert digest == result["config_sha256"]
    rows = [json.loads(row) for row in (output / "trajectory.jsonl").read_text().splitlines()]
    assert rows[0]["decision_at"] == rows[0]["applied_at"] == 0.0
    assert rows[-1]["time"] == pytest.approx(0.1)
    assert rows[-1]["terminal"] is True

import json

import numpy as np
import pytest

from aic_drive.learning import (
    FEATURE_SCHEMA, CalibratedAnalyticPredictor, LearnedPredictor, _load_dataset, build_features,
    calibrate_model, train_model,
)
from aic_drive.types import TrackSample, TrackState


def sample(time, vx=10.0, vy=0.0):
    return TrackSample(time, TrackState(1, time * vx, time * vy, vx, vy))


def make_dataset(tmp_path, leak=False):
    rows = []
    for split, level in (("train", 1.0), ("validation", 10.0),
                         ("calibration", 100.0), ("test", 1000.0)):
        for index in range(4):
            features = np.full(32, level + index, dtype=np.float32)
            features[-2] = 0.5 / 3
            rows.append((features, [level, level / 2], 0.5,
                         "shared" if leak else f"{split}-{index // 2}", split))
    path = tmp_path / "dataset.npz"
    np.savez_compressed(path, features=np.stack([r[0] for r in rows]),
                        targets=np.asarray([r[1] for r in rows], dtype=np.float32),
                        horizons=np.asarray([r[2] for r in rows]),
                        episode_ids=np.asarray([r[3] for r in rows]),
                        split=np.asarray([r[4] for r in rows]))
    (tmp_path / "metadata.json").write_text(json.dumps({"feature_schema": FEATURE_SCHEMA}))
    return path


def test_no_future_history_fill_and_no_future_query():
    features = build_features((sample(0.85, 4), sample(1.0, 10)), 1.6)
    np.testing.assert_allclose(features[:9], [0, 0, 1, -0.6, 0, 1, 0, 0, 0])
    np.testing.assert_allclose(features[9:30], 0)
    assert features[-2] == pytest.approx(0.2)
    with pytest.raises(ValueError, match="precede"):
        build_features((sample(1.0),), 0.9)


def test_older_than_window_ignored_and_order_independent():
    samples = (sample(-2, 90), sample(0, 8), sample(0.1, 9), sample(0.2, 10))
    expected = build_features(samples[1:], 1.2)
    np.testing.assert_array_equal(build_features(samples[::-1], 1.2), expected)


def test_episode_leakage_rejected(tmp_path):
    with pytest.raises(ValueError, match="leakage"):
        _load_dataset(make_dataset(tmp_path, leak=True))


def test_train_only_statistics_and_independent_calibration(tmp_path):
    pytest.importorskip("torch")
    dataset = make_dataset(tmp_path)
    output = tmp_path / "model"
    metadata = train_model(dataset, output, epochs=2, seed=7)
    expected = np.full(32, 2.5)
    expected[-2] = 0.5 / 3
    np.testing.assert_allclose(metadata["input_mean"], expected)
    assert metadata["normalization_source"] == "train"
    assert metadata["heldout_errors"]["analytic"]["mae_xy_m"] == [1000, 500]
    calibrated = calibrate_model(dataset, output, tmp_path / "calibration.json")
    np.testing.assert_allclose(calibrated["analytic"]["quantiles"], [[100, 50]])
    assert calibrated["analytic"]["heldout_coverage"]["coordinate_coverage"] == [0, 0]
    assert calibrated["analytic"]["heldout_coverage"]["used_for_fitting"] is False
    analytic = CalibratedAnalyticPredictor(tmp_path / "calibration.json")
    np.testing.assert_allclose(analytic.uncertainty([0.5]), [[100, 50]])
    assert np.isfinite(analytic.uncertainty([2])).all()
    assert analytic.last_uncertainty_degraded
    learned = LearnedPredictor(output, tmp_path / "calibration.json")
    learned.predict((sample(0),), [0.5])
    np.testing.assert_allclose(learned.uncertainty([0.5]), [[100, 50]])
    assert learned.last_uncertainty_degraded


def test_online_offline_features_identical(tmp_path, monkeypatch):
    pytest.importorskip("torch")
    dataset = make_dataset(tmp_path)
    train_model(dataset, tmp_path / "model", epochs=1, seed=7)
    predictor = LearnedPredictor(tmp_path / "model")
    history = (sample(0, 8), sample(0.1, 9), sample(0.2, 10))
    offline = np.stack([build_features(history, 0.7)])
    captured = []
    def residuals(features):
        captured.append(features.copy())
        return np.zeros((len(features), 2))
    monkeypatch.setattr(predictor, "residuals", residuals)
    result = predictor.predict(history, [0.7])
    assert result.shape == (1, 4)
    np.testing.assert_array_equal(captured[0], offline)
    assert predictor.calibration_status == "uncalibrated_fallback"
    assert np.isfinite(predictor.uncertainty([0.5])).all()
    predictor.predict((sample(0.2),), [0.7])
    assert predictor.last_degraded


def test_online_horizon_float32_boundaries(tmp_path):
    pytest.importorskip("torch")
    train_model(make_dataset(tmp_path), tmp_path / "model", epochs=1, seed=7)
    predictor = LearnedPredictor(tmp_path / "model")
    predictor.supported_horizons = [float(np.float32(0.1)), float(np.float32(2.4))]
    history = tuple(sample(time) for time in (0, 0.1, 0.2, 0.3))
    predictor.predict(history, [0.4])
    assert not predictor.last_degraded
    assert not predictor._last_fallback[0]
    at_observation = predictor.predict(history, [0.3])
    np.testing.assert_allclose(at_observation[0], [3, 0, 10, 0])
    assert not predictor.last_degraded
    predictor.predict(history, [2.8])
    assert predictor.last_degraded
    assert predictor._last_fallback[0]

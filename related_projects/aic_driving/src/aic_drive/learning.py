"""CPU residual prediction, episode-isolated fitting, and empirical calibration.

Only ``build_features`` is used when constructing offline inputs and online
predictions. Targets are future position minus the frozen CA position forecast.
Learned positions retain CA velocities as an approximation, not the derivative
of the learned position curve. Calibration is empirical, not a safety guarantee.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from .types import TrackSample

FEATURE_SCHEMA = {
    "version": "velocity-history-v1",
    "size": 32,
    "history_seconds": 1.0,
    "slot_offsets_seconds": [round(i * 0.1, 1) for i in range(10)],
    "slot_order": "newest-first",
    "slot_fields": ["(vx-latest_vx)/10", "(vy-latest_vy)/5", "valid"],
    "tail_fields": ["(query_time-latest_captured_at)/3", "latest_vy/5"],
    "sampling": "latest captured_at <= slot_time; no future interpolation",
}
SPLITS = ("train", "validation", "calibration", "test")
MIN_HISTORY_SECONDS = 0.2
MIN_HISTORY_SAMPLES = 3


def build_features(samples: tuple[TrackSample, ...], query_time: float) -> np.ndarray:
    """Construct 32 causal features from the already-arrived sample history.

    The caller must only supply received observations. This function cannot infer
    packet arrival time from captured_at. Missing old slots remain all zero;
    the next available (future relative to a slot) sample never fills that slot.
    """
    if not samples:
        raise ValueError("At least one received TrackSample is required")
    ordered = sorted(samples, key=lambda sample: sample.captured_at)
    if len({sample.state.track_id for sample in ordered}) != 1:
        raise ValueError("History must contain exactly one track identity")
    times = np.asarray([sample.captured_at for sample in ordered], dtype=np.float64)
    if not np.isfinite(times).all() or not np.isfinite(query_time):
        raise ValueError("Observation and query times must be finite")
    latest = ordered[-1]
    horizon = float(query_time - latest.captured_at)
    if horizon < -1e-9:
        raise ValueError("Query time cannot precede the latest observation")
    first = int(np.searchsorted(times, latest.captured_at - 1.0 - 1e-9))
    ordered, times = ordered[first:], times[first:]
    result = np.zeros(32, dtype=np.float32)
    for slot in range(10):
        slot_time = latest.captured_at - 0.1 * slot
        index = int(np.searchsorted(times, slot_time + 1e-9, side="right") - 1)
        if index >= 0:
            state = ordered[index].state
            result[3 * slot:3 * slot + 3] = (
                (state.vx - latest.state.vx) / 10.0,
                (state.vy - latest.state.vy) / 5.0,
                1.0,
            )
    result[-2:] = (max(0.0, horizon) / 3.0, latest.state.vy / 5.0)
    if not np.isfinite(result).all():
        raise ValueError("Feature velocities must be finite")
    return result


def _torch():
    # Keep feature extraction usable before the optional torch dependency exists.
    import torch
    torch.set_num_threads(1)
    return torch


def _network():
    torch = _torch()
    return torch.nn.Sequential(
        torch.nn.Linear(32, 64), torch.nn.ReLU(),
        torch.nn.Linear(64, 64), torch.nn.ReLU(),
        torch.nn.Linear(64, 2),
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def _load_dataset(dataset_path: str | Path) -> tuple[dict[str, np.ndarray], dict]:
    path = Path(dataset_path)
    with np.load(path, allow_pickle=False) as archive:
        required = ("features", "targets", "horizons", "episode_ids", "split")
        if missing := set(required) - set(archive.files):
            raise ValueError(f"Dataset missing arrays: {sorted(missing)}")
        data = {key: np.array(archive[key]) for key in required}
    count = len(data["features"])
    if data["features"].shape != (count, 32) or data["targets"].shape != (count, 2):
        raise ValueError("Expected features (N,32) and residual targets (N,2)")
    if any(data[key].shape != (count,) for key in ("horizons", "episode_ids", "split")):
        raise ValueError("Horizons, episode_ids, and split must be one-dimensional")
    for key in ("features", "targets", "horizons"):
        if not np.isfinite(data[key]).all():
            raise ValueError(f"Non-finite values in {key}")
    if np.any(data["horizons"] < 0):
        raise ValueError("Negative forecast horizon")
    if not np.allclose(data["features"][:, -2], data["horizons"] / 3, atol=1e-6):
        raise ValueError("Feature horizon does not match horizon labels")
    data["split"] = data["split"].astype(str)
    data["episode_ids"] = data["episode_ids"].astype(str)
    if set(data["split"]) != set(SPLITS):
        raise ValueError("All four nonempty train/validation/calibration/test splits are required")
    owner: dict[str, str] = {}
    for episode, split in zip(data["episode_ids"], data["split"]):
        if episode in owner and owner[episode] != split:
            raise ValueError(f"Episode leakage across splits: {episode}")
        owner[episode] = split
    metadata_path = path.with_name("metadata.json")
    metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    if "feature_schema" in metadata and metadata["feature_schema"] != FEATURE_SCHEMA:
        raise ValueError("Dataset feature schema differs from the online extractor")
    return data, metadata


def _residuals(model, features: np.ndarray, mean: np.ndarray, scale: np.ndarray) -> np.ndarray:
    torch = _torch()
    if len(features) == 0:
        return np.empty((0, 2), dtype=np.float32)
    parts = []
    model.eval()
    with torch.inference_mode():
        for offset in range(0, len(features), 4096):
            inputs = ((features[offset:offset + 4096] - mean) / scale).astype(np.float32)
            parts.append(model(torch.from_numpy(inputs)).cpu().numpy())
    return np.concatenate(parts)


def _error_summary(errors: np.ndarray) -> dict[str, Any]:
    absolute = np.abs(errors)
    return {
        "count": int(len(errors)),
        "mae_xy_m": absolute.mean(axis=0).tolist(),
        "rmse_xy_m": np.sqrt(np.mean(errors ** 2, axis=0)).tolist(),
        "p95_absolute_xy_m": np.quantile(absolute, 0.95, axis=0, method="higher").tolist(),
        "mean_displacement_error_m": float(np.linalg.norm(errors, axis=1).mean()),
    }


def train_model(dataset_path: str | Path, output_dir: str | Path,
                epochs: int = 80, seed: int = 0) -> dict[str, Any]:
    """Fit on train only, select epochs on validation, report untouched test errors."""
    if epochs < 1:
        raise ValueError("epochs must be positive")
    data, source_metadata = _load_dataset(dataset_path)
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    if (output / "model.pt").exists():
        raise FileExistsError("Model already exists; choose a fresh output directory")
    torch = _torch()
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    train_mask = data["split"] == "train"
    validation_mask = data["split"] == "validation"
    train_features = data["features"][train_mask].astype(np.float64)
    mean = train_features.mean(axis=0).astype(np.float32)
    scale = train_features.std(axis=0).astype(np.float32)
    scale[scale < 1e-6] = 1.0
    inputs = torch.from_numpy(((train_features - mean) / scale).astype(np.float32))
    labels = torch.from_numpy(data["targets"][train_mask].astype(np.float32))
    validation_labels = torch.from_numpy(data["targets"][validation_mask].astype(np.float32))
    model = _network()
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-4)
    criterion = torch.nn.SmoothL1Loss(beta=1.0)
    best_loss, best_epoch, stale = float("inf"), 0, 0
    best_state = None
    history = []
    for epoch in range(1, epochs + 1):
        model.train()
        permutation = rng.permutation(len(inputs))
        total_loss = 0.0
        for offset in range(0, len(inputs), 256):
            indices = permutation[offset:offset + 256]
            optimizer.zero_grad(set_to_none=True)
            loss = criterion(model(inputs[indices]), labels[indices])
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
            total_loss += float(loss.detach()) * len(indices)
        validation_prediction = _residuals(model, data["features"][validation_mask], mean, scale)
        validation_loss = float(criterion(torch.from_numpy(validation_prediction), validation_labels))
        history.append({"epoch": epoch, "train_loss": total_loss / len(inputs),
                        "validation_loss": validation_loss})
        if validation_loss < best_loss - 1e-5:
            best_loss, best_epoch, stale = validation_loss, epoch, 0
            best_state = copy.deepcopy(model.state_dict())
        else:
            stale += 1
        if stale >= 10:
            break
    model.load_state_dict(best_state)
    split_summary = {
        split: {"rows": int(np.sum(data["split"] == split)),
                "episodes": sorted(set(data["episode_ids"][data["split"] == split]))}
        for split in SPLITS
    }
    supported_horizons = [float(data["horizons"][train_mask].min()),
                          float(data["horizons"][train_mask].max())]
    metadata = {
        "format_version": 1, "architecture": [32, 64, 64, 2],
        "feature_schema": FEATURE_SCHEMA, "target": "actual_xy - CA_xy",
        "device": "cpu", "torch_threads": 1, "torch_version": str(torch.__version__),
        "seed": seed, "requested_epochs": epochs, "completed_epochs": len(history),
        "best_epoch": best_epoch, "best_validation_loss": best_loss,
        "loss": "SmoothL1(beta=1)", "early_stopping_patience": 10,
        "normalization_source": "train", "input_mean": mean.tolist(),
        "input_scale": scale.tolist(), "supported_horizons_seconds": supported_horizons,
        "split_summary": split_summary, "episode_split_leakage_checked": True,
        "dataset_sha256": _sha256(Path(dataset_path)), "dataset_metadata": source_metadata,
        "history": history,
        "limitations": ["CA velocities are retained; they are not derivatives of learned positions",
                        "Short histories and unsupported forecast horizons use CA fallback",
                        "Test metrics are reporting only; model selection uses validation loss"],
    }
    # JSON round-trip removes NumPy scalar/string subclasses from metadata, so
    # checkpoints can use torch's restricted weights_only loader throughout.
    metadata = json.loads(json.dumps(metadata, allow_nan=False))
    torch.save({"state_dict": model.state_dict(), "metadata": metadata}, output / "model.pt")
    test_mask = data["split"] == "test"
    predictions = _residuals(model, data["features"][test_mask], mean, scale)
    targets = data["targets"][test_mask]
    heldout = {"split": "test", "analytic": _error_summary(targets),
               "learned": _error_summary(targets - predictions), "per_horizon": []}
    rounded_test_horizons = np.round(data["horizons"][test_mask], 6)
    for horizon in np.unique(rounded_test_horizons):
        mask = rounded_test_horizons == horizon
        heldout["per_horizon"].append({"horizon_seconds": float(horizon),
                                       "analytic": _error_summary(targets[mask]),
                                       "learned": _error_summary((targets - predictions)[mask])})
    np.savez_compressed(output / "heldout_errors.npz", targets=targets,
                        residual_predictions=predictions, analytic_errors=targets,
                        learned_errors=targets - predictions,
                        horizons=data["horizons"][test_mask],
                        episode_ids=data["episode_ids"][test_mask])
    _write_json(output / "heldout_errors.json", heldout)
    metadata["heldout_errors"] = heldout
    metadata["model_sha256"] = _sha256(output / "model.pt")
    _write_json(output / "metadata.json", metadata)
    return metadata


class LearnedPredictor:
    """CA plus learned position residuals; velocities remain the CA approximation."""

    def __init__(self, model_path: str | Path, calibration_path: str | Path | None = None):
        torch = _torch()
        path = Path(model_path)
        if path.is_dir():
            path = path / "model.pt"
        checkpoint = torch.load(path, map_location="cpu", weights_only=True)
        self.metadata = checkpoint["metadata"]
        if self.metadata["feature_schema"] != FEATURE_SCHEMA:
            raise ValueError("Unsupported model feature schema")
        self.model = _network()
        self.model.load_state_dict(checkpoint["state_dict"])
        self.model.eval()
        self.mean = np.asarray(self.metadata["input_mean"], dtype=np.float32)
        self.scale = np.asarray(self.metadata["input_scale"], dtype=np.float32)
        self.supported_horizons = self.metadata["supported_horizons_seconds"]
        self.calibration = None
        self.calibration_status = "uncalibrated_fallback"
        self.uncertainty_kind = "engineering_heuristic_not_calibrated"
        if calibration_path is not None:
            self.calibration = json.loads(Path(calibration_path).read_text())
            if self.calibration["model_sha256"] != _sha256(path):
                raise ValueError("Calibration belongs to another model")
            self.calibration_status = "empirically_calibrated"
            self.uncertainty_kind = "empirically_calibrated"
        self.last_degraded = False
        self.last_uncertainty_degraded = False
        self._last_horizons = np.array([])
        self._last_fallback = np.array([], dtype=bool)
        self._last_short_history = False

    def residuals(self, features: np.ndarray) -> np.ndarray:
        features = np.asarray(features, dtype=np.float32)
        if features.ndim != 2 or features.shape[1] != 32:
            raise ValueError("Expected features (N,32)")
        return _residuals(self.model, features, self.mean, self.scale)

    def predict(self, samples: tuple[TrackSample, ...], query_times) -> np.ndarray:
        from .prediction import AnalyticPredictor
        if not samples:
            raise ValueError("At least one received observation is required")
        samples = tuple(sorted(samples, key=lambda sample: sample.captured_at))
        query_times = np.asarray(query_times, dtype=float)
        if query_times.ndim != 1:
            raise ValueError("query_times must be one-dimensional")
        if not len(query_times):
            return np.empty((0, 4), dtype=float)
        features = np.stack([build_features(samples, query) for query in query_times])
        latest_time = max(sample.captured_at for sample in samples)
        horizons = query_times - latest_time
        recent = {sample.captured_at for sample in samples if sample.captured_at >= latest_time - 1.0 - 1e-9}
        short = len(recent) < MIN_HISTORY_SAMPLES or max(recent) - min(recent) < MIN_HISTORY_SECONDS - 1e-9
        lower, upper = self.supported_horizons
        # Dataset horizons are float32 while online clock arithmetic is float64.
        # A microsecond tolerance avoids rejecting the same physical endpoint.
        fallback = (horizons < lower - 1e-6) | (horizons > upper + 1e-6) | (horizons <= 1e-9)
        if short:
            fallback[:] = True
        prediction = np.asarray(AnalyticPredictor("ca").predict(samples, query_times), dtype=float).copy()
        if np.any(~fallback):
            prediction[~fallback, :2] += self.residuals(features[~fallback])
        self._last_horizons, self._last_fallback = horizons, fallback
        self._last_short_history = short
        # Identity at h=0 deliberately bypasses the learned residual; that exact
        # observation is not a degraded forecast.
        self.last_degraded = bool(short or np.any(fallback & (horizons > 1e-9)))
        return prediction

    def uncertainty(self, horizons) -> np.ndarray:
        """Empirical coordinate margins, or explicitly uncalibrated fallback margins."""
        horizons = np.asarray(horizons, dtype=float)
        if horizons.ndim != 1 or not np.isfinite(horizons).all() or np.any(horizons < 0):
            raise ValueError("horizons must be a finite nonnegative vector")
        if self.calibration is None:
            self.last_uncertainty_degraded = True
            return _fallback_margins(horizons)
        edges = np.asarray(self.calibration["bucket_edges"], dtype=float)
        indices = np.searchsorted(edges[1:], horizons, side="left")
        outside = (horizons < edges[0]) | (horizons > edges[-1])
        indices = np.clip(indices, 0, len(edges) - 2)
        margins = np.asarray(self.calibration["learned"]["quantiles"], dtype=float)[indices].copy()
        if self._last_horizons.shape == horizons.shape and np.allclose(self._last_horizons, horizons):
            analytic = np.asarray(self.calibration["analytic"]["quantiles"], dtype=float)[indices]
            margins[self._last_fallback] = analytic[self._last_fallback]
            if self._last_short_history:
                margins = np.maximum(margins, _fallback_margins(horizons))
        excess = np.maximum(horizons - edges[-1], 0)
        margins += np.column_stack((0.8 * excess + 0.15 * excess ** 2, 0.3 * excess))
        self.last_uncertainty_degraded = bool(outside.any() or self._last_short_history)
        return margins


def _fallback_margins(horizons: np.ndarray) -> np.ndarray:
    """Declared engineering margins outside calibrated operating assumptions."""
    return np.column_stack((0.5 + 0.8 * horizons + 0.15 * horizons ** 2,
                            0.3 + 0.3 * horizons))


class CalibratedAnalyticPredictor:
    """The frozen CA predictor with its own independent calibration table."""

    uncertainty_kind = "empirically_calibrated"
    calibration_status = "empirically_calibrated"

    def __init__(self, calibration_path: str | Path, kind: str = "ca"):
        from .prediction import AnalyticPredictor
        if kind != "ca":
            raise ValueError("Calibration residual labels were defined against CA, not CV")
        self.base = AnalyticPredictor(kind)
        self.kind = kind
        self.calibration = json.loads(Path(calibration_path).read_text())
        self.last_degraded = False
        self.last_uncertainty_degraded = False

    def predict(self, samples: tuple[TrackSample, ...], query_times) -> np.ndarray:
        samples = tuple(sorted(samples, key=lambda sample: sample.captured_at))
        self.last_degraded = len(samples) < 2
        return self.base.predict(samples, query_times)

    def uncertainty(self, horizons) -> np.ndarray:
        horizons = np.asarray(horizons, dtype=float)
        if horizons.ndim != 1 or not np.isfinite(horizons).all() or np.any(horizons < 0):
            raise ValueError("horizons must be a finite nonnegative vector")
        edges = np.asarray(self.calibration["bucket_edges"], dtype=float)
        indices = np.clip(np.searchsorted(edges[1:], horizons, side="left"), 0, len(edges) - 2)
        margins = np.asarray(self.calibration["analytic"]["quantiles"], dtype=float)[indices].copy()
        outside = horizons > edges[-1]
        excess = np.maximum(horizons - edges[-1], 0)
        margins += np.column_stack((0.8 * excess + 0.15 * excess ** 2, 0.3 * excess))
        if self.last_degraded:
            margins = np.maximum(margins, _fallback_margins(horizons))
        self.last_uncertainty_degraded = bool(outside.any() or self.last_degraded)
        return margins


def calibrate_model(dataset_path: str | Path, model_path: str | Path,
                    output_path: str | Path) -> dict[str, Any]:
    """Estimate per-horizon 95% absolute-error quantiles on calibration only."""
    data, _ = _load_dataset(dataset_path)
    predictor = LearnedPredictor(model_path)
    if predictor.metadata["dataset_sha256"] != _sha256(Path(dataset_path)):
        raise ValueError("Calibration dataset differs from the model's recorded dataset")
    calibration_mask = data["split"] == "calibration"
    horizons = data["horizons"][calibration_mask]
    targets = data["targets"][calibration_mask]
    learned_errors = targets - predictor.residuals(data["features"][calibration_mask])
    maximum = max(0.5, float(horizons.max()))
    edges = np.arange(0.0, np.ceil(maximum * 2) / 2 + 0.25, 0.5)
    bin_index = np.searchsorted(edges[1:], horizons, side="left")
    tables = {}
    for method, errors in (("analytic", targets), ("learned", learned_errors)):
        absolute = np.abs(errors)
        global_quantile = np.quantile(absolute, 0.95, axis=0, method="higher")
        quantiles = []
        for index in range(len(edges) - 1):
            selected = absolute[bin_index == index]
            quantiles.append((np.quantile(selected, 0.95, axis=0, method="higher")
                              if len(selected) else global_quantile).tolist())
        tables[method] = {"quantiles": quantiles,
                          "global_quantile": global_quantile.tolist(),
                          "calibration_errors": _error_summary(errors)}
    model_file = Path(model_path)
    if model_file.is_dir():
        model_file /= "model.pt"
    result = {
        "version": 1, "quantile": 0.95, "quantile_method": "higher", "units": "metres",
        "empirical_only": True, "fit_split": "calibration",
        "model_sha256": _sha256(model_file), "dataset_sha256": _sha256(Path(dataset_path)),
        "bucket_edges": edges.tolist(),
        "bucket_interval": "first [lower,upper], subsequent (lower,upper]",
        "buckets": [{"lower": float(edges[index]), "upper": float(edges[index + 1]),
                     "count": int(np.sum(bin_index == index))}
                    for index in range(len(edges) - 1)],
        "empty_bucket_policy": "global calibration quantile; count remains zero",
        "calibration_episode_ids": sorted(set(data["episode_ids"][calibration_mask])),
        **tables,
    }
    test_mask = data["split"] == "test"
    test_targets = data["targets"][test_mask]
    test_horizons = data["horizons"][test_mask]
    test_indices = np.clip(np.searchsorted(edges[1:], test_horizons, side="left"), 0, len(edges) - 2)
    supported = (test_horizons >= edges[0]) & (test_horizons <= edges[-1])
    test_learned_errors = test_targets - predictor.residuals(data["features"][test_mask])
    for method, errors in (("analytic", test_targets), ("learned", test_learned_errors)):
        margins = np.asarray(tables[method]["quantiles"])[test_indices]
        inside = np.abs(errors) <= margins
        result[method]["heldout_coverage"] = {
            "split": "test", "count_in_calibrated_range": int(supported.sum()),
            "count_outside_calibrated_range": int((~supported).sum()),
            "coordinate_coverage": inside[supported].mean(axis=0).tolist() if supported.any() else None,
            "joint_coverage": float(inside[supported].all(axis=1).mean()) if supported.any() else None,
            "used_for_fitting": False,
        }
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise FileExistsError("Calibration file already exists; choose a fresh output path")
    _write_json(destination, result)
    return result

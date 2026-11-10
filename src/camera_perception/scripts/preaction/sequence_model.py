"""Untrained action-sequence ROI model; no policy, training loop or device APIs.

The interface consumes the complete 10-step action contract. Its four outputs
per step use the legacy translation/log-scale parameter convention (scales
10, 10, 5, 5). They are not grasp, contact or collision predictions. A newly
constructed model is untrained and its numbers are for contract tests only.
Even loading a declared trained checkpoint does not establish forecast quality.
"""
from collections.abc import Mapping
from pathlib import Path
import json

import torch
from torch import nn


MODEL_SCHEMA = "preaction-sequence-gru-v1"
CHECKPOINT_SCHEMA = "preaction-sequence-checkpoint-v1"
INPUT_SPEC = {
    "rgb_mask": [4, 96, 96],
    "q_features": [7],
    "sequence_features": [10, 17],
    "sequence_columns": "delta[0:7],command_mask[7:14],offset_seconds[14],valid[15],future_at_completion[16]",
    "output_parameters": "scaled_translation_xy_log_scale_wh; scales=10,10,5,5",
}


def _binary(values, name):
    if not bool(((values == 0) | (values == 1)).all()):
        raise ValueError(f"{name} must contain only binary 0/1 values")


class PreActionSequence(nn.Module):
    """Small, order-sensitive untrained prototype with a frozen legacy vision.

    ``load_legacy_vision`` must be called explicitly to reuse existing encoder
    weights. It never imports the old endpoint-only prediction head. Otherwise
    the frozen encoder is randomly initialized, suitable only for unit tests.
    Expired but valid actions update the recurrent state; padding never does.
    An all-padding batch item is explicitly defined to return ten zero outputs.
    """

    def __init__(self, hidden_dim=32):
        super().__init__()
        if type(hidden_dim) is not int or hidden_dim < 4:
            raise ValueError("hidden_dim must be an integer >= 4")
        self.hidden_dim = hidden_dim
        self.vision = nn.Sequential(
            nn.Conv2d(3, 8, 3, padding=1), nn.ReLU(),
            nn.Conv2d(8, 8, 3, padding=1), nn.ReLU(),
        )
        self.vision.requires_grad_(False)
        self.spatial = nn.Sequential(
            nn.Conv2d(9, 16, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(16, 16, 3, stride=2, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d((3, 3)), nn.Flatten(),
        )
        self.initial_state = nn.Linear(16 * 3 * 3 + 7, hidden_dim)
        self.recurrent = nn.GRUCell(17, hidden_dim)
        self.motion_head = nn.Linear(hidden_dim, 4)
        self.legacy_vision_loaded = False
        self.training_status = "untrained_contract_test"
        self.training_metadata = {"training_performed": False}

    def load_legacy_vision(self, state_dict):
        """Copy only ``vision.*`` (or an exact bare vision state dictionary).

        Full legacy PreAction dictionaries are permitted as the source, but
        every ``head.*`` value is ignored. No legacy head conversion exists.
        ``load_sequence_checkpoint`` rejects legacy model checkpoints outright.
        """
        if not isinstance(state_dict, Mapping):
            raise ValueError("legacy vision source must be a state dictionary")
        expected = self.vision.state_dict()
        selected = {k[len("vision."):]: v for k, v in state_dict.items()
                    if isinstance(k, str) and k.startswith("vision.")}
        if not selected and set(state_dict) == set(expected):
            selected = dict(state_dict)
        if set(selected) != set(expected):
            raise ValueError("legacy source must contain exactly the two vision convolutions")
        for key, value in selected.items():
            if (not isinstance(value, torch.Tensor) or not value.is_floating_point()
                    or value.shape != expected[key].shape or not bool(torch.isfinite(value).all())):
                raise ValueError(f"invalid legacy vision tensor: {key}")
        self.vision.load_state_dict(selected, strict=True)
        self.vision.requires_grad_(False)
        self.legacy_vision_loaded = True

    def _validate_inputs(self, rgb_mask, q_features, sequence_features):
        reference = self.initial_state.weight
        inputs = (("rgb_mask", rgb_mask, (4, 96, 96)),
                  ("q_features", q_features, (7,)),
                  ("sequence_features", sequence_features, (10, 17)))
        batch = None
        for name, value, tail in inputs:
            if not isinstance(value, torch.Tensor):
                raise TypeError(f"{name} must be a torch.Tensor")
            if value.ndim != len(tail) + 1 or tuple(value.shape[1:]) != tail or value.shape[0] < 1:
                raise ValueError(f"{name} must have shape [B,{','.join(map(str, tail))}] with B >= 1")
            if batch is None:
                batch = value.shape[0]
            if value.shape[0] != batch:
                raise ValueError("input batch sizes must agree")
            if not value.is_floating_point() or value.dtype != reference.dtype:
                raise TypeError(f"{name} must use the model floating dtype {reference.dtype}")
            if value.device != reference.device:
                raise ValueError(f"{name} must be on the model device")
            # Reject nonfinite values even in padding; masks never hide corrupt data.
            if not bool(torch.isfinite(value).all()):
                raise ValueError(f"{name} contains nonfinite values")
        _binary(rgb_mask[:, 3], "current ROI mask")
        _binary(sequence_features[:, :, 7:14], "commanded joint mask")
        _binary(sequence_features[:, :, 15:17], "valid/future step flags")

    def forward(self, rgb_mask, q_features, sequence_features):
        """Return uncalibrated prototype motion plus explicit Boolean masks.

        A false valid flag suppresses both state updates and output. A false
        future flag only suppresses downstream future eligibility: it does not
        erase an expired action's contribution to subsequent predictions.
        Uncommanded axes' delta values never enter the recurrent encoder.
        """
        self._validate_inputs(rgb_mask, q_features, sequence_features)
        valid = sequence_features[:, :, 15].bool()
        future = valid & sequence_features[:, :, 16].bool()
        axis_mask = sequence_features[:, :, 7:14].bool()
        delta = torch.where(axis_mask, sequence_features[:, :, :7],
                            torch.zeros_like(sequence_features[:, :, :7]))
        features = torch.cat((delta, sequence_features[:, :, 7:]), dim=-1)
        features = torch.where(valid[:, :, None], features, torch.zeros_like(features))
        visual = self.vision(rgb_mask[:, :3])
        context = self.spatial(torch.cat((visual, rgb_mask[:, 3:4]), dim=1))
        hidden = torch.tanh(self.initial_state(torch.cat((context, q_features), dim=1)))
        outputs = []
        for index in range(10):
            candidate = self.recurrent(features[:, index], hidden)
            hidden = torch.where(valid[:, index, None], candidate, hidden)
            motion = self.motion_head(hidden)
            outputs.append(torch.where(valid[:, index, None], motion, torch.zeros_like(motion)))
        return {"motion": torch.stack(outputs, dim=1), "valid_steps": valid,
                "future_mask": future}


def save_sequence_checkpoint(model, path, *, training_metadata=None):
    """Save only the new schema, without overwriting an existing checkpoint.

    By default the file explicitly says ``untrained_contract_test``. A future
    training caller must provide JSON metadata with training_performed=true
    to label it a trained candidate. That declaration is not an accuracy claim.
    Saving does not train or change the in-memory model's status.
    """
    if not isinstance(model, PreActionSequence):
        raise TypeError("only PreActionSequence checkpoints are supported")
    metadata = dict(model.training_metadata if training_metadata is None else training_metadata)
    if type(metadata.get("training_performed")) is not bool:
        raise ValueError("training_metadata requires Boolean training_performed")
    try:
        json.dumps(metadata, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError("training_metadata must be finite JSON data") from error
    status = "trained_candidate_unvalidated" if metadata["training_performed"] else "untrained_contract_test"
    payload = {"schema": CHECKPOINT_SCHEMA, "model_schema": MODEL_SCHEMA,
               "input_spec": dict(INPUT_SPEC), "config": {"hidden_dim": model.hidden_dim},
               "training_status": status, "training_metadata": metadata,
               "legacy_vision_loaded": model.legacy_vision_loaded,
               "forecast_scope": "ROI translation/scale only; quality not established by checkpoint loading",
               "state_dict": {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}}
    with Path(path).open("xb") as stream:
        torch.save(payload, stream)


def load_sequence_checkpoint(path):
    """Load only this schema on CPU; old endpoint-head checkpoints are rejected."""
    payload = torch.load(Path(path), map_location="cpu", weights_only=True)
    if (not isinstance(payload, dict) or payload.get("schema") != CHECKPOINT_SCHEMA
            or payload.get("model_schema") != MODEL_SCHEMA or payload.get("input_spec") != INPUT_SPEC):
        raise ValueError("not a supported sequence checkpoint; legacy endpoint checkpoints are rejected")
    metadata = payload.get("training_metadata")
    if not isinstance(metadata, dict) or type(metadata.get("training_performed")) is not bool:
        raise ValueError("sequence checkpoint is missing explicit training status")
    try:
        json.dumps(metadata, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError("sequence checkpoint training metadata must be finite JSON data") from error
    expected_status = "trained_candidate_unvalidated" if metadata["training_performed"] else "untrained_contract_test"
    if payload.get("training_status") != expected_status:
        raise ValueError("sequence checkpoint training status mismatch")
    if type(payload.get("legacy_vision_loaded")) is not bool:
        raise ValueError("sequence checkpoint vision provenance missing")
    config = payload.get("config")
    if not isinstance(config, dict) or set(config) != {"hidden_dim"}:
        raise ValueError("sequence checkpoint config mismatch")
    model = PreActionSequence(**config)
    weights = payload.get("state_dict")
    if not isinstance(weights, Mapping) or any(not isinstance(v, torch.Tensor)
            or not v.is_floating_point() or not bool(torch.isfinite(v).all()) for v in weights.values()):
        raise ValueError("sequence checkpoint has invalid state tensors")
    try:
        model.load_state_dict(weights, strict=True)
    except RuntimeError as error:
        raise ValueError("sequence checkpoint state layout mismatch; legacy heads are not supported") from error
    model.vision.requires_grad_(False)
    model.legacy_vision_loaded = payload["legacy_vision_loaded"]
    model.training_status = expected_status
    model.training_metadata = metadata
    model.eval()
    return model

"""Offline ACT observation/action boundary; no robot, network or SDK imports.

Images enter as RGB uint8 HWC. Saved LeRobot processors own normalization.
Actions leave as six absolute degrees and gripper width metres, in that order.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path

DEFAULT_PROFILE = Path(__file__).resolve().parents[3] / "deployment/act_launch_contact_repair.json"
ACTION_NAMES = tuple([f"joint_{i}.deg" for i in range(1, 7)] + ["gripper.m"])
ACTION_UNITS = ("deg",) * 6 + ("m",)
CAMERAS = ("global", "wrist")


def current_profile(path=DEFAULT_PROFILE):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def load_processors(checkpoint=None, device="cpu"):
    """Load two saved pipelines locally; never instantiate a robot or policy."""
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies import make_pre_post_processors
    checkpoint = Path(checkpoint or current_profile()["checkpoint"]).resolve()
    cfg = PreTrainedConfig.from_pretrained(
        checkpoint, local_files_only=True,
        cli_overrides=[f"--device={device}", "--use_amp=false", "--push_to_hub=false",
                       "--pretrained_backbone_weights=null"],
    )
    if cfg.type != "act" or cfg.n_obs_steps != 1:
        raise ValueError("Only this saved single-observation ACT contract is supported")
    pre, post = make_pre_post_processors(
        policy_cfg=cfg, pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": device},
                                "normalizer_processor": {"device": device}},
        postprocessor_overrides={"device_processor": {"device": "cpu"},
                                 "unnormalizer_processor": {"device": device}},
    )
    return cfg, pre, post


class ObservationBuilder:
    def __init__(self, checkpoint=None, device="cpu"):
        self.config, self.preprocessor, _ = load_processors(checkpoint, device)

    def raw_observation(self, images, state):
        import numpy as np
        import torch
        if set(images) != set(CAMERAS):
            raise ValueError("Exactly global and wrist RGB cameras are required")
        state_tensor = torch.as_tensor(state, dtype=torch.float32).clone()
        if state_tensor.shape != (7,) or not torch.isfinite(state_tensor).all():
            raise ValueError("state must contain seven finite physical values")
        result = {"observation.state": state_tensor}
        for role in CAMERAS:
            value = np.asarray(images[role])
            if value.dtype != np.uint8 or value.shape != (480, 640, 3):
                raise ValueError(f"{role}: expected RGB uint8 HWC 480x640x3; no implicit resize")
            result[f"observation.images.{role}"] = (
                torch.from_numpy(value.copy()).permute(2, 0, 1).contiguous().to(torch.float32) / 255.0
            )
        return result

    def build(self, images, state):
        return self.preprocessor(self.raw_observation(images, state))

    def reset(self):
        self.preprocessor.reset()


@dataclass(frozen=True)
class ActionResult:
    values: object
    clipped_mask: object


class ActionPostprocessor:
    def __init__(self, checkpoint=None, device="cpu", bounds=None):
        self.config, _, self.postprocessor = load_processors(checkpoint, device)
        # Reviewed deployment envelope is explicit metadata, not learned data limits.
        self.bounds = bounds if bounds is not None else current_profile()["policy"]["limits"]["bounds"]
        import torch
        bounds_tensor = torch.as_tensor(self.bounds, dtype=torch.float64)
        if (bounds_tensor.shape != (7, 2) or not torch.isfinite(bounds_tensor).all()
                or not (bounds_tensor[:, 0] < bounds_tensor[:, 1]).all()):
            raise ValueError("bounds must be seven finite increasing intervals")

    @staticmethod
    def _tensor(action):
        import torch
        result = torch.as_tensor(action).detach()
        if not result.is_floating_point():
            raise ValueError("action must be floating point")
        if result.ndim < 1 or result.shape[-1] != 7 or not torch.isfinite(result).all():
            raise ValueError("action must end in seven finite values")
        return result

    def normalized_to_physical(self, normalized):
        """Numerical alignment path: saved unnormalizer exactly once, no clipping."""
        return self.postprocessor(self._tensor(normalized).clone())

    def denormalize(self, normalized):
        return self.normalized_to_physical(normalized)

    def validate_physical(self, physical, *, clip=False):
        import torch
        values = self._tensor(physical).clone()
        bounds = torch.as_tensor(self.bounds, device=values.device, dtype=values.dtype)
        mask = (values < bounds[:, 0]) | (values > bounds[:, 1])
        if mask.any() and not clip:
            raise ValueError("physical action outside explicit deployment bounds")
        if clip:
            values = values.clamp(bounds[:, 0], bounds[:, 1])
        return ActionResult(values, mask)

    def process(self, normalized, *, clip=False):
        return self.validate_physical(self.normalized_to_physical(normalized), clip=clip)

    def project(self, physical, *, clip=False):
        return self.validate_physical(physical, clip=clip)

    def to_q16(self, physical, *, clip=False):
        """Nearest Q16.16 code inside each physical interval (own axis unit).

        Endpoints off the integer grid round inward; their error can approach
        one LSB. An interval with no representable code is rejected.
        """
        import torch
        values = self.validate_physical(physical, clip=clip).values.to(torch.float64)
        bounds = torch.as_tensor(self.bounds, device=values.device, dtype=torch.float64)
        lower_codes = torch.ceil(bounds[:, 0] * 65536)
        upper_codes = torch.floor(bounds[:, 1] * 65536)
        if (not torch.isfinite(lower_codes).all() or not torch.isfinite(upper_codes).all()
                or (lower_codes > upper_codes).any()
                or (lower_codes > 2**31 - 1).any() or (upper_codes < -(2**31)).any()):
            raise ValueError("physical interval contains no representable Q16.16 int32 code")
        encoded = torch.round(values * 65536)
        if ((encoded < -(2**31)) | (encoded > 2**31 - 1)).any():
            raise ValueError("Q16.16 signed int32 overflow")
        encoded = torch.maximum(lower_codes, torch.minimum(upper_codes, encoded))
        return encoded.to(torch.int32)

    def reset(self):
        self.postprocessor.reset()


class ACTQueueAdapter:
    """Small production queue adapter, independent of official select_action queue.

    Timing/staleness remains the generic ChunkScheduler's responsibility. This
    class performs no sleep, clock rewriting, blending, transport or movement.
    """
    def __init__(self, policy, observation_builder, postprocessor, n_action_steps=30,
                 *, enforce_bounds=True, clip=False):
        if type(n_action_steps) is not int or not 1 <= n_action_steps <= policy.config.chunk_size:
            raise ValueError("n_action_steps must fit the saved chunk")
        if policy.config.temporal_ensemble_coeff is not None:
            raise ValueError("Queue adapter requires temporal ensemble disabled")
        self.policy, self.builder, self.post = policy, observation_builder, postprocessor
        self.n_action_steps, self.enforce_bounds, self.clip = n_action_steps, enforce_bounds, clip
        self.queue = deque()

    def select_action(self, images, state):
        import torch
        with torch.inference_mode():
            if not self.queue:
                chunk = self.policy.predict_action_chunk(self.builder.build(images, state))
                self.queue.extend(chunk[:, :self.n_action_steps].transpose(0, 1))
            normalized = self.queue.popleft()
            return (self.post.process(normalized, clip=self.clip).values if self.enforce_bounds
                    else self.post.normalized_to_physical(normalized))

    def reset(self):
        self.queue.clear()
        self.policy.reset()
        self.builder.reset()
        self.post.reset()

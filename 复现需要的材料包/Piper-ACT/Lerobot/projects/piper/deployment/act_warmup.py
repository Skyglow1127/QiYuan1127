"""Warm the saved ACT pipeline on disposable synthetic inputs, without hardware.

Call this after strict checkpoint loading and before constructing a Robot. The
returned actions are validated and discarded; every pass computes a new chunk,
and all policy/processor state is reset before returning or propagating errors.
"""

from collections.abc import Mapping
import math
import time


def _feature_shape(feature, name):
    shape = tuple(feature.shape)
    if not shape or any(type(size) is not int or size < 1 for size in shape):
        raise ValueError(f"Invalid ACT warmup feature shape: {name}")
    return shape


def warm_up_act(policy, preprocessor, postprocessor, *, passes=3, clock=time.monotonic):
    """Return per-pass pipeline timings; never expose a synthetic action to TX."""
    if type(passes) is not int or passes < 1:
        raise ValueError("ACT warmup passes must be a positive integer")

    import torch

    durations_ms = []
    try:
        features = policy.config.input_features
        if not isinstance(features, Mapping) or not features:
            raise ValueError("ACT warmup requires saved input features")
        shapes = {}
        for name, feature in features.items():
            shape = _feature_shape(feature, name)
            if name == "observation.state":
                valid = len(shape) == 1
            elif isinstance(name, str) and name.startswith("observation.images."):
                valid = len(shape) == 3 and shape[0] == 3
            else:
                valid = False
            if not valid:
                raise ValueError(f"Unsupported ACT warmup feature: {name}")
            shapes[name] = (1, *shape)
        if "observation.state" not in shapes:
            raise ValueError("ACT warmup requires the saved observation.state feature")
        output_shape = (1, *_feature_shape(policy.config.output_features["action"], "action"))

        def require_action(action):
            if (not isinstance(action, torch.Tensor) or tuple(action.shape) != output_shape
                    or not bool(torch.isfinite(action).all())):
                raise RuntimeError("ACT warmup produced an invalid or non-finite action")

        with torch.inference_mode():
            for _ in range(passes):
                # Reset before every select_action so cached targets cannot
                # turn a requested warmup pass into a cheap queue pop.
                policy.reset()
                started = clock()
                # These already have the same batch dimension as the official
                # sync engine's prepared inputs. The saved to_batch processor
                # only adds it to 1-D state / 3-D image tensors.
                batch = {name: torch.zeros(shape, dtype=torch.float32)
                         for name, shape in shapes.items()}
                processed = preprocessor(batch)
                action = policy.select_action(processed)
                require_action(action)
                action = postprocessor(action)
                require_action(action)
                finished = clock()
                if not (math.isfinite(started) and math.isfinite(finished) and finished >= started):
                    raise RuntimeError("Invalid ACT warmup monotonic timing")
                durations_ms.append((finished - started) * 1000)
        return {
            "passes": passes,
            "duration_ms": durations_ms,
            "mean_duration_ms": sum(durations_ms) / passes,
            "max_duration_ms": max(durations_ms),
            "input_shapes": {name: list(shape) for name, shape in shapes.items()},
            "finite_actions": True,
            "synthetic_inputs_only": True,
            "warmup_actions_discarded": True,
        }
    finally:
        # Attempt every reset even if an earlier reset fails. Formal rollout
        # must never inherit a warmup chunk or processor history.
        try:
            policy.reset()
        finally:
            try:
                preprocessor.reset()
            finally:
                postprocessor.reset()

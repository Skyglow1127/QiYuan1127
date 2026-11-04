"""Strict boundary for the inspected pi05-local-grasp/v1 postprocessed response.

No model imports, normalization, clipping, transport, or clock inference. The
caller owns observation_at/available_at in ONE local monotonic clock domain.
Server timing is diagnostic only. Axis order is attested by checkpoint config,
not supplied by this service, and must be passed explicitly by the caller.
"""
from dataclasses import dataclass
import numpy as np
from .scheduler import TimedChunk

AXES = tuple(f'joint_{i}.deg' for i in range(1, 7)) + ('gripper.m',)
UNITS = ('degree',)*6 + ('metre',)
REPRESENTATION = 'absolute joint targets and gripper width'
PROTOCOL = 'pi05-local-grasp/v1'


@dataclass(frozen=True)
class Pi05Contract:
    axis_order: tuple[str, ...]
    sample_hz: float
    use_relative_actions: bool
    postprocessed: bool
    model_id: str

    def __post_init__(self):
        if tuple(self.axis_order) != AXES:
            raise ValueError('checkpoint axis order must be joint_1..6.deg, gripper.m')
        if isinstance(self.sample_hz, bool) or self.sample_hz != 20:
            raise ValueError('this checkpoint requires explicit 20 Hz dataset sampling')
        if self.use_relative_actions is not False or self.postprocessed is not True:
            raise ValueError('only already postprocessed absolute actions are supported')
        if not isinstance(self.model_id, str) or not self.model_id:
            raise ValueError('expected model_id required')
        object.__setattr__(self, 'axis_order', tuple(self.axis_order))

    def adapt(self, response, *, sequence, observation_at, available_at,
              request_id, input_sha256):
        for key, expected in (('protocol_version', PROTOCOL), ('model_id', self.model_id),
                              ('request_id', request_id), ('input_sha256', input_sha256),
                              ('representation', REPRESENTATION)):
            if not isinstance(expected, str) or not expected or response.get(key) != expected:
                raise ValueError(f'pi05 response {key} mismatch')
        if response.get('action_shape') != [50, 7] or tuple(response.get('units', ())) != UNITS:
            raise ValueError('pi05 requires declared shape [50,7] and degree/metre units')
        if 'axis_order' in response and tuple(response['axis_order']) != self.axis_order:
            raise ValueError('response axis order differs from checkpoint')
        values = response['actions']
        if isinstance(values, (list, tuple)) and any(isinstance(v, (bool, str))
                for row in values if isinstance(row, (list, tuple)) for v in row):
            raise ValueError('pi05 actions cannot contain bool/string values')
        if any(isinstance(t, bool) for t in (observation_at,available_at)):
            raise ValueError('timestamps must be local monotonic numeric values')
        raw = np.asarray(values)
        if raw.shape != (50, 7) or raw.dtype.kind not in 'fiu' or not np.isfinite(raw).all():
            raise ValueError('pi05 actions must be finite numeric 50x7 (no bool/string)')
        # Conversion is a copy, not another postprocessor. Do not use ACT stats.
        return TimedChunk(sequence, observation_at, available_at, self.sample_hz, raw)

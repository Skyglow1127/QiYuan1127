"""Offline ACT-to-PREACTION sequence input contract, not a trained sequence model.

This module only validates and encodes an already predicted action chunk. It
neither predicts an image nor changes any target or its deadline. Absolute times
are Python floats (double precision); only the model-input features are rounded
to float32. No future measured state or teacher action is an input.

All timestamps must share one monotonic clock domain. A historical replay may
use a virtual completion time: historical prediction_started_at plus measured
inference duration. Such a time describes replay scheduling, not a new physical
observation. The caller must document whether its clock is live or replayed;
this numeric contract cannot establish clock provenance by itself.
"""

from collections.abc import Mapping
from dataclasses import asdict, is_dataclass
import json
import math
import re
import struct


SCHEMA = 'd1-preaction-act-sequence-v1'
N_STEPS = 10
N_JOINTS = 7
UNITS = ['degree'] * 6 + ['uncalibrated_sdk_raw']
DELTA_SCALE = [5.0] * 6 + [20.0]
Q_SCALE = [180.0] * 6 + [100.0]
MAX_Q_AGE_S = 0.6
FEATURE_COLUMNS = (
    [f'target_minus_current_q_scaled_j{i}' for i in range(N_JOINTS)]
    + [f'commanded_joint_mask_j{i}' for i in range(N_JOINTS)]
    + ['target_time_from_prediction_start_s', 'valid_step', 'future_at_completion']
)


def _number(value, name):
    # JSON numbers only: in particular, True is not an angle or a timestamp.
    if type(value) not in (int, float):
        raise ValueError(f'{name}: expected a finite number, not bool')
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f'{name}: expected a finite number')
    return value


def _items(value, length, name):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f'{name}: expected length {length}')
    return list(value)


def _vector(value, length, name):
    return [_number(x, f'{name}[{i}]')
            for i, x in enumerate(_items(value, length, name))]


def _bool_vector(value, length, name):
    values = _items(value, length, name)
    if any(type(x) is not bool for x in values):
        raise ValueError(f'{name}: every mask element must be an explicit bool')
    return values


def _float32(value, name):
    try:
        result = struct.unpack('!f', struct.pack('!f', value))[0]
    except (OverflowError, struct.error) as exc:
        raise ValueError(f'{name}: not representable as finite float32') from exc
    if not math.isfinite(result):
        raise ValueError(f'{name}: not representable as finite float32')
    return result


def _identifier(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{name}: expected a nonempty string')
    return value


def _chunk_dict(chunk):
    if is_dataclass(chunk) and not isinstance(chunk, type):
        chunk = asdict(chunk)
    if not isinstance(chunk, Mapping):
        raise ValueError('chunk: expected an ActionChunk dataclass or mapping')
    required = {'commands', 'target_times', 'observation_id', 'model_sha256',
                'action_type', 'coordinate_frame'}
    if not required.issubset(chunk) or set(chunk) - (required | {'units'}):
        raise ValueError('chunk: missing or unsupported contract fields')
    if (chunk['action_type'] != 'sdk_joint_position'
            or chunk['coordinate_frame'] != 'sdk_native'):
        raise ValueError('chunk: expected absolute SDK joint positions in sdk_native')
    # ActionChunk's established SDK contract implies these units. If units are
    # explicitly supplied, they must agree; J6 is never interpreted as degrees.
    if 'units' in chunk and chunk['units'] != UNITS:
        raise ValueError('chunk: unsupported position units')
    observation_id = _identifier(chunk['observation_id'], 'chunk.observation_id')
    sha = chunk['model_sha256']
    if not isinstance(sha, str) or re.fullmatch(r'[0-9a-fA-F]{64}', sha) is None:
        raise ValueError('chunk.model_sha256: expected 64 hexadecimal characters')
    commands = [_vector(row, N_JOINTS, f'chunk.commands[{i}]')
                for i, row in enumerate(_items(chunk['commands'], N_STEPS,
                                               'chunk.commands'))]
    times = [float(x) for x in _vector(chunk['target_times'], N_STEPS,
                                       'chunk.target_times')]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('chunk.target_times: must be strictly increasing')
    return dict(commands=commands, target_times=times,
                observation_id=observation_id, model_sha256=sha,
                action_type='sdk_joint_position', coordinate_frame='sdk_native',
                units=list(UNITS))


def build_act_sequence(chunk, *, q_current, observation_id, q_observed_at,
                       prediction_started_at, prediction_completed_at,
                       valid_steps, commanded_joint_mask):
    """Return a JSON-compatible, auditable 10-step sequence input packet.

    ``valid_steps`` is ten explicit booleans describing the ACT predictions
    themselves. Do not derive it from a historical teacher's ``is_pad``: the
    teacher's available label horizon does not establish prediction validity.
    ``commanded_joint_mask`` must be 10x7 explicit booleans, with no broadcasting.
    For ACT candidates this describes proposed command coverage, not evidence
    that a command was sent. False means no target update is specified for that
    axis, not measured zero motion or a confirmed hold. All raw values remain in ``chunk`` even when a
    step is invalid, uncommanded or past its deadline.

    ``q_observed_at`` must be the time this actual observation became available,
    strictly before prediction start and at most 0.6 seconds old. Future actual
    feedback is not accepted. Completion may equal start, but cannot precede it.
    Target deadlines may already have passed, including raw index zero; these
    are recorded rather than shifted or used to reject the whole chunk.

    ``sequence_features`` has 17 float32 columns: seven (target-current_q)
    values scaled by [5]*6+[20], seven command-mask values, seconds relative to
    prediction start, original step validity, and deadline strictly after
    completion. Invalid/uncommanded delta columns are zero. Expired but valid
    deltas remain present; ``valid_future_steps`` is the separate usable-future
    mask. ``q_features`` uses [180]*6+[100]. No value is clipped.

    The two original masks, completion mask and their conjunction remain bool
    lists. Their float32 feature copies do not replace the semantic masks.
    This is an input interface only, with no trained sequence predictor.
    """
    raw = _chunk_dict(chunk)
    observation_id = _identifier(observation_id, 'observation_id')
    if raw['observation_id'] != observation_id:
        raise ValueError('observation_id: does not match chunk provenance')
    q = _vector(q_current, N_JOINTS, 'q_current')
    observed = float(_number(q_observed_at, 'q_observed_at'))
    started = float(_number(prediction_started_at, 'prediction_started_at'))
    completed = float(_number(prediction_completed_at, 'prediction_completed_at'))
    if not observed < started:
        raise ValueError('q_observed_at: must be strictly before prediction start')
    q_age = started - observed
    if not math.isfinite(q_age) or q_age > MAX_Q_AGE_S:
        raise ValueError('q_observed_at: observation exceeds 0.6 second age limit')
    if completed < started:
        raise ValueError('prediction_completed_at: precedes prediction start')
    valid = _bool_vector(valid_steps, N_STEPS, 'valid_steps')
    masks = [_bool_vector(row, N_JOINTS, f'commanded_joint_mask[{i}]')
             for i, row in enumerate(_items(commanded_joint_mask, N_STEPS,
                                            'commanded_joint_mask'))]
    future = [t > completed for t in raw['target_times']]
    valid_future = [v and f for v, f in zip(valid, future)]
    features = []
    for i, (target, target_t) in enumerate(zip(raw['commands'], raw['target_times'])):
        delta = [(target[j] - q[j]) / DELTA_SCALE[j]
                 if valid[i] and masks[i][j] else 0.0
                 for j in range(N_JOINTS)]
        row = delta + [float(x) for x in masks[i]]
        row += [target_t - started, float(valid[i]), float(future[i])]
        features.append([_float32(x, f'sequence_features[{i}][{j}]')
                         for j, x in enumerate(row)])
    q_features = [_float32(x / scale, f'q_features[{i}]')
                  for i, (x, scale) in enumerate(zip(q, Q_SCALE))]
    return dict(
        schema=SCHEMA, chunk=raw, observation_id=observation_id, q_current=q,
        q_observed_at=observed, prediction_started_at=started,
        prediction_completed_at=completed, q_age_s=q_age,
        valid_steps=valid, commanded_joint_mask=masks,
        future_at_completion_mask=future, valid_future_steps=valid_future,
        valid_future_count=sum(valid_future),
        sequence_features=features, q_features=q_features,
        feature_columns=list(FEATURE_COLUMNS), feature_dtype='float32',
        feature_shape=[N_STEPS, len(FEATURE_COLUMNS)],
        delta_scale=list(DELTA_SCALE), q_scale=list(Q_SCALE),
        absolute_time_dtype='float64', max_q_age_s=MAX_Q_AGE_S,
        clock_semantics='one caller-supplied monotonic domain; replay completion may be start plus measured inference duration',
        valid_steps_semantics='ACT prediction validity; independent of teacher labels or is_pad',
        commanded_joint_mask_semantics='proposed ACT command coverage only; not evidence of a sent command',
        inactive_axis_semantics='no target update specified; not observed zero motion or a confirmed hold',
        expired_step_semantics='raw deadline at or before completion; retained without retiming',
        mask_semantics='valid_future_steps = valid_steps AND future_at_completion_mask; command masks remain per axis',
        not_live_eligible=True, no_model_prediction=True, robot_commands=0,
    )


def validate_sequence_packet(packet):
    """Rebuild and verify the entire packet, returning its canonical copy.

    Derived feature/mask/metadata edits, unexpected fields and numeric booleans
    are rejected. This detects internal inconsistency, not forged provenance:
    authenticating the observation and model hash belongs to the caller.
    """
    if not isinstance(packet, Mapping):
        raise ValueError('packet: expected a mapping')
    try:
        rebuilt = build_act_sequence(
            packet['chunk'], q_current=packet['q_current'],
            observation_id=packet['observation_id'],
            q_observed_at=packet['q_observed_at'],
            prediction_started_at=packet['prediction_started_at'],
            prediction_completed_at=packet['prediction_completed_at'],
            valid_steps=packet['valid_steps'],
            commanded_joint_mask=packet['commanded_joint_mask'],
        )
        # JSON comparison distinguishes 0/False and 1/True in derived fields;
        # ordinary dict/list equality would silently treat them as equivalent.
        actual = json.dumps(dict(packet), sort_keys=True, allow_nan=False,
                            separators=(',', ':'))
        expected = json.dumps(rebuilt, sort_keys=True, allow_nan=False,
                              separators=(',', ':'))
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('packet: missing or non-JSON contract fields') from exc
    if actual != expected:
        raise ValueError('packet: derived fields or metadata do not match raw inputs')
    return rebuilt

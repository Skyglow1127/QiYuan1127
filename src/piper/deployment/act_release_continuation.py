"""Opt-in continuation of an ACT release already proposed by the model.

The joint window identifies the recorded task phase, not table contact or a
collision-free pose. This module never creates a target or proves placement.
"""
import math

# Canonical feature names only; config validation does not load LeRobot/torch.
ACTION_KEYS = tuple(f'joint_{i}.deg' for i in range(1, 7)) + ('gripper.m',)


RELEASE_CONTINUATION = {
    'extra_steps': 20,
    'trigger_open_m': .055,
    'released_open_m': .060,
    'min_held_width_m': .035,
    'settle_s': .15,
    'max_hold_s': 2.,
    'placement_joint_bounds_deg': [
        [25., 38.], [118., 133.], [-105., -75.],
        [-15., 15.], [55., 70.], [-75., -40.]],
}


def validate_release_continuation(config, *, bounds):
    label = 'policy_release_chunk_extension'
    if not isinstance(config, dict) or set(config) != set(RELEASE_CONTINUATION):
        raise ValueError(label + ' requires the documented fields')
    if type(config['extra_steps']) is not int or not 1 <= config['extra_steps'] <= 20:
        raise ValueError(label + ' extra_steps must be an integer in [1, 20]')
    for key, low, high in (('settle_s', .05, .3), ('max_hold_s', .5, 2.)):
        value = config[key]
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(label + f' {key} must be a finite number in [{low}, {high}]')
    widths = [config[k] for k in ('min_held_width_m', 'trigger_open_m', 'released_open_m')]
    if (any(type(v) not in (int, float) or not math.isfinite(v) for v in widths)
            or not bounds[6][0] <= widths[0] < widths[1] < widths[2] <= bounds[6][1]):
        raise ValueError(label + ' requires ordered finite gripper thresholds within the working range')
    window = config['placement_joint_bounds_deg']
    if not isinstance(window, (list, tuple)) or len(window) != 6:
        raise ValueError(label + ' requires six placement joint intervals')
    for interval, (low, high) in zip(window, bounds[:6], strict=True):
        if (not isinstance(interval, (list, tuple)) or len(interval) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in interval)
                or not low <= interval[0] < interval[1] <= high):
            raise ValueError(label + ' placement window exceeds the working range')
    return config


def release_is_pending(config, *, raw_action, state):
    """Only an acknowledged opening proposal in the placement phase qualifies."""
    if raw_action is None:
        return False
    values = [state[k] for k in ACTION_KEYS] + [raw_action['gripper.pos']]
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        return False
    width, requested = state['gripper.m'], raw_action['gripper.pos']
    return (config['min_held_width_m'] <= width < config['released_open_m']
        and requested >= config['trigger_open_m'] and requested > width + .003
        and all(lo <= state[key] <= hi for key, (lo, hi) in
            zip(ACTION_KEYS[:6], config['placement_joint_bounds_deg'], strict=True)))


def gate_release_motion(config, *, action, state, previous_target, committed, now):
    """Preview a bounded joint hold; gripper always remains the ACT proposal.

    Return candidate state separately: commit it only after the original guarded
    send succeeds. Opening width confirms clearance, not block/table contact.
    """
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in action.values()):
        raise ValueError('Release gate requires finite ACT proposals')
    phase = dict(committed) if committed is not None else None
    raw = {'gripper.pos': action['gripper.m']}
    if phase is None and release_is_pending(config, raw_action=raw, state=state):
        phase = {'started_at': now, 'anchor': tuple(previous_target[:6]), 'cleared_at': None}
    if phase is None:
        return action, None, 'inactive'
    if state['gripper.m'] >= config['released_open_m']:
        if phase['cleared_at'] is None:
            phase['cleared_at'] = now
        if now-phase['cleared_at'] >= config['settle_s']:
            return action, None, 'released'
    else:
        phase['cleared_at'] = None
    if now-phase['started_at'] >= config['max_hold_s']:
        raise ValueError(f"Release opening not confirmed within{config['max_hold_s']:g}s; retreat remains blocked")
    held = dict(action)
    held.update(zip(ACTION_KEYS[:6], phase['anchor'], strict=True))
    return held, phase, 'settling' if phase['cleared_at'] is not None else 'opening'

"""Light joint-only proposal smoothing, before the original guarded projection."""
from .guard import GuardError, vector


def validate_joint_smoothing_alpha(alpha):
    if type(alpha) not in (int, float) or not .5 <= alpha <= 1.:
        raise GuardError('policy_joint_smoothing_alpha must be a finite JSON number in [0.5, 1]')
    return alpha


def smooth_joint_targets(action, *, previous_target, alpha):
    """Blend with the last committed target; gripper and timestamps are untouched.

    No separate filter state is retained: a failed send cannot advance it.
    The result still requires feedback/rate/bounds/CAN-grid intersection and
    both original guards. It is not a final directly sendable target.
    """
    alpha = validate_joint_smoothing_alpha(alpha)
    desired, previous = vector(action), vector(previous_target)
    return tuple(alpha*new+(1-alpha)*old for new,old in zip(desired[:6],previous[:6]))+(desired[6],)

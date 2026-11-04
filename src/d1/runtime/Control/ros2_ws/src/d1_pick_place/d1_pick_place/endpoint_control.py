"""Pure validation for a reviewed fixed-pose endpoint retry policy."""
import math


ENDPOINT_RETRY_POLICY = dict(mode='retry_approved_target_v1', retry_interval_s=1.,
                             max_seek_s=30., stall_timeout_s=5., min_progress_deg=.2)


def checked_endpoint_policy(value):
    """Return an independent validated policy, or None for passive settling."""
    if value is None:
        return None
    if not isinstance(value, dict) or set(value) != set(ENDPOINT_RETRY_POLICY):
        raise ValueError('Malformed approved endpoint policy')
    if value['mode'] != ENDPOINT_RETRY_POLICY['mode']:
        raise ValueError('Unknown approved endpoint policy mode')
    result = dict(value)
    for key in set(result) - {'mode'}:
        number = result[key]
        if (isinstance(number, bool) or not isinstance(number, (int, float))
                or not math.isfinite(number) or number <= 0):
            raise ValueError('Endpoint policy values must be positive finite numbers: ' + key)
        result[key] = float(number)
    if not result['retry_interval_s'] <= result['stall_timeout_s'] <= result['max_seek_s']:
        raise ValueError('Endpoint retry interval must not exceed stall or seek timeout')
    return result

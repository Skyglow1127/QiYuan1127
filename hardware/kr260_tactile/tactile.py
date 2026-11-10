"""Digital contact interpretation; contains no robot motion commands."""
import math


class Debouncer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.candidate = self.since = self.stable = None

    def update(self, level, t_s, duration_ms):
        if level is None:
            self.reset()
            return None
        if level != self.candidate:
            self.candidate, self.since = level, t_s
        if (t_s - self.since) * 1000 + 1e-7 >= duration_ms:
            self.stable = level
        return self.stable


def integration_view(payload):
    """Fail closed for consumers, including simulated/stale/disconnected sources."""
    if (not isinstance(payload, dict) or payload.get('schema') != 'kr260-tactile/v1'
            or payload.get('source') != 'real' or payload.get('valid') is not True
            or payload.get('through_pl_ps') is not True or payload.get('fresh') is not True
            or payload.get('sensor_connected') is not True):
        return {'decision': 'HOLD', 'stop_closing': True, 'both_contact': None,
                'reason': '触觉数据无效、未连接或尚未核实'}
    age, limit = payload.get('receive_age_ms'), payload.get('stale_ms')
    if (type(age) not in (int, float) or not math.isfinite(age) or age < 0
            or type(limit) not in (int, float) or not math.isfinite(limit)
            or not 0 < limit <= 5000 or age > limit):
        return {'decision': 'HOLD', 'stop_closing': True, 'both_contact': None,
                'reason': '采样时间无效或过期'}
    channels = payload.get('channels')
    if not isinstance(channels, list) or len(channels) != 2 or any(not isinstance(c, dict) for c in channels):
        return {'decision': 'HOLD', 'stop_closing': True, 'both_contact': None, 'reason': '通道结构无效'}
    active = [c for c in channels if c.get('enabled', True) is True]
    if (not active or any(c.get('valid') is not True or type(c.get('contact')) is not bool
                         or type(c.get('raw_contact')) is not bool for c in active)):
        return {'decision': 'HOLD', 'stop_closing': True, 'both_contact': None,
                'reason': '启用通道的接触状态未就绪'}
    contact = [c['contact'] for c in active]
    # Raw contact stops closure immediately; stable contact is used for display/status.
    immediate = any(c.get('raw_contact') is not False for c in active)
    return {'decision': 'CONTACT' if any(contact) or immediate else 'NO_CONTACT',
            'stop_closing': bool(any(contact) or immediate), 'both_contact': all(contact) if len(active)==2 else None,
            'reason': '检测到接触阈值，不代表已抓稳' if any(contact) or immediate else '当前未越过接触阈值'}

"""Passive PiPER protocol decoding, based on the installed official pyAgxArm source.

Joint payloads: two BE signed int32 millidegrees, converted to degrees. Gripper mode is explicit;
never interpret an angle-mode value as a width. No robot SDK or transmit API.
"""
import math
import struct

MDEG_TO_DEG = 0.001


def decode(can_id, data, mapping):
    if len(data) != 8:
        return None
    if can_id == mapping.get('state_status_id'):
        return {'kind': 'status', 'part': 0, 'ctrl_mode': data[0],
                'arm_status': data[1], 'mode_feedback': data[2],
                'teach_status': data[3], 'motion_status': data[4],
                'err_code': int.from_bytes(data[6:8], 'big')}
    for kind in ('action', 'state'):
        ids = mapping[kind + '_joint_ids']
        if can_id in ids:
            return {'kind': kind, 'part': ids.index(can_id),
                    'values': [x * MDEG_TO_DEG for x in struct.unpack('>ii', data)],
                    'raw': list(struct.unpack('>ii', data))}
        if can_id == mapping[kind + '_gripper_id']:
            raw, force, status, mode_byte = struct.unpack('>ihBB', data)
            if kind == 'action':
                mode = 'width' if status in range(4) else 'angle' if status in range(4, 8) else 'unknown'
                fault = None
            else:
                mode = {0: 'width', 1: 'angle'}.get(mode_byte, 'unknown')
                fault = status & 0x3f
            value = raw * (1e-6 if mode == 'width' else MDEG_TO_DEG) if mode != 'unknown' else None
            return {'kind': kind, 'part': 3, 'values': [value], 'raw': [raw],
                    'mode': mode, 'force_n': force * 1e-3,
                    'status': status, 'mode_byte': mode_byte, 'fault_bits': fault}
    return None


def unpack_frame(packet):
    if len(packet) != 16:
        return None
    can_id, size, data = struct.unpack('=IB3x8s', packet)
    # Extended, RTR and error frames are not PiPER standard-ID data frames.
    if can_id & 0xe0000000 or size > 8:
        return None
    return can_id & 0x7ff, data[:size]


def position_target_hold(parts, now, config):
    """Assess held absolute position targets, never synthesize fresh CAN data.

    An already commanded closed gripper can be blocked by the cube: its width
    must NOT be forced equal to the target. Joint tracking, live feedback,
    enabled gripper and controller mode/faults guard the short hold instead.
    A new episode must separately require a fresh post-arm target group.
    """
    result = {'policy': 'fresh_targets', 'allowed': False, 'held_parts': [], 'source_age_ms': []}
    try:
        action = [parts[f'action_{i}'] for i in range(4)]
        state = [parts[f'state_{i}'] for i in range(4)]
        status = parts['status_0']
        ages = [(now-item['receive_monotonic'])*1000 for item in action]
        result['source_age_ms'] = ages
        result['held_parts'] = [i for i, age in enumerate(ages) if age > config['max_can_age_ms']]
        policy = config.get('position_target_hold', {})
        if not policy.get('operator_confirmed'):
            return result
        if min(ages) < 0 or max(ages) > policy['max_seconds']*1000:
            return result
        if any(not 0 <= (now-item['receive_monotonic'])*1000 <= config['max_can_age_ms']
               for item in state+[status]):
            return result
        if (status['ctrl_mode'] != 1 or status['mode_feedback'] != 1 or
            status['arm_status'] or status['err_code']):
            return result
        if (state[3].get('fault_bits') or not state[3]['status'] & 0x40 or
            action[3]['status'] not in (1, 3, 5, 7) or action[3].get('mode_byte') != 0 or
            action[3]['mode'] != state[3]['mode'] or state[3]['mode'] not in ('width', 'angle')):
            return result
        actual = [value for item in state for value in item['values']]
        targets = [value for item in action for value in item['values']]
        if len(actual) != 7 or len(targets) != 7 or any(not math.isfinite(v) for v in actual+targets):
            return result
        if max(abs(a-b) for a, b in zip(actual[:6], targets[:6], strict=True)) > policy['joint_tolerance_deg']:
            return result
        result['allowed'] = True
        if result['held_parts']:
            result['policy'] = 'held_position_targets'
    except (KeyError, TypeError, ValueError):
        pass
    return result

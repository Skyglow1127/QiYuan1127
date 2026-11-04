"""Motion settings used only by the fixed Gazebo ready posture."""
import math

FIXED_MOTION_DEFAULTS = dict(speed_deg_s=20., acceleration_deg_s2=40.,
                           gripper_speed_sdk_s=10., rate_hz=10.)


def fixed_motion_checked(values=None):
    result = dict(FIXED_MOTION_DEFAULTS)
    for key, value in (values or {}).items():
        if key not in result:
            raise ValueError('未知固定姿态速度参数：' + str(key))
        if isinstance(value, bool):
            raise ValueError('固定姿态速度参数需要正数')
        result[key] = float(value)
    if any(not math.isfinite(value) or value <= 0 for value in result.values()):
        raise ValueError('固定姿态速度与加速度必须是正有限数值')
    if result['rate_hz'] > 10:
        raise ValueError('固定姿态通信频率不能超过现有传输支持的 10 Hz')
    return result

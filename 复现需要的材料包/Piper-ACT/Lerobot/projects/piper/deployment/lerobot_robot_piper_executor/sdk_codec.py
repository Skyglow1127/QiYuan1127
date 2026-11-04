"""Offline SDK encoding only. No driver, bus, socket, factory or connect calls.

Uses the existing pyAgxArm source on PYTHONPATH. Byte output is a review artifact,
not evidence that the installed robot firmware supports the proposed mode.
"""

from dataclasses import dataclass
from functools import lru_cache
import math
import struct
import time
from .guard import vector


@dataclass(frozen=True)
class Frame:
    arbitration_id: int
    data: bytes


@lru_cache(maxsize=1)
def _target_components():
    # Loading the SDK also loads its API/driver modules. Resolve and instantiate
    # these while unarmed, rather than inside the first command's deadline.
    from pyAgxArm.protocols.can_protocol.drivers.piper.default.parser import Codec
    from pyAgxArm.protocols.can_protocol.msgs.piper.default import (
        ArmMsgJointCtrl12, ArmMsgJointCtrl34, ArmMsgJointCtrl56)
    from pyAgxArm.protocols.can_protocol.drivers.effector.agx_gripper.default.parser import Codec as GripperCodec
    from pyAgxArm.protocols.can_protocol.msgs.effector.agx_gripper.default import ArmMsgGripperCtrl
    return (Codec(), GripperCodec(), ArmMsgJointCtrl12, ArmMsgJointCtrl34,
            ArmMsgJointCtrl56, ArmMsgGripperCtrl)


def prepare_target_codec(*, clock=time.monotonic):
    """Exercise all four target encoders offline and discard the resulting bytes."""
    expected = (Frame(0x155, bytes(8)), Frame(0x156, bytes(8)),
                Frame(0x157, bytes(8)), Frame(0x159, struct.pack(">iHBB", 10000, 1000, 1, 0)))
    durations = []
    for _ in range(2):
        started = clock()
        frames = encode_target((0.,) * 6 + (.01,), gripper_force_n=1.)
        finished = clock()
        if frames != expected:
            raise RuntimeError("Target codec preparation produced unexpected CAN bytes")
        if not (math.isfinite(started) and math.isfinite(finished) and finished >= started):
            raise RuntimeError("Invalid target codec preparation clock")
        durations.append((finished-started)*1000)
    return {"passes": 2, "duration_ms": durations,
            "frames_discarded": True, "hardware_io": False,
            "frame_ids": [frame.arbitration_id for frame in expected]}


def encode_j_position_speed(*, speed_percent: int, installation_pos: int) -> Frame:
    """MotionCtrl2=0x151, CAN/J/POS_VEL, explicitly never JS/MIT.

    Speed must be reviewed (1..10% accepted by this preparation layer).
    Installation position must be supplied; 0=unchanged, 1=horizontal, 2=left,
    3=right. An explicit 0 is not a confirmation of the physical installation.
    This function DOES NOT change the robot mode.
    """
    if type(speed_percent) is not int or not 1 <= speed_percent <= 10:
        raise ValueError("Reviewed low speed 1..10 percent required")
    if installation_pos not in (0, 1, 2, 3):
        raise ValueError("Explicit installation position required")
    from pyAgxArm.protocols.can_protocol.drivers.piper.default.parser import Codec
    from pyAgxArm.protocols.can_protocol.msgs.piper.default import ArmMsgModeCtrl
    msg = ArmMsgModeCtrl(ctrl_mode=0x01, move_mode=0x01,
        move_spd_rate_ctrl=speed_percent, mit_mode=0x00,
        residence_time=0, installation_pos=installation_pos)
    return Frame(0x151, bytes(Codec().encode_151_mode_ctrl(msg)))


def encode_target(action, *, gripper_force_n: float) -> tuple[Frame, ...]:
    """Six absolute degree targets plus signed width metres; no limits guessed.

    Caller must validate with ActionGuard first. Gripper is width/enable and
    set_zero=0; its force is an explicit deployment parameter, absent in data.
    """
    values = vector(action)
    if not math.isfinite(gripper_force_n) or not 0 < gripper_force_n <= 65.535:
        raise ValueError("Explicit finite positive encodable gripper force required")
    joints = tuple(round(v * 1000) for v in values[:6])
    width = round(values[6] * 1_000_000)
    if any(not -(2**31) <= v < 2**31 for v in (*joints, width)):
        raise ValueError("Target outside signed int32 protocol representation")
    codec, gripper_codec, ArmMsgJointCtrl12, ArmMsgJointCtrl34, ArmMsgJointCtrl56, ArmMsgGripperCtrl = _target_components()
    return (
        Frame(0x155, bytes(codec.encode_155_joint_ctrl_12(ArmMsgJointCtrl12(*joints[:2])))),
        Frame(0x156, bytes(codec.encode_156_joint_ctrl_34(ArmMsgJointCtrl34(*joints[2:4])))),
        Frame(0x157, bytes(codec.encode_157_joint_ctrl_56(ArmMsgJointCtrl56(*joints[4:])))),
        Frame(0x159, bytes(gripper_codec.encode_159_gripper_ctrl(ArmMsgGripperCtrl(
            value=width, force=round(gripper_force_n * 1000), status_code=0x01, set_zero=0x00)))),
    )

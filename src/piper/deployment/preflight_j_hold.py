#!/usr/bin/env python3
"""J/POS_VEL 5% recovery or reviewed enabled-arm hold; check-only default.

No ACT/model action, reset, homing or calibration is sent. An explicit reviewed
option enables the empty gripper at its freshly measured width and 1 N.
Legacy-firmware target latching is not proven: physical support is required
throughout the short enable test and any fault-triggered disable.
"""

import argparse
import fcntl
import json
import math
from pathlib import Path
import select
import signal
import socket
import struct
import time

from reset_piper_once import (ResetCheckError, ResetMonitor, load_helpers,
    receive_available, verify_endpoint, MOTOR_IDS, POSE_IDS, STATUS_ID)

MODE = bytes.fromhex("0101050000000000")  # CAN/J/5%/POS_VEL/no installation change
JOINT_TARGET_IDS = (0x155, 0x156, 0x157)


class FatalCheckError(ResetCheckError):
    pass


def mode_payload(speed_percent=5):
    """Return only CAN/J/POS_VEL, speed 1..10%, with installation unchanged."""
    if type(speed_percent) is not int or not 1 <= speed_percent <= 10:
        raise ResetCheckError("Reviewed takeover speed must be an integer in [1, 10]")
    return bytes((1, 1, speed_percent, 0, 0, 0, 0, 0))


def takeover_speed(review):
    """Cold enable/recovery stays at 5%; another speed requires enabled takeover."""
    speed = review.get("takeover_speed_percent", 5)
    mode_payload(speed)
    if speed != 5 and (review.get("reuse_enabled") is not True or "initial_target_deg" in review):
        raise ResetCheckError("Non-5% speed requires enabled takeover without initial-target recovery")
    return speed


def check_fresh_faults(monitor, *, now):
    """Inspect each fresh group immediately; do not wait for a full cohort."""
    for can_id in MOTOR_IDS:
        part = monitor.parts.get(can_id)
        if part is not None and 0 <= now-part[1] < .2 and part[0][5] & 0xbf:
            raise FatalCheckError(f"Fresh motor {can_id:#x} fault bits={part[0][5] & 0xbf:#x}")
    part = monitor.parts.get(STATUS_ID)
    if part is not None and 0 <= now-part[1] < .2:
        data = part[0]
        if data[1] or int.from_bytes(data[6:8], "big"):
            raise FatalCheckError("Fresh controller fault in 0x2A1")
    part = monitor.parts.get(0x2A8)
    if part is not None and 0 <= now-part[1] < .2 and part[0][6] & 0x3f:
        raise FatalCheckError("Fresh gripper fault flags")


def sdk_codec_plan(speed_percent=5):
    """AIPC-only pure codec verification; no SDK factory/driver/connect calls."""
    expected_mode = mode_payload(speed_percent)
    from pyAgxArm.protocols.can_protocol.drivers.piper.default.parser import Codec
    from pyAgxArm.protocols.can_protocol.msgs.piper.default import (
        ArmMsgModeCtrl, ArmMsgMotorEnableDisableConfig)
    codec = Codec()
    mode = bytes(codec.encode_151_mode_ctrl(ArmMsgModeCtrl(ctrl_mode=1, move_mode=1,
        move_spd_rate_ctrl=speed_percent, mit_mode=0, residence_time=0, installation_pos=0)))
    enable = [bytes(codec.encode_471_motor_enable_disable_config(
        ArmMsgMotorEnableDisableConfig(joint_index=i, enable_flag=2))) for i in range(1, 7)]
    disable = [bytes(codec.encode_471_motor_enable_disable_config(
        ArmMsgMotorEnableDisableConfig(joint_index=i, enable_flag=1))) for i in range(1, 7)]
    if mode != expected_mode or any(d != bytes([i, 2, 0, 0, 0, 0, 0, 0])
            for i, d in enumerate(enable, 1)) or any(d != bytes([i, 1, 0, 0, 0, 0, 0, 0])
            for i, d in enumerate(disable, 1)):
        raise ResetCheckError("SDK codec differs from reviewed fixed payloads")
    return {"mode_id": "0x151", "mode": mode.hex(), "motor_id": "0x471",
        "enable_axis_1_to_6": [d.hex() for d in enable],
        "disable_axis_1_to_6": [d.hex() for d in disable], "no_gripper_target": True}


def validate_review(review):
    takeover_speed(review)
    bounds = review["joint_bounds_deg"]
    pose = review["expected_pose_deg_m"]
    if (len(bounds) != 6 or len(pose) != 7 or not all(math.isfinite(v) for v in pose)
            or any(len(b) != 2 or not all(math.isfinite(v) for v in b) or b[0] >= b[1] for b in bounds)):
        raise ResetCheckError("Explicit six finite degree bounds and seven finite expected pose values required")
    for key in ("start_joint_tolerance_deg", "start_gripper_tolerance_m", "max_deviation_deg"):
        if not math.isfinite(review[key]) or review[key] <= 0:
            raise ResetCheckError("Explicit finite positive "+key+" required")
    recovery = "initial_target_deg" in review
    if review["max_deviation_deg"] > (4.5 if recovery else 2.):
        raise ResetCheckError("Deviation cap is 4.5deg only for explicit initial-target recovery; otherwise 2deg")
    if (review.get("support_and_disable_on_fault_reviewed") is not True
            or not isinstance(review.get("approval_note"), str) or not review["approval_note"].strip()):
        raise ResetCheckError("External support, fault-disable and initialization approval must be explicit")
    check_pose(pose, review, allow_recovery_feedback=recovery)
    if recovery:
        check_initial_target(review["initial_target_deg"], pose, review)
    if "gripper_hold_force_n" in review and (
            type(review["gripper_hold_force_n"]) not in (int, float)
            or review["gripper_hold_force_n"] != 1.0
            or not 0 <= pose[6] <= .1):
        raise ResetCheckError("Optional gripper hold requires reviewed 1N and width within [0, 0.1]m")


def check_initial_target(target, current_pose, review):
    if len(target) != 6 or not all(math.isfinite(v) for v in target):
        raise ResetCheckError("Explicit six finite initial_target_deg values required")
    for axis, (q, actual, (lo, hi)) in enumerate(zip(target, current_pose[:6], review["joint_bounds_deg"], strict=True), 1):
        encoded_q = round(q*1000)/1000
        if not lo < q < hi or not lo < encoded_q < hi:
            raise FatalCheckError(f"initial target joint_{axis}.deg={q} must be strictly inside ({lo}, {hi})")
        if abs(q-actual) > 4. or abs(encoded_q-actual) > 4.:
            raise FatalCheckError(f"initial target joint_{axis} delta={q-actual}deg exceeds explicit 4deg recovery cap")
        if (axis == 2 and actual < lo and q <= actual) or (axis == 3 and actual > hi and q >= actual):
            raise FatalCheckError(f"initial target joint_{axis} does not point into the valid interval")


def check_pose(pose, review, *, anchor=None, allow_recovery_feedback=False, enforce_start=True):
    if len(pose) != 7 or not all(math.isfinite(v) for v in pose):
        raise ResetCheckError("Seven finite width-mode pose values required")
    for axis, (q, (lo, hi)) in enumerate(zip(pose[:6], review["joint_bounds_deg"], strict=True), 1):
        if not lo <= q <= hi:
            allowed = allow_recovery_feedback and "initial_target_deg" in review and (
                (axis == 2 and lo == 0 and -4. <= q < 0) or
                (axis == 3 and hi == 0 and 0 < q <= 4.))
            if not allowed:
                raise FatalCheckError(f"joint_{axis}.deg={q} outside [{lo}, {hi}]; no clamp/tolerance applied")
    expected = review["expected_pose_deg_m"]
    if enforce_start and any(abs(a-b) > review["start_joint_tolerance_deg"] for a, b in zip(pose[:6], expected[:6], strict=True)):
        raise FatalCheckError("Pose no longer matches the reviewed starting pose")
    if abs(pose[6]-expected[6]) > review["start_gripper_tolerance_m"]:
        raise FatalCheckError("Gripper feedback no longer matches reviewed width")
    if anchor is not None and any(abs(a-b) > review["max_deviation_deg"]
            for a, b in zip(pose[:6], anchor[:6], strict=True)):
        raise FatalCheckError("Joint deviation exceeded supported-check threshold")


class LimitedSender:
    """Bounded, phase-limited sends. No retry; only reviewed fixed operations."""
    def __init__(self, bus, *, clock=time.monotonic, mode=MODE):
        if (not isinstance(mode, bytes) or len(mode) != 8
                or mode[:2] != bytes((1, 1)) or not 1 <= mode[2] <= 10
                or mode[3:] != bytes(5)):
            raise ResetCheckError("Sender mode must be exact CAN/J/POS_VEL 1..10% with unchanged installation")
        self.bus, self.clock, self.log, self.used = bus, clock, [], set()
        self.mode = mode

    def send(self, phase, can_id, data):
        if not isinstance(data, bytes) or len(data) != 8:
            raise ResetCheckError("Exactly eight immutable payload bytes required")
        key = (phase, can_id, data[0] if phase in ("enable", "fault_disable") else None)
        if key in self.used:
            raise ResetCheckError("Duplicate phase send forbidden")
        valid = ((phase in ("preload", "before_enable", "after_enable") and can_id in JOINT_TARGET_IDS)
            or (phase == "mode" and can_id == 0x151 and data == self.mode)
            or (phase in ("enable", "fault_disable") and can_id == 0x471
                and data == bytes([data[0], 2 if phase == "enable" else 1, 0, 0, 0, 0, 0, 0])
                and 1 <= data[0] <= 6)
            or (phase == "gripper_hold" and can_id == 0x159
                and 0 <= int.from_bytes(data[:4], "big", signed=True) <= 100000
                and data[4:] == bytes.fromhex("03e80100"))
            or (phase == "fault_gripper_disable" and can_id == 0x159
                and 0 <= int.from_bytes(data[:4], "big", signed=True) <= 100000
                and data[4:] == bytes(4)))
        if not valid:
            raise ResetCheckError("Operation outside initialization allowlist")
        self.used.add(key)
        item = {"phase": phase, "id": hex(can_id), "data": data.hex(),
            "attempt_at": self.clock(), "socket_accepted": False}
        self.log.append(item)
        packet = struct.pack("=IB3x8s", can_id, 8, data)
        if self.bus.send(packet, socket.MSG_DONTWAIT) != len(packet):
            raise ResetCheckError("Partial/unknown socket send; no retry")
        item["socket_accepted"] = True
        item["completed_at"] = self.clock()
        return item["completed_at"]

    def current_target(self, phase, monitor, review):
        check_fresh_faults(monitor, now=self.clock())
        current = monitor.snapshot(now=self.clock())["pose"]
        recovery = "initial_target_deg" in review
        check_pose(current, review, allow_recovery_feedback=recovery)
        if recovery:
            target = [round(v*1000)/1000 for v in review["initial_target_deg"]]
            check_initial_target(target, current, review)
            # Same millidegree rounding and BE signed pair encoding as the
            # official SDK joint target Codec; verified offline, no clamping.
            raw = [round(v*1000) for v in target]
            packets = [struct.pack(">ii", *raw[i:i+2]) for i in (0, 2, 4)]
        else:
            target = current[:6]
            check_pose([*target, current[6]], review)
            packets = [monitor.parts[state_id][0] for state_id in (0x2A5, 0x2A6, 0x2A7)]
        for target_id, data in zip(JOINT_TARGET_IDS, packets, strict=True):
            self.send(phase, target_id, data)
        return target


class SupportedFaultScope:
    """Runs before socket close; never enables or retries a failed operation."""
    def __init__(self, result):
        self.result, self.sender, self.monitor = result, None, None

    def __enter__(self): return self

    def __exit__(self, kind, error, traceback):
        if kind is None or self.sender is None:
            return False
        mode_attempted = any(item["phase"] == "mode" for item in self.sender.log)
        enable_attempted = any(item["phase"] == "enable" for item in self.sender.log)
        # After a mode attempt legacy firmware may have changed internal state
        # before fresh enable feedback arrives. Under the explicit supported
        # fault-disable review, do not assume it remained disabled.
        if enable_attempted or mode_attempted:
            failures = []
            for axis in range(1, 7):
                try:
                    self.sender.send("fault_disable", 0x471, bytes([axis, 1, 0, 0, 0, 0, 0, 0]))
                except Exception as disable_error:
                    failures.append(f"axis {axis}: {disable_error}")
            self.result["fault_disable_attempted"] = True
            self.result["fault_disable_errors"] = failures
            self.result["fault_disable_confirmed"] = False
            self.result["support_required_until_manual_verification"] = True
        gripper = next((item for item in self.sender.log if item["phase"] == "gripper_hold"), None)
        if gripper is not None:
            try:
                self.sender.send("fault_gripper_disable", 0x159,
                    bytes.fromhex(gripper["data"])[:4]+bytes(4))
                self.result["fault_gripper_disable_attempted"] = True
            except Exception as disable_error:
                self.result["fault_gripper_disable_error"] = str(disable_error)
        return False


def send_target_after_enable(sender, monitor, review, *, enable_at, now):
    """Send the reviewed target only after all six fresh enable acknowledgements.

    A successful socket write of 0x471 is not proof the drives are ready to
    accept a target. The final target must follow their actual feedback.
    """
    check_fresh_faults(monitor, now=now)
    monitor.fresh(MOTOR_IDS, now=now, after=enable_at)
    if not all(monitor.parts[i][0][5] & 0x40 for i in MOTOR_IDS):
        raise FatalCheckError("Cannot send initial target before all six enable acknowledgements")
    return sender.current_target("after_enable", monitor, review)


def run(*, review, helper_path, interface, owner_path):
    if review.get("reuse_enabled") is True:
        return run_reuse_enabled(review=review, helper_path=helper_path,
                                 interface=interface, owner_path=owner_path)
    result = {"operation": "supported_j5_current_pose_check", "success": False,
        "no_model_actions": True, "no_reset_or_zero": True,
        "gripper_hold_requested": "gripper_hold_force_n" in review,
        "requested_mode": MODE.hex(), "posvel_direct_feedback_available": False}
    sender = None
    enable_attempted = False
    try:
        validate_review(review)
        recovery = "initial_target_deg" in review
        result["explicit_initial_target_recovery"] = recovery
        if recovery:
            result["reviewed_initial_target_deg"] = review["initial_target_deg"]
            result["reviewed_expected_delta_deg"] = [q-a for q, a in zip(
                review["initial_target_deg"], review["expected_pose_deg_m"][:6], strict=True)]
        helpers = load_helpers(helper_path)
        with open(owner_path, "a") as owner:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result["identity"], result["link"] = verify_endpoint(helpers, interface, review["expected_identity"])
            mapper = helpers.ClockMapper(); mapper.sample()
            with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus, SupportedFaultScope(result) as fault_scope:
                bus.setsockopt(socket.SOL_SOCKET, helpers.SO_TIMESTAMPNS_OLD, 1)
                bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
                bus.bind((interface,)); bus.setblocking(False)
                monitor = ResetMonitor(started_at=time.monotonic())
                sender = LimitedSender(bus)
                fault_scope.sender, fault_scope.monitor = sender, monitor
                foreign_baseline = None
                def read():
                    select.select([bus], [], [], .005)
                    if not receive_available(bus, helpers, mapper, monitor):
                        raise ResetCheckError("CAN receive queue did not drain")
                    if foreign_baseline is not None and monitor.last_control_at > foreign_baseline:
                        raise FatalCheckError("Unexpected external CAN control source")
                    check_fresh_faults(monitor, now=time.monotonic())
                    if enable_attempted:
                        try:
                            monitor.fresh((*MOTOR_IDS, *POSE_IDS, STATUS_ID), now=time.monotonic())
                        except ResetCheckError as stale:
                            raise FatalCheckError("Feedback became stale after enable: "+str(stale)) from stale
                def wait_for(check, timeout_s=2.):
                    deadline, last_error = time.monotonic()+timeout_s, "No fresh feedback"
                    while time.monotonic() < deadline:
                        read()
                        try: return check(time.monotonic())
                        except FatalCheckError: raise
                        except ResetCheckError as error: last_error = str(error)
                    raise ResetCheckError("Phase not confirmed: "+last_error)
                def before(now):
                    check_fresh_faults(monitor, now=now)
                    sample = monitor.preflight(now=now)
                    check_pose(sample["pose"], review, allow_recovery_feedback=recovery)
                    if recovery: check_initial_target(review["initial_target_deg"], sample["pose"], review)
                    if any(sample["enabled"]): raise ResetCheckError("Expected six disabled motors after reset")
                    return sample
                result["before"] = wait_for(before, 5.)
                anchor = result["before"]["pose"]
                if recovery:
                    result["actual_initial_delta_deg"] = [q-a for q, a in zip(
                        review["initial_target_deg"], anchor[:6], strict=True)]
                foreign_baseline = monitor.last_control_at
                verify_endpoint(helpers, interface, review["expected_identity"])
                read(); before(time.monotonic())
                sender.current_target("preload", monitor, review)
                mode_at = sender.send("mode", 0x151, MODE)
                def mode_ok(now):
                    check_fresh_faults(monitor, now=now)
                    monitor.fresh((0x2A1,), now=now, after=mode_at)
                    sample = monitor.snapshot(now=now)
                    check_pose(sample["pose"], review, anchor=anchor, allow_recovery_feedback=recovery)
                    status = monitor.parts[0x2A1][0]
                    if status[1] or int.from_bytes(status[6:8], "big"):
                        raise FatalCheckError("Controller fault during mode phase")
                    if status[0] != 1 or status[1] != 0 or status[2] != 1 or int.from_bytes(status[6:8], "big"):
                        raise ResetCheckError("Fresh controller feedback has not confirmed healthy CAN/J")
                    if any(sample["enabled"]): raise FatalCheckError("Motor enabled before explicit enable phase")
                    return sample
                result["after_mode"] = wait_for(mode_ok)
                # Refresh the latest-feedback hold or the explicitly reviewed
                # small recovery target. No training/model/old target is used.
                commanded = sender.current_target("before_enable", monitor, review)
                target_pose = [*commanded, monitor.snapshot(now=time.monotonic())["pose"][6]]
                result["commanded_initial_target_deg"] = commanded
                for axis in range(1, 7):
                    enable_attempted = True
                    sender.send("enable", 0x471, bytes([axis, 2, 0, 0, 0, 0, 0, 0]))
                enable_at = time.monotonic()
                def six_new_enabled(now):
                    check_fresh_faults(monitor, now=now)
                    monitor.fresh(MOTOR_IDS, now=now, after=enable_at)
                    enabled = [bool(monitor.parts[i][0][5] & 0x40) for i in MOTOR_IDS]
                    if not all(enabled):
                        raise ResetCheckError("Six new enabled bits not all confirmed")
                    return {"enabled": enabled,
                        "source_monotonic": [monitor.parts[i][1] for i in MOTOR_IDS]}
                # Motor-enable acknowledgement gets only 200ms; the following
                # supported small recovery can take up to 2s to converge.
                result["six_new_enabled"] = wait_for(six_new_enabled, .2)
                send_target_after_enable(sender, monitor, review,
                    enable_at=enable_at, now=time.monotonic())
                result["target_sent_after_enabled_monotonic"] = sender.log[-1]["completed_at"]
                result["convergence_timeout_s"] = 2.
                result["convergence_trace"] = []
                def enabled_ok(now):
                    check_fresh_faults(monitor, now=now)
                    sample = monitor.snapshot(now=now, after=enable_at)
                    sample["target_error_deg"] = [q-t for q, t in zip(sample["pose"][:6], target_pose[:6], strict=True)]
                    sample["anchor_delta_deg"] = [q-a for q, a in zip(sample["pose"][:6], anchor[:6], strict=True)]
                    result["last_target_observation"] = sample
                    trace = result["convergence_trace"]
                    source_at = min(sample["motor_source_monotonic"])
                    if not trace or source_at-trace[-1]["source_monotonic"] >= .1:
                        trace.append({"source_monotonic": source_at, "pose": sample["pose"],
                            "target_error_deg": sample["target_error_deg"]})
                    check_pose(sample["pose"], review, anchor=anchor if recovery else target_pose,
                        allow_recovery_feedback=recovery, enforce_start=not recovery)
                    if any(b & 0xbf for b in sample["motor_status_bytes"]):
                        raise FatalCheckError("Fresh motor fault flags present")
                    if not all(sample["enabled"]): raise ResetCheckError("Six new enabled bits not all confirmed")
                    status = monitor.parts[0x2A1][0]
                    if status[0] != 1 or status[1] or status[2] != 1 or int.from_bytes(status[6:8], "big"):
                        raise FatalCheckError("Controller feedback left healthy CAN/J")
                    if recovery and any(abs(q-t) > .5 for q, t in zip(sample["pose"][:6], target_pose[:6], strict=True)):
                        pending = [f"joint_{i}={q:.3f}deg target={t:.3f}deg error={q-t:+.3f}deg"
                            for i, (q, t) in enumerate(zip(sample["pose"][:6], target_pose[:6], strict=True), 1)
                            if abs(q-t) > .5]
                        raise ResetCheckError("Initial target not reached (tolerance 0.5deg): "+"; ".join(pending))
                    return sample
                try:
                    result["enabled"] = wait_for(enabled_ok)
                    observe_until = time.monotonic()+.5
                    while time.monotonic() < observe_until:
                        read()
                        result["after"] = enabled_ok(time.monotonic())
                    if "gripper_hold_force_n" in review:
                        read()
                        grip = monitor.parts[0x2A8][0]
                        width_raw = int.from_bytes(grip[:4], "big", signed=True)
                        if grip[7] != 0 or not 0 <= width_raw <= 100000:
                            raise FatalCheckError("Current gripper is not a valid [0,100]mm width")
                        grip_at = sender.send("gripper_hold", 0x159,
                            grip[:4]+bytes.fromhex("03e80100"))
                        def gripper_ok(now):
                            enabled_ok(now)
                            monitor.fresh((0x2A8,), now=now, after=grip_at)
                            data = monitor.parts[0x2A8][0]
                            if data[6] & 0x3f or data[7] != 0:
                                raise FatalCheckError("Gripper fault or mode change while enabling hold")
                            actual = int.from_bytes(data[:4], "big", signed=True)
                            if abs(actual-width_raw) > 2000:
                                raise FatalCheckError("Gripper deviated more than 2mm from measured hold width")
                            if not data[6] & 0x40:
                                raise ResetCheckError("Fresh gripper feedback not enabled yet")
                            return {"target_width_m": width_raw*1e-6,
                                "measured_width_m": actual*1e-6, "force_n": 1.,
                                "status": data[6], "mode": data[7],
                                "source_monotonic": monitor.parts[0x2A8][1]}
                        result["gripper_hold"] = wait_for(gripper_ok, .2)
                    result["success"] = True
                    result["holds_current_pose_enabled"] = True
                except BaseException:
                    raise
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = str(error) or type(error).__name__
        # SupportedFaultScope already handled the bounded fault-disable
        # attempt before socket close. No reopen or automatic retry occurs.
    finally:
        result["tx_log"] = [] if sender is None else sender.log
        result["enable_attempted"] = enable_attempted
    return result


def check_enabled_sample(monitor, review, *, now, after=None, anchor=None):
    """Fresh healthy enabled feedback, without changing motor enable state."""
    check_fresh_faults(monitor, now=now)
    sample = monitor.snapshot(now=now, after=after)
    if not all(sample["enabled"]):
        raise FatalCheckError("Enabled takeover lost a motor enable acknowledgement")
    status = bytes.fromhex(sample["arm_status_raw"])
    if status[0] != 1 or status[1] or status[2] != 1 or status[3] or int.from_bytes(status[6:8], "big"):
        raise FatalCheckError("Enabled takeover requires healthy CAN/J feedback")
    pose = sample["pose"]
    # The existing closed gripper can read slightly below zero. This permits
    # observation only; every transmitted width remains in [0,100]mm.
    if not -.001 <= pose[6] <= .1:
        raise FatalCheckError("Gripper feedback outside enabled-takeover range")
    reference_width = review["expected_pose_deg_m"][6]
    check_pose([*pose[:6], reference_width], review,
               anchor=None if anchor is None else [*anchor[:6], reference_width])
    return sample


def run_reuse_enabled(*, review, helper_path, interface, owner_path):
    """Preserve an enabled arm. Never reset, enable or disable any motor.

    The low-speed mode request is observed independently by the rollout RX
    bridge. Old firmware does not expose a separate POS_VEL acknowledgement.
    Failure leaves the motor-enable state unchanged and never retries.
    """
    result = {"operation": "reuse_enabled_j5_current_pose", "reused_enabled": True,
              "success": False, "no_model_actions": True, "no_reset_or_zero": True,
              "enable_attempted": False, "requested_mode": MODE.hex(),
              "posvel_direct_feedback_available": False}
    sender = None
    try:
        validate_review(review)
        speed = takeover_speed(review)
        selected_mode = mode_payload(speed)
        result.update(operation=f"reuse_enabled_j{speed}_current_pose",
                      requested_mode=selected_mode.hex(), takeover_speed_percent=speed)
        if "initial_target_deg" in review or "gripper_hold_force_n" in review:
            raise ResetCheckError("Enabled takeover cannot use cold recovery parameters")
        width = review["warm_gripper_target_m"]
        if not isinstance(width, (int, float)) or not math.isfinite(width) or not 0 <= width <= .1:
            raise ResetCheckError("Explicit valid warm gripper target required")
        if abs(width-review["expected_pose_deg_m"][6]) > .015:
            raise ResetCheckError("Warm gripper change exceeds15mm")
        helpers = load_helpers(helper_path)
        with open(owner_path, "a") as owner:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result["identity"], result["link"] = verify_endpoint(helpers, interface, review["expected_identity"])
            mapper = helpers.ClockMapper(); mapper.sample()
            with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
                bus.setsockopt(socket.SOL_SOCKET, helpers.SO_TIMESTAMPNS_OLD, 1)
                bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
                bus.bind((interface,)); bus.setblocking(False)
                monitor = ResetMonitor(started_at=time.monotonic())
                sender = LimitedSender(bus, mode=selected_mode)
                baseline = None
                def read():
                    select.select([bus], [], [], .005)
                    if not receive_available(bus, helpers, mapper, monitor):
                        raise FatalCheckError("CAN receive queue did not drain")
                    if baseline is not None and monitor.last_control_at > baseline:
                        raise FatalCheckError("Other control source detected during enabled takeover")
                    check_fresh_faults(monitor, now=time.monotonic())
                deadline = time.monotonic()+3.
                while True:
                    read()
                    try:
                        monitor.preflight(now=time.monotonic())
                        result["before"] = check_enabled_sample(monitor, review, now=time.monotonic())
                        break
                    except FatalCheckError: raise
                    except ResetCheckError:
                        if time.monotonic() >= deadline: raise
                anchor = result["before"]["pose"]
                baseline = monitor.last_control_at
                verify_endpoint(helpers, interface, review["expected_identity"])
                read()
                check_enabled_sample(monitor, review, now=time.monotonic(), anchor=anchor)
                sender.current_target("preload", monitor, review)
                mode_at = sender.send("mode", 0x151, selected_mode)
                sender.current_target("before_enable", monitor, review)
                raw_width = round(width*1e6)
                grip_at = sender.send("gripper_hold", 0x159,
                                     struct.pack(">i", raw_width)+bytes.fromhex("03e80100"))
                result["warm_gripper_target_m"] = width
                deadline = time.monotonic()+2.
                stable_since = None
                while True:
                    read()
                    now = time.monotonic()
                    try:
                        sample = check_enabled_sample(monitor, review, now=now,
                                                      after=max(mode_at, grip_at), anchor=anchor)
                        grip = monitor.parts[0x2A8][0]
                        if not grip[6] & 0x40 or abs(sample["pose"][6]-width) > .001:
                            raise ResetCheckError("Warm gripper target not yet reached")
                        if any(abs(a-b) > .5 for a,b in zip(sample["pose"][:6], anchor[:6])):
                            raise FatalCheckError("Current-pose hold moved more than0.5deg")
                        result["enabled"] = result["after"] = sample
                        if stable_since is None: stable_since = now
                        if now-stable_since >= .3: break
                    except FatalCheckError: raise
                    except ResetCheckError:
                        stable_since = None
                        if now >= deadline: raise
                result["success"] = result["holds_current_pose_enabled"] = True
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = str(error) or type(error).__name__
    finally:
        result["tx_log"] = [] if sender is None else sender.log
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-only", action="store_true")
    mode.add_argument("--prepare-j-hold-supported-reviewed", action="store_true")
    mode.add_argument("--sdk-codec-plan", action="store_true", help="AIPC pure codec check; no CAN")
    parser.add_argument("--review-json")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--rx-helper", default="/home/ubuntu/kr260-camera/remote_can_source.py")
    parser.add_argument("--owner-path", default="/tmp/piper_policy_executor_can0.lock")
    args = parser.parse_args(argv)
    if args.sdk_codec_plan:
        speed = 5
        if args.review_json:
            review = json.loads(Path(args.review_json).read_text())
            validate_review(review)
            speed = takeover_speed(review)
        print(json.dumps(sdk_codec_plan(speed), indent=2)); return 0
    if not args.prepare_j_hold_supported_reviewed:
        plan = {"check_only": True, "opens_can": False, "tx": [],
            "plan": ["fresh disabled pose + reviewed bounds + 1s silence", "current joint target preload",
                "0x151 CAN/J/POS_VEL 5% once", "fresh CAN/J feedback", "refresh current joint target",
                "per-axis 1..6 enable once", "wait for six fresh enabled acknowledgements",
                "send the same reviewed target after acknowledgements", "0.5s fresh feedback check"],
            "requires": ["secure physical support", "external explicit initialization approval",
                "fault disable reviewed", "caller-supplied identity/bounds/expected pose/tolerances"],
            "warning": "Legacy latching is unproven. Normal hold cap2deg; explicit initial_target_deg recovery cap4.5deg, target delta<=4deg and target error<=0.5deg. No model actions."}
        if args.review_json:
            review = json.loads(Path(args.review_json).read_text())
            validate_review(review)
            plan["expected_pose_deg_m"] = review["expected_pose_deg_m"]
            plan["joint_bounds_deg"] = review["joint_bounds_deg"]
            plan["max_deviation_deg"] = review["max_deviation_deg"]
            plan["takeover_speed_percent"] = takeover_speed(review)
            plan["requested_mode"] = mode_payload(takeover_speed(review)).hex()
            if review.get("reuse_enabled") is True:
                plan["plan"] = ["fresh healthy six-axis enabled pose + reviewed bounds",
                    "current joint target preload",
                    f"0x151 CAN/J/POS_VEL {takeover_speed(review)}% once",
                    "refresh current joint target", "reviewed 1N gripper width hold",
                    "0.3s stable fresh feedback; no reset, enable or disable requests"]
            if "initial_target_deg" in review:
                plan["explicit_initial_target_deg"] = review["initial_target_deg"]
                plan["delta_from_expected_deg"] = [q-a for q, a in zip(
                    review["initial_target_deg"], review["expected_pose_deg_m"][:6], strict=True)]
                plan["recovery_feedback_exception"] = {"joint_2.deg": [-4., 0.], "joint_3.deg": [0., 4.]}
                plan["plan"][1] = "preload the explicit reviewed small recovery target"
                plan["plan"][4] = "repeat the same explicit recovery target before enable"
        print(json.dumps(plan, indent=2)); return 0
    if not args.review_json: parser.error("--review-json is required for supported initialization")
    def terminate(_signum, _frame):
        # Graceful process/SSH termination must unwind the supported-fault
        # scope before the CAN socket closes. SIGKILL/power loss cannot unwind.
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, terminate)
    review = json.loads(Path(args.review_json).read_text())
    result = run(review=review, helper_path=args.rx_helper, interface=args.interface, owner_path=args.owner_path)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

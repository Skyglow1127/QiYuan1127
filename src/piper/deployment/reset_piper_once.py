#!/usr/bin/env python3
"""Reviewable reset-only utility: default check-only, stdlib, no automatic enable.

WARNING: reset can remove motor support. The execution flag represents prior
explicit user approval with the arm securely supported; this program cannot
verify physical support. It never requests permission or performs a grasp.

SDK evidence: piper/default/driver.py:844 reset -> ArmMsgMotionCtrl(2);
piper/default/parser.py:392 encode_150_motion_ctrl; low-speed feedback
msgs/piper/default/feedback/arm_feedback_low_spd.py:42 Byte5 bit6 is enable.
"""

import argparse
import fcntl
import importlib.util
import json
import math
from pathlib import Path
import select
import socket
import struct
import time

RESET_ID = 0x150
RESET_DATA = bytes.fromhex("0200000000000000")
RESET_PACKET = struct.pack("=IB3x8s", RESET_ID, 8, RESET_DATA)
MOTOR_IDS = tuple(range(0x261, 0x267))
POSE_IDS = (0x2A5, 0x2A6, 0x2A7, 0x2A8)
STATUS_ID = 0x2A1
# Includes observed leader motion/target frames plus the linkage command.
# Silence is supplementary evidence, never proof of physical isolation.
CONTROL_IDS = (*range(0x150, 0x160), 0x470)
MAX_AGE_S = .2
QUIET_S = 1.


class ResetCheckError(RuntimeError):
    pass


class ResetMonitor:
    def __init__(self, *, started_at):
        self.parts = {}
        self.last_control_at = started_at

    def ingest(self, can_id, data, *, source_at, now):
        if not math.isfinite(source_at) or source_at > now:
            raise ResetCheckError("Invalid kernel source timestamp")
        if len(data) != 8:
            return
        if can_id in CONTROL_IDS:
            self.last_control_at = max(self.last_control_at, source_at)
        if can_id in (*MOTOR_IDS, *POSE_IDS, STATUS_ID):
            previous = self.parts.get(can_id)
            if previous is None or source_at >= previous[1]:
                self.parts[can_id] = (bytes(data), source_at)

    def fresh(self, ids, *, now, after=None):
        errors = []
        for can_id in ids:
            part = self.parts.get(can_id)
            if part is None:
                errors.append(f"{can_id:#x}:missing")
            elif not 0 <= now-part[1] < MAX_AGE_S:
                errors.append(f"{can_id:#x}:stale")
            elif after is not None and part[1] <= after:
                errors.append(f"{can_id:#x}:not-after-reset")
        if errors:
            raise ResetCheckError("; ".join(errors))

    def snapshot(self, *, now, after=None):
        self.fresh((*MOTOR_IDS, *POSE_IDS, STATUS_ID), now=now, after=after)
        joints = [v*.001 for can_id in POSE_IDS[:3]
            for v in struct.unpack(">ii", self.parts[can_id][0])]
        gripper = self.parts[0x2A8][0]
        if gripper[7] != 0:
            raise ResetCheckError("Gripper feedback is not width mode; no metre pose inferred")
        pose = joints+[int.from_bytes(gripper[:4], "big", signed=True)*1e-6]
        status = self.parts[STATUS_ID][0]
        return {
            "pose_order": [f"joint_{i}.deg" for i in range(1, 7)]+["gripper.m"],
            "pose": pose,
            "enabled": [bool(self.parts[i][0][5] & 0x40) for i in MOTOR_IDS],
            "motor_status_bytes": [self.parts[i][0][5] for i in MOTOR_IDS],
            "motor_source_monotonic": [self.parts[i][1] for i in MOTOR_IDS],
            "motor_ages_s": [now-self.parts[i][1] for i in MOTOR_IDS],
            "state_ages_s": {hex(i): now-self.parts[i][1] for i in (*POSE_IDS, STATUS_ID)},
            "arm_status_raw": status.hex(),
        }

    def preflight(self, *, now):
        result = self.snapshot(now=now)
        if now-self.last_control_at < QUIET_S:
            raise ResetCheckError("Leader/control traffic has not been quiet for at least 1 second")
        result["control_quiet_s"] = now-self.last_control_at
        return result

    def disabled_after(self, *, reset_at, now):
        result = self.snapshot(now=now, after=reset_at)
        if any(result["enabled"]):
            raise ResetCheckError("At least one fresh post-reset motor feedback still says enabled")
        return result


class OneReset:
    """Exactly one socket send attempt; no retry even on send failure."""
    def __init__(self):
        self.attempts = 0
        self.sent_at = None
        self.accepted_by_socket = False

    def send(self, bus, *, clock=time.monotonic):
        if self.attempts:
            raise ResetCheckError("Reset already attempted; automatic repeat is forbidden")
        self.attempts = 1
        # This is the utility's only CAN transmission site and fixed payload.
        written = bus.send(RESET_PACKET, socket.MSG_DONTWAIT)
        self.sent_at = clock()
        if written != len(RESET_PACKET):
            raise ResetCheckError("Reset socket result uncertain; do not repeat automatically")
        self.accepted_by_socket = True


def load_helpers(path):
    spec = importlib.util.spec_from_file_location("piper_reset_rx_helpers", path)
    helpers = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helpers)
    return helpers


def verify_endpoint(helpers, interface, expected):
    identity = helpers.interface_identity(interface)
    if identity != expected:
        raise ResetCheckError("CAN endpoint identity does not match the reviewed identity")
    link = helpers.link_status(interface)
    if not link.get("up") or link.get("listen_only") or not link.get("bitrate_ok"):
        raise ResetCheckError("Existing UP + normal CAN + 1Mbps required; tool never changes interface")
    return identity, link


def receive_available(bus, helpers, mapper, monitor):
    # A complete drain prevents a queued recent control frame being hidden
    # behind a few fresh status packets before the quiet gate is evaluated.
    for _ in range(4096):
        try:
            packet, ancillary, flags, _ = bus.recvmsg(16, socket.CMSG_SPACE(helpers.TIMESPEC.size))
        except BlockingIOError:
            return True
        if len(packet) != 16:
            raise ResetCheckError("Malformed classic CAN packet")
        can_id, size, data = struct.unpack("=IB3x8s", packet)
        if can_id & 0xe0000000 or size != 8:
            continue
        source_ns, _ = mapper.convert(helpers._kernel_timestamp(ancillary, flags))
        monitor.ingest(can_id, data, source_at=source_ns/1e9, now=time.monotonic())
    return False


def run_reset(*, interface, expected_identity, helper_path, owner_path,
              preflight_timeout_s=5., confirm_timeout_s=3.):
    transaction = OneReset()
    result = {"operation": "one_reset_only", "reset_id": "0x150",
        "reset_data": RESET_DATA.hex(), "confirmed_disabled": False,
        "no_mode_enable_targets_or_zero": True}
    bus = None
    try:
        helpers = load_helpers(helper_path)
        if struct.calcsize("@l") != 8:
            raise ResetCheckError("64-bit Linux kernel timestamp support required")
        with open(owner_path, "a") as owner:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result["identity"], result["link"] = verify_endpoint(helpers, interface, expected_identity)
            mapper = helpers.ClockMapper()
            mapper.sample()
            with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
                bus.setsockopt(socket.SOL_SOCKET, helpers.SO_TIMESTAMPNS_OLD, 1)
                bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
                bus.bind((interface,))
                bus.setblocking(False)
                monitor = ResetMonitor(started_at=time.monotonic())
                deadline = time.monotonic()+preflight_timeout_s
                last_error = "No complete feedback yet"
                while time.monotonic() < deadline:
                    select.select([bus], [], [], .01)
                    if not receive_available(bus, helpers, mapper, monitor):
                        continue
                    try:
                        monitor.preflight(now=time.monotonic())
                        # Recheck interface/identity before the final fresh gate.
                        verify_endpoint(helpers, interface, expected_identity)
                        if not receive_available(bus, helpers, mapper, monitor):
                            continue
                        result["before"] = monitor.preflight(now=time.monotonic())
                        break
                    except ResetCheckError as error:
                        last_error = str(error)
                else:
                    raise ResetCheckError("Preflight timed out; NO reset sent: "+last_error)
                transaction.send(bus)
                result["reset_sent_monotonic"] = transaction.sent_at
                deadline = time.monotonic()+confirm_timeout_s
                last_error = "No post-reset feedback yet"
                while time.monotonic() < deadline:
                    select.select([bus], [], [], .01)
                    if not receive_available(bus, helpers, mapper, monitor):
                        continue
                    try:
                        result["after"] = monitor.disabled_after(
                            reset_at=transaction.sent_at, now=time.monotonic())
                        result["confirmed_disabled"] = True
                        break
                    except ResetCheckError as error:
                        last_error = str(error)
                else:
                    raise ResetCheckError("Reset was attempted once, disabled state NOT CONFIRMED: "+last_error)
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = str(error) or type(error).__name__
    finally:
        result["send_attempts"] = transaction.attempts
        result["socket_accepted_reset"] = transaction.accepted_by_socket
        result["automatic_retry"] = False
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-only", action="store_true", help="default: print plan; do not open CAN")
    mode.add_argument("--reset-supported-reviewed", action="store_true",
        help="PRIOR explicit user reset approval and secure physical support were confirmed externally")
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--rx-helper", default="/home/ubuntu/kr260-camera/remote_can_source.py")
    parser.add_argument("--expected-identity", help="JSON file containing the reviewed exact CAN identity")
    parser.add_argument("--owner-path", default="/tmp/piper_policy_executor_can0.lock")
    args = parser.parse_args(argv)
    if not args.reset_supported_reviewed:
        print(json.dumps({"check_only": True, "opens_can": False, "send_attempts": 0,
            "would_send_once": {"id": "0x150", "data": RESET_DATA.hex()},
            "requires": ["explicit user approval", "secure arm support", "pinned CAN identity",
                "existing UP/normal/1Mbps", "exclusive owner", "fresh six-axis/state feedback",
                "at least 1 second control silence"],
            "postcondition": "six post-reset fresh Byte5 bit6=0 and fresh 7D width pose; never retry",
            "warning": "Reset can remove motor support. No mode/enable/targets/zero are sent."}, indent=2))
        return 0
    if not args.expected_identity:
        parser.error("execution requires --expected-identity with the reviewed endpoint JSON")
    expected = json.loads(Path(args.expected_identity).read_text())
    result = run_reset(interface=args.interface, expected_identity=expected,
        helper_path=args.rx_helper, owner_path=args.owner_path)
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["confirmed_disabled"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

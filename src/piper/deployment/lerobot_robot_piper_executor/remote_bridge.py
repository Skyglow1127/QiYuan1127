#!/usr/bin/env python3
"""Independent stdlib SocketCAN RPC bridge; RX by default, no interface changes.

Run only after review/deployment. Importing this module opens no resources.
No enable, zero, reset, mode-switch or arbitrary-frame RPC exists.
"""

import argparse
from dataclasses import asdict, dataclass
import fcntl
import importlib.util
import json
import math
import os
from pathlib import Path
import secrets
import select
import signal
import socket
import struct
import sys
import threading
import time

try:
    from .guard import ACTION_KEYS, ActionGuard, GuardError, GuardLimits, LocalOwner, Readiness
except ImportError:  # Script deployment: guard.py is next to this file.
    from guard import ACTION_KEYS, ActionGuard, GuardError, GuardLimits, LocalOwner, Readiness

PROTOCOL = "piper-deploy-rpc-v1"
TX_IDS = (0x155, 0x156, 0x157, 0x159)
STATE_IDS = (0x2A5, 0x2A6, 0x2A7, 0x2A8, 0x2A1)
CONTROL_IDS = (*TX_IDS, 0x150, 0x151, 0x470)
MAX_LINE = 16384


def readable_link(link):
    """RX can observe an existing UP/1Mbps interface in either CAN mode.

    Normal CAN hardware may acknowledge bus traffic. This function neither
    changes interface mode nor authorizes application-level CAN transmission.
    Arming remains separately restricted to normal mode by the engine.
    """
    return link.get("up") is True and link.get("bitrate_ok") is True


@dataclass(frozen=True)
class ExecutionPolicy:
    execute: bool = False
    readiness: Readiness = Readiness()
    limits: GuardLimits | None = None
    lease_s: float = .25
    max_ttl_s: float = .15
    max_feedback_age_s: float = .10
    max_feedback_skew_s: float = .04
    max_link_age_s: float = .50
    quiet_before_arm_s: float = 1.
    motion_ctrl2_max_age_s: float = 5.
    # Installation byte 0 is intentionally allowed by the SDK for no change.
    # This is evidence to compare, NEVER a mode command sent by this bridge.
    expected_motion_ctrl2: tuple[int, ...] | None = None
    stop_strategy: str = "unconfirmed"
    stop_gripper_force_n: float | None = None
    max_gripper_force_n: float | None = None

    def validate_execute(self):
        if type(self.execute) is not bool or not self.execute:
            raise GuardError("Read-only bridge: execution is disabled")
        self.readiness.require_complete()
        if self.limits is None:
            raise GuardError("Explicit reviewed execution limits required")
        for value in (self.lease_s, self.max_ttl_s, self.max_feedback_age_s,
                      self.max_feedback_skew_s, self.max_link_age_s,
                      self.quiet_before_arm_s, self.motion_ctrl2_max_age_s):
            if not math.isfinite(value) or value <= 0:
                raise GuardError("Invalid execution time limit")
        if self.lease_s > 1. or self.max_ttl_s > .25:
            raise GuardError("Execution lease/TTL exceed preparation limits")
        mode = self.expected_motion_ctrl2
        if (mode is None or len(mode) != 8 or mode[:2] != (1, 1)
                or not 1 <= mode[2] <= 10 or mode[3] != 0 or mode[4] != 0
                or mode[5] not in (0, 1, 2, 3) or mode[6:] != (0, 0)):
            raise GuardError("Reviewed CAN/J/POS_VEL low-speed MotionCtrl2 evidence required")
        if self.stop_strategy != "fresh_feedback_target_once":
            raise GuardError("Firmware-specific fresh feedback hold strategy is unconfirmed")
        if (self.max_gripper_force_n is None or self.stop_gripper_force_n is None
                or not 0 < self.stop_gripper_force_n <= self.max_gripper_force_n <= 65.535):
            raise GuardError("Explicit gripper normal/stop force limits required")


def policy_from_dict(data):
    data = dict(data)
    data["readiness"] = Readiness(**data.get("readiness", {}))
    if data.get("limits") is not None:
        data["limits"] = GuardLimits(**data["limits"])
    if data.get("expected_motion_ctrl2") is not None:
        data["expected_motion_ctrl2"] = tuple(data["expected_motion_ctrl2"])
    policy = ExecutionPolicy(**data)
    if policy.execute:
        policy.validate_execute()
    return policy


class ControlEngine:
    """Pure control state machine. Tests inject fake clock/socket/owner.

    send_frame must be nonblocking and accept exactly one classic CAN frame.
    Main loop feeds KERNEL receive timestamps in remote monotonic seconds.
    The owner lock is advisory; physical leader isolation remains mandatory.
    """
    def __init__(self, *, policy=None, send_frame=None, owner=None, clock=time.monotonic):
        self.policy = policy or ExecutionPolicy()
        self.send_frame = send_frame
        self.owner = owner
        self.clock = clock
        self.guard = ActionGuard(self.policy.limits) if self.policy.limits else None
        self.parts = {}
        self.last_foreign_control = clock()
        self.last_motion_ctrl2 = None
        self.link, self.link_at = None, None
        self.lease, self.lease_until = None, None
        self.sequence = 0
        self.last_event = "read-only/unarmed"

    def set_link(self, link, *, sampled_at=None):
        self.link, self.link_at = dict(link), self.clock() if sampled_at is None else sampled_at
        if self.lease and (not link.get("up") or link.get("listen_only") or not link.get("bitrate_ok")):
            self.revoke("CAN link is not normal UP/1Mbps", hold=False)

    def ingest(self, can_id, data, source_at):
        now = self.clock()
        if not math.isfinite(source_at) or source_at > now:
            raise GuardError("Invalid CAN source time")
        if len(data) != 8:
            return
        if can_id in STATE_IDS:
            self.parts[can_id] = (bytes(data), source_at)
        if can_id == 0x151:
            self.last_motion_ctrl2 = (bytes(data), source_at)
        if can_id in CONTROL_IDS:
            self.last_foreign_control = source_at
            # CAN_RAW_RECV_OWN_MSGS is deliberately off. Thus control frames
            # seen here were not sent through this socket by this bridge.
            if self.lease:
                self.revoke("Other CAN control source detected", hold=False)
        if self.lease:
            try:
                self.snapshot(require_healthy=True)
            except GuardError:
                self.revoke("Feedback fault/staleness detected", hold=False)

    def snapshot(self, *, require_healthy=False):
        now = self.clock()
        missing = [hex(can_id) for can_id in STATE_IDS if can_id not in self.parts]
        if missing:
            raise GuardError("Missing feedback groups: " + ",".join(missing))
        raw = [self.parts[i][0] for i in STATE_IDS]
        times = [self.parts[i][1] for i in STATE_IDS]
        ages = [now - t for t in times]
        state = tuple(v * .001 for d in raw[:3] for v in struct.unpack(">ii", d))
        width, force, gripper_status, width_mode = struct.unpack(">ihBB", raw[3])
        # Never label an angle-mode integer as width metres in an RX result.
        state += (width * 1e-6 if width_mode == 0 else None,)
        status = {"ctrl_mode": raw[4][0], "arm_status": raw[4][1],
            "mode_feedback": raw[4][2], "teach_status": raw[4][3],
            "motion_status": raw[4][4], "err_code": int.from_bytes(raw[4][6:8], "big"),
            "gripper_status": gripper_status, "gripper_mode": width_mode,
            "gripper_force_n": force * .001, "gripper_raw": width}
        reasons = []
        if min(ages) < 0 or max(ages) > self.policy.max_feedback_age_s:
            reasons.append("stale feedback")
        if max(times) - min(times) > self.policy.max_feedback_skew_s:
            reasons.append("feedback group skew")
        if (status["ctrl_mode"] != 1 or status["arm_status"] or status["mode_feedback"] != 1
                or status["teach_status"] != 0 or status["err_code"]):
            reasons.append("arm not healthy CAN/J")
        if width_mode != 0 or gripper_status & 0x3f or not gripper_status & 0x40:
            reasons.append("gripper not healthy enabled width mode")
        result = {"state": state, "source_ns": int(min(times) * 1e9),
            "group_ages_s": ages, "status": status, "healthy": not reasons,
            "health_reasons": reasons, "armed": self.lease is not None,
            "event": self.last_event}
        if require_healthy and reasons:
            raise GuardError("; ".join(reasons))
        return result

    def _require_normal_link(self):
        now = self.clock()
        if (self.link is None or self.link_at is None or not 0 <= now-self.link_at <= self.policy.max_link_age_s
                or not self.link.get("up") or self.link.get("listen_only") or not self.link.get("bitrate_ok")):
            raise GuardError("Fresh normal-mode UP/1Mbps link required; bridge never changes interface")

    def arm(self, readiness):
        self.policy.validate_execute()
        if self.lease:
            raise GuardError("Already armed")
        if Readiness(**readiness) != self.policy.readiness:
            raise GuardError("Arm review does not match receiver's reviewed policy")
        self._require_normal_link()
        now = self.clock()
        if now-self.last_foreign_control < self.policy.quiet_before_arm_s:
            raise GuardError("CAN control source has not been quiet long enough")
        if (self.last_motion_ctrl2 is None or
                now-self.last_motion_ctrl2[1] > self.policy.motion_ctrl2_max_age_s or
                tuple(self.last_motion_ctrl2[0]) != self.policy.expected_motion_ctrl2):
            raise GuardError("Observed MotionCtrl2 does not match reviewed low-speed J/POS_VEL mode")
        snapshot = self.snapshot(require_healthy=True)
        if self.owner is None or self.send_frame is None:
            raise GuardError("Receiver owner/send implementation missing")
        self.owner.acquire()
        try:
            self.guard.arm(snapshot["state"], sampled_at=snapshot["source_ns"]/1e9,
                now=now, readiness=self.policy.readiness)
            self.lease, self.lease_until = secrets.token_hex(24), now+self.policy.lease_s
            self.sequence = 0
            self.last_event = "armed from fresh feedback; no TX"
            return {"lease": self.lease, "lease_until_ns": int(self.lease_until*1e9)}
        except BaseException:
            self.owner.release()
            raise

    def _decode_transaction(self, frames):
        if not isinstance(frames, list) or len(frames) != 4:
            raise GuardError("Exactly four target frames required")
        decoded = []
        for expected_id, frame in zip(TX_IDS, frames, strict=True):
            if (not isinstance(frame, dict) or set(frame) != {"id", "data"}
                    or type(frame["id"]) is not int or frame["id"] != expected_id):
                raise GuardError("Unknown/out-of-order CAN ID; target allowlist only")
            try:
                data = bytes.fromhex(frame["data"])
            except (TypeError, ValueError) as error:
                raise GuardError("Invalid frame payload") from error
            if len(data) != 8:
                raise GuardError("Exactly eight payload bytes required")
            decoded.append((expected_id, data))
        raw = [d for _, d in decoded]
        state = tuple(v*.001 for d in raw[:3] for v in struct.unpack(">ii", d))
        width, force, status, zero = struct.unpack(">iHBB", raw[3])
        if status != 1 or zero != 0:
            raise GuardError("Only enabled width target without clear/zero is allowed")
        if not 0 < force*.001 <= self.policy.max_gripper_force_n:
            raise GuardError("Gripper force exceeds reviewed limit")
        return decoded, state+(width*1e-6,)

    def target(self, *, lease, sequence, deadline_ns, frames):
        self.tick()
        now = self.clock()
        if self.lease is None or not secrets.compare_digest(str(lease), self.lease):
            raise GuardError("No matching armed lease")
        try:
            if type(sequence) is not int or sequence != self.sequence+1:
                raise GuardError("Target sequence/replay rejected")
            if type(deadline_ns) is not int or not now < deadline_ns/1e9 <= now+self.policy.max_ttl_s:
                raise GuardError("Target expired or exceeds receiver TTL bound")
            self._require_normal_link()
            snapshot = self.snapshot(require_healthy=True)
            decoded, target = self._decode_transaction(frames)
            command = self.guard.prepare(target, observed_state=snapshot["state"],
                sampled_at=snapshot["source_ns"]/1e9, proposal_at=now, now=now)
            for can_id, data in decoded:
                if self.clock() >= deadline_ns/1e9:
                    raise GuardError("Transaction expired during bounded send")
                self.guard.require_sendable(command, now=self.clock())
                self.send_frame(can_id, data)
            self.guard.commit(command, now=self.clock())
            self.sequence = sequence
            self.lease_until = min(self.clock()+self.policy.lease_s,
                deadline_ns/1e9+self.policy.lease_s)
            return {"accepted_sequence": sequence, "source_ns": snapshot["source_ns"]}
        except BaseException:
            self.revoke("Target rejected/partial transaction", hold=True)
            raise

    def revoke(self, reason, *, hold):
        was_armed = self.lease is not None
        self.lease, self.lease_until = None, None
        if self.guard:
            self.guard.disarm(reason)
        outcome = "no TX"
        try:
            if was_armed and hold and self.policy.stop_strategy == "fresh_feedback_target_once":
                # Hold uses latest actual feedback, not the last policy target.
                # Never attempt it with faults, stale groups or uncertain link.
                self._require_normal_link()
                snapshot = self.snapshot(require_healthy=True)
                for value, (lo, hi) in zip(snapshot["state"], self.policy.limits.bounds, strict=True):
                    if not lo <= value <= hi:
                        raise GuardError("Feedback hold would exceed reviewed limits")
                packets = [(i, self.parts[i+0x150][0]) for i in TX_IDS[:3]]
                width_bytes = self.parts[0x2A8][0][:4]
                grip = width_bytes+struct.pack(">HBB", round(self.policy.stop_gripper_force_n*1000), 1, 0)
                packets.append((0x159, grip))
                for can_id, data in packets:
                    self.send_frame(can_id, data)
                outcome = "fresh-feedback target sent once"
        except Exception as error:
            outcome = "hold unavailable: "+str(error)
        finally:
            if was_armed and self.owner:
                self.owner.release()
            self.last_event = reason+"; "+outcome

    def tick(self):
        if not self.lease:
            return
        now = self.clock()
        if now >= self.lease_until:
            self.revoke("Command lease expired", hold=True)
            return
        try:
            self._require_normal_link()
            self.snapshot(require_healthy=True)
        except GuardError:
            self.revoke("Feedback/link supervision failed", hold=False)

    def rpc(self, request, received_ns):
        op = request.get("op")
        if op == "ping":
            return {"remote_receive_ns": received_ns, "remote_send_ns": time.monotonic_ns()}
        if op == "snapshot":
            return self.snapshot()
        if op == "arm":
            return self.arm(request.get("readiness", {}))
        if op == "target":
            return self.target(lease=request.get("lease"), sequence=request.get("sequence"),
                deadline_ns=request.get("deadline_ns"), frames=request.get("frames"))
        if op == "disarm":
            self.revoke("Client disarmed", hold=True)
            return {"event": self.last_event}
        # There is deliberately no raw CAN, mode, enable, reset or zero RPC.
        raise GuardError("Unknown RPC operation")


def load_helpers(path):
    spec = importlib.util.spec_from_file_location("piper_rx_helpers", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def serve(interface, helper_path, policy, owner_path):
    helper = load_helpers(helper_path)
    identity, link = helper.interface_identity(interface), helper.link_status(interface)
    if not readable_link(link):
        raise GuardError("RX needs existing UP/1Mbps in listen-only or normal mode; bridge never changes interface")
    if struct.calcsize("@l") != 8:
        raise GuardError("64-bit Linux required for kernel timestamping")
    clock_mapper = helper.ClockMapper()
    clock_mapper.sample()
    output = bytearray()
    def emit(message):
        output.extend((json.dumps({"protocol": PROTOCOL, **message},
            allow_nan=False, separators=(",", ":"))+"\n").encode())
        if len(output) > 65536:
            raise GuardError("SSH output backpressure limit exceeded")
    bus = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.setsockopt(socket.SOL_SOCKET, helper.SO_TIMESTAMPNS_OLD, 1)
    # Own-message reception is explicitly disabled; foreign-source supervision
    # must not interpret this socket's echo as a competing controller.
    bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
    bus.bind((interface,))
    bus.setblocking(False)
    def send_frame(can_id, data):
        if can_id not in TX_IDS or len(data) != 8:
            raise GuardError("TX allowlist violation")
        packet = struct.pack("=IB3x8s", can_id, 8, data)
        if bus.send(packet, socket.MSG_DONTWAIT) != len(packet):
            raise GuardError("Partial CAN socket write")
    engine = ControlEngine(policy=policy, send_frame=send_frame,
        owner=LocalOwner(owner_path), clock=time.monotonic)
    engine.set_link(link)
    emit({"type": "hello", "identity": identity, "link": link,
        "execute": policy.execute, "kernel_timestamps": True})
    lock = threading.RLock()
    stopped = threading.Event()
    def refresh_link():
        while not stopped.wait(.2):
            try:
                sampled_at = time.monotonic()
                latest = helper.link_status(interface)
                with lock: engine.set_link(latest, sampled_at=sampled_at)
            except Exception:
                with lock: engine.set_link({"up": False}, sampled_at=time.monotonic())
    thread = threading.Thread(target=refresh_link, daemon=True)
    thread.start()
    stdin, stdout = sys.stdin.fileno(), sys.stdout.fileno()
    os.set_blocking(stdin, False)
    os.set_blocking(stdout, False)
    buffer = bytearray()
    try:
        while True:
            with lock: engine.tick()
            readable, writable, _ = select.select([stdin, bus], [stdout] if output else [], [], .01)
            if stdout in writable:
                try: del output[:os.write(stdout, output)]
                except BlockingIOError: pass
            if bus in readable:
                for _ in range(256):
                    try:
                        packet, ancillary, flags, _ = bus.recvmsg(16, socket.CMSG_SPACE(helper.TIMESPEC.size))
                    except BlockingIOError: break
                    if len(packet) != 16:
                        raise GuardError("Malformed classic CAN packet")
                    can_id, size, data = struct.unpack("=IB3x8s", packet)
                    if can_id & 0xe0000000 or size != 8:
                        continue
                    source_ns, _ = clock_mapper.convert(helper._kernel_timestamp(ancillary, flags))
                    with lock: engine.ingest(can_id, data, source_ns/1e9)
            if stdin in readable:
                data = os.read(stdin, 4096)
                if not data:
                    with lock: engine.revoke("SSH stdin disconnected", hold=True)
                    return
                buffer.extend(data)
                if len(buffer) > MAX_LINE:
                    raise GuardError("RPC line limit exceeded")
                while b"\n" in buffer:
                    line, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    request = json.loads(line)
                    if not isinstance(request, dict) or request.get("protocol") != PROTOCOL:
                        raise GuardError("Invalid RPC envelope")
                    request_id = request.get("id")
                    if type(request_id) is not int:
                        raise GuardError("Integer RPC request id required")
                    try:
                        with lock: result = engine.rpc(request, time.monotonic_ns())
                        emit({"type": "reply", "id": request_id, "ok": True, "result": result})
                    except Exception as error:
                        emit({"type": "reply", "id": request_id, "ok": False, "error": str(error)})
    finally:
        stopped.set()
        with lock: engine.revoke("Bridge exiting", hold=True)
        bus.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--rx-helper", required=True, help="existing receive-only remote_can_source.py")
    parser.add_argument("--execution-policy", help="reviewed local JSON; omitted means RX only")
    parser.add_argument("--owner-path", default="/tmp/piper_policy_executor_can0.lock")
    args = parser.parse_args(argv)
    def terminate(_signum, _frame):
        # Unwind serve's finally block on a graceful process termination.
        # SIGKILL/board power loss cannot be handled by a Python watchdog.
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    policy = ExecutionPolicy() if args.execution_policy is None else policy_from_dict(
        json.loads(Path(args.execution_policy).read_text()))
    serve(args.interface, args.rx_helper, policy, args.owner_path)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

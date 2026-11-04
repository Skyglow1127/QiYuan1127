"""AIPC side of the independent deployment RPC, default read-only.

The only subprocess is authenticated SSH to the pinned KR260 endpoint. No
interface configuration, mode switching, enable or calibration is performed.
"""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
import logging
import math
import os
from pathlib import Path
import selectors
import shlex
import subprocess
import time

from .guard import ACTION_KEYS, GuardError, GuardLimits, vector
from .remote_bridge import PROTOCOL, readable_link
from .sdk_codec import encode_target, prepare_target_codec


class ReceiverClock:
    """Conservative four-timestamp bounds for this host/KR260 pair.

    During CPU inference the measured offset changed12.2ms in2.04s. Bound
    relative rate error by1% and resample every250ms. Wider uncertainty ages
    observations MORE and SHORTENS delivery deadlines; it never refreshes an
    old observation. RTT/age checks remain unchanged. This does not modify the
    collection adapter's clock. The physical cause of the rate error is open.
    """
    drift_rate = .01

    def __init__(self, max_rtt_ms=50, max_age_s=6):
        self.max_rtt_ns = int(max_rtt_ms*1e6)
        self.max_age_ns = int(max_age_s*1e9)
        self.lower = self.upper = self.synced_at = self.rtt_ns = None

    def update(self, sent, received, remote_receive, remote_send):
        duration, processing = received-sent, remote_send-remote_receive
        if not 0 <= processing <= duration or not 0 < duration <= self.max_rtt_ns:
            raise GuardError("Receiver clock RTT or timestamps invalid")
        lower, upper = remote_send-received, remote_receive-sent
        if self.synced_at is not None:
            elapsed = received-self.synced_at
            if elapsed < 0:
                raise GuardError("Host monotonic clock moved backwards")
            drift = int(elapsed*self.drift_rate)+1_000_000
            if lower > self.upper+drift or upper < self.lower-drift:
                raise GuardError(f"Receiver monotonic clock discontinuity: elapsed_ns={elapsed}, old_bounds=({self.lower},{self.upper}), new_bounds=({lower},{upper}), drift_ns={drift}")
        self.lower, self.upper = lower, upper
        self.synced_at, self.rtt_ns = received, duration-processing

    def host_time(self, remote_ns, received_ns):
        if self.synced_at is None or not 0 <= received_ns-self.synced_at <= self.max_age_ns:
            raise GuardError("Receiver source clock expired")
        mapped = remote_ns-self.upper-int((received_ns-self.synced_at)*self.drift_rate)
        if mapped > received_ns:
            raise GuardError("Receiver source timestamp is in the future")
        return mapped/1e9


class RPCClient:
    def __init__(self, command, expected_identity, *, execute=False, timeout_s=1., popen=subprocess.Popen):
        self.command, self.expected_identity = command, expected_identity
        self.execute, self.timeout_s, self.popen = execute, timeout_s, popen
        self.process = self.selector = None
        self.buffer, self.diagnostics = bytearray(), bytearray()
        self.next_id = 0
        self.hello = None
        self.source_clock = None

    def open(self):
        self.source_clock = ReceiverClock(max_rtt_ms=50)
        self.process = self.popen(self.command, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
            os.set_blocking(stream.fileno(), False)
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ, "stdout")
        self.selector.register(self.process.stderr, selectors.EVENT_READ, "stderr")
        try:
            hello = self._read_message(time.monotonic()+5.)
            if (hello.get("type") != "hello" or hello.get("identity") != self.expected_identity
                    or hello.get("execute") is not self.execute or not hello.get("kernel_timestamps")):
                raise GuardError("Deployment receiver identity/capabilities do not match")
            link = hello.get("link", {})
            if not readable_link(link):
                raise GuardError("Receiver CAN link must already be UP/1Mbps for read-only connection")
            self.hello = hello
            self.synchronize_clock()
        except BaseException:
            self.close()
            raise

    def _read_message(self, deadline):
        while True:
            if b"\n" in self.buffer:
                line, _, rest = self.buffer.partition(b"\n")
                self.buffer = bytearray(rest)
                message = json.loads(line)
                if not isinstance(message, dict) or message.get("protocol") != PROTOCOL:
                    raise GuardError("Invalid deployment RPC response")
                return message
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Deployment SSH RPC timed out")
            for key, _ in self.selector.select(min(.05, remaining)):
                data = os.read(key.fileobj.fileno(), 65536)
                if key.data == "stderr":
                    if data:
                        self.diagnostics = (self.diagnostics+data)[-2000:]
                    else:
                        self.selector.unregister(key.fileobj)
                    continue
                if not data:
                    raise GuardError("Deployment SSH disconnected: "+self.diagnostics.decode(errors="replace")[-500:])
                self.buffer.extend(data)
                if len(self.buffer) > 65536:
                    raise GuardError("Deployment SSH response buffer limit exceeded")

    def request(self, op, *, timeout_s=None, **payload):
        if self.process is None:
            raise GuardError("RPC not connected")
        self.next_id += 1
        request_id = self.next_id
        data = (json.dumps({"protocol": PROTOCOL, "id": request_id, "op": op, **payload},
            allow_nan=False, separators=(",", ":"))+"\n").encode()
        if len(data) > 16384:
            raise GuardError("RPC request too large")
        deadline = time.monotonic()+(self.timeout_s if timeout_s is None else timeout_s)
        offset = 0
        while offset < len(data):
            if time.monotonic() >= deadline:
                raise TimeoutError("Deployment SSH write timed out")
            try:
                written = os.write(self.process.stdin.fileno(), data[offset:])
                if not written:
                    raise GuardError("Deployment SSH stdin closed")
                offset += written
            except BlockingIOError:
                time.sleep(.001)
        message = self._read_message(deadline)
        if message.get("type") != "reply" or message.get("id") != request_id:
            raise GuardError("Unexpected/replayed RPC response")
        if message.get("ok") is not True:
            raise GuardError(message.get("error", "Receiver rejected request"))
        return message["result"]

    def synchronize_clock(self):
        sent = time.monotonic_ns()
        result = self.request("ping")
        received = time.monotonic_ns()
        self.source_clock.update(sent, received, result["remote_receive_ns"], result["remote_send_ns"])

    def require_clock(self):
        if self.source_clock.synced_at is None or time.monotonic_ns()-self.source_clock.synced_at > 250_000_000:
            self.synchronize_clock()

    def host_source_time(self, remote_ns):
        return self.source_clock.host_time(remote_ns, time.monotonic_ns())

    def remote_deadline(self, host_deadline):
        self.require_clock()
        age = time.monotonic_ns()-self.source_clock.synced_at
        if not 0 <= age <= self.source_clock.max_age_ns:
            raise GuardError("Receiver source clock is stale")
        # lower is remote-minus-host lower bound. Subtract drift to choose an
        # EARLIER remote deadline, never extend TTL by network uncertainty.
        return int(host_deadline*1e9)+self.source_clock.lower-int(age*ReceiverClock.drift_rate)

    def close(self):
        process, self.process = self.process, None
        if self.selector is not None:
            self.selector.close()
            self.selector = None
        if process is not None:
            # Closing stdin tells the independent receiver to revoke its lease.
            try: process.stdin.close()
            except OSError: pass
            try:
                process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try: process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=.5)
            for stream in (process.stdout, process.stderr):
                stream.close()


def ssh_command(config, io):
    transport = io["transport"]
    target = transport["ssh_host"]
    if not isinstance(target, str) or not target or target.startswith("-"):
        raise ValueError("Invalid SSH host")
    command = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
        "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=1",
        "-o", "ServerAliveCountMax=2", "-o", "LogLevel=ERROR"]
    if transport.get("ssh_hostname"):
        command += ["-o", "HostName="+transport["ssh_hostname"]]
    remote = ["python3", "-u", config["remote_bridge"], "--interface", io["interface"],
        "--rx-helper", config["remote_rx_helper"]]
    if config.get("execute", False):
        if not config.get("remote_execution_policy"):
            raise GuardError("Explicit reviewed receiver-side execution policy path required")
        remote += ["--execution-policy", config["remote_execution_policy"]]
    return command+[target, shlex.join(remote)]


class PiperBackend:
    def __init__(self, config, io, *, client=None):
        self.config, self.io = config, io
        self.execution_enabled = config.get("execute", False)
        if type(self.execution_enabled) is not bool:
            raise ValueError("execute must be a JSON boolean")
        self.tx_ttl_s = config.get("tx_ttl_s")
        if self.execution_enabled and (
                isinstance(self.tx_ttl_s, bool)
                or not isinstance(self.tx_ttl_s, (int, float))
                or not math.isfinite(self.tx_ttl_s)
                or not 0 < self.tx_ttl_s <= .25):
            raise ValueError("Execution requires explicit finite 0 < tx_ttl_s <= 0.25")
        self.guard_limits = GuardLimits(**config["guard_limits"]) if config.get("guard_limits") else None
        expected = json.loads(json.dumps(io["transport"]["expected_identity"]))
        if "expected_ifindex" in config:
            expected["ifindex"] = config["expected_ifindex"]
        self.client = client or RPCClient(ssh_command(config, io), expected,
            execute=self.execution_enabled, timeout_s=config.get("rpc_timeout_s", 1.))
        self.cameras = {}
        self.pool = None
        self.lease = None
        self.sequence = 0
        self.connected = False
        self.target_codec_preparation = None

    @classmethod
    def from_file(cls, path):
        config = json.loads(Path(path).read_text())
        io = json.loads(Path(config["io_config"]).read_text())
        return cls(config, io)

    def open_read_only(self):
        # execute=True still opens UNARMED. No mode/enable packet is sent.
        try:
            if self.execution_enabled:
                self.target_codec_preparation = prepare_target_codec()
            from lerobot_robot_piper_shared.devices import PiperHttpCamera, PiperHttpCameraConfig
            self.client.open()
            for role in ("global", "wrist"):
                camera = PiperHttpCamera(PiperHttpCameraConfig(role=role,
                    url=self.io["cameras"][role]["url"], serial=self.io["cameras"][role]["serial"]))
                self.cameras[role] = camera
                camera.connect()
            self.pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="piper-deploy-camera")
            self.connected = True
        except BaseException:
            self.close()
            raise

    def _read_camera_pair(self):
        images = {}
        for _ in range(3):
            futures = {role: self.pool.submit(camera.async_read) for role, camera in self.cameras.items()}
            errors = []
            for role, future in futures.items():
                try: images[role] = future.result()
                except Exception as error:
                    camera = self.cameras[role]
                    errors.append(GuardError(f'{role} camera acquisition failed '
                        f'(serial={getattr(camera.config,"serial","unknown")}, '
                        f'last_index={getattr(camera,"last_index",None)}): '
                        f'{type(error).__name__}: {error}'))
            if errors:
                raise errors[0]
            times = [camera.last_metadata["estimated_capture_monotonic"] for camera in self.cameras.values()]
            now = time.monotonic()
            if max(times)-min(times) <= .060 and all(0 <= now-t <= .150 for t in times):
                return images, min(times)
        raise GuardError("Camera pair exceeded 60ms skew/150ms frame age")

    def observation(self):
        from .robot import Observation
        if not self.connected:
            raise GuardError("Backend not connected")
        self.client.require_clock()
        images, camera_at = self._read_camera_pair()
        snapshot = self.client.request("snapshot")
        source_at = self.client.host_source_time(snapshot["source_ns"])
        now = time.monotonic()
        if not 0 <= now-source_at <= .150 or not 0 <= now-camera_at <= .150:
            raise GuardError("State/camera acquisition became stale")
        state = vector(snapshot["state"])
        return Observation({**dict(zip(ACTION_KEYS, state, strict=True)), **images},
            min(source_at, camera_at))

    def require_execution_lease(self, readiness):
        if not self.execution_enabled or self.guard_limits is None:
            raise GuardError("Read-only backend: execution/limits not configured")
        readiness.require_complete()
        if self.target_codec_preparation is None:
            self.target_codec_preparation = prepare_target_codec()
        self.client.require_clock()
        result = self.client.request("arm", readiness=asdict(readiness))
        self.lease, self.sequence = result["lease"], 0

    def refresh_unarmed_clock(self):
        """Start a new source-time epoch before acquiring fresh arm feedback.

        Initialization can be idle for seconds; no old observation or deadline
        is reused. Once a lease exists, clock continuity cannot be reset.
        """
        if self.lease is not None:
            raise GuardError("Cannot reset source clock during an execution lease")
        self.client.source_clock = ReceiverClock(max_rtt_ms=50)
        self.client.synchronize_clock()

    def send_checked(self, command):
        if not self.execution_enabled or self.lease is None:
            raise GuardError("No armed execution lease")
        force = self.config.get("normal_gripper_force_n")
        if force is None:
            raise GuardError("Explicit reviewed normal gripper force required")
        frames = encode_target(command.target, gripper_force_n=force)
        now = time.monotonic()
        if not math.isfinite(command.expires_at):
            raise GuardError("Host command has an invalid expiration time")
        # Source-observation/proposal age remains governed by command.expires_at.
        # The receiver's delivery TTL is a separate, shorter reviewed window;
        # never refresh the source time or extend its original deadline.
        send_deadline = min(command.expires_at, now+self.tx_ttl_s)
        if send_deadline <= now:
            raise GuardError("Host command expired before RPC (after encoding; "
                f"budget_at_creation_ms={(command.expires_at-command.created_at)*1000:.2f}, "
                f"elapsed_since_creation_ms={(now-command.created_at)*1000:.2f})")
        deadline = self.client.remote_deadline(send_deadline)
        remaining = send_deadline-time.monotonic()
        if remaining <= 0:
            raise GuardError("Host command expired before RPC (after clock mapping; "
                f"budget_at_creation_ms={(command.expires_at-command.created_at)*1000:.2f}, "
                f"elapsed_since_creation_ms={(time.monotonic()-command.created_at)*1000:.2f})")
        self.sequence += 1
        result = self.client.request("target", timeout_s=min(remaining, .25), lease=self.lease,
            sequence=self.sequence, deadline_ns=deadline,
            frames=[{"id": f.arbitration_id, "data": f.data.hex()} for f in frames])
        if result.get("accepted_sequence") != self.sequence:
            raise GuardError("Receiver did not accept the complete target sequence")
        logging.getLogger(__name__).info("ACT target accepted sequence=%d target=%s", self.sequence, command.target)

    def invalidate_execution_lease(self, reason):
        if self.lease is not None:
            self.lease = None
            try: self.client.request("disarm", timeout_s=.2)
            except Exception:
                # The receiver's own short lease supervises broken SSH. Do not
                # auto-reset/disable or claim a physical stop was confirmed.
                self.client.close()

    def close(self):
        try:
            self.invalidate_execution_lease("backend close")
        finally:
            if self.pool is not None:
                self.pool.shutdown(wait=True, cancel_futures=True)
                self.pool = None
            for camera in self.cameras.values(): camera.disconnect()
            self.cameras = {}
            self.client.close()
            self.connected = False

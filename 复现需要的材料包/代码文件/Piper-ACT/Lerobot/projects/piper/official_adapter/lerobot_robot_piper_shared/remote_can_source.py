#!/usr/bin/env python3
"""Receive-only SocketCAN source for an SSH stdio connection.

This file is intentionally standalone: the KR260 only needs Python's standard
library and the existing ``ip`` command.  Stdout contains JSONL exclusively;
human-readable diagnostics go to stderr.  The source never transmits CAN frames
or changes the interface state, bitrate, mode, or device configuration.
"""

import argparse
import json
import os
from pathlib import Path
import re
import select
import signal
import socket
import struct
import subprocess
import sys
import time


PROTOCOL = "piper-can-rx-v1"
FRAME_BYTES = 16
ARPHRD_CAN = 280
BATCH_NS = 2_000_000
STATUS_NS = 500_000_000
MAX_BATCH_FRAMES = 256
MAX_STDIN_BYTES = 65_536
MAX_CLOCK_CHANGE_NS = 5_000_000
MAX_CLOCK_SAMPLE_ATTEMPTS = 5
# Linux asm-generic/socket.h defines SO_TIMESTAMPNS_OLD=35.  Its old timespec
# contains two native longs, which this source explicitly restricts to 64 bits.
# It is valid on the KR260's aarch64 userspace and 64-bit x86 Linux hosts.
SO_TIMESTAMPNS_OLD = 35
TIMESPEC = struct.Struct("@ll")


class _Terminated(BaseException):
    """Interrupt select/recvmsg/stdout writes promptly on process termination."""


def _terminate(_signum, _frame):
    raise _Terminated()


def emit(message):
    """Write and flush one protocol message, allowing SSH pipe backpressure."""
    item = {"protocol": PROTOCOL, **message}
    sys.stdout.write(json.dumps(item, separators=(",", ":"), allow_nan=False) + "\n")
    sys.stdout.flush()


def _read_text(path):
    try:
        return Path(path).read_text(encoding="utf-8").strip()
    except (FileNotFoundError, PermissionError, OSError):
        return None


def validate_interface(interface):
    # Linux IFNAMSIZ is 16 including its terminator.  A deliberately narrow
    # character set also makes the ip argument and sysfs path unambiguous.
    if not isinstance(interface, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,14}", interface):
        raise ValueError("invalid CAN interface name")
    net = Path("/sys/class/net") / interface
    if not net.is_dir():
        raise ValueError("CAN interface does not exist: " + interface)
    if _read_text(net / "type") != str(ARPHRD_CAN):
        raise ValueError("interface is not an ARPHRD_CAN device: " + interface)
    return net


def interface_identity(interface):
    net = validate_interface(interface)
    device = (net / "device").resolve()
    driver = device / "driver"
    usb = dict.fromkeys(("idVendor", "idProduct", "serial", "manufacturer", "product"))
    # A gs_usb network device may sit beneath a USB interface directory.  Read
    # only the first ancestor carrying the USB device's vendor/product identity.
    for candidate in (device, *device.parents):
        vendor = _read_text(candidate / "idVendor")
        product_id = _read_text(candidate / "idProduct")
        if vendor is not None and product_id is not None:
            usb = {key: _read_text(candidate / key) for key in usb}
            break
    machine_id = _read_text("/etc/machine-id") or _read_text("/var/lib/dbus/machine-id")
    return {
        "interface": interface,
        "ifindex": int((net / "ifindex").read_text(encoding="ascii").strip()),
        "device_path": str(device),
        "host": {"hostname": socket.gethostname(), "machine_id": machine_id},
        "driver": driver.resolve().name if driver.is_symlink() else None,
        "usb": usb,
    }


def link_status(interface):
    """Mirror the local adapter's UP + LISTEN-ONLY + 1 Mbps readiness gate."""
    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", "dev", interface],
            capture_output=True, text=True, timeout=2, check=False,
        )
        detail = result.stdout.strip() or result.stderr.strip()
        up = result.returncode == 0 and "state UP" in detail
    except (OSError, subprocess.TimeoutExpired) as error:
        detail, up = str(error), False
    listen_only = "LISTEN-ONLY" in detail.upper()
    bitrate_ok = "bitrate 1000000 " in detail
    return {
        "up": up, "listen_only": listen_only, "bitrate_ok": bitrate_ok,
        "detail": detail, "ready": up and listen_only and bitrate_ok,
    }


class ClockMapper:
    """Map kernel realtime stamps to a conservative source-monotonic instant.

    A userspace receive timestamp would hide frames queued while stdout blocks.
    SO_TIMESTAMPNS timestamps kernel reception instead.  Sampling realtime
    between two monotonic reads bounds the clock-conversion uncertainty; using
    the earlier monotonic read gives the earliest plausible receive instant.
    """

    def __init__(self):
        self.previous_offset_ns = None

    def sample(self):
        # Scheduling between clock reads widens the conversion interval; it is
        # not evidence that either clock jumped. Discard that calibration and
        # resample briefly, keeping the same accepted uncertainty bound.
        for _ in range(MAX_CLOCK_SAMPLE_ATTEMPTS):
            before = time.monotonic_ns()
            realtime = time.time_ns()
            after = time.monotonic_ns()
            uncertainty = after - before
            if uncertainty < 0:
                raise RuntimeError("source monotonic clock moved backwards")
            if uncertainty <= MAX_CLOCK_CHANGE_NS:
                break
        else:
            raise RuntimeError(
                "clock calibration uncertainty exceeds 5 ms after "
                f"{MAX_CLOCK_SAMPLE_ATTEMPTS} samples (last={uncertainty / 1e6:.3f} ms)")
        offset = realtime - ((before + after) // 2)
        if (self.previous_offset_ns is not None and
                abs(offset - self.previous_offset_ns) > MAX_CLOCK_CHANGE_NS):
            raise RuntimeError("realtime-to-monotonic clock offset jumped by more than 5 ms")
        self.previous_offset_ns = offset
        return before, realtime, uncertainty

    def convert(self, kernel_realtime_ns):
        before, realtime, uncertainty = self.sample()
        if kernel_realtime_ns > realtime:
            raise RuntimeError("CAN kernel receive timestamp is in the future")
        source_ns = before + kernel_realtime_ns - realtime
        if source_ns <= 0:
            raise RuntimeError("CAN kernel receive timestamp predates this boot")
        return source_ns, uncertainty


def _kernel_timestamp(ancillary, message_flags):
    if message_flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
        raise RuntimeError("CAN frame or kernel timestamp was truncated")
    timestamps = [data for level, kind, data in ancillary
                  if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS_OLD]
    if len(timestamps) != 1 or len(timestamps[0]) != TIMESPEC.size:
        raise RuntimeError("required SO_TIMESTAMPNS kernel receive timestamp is missing")
    seconds, nanoseconds = TIMESPEC.unpack(timestamps[0])
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise RuntimeError("invalid CAN kernel receive timestamp")
    return seconds * 1_000_000_000 + nanoseconds


def _handle_stdin(data, buffer, received_ns):
    buffer += data
    # Limit complete lines as well as partial lines, so an untrusted peer cannot
    # make either the JSON parser or the accumulated input use unbounded memory.
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        if len(line) > MAX_STDIN_BYTES:
            raise ValueError("stdin protocol line is too long")
        if not line.strip():
            continue
        message = json.loads(line.decode("utf-8"))
        if (not isinstance(message, dict) or message.get("type") != "ping" or
                not isinstance(message.get("token"), str)):
            raise ValueError("stdin accepts only ping messages with a string token")
        if message.get("protocol", PROTOCOL) != PROTOCOL:
            raise ValueError("unsupported stdin protocol")
        emit({"type": "pong", "token": message["token"],
              "remote_receive_ns": received_ns, "remote_send_ns": time.monotonic_ns()})
    if len(buffer) > MAX_STDIN_BYTES:
        raise ValueError("stdin protocol line is too long")
    return buffer


def stream(interface, clock):
    if struct.calcsize("@l") != 8 or TIMESPEC.size != 16:
        raise RuntimeError("SO_TIMESTAMPNS_OLD requires a 64-bit native-long Linux userspace")
    sequence, frames, uncertainty_ns = 0, [], 0
    stdin_buffer = b""
    stdin_fd = sys.stdin.fileno()
    batch_deadline = None
    status_deadline = time.monotonic_ns() + STATUS_NS
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        # Receive timestamping is the only socket option changed.  In particular,
        # this program has no CAN transmission API, SDK, or subprocess command.
        bus.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS_OLD, 1)
        bus.bind((interface,))
        bus.setblocking(False)
        while True:
            now = time.monotonic_ns()
            if frames and (now >= batch_deadline or len(frames) >= MAX_BATCH_FRAMES):
                emit({"type": "frames", "frames": frames,
                      "timestamp_uncertainty_ns": uncertainty_ns})
                frames, uncertainty_ns, batch_deadline = [], 0, None
                now = time.monotonic_ns()
            if now >= status_deadline:
                # A status query can take longer than the normal 2 ms batch
                # period; publish frames collected before it first.
                if frames:
                    emit({"type": "frames", "frames": frames,
                          "timestamp_uncertainty_ns": uncertainty_ns})
                    frames, uncertainty_ns, batch_deadline = [], 0, None
                clock.sample()
                link = link_status(interface)
                emit({"type": "status", "link": link})
                if not link["ready"]:
                    raise RuntimeError("CAN link left UP + LISTEN-ONLY + 1 Mbps state")
                status_deadline = time.monotonic_ns() + STATUS_NS
                now = time.monotonic_ns()
            deadline = min(status_deadline, batch_deadline or status_deadline)
            readable, _, _ = select.select([stdin_fd, bus], [], [], max(0, deadline - now) / 1e9)
            if stdin_fd in readable:
                data = os.read(stdin_fd, 4096)
                received_ns = time.monotonic_ns()
                if not data:
                    return
                stdin_buffer = _handle_stdin(data, stdin_buffer, received_ns)
            if bus in readable:
                try:
                    packet, ancillary, flags, _ = bus.recvmsg(
                        FRAME_BYTES, socket.CMSG_SPACE(TIMESPEC.size))
                except BlockingIOError:
                    continue
                if len(packet) != FRAME_BYTES or packet[4] > 8:
                    raise RuntimeError("received a malformed or non-classic CAN frame")
                source_ns, uncertainty = clock.convert(_kernel_timestamp(ancillary, flags))
                sequence += 1
                frames.append([sequence, source_ns, packet.hex()])
                uncertainty_ns = max(uncertainty_ns, uncertainty)
                if batch_deadline is None:
                    batch_deadline = time.monotonic_ns() + BATCH_NS


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0", help="existing receive-only SocketCAN interface")
    parser.add_argument("--probe", action="store_true", help="emit one hello without opening a CAN socket")
    args = parser.parse_args(argv)
    signal.signal(signal.SIGTERM, _terminate)
    signal.signal(signal.SIGINT, _terminate)
    try:
        identity = interface_identity(args.interface)
        link = link_status(args.interface)
        clock = ClockMapper()
        _, _, uncertainty = clock.sample()
        emit({"type": "hello", "identity": identity, "link": link,
              "timestamping": {
                  "kernel": "SO_TIMESTAMPNS_OLD realtime_ns",
                  "source_clock": "monotonic_ns",
                  "mapping": "monotonic_before_ns + kernel_realtime_ns - sampled_realtime_ns",
                  "bound": "earliest plausible kernel receive instant; includes kernel queue age",
                  "initial_uncertainty_ns": uncertainty,
                  "batch_uncertainty_field": "timestamp_uncertainty_ns",
                  "max_clock_change_ns": MAX_CLOCK_CHANGE_NS,
              }})
        if args.probe:
            return 0
        if not link["ready"]:
            raise RuntimeError("CAN link must be UP + LISTEN-ONLY + 1 Mbps before streaming")
        stream(args.interface, clock)
        return 0
    except (_Terminated, KeyboardInterrupt):
        return 0
    except BrokenPipeError:
        # Suppress a second failed flush when Python exits after SSH disconnects.
        with open(os.devnull, "w") as sink:
            os.dup2(sink.fileno(), sys.stdout.fileno())
        return 0
    except Exception as error:
        message = str(error) or type(error).__name__
        print("remote CAN source: " + message, file=sys.stderr, flush=True)
        try:
            emit({"type": "error", "message": message})
        except (BrokenPipeError, OSError):
            pass
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Read two *digital* comparator outputs. No ADC or pressure-unit conversion.

Compatible with the official distribution-provided libgpiod Python v1 and v2
bindings. May run as ``python3 -u - < board_reader.py`` over SSH. Pin offsets
must come from the board's actual device tree; no pin assignment is guessed.
"""
import argparse
import datetime
import glob
import importlib
import json
import math
import os
import platform
from pathlib import Path
import struct
import stat
import re
import shutil
import signal
import subprocess
import sys
import threading
import time


class ReaderError(Exception):
    pass


def verify_pl_mapping(chip_path='auto', offsets=(0, 1), dt_root='/sys/firmware/devicetree/base',
                      gpio_root='/sys/bus/gpio/devices', dev_root='/dev'):
    """Verify the existing r2 PL AXI GPIO via DT and its exact character device."""
    if list(offsets) != [0, 1]:
        raise ReaderError('r2 requires DO1/DO2 offsets 0/1 in that order')
    node = Path(dt_root) / 'axi/gpio@a0000000'
    if not node.is_dir():
        raise ReaderError('未发现 r2 PL AXI GPIO（/axi/gpio@a0000000）。当前固件未提供压力接口；本程序不会自动重载 FPGA。')
    firmware = Path(dt_root) / 'fpga-region/firmware-name'
    if firmware.read_bytes().rstrip(b'\0') != b'kr260-fsr.bit.bin':
        raise ReaderError('FPGA region firmware identity does not match the existing pressure design')
    if (node / 'status').exists() and (node / 'status').read_bytes().rstrip(b'\0') not in (b'okay', b'ok'):
        raise ReaderError('Pressure GPIO device-tree node is disabled')
    if b'xlnx,xps-gpio-1.00.a' not in (node / 'compatible').read_bytes().split(b'\0'):
        raise ReaderError('Target is not Xilinx PL AXI GPIO')
    expected = {'reg': [0, 0xa0000000, 0, 0x10000], 'xlnx,gpio-width': [2],
                'xlnx,all-inputs': [1], 'xlnx,is-dual': [0], '#gpio-cells': [2]}
    for key, values in expected.items():
        data = (node / key).read_bytes()
        if data != struct.pack('>' + 'I' * len(values), *values):
            raise ReaderError('PL device-tree mismatch: ' + key)
    if not (node / 'gpio-controller').is_file():
        raise ReaderError('Missing GPIO controller property')
    if (node / 'gpio-line-names').read_bytes().rstrip(b'\0').split(b'\0') != [b'FSR_DO1', b'FSR_DO2']:
        raise ReaderError('Pressure line names/order do not match r2')
    candidates = []
    for chip in Path(gpio_root).glob('gpiochip*'):
        if (chip / 'of_node').resolve() != node.resolve():
            continue
        major, minor = map(int, (chip / 'dev').read_text().strip().split(':'))
        for device in Path(dev_root).glob('gpiochip*'):
            info = device.stat()
            if stat.S_ISCHR(info.st_mode) and (os.major(info.st_rdev), os.minor(info.st_rdev)) == (major, minor):
                candidates.append(str(device))
    if len(candidates) != 1 or (chip_path != 'auto' and chip_path != candidates[0]):
        raise ReaderError('PL GPIO character device missing, ambiguous, or different from selection')
    return dict(type='pl_mapping', verified=True, chip=candidates[0], offsets=[0, 1],
                of_node='/axi/gpio@a0000000', address='0xa0000000', width=2,
                line_names=['FSR_DO1', 'FSR_DO2'], pipeline='DO → PL synchronizer → AXI GPIO → PS Linux')


class JsonArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        raise ReaderError(message)


def emit(value, stream):
    print(json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False),
          file=stream, flush=True)


def load_gpiod():
    try:
        module = importlib.import_module("gpiod")
    except ImportError as exc:
        raise ReaderError("Cannot import system gpiod for Python 3. Install the board distribution's python3-libgpiod package; real acquisition has stopped.") from exc
    if not hasattr(module, "Chip"):
        raise ReaderError("Unsupported gpiod module: official libgpiod Python bindings are required.")
    return module


def api_version(module):
    if hasattr(module, "request_lines") and hasattr(module, "LineSettings"):
        return 2
    if hasattr(module, "LINE_REQ_DIR_IN"):
        return 1
    raise ReaderError("Unsupported gpiod Python API; expected official libgpiod v1 or v2 bindings.")


def line_enums(module):
    enums = getattr(module, "line", None)
    return enums if enums is not None else importlib.import_module("gpiod.line")


def _v1_direction(line):
    direction = line.direction()
    if direction == line.DIRECTION_INPUT:
        return "input"
    if direction == line.DIRECTION_OUTPUT:
        return "output"
    return "unknown"


def _line_info(chip, offset, version, module):
    if version == 2:
        info = chip.get_line_info(offset)
        enums = line_enums(module)
        direction = ("input" if info.direction == enums.Direction.INPUT else
                     "output" if info.direction == enums.Direction.OUTPUT else "unknown")
        return {"offset": offset, "name": info.name, "consumer": info.consumer,
                "direction": direction, "used": bool(info.used),
                "active_low": bool(info.active_low)}
    line = chip.get_line(offset)
    return {"offset": offset, "name": line.name(), "consumer": line.consumer(),
            "direction": _v1_direction(line), "used": bool(line.is_used()),
            "active_low": line.active_state() == line.ACTIVE_LOW}


def _chip_info(chip, version):
    if version == 2:
        info = chip.get_info()
        return {"name": info.name, "label": info.label, "num_lines": info.num_lines}
    return {"name": chip.name(), "label": chip.label(), "num_lines": chip.num_lines()}


def inspect_chip(module, path, offsets=None):
    """Metadata ioctls only. Never request lines or change their configuration."""
    version = api_version(module)
    chip = module.Chip(path)
    try:
        data = {"path": path, **_chip_info(chip, version)}
        wanted = range(data["num_lines"]) if offsets is None else offsets
        data["lines"] = []
        for offset in wanted:
            if offset < 0 or offset >= data["num_lines"]:
                raise ReaderError("GPIO offset %s outside %s line range 0..%s" %
                                  (offset, path, data["num_lines"] - 1))
            data["lines"].append(_line_info(chip, offset, version, module))
        return data
    finally:
        chip.close()


class RealReader:
    def __init__(self, chip_path, offsets, module=None):
        self.module = module if module is not None else load_gpiod()
        self.version = api_version(self.module)
        self.path, self.offsets = chip_path, tuple(offsets)
        self.chip = None
        self.request = None
        self.requested = False

    def __enter__(self):
        try:
            self.chip = self.module.Chip(self.path)
            info = _chip_info(self.chip, self.version)
            for offset in self.offsets:
                if offset < 0 or offset >= info["num_lines"]:
                    raise ReaderError("Offset %s outside %s line range 0..%s" %
                                      (offset, self.path, info["num_lines"] - 1))
                line = _line_info(self.chip, offset, self.version, self.module)
                if line["used"] or line["consumer"]:
                    raise ReaderError("Refusing busy line %s:%s (consumer=%r)." %
                                      (self.path, offset, line["consumer"]))
                if line["direction"] != "input":
                    raise ReaderError("Refusing line %s:%s with direction %s; verify the device tree/pin assignment first." %
                                      (self.path, offset, line["direction"]))
            if self.version == 2:
                enums = line_enums(self.module)
                settings = self.module.LineSettings(direction=enums.Direction.INPUT,
                                                    active_low=False)
                # AS_IS bias is the libgpiod default: preserve the board's bias.
                self.request = self.module.request_lines(
                    self.path, consumer="kr260-pressure-reader",
                    config={self.offsets: settings})
                self.requested = True
            else:
                self.request = self.chip.get_lines(list(self.offsets))
                self.request.request(consumer="kr260-pressure-reader",
                                     type=self.module.LINE_REQ_DIR_IN, flags=0)
                self.requested = True
            return self
        except BaseException:
            self.close()
            raise

    def read(self):
        if self.version == 2:
            values = self.request.get_values(list(self.offsets))
            value_enum = line_enums(self.module).Value
            result = []
            for value in values:
                if value == value_enum.ACTIVE:
                    result.append(1)
                elif value == value_enum.INACTIVE:
                    result.append(0)
                else:
                    raise ReaderError("Unexpected libgpiod v2 value: %r" % (value,))
        else:
            values = self.request.get_values()
            result = []
            for value in values:
                if not isinstance(value, int) or value not in (0, 1):
                    raise ReaderError("Unexpected libgpiod v1 value: %r" % (value,))
                result.append(int(value))
        if len(result) != 2:
            raise ReaderError("Expected two GPIO values, received %s" % len(result))
        return result

    def close(self):
        try:
            if self.request is not None and self.requested:
                self.requested = False
                self.request.release()
        finally:
            self.request = None
            if self.chip is not None:
                chip, self.chip = self.chip, None
                chip.close()

    def __exit__(self, exc_type, exc, traceback):
        self.close()


class DemoReader:
    """A deterministic digital pulse pattern, explicitly marked simulated."""
    def __init__(self, rate_hz):
        self.index = 0
        self.rate_hz = rate_hz

    def __enter__(self):
        return self

    def read(self):
        t_s = self.index / self.rate_hz
        self.index += 1
        return [int((t_s % 4.0) < 2.0), int((t_s % 6.0) < 1.5)]

    def __exit__(self, exc_type, exc, traceback):
        pass


def _tool_readonly(name, args):
    path = shutil.which(name)
    if not path:
        return {"available": False}
    try:
        result = subprocess.run([path] + args, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, universal_newlines=True,
                                timeout=6, check=False)
        return {"available": True, "returncode": result.returncode,
                "stdout": result.stdout[:24000], "stderr": result.stderr[:4000]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"available": True, "error": str(exc)}


def probe(chip_path=None):
    data = {"type": "probe", "read_only": True,
            "python": sys.version.split()[0], "platform": platform.platform(),
            "hostname": platform.node(), "uid": os.getuid(), "groups": os.getgroups(),
            "chips": []}
    try:
        data['pressure_pl'] = verify_pl_mapping()
    except Exception as exc:
        data['pressure_pl'] = {'verified': False, 'error': str(exc)}
    paths = [chip_path] if chip_path else sorted(glob.glob("/dev/gpiochip*"))
    errors = False
    try:
        module = load_gpiod()
        data["gpiod_api"] = api_version(module)
        data["gpiod_version"] = str(getattr(module, "__version__", "system binding; not exposed"))
    except Exception as exc:
        data["gpiod_error"] = str(exc)
        data["device_paths"] = paths
        # gpioinfo/gpiodetect inspect metadata without requesting lines, on v1/v2.
        data["gpiodetect"] = _tool_readonly("gpiodetect", [])
        data["gpioinfo"] = _tool_readonly("gpioinfo", [])
        return data, 1
    for path in paths:
        try:
            data["chips"].append(inspect_chip(module, path))
        except Exception as exc:
            data["chips"].append({"path": path, "error": str(exc)})
            errors = True
    return data, int(errors)


def parse_args(argv):
    parser = JsonArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("real", "demo"), default="real")
    parser.add_argument("--chip", help="Verified GPIO character device, e.g. /dev/gpiochip2")
    parser.add_argument("--offsets", nargs=2, type=int, metavar=("DO1", "DO2"),
                        help="Verified chip-relative line offsets, in channel order")
    parser.add_argument("--rate-hz", type=float, default=100.0,
                        help="Requested polling rate 1..500 Hz; Linux scheduling is not realtime")
    parser.add_argument("--probe", action="store_true", help="Read metadata only; never request GPIO lines")
    parser.add_argument("--verify-pl", action="store_true", help="Require verified r2 PL AXI GPIO and report mapping")
    parser.add_argument("--samples", type=int, default=0, help="Stop after N samples; 0 means continuous")
    args = parser.parse_args(argv)
    if not math.isfinite(args.rate_hz) or not 1 <= args.rate_hz <= 500:
        parser.error("--rate-hz must be a finite number from 1 to 500")
    if args.samples < 0:
        parser.error("--samples must be nonnegative")
    if args.chip and not (args.chip == 'auto' and args.verify_pl) and not re.fullmatch(r"/dev/gpiochip[0-9]+", args.chip):
        parser.error("--chip must be an explicit /dev/gpiochipN path")
    if args.offsets is not None and (min(args.offsets) < 0 or args.offsets[0] == args.offsets[1]):
        parser.error("--offsets requires two different nonnegative offsets")
    if not args.probe and args.source == "real" and (not args.chip or args.offsets is None):
        parser.error("real acquisition requires verified --chip and --offsets")
    if args.offsets is None:
        args.offsets = [0, 1]  # Demo only; real acquisition was checked above.
    return args


def stream_samples(reader, args, output, stop_event):
    period = 1.0 / args.rate_hz
    start = time.monotonic()
    previous = None
    deadline = start
    seq = 0
    while not stop_event.is_set():
        wait = deadline - time.monotonic()
        if wait > 0 and stop_event.wait(wait):
            break
        if stop_event.is_set():
            break
        values = reader.read()
        now = time.monotonic()
        seq += 1
        emit({"seq": seq, "t_s": now - start,
              "wall_time": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "do1": values[0], "do2": values[1],
              "source": "real" if args.source == "real" else "simulated",
              "chip": args.chip if args.source == "real" else "demo",
              "offsets": args.offsets, "dt_s": 0.0 if previous is None else now - previous}, output)
        previous = now
        if args.samples and seq >= args.samples:
            break
        deadline += period
        after_emit = time.monotonic()
        if deadline <= after_emit:
            # Skip missed polling slots; do not burst-read to catch up.
            deadline += (math.floor((after_emit - deadline) / period) + 1) * period


def main(argv=None, stdout=None, stderr=None):
    output = sys.stdout if stdout is None else stdout
    errors = sys.stderr if stderr is None else stderr
    handlers = {}
    stop_event = threading.Event()
    try:
        args = parse_args(argv)
        if args.probe:
            data, code = probe(args.chip)
            emit(data, output)
            return code
        if args.source == 'real' and args.verify_pl:
            mapping = verify_pl_mapping(args.chip, args.offsets)
            args.chip = mapping['chip']
            emit(mapping, output)
        # Install before acquiring to ensure every termination path releases lines.
        if threading.current_thread() is threading.main_thread():
            for signum in (signal.SIGINT, signal.SIGTERM):
                handlers[signum] = signal.getsignal(signum)
                signal.signal(signum, lambda _signum, _frame: stop_event.set())
        reader = RealReader(args.chip, args.offsets) if args.source == "real" else DemoReader(args.rate_hz)
        with reader:
            stream_samples(reader, args, output, stop_event)
        return 0
    except BrokenPipeError:
        # Context manager above releases GPIO before this branch is reached.
        if output is sys.stdout:
            sys.stdout = open(os.devnull, "w")
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        emit({"type": "error", "error": str(exc)}, errors)
        return 1
    finally:
        for signum, handler in handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    sys.exit(main())

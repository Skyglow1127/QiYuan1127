"""Behavior checks using official v1/v2 API-shaped fakes, no GPIO hardware."""
import enum
import io
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import board_reader as reader


class Direction(enum.Enum):
    INPUT = 1
    OUTPUT = 2


class Value(enum.Enum):
    INACTIVE = 0
    ACTIVE = 1


def fake_v2(direction=Direction.INPUT, used=False, read_error=None):
    state = types.SimpleNamespace(closes=0, releases=0, requests=[], reads=[], values=[Value.INACTIVE, Value.ACTIVE])

    class Chip:
        def __init__(self, path):
            self.path = path

        def get_info(self):
            return types.SimpleNamespace(name="gpiochip2", label="axi_gpio", num_lines=2)

        def get_line_info(self, offset):
            return types.SimpleNamespace(name="DO" + str(offset + 1), consumer="busy" if used else None,
                                         direction=direction, used=used, active_low=True)

        def close(self):
            state.closes += 1

    class Request:
        def get_values(self, offsets):
            state.reads.append(offsets)
            if read_error:
                raise read_error
            return state.values

        def release(self):
            state.releases += 1

    def request_lines(path, **kwargs):
        state.requests.append((path, kwargs))
        return Request()

    module = types.SimpleNamespace(Chip=Chip, request_lines=request_lines,
                                   LineSettings=lambda **kwargs: kwargs,
                                   line=types.SimpleNamespace(Direction=Direction, Value=Value),
                                   __version__="2.mock")
    return module, state


def fake_v1(direction=1, used=False, request_error=None, read_error=None):
    state = types.SimpleNamespace(closes=0, releases=0, requests=[], reads=0, values=[0, 1], bulk_offsets=[])

    class Line:
        DIRECTION_INPUT = 1
        DIRECTION_OUTPUT = 2
        ACTIVE_LOW = 2

        def __init__(self, offset):
            self.offset = offset

        def name(self):
            return "DO" + str(self.offset + 1)

        def consumer(self):
            return "busy" if used else None

        def direction(self):
            return direction

        def is_used(self):
            return used

        def active_state(self):
            return 2

    class Bulk:
        def request(self, **kwargs):
            state.requests.append(kwargs)
            if request_error:
                raise request_error

        def get_values(self):
            state.reads += 1
            if read_error:
                raise read_error
            return state.values

        def release(self):
            state.releases += 1

    class Chip:
        def __init__(self, path):
            self.path = path

        def name(self):
            return "gpiochip2"

        def label(self):
            return "axi_gpio"

        def num_lines(self):
            return 2

        def get_line(self, offset):
            return Line(offset)

        def get_lines(self, offsets):
            state.bulk_offsets = offsets
            return Bulk()

        def close(self):
            state.closes += 1

    return types.SimpleNamespace(Chip=Chip, LINE_REQ_DIR_IN=2), state


class ReaderTests(unittest.TestCase):
    def test_v2_enum_is_not_converted_by_truthiness(self):
        module, state = fake_v2()
        with reader.RealReader("/dev/gpiochip2", [1, 0], module) as handle:
            self.assertEqual(handle.read(), [0, 1])
            self.assertEqual(handle.read(), [0, 1])
            self.assertEqual(state.releases, 0)
        self.assertEqual(len(state.requests), 1)
        self.assertEqual(state.reads, [[1, 0], [1, 0]])
        settings = state.requests[0][1]["config"][(1, 0)]
        self.assertEqual(settings, {"direction": Direction.INPUT, "active_low": False})
        self.assertEqual((state.releases, state.closes), (1, 1))

    def test_v1_bulk_input_preserves_channel_order(self):
        module, state = fake_v1()
        with reader.RealReader("/dev/gpiochip2", [1, 0], module) as handle:
            self.assertEqual(handle.read(), [0, 1])
            self.assertEqual(handle.read(), [0, 1])
        self.assertEqual(state.bulk_offsets, [1, 0])
        self.assertEqual(state.requests, [{"consumer": "kr260-pressure-reader", "type": 2, "flags": 0}])
        self.assertEqual((state.releases, state.closes), (1, 1))

    def test_refuses_busy_output_unknown_and_range_before_request(self):
        cases = [fake_v2(used=True), fake_v2(direction=Direction.OUTPUT),
                 fake_v2(direction=None), fake_v1(used=True), fake_v1(direction=2), fake_v1(direction=99)]
        for module, state in cases:
            with self.subTest(module=module):
                with self.assertRaises(reader.ReaderError):
                    with reader.RealReader("/dev/gpiochip2", [0, 1], module):
                        self.fail("must not acquire")
                self.assertEqual(state.requests, [])
                self.assertEqual(state.closes, 1)
        module, state = fake_v2()
        with self.assertRaises(reader.ReaderError):
            with reader.RealReader("/dev/gpiochip2", [0, 2], module):
                pass
        self.assertEqual(state.requests, [])
        self.assertEqual(state.closes, 1)

    def test_request_failure_closes_chip_without_releasing_unowned_bulk(self):
        module, state = fake_v1(request_error=OSError("line busy"))
        with self.assertRaises(OSError):
            with reader.RealReader("/dev/gpiochip2", [0, 1], module):
                pass
        self.assertEqual((state.releases, state.closes), (0, 1))

    def test_read_error_and_bad_values_release(self):
        for factory in (fake_v1, fake_v2):
            module, state = factory(read_error=OSError("read failed"))
            with self.assertRaises(OSError):
                with reader.RealReader("/dev/gpiochip2", [0, 1], module) as handle:
                    handle.read()
            self.assertEqual((state.releases, state.closes), (1, 1))
            module, state = factory()
            state.values = [9, 0]
            with self.assertRaises(reader.ReaderError):
                with reader.RealReader("/dev/gpiochip2", [0, 1], module) as handle:
                    handle.read()
            self.assertEqual((state.releases, state.closes), (1, 1))

    def test_readonly_probe_v1_v2_does_not_request(self):
        for factory in (fake_v1, fake_v2):
            module, state = factory(used=True)
            with mock.patch.object(reader, "load_gpiod", return_value=module), mock.patch.object(reader.glob, "glob", return_value=["/dev/gpiochip2"]):
                result, code = reader.probe()
            self.assertEqual(code, 0)
            self.assertTrue(result["read_only"])
            chip = result["chips"][0]
            self.assertEqual(chip["label"], "axi_gpio")
            self.assertEqual(chip["lines"][1]["name"], "DO2")
            self.assertEqual(chip["lines"][0]["consumer"], "busy")
            self.assertEqual(chip["lines"][0]["direction"], "input")
            self.assertEqual(state.requests, [])
            self.assertEqual(state.closes, 1)

    def test_real_error_never_falls_back(self):
        for exception in (ReaderErrorForTest("missing lib"), PermissionError("permission denied")):
            output, errors = io.StringIO(), io.StringIO()
            with mock.patch.object(reader, "load_gpiod", side_effect=exception), mock.patch.object(reader, "DemoReader") as demo:
                code = reader.main(["--chip", "/dev/gpiochip2", "--offsets", "0", "1", "--samples", "1"], output, errors)
            self.assertEqual(code, 1)
            self.assertEqual(output.getvalue(), "")
            self.assertEqual(json.loads(errors.getvalue())["type"], "error")
            demo.assert_not_called()

    def test_main_release_on_read_failure_broken_pipe_and_signal(self):
        module, state = fake_v2(read_error=OSError("disconnected"))
        with mock.patch.object(reader, "load_gpiod", return_value=module):
            code = reader.main(["--chip", "/dev/gpiochip2", "--offsets", "0", "1"], io.StringIO(), io.StringIO())
        self.assertEqual(code, 1)
        self.assertEqual((state.releases, state.closes), (1, 1))
        module, state = fake_v2()
        broken = mock.Mock()
        broken.write.side_effect = BrokenPipeError()
        with mock.patch.object(reader, "load_gpiod", return_value=module):
            code = reader.main(["--chip", "/dev/gpiochip2", "--offsets", "0", "1"], broken, io.StringIO())
        self.assertEqual(code, 0)
        self.assertEqual(state.releases, 1)
        for signum in (signal.SIGINT, signal.SIGTERM):
            module, state = fake_v2()
            previous = signal.getsignal(signum)
            def terminate(value, output):
                signal.getsignal(signum)(signum, None)
            with mock.patch.object(reader, "load_gpiod", return_value=module), mock.patch.object(reader, "emit", side_effect=terminate):
                code = reader.main(["--chip", "/dev/gpiochip2", "--offsets", "0", "1"], io.StringIO(), io.StringIO())
            self.assertEqual(code, 0)
            self.assertEqual(state.releases, 1)
            self.assertIs(signal.getsignal(signum), previous)

    def test_argument_validation_reports_json(self):
        for args in ([], ["--chip", "/dev/gpiochip2"], ["--source", "demo", "--rate-hz", "nan"],
                     ["--source", "demo", "--offsets", "1", "1"], ["--source", "demo", "--rate-hz", "501"],
                     ["--source", "demo", "--offsets", "-1", "2"], ["--chip", "gpiochip2", "--offsets", "0", "1"]):
            output, errors = io.StringIO(), io.StringIO()
            self.assertEqual(reader.main(args, output, errors), 1)
            self.assertEqual(json.loads(errors.getvalue())["type"], "error")

    def test_samples_schema_and_stdin_script(self):
        program = Path(reader.__file__).read_text()
        result = subprocess.run([sys.executable, "-u", "-", "--source", "demo", "--samples", "2", "--rate-hz", "500"],
                                input=program, text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual(len(rows), 2)
        self.assertEqual(set(rows[0]), {"seq", "t_s", "wall_time", "do1", "do2", "source", "chip", "offsets", "dt_s"})
        self.assertEqual([row["seq"] for row in rows], [1, 2])
        self.assertTrue(all(row["source"] == "simulated" for row in rows))
        self.assertGreater(rows[1]["t_s"], rows[0]["t_s"])
        self.assertEqual(rows[0]["dt_s"], 0)
        self.assertTrue(rows[0]["wall_time"].endswith("+00:00"))

    def test_slow_reads_skip_missed_deadlines(self):
        clock = [0.0]
        waits = []
        class Stop:
            def is_set(self):
                return False
            def wait(self, delay):
                waits.append(delay)
                clock[0] += delay
                return False
        class SlowReader:
            def read(self):
                clock[0] += 0.25
                return [0, 1]
        args = reader.parse_args(["--source", "demo", "--samples", "3", "--rate-hz", "10"])
        output = io.StringIO()
        with mock.patch.object(reader.time, "monotonic", side_effect=lambda: clock[0]):
            reader.stream_samples(SlowReader(), args, output, Stop())
        rows = [json.loads(line) for line in output.getvalue().splitlines()]
        self.assertEqual(len(waits), 2)
        self.assertTrue(all(0 < delay <= 0.1 for delay in waits))
        self.assertAlmostEqual(rows[1]["dt_s"], 0.3)
        self.assertAlmostEqual(rows[2]["dt_s"], 0.3)


class ReaderErrorForTest(Exception):
    pass


if __name__ == "__main__":
    unittest.main()

"""Keepwarm lifecycle tests with simulated HTTP; no network or robot access."""
import copy
import json
import threading
import time
import unittest
from unittest.mock import patch

from connection_warmth import ConnectionWarmth, MAX_RESPONSE_BYTES

META = {"startup_id": "startup-fixed", "model_id": "pi05-same-model",
        "wire_image_codecs": ["raw-npz-v1", "png-rgb-u8-v1"]}


def wait_for(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("Timed out waiting for worker state")
        time.sleep(.001)


class Response:
    def __init__(self, document=None, status=200, raw=None, length=None):
        self.status = status
        self.raw = json.dumps({"ready": True, **META} if document is None else document).encode() if raw is None else raw
        self.length = str(len(self.raw)) if length is None else length
        self.read_sizes = []

    def getheader(self, key):
        assert key == "Content-Length"
        return self.length

    def read(self, size):
        self.read_sizes.append(size)
        return self.raw[:size]


class FakeHTTP:
    def __init__(self, response=None, error=None, blocked=None):
        self.response = response or Response()
        self.error = error
        self.blocked = blocked
        self.requests = []
        self.request_times = []
        self.entered = threading.Event()
        self.closed = threading.Event()

    def request(self, method, path, body=None, headers=None):
        self.requests.append((method, path, body, headers))
        self.request_times.append(time.monotonic())
        self.entered.set()

    def getresponse(self):
        if self.blocked is not None:
            self.blocked.wait()
        if self.error is not None:
            raise self.error
        return self.response

    def close(self):
        self.closed.set()


class KeepwarmTests(unittest.TestCase):
    def make(self, busy=None, **kwargs):
        return ConnectionWarmth("127.0.0.1", 18814, META, busy or threading.Event(),
                                interval_s=kwargs.pop("interval_s", .01), **kwargs)

    def test_start_pause_resume_and_close_only_get_no_body(self):
        busy = threading.Event()
        busy.set()
        http = FakeHTTP()
        with patch("connection_warmth.http.client.HTTPConnection", return_value=http) as factory:
            worker = self.make(busy).start()
            try:
                wait_for(lambda: worker.snapshot()["paused_skips"] >= 2)
                self.assertEqual(worker.snapshot()["sent"], 0)
                factory.assert_not_called()
                busy.clear()
                wait_for(lambda: worker.snapshot()["ok"] >= 2)
                busy.set()
                time.sleep(.025)  # One already in-flight ping may finish.
                sent = worker.snapshot()["sent"]
                time.sleep(.025)
                self.assertEqual(worker.snapshot()["sent"], sent)
                busy.clear()
                wait_for(lambda: worker.snapshot()["sent"] > sent)
            finally:
                stats = worker.close()
        self.assertFalse(stats["worker_alive"])
        self.assertTrue(http.closed.is_set())
        factory.assert_called_once_with("127.0.0.1", 18814, timeout=.2)
        self.assertTrue(all(method == "GET" and path == "/ping" and body is None
                            for method, path, body, headers in http.requests))
        self.assertTrue(all(size == MAX_RESPONSE_BYTES + 1 for size in http.response.read_sizes))

    def test_blocked_worker_never_delays_main_inference_or_zero_join(self):
        release = threading.Event()
        http = FakeHTTP(blocked=release)
        busy = threading.Event()
        with patch("connection_warmth.http.client.HTTPConnection", return_value=http):
            worker = self.make(busy).start()
            try:
                self.assertTrue(http.entered.wait(1))
                began = time.monotonic()
                busy.set()
                # An entirely separate main HTTP connection remains usable.
                main_connection = FakeHTTP()
                main_connection.request("POST", "/infer", body=b"recorded-input")
                main_connection.getresponse().read(MAX_RESPONSE_BYTES)
                stats = worker.close(join_timeout=0)
                elapsed = time.monotonic() - began
                self.assertLess(elapsed, .05)
                self.assertTrue(stats["worker_alive"])
                self.assertFalse(http.closed.is_set())
                self.assertFalse(main_connection.closed.is_set())
                self.assertEqual(len(main_connection.requests), 1)
            finally:
                release.set()
                final = worker.close()
        self.assertFalse(final["worker_alive"])
        self.assertTrue(http.closed.is_set())
        self.assertEqual(len(http.requests), 1)

    def test_default_close_bounded_even_noncooperative_http(self):
        release = threading.Event()
        http = FakeHTTP(blocked=release)
        with patch("connection_warmth.http.client.HTTPConnection", return_value=http):
            worker = self.make(timeout_s=.02).start()
            try:
                self.assertTrue(http.entered.wait(1))
                began = time.monotonic()
                stats = worker.close()
                self.assertLess(time.monotonic() - began, .2)
                self.assertTrue(stats["worker_alive"])
                self.assertTrue(worker._thread.daemon)
            finally:
                release.set()
                worker.close()

    def test_identity_mismatch_stops_itself_only(self):
        for key, wrong in (("ready", False), ("startup_id", "foreign"), ("model_id", "foreign"),
                           ("wire_image_codecs", list(reversed(META["wire_image_codecs"])))):
            with self.subTest(key=key):
                document = {"ready": True, **META, key: wrong}
                http = FakeHTTP(response=Response(document))
                busy = threading.Event()
                with patch("connection_warmth.http.client.HTTPConnection", return_value=http):
                    worker = self.make(busy).start()
                    wait_for(lambda: worker.snapshot()["identity_failures"] == 1)
                    stats = worker.close()
                self.assertEqual(stats["sent"], 1)
                self.assertEqual(stats["ok"], 0)
                self.assertEqual(stats["errors"], 1)
                self.assertTrue(stats["stop_requested"])
                self.assertFalse(busy.is_set())
                # The main caller can still issue its own independently pinned
                # inference; there is no callback or main connection to cancel.
                self.assertIn("identity mismatch", stats["last_error"])

    def test_transport_failure_backoff_owns_only_get_and_recovers(self):
        first = FakeHTTP(error=TimeoutError("diagnostic timeout"))
        second = FakeHTTP()
        with patch("connection_warmth.http.client.HTTPConnection", side_effect=[first, second]):
            worker = self.make(interval_s=.02).start()
            try:
                wait_for(lambda: worker.snapshot()["ok"] >= 1)
            finally:
                stats = worker.close()
        self.assertEqual(stats["errors"], 1)
        self.assertEqual(stats["identity_failures"], 0)
        self.assertTrue(first.closed.is_set())
        self.assertGreaterEqual(second.request_times[0] - first.request_times[0], .035)
        self.assertGreaterEqual(stats["sent"], 2)
        self.assertTrue(all(method == "GET" and path == "/ping" and body is None
                            for h in (first, second) for method, path, body, _ in h.requests))
        self.assertGreater(stats["max_rtt_ms"], 0)

    def test_oversized_header_or_body_rejected_without_printing(self):
        cases = [Response(length=str(MAX_RESPONSE_BYTES + 1)),
                 Response(raw=b"x" * (MAX_RESPONSE_BYTES + 1), length="0")]
        for response in cases:
            http = FakeHTTP(response=response)
            with patch("connection_warmth.http.client.HTTPConnection", return_value=http), \
                    patch("builtins.print") as output:
                worker = self.make(interval_s=.1).start()
                wait_for(lambda: worker.snapshot()["errors"] >= 1)
                stats = worker.close()
            self.assertEqual(stats["ok"], 0)
            self.assertIn("64 KiB", stats["last_error"])
            output.assert_not_called()

    def test_metadata_snapshot_idempotence_and_no_restart_after_close(self):
        expected = copy.deepcopy(META)
        busy = threading.Event()
        busy.set()
        worker = ConnectionWarmth("127.0.0.1", 18814, expected, busy)
        expected["wire_image_codecs"].reverse()
        self.assertEqual(worker.expected["wire_image_codecs"], META["wire_image_codecs"])
        worker.start()
        thread = worker._thread
        self.assertIs(worker.start(), worker)
        self.assertIs(worker._thread, thread)
        stats = worker.close()
        self.assertEqual(stats, worker.close())
        self.assertEqual(stats, worker.statistics())
        with self.assertRaises(RuntimeError):
            worker.start()

    def test_validate_timeout_and_join_bounds(self):
        for timeout in (0, -.1, .251, float("nan")):
            with self.assertRaises(ValueError):
                self.make(timeout_s=timeout)
        for interval in (0, -.1, float("inf")):
            with self.assertRaises(ValueError):
                self.make(interval_s=interval)
        worker = self.make()
        for limit in (-1, .301, float("nan")):
            with self.assertRaises(ValueError):
                worker.close(join_timeout=limit)
        self.assertFalse(worker.close()["worker_alive"])


if __name__ == "__main__":
    unittest.main(verbosity=2)

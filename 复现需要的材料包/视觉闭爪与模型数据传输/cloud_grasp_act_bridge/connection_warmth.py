"""Independent idle-only HTTP keepwarm; never owns inference or robot state."""
from __future__ import annotations

import http.client
import json
import math
import threading
import time

MAX_RESPONSE_BYTES = 64 * 1024


class _IdentityMismatch(RuntimeError):
    pass


class ConnectionWarmth:
    """Send small GET /ping requests while the caller's busy event is clear.

    The caller sets busy_event before inference and clears it in finally. One
    already in-flight ping may finish, but inference never waits for this worker.
    Failures are diagnostics only; this class cannot retry inference or actions.
    """

    def __init__(self, host, port, expected_metadata, busy_event, interval_s=.075, timeout_s=.2):
        if not isinstance(host, str) or not host or type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("Expected HTTP host and integer port")
        if not math.isfinite(interval_s) or interval_s <= 0 \
                or not math.isfinite(timeout_s) or not 0 < timeout_s <= .25:
            raise ValueError("Use positive interval and HTTP timeout <= .25 seconds")
        expected = {key: expected_metadata[key] for key in ("startup_id", "model_id", "wire_image_codecs")}
        if any(not isinstance(expected[key], str) or not expected[key] for key in ("startup_id", "model_id")) \
                or not isinstance(expected["wire_image_codecs"], list) \
                or not expected["wire_image_codecs"] \
                or any(not isinstance(item, str) for item in expected["wire_image_codecs"]):
            raise ValueError("Expected verified startup, model and codec metadata")
        expected["wire_image_codecs"] = list(expected["wire_image_codecs"])
        if not callable(getattr(busy_event, "is_set", None)):
            raise TypeError("busy_event must support is_set()")
        self.host, self.port = host, port
        self.expected, self.busy_event = expected, busy_event
        self.interval_s, self.timeout_s = float(interval_s), float(timeout_s)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._closed = False
        self._stats = {"sent": 0, "ok": 0, "errors": 0, "identity_failures": 0,
                       "paused_skips": 0, "response_bytes": 0, "max_rtt_ms": 0.0,
                       "last_error": None, "connection_close_errors": 0}

    def start(self):
        with self._lock:
            if self._closed:
                raise RuntimeError("Cannot restart a closed keepwarm worker")
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name="pi05-http-keepwarm", daemon=True)
                self._thread.start()
        return self

    def snapshot(self):
        with self._lock:
            return {**self._stats, "worker_alive": bool(self._thread and self._thread.is_alive()),
                    "closed": self._closed, "stop_requested": self._stop.is_set(),
                    "interval_s": self.interval_s, "timeout_s": self.timeout_s}

    def statistics(self):
        return self.snapshot()

    def close(self, join_timeout=None):
        limit = self.timeout_s + .1 if join_timeout is None else float(join_timeout)
        if not math.isfinite(limit) or not 0 <= limit <= self.timeout_s + .1:
            raise ValueError("join_timeout must be between zero and timeout_s + .1")
        # Only the worker touches its connection. Closing a response from here
        # could block on its buffered reader lock and delay controller teardown.
        self._stop.set()
        with self._lock:
            self._closed = True
            thread = self._thread
        if thread is not None and thread is not threading.current_thread() and limit:
            thread.join(limit)
        return self.snapshot()

    def _run(self):
        connection = None
        consecutive_errors = 0

        def discard_connection():
            nonlocal connection
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    with self._lock:
                        self._stats["connection_close_errors"] += 1
                connection = None

        try:
            while not self._stop.is_set():
                if self.busy_event.is_set():
                    with self._lock:
                        self._stats["paused_skips"] += 1
                    self._stop.wait(self.interval_s)
                    continue
                began = time.monotonic()
                failed = False
                identity_failed = False
                try:
                    if connection is None:
                        connection = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout_s)
                    # This check narrows the race; one in-flight ping remains
                    # permitted when the caller sets busy immediately afterward.
                    if self._stop.is_set() or self.busy_event.is_set():
                        continue
                    with self._lock:
                        self._stats["sent"] += 1
                    connection.request("GET", "/ping", headers={"Cache-Control": "no-store"})
                    response = connection.getresponse()
                    length = response.getheader("Content-Length")
                    if length is not None and not 0 <= int(length) <= MAX_RESPONSE_BYTES:
                        raise ValueError("Keepwarm response exceeds 64 KiB")
                    raw = response.read(MAX_RESPONSE_BYTES + 1)
                    if len(raw) > MAX_RESPONSE_BYTES:
                        raise ValueError("Keepwarm response exceeds 64 KiB")
                    with self._lock:
                        self._stats["response_bytes"] += len(raw)
                    if response.status != 200:
                        raise RuntimeError(f"Keepwarm HTTP {response.status}")
                    document = json.loads(raw)
                    if not isinstance(document, dict) or document.get("ready") is not True:
                        raise _IdentityMismatch("Keepwarm ready identity mismatch")
                    for key, expected in self.expected.items():
                        if document.get(key) != expected:
                            raise _IdentityMismatch(f"Keepwarm {key} identity mismatch")
                    with self._lock:
                        self._stats["ok"] += 1
                    consecutive_errors = 0
                except Exception as error:
                    failed = True
                    identity_failed = isinstance(error, _IdentityMismatch)
                    consecutive_errors += 1
                    with self._lock:
                        self._stats["errors"] += 1
                        self._stats["identity_failures"] += int(identity_failed)
                        self._stats["last_error"] = f"{type(error).__name__}: {error}"[:512]
                    discard_connection()
                    if identity_failed:
                        self._stop.set()
                finally:
                    elapsed = time.monotonic() - began
                    with self._lock:
                        self._stats["max_rtt_ms"] = max(self._stats["max_rtt_ms"], elapsed * 1000)
                if identity_failed:
                    break
                delay = min(1.0, self.interval_s * 2 ** min(consecutive_errors, 4)) if failed else self.interval_s
                self._stop.wait(max(0.0, delay - (time.monotonic() - began)))
        finally:
            discard_connection()

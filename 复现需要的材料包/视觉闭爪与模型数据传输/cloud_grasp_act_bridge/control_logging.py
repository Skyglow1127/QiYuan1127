"""VLA-only diagnostic queues; command timing, guards and target trace stay direct.

Only diagnostic logging/terminal copies may be dropped. A blocked sink runs in
one daemon, never in the control thread, and shutdown never joins it forever.
"""
from contextlib import contextmanager
import copy
from functools import wraps
import json
import logging
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

QUEUE_SIZE = 4096
_active_logging_worker = None


class BoundedDiagnosticWorker:
    def __init__(self, deliver, *, capacity=QUEUE_SIZE, name='pi05-diagnostic', duration_key='max_handler_duration_ms'):
        self.queue = queue.Queue(maxsize=capacity)
        self.deliver, self.duration_key = deliver, duration_key
        self._stop, self._discard = threading.Event(), threading.Event()
        self._lock = threading.Lock()
        self.accepting = True
        self.counts = {'capacity': capacity, 'enqueued': 0, 'drained': 0, 'dropped': 0,
                       'discarded_on_shutdown': 0, 'sink_errors': 0, 'prepare_errors': 0, duration_key: 0.}
        self._in_flight_at = None
        self.thread = threading.Thread(target=self._run, name=name, daemon=True)
        self.thread.start()

    def enqueue(self, item):
        # No handler formatting, IO or handleError fallback on this path.
        with self._lock:
            if not self.accepting:
                self.counts['dropped'] += 1
                return False
            try:
                self.queue.put_nowait(item)
            except queue.Full:
                self.counts['dropped'] += 1
                return False
            self.counts['enqueued'] += 1
            return True

    def _run(self):
        while not (self._stop.is_set() and self.queue.empty()):
            try:
                item = self.queue.get(timeout=.02)
            except queue.Empty:
                continue
            if self._discard.is_set():
                with self._lock:
                    self.counts['discarded_on_shutdown'] += 1
                self.queue.task_done()
                continue
            began = time.monotonic()
            with self._lock:
                self._in_flight_at = began
            try:
                self.deliver(item)
            except Exception:
                # A diagnostic sink failure cannot recursively print or cause
                # target replay. Its count remains available in the report.
                with self._lock:
                    self.counts['sink_errors'] += 1
            finally:
                duration = (time.monotonic()-began)*1000
                with self._lock:
                    self.counts['drained'] += 1
                    self.counts[self.duration_key] = max(self.counts[self.duration_key], duration)
                    self._in_flight_at = None
                self.queue.task_done()

    def statistics(self):
        with self._lock:
            return {**self.counts, 'pending': self.queue.qsize(), 'worker_alive': self.thread.is_alive(),
                    'in_flight': self._in_flight_at is not None,
                    'in_flight_duration_ms': 0. if self._in_flight_at is None else
                        (time.monotonic()-self._in_flight_at)*1000}

    def stop(self, timeout=.25):
        with self._lock:
            self.accepting = False
        self._stop.set()
        self.thread.join(timeout=max(0., timeout))
        if self.thread.is_alive():
            # A blocked original handler cannot be forcibly killed safely.
            # Empty its pending diagnostic queue and report the daemon clearly.
            self._discard.set()
            while True:
                try:
                    self.queue.get_nowait()
                except queue.Empty:
                    break
                with self._lock:
                    self.counts['discarded_on_shutdown'] += 1
                self.queue.task_done()
        return self.statistics()


def _deliver_record(item):
    handlers, record = item
    for handler in handlers:
        if record.levelno >= handler.level:
            handler.handle(record)  # Retains each original handler's filters.


class _QueuedRoute(logging.Handler):
    def __init__(self, worker, handlers):
        super().__init__(logging.NOTSET)
        self.worker, self.original_handlers = worker, tuple(handlers)

    def emit(self, record):
        # Preserve creation time, args and exception/filter fields. Formatting
        # remains on the worker, using the original handlers and formatters.
        try:
            prepared = copy.copy(record)
            prepared.msg, prepared.args = _snapshot(record.msg), _snapshot(record.args)
            self.worker.enqueue((self.original_handlers, prepared))
        except Exception:
            with self.worker._lock:
                self.worker.counts['dropped'] += 1
                self.worker.counts['prepare_errors'] += 1


def _snapshot(value):
    """Snapshot only diagnostic built-in containers, never images/tracebacks."""
    if isinstance(value, dict):
        return {key: _snapshot(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_snapshot(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_snapshot(item) for item in value)
    if isinstance(value, set):
        return {_snapshot(item) for item in value}
    return value


def write_statistics(path, statistics):
    if path is not None:
        original_error = sys.exc_info()[1]
        try:
            Path(path).write_text(json.dumps(statistics, ensure_ascii=False, indent=2)+'\n')
        except Exception as error:
            if original_error is None:
                raise
            original_error.add_note('Diagnostic statistics could not be written: '+str(error))


@contextmanager
def asynchronous_control_logging(*, report_path=None, capacity=QUEUE_SIZE, stop_timeout=.25):
    global _active_logging_worker
    if _active_logging_worker is not None and _active_logging_worker.thread.is_alive():
        raise RuntimeError('Previous VLA diagnostic worker is still blocked; no new control scope started')
    worker = BoundedDiagnosticWorker(_deliver_record, capacity=capacity, name='pi05-control-log')
    _active_logging_worker = worker
    root = logging.getLogger()
    loggers = [root, *(logger for logger in logging.Logger.manager.loggerDict.values()
                       if isinstance(logger, logging.Logger))]
    saved = []
    try:
        for logger in loggers:
            if logger.handlers:
                originals = list(logger.handlers)
                route = _QueuedRoute(worker, originals)
                saved.append((logger, originals))
                logger.handlers = [route]
        yield worker
    finally:
        # This scope encloses the official adapter's entire finally cleanup.
        # Restore original identities before stopping; no queued handler remains.
        for logger, originals in reversed(saved):
            logger.handlers = originals
        statistics = worker.stop(timeout=stop_timeout)
        write_statistics(report_path, {'purpose': 'VLA diagnostic logging only; target trace remains direct',
                                      **statistics})


def with_asynchronous_control_logging(function):
    """Keep logging asynchronous until the wrapped function finishes cleanup."""
    @wraps(function)
    def execute(*args, **kwargs):
        result = kwargs.get('startup_result')
        report = None if result is None else Path(result).with_name('control_logging_statistics.json')
        with asynchronous_control_logging(report_path=report):
            return function(*args, **kwargs)
    return execute


class AsyncTerminalMirror:
    def __init__(self, stream=None, *, capacity=QUEUE_SIZE):
        self.stream = sys.stdout if stream is None else stream
        def write(line):
            self.stream.write(line)
            self.stream.flush()
        self.worker = BoundedDiagnosticWorker(write, capacity=capacity, name='pi05-terminal-mirror',
                                              duration_key='max_write_ms')

    def enqueue(self, line):
        return self.worker.enqueue(line)

    def stop(self, timeout=.25):
        return self.worker.stop(timeout=timeout)


def persist_and_mirror(lines, log, mirror):
    """Always persist every raw child line before a best-effort terminal copy."""
    for line in lines:
        log.write(line)
        log.flush()
        mirror.enqueue(line)


def stop_failed_log_child(process, original_error):
    """A failed full log must end the child through its existing signal cleanup."""
    try:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
    except Exception as error:
        original_error.add_note('Child cleanup after log failure: '+str(error))

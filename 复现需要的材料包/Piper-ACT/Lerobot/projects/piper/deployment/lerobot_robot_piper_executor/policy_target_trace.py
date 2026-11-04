"""Buffered local policy diagnostics. No camera, transport or motion operations.

Times use the host monotonic clock. ``backend_ack_at`` means the RPC returned;
it does not attest when CAN frames were transmitted or when the robot moved.
Records are buffered until close, without per-target flush/fsync operations.
"""

from collections.abc import Mapping
import json
import math
from pathlib import Path

from .guard import GuardError


def _plain(value):
    """Snapshot scalar data, preserving rejected nonfinite values as JSON text."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_plain(item) for item in value]
    try:
        number = float(value)
    except Exception:
        return {"invalid_scalar_type": type(value).__name__}
    if math.isnan(number):
        return "NaN"
    if math.isinf(number):
        return "Infinity" if number > 0 else "-Infinity"
    return number


class PolicyTargetTrace:
    def __init__(self, path):
        if not isinstance(path, (str, Path)) or not Path(path).is_absolute():
            raise GuardError("policy_target_trace_path must be an absolute local path")
        self.path = Path(path)
        self.attempt = 0
        try:
            self._stream = self.path.open("a", encoding="utf-8", buffering=65536)
        except Exception as error:
            raise GuardError(f"Cannot open policy target trace {self.path}: {error}") from error

    def start(self, *, raw_prediction, observed_state, sampled_at, timing, requested_at):
        self.attempt += 1
        timing_values = None if timing is None else {
            name: getattr(timing, name, None) for name in (
                "observed_at", "generated_at", "selected_at", "action_index",
                "action_count", "fps")}
        # Only the seven scalar state fields are supplied, never image arrays.
        return {
            "schema_version": 1,
            "attempt": self.attempt,
            "requested_at": _plain(requested_at),
            "raw_prediction": _plain(raw_prediction),
            "j5_projected_prediction": None,
            "relative_capped_prediction": None,
            "quantized_target": None,
            "final_target": None,
            "observed_state": _plain(observed_state),
            "sampled_at": _plain(sampled_at),
            "act_timing": _plain(timing_values),
            "proposal_at": None,
            "command_sequence": None,
            "created_at": None,
            "expires_at": None,
            "backend_ack_at": None,
            "outcome": None,
            "transport_outcome_unknown": False,
            "error": None,
        }

    def append(self, record):
        try:
            self._stream.write(json.dumps(_plain(record), ensure_ascii=False,
                                          allow_nan=False, separators=(",", ":")) + "\n")
        except Exception as error:
            raise GuardError(f"Cannot append policy target trace {self.path}: {error}") from error

    def close(self):
        try:
            self._stream.close()
        except Exception as error:
            raise GuardError(f"Cannot close policy target trace {self.path}: {error}") from error

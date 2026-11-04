"""Source-time preserving rolling chunk scheduler; never touches a device."""
from dataclasses import dataclass
import math
import numpy as np
from .trajectory import MotionBounds, TrajectoryLimitError


@dataclass(frozen=True)
class TimedChunk:
    sequence: int
    observation_at: float
    produced_at: float
    sample_hz: float
    targets: np.ndarray

    def __post_init__(self):
        if not isinstance(self.sequence, int) or isinstance(self.sequence, bool) or self.sequence < 0:
            raise ValueError("nonnegative integer chunk sequence required")
        if not all(math.isfinite(x) for x in (self.observation_at, self.produced_at, self.sample_hz)) or self.sample_hz <= 0 or self.produced_at < self.observation_at:
            raise ValueError("invalid original observation/production times or model sampling rate")
        targets = np.asarray(self.targets, dtype=float).copy()
        if targets.ndim != 2 or not len(targets) or not targets.shape[1] or not np.all(np.isfinite(targets)):
            raise ValueError("finite NxD targets required")
        targets.setflags(write=False)
        object.__setattr__(self, "targets", targets)

    def slot_at(self, index):
        return self.observation_at+index/self.sample_hz


@dataclass(frozen=True)
class ScheduledSample:
    target: np.ndarray
    scheduled_at: float
    observation_at: float
    chunk_sequence: int
    slot_index: int


class ChunkScheduler:
    def __init__(self, *, consume_steps, source_max_age_s, slot_lateness_s, bounds: MotionBounds):
        if not isinstance(consume_steps, int) or isinstance(consume_steps, bool) or consume_steps <= 0:
            raise ValueError("explicit positive integer consume_steps required")
        if not math.isfinite(source_max_age_s) or not math.isfinite(slot_lateness_s) or source_max_age_s <= 0 or slot_lateness_s < 0:
            raise ValueError("explicit source lifetime and nonnegative slot lateness required")
        self.consume_steps, self.source_max_age_s, self.slot_lateness_s = consume_steps, source_max_age_s, slot_lateness_s
        self.bounds, self.chunk, self.end, self.first_index = bounds, None, 0, 0
        self.last_sequence, self.last_emitted_index, self.rejected_stale_slots = -1, -1, 0
        self.last_seen_time = -float("inf")

    def _clock(self, now):
        if not math.isfinite(now) or now < self.last_seen_time:
            raise ValueError("scheduler needs finite monotonic evaluation time")
        self.last_seen_time = now

    def accept(self, chunk: TimedChunk, *, now, current_target, additional_steps=0):
        self._clock(now)
        if not math.isfinite(now) or chunk.produced_at > now+1e-9:
            raise ValueError("chunk was produced in the future")
        if chunk.sequence <= self.last_sequence:
            raise ValueError("out-of-order/replayed chunk sequence")
        if now-chunk.observation_at > self.source_max_age_s:
            raise ValueError("stale observation: no source retimestamping or time stretching")
        if not isinstance(additional_steps, int) or isinstance(additional_steps, bool) or not 0 <= additional_steps <= 20:
            raise ValueError("profile-specific optional extension must be explicit 0..20")
        if chunk.targets.shape[1] != len(self.bounds.lower):
            raise ValueError("chunk dimension differs from physical-unit bounds")
        if np.any(chunk.targets < self.bounds.lower) or np.any(chunk.targets > self.bounds.upper):
            raise TrajectoryLimitError("incoming block violates joint/physical-unit bounds")
        end = min(len(chunk.targets), self.consume_steps+additional_steps)
        first = max(0, int(math.ceil((now-self.slot_lateness_s-chunk.observation_at)*chunk.sample_hz-1e-9)))
        if first >= end:
            raise ValueError("all slots in the permitted original consumption window expired")
        current_target = self.bounds.position(current_target)
        if np.any(abs(chunk.targets[first]-current_target) > self.bounds.dq_max):
            raise TrajectoryLimitError("incoming block boundary exceeds supplied DQ_MAX")
        if len(chunk.targets) > 1 and np.any(abs(np.diff(chunk.targets[:end], axis=0)) > self.bounds.dq_max):
            raise TrajectoryLimitError("incoming adjacent targets exceed supplied DQ_MAX")
        self.chunk, self.end, self.first_index = chunk, end, first
        self.last_sequence, self.last_emitted_index = chunk.sequence, first-1
        self.rejected_stale_slots += first
        return {"first_valid_slot": first, "permitted_steps": end, "observation_at": chunk.observation_at}

    def _index(self, now):
        self._clock(now)
        if self.chunk is None:
            return None
        if now-self.chunk.observation_at > self.source_max_age_s:
            return None
        index = int(math.floor((now-self.chunk.observation_at)*self.chunk.sample_hz+1e-9))
        if index < self.first_index or index >= self.end:
            return None
        return index

    def pop_due(self, *, now):
        index = self._index(now)
        if index is None or index <= self.last_emitted_index:
            return None
        chunk = self.chunk
        if now-chunk.slot_at(index) > self.slot_lateness_s+1e-9:
            self.rejected_stale_slots += index-self.last_emitted_index
            self.last_emitted_index = index
            return None
        self.rejected_stale_slots += max(0, index-self.last_emitted_index-1)
        self.last_emitted_index = index
        return ScheduledSample(chunk.targets[index].copy(), chunk.slot_at(index), chunk.observation_at, chunk.sequence, index)

    def reference_at(self, *, now):
        """Current original slot's linear reference, never extrapolated beyond window."""
        index = self._index(now)
        if index is None or index < self.last_emitted_index and self.last_emitted_index >= 0:
            return None
        chunk = self.chunk
        if index < self.end-1:
            weight = (now-chunk.slot_at(index))*chunk.sample_hz
            target = (1-weight)*chunk.targets[index]+weight*chunk.targets[index+1]
        else:
            target = chunk.targets[index]
        return ScheduledSample(target.copy(), chunk.slot_at(index), chunk.observation_at, chunk.sequence, index)

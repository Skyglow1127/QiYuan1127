"""Pure candidate for coordinating a visual closure with feedback.

No transport, timing refresh, queue mutation or contact sensor is implemented.
Callers must commit the proposed state only after a known successful ACK, and
retain every existing model-slot, feedback, position and receiver guard.
Width/settling evidence is a *holding candidate*, never proof of a grasp. Force
is diagnostic only: a successful-force calibration does not exist.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
from collections.abc import Mapping
import math


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _source(value, name):
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _vector(value, name):
    if isinstance(value, (str, bytes, Mapping)):
        raise ValueError(f"{name} must contain seven numbers")
    try:
        result = tuple(value)
    except TypeError as exc:
        raise ValueError(f"{name} must contain seven numbers") from exc
    if len(result) != 7:
        raise ValueError(f"{name} must contain seven numbers")
    return tuple(_number(v, name) for v in result)


@dataclass(frozen=True)
class ClosureSyncConfig:
    width_min_m: float = .0425
    width_max_m: float = .0455
    stability_s: float = .30
    stability_range_m: float = .0005
    minimum_pressure_m: float = .002
    max_closing_s: float = 4.
    max_anchor_error_deg: float = 3.
    empty_width_m: float = .035
    reopening_margin_m: float = .001
    max_feedback_age_s: float = .1
    max_sample_gap_s: float = .15
    state_joint_tolerance_deg: float = 1e-5
    state_gripper_tolerance_m: float = 1e-8
    placement_joint_bounds_deg: tuple[tuple[float, float], ...] = (
        (25.,38.),(118.,133.),(-105.,-75.),(-15.,15.),(55.,70.),(-75.,-40.))

    def __post_init__(self):
        for field in fields(self):
            if field.name == "placement_joint_bounds_deg":
                continue
            if _number(getattr(self, field.name), field.name) <= 0:
                raise ValueError(f"{field.name} must be positive")
        if not 0 < self.empty_width_m < self.width_min_m < self.width_max_m <= .1:
            raise ValueError("Empty and candidate-width thresholds must be ordered inside [0,.1]m")
        if not self.stability_range_m <= self.width_max_m-self.width_min_m:
            raise ValueError("Stability range must fit inside the candidate-width interval")
        if not self.max_feedback_age_s <= .1 or self.max_sample_gap_s > .15:
            raise ValueError("Feedback age/gap cannot exceed .1s/.15s respectively")
        if not self.max_sample_gap_s < self.stability_s < self.max_closing_s <= 4.:
            raise ValueError("Sample gap, stability and closing-time bounds must be strictly ordered, closing<=4s")
        if not self.minimum_pressure_m < self.empty_width_m:
            raise ValueError("Closing pressure must be smaller than the empty-width threshold")
        if self.max_anchor_error_deg > 3 or self.reopening_margin_m > .001:
            raise ValueError("Anchor/reopening tolerance exceeds the reviewed candidate")
        if self.state_joint_tolerance_deg > 1e-5 or self.state_gripper_tolerance_m > 1e-8:
            raise ValueError("Snapshot equality tolerance exceeds float32 allowance")
        pairs = self.placement_joint_bounds_deg
        expected = ((25.,38.),(118.,133.),(-105.,-75.),(-15.,15.),(55.,70.),(-75.,-40.))
        if (type(pairs) is not tuple or len(pairs) != 6
                or any(type(pair) is not tuple or len(pair) != 2 for pair in pairs)):
            raise ValueError("Placement window requires six immutable bound pairs")
        for pair in pairs:
            for value in pair:
                _number(value,"placement joint bound")
        if pairs != expected:
            raise ValueError("Placement window must equal the existing reviewed release window")

    def to_dict(self):
        return asdict(self)


def validate_config(config):
    if isinstance(config, Mapping):
        names = {f.name for f in fields(ClosureSyncConfig)}
        if set(config) != names:
            raise ValueError("Closure-sync JSON must specify exactly the complete config")
        value = dict(config)
        try:
            value['placement_joint_bounds_deg'] = tuple(tuple(p) for p in value['placement_joint_bounds_deg'])
        except TypeError as exc:
            raise ValueError("Invalid placement joint window") from exc
        config = ClosureSyncConfig(**value)
    if type(config) is not ClosureSyncConfig:
        raise ValueError("Expected ClosureSyncConfig or its complete JSON mapping")
    config.__post_init__()
    return config


@dataclass(frozen=True)
class ClosureSyncState:
    phase: str = "idle"
    anchor: tuple[float, ...] | None = None
    anchor_source_ns: int | None = None
    target_gripper_m: float | None = None
    target_model_source_at: float | None = None
    target_model_source_ns: int | None = None
    started_at: float | None = None
    initial_force_n: float | None = None
    samples: tuple[tuple[float, float, int], ...] = ()
    last_source_ns: int | None = None
    last_snapshot_at: float | None = None
    last_now: float | None = None
    contact_ready_at: float | None = None
    carry_ceiling_m: float | None = None
    failure_reason: str | None = None


@dataclass(frozen=True)
class ClosureSyncDiagnostic:
    decision: str
    phase_before: str
    phase_candidate: str
    candidate_only: bool = True
    contact_or_grasp_proven: bool = False
    force_is_calibrated_contact_evidence: bool = False
    request_replan: bool = False
    failure_reason: str | None = None
    snapshot_source_ns: int | None = None
    snapshot_age_s: float | None = None
    gripper_force_n: float | None = None
    initial_force_n: float | None = None
    force_delta_n: float | None = None
    width_m: float | None = None
    pressure_m: float | None = None
    unique_sample: bool = False
    stable_elapsed_s: float = 0.
    stable_range_m: float | None = None
    elapsed_s: float | None = None
    anchor_error_deg: float | None = None
    anchor_source_ns: int | None = None
    target_gripper_m: float | None = None
    target_model_source_at: float | None = None
    target_model_source_ns: int | None = None
    carry_ceiling_m: float | None = None


def _validate_state(state):
    if type(state) is not ClosureSyncState or state.phase not in {"idle", "closing", "contact_ready", "completed", "released", "failed"}:
        raise ValueError("Invalid closure-sync state")
    if state.phase != "idle" and state.phase != "failed":
        if type(state.anchor) is not tuple or len(state.anchor) != 6:
            raise ValueError("Active closure state requires a six-joint anchor")
        for v in state.anchor:
            _number(v, "state anchor")
        for name in ("target_gripper_m", "target_model_source_at", "started_at", "initial_force_n", "last_snapshot_at", "last_now"):
            _number(getattr(state, name), name)
        _source(state.anchor_source_ns, "anchor_source_ns")
        _source(state.last_source_ns, "last_source_ns")
        if state.target_model_source_ns is not None:
            _source(state.target_model_source_ns, "target_model_source_ns")
        if state.phase in {"contact_ready", "completed", "released"}:
            _number(state.contact_ready_at, "contact_ready_at")
            _number(state.carry_ceiling_m, "carry_ceiling_m")
        if not state.target_model_source_at <= state.last_now or not state.started_at <= state.last_now:
            raise ValueError("Active closure state has future timing")
        if state.last_snapshot_at > state.last_now:
            raise ValueError("Active closure state has a future feedback timestamp")
    if not isinstance(state.samples, tuple):
        raise ValueError("Stable evidence must be an immutable tuple")
    previous_at = previous_source = None
    for sample in state.samples:
        if type(sample) is not tuple or len(sample) != 3:
            raise ValueError("Invalid stable evidence")
        at, width, source = sample
        _number(at, "sample timestamp"); _number(width, "sample width"); _source(source, "sample source")
        if previous_at is not None and (at <= previous_at or source <= previous_source):
            raise ValueError("Stable evidence timestamps/sources must strictly increase")
        previous_at, previous_source = at, source


def _snapshot(snapshot, observed, observed_source_ns, snapshot_at, now, config):
    if not isinstance(snapshot, Mapping) or snapshot.get("healthy") is not True:
        raise ValueError("Snapshot is not healthy")
    source = _source(snapshot.get("source_ns"), "snapshot.source_ns")
    if source != _source(observed_source_ns, "observed_source_ns"):
        raise ValueError("Snapshot and observation source_ns differ")
    state = _vector(snapshot.get("state"), "snapshot.state")
    tolerance = (config.state_joint_tolerance_deg,)*6+(config.state_gripper_tolerance_m,)
    if any(abs(a-b)>cap for a,b,cap in zip(state,observed,tolerance,strict=True)):
        raise ValueError("Snapshot and observed state are not the same source values")
    if not 0 <= now-snapshot_at <= config.max_feedback_age_s:
        raise ValueError("Snapshot is stale or from the future")
    status = snapshot.get("status")
    if not isinstance(status, Mapping):
        raise ValueError("Snapshot status is absent")
    for key, expected in (("ctrl_mode",1),("arm_status",0),("mode_feedback",1),("teach_status",0),("err_code",0),("gripper_mode",0)):
        if type(status.get(key)) is not int or status[key] != expected:
            raise ValueError("Snapshot controller/gripper mode or fault status is invalid")
    gripper_status = status.get("gripper_status")
    if type(gripper_status) is not int or not 0 <= gripper_status <= 255 or gripper_status & 0x3f or not gripper_status & 0x40:
        raise ValueError("Gripper is not enabled and fault-free")
    force = _number(status.get("gripper_force_n"), "snapshot gripper_force_n")
    if not 0 <= observed[6] <= .1:
        raise ValueError("Feedback gripper width is outside [0,.1]m")
    return source, force


def shape_closure(raw, observed, phase, *, now, observed_source_ns, snapshot,
                  snapshot_at, raw_source_at, previous_ack_target7, closure_latched=False,
                  new_prediction=False, reset=False, raw_source_ns=None,
                  config=ClosureSyncConfig()):
    """Return (target|None, proposed state, diagnostic), with no side effects.

    ``raw_source_at`` is the *unchanged* original model chunk source timestamp.
    ``snapshot_at`` is its independent feedback source mapped to the host clock.
    A latched target is a local stage constraint; retaining it does not create a
    fresh model prediction or extend a model slot. Caller guards still apply.
    ``new_prediction`` may be true only after the caller discarded the old FIFO
    and successfully inferred from a subsequent fresh observation.
    """
    config = validate_config(config)
    before = phase.phase if type(phase) is ClosureSyncState else "invalid"
    common = {}
    def result(target, state, decision, **more):
        values = {**common, **more}
        values.update(anchor_source_ns=state.anchor_source_ns,
                      target_gripper_m=state.target_gripper_m,
                      target_model_source_at=state.target_model_source_at,
                      target_model_source_ns=state.target_model_source_ns,
                      carry_ceiling_m=state.carry_ceiling_m)
        diagnostic = ClosureSyncDiagnostic(decision, before, state.phase, **values)
        return target, state, diagnostic
    def fail(reason):
        original = phase if type(phase) is ClosureSyncState else ClosureSyncState()
        state = replace(original, phase="failed", failure_reason=reason)
        return result(None, state, "failed", failure_reason=reason)
    try:
        _validate_state(phase)
        if any(type(v) is not bool for v in (closure_latched,new_prediction,reset)):
            raise ValueError("Closure control flags must be actual booleans")
        now = _number(now,"now"); snapshot_at = _number(snapshot_at,"snapshot_at")
        raw_source_at = _number(raw_source_at,"raw_source_at")
        if raw_source_at > now:
            raise ValueError("Model source is in the future")
        if raw_source_ns is not None:
            _source(raw_source_ns,"raw_source_ns")
        raw = _vector(raw,"raw"); observed = _vector(observed,"observed")
        previous_ack = _vector(previous_ack_target7,"previous_ack_target7")
        source, force = _snapshot(snapshot,observed,observed_source_ns,snapshot_at,now,config)
        common.update(snapshot_source_ns=source,snapshot_age_s=now-snapshot_at,
                      gripper_force_n=force,width_m=observed[6])
        if phase.phase == "failed":
            return fail(phase.failure_reason or "Previously failed")
        if phase.phase == "completed" and observed[6] < config.empty_width_m:
            raise ValueError("Holding candidate lost: actual opening fell below the empty-width threshold")
        if reset:
            if phase.phase in {"closing","contact_ready"}:
                raise ValueError("Visual reset cannot cancel an unfinished coordinated closure")
            if (phase.phase == "completed"
                    and not all(low <= value <= high for value,(low,high) in zip(observed[:6],config.placement_joint_bounds_deg,strict=True))):
                raise ValueError("Visual reset cannot authorize reopening before the placement window")
            phase = ClosureSyncState()
        if phase.last_now is not None and now < phase.last_now:
            raise ValueError("Host time moved backwards")
        if phase.last_source_ns is not None and source < phase.last_source_ns:
            raise ValueError("Feedback source moved backwards")
        if phase.phase == "idle":
            if not closure_latched:
                return result(raw,phase,"idle_passthrough")
            if raw[6] >= observed[6]:
                raise ValueError("Visual latch did not request closure")
            phase = ClosureSyncState(phase="closing",anchor=observed[:6],
                anchor_source_ns=source,target_gripper_m=raw[6],
                target_model_source_at=raw_source_at,target_model_source_ns=raw_source_ns,
                started_at=now,initial_force_n=force,last_source_ns=source,
                last_snapshot_at=snapshot_at,last_now=now)
            unique = True
        else:
            unique = source != phase.last_source_ns
            if unique and snapshot_at <= phase.last_snapshot_at:
                raise ValueError("New feedback source must have a later mapped timestamp")
        error = max(abs(a-b) for a,b in zip(observed[:6],phase.anchor,strict=True))
        common.update(anchor_error_deg=error,initial_force_n=phase.initial_force_n,
                      force_delta_n=force-phase.initial_force_n,unique_sample=unique,
                      elapsed_s=now-phase.started_at)
        if phase.phase in {"completed","released"}:
            next_phase = replace(phase,last_source_ns=source,last_snapshot_at=snapshot_at,last_now=now)
            if phase.phase == "completed" and raw[6] > phase.carry_ceiling_m:
                if not all(low <= value <= high for value,(low,high) in zip(observed[:6],config.placement_joint_bounds_deg,strict=True)):
                    raise ValueError("Model requests reopening before the reviewed placement window")
                next_phase = replace(next_phase,phase="released")
                return result(raw,next_phase,"placement_release_passthrough")
            return result(raw,next_phase,phase.phase+"_passthrough")
        if error > config.max_anchor_error_deg:
            raise ValueError("Actual joints left the closing anchor tolerance")
        if observed[6] < config.empty_width_m:
            raise ValueError("Empty closure below the reviewed minimum width")
        if now-phase.started_at >= config.max_closing_s:
            raise ValueError("Coordinated closure exceeded its four-second deadline")
        if phase.phase == "contact_ready":
            if new_prediction:
                if not unique or snapshot_at <= phase.contact_ready_at or raw_source_at <= phase.contact_ready_at:
                    raise ValueError("Replan does not originate after contact-candidate readiness")
                if raw[6] > phase.carry_ceiling_m:
                    raise ValueError("Fresh model requests reopening at the grasp anchor")
                completed = replace(phase,phase="completed",last_source_ns=source,
                                    last_snapshot_at=snapshot_at,last_now=now)
                return result(raw,completed,"fresh_replan_released")
            held = phase.anchor+(phase.target_gripper_m,)
            return result(held,replace(phase,last_source_ns=source,last_snapshot_at=snapshot_at,last_now=now),
                          "await_fresh_replan",request_replan=True)
        if new_prediction:
            raise ValueError("Cannot release closing before a holding candidate")
        if raw[6] < phase.target_gripper_m:
            phase = replace(phase,target_gripper_m=raw[6],target_model_source_at=raw_source_at,
                            target_model_source_ns=raw_source_ns)
        pressure = observed[6]-previous_ack[6]
        common["pressure_m"] = pressure
        qualified = (config.width_min_m <= observed[6] <= config.width_max_m
                     and pressure+config.state_gripper_tolerance_m >= config.minimum_pressure_m)
        samples = phase.samples
        if not qualified:
            samples = ()
        elif unique:
            if samples and snapshot_at-samples[-1][0] > config.max_sample_gap_s:
                samples = ()
            samples += ((snapshot_at,observed[6],source),)
            # Retain a trailing continuous interval, removing varying old
            # widths until every included pair meets the range requirement.
            while samples and max(v[1] for v in samples)-min(v[1] for v in samples)>config.stability_range_m:
                samples = samples[1:]
        stable_elapsed = samples[-1][0]-samples[0][0] if len(samples)>1 else 0.
        stable_range = max(v[1] for v in samples)-min(v[1] for v in samples) if samples else None
        common.update(stable_elapsed_s=stable_elapsed,stable_range_m=stable_range)
        next_phase = replace(phase,samples=samples,last_source_ns=source,
                             last_snapshot_at=snapshot_at,last_now=now)
        if len(samples)>1 and stable_elapsed >= config.stability_s:
            ceiling = min(previous_ack[6]+config.reopening_margin_m,
                          observed[6]-config.minimum_pressure_m)
            next_phase = replace(next_phase,phase="contact_ready",contact_ready_at=now,
                                 carry_ceiling_m=ceiling)
            return result(phase.anchor+(phase.target_gripper_m,),next_phase,
                          "holding_candidate",request_replan=True)
        return result(phase.anchor+(phase.target_gripper_m,),next_phase,"closing_hold")
    except (ValueError,TypeError,KeyError,OverflowError) as exc:
        return fail(str(exc))

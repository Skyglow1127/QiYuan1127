"""LeRobot 0.6.1 adapter; default connection is an independent RX-only backend.

Opening this Robot never switches mode, enables, calibrates or arms the robot.
Receiver-side TX is separately gated by an explicit reviewed execution policy.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import logging
from pathlib import Path
import time
from typing import Protocol

from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot
from lerobot.robots.utils import ensure_safe_goal_position

from .guard import ACTION_KEYS, ActionGuard, CheckedCommand, GuardError, GuardLimits, LocalOwner, Readiness
from .target_rate import shape_target_rate, target_rate_wait_s
from .policy_bounds import (joint_policy_target_bounds, project_joint_predictions,
    j5_upper_target_bound, project_j5_upper_prediction, saturate_j5_upper_prediction)
from .policy_target_trace import PolicyTargetTrace
from .joint_smoothing import smooth_joint_targets, validate_joint_smoothing_alpha
from act_release_continuation import gate_release_motion, validate_release_continuation

LOG = logging.getLogger(__name__)

# Official 0.6.1 rollout filters scalar hardware features by .pos/.vel suffix.
# These are KEY ALIASES ONLY: joints remain degrees, gripper remains metres.
ROLLOUT_KEYS = tuple(f"joint_{i}.pos" for i in range(1, 7)) + ("gripper.pos",)


@dataclass(frozen=True)
class Observation:
    # Timestamp must be acquisition time in this process's monotonic domain,
    # not the time an old observation was retrieved from a queue.
    values: dict
    sampled_at: float


class Backend(Protocol):
    """Implemented by backend.PiperBackend with an independent SSH RPC bridge.

    The implementation must verify the remote receiver/firmware, take an
    exclusive receiver-side lease, use receiver-local TTL after clock/latency
    checks, reject expired/sequencing errors, and implement the reviewed stop.
    Host monotonic timestamps cannot be sent as KR260 deadlines unchanged.
    send_checked must have a bounded timeout and fail on a partial batch. The
    watchdog must act even if this process/SSH link crashes or inference stalls.
    """
    def open_read_only(self) -> None: ...
    def observation(self) -> Observation: ...
    def require_execution_lease(self, readiness: Readiness) -> None: ...
    def send_checked(self, command: CheckedCommand) -> None: ...
    def invalidate_execution_lease(self, reason: str) -> None: ...
    def close(self) -> None: ...


@RobotConfig.register_subclass("piper_policy_executor")
@dataclass
class PiperPolicyExecutorConfig(RobotConfig):
    calibration_dir: Path = Path(__file__).resolve().parent.parent / "calibration"
    deployment_config: Path = Path(__file__).resolve().parent.parent / "observe_config.json"


class PiperPolicyExecutor(Robot):
    config_class = PiperPolicyExecutorConfig
    name = "piper_policy_executor"

    def __init__(self, config, *, backend: Backend | None = None,
                 limits: GuardLimits | None = None, clock=time.monotonic,
                 sleep=time.sleep):
        super().__init__(config)
        self.config = config
        self.backend, self.clock = backend, clock
        self.sleep = sleep
        self.guard = ActionGuard(limits) if limits is not None else None
        self.owner = LocalOwner(Path(__file__).resolve().parent.parent / "executor.lock")
        self.connected = False
        self.latest = None
        self.require_policy_timing = False
        self.policy_action_timing = None
        self.last_policy_command_at = None
        self.last_rate_log_at = None
        self.policy_bound_correction_count = 0
        self.policy_bound_saturation_count = 0
        self.policy_bound_saturation_streak = 0
        self.policy_bound_saturation_started_at = None
        self.policy_bound_saturation_duration_s = 0.
        self.policy_bound_saturation_max_streak = 0
        self.policy_bound_saturation_max_duration_s = 0.
        self.policy_target_trace = None
        self.joint_projection_counts = {}
        self.last_joint_projection_log_at = None
        self.release_motion_state = None

    def _ensure_policy_target_trace(self):
        path = getattr(self.backend, "config", {}).get("policy_target_trace_path")
        if path is not None and self.policy_target_trace is None:
            self.policy_target_trace = PolicyTargetTrace(path)
        return self.policy_target_trace

    def _close_policy_target_trace(self, *, preserve_error=None):
        trace, self.policy_target_trace = self.policy_target_trace, None
        if trace is None:
            return
        try:
            trace.close()
        except BaseException as error:
            if preserve_error is None:
                raise
            preserve_error.add_note(f"Policy target trace close also failed: {error}")
            LOG.error("Policy target trace close failed while retaining the original error", exc_info=True)

    def _disarm_preserving_error(self, reason, error):
        try:
            self.disarm(reason)
        except BaseException as disarm_error:
            error.add_note(f"Disarm/trace cleanup also failed: {disarm_error}")
            LOG.error("Disarm/trace cleanup failed while retaining the original error", exc_info=True)

    @property
    def action_features(self):
        return dict.fromkeys(ROLLOUT_KEYS, float)

    @property
    def observation_features(self):
        return {**self.action_features, "global": (480, 640, 3), "wrist": (480, 640, 3)}

    @property
    def is_connected(self):
        return self.connected

    @property
    def is_calibrated(self):
        # Absolute protocol feedback requires no automatic zeroing. This does
        # not attest the physical gripper mapping, which remains an arm gate.
        return True

    def calibrate(self):
        raise GuardError("Automatic calibration/zeroing is not implemented or authorized")

    def configure(self):
        raise GuardError("Automatic mode switching/enable is not implemented")

    def connect(self, calibrate=True):
        if self.connected:
            raise GuardError("Already connected")
        if self.backend is None:
            from .backend import PiperBackend
            self.backend = PiperBackend.from_file(self.config.deployment_config)
            if self.backend.guard_limits is not None:
                self.guard = ActionGuard(self.backend.guard_limits)
        self.owner.acquire()
        try:
            self.backend.open_read_only()
            self.connected = True
        except BaseException:
            try:
                self.backend.close()
            finally:
                self.owner.release()
            raise

    def get_observation(self):
        if not self.connected:
            raise GuardError("Not connected")
        try:
            observation = self.backend.observation()
            if set(observation.values) != set(ACTION_KEYS) | {"global", "wrist"}:
                raise GuardError("Missing/unexpected state or camera fields")
            for name in ("global", "wrist"):
                if getattr(observation.values[name], "shape", None) != (480, 640, 3):
                    raise GuardError("Expected two 640x480 RGB images")
            self.latest = observation
            return {**{alias: observation.values[canonical]
                for canonical, alias in zip(ACTION_KEYS, ROLLOUT_KEYS, strict=True)},
                **{name: observation.values[name] for name in ("global", "wrist")}}
        except BaseException as error:
            self.latest = None
            self._disarm_preserving_error("Observation acquisition failed", error)
            raise

    def arm_from_current_state(self, readiness: Readiness):
        if not self.connected:
            raise GuardError("Not connected")
        if self.guard is None:
            raise GuardError("Read-only connection: no reviewed execution limits")
        # No movement to episode 0 or model's first predicted pose. The first
        # policy command must pass the step gate from actual current feedback.
        self.disarm("Rearming requires a new review and current-state anchor")
        try:
            # Resolve/open the buffered local diagnostic file before acquiring
            # the arm observation or lease, outside live command deadlines.
            self._ensure_policy_target_trace()
            observation = self.get_observation()
            readiness.require_complete()
            self.backend.require_execution_lease(readiness)
            self.guard.arm({k: self.latest.values[k] for k in ACTION_KEYS},
                sampled_at=self.latest.sampled_at, now=self.clock(), readiness=readiness)
        except BaseException as error:
            self._disarm_preserving_error("Arming failed", error)
            raise

    def send_action(self, action):
        if not self.connected or self.latest is None:
            raise GuardError("Connected fresh observation required")
        if self.guard is None:
            raise GuardError("Read-only connection cannot send policy targets")
        trace = trace_record = None
        trace_append_attempted = False
        transport_started = False
        try:
            # Normal execution opened this before arm. Lazy opening only keeps
            # synthetic/manual adapter clients compatible with the same hook.
            trace = self._ensure_policy_target_trace()
            if trace is not None:
                raw = ({key: action.get(alias) for key, alias in zip(ACTION_KEYS, ROLLOUT_KEYS, strict=True)}
                       if isinstance(action, Mapping) else None)
                trace_record = trace.start(raw_prediction=raw,
                    observed_state={key: self.latest.values.get(key) for key in ACTION_KEYS},
                    sampled_at=self.latest.sampled_at, timing=self.policy_action_timing,
                    requested_at=self.clock())
            timing = self.policy_action_timing
            self.policy_action_timing = None
            if self.require_policy_timing and timing is None:
                raise GuardError("Missing original ACT chunk timing")
            if timing is not None:
                timing.proposal_at(now=self.clock(),
                    max_age_s=self.guard.limits.max_command_age_s)
                if self.last_policy_command_at is not None:
                    # A slow prediction can be followed immediately by a
                    # cached action in the official sync loop. Space command
                    # creation times, not RPC completions: transport time is
                    # part of the period, rather than added on every frame.
                    remaining = self.last_policy_command_at + 1 / timing.fps - self.clock()
                    if remaining > 0:
                        self.sleep(remaining)
            # Cached targets keep their original scheduled time. Reading a
            # new feedback/camera sample or waiting never refreshes a cached
            # target. Check its original deadline again after pacing.
            proposal_at = (timing.proposal_at(now=self.clock(),
                max_age_s=self.guard.limits.max_command_age_s)
                if timing is not None else self.latest.sampled_at)
            if trace_record is not None:
                trace_record["proposal_at"] = proposal_at
            if set(action) != set(ROLLOUT_KEYS):
                raise GuardError("Expected exactly the seven rollout .pos aliases")
            canonical_action = {canonical: action[alias]
                for canonical, alias in zip(ACTION_KEYS, ROLLOUT_KEYS, strict=True)}
            config = getattr(self.backend, "config", {})
            release_candidate = self.release_motion_state
            if 'policy_release_chunk_extension' in config:
                if not self.guard.armed:
                    raise GuardError('Disarmed: ' + self.guard.reason)
                release_config = config['policy_release_chunk_extension']
                validate_release_continuation(release_config, bounds=self.guard.limits.bounds)
                if (not self.require_policy_timing or timing is None
                        or 'policy_joint_target_margin_deg' not in config
                        or 'policy_gripper_target_margin_m' not in config
                        or config.get('max_relative_target') is None
                        or config.get('policy_target_rate_fraction') is None):
                    raise GuardError('Release continuation requires original timing, joint/gripper projection, feedback-cap and rate shaping')
                self.guard._bounded({key: self.latest.values[key] for key in ACTION_KEYS})
                try:
                    canonical_action, release_candidate, release_phase = gate_release_motion(
                        release_config, action=canonical_action, state=self.latest.values,
                        previous_target=self.guard.last_target, committed=self.release_motion_state,
                        now=self.clock())
                except ValueError as error:
                    raise GuardError(str(error)) from error
                if trace_record is not None:
                    trace_record['release_motion_phase'] = release_phase
                    trace_record['release_gated_prediction'] = dict(canonical_action)
            target_bounds = None
            smoothing_alpha = config.get("policy_joint_smoothing_alpha")
            if "policy_joint_smoothing_alpha" in config:
                validate_joint_smoothing_alpha(smoothing_alpha)
                if ("policy_joint_target_margin_deg" not in config
                        or not self.require_policy_timing or timing is None
                        or config.get("max_relative_target") is None
                        or config.get("policy_target_rate_fraction") is None):
                    raise GuardError("Joint smoothing requires original ACT timing, joint projection, feedback-cap and target-rate shaping")
            if "policy_gripper_target_margin_m" in config:
                if "policy_joint_target_margin_deg" not in config:
                    raise GuardError("policy_gripper_target_margin_m requires joint prediction projection")
                if config["policy_gripper_target_margin_m"] is None:
                    raise GuardError("policy_gripper_target_margin_m must be a finite JSON number in [0, 0.005]")
            if "policy_joint_target_margin_deg" in config:
                if any(key in config for key in ("j5_upper_prediction_saturation",
                        "j5_upper_prediction_tolerance_deg", "j5_upper_target_margin_deg")):
                    raise GuardError("Joint prediction projection and legacy J5 settings are mutually exclusive")
                if not self.require_policy_timing or timing is None:
                    raise GuardError("Joint prediction projection requires original ACT chunk timing")
                if config.get("max_relative_target") is None or config.get("policy_target_rate_fraction") is None:
                    raise GuardError("Joint prediction projection requires feedback-cap and target-rate shaping")
                joint_margin = config["policy_joint_target_margin_deg"]
                gripper_margin = config.get("policy_gripper_target_margin_m")
                target_bounds = joint_policy_target_bounds(self.guard.limits, joint_margin, gripper_margin)
                self.guard._bounded({key: self.latest.values[key] for key in ACTION_KEYS})
                projected, corrections = project_joint_predictions(canonical_action,
                    limits=self.guard.limits, margin_deg=joint_margin, gripper_margin_m=gripper_margin)
                canonical_action = dict(zip(ACTION_KEYS, projected, strict=True))
                for correction in corrections:
                    key = correction["key"]
                    self.joint_projection_counts[key] = self.joint_projection_counts.get(key, 0) + 1
                if trace_record is not None:
                    trace_record["policy_projected_prediction"] = dict(canonical_action)
                    trace_record["policy_target_bounds"] = dict(zip(ACTION_KEYS, target_bounds, strict=True))
                    trace_record["policy_projection_corrections"] = corrections
                    trace_record["policy_projection_counts"] = dict(self.joint_projection_counts)
                    if gripper_margin is not None:
                        trace_record["policy_gripper_target_margin_m"] = gripper_margin
                projected_at = self.clock()
                if corrections and (self.last_joint_projection_log_at is None
                        or projected_at-self.last_joint_projection_log_at >= 1.):
                    LOG.warning("Policy predictions projected corrections=%s counts=%s",
                        corrections, self.joint_projection_counts)
                    self.last_joint_projection_log_at = projected_at
            saturation_present = "j5_upper_prediction_saturation" in config
            saturation = config.get("j5_upper_prediction_saturation", False)
            tolerance = config.get("j5_upper_prediction_tolerance_deg")
            if saturation_present and type(saturation) is not bool:
                raise GuardError("j5_upper_prediction_saturation must be an explicit boolean")
            if saturation_present and "j5_upper_prediction_tolerance_deg" in config:
                raise GuardError("J5 prediction saturation and legacy tolerance cannot both be configured")
            if saturation and (not self.require_policy_timing or timing is None):
                raise GuardError("J5 prediction saturation requires original ACT chunk timing")
            margin = config.get("j5_upper_target_margin_deg", 0.)
            if "j5_upper_target_margin_deg" in config:
                if not saturation:
                    raise GuardError("j5_upper_target_margin_deg requires J5 prediction saturation")
                target_upper = j5_upper_target_bound(self.guard.limits, margin)
                if config.get("max_relative_target") is None or config.get("policy_target_rate_fraction") is None:
                    raise GuardError("J5 target margin requires feedback-cap and target-rate shaping")
                bounds = list(self.guard.limits.bounds)
                bounds[4] = (bounds[4][0], target_upper)
                target_bounds = tuple(bounds)
                if trace_record is not None:
                    trace_record["j5_target_upper_deg"] = target_upper
                    trace_record["j5_upper_target_margin_deg"] = margin
                # An actual limit violation cannot be repaired by a target
                # interval or waiting. Preserve the original strict feedback
                # error before computing the inward target intersection.
                self.guard._bounded({key: self.latest.values[key] for key in ACTION_KEYS})
            if saturation or tolerance is not None:
                if saturation:
                    projected, corrections = saturate_j5_upper_prediction(
                        canonical_action, limits=self.guard.limits, upper_margin_deg=margin)
                else:
                    projected, corrections = project_j5_upper_prediction(
                        canonical_action, limits=self.guard.limits, tolerance_deg=tolerance)
                canonical_action = dict(zip(ACTION_KEYS, projected, strict=True))
                if trace_record is not None:
                    trace_record["j5_projected_prediction"] = dict(canonical_action)
                if saturation:
                    if corrections:
                        saturated_at = self.clock()
                        if not self.policy_bound_saturation_streak:
                            self.policy_bound_saturation_started_at = saturated_at
                        self.policy_bound_saturation_count += 1
                        self.policy_bound_saturation_streak += 1
                        self.policy_bound_saturation_duration_s = max(
                            0., saturated_at-self.policy_bound_saturation_started_at)
                        self.policy_bound_saturation_max_streak = max(
                            self.policy_bound_saturation_max_streak, self.policy_bound_saturation_streak)
                        self.policy_bound_saturation_max_duration_s = max(
                            self.policy_bound_saturation_max_duration_s, self.policy_bound_saturation_duration_s)
                    else:
                        self.policy_bound_saturation_streak = 0
                        self.policy_bound_saturation_started_at = None
                        self.policy_bound_saturation_duration_s = 0.
                    if trace_record is not None:
                        trace_record["j5_saturation"] = {
                            "total": self.policy_bound_saturation_count,
                            "consecutive": self.policy_bound_saturation_streak,
                            "duration_s": self.policy_bound_saturation_duration_s}
                for correction in corrections:
                    self.policy_bound_correction_count += 1
                    if saturation:
                        LOG.warning("Policy upper prediction corrected field=%s raw=%.9f projected=%.9f count=%d saturation_total=%d consecutive=%d duration_s=%.6f",
                            correction["key"], correction["original"], correction["projected"],
                            self.policy_bound_correction_count, self.policy_bound_saturation_count,
                            self.policy_bound_saturation_streak, self.policy_bound_saturation_duration_s)
                    else:
                        LOG.warning("Policy upper prediction corrected field=%s raw=%.9f projected=%.9f count=%d",
                            correction["key"], correction["original"], correction["projected"],
                            self.policy_bound_correction_count)
            cap = getattr(self.backend, "config", {}).get("max_relative_target")
            if cap is not None:
                # As in official follower adapters: cap target displacement
                # from feedback before sending. Bounds remain strict after
                # the explicitly configured J5 policy-output transformation.
                self.guard._bounded(canonical_action)
                canonical_action = ensure_safe_goal_position(
                    {k: (canonical_action[k], self.latest.values[k]) for k in ACTION_KEYS}, cap)
                if trace_record is not None:
                    trace_record["relative_capped_prediction"] = dict(canonical_action)
            if smoothing_alpha is not None:
                if not self.guard.armed:
                    raise GuardError("Disarmed: " + self.guard.reason)
                smoothed = smooth_joint_targets(canonical_action,
                    previous_target=self.guard.last_target, alpha=smoothing_alpha)
                canonical_action = dict(zip(ACTION_KEYS,smoothed,strict=True))
                if trace_record is not None:
                    trace_record["joint_smoothed_prediction"] = dict(canonical_action)
                    trace_record["policy_joint_smoothing_alpha"] = smoothing_alpha
            rate_fraction = getattr(self.backend, "config", {}).get("policy_target_rate_fraction")
            if rate_fraction is not None:
                if not self.guard.armed:
                    raise GuardError("Disarmed: " + self.guard.reason)
                checked_at = self.clock()
                desired = tuple(canonical_action[k] for k in ACTION_KEYS)
                state = tuple(self.latest.values[k] for k in ACTION_KEYS)
                wait_s = target_rate_wait_s(state=state,
                    previous_target=self.guard.last_target,
                    elapsed_s=checked_at-self.guard.last_command_at,
                    limits=self.guard.limits, relative_cap=cap,
                    speed_fraction=rate_fraction, target_bounds=target_bounds)
                if wait_s > 0:
                    # Feedback can keep moving towards a preceding target as
                    # the next chunk reverses direction. Wait only when the
                    # two existing intervals need time to overlap; do not add
                    # RPC completion time to every control period.
                    wait_cap_s = min(.05, 1/timing.fps) if timing is not None else .05
                    if wait_s > wait_cap_s:
                        raise GuardError("Feedback/rate constraints require more than one control period of waiting")
                    original_deadline = min(
                        proposal_at+self.guard.limits.max_command_age_s,
                        self.latest.sampled_at+self.guard.limits.max_observation_age_s,
                        self.guard.last_command_at+self.guard.limits.max_command_gap_s)
                    if checked_at+wait_s >= original_deadline:
                        raise GuardError("Original target or feedback would expire during feedback/rate wait")
                    LOG.info("Waiting %.2fms for feedback/target-rate intervals to overlap", wait_s*1000)
                    self.sleep(wait_s)
                    checked_at = self.clock()
                    if timing is not None:
                        proposal_at = timing.proposal_at(now=checked_at,
                            max_age_s=self.guard.limits.max_command_age_s)
                shaped = shape_target_rate(desired,
                    state=state,
                    previous_target=self.guard.last_target,
                    elapsed_s=checked_at-self.guard.last_command_at,
                    limits=self.guard.limits, relative_cap=cap,
                    speed_fraction=rate_fraction, target_bounds=target_bounds)
                changed = [k for i, k in enumerate(ACTION_KEYS)
                    if abs(desired[i]-shaped[i]) > (0.001 if i < 6 else 0.000001) + 1e-12]
                if changed and (self.last_rate_log_at is None or checked_at-self.last_rate_log_at >= 1.):
                    LOG.info("Policy target rate limited fields=%s elapsed_ms=%.1f fraction=%.2f",
                        changed, (checked_at-self.guard.last_command_at)*1000, rate_fraction)
                    self.last_rate_log_at = checked_at
                canonical_action = dict(zip(ACTION_KEYS, shaped, strict=True))
                if trace_record is not None:
                    trace_record["quantized_target"] = dict(canonical_action)
            command = self.guard.prepare(canonical_action,
                observed_state={k: self.latest.values[k] for k in ACTION_KEYS},
                sampled_at=self.latest.sampled_at,
                proposal_at=proposal_at, now=self.clock())
            if trace_record is not None:
                trace_record.update(final_target=dict(zip(ACTION_KEYS, command.target, strict=True)),
                    command_sequence=command.sequence, created_at=command.created_at,
                    expires_at=command.expires_at, proposal_at=proposal_at)
            self.guard.require_sendable(command, now=self.clock())
            transport_started = True
            self.backend.send_checked(command)
            if trace_record is not None:
                # Host observation of RPC completion, not actual CAN/motion.
                trace_record["backend_ack_at"] = self.clock()
            self.guard.commit(command, now=self.clock())
            self.release_motion_state = release_candidate
            if timing is not None:
                self.last_policy_command_at = command.created_at
            if trace_record is not None:
                trace_record["outcome"] = "accepted"
                trace_append_attempted = True
                trace.append(trace_record)
            return dict(zip(ROLLOUT_KEYS, command.target, strict=True))
        except BaseException as error:
            if trace_record is not None and not trace_append_attempted:
                trace_record["outcome"] = "rejected"
                trace_record["transport_outcome_unknown"] = (
                    transport_started and trace_record["backend_ack_at"] is None)
                trace_record["error"] = {"type": type(error).__name__, "message": str(error)}
                try:
                    trace.append(trace_record)
                except BaseException as trace_error:
                    error.add_note(f"Policy target trace append also failed: {trace_error}")
                    LOG.error("Policy target trace append failed while retaining the original error", exc_info=True)
            elif trace_record is not None:
                LOG.error("Policy target trace failed after command outcome=%s sequence=%s: %s",
                    trace_record["outcome"], trace_record["command_sequence"], error)
            self._disarm_preserving_error("Target rejected or transport outcome unknown", error)
            raise

    def disarm(self, reason="explicit disarm"):
        if self.joint_projection_counts:
            LOG.info("Policy prediction projection summary counts=%s reason=%s", self.joint_projection_counts, reason)
            self.joint_projection_counts = {}
        self.last_joint_projection_log_at = None
        if self.policy_bound_correction_count:
            LOG.info("Policy upper-bound corrections total=%d; disarm reason=%s",
                self.policy_bound_correction_count, reason)
            self.policy_bound_correction_count = 0
        if self.policy_bound_saturation_count:
            LOG.info("Policy J5 saturation summary total=%d max_consecutive=%d max_duration_s=%.6f reason=%s",
                self.policy_bound_saturation_count, self.policy_bound_saturation_max_streak,
                self.policy_bound_saturation_max_duration_s, reason)
        self.policy_bound_saturation_count = 0
        self.policy_bound_saturation_streak = 0
        self.policy_bound_saturation_started_at = None
        self.policy_bound_saturation_duration_s = 0.
        self.policy_bound_saturation_max_streak = 0
        self.policy_bound_saturation_max_duration_s = 0.
        self.policy_action_timing = None
        self.release_motion_state = None
        self.last_policy_command_at = None
        self.last_rate_log_at = None
        try:
            if self.guard is not None:
                self.guard.disarm(reason)
            if self.backend is not None and self.connected:
                self.backend.invalidate_execution_lease(reason)
        except BaseException as error:
            self._close_policy_target_trace(preserve_error=error)
            raise
        self._close_policy_target_trace()

    def disconnect(self):
        try:
            self.disarm("disconnect")
        finally:
            try:
                if self.backend is not None and self.connected:
                    self.backend.close()
            finally:
                self.connected = False
                self.latest = None
                self.owner.release()

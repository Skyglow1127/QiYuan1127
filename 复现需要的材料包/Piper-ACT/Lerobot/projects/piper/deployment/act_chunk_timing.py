"""Timing metadata around official ACT.select_action; no hardware or control loop.

ACT's cached targets have explicit future time slots. Their slot is derived
from the observation that generated the chunk, never from a later camera frame.
The existing command-age bound becomes the allowed lateness for each slot;
feedback age, receiver TTL, lease and motion limits remain independently checked.
"""
from contextlib import contextmanager
from dataclasses import dataclass
import math
import logging

from lerobot_robot_piper_executor.guard import GuardError
from act_release_continuation import release_is_pending, validate_release_continuation

LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class ACTActionTiming:
    observed_at: float
    generated_at: float
    selected_at: float
    action_index: int
    action_count: int
    fps: float

    @property
    def scheduled_at(self):
        return self.observed_at + self.action_index / self.fps

    def proposal_at(self, *, now, max_age_s):
        values = (self.observed_at, self.generated_at, self.selected_at,
                  self.fps, now, max_age_s)
        if (not all(math.isfinite(v) for v in values) or self.fps <= 0
                or max_age_s <= 0 or not 0 <= self.action_index < self.action_count
                or not self.observed_at <= self.generated_at <= self.selected_at <= now):
            raise GuardError("Invalid ACT chunk timing")
        # A later observation must not make an old queued action fresh again.
        if not 0 <= now - self.scheduled_at <= max_age_s:
            raise GuardError("ACT cached target missed its original scheduled window")
        return self.scheduled_at


@contextmanager
def track_act_chunks(policy, robot, *, fps, release_extension=None):
    """Keep the official action queue and attach one-use timing to its outputs."""
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("Positive finite ACT control frequency required")
    original = policy.select_action
    owned_method = "select_action" in vars(policy)
    previous_required = getattr(robot, "require_policy_timing", False)
    observed_at = generated_at = None
    action_index = 0
    count = policy.config.n_action_steps
    ensemble = policy.config.temporal_ensemble_coeff is not None
    if not 1 <= count <= policy.config.chunk_size:
        raise ValueError("ACT action count must fit its prediction chunk")
    active_count = count
    cached_prediction = last_sent_raw = None
    if release_extension is not None:
        validate_release_continuation(release_extension, bounds=robot.guard.limits.bounds)
        if ensemble:
            raise ValueError('Release continuation requires the ACT action queue, without temporal ensembling')
        original_predict = policy.predict_action_chunk
        owned_predict = 'predict_action_chunk' in vars(policy)
        original_send = robot.send_action
        owned_send = 'send_action' in vars(robot)

        def predict_and_cache(*args, **kwargs):
            nonlocal cached_prediction
            cached_prediction = None
            prediction = original_predict(*args, **kwargs)
            # Preserve the full original prediction before official select_action
            # slices its first n_action_steps. No additional inference or target.
            cached_prediction = prediction.detach().clone()
            return prediction

        def observe_successful_send(action):
            nonlocal last_sent_raw
            last_sent_raw = None
            result = original_send(action)
            last_sent_raw = dict(action)
            return result

    def select_action(batch):
        nonlocal observed_at, generated_at, action_index, active_count, last_sent_raw
        if robot.latest is None:
            raise GuardError("ACT prediction requires a source observation")
        if robot.policy_action_timing is not None:
            raise GuardError("Previous ACT target timing was not consumed")
        if (release_extension is not None and not policy._action_queue
                and action_index == count and observed_at is not None
                and release_is_pending(release_extension, raw_action=last_sent_raw,
                    state=robot.latest.values)):
            extra = min(release_extension['extra_steps'], count, policy.config.chunk_size-count)
            if extra:
                if (cached_prediction is None or cached_prediction.ndim != 3
                        or cached_prediction.shape[1] != policy.config.chunk_size
                        or cached_prediction.shape[2] != 7):
                    raise GuardError('Missing full original ACT prediction for release continuation')
                # Do not refresh a cached target's source or original deadline.
                ACTActionTiming(observed_at, generated_at, robot.clock(),
                    action_index, count+extra, fps).proposal_at(now=robot.clock(),
                        max_age_s=robot.guard.limits.max_command_age_s)
                active_count = count+extra
                policy._action_queue.extend(cached_prediction[:, count:active_count].transpose(0, 1))
                LOG.info('ACT release continuation original_steps=%d extra_steps=%d actual_gripper_mm=%.1f model_gripper_mm=%.1f',
                    count, extra, robot.latest.values['gripper.m']*1000, last_sent_raw['gripper.pos']*1000)
        new_chunk = ensemble or not policy._action_queue
        if new_chunk:
            observed_at = robot.latest.sampled_at
            action_index = 0
            active_count = count
            last_sent_raw = None
        elif observed_at is None:
            raise GuardError("ACT queue has no original observation timestamp")
        inference_started = robot.clock()
        action = original(batch)
        selected_at = robot.clock()
        if new_chunk:
            generated_at = selected_at
            LOG.info("ACT chunk generated actions=%d inference_ms=%.1f source_age_ms=%.1f",
                1 if ensemble else count, (selected_at-inference_started)*1000,
                (selected_at-observed_at)*1000)
        robot.policy_action_timing = ACTActionTiming(
            observed_at, generated_at, selected_at, action_index,
            1 if ensemble else active_count, fps)
        action_index += 1
        return action

    robot.require_policy_timing = True
    robot.policy_action_timing = None
    policy.select_action = select_action
    if release_extension is not None:
        policy.predict_action_chunk = predict_and_cache
        robot.send_action = observe_successful_send
    try:
        yield
    finally:
        if release_extension is not None:
            if owned_predict:
                policy.predict_action_chunk = original_predict
            else:
                delattr(policy, 'predict_action_chunk')
            if owned_send:
                robot.send_action = original_send
            else:
                delattr(robot, 'send_action')
            policy._action_queue.clear()
        if owned_method:
            policy.select_action = original
        else:
            delattr(policy, "select_action")
        robot.policy_action_timing = None
        robot.require_policy_timing = previous_required

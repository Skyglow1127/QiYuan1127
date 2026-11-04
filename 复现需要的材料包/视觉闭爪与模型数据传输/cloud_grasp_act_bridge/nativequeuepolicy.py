"""Cloud transport plus official PI05 FIFO with explicit runtime20 override.

No model is allocated here. Saved model processors run once in the cloud; the
local official select_action/reset methods operate on already postprocessed
absolute chunks. Hardware timing metadata is independent of policy selection.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import logging
import math

import numpy as np
import torch
from lerobot.policies.pi05.modeling_pi05 import PI05Policy

TASK = 'Pick up the red cube and place it inside the black square on the left side of the table.'
CHUNK_SIZE = 50
CHECKPOINT_SAVED_ACTION_STEPS = 10
RUNTIME_ACTION_STEPS = 20
DATASET_FPS = 20.
SLOT_TIMING_ID = 'pi05-runtime20-queue-original-source-slots/v1'
QUEUE_CONTRACT = {
    'policy_type': 'pi05', 'chunk_size': CHUNK_SIZE,
    'n_action_steps': RUNTIME_ACTION_STEPS,
    'n_action_steps_source': 'explicit user-authorized runtime reduction20; checkpoint saved10',
    'action_indices': list(range(RUNTIME_ACTION_STEPS)),
    'selection': 'official_PI05Policy.select_action',
    'reset': 'official_PI05Policy.reset',
    'fps': DATASET_FPS,
    'timing_id': SLOT_TIMING_ID,
    'timing': 'original_source_plus_action_index_over_fps',
    'scheduled_slot': 'original_chunk_source_monotonic + action_index / fps',
    'max_slot_lateness_s': .4,
    'original_source_age_is_separately_reported': True,
    'current_feedback_is_separately_checked': True,
    'postprocessing': 'saved official pipeline once in cloud; local identity boundary',
}
LOG = logging.getLogger(__name__)


class CloudExpertPolicy(PI05Policy):
    """Only transport overrides; select_action/reset are the official methods."""

    def __init__(self, config, client):
        # PI05Policy.__init__ allocates the 3B model. This transport adapter owns
        # no parameters; only its inherited queue needs initialization locally.
        torch.nn.Module.__init__(self)
        if (config.type != 'pi05' or config.chunk_size != CHUNK_SIZE
                or config.n_action_steps != RUNTIME_ACTION_STEPS
                or config.num_inference_steps != 10 or config.use_relative_actions
                or config.rtc_config is not None):
            raise ValueError('Cloud proxy requires the explicitly overridden absolute PI05 chunk50/consume20, 10-step, non-RTC config')
        self.config, self.client = config, client
        self.calls = 0
        self.last_result = None
        self.reset()

    def predict_action_chunk(self, observation, **kwargs):
        if kwargs:
            raise ValueError('The saved non-RTC remote queue does not accept action transformations or RTC kwargs')
        if observation.get('task') != TASK:
            raise RuntimeError('Unexpected task in the official inference pipeline')
        result = self.client.infer(observation['observation.state'],
            observation['observation.images.global'], observation['observation.images.wrist'], TASK)
        actions = np.asarray(result['actions'], dtype=np.float32)
        if actions.shape != (CHUNK_SIZE, 7) or not np.isfinite(actions).all():
            raise RuntimeError('Cloud GPU returned invalid action shape or non-finite values')
        self.calls += 1
        self.last_result = result
        LOG.info('Official PI05 FIFO fill request=%s chunk=%d consume=%d model_ms=%.1f total_ms=%.1f; '
                 'raw absolute targets unchanged, original source bound by queue timing adapter',
                 result.get('request_id'), CHUNK_SIZE, self.config.n_action_steps,
                 result['timing_s']['model']*1000,
                 result['timing_s']['input_ready_to_validated_actions']*1000)
        LOG.info('PI05 transport timing request=%s request_bytes=%s phases_s=%s server_phases_s=%s',
                 result.get('request_id'), result.get('request_bytes'), result['timing_s'],
                 result.get('response', {}).get('server_timing_s', {}))
        return torch.from_numpy(actions.copy()).unsqueeze(0)


@dataclass(frozen=True)
class NativeQueueTiming:
    observed_at: float
    generated_at: float
    selected_at: float
    action_index: int
    action_count: int
    fps: float

    @property
    def scheduled_at(self):
        return self.observed_at + self.action_index/self.fps

    def original_source_age_s(self, *, now):
        return now-self.observed_at

    def slot_lateness_s(self, *, now):
        return now-self.scheduled_at

    def proposal_at(self, *, now, max_age_s):
        # This is explicitly a source-derived scheduled slot, not a fabricated
        # new capture time. Fresh current feedback remains a separate guard.
        from lerobot_robot_piper_executor.guard import GuardError
        values = (self.observed_at, self.generated_at, self.selected_at,
                  self.fps, now, max_age_s)
        if (not all(isinstance(value, (int, float)) and not isinstance(value, bool)
                    and math.isfinite(value) for value in values)
                or self.fps <= 0 or max_age_s <= 0
                or type(self.action_index) is not int or type(self.action_count) is not int
                or not 0 <= self.action_index < self.action_count
                or not self.observed_at <= self.generated_at <= self.selected_at <= now):
            raise GuardError('Invalid original PI05 queue timing')
        if not 0 <= self.slot_lateness_s(now=now) <= max_age_s:
            raise GuardError('PI05 cached target missed its original source-derived scheduled slot')
        return self.scheduled_at


@contextmanager
def track_native_queue(policy, robot, *, fps):
    """Attach one-use source/slot metadata without replacing official FIFO logic.

    No target, source timestamp or queue index is transformed. The existing
    hardware adapter separately checks current feedback, pacing, actual target,
    delivery TTL, receiver lease and independent receiver limits.
    """
    from lerobot_robot_piper_executor.guard import GuardError, vector
    if (not math.isfinite(fps) or fps != DATASET_FPS
            or policy.config.n_action_steps != RUNTIME_ACTION_STEPS
            or policy.config.chunk_size != CHUNK_SIZE):
        raise GuardError('PI05 FIFO requires explicit runtime20, checkpoint saved10, chunk50 and dataset20Hz scheduling')
    if type(policy).select_action is not PI05Policy.select_action or type(policy).reset is not PI05Policy.reset:
        raise GuardError('PI05 selection/reset must be the installed unmodified official methods')
    original = policy.select_action
    owned_method = 'select_action' in vars(policy)
    previous_required = robot.require_policy_timing
    observed_at = generated_at = None
    action_index = 0

    def select_action(batch):
        nonlocal observed_at, generated_at, action_index
        if robot.latest is None or robot.policy_action_timing is not None:
            raise GuardError('PI05 requires current feedback and previously consumed target timing')
        new_chunk = not policy._action_queue
        source = robot.latest
        if new_chunk:
            observed_at = source.sampled_at
            action_index = 0
        elif observed_at is None or generated_at is None:
            raise GuardError('Official PI05 queue has no original chunk source metadata')
        action = original(batch)
        selected_at = robot.clock()
        if new_chunk:
            if robot.latest is not source:
                raise GuardError('PI05 source changed during synchronous chunk prediction')
            generated_at = selected_at
        raw = action.detach().cpu().numpy() if hasattr(action, 'detach') else action
        if getattr(raw, 'shape', None) == (1, 7):
            raw = raw[0]
        vector(raw)  # Reject malformed/non-finite outputs; do not change targets.
        if len(policy._action_queue) != RUNTIME_ACTION_STEPS-action_index-1:
            raise GuardError('Official PI05 FIFO length diverged from the explicit runtime action count')
        timing = NativeQueueTiming(observed_at, generated_at, selected_at,
            action_index, RUNTIME_ACTION_STEPS, fps)
        try:
            timing.proposal_at(now=selected_at, max_age_s=robot.guard.limits.max_command_age_s)
        except GuardError:
            LOG.error('PI05 queue timing rejected before send: request=%s slot=%d/%d '
                      'source=%.9f generated=%.9f selected=%.9f scheduled=%.9f '
                      'source_age_ms=%.3f slot_lateness_ms=%.3f limit_ms=%.3f; '
                      'original source retained, no target sent',
                      (policy.last_result or {}).get('request_id'), action_index,
                      RUNTIME_ACTION_STEPS, observed_at, generated_at, selected_at,
                      timing.scheduled_at, timing.original_source_age_s(now=selected_at)*1000,
                      timing.slot_lateness_s(now=selected_at)*1000,
                      robot.guard.limits.max_command_age_s*1000)
            raise
        LOG.info('PI05 queue timing contract=%s slot=%d/%d chunk_source=%.9f scheduled_slot=%.9f '
                 'selected=%.9f original_source_age_ms=%.1f slot_lateness_ms=%.1f '
                 'current_feedback_sampled_at=%.9f; no source timestamp refreshed',
                 SLOT_TIMING_ID, action_index, RUNTIME_ACTION_STEPS, observed_at,
                 timing.scheduled_at, selected_at, timing.original_source_age_s(now=selected_at)*1000,
                 timing.slot_lateness_s(now=selected_at)*1000, robot.latest.sampled_at)
        robot.policy_action_timing = timing
        action_index += 1
        return action

    robot.require_policy_timing = True
    robot.policy_action_timing = None
    policy.select_action = select_action
    try:
        yield
    finally:
        if owned_method:
            policy.select_action = original
        else:
            delattr(policy, 'select_action')
        # Clear an unconsumed block on all exits; no old target crosses trials.
        policy.reset()
        robot.policy_action_timing = None
        robot.require_policy_timing = previous_required

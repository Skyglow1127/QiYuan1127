"""VLA-only closure veto around the existing guarded hardware adapter.

The RGB condition is an empirical rejection of an obviously distant cube. It
does not certify alignment, contact or a grasp. All physical sends still use
the existing adapter/receiver and the original policy timing object.
"""
from contextlib import contextmanager
from dataclasses import asdict
import logging

from visual_gripper_gate import GateState, red_geometry, shape_gripper, validate_config

LOG = logging.getLogger(__name__)
KEYS = tuple(f'joint_{i}.deg' for i in range(1, 7)) + ('gripper.m',)
ALIASES = tuple(f'joint_{i}.pos' for i in range(1, 7)) + ('gripper.pos',)
WRIST_SERIAL = 'CC1365301EY'
MAX_DEFER_S = 8.0


@contextmanager
def visual_gripper_control(robot, config, *, closure_config=None, policy=None):
    """Commit phase only after acknowledged send; never retry or refresh timing."""
    from lerobot_robot_piper_executor.guard import GuardError, vector
    validate_config(config)
    sync_enabled = closure_config is not None
    if sync_enabled:
        from closure_sync import ClosureSyncState, shape_closure, validate_config as validate_sync
        closure_config = validate_sync(closure_config)
        if policy is None or not callable(getattr(policy, 'reset', None)):
            raise GuardError('Coordinated closure requires the original policy reset interface')
        sync_phase = ClosureSyncState()
    else:
        sync_phase = None
    original = robot.send_action
    owned = 'send_action' in vars(robot)
    trace = robot.policy_target_trace
    if trace is None:
        raise GuardError('Visual gripper control requires the existing policy target trace')
    trace_start = trace.start
    trace_owned = 'start' in vars(trace)
    pending_trace = None
    phase = GateState()
    deferred_at = None
    counts = {'calls': 0, 'held': 0, 'closing_started': 0}

    def traced_start(*args, **kwargs):
        record = trace_start(*args, **kwargs)
        if pending_trace is not None:
            # Keep the model's actual raw FIFO output, plus the explicit local
            # veto output. Do not label the veto output as the raw prediction.
            record['raw_prediction'] = dict(zip(KEYS, pending_trace['raw'], strict=True))
            record['visual_gated_prediction'] = dict(zip(KEYS, pending_trace['candidate'], strict=True))
            record['visual_gripper_gate'] = pending_trace['diagnostic']
            if pending_trace.get('sync') is not None:
                record['local_closure_sync'] = pending_trace['sync']
                record['local_phase_prediction'] = dict(zip(KEYS, pending_trace['local'], strict=True))
                record['model_input_observed_state'] = pending_trace['input_state']
                record['model_input_sampled_at'] = pending_trace['input_sampled_at']
                record['model_input_fields_semantics'] = 'Current rollout observation before extra feedback read; for cached FIFO actions this is NOT the original chunk model input. Original model source is unchanged act_timing.observed_at.'
                record['closure_feedback_snapshot'] = pending_trace['feedback']
        return record

    def send_action(action):
        nonlocal phase, deferred_at, pending_trace, sync_phase
        if set(action) != set(ALIASES) or robot.latest is None or not robot.guard.armed:
            raise GuardError('Visual gripper control requires the armed seven-channel action and current observation')
        raw = vector(tuple(action[key] for key in ALIASES))
        input_observation = robot.latest
        input_state = {key: input_observation.values[key] for key in KEYS}
        feedback = None
        if sync_enabled:
            # Feedback-only read after inference. Preserve camera capture time
            # and the *unchanged* model timing object; no new model source is
            # claimed. This is a local reviewed feedback phase controller.
            snapshot = robot.backend.client.request('snapshot')
            snapshot_at = robot.backend.client.host_source_time(snapshot['source_ns'])
            fresh = vector(snapshot['state'])
            robot.latest = type(input_observation)(
                {**input_observation.values, **dict(zip(KEYS, fresh, strict=True))},
                min(input_observation.sampled_at, snapshot_at))
            feedback = {**snapshot, 'mapped_source_at': snapshot_at,
                        'purpose': 'fresh local phase feedback; never replaces original model source'}
        observed = vector(tuple(robot.latest.values[key] for key in KEYS))
        camera = robot.backend.cameras.get('wrist')
        if camera is None or camera.config.serial != WRIST_SERIAL:
            raise GuardError('Visual gripper control requires the reviewed wrist camera identity')
        frame_id = camera.last_index
        geometry = red_geometry(robot.latest.values.get('wrist'), config)
        candidate, proposed, diagnostic = shape_gripper(raw, observed,
            robot.guard.last_target, phase, geometry, frame_id, config)
        visual_candidate = candidate
        sync_diagnostic = None
        proposed_sync = sync_phase
        if sync_enabled:
            timing = robot.policy_action_timing
            if timing is None:
                raise GuardError('Coordinated closure requires the unchanged original model timing')
            awaiting_replan = sync_phase.phase == 'contact_ready'
            if awaiting_replan and timing.action_index != 0:
                raise GuardError('Coordinated closure requires a fresh FIFO prediction, not remaining cached actions')
            candidate, proposed_sync, sync_diagnostic = shape_closure(
                visual_candidate, observed, sync_phase,
                now=robot.clock(), observed_source_ns=snapshot['source_ns'],
                snapshot=snapshot, snapshot_at=snapshot_at,
                raw_source_at=timing.observed_at,
                previous_ack_target7=robot.guard.last_target,
                closure_latched=proposed.closing_started and not phase.closing_started,
                new_prediction=awaiting_replan and timing.action_index == 0,
                reset=diagnostic['reset'], config=closure_config)
            if candidate is None or sync_diagnostic.failure_reason:
                raise GuardError('Coordinated closure stopped: ' + str(sync_diagnostic.failure_reason))
        held = visual_candidate[6] != raw[6]
        now = robot.clock()
        pending_deferred = deferred_at
        if held and pending_deferred is None:
            pending_deferred = now
        if proposed.closing_started:
            pending_deferred = None
        if pending_deferred is not None and now-pending_deferred > MAX_DEFER_S:
            raise GuardError('Red cube has not met the reviewed wrist approach condition within8s of deferred closure; no grasp confirmed')
        diagnostic = {**diagnostic, 'geometry': geometry,
            'phase_before': asdict(phase), 'phase_candidate': asdict(proposed),
            'frame_id': frame_id, 'source_sampled_at': robot.latest.sampled_at,
            'raw_gripper_m': raw[6], 'candidate_gripper_m': visual_candidate[6],
            'held': held, 'deferred_elapsed_s': 0. if pending_deferred is None else now-pending_deferred}
        pending_trace = {'raw': raw, 'candidate': visual_candidate, 'local': candidate,
            'diagnostic': diagnostic, 'sync': None if sync_diagnostic is None else asdict(sync_diagnostic),
            'input_state': input_state, 'input_sampled_at': input_observation.sampled_at,
            'feedback': feedback}
        try:
            # Local phase targets and unchanged original model timing pass through. The adapter
            # still applies projection, feedback cap, pacing and both guards.
            result = original(dict(zip(ALIASES, candidate, strict=True)))
        finally:
            pending_trace = None
        if robot.policy_action_timing is not None:
            raise GuardError('Existing adapter did not consume the original policy timing')
        # No state is committed on a rejected/unknown send.
        counts['calls'] += 1
        counts['held'] += int(held)
        counts['closing_started'] += int(proposed.closing_started and not phase.closing_started)
        phase, deferred_at = proposed, pending_deferred
        if sync_enabled:
            # The completed local constraint is committed only after the ACK.
            # Discard old FIFO once, then require a subsequent real prediction.
            should_replan = proposed_sync.phase == 'contact_ready' and sync_phase.phase != 'contact_ready'
            sync_phase = proposed_sync
            if should_replan:
                policy.reset()
                LOG.info('Local closure holding candidate: discarded unexecuted FIFO; next action must use a fresh observation. No grasp success claimed.')
        if held and counts['held'] % 20 == 1:
            LOG.info('Visual gripper closure deferred raw_mm=%.2f candidate_mm=%.2f geometry=%s',
                raw[6]*1000, candidate[6]*1000, geometry)
        return result

    trace.start = traced_start
    robot.send_action = send_action
    try:
        yield counts
    finally:
        if owned:
            robot.send_action = original
        else:
            delattr(robot, 'send_action')
        if trace_owned:
            trace.start = trace_start
        else:
            delattr(trace, 'start')
        LOG.info('Visual gripper veto summary=%s final_phase=%s closure_sync=%s; visual/width evidence is not grasp confirmation', counts, asdict(phase), None if sync_phase is None else asdict(sync_phase))

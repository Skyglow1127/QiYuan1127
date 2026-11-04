"""Deliver the approved arm reference and its time axis to the board as a unit.

This path has no per-axis ACK waits and never silently falls back to the old
serial CLI. The board establishes the epoch, owns segment dispatch and judges
tracking/arrival from valid feedback. Gripper changes remain a separate stage.
"""
from __future__ import annotations
import math
import uuid
import numpy as np


def execution_parameters(plan):
    return dict(schema='d1_timed_trajectory_v1', scheduling_policy='timed_group_v1',
        chunk_points=64, start_after_ms=1000, max_lateness_ms=20,
        max_tracking_error_deg=3., velocity_limit_deg_s=float(plan['max_velocity_deg_s']),
        acceleration_limit_deg_s2=float(plan['max_acceleration_deg_s2']),
        gripper_mode='preserve', endpoint_tolerance_deg=1., endpoint_stability_span_deg=.3,
        endpoint_valid_frames=3, endpoint_timeout_s=5., board_buffer_points=4096,
        maximum_duration_s=120., minimum_segment_interval_ms=20.)


def execute_timed(plan, transport, *, cancel=lambda: False,
                  external_check=lambda: None, feedback=lambda q: None,
                  gripper_target=None, endpoint_tolerance_deg=1., endpoint_policy=None):
    from .sdk_executor import Cancelled, healthy, references
    if not hasattr(transport, 'submit_chunk') or not hasattr(transport, 'cancel'):
        raise ValueError('timed_group_v1 requires TimedGroupTransport; legacy CLI fallback is forbidden')
    if endpoint_policy is not None:
        raise ValueError('Timed execution does not resend endpoints or change targets')
    if endpoint_tolerance_deg != 1.:
        raise ValueError('The current board arrival contract is fixed at one degree')
    parameters=execution_parameters(plan)
    speed=parameters['velocity_limit_deg_s'];acceleration=parameters['acceleration_limit_deg_s2']
    if not 0<speed<=5. or not 0<acceleration<=10.:
        raise ValueError('Timed reference exceeds the coded speed/acceleration envelope')
    if not 0<float(plan['duration_s'])<=120.:
        raise ValueError('Timed arm stage must fit the 120-second board buffer')
    # No command is sent while obtaining and validating the present start.
    ready_deadline=transport.now()+4.
    while True:
        external_check()
        if cancel():raise Cancelled('Cancelled before timed submission')
        snapshot=transport.snapshot()
        if snapshot.get('fault'):raise RuntimeError(snapshot['fault'])
        if snapshot.get('q') is not None:
            actual=healthy(snapshot,transport.now(),allow_unstarted_position=True).copy();break
        if transport.now()>=ready_deadline:raise TimeoutError('No valid starting feedback for timed trajectory')
        transport.wait(.02)
    grip=float(actual[6])
    if gripper_target is not None and abs(float(gripper_target)-grip)>.5:
        raise ValueError('Timed arm trajectory preserves measured gripper; run an approved gripper stage first')
    points=[]
    for seconds, angles in references(plan):
        value=np.asarray(angles,dtype=float)
        if value.shape!=(6,) or not np.isfinite(value).all() or not math.isfinite(float(seconds)):
            raise ValueError('Malformed timed reference')
        points.append(dict(time_from_start_ms=round(float(seconds)*1000.,6),
                           sdk_angles=[*map(float,value),grip]))
    if not 2<=len(points)<=4096:raise ValueError('Timed buffer requires 2..4096 points')
    if points[0]['time_from_start_ms']!=0 or any(
            b['time_from_start_ms']-a['time_from_start_ms']<20.
            for a,b in zip(points,points[1:])):
        raise ValueError('Timed reference needs time zero and at least 20 ms between points')
    if np.max(abs(np.array(points[0]['sdk_angles'])-actual))>.3+1e-5:
        raise ValueError('Timed reference start differs from valid feedback by more than 0.3 degrees')
    trajectory_id=uuid.uuid4().hex
    # Pre-materialise the complete reference before the first publish starts
    # the board's lead-in clock. Chunks are transmitted without ACK gating.
    chunk_size=parameters['chunk_points']
    chunks=[points[i:i+chunk_size] for i in range(0,len(points),chunk_size)]
    submitted=False;last_feedback=snapshot['q_seq'];begin=transport.now()
    deadline=begin+1.+float(plan['duration_s'])+8.
    try:
        for index,chunk in enumerate(chunks):
            external_check()
            if cancel():raise Cancelled('Cancelled during timed buffer submission')
            transport.submit_chunk(trajectory_id,index,chunk,final=index==len(chunks)-1,
                start_after_ms=parameters['start_after_ms'],max_lateness_ms=parameters['max_lateness_ms'],
                max_tracking_error_deg=parameters['max_tracking_error_deg'],
                velocity_limit_deg_s=speed,acceleration_limit_deg_s2=acceleration)
            submitted=True
        while True:
            external_check()
            if cancel():raise Cancelled('Timed trajectory cancellation requested')
            snapshot=transport.snapshot()
            if snapshot.get('fault'):raise RuntimeError(snapshot['fault'])
            actual=healthy(snapshot,transport.now(),allow_unstarted_position=True)
            if snapshot['q_seq']!=last_feedback:
                last_feedback=snapshot['q_seq'];feedback(actual.copy())
            state=snapshot.get('trajectory_states',{}).get(trajectory_id,{})
            if state.get('state') in ('fault','cancelled'):
                raise RuntimeError(state.get('reason') or 'Board terminated timed trajectory')
            if state.get('state')=='completed':
                endpoint=np.array(points[-1]['sdk_angles'])
                if np.max(abs(actual[:6]-endpoint[:6]))>1.+1e-5:
                    raise RuntimeError('Board completion disagrees with latest valid endpoint feedback')
                return dict(completed=True,trajectory_id=trajectory_id,scheduling_policy='timed_group_v1',
                    original_reference_duration_s=float(plan['duration_s']),execution_parameters=parameters,
                    elapsed_host_s=transport.now()-begin,points=len(points),chunks=len(chunks),
                    actual_sdk=actual.tolist(),board_state=state,gripper_mode='preserve',
                    per_axis_ack_wait=False,clock_owner='board steady_clock',
                    arrival_basis='three fresh valid board encoder frames within 1 degree, span <=0.3 degree',
                    hardware_atomic=False,physical_accuracy_verified=False)
            if transport.now()>=deadline:raise TimeoutError('Board timed trajectory exceeded its explicit completion budget')
            transport.wait(.01)
    except BaseException as error:
        if submitted:
            try:transport.cancel(trajectory_id)
            except Exception as cancel_error:error.cancel_delivery_error=str(cancel_error)
        error.trajectory_id=trajectory_id;error.future_commands_stopped=True
        error.physical_stop_verified=False
        raise

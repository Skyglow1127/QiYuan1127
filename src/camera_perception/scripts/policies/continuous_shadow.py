"""Continuous historical-observation ACT interface diagnostics, without robot I/O.

Historical observations remain teacher-trajectory observations; they are never
represented as feedback caused by the policy. No proposal is sent or clipped.
The explicit candidate schedule chooses raw index 1 at its original timestamp
(next nominal tick), rather than moving index 0 to a later timestamp.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import time

import numpy as np

from .contracts import ActionChunk,LIMITS,validate_sample
from .prepare import read,sha

SPEED=.5
ACCEL=1.
EPS=1e-5
SCHEDULE='raw_index_1_at_original_next_tick_no_time_shift'


def connection(points):
    """Finite differences using actual timestamps and velocity midpoint times.

    Two positions determine one average velocity, never an initial acceleration.
    Arm derivative bounds do not apply degree units to the raw J6 position.
    """
    if len(points)<2:
        return dict(speed_known=False,acceleration_known=False,speed_pass=None,acceleration_pass=None)
    t=np.asarray([p['time'] for p in points],float)
    q=np.asarray([p['q'] for p in points],float)
    if q.shape!=(len(points),7) or not np.isfinite(q).all() or not np.isfinite(t).all() or np.any(np.diff(t)<=0):
        raise ValueError('connection_requires_finite_positions_and_increasing_times')
    dt=np.diff(t);velocity=np.diff(q[:,:6],axis=0)/dt[:,None]
    middle=(t[1:]+t[:-1])/2
    acceleration=np.diff(velocity,axis=0)/np.diff(middle)[:,None] if len(velocity)>1 else np.empty((0,6))
    return dict(speed_known=True,acceleration_known=bool(len(acceleration)),
        time_intervals_s=dt.tolist(),last_velocity_deg_s=velocity[-1].tolist(),
        last_speed_deg_s=float(abs(velocity[-1]).max()),
        last_acceleration_deg_s2=acceleration[-1].tolist() if len(acceleration) else None,
        last_acceleration_max_deg_s2=float(abs(acceleration[-1]).max()) if len(acceleration) else None,
        speed_pass=bool(abs(velocity[-1]).max()<=SPEED+EPS),
        acceleration_pass=bool(abs(acceleration[-1]).max()<=ACCEL+EPS) if len(acceleration) else None,
        gripper_last_change_sdk_raw=float(q[-1,6]-q[-2,6]),
        gripper_derivative_units='not degrees; no force/contact or gripper speed certification')


def break_reason(previous,current,rate,tolerance=.05):
    if previous is None:return 'start_of_selected_records'
    if any(previous.get(k)!=current.get(k) for k in ('split','episode_group','segment_id','action_source_log')):
        return 'source_segment_or_episode_changed'
    if current['sample_index']!=previous['sample_index']+1:return 'sample_index_gap'
    dt=current['command_host_monotonic']-previous['command_host_monotonic']
    if abs(dt-1/rate)>tolerance:return 'historical_command_time_gap'
    return None


def review(chunk,observation,previous_observations,previous_commands,previous_proposals,
           trigger_time,latency_s,rate,candidate_index=1):
    """Compare independent history anchors and the hypothetical raw proposal stream."""
    chunk.validate()
    if candidate_index!=1:raise ValueError('Only the explicitly declared next-tick index-1 schedule is supported')
    if not math.isfinite(latency_s) or latency_s<0:raise ValueError('invalid_inference_latency')
    if len(chunk.commands)<=candidate_index:raise ValueError('candidate_index_outside_chunk')
    targets=np.asarray(chunk.commands,float);times=np.asarray(chunk.target_times,float)
    expected=trigger_time+np.arange(len(times))/rate
    if not np.allclose(times,expected,atol=1e-7,rtol=0):raise ValueError('raw_chunk_time_origin_or_rate_mismatch')
    completion=trigger_time+latency_s
    reasons=[]
    qtime=observation['time']
    if not qtime<trigger_time or trigger_time-qtime>.6:reasons.append('noncausal_or_stale_current_feedback')
    for name,history in [('observations',previous_observations),('commands',previous_commands)]:
        if any(p['time']>=trigger_time for p in history):raise ValueError('future_recorded_'+name+'_cannot_be_a_boundary_anchor')
    selected=dict(time=float(times[candidate_index]),q=targets[candidate_index].tolist(),
                  observation_id=chunk.observation_id,kind='unsent_raw_policy_proposal')
    late=completion>=selected['time']
    if late:reasons.append('candidate_deadline_missed_no_catchup')
    limits=np.asarray(LIMITS)
    within=bool(((targets>=limits[:,0])&(targets<=limits[:,1])).all())
    if not within:reasons.append('raw_chunk_joint_limit_violation')
    # q-derived speed is a required-displacement diagnostic, not predicted motion.
    observed_anchor=list(previous_observations[-1:])+[observation,selected]
    # Reused feedback samples do not create a made-up observed velocity.
    if len(observed_anchor)>2 and observed_anchor[0]['time']>=qtime:observed_anchor=observed_anchor[1:]
    observed=connection(observed_anchor)
    recorded=connection(list(previous_commands[-2:])+[selected])
    proposed=connection(list(previous_proposals[-2:])+[selected])
    for label,check in [('observed_entry',observed),('recorded_command_entry',recorded),('raw_proposal_boundary',proposed)]:
        if check['speed_pass'] is False:reasons.append(label+'_speed_exceeded')
        if check['acceleration_pass'] is False:reasons.append(label+'_acceleration_exceeded')
        if not check['speed_known']:reasons.append(label+'_speed_unobserved')
        if not check['acceleration_known']:reasons.append(label+'_acceleration_unobserved')
    # Inspect the whole untouched chunk and entry into its remaining suffix.
    speed=np.diff(targets[:,:6],axis=0)/np.diff(times)[:,None]
    acc=np.diff(speed,axis=0)/np.diff((times[1:]+times[:-1])/2)[:,None] if len(speed)>1 else np.empty((0,6))
    suffix=[observation]+[dict(time=float(t),q=q.tolist()) for t,q in zip(times[candidate_index:],targets[candidate_index:])]
    suffix_checks=[connection(suffix[max(0,i-2):i+1]) for i in range(1,len(suffix))]
    if speed.size and abs(speed).max()>SPEED+EPS:reasons.append('raw_chunk_internal_speed_exceeded')
    if acc.size and abs(acc).max()>ACCEL+EPS:reasons.append('raw_chunk_internal_acceleration_exceeded')
    if any(c['acceleration_pass'] is False for c in suffix_checks):reasons.append('observed_entry_to_suffix_acceleration_exceeded')
    return dict(candidate=selected,candidate_raw_index=candidate_index,
        timing=dict(historical_trigger_time=trigger_time,inference_latency_s=latency_s,
            virtual_completion_time=completion,raw_index0_target_time=float(times[0]),
            raw_index0_expired=bool(completion>=times[0]),candidate_target_time=selected['time'],
            candidate_deadline_missed=late,deadline_margin_s=selected['time']-completion,
            expired_raw_indices=np.flatnonzero(times<=completion).tolist(),time_shift_applied_s=0.,
            raw_index0_disposition='not scheduled: its original target time equals the inference trigger'),
        first_target_difference_deg=(targets[0,:6]-np.asarray(observation['q'])[:6]).tolist(),
        selected_target_difference_deg=(targets[candidate_index,:6]-np.asarray(observation['q'])[:6]).tolist(),
        observed_state_entry=observed,previous_recorded_command_entry=recorded,
        consecutive_raw_proposal_boundary=proposed,
        raw_chunk_internal=dict(max_speed_deg_s=float(abs(speed).max()) if speed.size else None,
            max_acceleration_deg_s2=float(abs(acc).max()) if acc.size else None),
        within_limits=within,diagnostic_pass=not reasons,rejection_reasons=reasons,
        unknown_initial_acceleration=any(not c['acceleration_known'] for c in (observed,recorded,proposed)),
        accepted_for_execution=False,collision_checked=False,gripper_contact_verified=False,live_eligible=False)


def load_records(dataset,split):
    root=Path(dataset).resolve();manifest=read(root/'manifest.json')
    if sha(manifest['examples_file'])!=manifest['examples_sha256']:raise ValueError('dataset_examples_changed')
    rows=[json.loads(line) for line in Path(manifest['examples_file']).read_text().splitlines()]
    rows=[r for r in rows if split=='all' or r['split']==split]
    if not rows:raise ValueError('empty_selected_split')
    # The causal files recover feedback receipt timestamps dropped by the cached
    # training examples. Bind their raw source logs using the frozen manifest.
    sources={};verified=set()
    def verify_image(path,expected):
        key=(path,expected)
        if key not in verified:
            if sha(path)!=expected:raise ValueError('source_image_hash_changed')
            verified.add(key)
    for stage in manifest['stages']:
        for line in Path(stage['export']['samples_file']).read_text().splitlines():
            source=json.loads(line)
            key=(source['action']['source_log'],source['sample_index'],source['segment_id'])
            sources[key]=source
    raw_logs={}
    for row in rows:
        key=(row['action_source_log'],row['sample_index'],row['segment_id']);source=sources[key]
        validate_sample(source,verify_image)
        if row['q']!=source['observation']['q'] or row['cameras']!=source['observation']['cameras']:
            raise ValueError('cached_observation_differs_from_causal_source')
        if source['command_host_monotonic']!=row['command_host_monotonic']:raise ValueError('source_clock_mismatch')
        log_path=row['action_source_log']
        if log_path not in raw_logs:
            if sha(log_path)!=manifest['source_sha256'].get(log_path):raise ValueError('raw_transport_log_changed')
            raw_logs[log_path]=[json.loads(line) for line in Path(log_path).read_text().splitlines()]
        raw=raw_logs[log_path]
        qline=raw[source['observation']['q_source_line']-1]
        qmessage=json.loads(qline['line'].split('[rx] ',1)[1]);values=qmessage['data']
        if [values[f'angle{i}'] for i in range(7)]!=row['q']:raise ValueError('raw_feedback_values_mismatch')
        cmd=raw[source['action']['source_line']-1]
        if cmd.get('kind')!='command' or cmd['sdk']!=source['action']['sdk_target']:raise ValueError('raw_command_values_mismatch')
        shift=source['command_host_monotonic']-source['command_t']
        qtime=source['observation']['q_available_t']+shift
        if not math.isclose(qline['t']+shift,qtime,abs_tol=1e-7,rel_tol=0):raise ValueError('raw_feedback_time_mismatch')
        if not math.isclose(cmd['t']+shift,row['command_host_monotonic'],abs_tol=1e-7,rel_tol=0):raise ValueError('raw_command_time_mismatch')
        row=dict(row)
        row['_observed']=dict(time=qtime,q=row['q'],kind='historical_teacher_trajectory_feedback',source_log=log_path,
                              source_line=source['observation']['q_source_line'])
        row['_recorded_command']=dict(time=row['command_host_monotonic'],q=cmd['sdk'],kind='historical_actual_sent_target',
                                     source_log=log_path,source_line=source['action']['source_line'])
        yield row,manifest


def replay(dataset,checkpoint,output,split='val'):
    import cv2
    import torch
    from .shadow import Predictor
    torch.set_num_threads(4);cv2.setNumThreads(1)
    entries=list(load_records(dataset,split));rows=[r for r,_ in entries]
    predictor=Predictor(checkpoint);rate=float(predictor.cfg['rate_hz'])
    ck=torch.load(checkpoint,weights_only=True,map_location='cpu')
    manifest_path=Path(dataset).resolve()/'manifest.json'
    if ck['dataset_manifest_sha256']!=sha(manifest_path):raise ValueError('checkpoint_dataset_mismatch')
    if predictor.cfg.get('execute_prefix_steps')!=1:raise ValueError('this_diagnostic_requires_existing_one_step_prefix_config')
    out=Path(output).resolve();out.mkdir(parents=True,exist_ok=False)
    protocol=dict(schema='d1-act-continuous-shadow-protocol-v1',dataset=str(Path(dataset).resolve()),
        dataset_manifest_sha256=sha(manifest_path),checkpoint=str(Path(checkpoint).resolve()),checkpoint_sha256=predictor.sha,
        module_sha256=sha(__file__),split=split,observations=len(rows),rate_hz=rate,
        schedule=SCHEDULE,candidate_raw_index=1,candidate_prefix_steps=1,
        schedule_is_new_offline_assumption=True,online_config_changed=False,
        distinction_from_existing_config='execute_prefix_steps=1 specifies a count, not this index-1 scheduling assumption',
        historical_clock_mapping='inference starts at each historical command timestamp; measured current-process latency is added to that timestamp only for readiness; raw target times are never shifted',
        serial_inference='a row arriving before the previous virtual inference completes is skipped, without a catch-up burst',
        first_step_reference='current recorded actual q at its original feedback receipt time; acceleration requires an earlier distinct feedback timestamp',
        boundary_references=['previous actually sent historical command(s), always before current trigger',
            'previous raw unsent model candidate(s); diagnostic proposal stream only, including earlier rejected proposals'],
        gap_policy='split/episode/segment/source-log change, nonconsecutive sample index, or cadence error >0.05s resets all histories',
        arm_velocity_limit_deg_s=SPEED,arm_acceleration_limit_deg_s2=ACCEL,
        gripper_semantics='J6 is uncalibrated SDK position. Values retained and limits checked, not interpreted as degrees, width, force or contact.',
        limitations=['All later observations came from the historical teacher execution, not from executing ACT.',
            'No plant simulation or physical closed-loop success is inferred.',
            'No collision validation, live camera acquisition, network/SDK timing, contact verification or route promotion.',
            'Finite-difference acceleration uses interval-midpoint times; unknown initial velocity is not replaced with zero.',
            'Full raw predictions and every rejection are retained; no clipping, smoothing, retiming or catch-up.'],
        robot_commands=0,hardware_accessed=False,training_executed=False,live_eligible=False)
    (out/'protocol.json').write_text(json.dumps(protocol,ensure_ascii=False,indent=2)+'\n')
    previous=None;observations=[];commands=[];proposals=[];busy_until=-math.inf;group=-1
    totals=Counter();reasons=Counter();latencies=[];segments=[];worst=Counter();began=time.perf_counter()
    with (out/'predictions.jsonl').open('x') as log:
        for index,row in enumerate(rows):
            reset=break_reason(previous,row,rate)
            if reset:
                observations=[];commands=[];proposals=[];busy_until=-math.inf;group+=1
                segments.append(dict(group=group,first_observation=row['id'],reason=reset))
            trigger=row['command_host_monotonic'];observed=row['_observed']
            record=dict(observation_id=row['id'],sample_index=row['sample_index'],split=row['split'],stage=row['stage'],
                source_segment=row['segment_id'],continuity_group=group,group_reset_reason=reset,
                historical_observation=observed,previous_recorded_commands=commands[-2:],
                previous_raw_unsent_proposals=proposals[-2:],sent_sdk_targets=[],actual_policy_robot_feedback=None,
                observation_origin='historical teacher trajectory, not policy-caused feedback')
            if trigger<busy_until:
                record.update(status='REJECTED',raw_policy_action=None,rejection_reasons=['previous_inference_still_busy'],
                              virtual_previous_completion_time=busy_until)
                reasons['previous_inference_still_busy']+=1;totals['busy_skipped']+=1
            else:
                # Never pass teacher future commands/q to the prediction API.
                inputs={key:row[key] for key in ('id','q','cameras','command_host_monotonic')}
                chunk,ms=predictor.predict(inputs);latency=ms/1000;busy_until=trigger+latency
                checked=review(chunk,observed,observations,commands,proposals,trigger,latency,rate)
                proposals.append(checked['candidate']);proposals=proposals[-2:]
                record.update(status='DIAGNOSTIC_PASS_ONLY' if checked['diagnostic_pass'] else 'REJECTED',
                    raw_policy_action=asdict(chunk),latency_decode_encode_policy_ms=ms,review=checked,
                    rejection_reasons=checked['rejection_reasons'])
                latencies.append(ms);totals['predictions']+=1
                totals['diagnostic_pass']+=checked['diagnostic_pass'];totals['raw_index0_expired']+=checked['timing']['raw_index0_expired']
                totals['index1_deadline_missed']+=checked['timing']['candidate_deadline_missed']
                totals['unknown_initial_acceleration']+=checked['unknown_initial_acceleration']
                reasons.update(checked['rejection_reasons'])
                for name,field in [('observed_entry','observed_state_entry'),('recorded_entry','previous_recorded_command_entry'),('block_boundary','consecutive_raw_proposal_boundary')]:
                    check=checked[field]
                    for key in ('last_speed_deg_s','last_acceleration_max_deg_s2'):
                        if check.get(key) is not None:worst[name+'_'+key]=max(worst[name+'_'+key],check[key])
            log.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n');log.flush()
            # Only past history is exposed to the next row. q is never advanced
            # from predictions, nor presented as the outcome of accepted actions.
            observations.append(observed);observations=observations[-2:]
            commands.append(row['_recorded_command']);commands=commands[-2:];previous=row
            if (index+1)%100==0:print(json.dumps(dict(processed=index+1,total=len(rows),diagnostic_pass=totals['diagnostic_pass'])),flush=True)
    summary=dict(status='OFFLINE_CONTINUOUS_INTERFACE_REVIEW_COMPLETE',counts=dict(totals),
        observations=len(rows),rejected_observations=len(rows)-totals['diagnostic_pass'],continuity_groups=segments,
        rejection_counts=dict(reasons),worst_connection_diagnostics=dict(worst),
        latency_ms=dict(median=float(np.median(latencies)),p95=float(np.quantile(latencies,.95)),maximum=max(latencies)),
        elapsed_s=time.perf_counter()-began,protocol_sha256=sha(out/'protocol.json'),predictions_sha256=sha(out/'predictions.jsonl'),
        schedule=SCHEDULE,model_sha256=predictor.sha,hardware_accessed=False,robot_commands=0,
        physical_closed_loop=False,teacher_observations_are_policy_feedback=False,live_eligible=False,
        scope='Continuous historical-observation interface diagnostics, not ACT physical trajectory execution or skill success')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',required=True);p.add_argument('--checkpoint',required=True);p.add_argument('--output',required=True)
    p.add_argument('--split',choices=('train','val','test','all'),default='val')
    a=p.parse_args();print(json.dumps(replay(a.dataset,a.checkpoint,a.output,a.split),ensure_ascii=False))


if __name__=='__main__':main()

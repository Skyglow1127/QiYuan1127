#!/usr/bin/env python3
"""Offline cycle/stage logs -> causal PREACTION pairs and a training handoff.

No camera, DDS, robot, or service API is imported or called. Historical replay
does not start training. Future images and feedback never become input features.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import shlex
import subprocess
import sys

EXPORTER = Path('/home/aup/D1/Control/tools/export_grasp_dataset.py')
TIMING = Path('/home/aup/D1/Control/ros2_ws/src/d1_pick_place/d1_pick_place/path_timing.py')
BASELINE = Path('/home/aup/Camera/output/preaction_20261023/data_work/pairs.json')
SPLITS = ('train', 'val', 'test')
TARGET_CACHE = {}
PARTIAL_ACTION_SEMANTICS = 'single_joint_command_no_updates_to_other_axes'


def plan_command_mask(plan):
    """Explicit single-joint commands are distinct from collision-only geometry."""
    mask=plan.get('commanded_joint_mask')
    if mask is None and plan.get('action_semantics') != PARTIAL_ACTION_SEMANTICS:
        return None  # Existing full-arm plan and gripper-hold contract.
    if mask==[True]*7 and all(type(v) is bool for v in mask) and plan.get('action_semantics')!=PARTIAL_ACTION_SEMANTICS:
        return None
    if (not isinstance(mask,list) or len(mask)!=7 or any(type(v) is not bool for v in mask)
            or sum(mask)!=1 or not (mask[0] or mask[4])
            or plan.get('action_semantics')!=PARTIAL_ACTION_SEMANTICS):
        raise ValueError('invalid_explicit_single_joint_command_contract')
    if plan.get('requested_gripper_target_sdk') is not None:
        raise ValueError('single_joint_command_must_not_request_gripper_target')
    return mask


def command_matches_plan(action,reference,mask):
    actual=action['sdk_target']
    if len(actual)!=7:return False
    if mask is None:
        return not any(v is None for v in actual[:6]) and max(abs(a-b) for a,b in zip(actual[:6],reference))<=1e-5
    # Verify both the raw command mask retained by the causal exporter and the
    # finite values. A non-active observation must never stand in for a command.
    if action.get('commanded_joint_mask')!=mask or [v is not None for v in actual]!=mask:
        return False
    return all(not active or (isinstance(actual[j],(int,float)) and math.isfinite(actual[j])
               and abs(actual[j]-reference[j])<=1e-5) for j,active in enumerate(mask))


def command_reference(reference,grip,mask):
    full=list(reference)+[grip]
    return full if mask is None else [v if mask[j] else None for j,v in enumerate(full)]


def action_vectors(current_reference,future_reference,q,grip,mask):
    current=command_reference(current_reference,grip,mask)
    target=command_reference(future_reference,grip,mask)
    # Zero on an inactive axis encodes no new command, never a hold target or
    # a claim that the physical joint remained motionless.
    planned=[b-a if b is not None else 0. for a,b in zip(current,target)]
    delta=[b-a if b is not None else 0. for a,b in zip(q,target)]
    return current,target,planned,delta


def interaction_context(value, source=None):
    """Preserve declared interaction state without inferring contact from motion.

    A false contact_verified flag means contact has not been verified; it does
    not assert that no contact occurred. Unknown historical state stays null.
    """
    value = {} if value is None else dict(value)
    for key in ('object_held', 'contact_verified'):
        if value.get(key) is not None and type(value[key]) is not bool:
            raise ValueError('interaction_context_'+key+'_must_be_boolean_or_null')
        value.setdefault(key, None)
    value.setdefault('state_source', 'unknown')
    if source is not None:
        value['declaration_source'] = source
    value['semantics'] = 'Declared collection context, not measured force/contact or a grasp-success label.'
    return value


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def jsonlines(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def usable_red_target(path):
    """Match the existing trainer's minimum visible red-component contract.

    This is only dataset eligibility, not an independent object annotation.
    """
    if path not in TARGET_CACHE:
        import cv2
        import numpy as np
        image=cv2.imread(str(path))
        if image is None:
            TARGET_CACHE[path]=False
        else:
            hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
            mask=(((hsv[:,:,0]<=10)|(hsv[:,:,0]>=170))&(hsv[:,:,1]>=85)&(hsv[:,:,2]>=45)).astype(np.uint8)
            mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
            count,_,stats,_=cv2.connectedComponentsWithStats(mask)
            TARGET_CACHE[path]=bool(count>1 and max(stats[1:,cv2.CC_STAT_AREA])>=50)
    return TARGET_CACHE[path]


def planned_references(plan):
    profile = plan.get('timing_profile', 'edge_quintic_v1')
    if profile == 'collinear_bounded_v1':
        # This module contains only numerical geometry; importing the executor
        # would unnecessarily introduce transport code into this offline tool.
        return [(float(t), list(map(float, q))) for t,q in load_module('preaction_path_timing', TIMING).references(plan)]
    if profile != 'edge_quintic_v1':
        raise ValueError('unsupported_timing_profile:'+str(profile))
    result=[(0., plan['sdk'][0])]; step=0
    for a,b,count in zip(plan['sdk'][:-1],plan['sdk'][1:],plan['ticks']):
        if count != int(count): raise ValueError('noninteger_legacy_tick')
        for i in range(1,int(count)+1):
            u=i/count;step+=1;alpha=10*u**3-15*u**4+6*u**5
            result.append((step/plan['rate'],[x+(y-x)*alpha for x,y in zip(a,b)]))
    return result


def discover(args):
    def logged(p): return (p/'transport.jsonl').exists() or (p/'motion/events.jsonl').exists()
    specs=[]
    if args.cycle:
        run=args.cycle.resolve()
        for p in sorted(run.iterdir()) if run.is_dir() else []:
            if not p.is_dir():continue
            if logged(p/'executed'):
                semantic=re.sub(r'^\d+_', '', p.name)
                specs.append(dict(path=str(p/'executed'),run=str(run),stage=semantic,segment_id=p.name,fsm=True))
            elif logged(p):
                specs.append(dict(path=str(p),run=str(run),stage=p.name,segment_id=p.name,fsm=False))
    else:
        document=read(args.stages_json)
        defaults=document.get('interaction_context') if isinstance(document,dict) else None
        declaration=dict(path=str(args.stages_json.resolve()),sha256=sha(args.stages_json))
        for i,item in enumerate(document.get('stages',[]) if isinstance(document,dict) else document):
            spec=dict(item);folder=Path(spec['path']).resolve()
            spec['interaction_context']=interaction_context(spec.get('interaction_context',defaults),declaration)
            fsm=folder.name=='executed' and bool(re.match(r'^\d+_',folder.parent.name))
            spec.update(path=str(folder),run=str(Path(spec.get('run',folder.parents[1] if fsm else folder.parent)).resolve()),
                        stage=spec.get('stage',re.sub(r'^\d+_','',folder.parent.name) if fsm else folder.name),
                        segment_id=spec.get('segment_id',f'{i:02d}_'+(folder.parent.name if fsm else folder.name)),fsm=fsm)
            specs.append(spec)
    if not specs: raise ValueError('no_recorded_stages: expected stage/transport.jsonl or numbered_stage/executed logs')
    if len({x['segment_id'] for x in specs}) != len(specs): raise ValueError('duplicate_segment_id')
    for spec in specs:
        if not re.fullmatch(r'[A-Za-z0-9_.-]+',spec['segment_id']):raise ValueError('unsafe_segment_id')
        if spec.get('split') not in (None,*SPLITS):raise ValueError('unknown_split')
        spec['interaction_context']=interaction_context(spec.get('interaction_context'))
    return specs


def prepare_stage(spec, exporter, out, args, hashes):
    folder,run=Path(spec['path']),Path(spec['run'])
    project_path=run/'project_status.json';project=read(project_path) if project_path.exists() else {}
    context=exporter.fsm_context(run,hashes) if spec['fsm'] else None
    segment=exporter.export_stage(folder,out/'causal',run,project,project_path,hashes,
        feedback_max_age=args.feedback_max_age,camera_max_age=args.camera_max_age,
        semantic_stage=spec['stage'],segment_name=spec['segment_id'],fsm=context)
    rows=jsonlines(segment['samples_file'])
    report=dict(**spec,causal_export=segment,candidates=0,selected=0,rejections={})
    result_path=next((p for p in [folder/'result.json',folder/'summary.json'] if p.exists()),None)
    result=read(result_path) if result_path else {}
    if result.get('hardware_commands_sent') is not True or not result.get('commands',0):
        raise ValueError('no_recorded_physical_commands: result must explicitly record hardware_commands_sent=true and commands>0')
    if not segment['camera_clock_aligned']:
        raise ValueError('camera_clock_unaligned: explicit session_origin_monotonic missing; no mtime/first-frame inference')
    plan_path=folder/'plan.json'
    if not plan_path.exists():
        raise ValueError('missing_preexecution_plan_json: gripper summary alone is not treated as a saved preexecution plan')
    plan=read(plan_path)
    if 'sdk_plan' not in plan:raise ValueError('missing_sdk_plan')
    command_mask=plan_command_mask(plan)
    refs=planned_references(plan['sdk_plan']);rate=float(plan['sdk_plan']['rate'])
    hop=round(args.horizon*rate)
    if hop<1 or abs(hop/rate-args.horizon)>1e-6:raise ValueError('horizon_not_on_plan_command_grid')
    frame_path=folder/'frames/frames.jsonl'
    if not frame_path.exists():raise ValueError('missing_frames_jsonl')
    frames=[];last_index=-1
    for line,frame in enumerate(jsonlines(frame_path),1):
        if frame.get('role')!='wrist':continue
        if frame.get('serial')!='CC1365301EY':continue
        stamp,age,index=frame.get('host_monotonic'),frame.get('age_ms'),frame.get('index')
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (stamp,age,index)):continue
        if age<0 or index<=last_index:continue
        last_index=index;p=(frame_path.parent/frame['file']).resolve()
        if not p.is_file() or not p.stat().st_size:continue
        frames.append(dict(path=str(p),sha256=sha(p),available_t=stamp,
            capture_time_estimate=stamp-age/1000,board_age_ms=age,frame_index=index,
            camera_serial=frame['serial'],source_log=str(frame_path),source_line=line))
    if not frames:raise ValueError('no_usable_wrist_frames')
    # Actual future command logs are compared for diagnostic continuity only.
    # Values passed to the model below are reconstructed from the prior plan.
    prefix=0
    for row,(_,ref) in zip(rows,refs):
        if not command_matches_plan(row['action'],ref,command_mask):break
        prefix+=1
    if prefix<=hop:raise ValueError('insufficient_verified_plan_prefix_or_interrupted_before_horizon')
    first_grip=rows[0]['action']['sdk_target'][6]
    grip=None if command_mask is not None else plan.get('requested_gripper_target_sdk',first_grip)
    if command_mask is None and grip is None:raise ValueError('missing_known_gripper_hold_target')
    reasons=Counter();candidates=[]
    required=['feedback_present','feedback_values_valid','feedback_fresh','command_valid',
              'camera_clock_aligned','power_status_valid','enable_status_valid','error_status_valid','wrist_image_valid']
    for i in range(prefix-hop):
        row=rows[i];now=row['command_host_monotonic'];invalid=[k for k in required if not row['masks'].get(k)]
        if invalid:
            reasons.update(invalid);continue
        image=row['observation']['cameras']['wrist']
        if not image['available_t']<now:reasons['current_frame_arrived_after_prediction']+=1;continue
        times=[r['command_host_monotonic'] for r in rows[i:i+hop+1]]
        if any(abs((times[k]-now)-k/rate)>.10 for k in range(len(times))):
            reasons['command_schedule_discontinuity']+=1;continue
        future=min(frames,key=lambda r:abs(r['capture_time_estimate']-(now+args.horizon)))
        if abs(future['capture_time_estimate']-now-args.horizon)>args.future_tolerance:
            reasons['future_capture_outside_horizon_tolerance']+=1;continue
        if future['capture_time_estimate']<=now or future['sha256']==image['sha256']:
            reasons['future_image_not_new']+=1;continue
        if not usable_red_target(image['path']):reasons['current_frame_no_usable_red_target']+=1;continue
        if not usable_red_target(future['path']):reasons['future_frame_no_usable_red_target']+=1;continue
        q=row['observation']['q']
        current_plan,target,planned_delta,delta=action_vectors(refs[i][1],refs[i+hop][1],q,grip,command_mask)
        if max(abs(x) for x in planned_delta[:6])<args.min_plan_motion:
            reasons['arm_motion_below_requested_minimum_deg']+=1;continue
        image=dict(path=image['path'],sha256=image['sha256'],available_t=image['available_t'],
            capture_time_estimate=now-image['capture_age_estimate_s'],frame_index=image['frame_index'],
            camera_serial=image['serial'],board_age_ms=image['board_age_at_fetch_ms'],
            source_log=image['source_log'],source_line=image['source_line'])
        candidates.append(dict(id=f"{spec['segment_id']}_{i:05d}",split=spec.get('split'),stage=spec['stage'],
            segment_id=spec['segment_id'],episode=str(run),clock='host_monotonic',prediction_t=now,
            interaction_context=spec['interaction_context'],
            horizon_s=args.horizon,label_actual_horizon_s=future['capture_time_estimate']-now,
            current_image=image['path'],future_image=future['path'],image_t=image,image_t1=future,
            q_current=q,q_command_future=target,delta_q_command=delta,
            state=dict(q=q,available_t=row['observation']['q_available_t'],age_s=row['observation']['q_age_s'],
                source_log=row['action']['source_log'],source_line=row['observation']['q_source_line']),
            action=dict(units=['degree']*6+['uncalibrated_sdk_position'],current_sdk=current_plan,target_sdk=target,
                commanded_joint_mask=command_mask if command_mask is not None else [True]*7,
                action_semantics=PARTIAL_ACTION_SEMANTICS if command_mask is not None else 'full_arm_plan_with_gripper_hold',
                inactive_delta_semantics='Zero means no new command, not a hold target or observed zero motion.' if command_mask is not None else None,
                delta_sdk=delta,planned_delta_sdk=planned_delta,plan_source=str(plan_path),plan_sha256=sha(plan_path),
                command_index=i,future_plan_index=i+hop,plan_elapsed_s=refs[i][0],
                future_plan_references=[dict(offset_s=t-refs[i][0],sdk=command_reference(v,grip,command_mask)) for t,v in refs[i:i+hop+1]],
                availability='Saved preexecution sdk_plan, reconstructed with its recorded timing profile.',
                future_feedback_used=False,future_logged_commands_used_as_features=False),
            label=dict(kind='real_saved_future_rgb',success_not_required=True,
                interaction_context=spec['interaction_context'],
                demonstration_success=row['labels']['demonstration_success'],physical_evidence=row['labels'],
                warning='Prediction pairs may include unsuccessful physical attempts; never relabelled as successful demonstrations.'),
            evidence=dict(causal_export=segment['samples_file'],causal_sample_index=i,
                physical_command_result=str(result_path),result_sha256=sha(result_path),
                verified_plan_prefix_commands=prefix,available_plan_total_commands=len(refs))))
    report.update(candidates=len(candidates),rejections=dict(reasons),verified_plan_prefix_commands=prefix,
        commanded_joint_mask=command_mask if command_mask is not None else [True]*7,
        action_semantics=PARTIAL_ACTION_SEMANTICS if command_mask is not None else 'full_arm_plan_with_gripper_hold')
    return candidates,report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--cycle',type=Path);source.add_argument('--stages-json',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--pairs-per-stage',type=int,default=5)
    p.add_argument('--horizon',type=float,default=2.);p.add_argument('--min-plan-motion',type=float,default=.25)
    p.add_argument('--feedback-max-age',type=float,default=.25);p.add_argument('--camera-max-age',type=float,default=.25)
    p.add_argument('--future-tolerance',type=float,default=.15)
    p.add_argument('--baseline-pairs',type=Path,default=BASELINE)
    p.add_argument('--training-output',type=Path)
    p.add_argument('--execute-training',action='store_true',help='Requires a cycle captured after --capture-after-unix; never implicit on replay.')
    p.add_argument('--capture-after-unix',type=float,help='Explicit collection-start wall time; checks original capture log mtime, not exposure synchronization.')
    p.add_argument('--npu',action='store_true',help='Include NPU evaluation in the handoff; training remains CPU.')
    a=p.parse_args(argv)
    if a.horizon<=0 or a.pairs_per_stage<1 or a.min_plan_motion<0:p.error('positive horizon/count and nonnegative motion threshold required')
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False);(out/'causal').mkdir()
    exporter=load_module('preaction_causal_export',EXPORTER);specs=discover(a);hashes={}
    candidates={};reports=[]
    for spec in specs:
        try:
            rows,report=prepare_stage(spec,exporter,out,a,hashes);candidates[spec['segment_id']]=rows;reports.append(report)
        except (OSError,ValueError,KeyError,TypeError,IndexError) as exc:
            reports.append(dict(**spec,selected=0,candidates=0,unusable_reason=str(exc)))
    usable=[s for s in specs if candidates.get(s['segment_id'])]
    usable.sort(key=lambda s:candidates[s['segment_id']][0]['prediction_t'])
    any_explicit=any(s.get('split') for s in usable)
    if any_explicit and not all(s.get('split') for s in usable):raise ValueError('supply splits for every usable stage or none')
    if not any_explicit:
        for i,s in enumerate(usable):s['split']='train' if i<len(usable)-2 else 'val' if i==len(usable)-2 else 'test'
    pairs=[];used=set()
    for spec in usable:
        rows=candidates[spec['segment_id']]
        chosen=[]
        for f in [(i+.5)/a.pairs_per_stage for i in range(a.pairs_per_stage)]:
            wanted=round(f*(len(rows)-1))
            for index in sorted(range(len(rows)),key=lambda k:abs(k-wanted)):
                item=rows[index];digests={item['image_t']['sha256'],item['image_t1']['sha256']}
                if not digests&used:
                    item['split']=spec['split'];chosen.append(item);used.update(digests);break
        pairs.extend(chosen)
        report=next(r for r in reports if r['segment_id']==spec['segment_id'])
        report.update(split=spec['split'],selected=len(chosen))
    baseline=read(a.baseline_pairs) if a.baseline_pairs.exists() else {'pairs':[]}
    baseline_hashes={x[k]['sha256'] for x in baseline['pairs'] for k in ('image_t','image_t1')}
    new_pairs=[x for x in pairs if x['image_t']['sha256'] not in baseline_hashes and x['image_t1']['sha256'] not in baseline_hashes]
    counts=Counter(x['split'] for x in pairs)
    stats={s:dict(pairs=counts[s],max_planned_delta_deg=max([max(abs(v) for v in x['action']['planned_delta_sdk'][:6]) for x in pairs if x['split']==s]or[0]),
        pairs_with_at_least_1deg_plan_motion=sum(max(abs(v) for v in x['action']['planned_delta_sdk'][:6])>=1 for x in pairs if x['split']==s),
        per_joint_min_planned_delta=[min([x['action']['planned_delta_sdk'][j] for x in pairs if x['split']==s]or[0]) for j in range(7)],
        per_joint_max_planned_delta=[max([x['action']['planned_delta_sdk'][j] for x in pairs if x['split']==s]or[0]) for j in range(7)]) for s in SPLITS}
    ready=all(counts[s]>0 for s in SPLITS)
    manifest=dict(schema='causal-preaction-image-pairs-v1',pairs=pairs,
        summary=dict(pair_count=len(pairs),split_counts=dict(counts),unique_images=len(used),
            unique_episodes=len({x['episode'] for x in pairs}),synthetic_pair_count=0,horizon_s=a.horizon,
            split_policy='Whole motion segment, chronological train/val/test unless explicitly mapped; no shared image hashes.',
            hash_overlap_between_splits=0,source_capture_is_historical_until_verified_fresh=True,
            new_pairs_relative_to_baseline=len(new_pairs),baseline_pairs=str(a.baseline_pairs.resolve()),
            hardware_commands_sent=0,training_performed=False,ready_for_offline_training=ready),
        movement_statistics=stats,stage_reports=reports,source_sha256=hashes,
        causal_contract=dict(input=['current image available before command','earlier valid feedback','preexecution future plan','requested horizon'],
            future_image='Target only; cannot select current ROI or become model input.',
            physical_labels='Copied from original causal exporter. Failed/unknown physical outcomes are not forged as success.',
            source_alignment='Host receipt clock; board age gives approximate capture time, not synchronized exposure.'))
    save(out/'pairs.json',manifest)
    training_out=(a.training_output or out/'experiment').resolve()
    command=[str(Path('/home/aup/Camera/.venv-models/bin/python')),str(Path(__file__).with_name('run_experiment.py')),
             '--pairs',str(out/'pairs.json'),'--output',str(training_out)]+(['--npu']if a.npu else [])
    fresh_sources=bool(a.capture_after_unix is not None and a.cycle and usable and all(
        (Path(s['path'])/'frames/frames.jsonl').stat().st_mtime>=a.capture_after_unix for s in usable))
    fresh_train=any(x['split']=='train' for x in new_pairs)
    gates=dict(all_splits_nonempty=ready,new_training_images=fresh_train,
               explicitly_bound_new_cycle_capture=fresh_sources,source_is_cycle=a.cycle is not None)
    handoff=dict(command=command,shell_command=shlex.join(command),training_requested=a.execute_training,
        automatic_promotion=False,robot_commands=0,new_capture_gate=gates,
        missing=[k for k,v in gates.items() if not v],
        note='Freshness mtime is only a collection guard; it never establishes image/action clock alignment or physical grasp success.')
    save(out/'handoff.json',handoff)
    if a.execute_training:
        if not all(gates.values()):
            print(json.dumps(dict(status='TRAINING_NOT_STARTED',output=str(out),missing=handoff['missing']),ensure_ascii=False));return 2
        subprocess.run(command,check=True)
        # The source manifest is immutable after training records its SHA-256.
        handoff.update(training_completed=True);save(out/'handoff.json',handoff)
    print(json.dumps(dict(status='PAIR_EXPORT_READY'if ready else'INSUFFICIENT_CAPTURE',output=str(out),
        pairs=len(pairs),splits=dict(counts),unusable=[dict(stage=r['stage'],reason=r['unusable_reason'])for r in reports if r.get('unusable_reason')],
        training_performed=bool(handoff.get('training_completed'))),ensure_ascii=False,indent=2))
    return 0 if ready else 2


if __name__=='__main__':
    raise SystemExit(main())

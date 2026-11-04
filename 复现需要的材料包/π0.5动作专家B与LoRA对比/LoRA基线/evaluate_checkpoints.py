#!/usr/bin/env python3
"""Offline held-out PI05 checkpoint comparison. Never imports robot control.

Fixed episodes 46..51, frames 0/20, seeds 0/1, 10 Euler steps. A single strict
base load and one default LoRA adapter are reused. All raw predictions persist.
Run independently of the production server so no live model/config is changed.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import time

EPISODES = tuple(range(46, 52))
FRAMES = (0, 20)
SEEDS = (0, 1)
STEPS = tuple(range(2000, 20001, 2000))
DEFAULT_BOUNDS = [[-60,30],[0,120],[-120,0],[-45,45],[-60,65],[-80,60],[0,.1]]
DEFAULT_MAX_STEP = [6.0]*6+[.008]
PROCESSOR_FILES = (
    'policy_preprocessor.json', 'policy_postprocessor.json',
    'policy_preprocessor_step_3_normalizer_processor.safetensors',
    'policy_postprocessor_step_0_unnormalizer_processor.safetensors')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    tmp = Path(path).with_suffix(Path(path).suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    tmp.replace(path)


def summarize_rows(rows, baseline_step=20000):
    """Pure-Python local summary; consumes rows.jsonl or the JSON result's rows.

    Thresholds are fixed before evaluating: >=10% and >=0.5 degree lower action0
    joint MAE, with neither full-50 joint MAE nor gripper errors increasing.
    This yields offline candidates, never execution approval.
    """
    import statistics
    summary = []
    for step in sorted({row['checkpoint_step'] for row in rows}):
        group = [r for r in rows if r['checkpoint_step'] == step]
        expected_cases = {(ep,fr,seed) for ep in EPISODES for fr in FRAMES for seed in SEEDS}
        actual_cases = {(r['episode_index'],r['frame_index'],r['seed']) for r in group}
        if len(group) != 24 or actual_cases != expected_cases:
            raise ValueError(f'Incomplete or duplicate case group for checkpoint {step}')
        mean = lambda key: statistics.fmean(row['metrics'][key] for row in group)
        summary.append({
            'checkpoint_step':step, 'cases':len(group),
            'first_joint_mae_deg':mean('first_joint_mae_deg'),
            'first_gripper_mae_mm':mean('first_gripper_mae_mm'),
            'full50_joint_mae_deg':mean('full50_joint_mae_deg'),
            'full50_gripper_mae_mm':mean('full50_gripper_mae_mm'),
            'first_joint_step_violation_rate':mean('first_joint_step_violation'),
            'first_gripper_step_violation_rate':mean('first_gripper_step_violation'),
            'first_bounds_violation_rate':mean('first_bounds_violation'),
            'first_any_numeric_violation_rate':mean('first_any_numeric_violation'),
            'model_post_median_s':statistics.median(r['timing_s']['model_post'] for r in group),
            'per_joint_first_mae_deg':[statistics.fmean(r['metrics']['per_joint_first_abs_error_deg'][i] for r in group) for i in range(6)],
            'per_joint_full50_mae_deg':[statistics.fmean(r['metrics']['per_joint_full50_mae_deg'][i] for r in group) for i in range(6)],
        })
    baseline = next(row for row in summary if row['checkpoint_step'] == baseline_step)
    for item in summary:
        reduction = baseline['first_joint_mae_deg']-item['first_joint_mae_deg']
        item['first_joint_mae_reduction_vs_020000_deg'] = reduction
        item['offline_candidate_criteria'] = {
            'first_joint_mae_at_least_10pct_lower': item['first_joint_mae_deg'] <= baseline['first_joint_mae_deg']*.90,
            'first_joint_mae_at_least_0p5deg_lower': reduction >= .5,
            'full50_joint_mae_not_worse': item['full50_joint_mae_deg'] <= baseline['full50_joint_mae_deg'],
            'first_gripper_mae_not_worse': item['first_gripper_mae_mm'] <= baseline['first_gripper_mae_mm'],
            'full50_gripper_mae_not_worse': item['full50_gripper_mae_mm'] <= baseline['full50_gripper_mae_mm'],
        }
        item['offline_candidate'] = item['checkpoint_step'] != baseline_step and all(item['offline_candidate_criteria'].values())
    candidates = sorted([row for row in summary if row['offline_candidate']], key=lambda r:r['first_joint_mae_deg'])
    return {'baseline_step':baseline_step,'checkpoint_summaries':summary,
            'candidate_step':None if not candidates else candidates[0]['checkpoint_step'],
            'candidate_scope':'Held-out error comparison only; requires later live read-only preview. No execution approval. Numeric guard pass rate is diagnostic and not a selection criterion.'}


def load_exported_cases(folder):
    import numpy as np
    cases = []
    for path in sorted(Path(folder).glob('*.npz')):
        with np.load(path, allow_pickle=False) as data:
            required = {'episode_index','frame_index','task','observation.state',
                        'observation.images.global','observation.images.wrist','target_actions'}
            if not required <= set(data.files):
                raise ValueError(f'Missing held-out NPZ fields: {path}')
            ep, fr = int(data['episode_index']), int(data['frame_index'])
            if ep not in EPISODES or fr not in FRAMES:
                raise ValueError(f'Unexpected held-out case {(ep,fr)} in {path}')
            case = {'episode_index':ep,'frame_index':fr,'task':str(data['task'].item()),
                    'observation.state':data['observation.state'].copy(),
                    'target_actions':data['target_actions'].copy(),
                    'action_is_pad':data['action_is_pad'].copy() if 'action_is_pad' in data else (~data['target_mask'].astype(bool) if 'target_mask' in data else np.zeros(50,dtype=bool)),
                    'input_file':str(path),'input_sha256':digest(path)}
            for key in ('observation.images.global','observation.images.wrist'):
                case[key] = data[key].copy()
            cases.append(case)
    expected = {(ep,fr) for ep in EPISODES for fr in FRAMES}
    if len(cases) != 12 or {(c['episode_index'],c['frame_index']) for c in cases} != expected:
        raise ValueError('Require exactly episodes46..51 x frames0/20 =12 unique held-out NPZ files')
    return sorted(cases,key=lambda c:(c['episode_index'],c['frame_index']))


def export_dataset_cases(dataset_root, output):
    import numpy as np
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset('local/piper_task', root=str(dataset_root), episodes=list(EPISODES),
                        video_backend='torchcodec',delta_timestamps={'action':[i/20 for i in range(50)]})
    if ds.fps != 20:
        raise ValueError('Held-out evaluation expects20Hz dataset')
    columns = ds.hf_dataset.select_columns(['episode_index','frame_index'])[:]
    wanted = {(ep,fr) for ep in EPISODES for fr in FRAMES}
    indices = {}
    for index,(ep,fr) in enumerate(zip(columns['episode_index'],columns['frame_index'])):
        key = int(ep), int(fr)
        if key in wanted:
            if key in indices: raise ValueError(f'Duplicate frame {key}')
            indices[key] = index
    if set(indices) != wanted:
        raise ValueError(f'Missing fixed held-out frame identities: {wanted-set(indices)}')
    output.mkdir(parents=True,exist_ok=False)
    for ep,fr in sorted(wanted):
        sample = ds[indices[(ep,fr)]]
        def array(key):
            value=sample[key]
            return value.detach().cpu().numpy() if hasattr(value,'detach') else np.asarray(value)
        fields = {key:array(key) for key in ('observation.state','observation.images.global','observation.images.wrist')}
        fields.update(episode_index=np.asarray(ep),frame_index=np.asarray(fr),task=np.asarray(sample['task']),
                      target_actions=array('action'),action_is_pad=array('action_is_pad') if 'action_is_pad' in sample else np.zeros(50,dtype=bool))
        np.savez_compressed(output/f'episode_{ep:03d}_frame_{fr:03d}.npz',**fields)
    return load_exported_cases(output)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--deploy-dir',type=Path,default=Path('/root/autodl-tmp/piper_pi05/cloud_grasp_v5'))
    parser.add_argument('--checkpoint-root',type=Path,default=Path('/root/autodl-tmp/piper_pi05/runs/pi05_train_v5_r06/checkpoints'))
    parser.add_argument('--dataset-root',type=Path,default=Path('/root/autodl-tmp/lerobot_act/datasets/rad_place_merged_v5'))
    parser.add_argument('--input-dir',type=Path,help='Exactly12 previously exported held-out NPZ files; skips dataset imports/video decoding')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--image-mode',choices=('compact_uint8','original_full'),default='compact_uint8')
    parser.add_argument('--summarize-only',type=Path,help='Summarize saved rows.jsonl locally without Torch or CUDA imports')
    args=parser.parse_args()
    if args.summarize_only:
        rows=[json.loads(line) for line in args.summarize_only.read_text().splitlines() if line.strip()]
        result=summarize_rows(rows); write_json(args.output,result); print(json.dumps(result,ensure_ascii=False)); return
    for key in ('HF_HUB_OFFLINE','TRANSFORMERS_OFFLINE','HF_DATASETS_OFFLINE','HF_HUB_DISABLE_TELEMETRY','PYTHONNOUSERSITE'):
        os.environ[key]='1'
    os.environ['TOKENIZERS_PARALLELISM']='false'
    if args.output.exists(): raise ValueError('Use a new output path; previous evidence is never overwritten')
    args.output.mkdir(parents=True)
    import numpy as np
    import torch
    from peft import PeftConfig,PeftModel,get_peft_model_state_dict,set_peft_model_state_dict
    from safetensors.torch import load_file
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy  # registers config
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.common.vla_utils import resize_with_pad_torch
    sys.path.insert(0,str(args.deploy_dir))
    from pi05_low_memory import load_base_streaming
    if not torch.cuda.is_available(): raise RuntimeError('CUDA required for offline model evaluation')
    torch.set_num_threads(8)
    settings=json.loads((args.deploy_dir/'cloud_config.json').read_text())
    base,tokenizer=Path(settings['base']),Path(settings['tokenizer'])
    adapter=args.checkpoint_root/'020000/pretrained_model'
    expected={name:digest(adapter/name) for name in PROCESSOR_FILES}
    expected['config.json']=digest(adapter/'config.json')
    expected['adapter_config.json']=digest(adapter/'adapter_config.json')
    identities={}
    # Check all processor/stat/config identities before loading or choosing a model.
    for step in STEPS:
        folder=args.checkpoint_root/f'{step:06d}/pretrained_model'
        hashes={name:digest(folder/name) for name in expected}
        if hashes != expected: raise RuntimeError(f'Checkpoint{step} processors/config differ; refusing mixed normalization/architecture')
        identities[step]={'adapter_sha256':digest(folder/'adapter_model.safetensors'),'pipeline_sha256':hashes}
    if identities[20000]['adapter_sha256'] != settings['expected_adapter_sha256']:
        raise RuntimeError('Final adapter differs from configured known final model')
    cases=load_exported_cases(args.input_dir) if args.input_dir else export_dataset_cases(args.dataset_root,args.output/'heldout_inputs')
    cfg=PreTrainedConfig.from_pretrained(str(adapter),local_files_only=True,cli_overrides=['--device=cuda','--dtype=bfloat16','--gradient_checkpointing=false','--compile_model=false'])
    if cfg.chunk_size!=50 or cfg.num_inference_steps!=10 or cfg.use_relative_actions:
        raise RuntimeError('Require saved50chunk,10step absolute-action model')
    cfg.pretrained_path=str(base)
    pre,post=make_pre_post_processors(policy_cfg=cfg,pretrained_path=str(adapter),
        preprocessor_overrides={'tokenizer_processor':{'tokenizer_name':str(tokenizer)},'device_processor':{'device':'cuda'}},
        postprocessor_overrides={'device_processor':{'device':'cpu'}})
    batch_cases=[]
    for c in cases:
        state=np.asarray(c['observation.state'],dtype=np.float32)
        target=np.asarray(c['target_actions'],dtype=np.float32)
        is_pad=np.asarray(c['action_is_pad'],dtype=bool)
        if state.shape!=(7,) or target.shape!=(50,7) or is_pad.shape!=(50,) or is_pad[0] or not np.isfinite(state).all() or not np.isfinite(target).all():
            raise ValueError(f'Invalid case arrays: {(c["episode_index"],c["frame_index"])}')
        if c['task']!=settings['task']: raise ValueError('Heldout task differs from trained configured task')
        sample={'task':c['task'],'observation.state':torch.from_numpy(state.copy())}
        for key in ('observation.images.global','observation.images.wrist'):
            image=torch.from_numpy(np.ascontiguousarray(c[key]))
            if image.ndim!=3: raise ValueError('RGB must have3 dimensions')
            if image.shape[-1]==3: image=image.permute(2,0,1)
            if image.shape[0]!=3: raise ValueError('Expected RGB CHW/HWC')
            image=image.to(torch.float32)/(255 if image.dtype==torch.uint8 else 1)
            if not torch.isfinite(image).all() or image.min()<0 or image.max()>1: raise ValueError('Unnormalized RGB required')
            if args.image_mode=='compact_uint8':
                image=resize_with_pad_torch(image.unsqueeze(0).permute(0,2,3,1),224,224)[0].permute(2,0,1)
                image=torch.round(image*255).clamp(0,255).to(torch.uint8).to(torch.float32)/255
            sample[key]=image
        batch_cases.append((c,state,target,~is_pad,pre(sample)))
    noises={seed:torch.randn((1,50,32),generator=torch.Generator(device='cpu').manual_seed(seed),dtype=torch.float32) for seed in SEEDS}
    noise_hashes={seed:hashlib.sha256(value.numpy().tobytes()).hexdigest() for seed,value in noises.items()}
    noises={seed:noise.to('cuda') for seed,noise in noises.items()}
    model,load_proof=load_base_streaming(base,cfg)
    pc=PeftConfig.from_pretrained(str(adapter),local_files_only=True);pc.base_model_name_or_path=str(base)
    policy=PeftModel.from_pretrained(model,str(adapter),config=pc,local_files_only=True,is_trainable=False)
    policy.eval().requires_grad_(False)
    rows=[]; adapter_verifications={}
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'robot_io':False,'production_model_or_config_changed':False,
            'training_performed':False,'episodes':list(EPISODES),'frames':list(FRAMES),'seeds':list(SEEDS),'inference_steps':10,
            'image_mode':args.image_mode,'noise_source':'fixed CPU float32 standard normal, same tensors for all checkpoints',
            'noise_sha256':noise_hashes,'checkpoint_identity':identities,'load_proof':load_proof,
            'bounds':DEFAULT_BOUNDS,'max_step':DEFAULT_MAX_STEP,'guard_metrics':'diagnostic vs input state only, not execution gates',
            'selection_thresholds':{'first_joint_mae_relative_reduction':.10,'first_joint_mae_absolute_reduction_deg':.5,
                                     'full50_joint_mae_must_not_increase':True,'first_and_full_gripper_mae_must_not_increase':True}}
    write_json(args.output/'manifest.json',report)
    first_batch=batch_cases[0][-1]
    with torch.inference_mode():
        policy.predict_action_chunk(first_batch,noise=noises[0].clone(),num_steps=10)
    torch.cuda.synchronize()
    with (args.output/'rows.jsonl').open('x') as stream:
        for step in STEPS:
            folder=args.checkpoint_root/f'{step:06d}/pretrained_model'
            saved=load_file(str(folder/'adapter_model.safetensors'),device='cpu')
            if len(saved)!=80: raise RuntimeError(f'Checkpoint{step} has unexpected LoRA tensor count')
            expected_keys=set(get_peft_model_state_dict(policy,adapter_name='default'))
            if set(saved)!=expected_keys: raise RuntimeError(f'Checkpoint{step} LoRA keys mismatch')
            applied=set_peft_model_state_dict(policy,saved,adapter_name='default',ignore_mismatched_sizes=False)
            if applied.unexpected_keys or any('lora_' in k for k in applied.missing_keys):
                raise RuntimeError(f'Checkpoint{step} adapter loading missing/unexpected keys: {applied}')
            actual=get_peft_model_state_dict(policy,adapter_name='default')
            for key,value in saved.items():
                v=actual[key].detach().cpu()
                if value.shape!=v.shape or not torch.equal(value.to(v.dtype),v):
                    raise RuntimeError(f'Checkpoint{step} LoRA tensor mismatch:{key}')
            adapter_verifications[step]={'verified_tensors':len(saved),'keys_shapes_values_exact':True}
            del saved,actual,applied
            policy.eval().requires_grad_(False)
            for case,state,target,valid,batch in batch_cases:
                for seed in SEEDS:
                    with torch.inference_mode():
                        policy.reset();torch.cuda.synchronize();started=time.perf_counter()
                        prediction=policy.predict_action_chunk(batch,noise=noises[seed].clone(),num_steps=10)
                        torch.cuda.synchronize();model_end=time.perf_counter()
                        actions=post(prediction).detach().cpu().numpy()[0]
                        torch.cuda.synchronize();finished=time.perf_counter()
                    if actions.shape!=(50,7) or not np.isfinite(actions).all(): raise RuntimeError('Invalid predicted actions')
                    error=np.abs(actions-target);delta=np.abs(actions[0]-state)
                    bounds=np.asarray(DEFAULT_BOUNDS,dtype=np.float32)
                    joint_bad=bool((delta[:6]>6).any());grip_bad=bool(delta[6]>.008)
                    bounds_bad=bool(((actions[0]<bounds[:,0])|(actions[0]>bounds[:,1])).any())
                    metrics={'first_joint_mae_deg':float(error[0,:6].mean()),'first_gripper_mae_mm':float(error[0,6]*1000),
                             'full50_joint_mae_deg':float(error[valid,:6].mean()),'full50_gripper_mae_mm':float(error[valid,6].mean()*1000),
                             'per_joint_first_abs_error_deg':error[0,:6].tolist(),'per_joint_full50_mae_deg':error[valid,:6].mean(axis=0).tolist(),
                             'first_joint_step_violation':joint_bad,'first_gripper_step_violation':grip_bad,
                             'first_bounds_violation':bounds_bad,'first_any_numeric_violation':joint_bad or grip_bad or bounds_bad}
                    row={'checkpoint_step':step,'episode_index':case['episode_index'],'frame_index':case['frame_index'],'seed':seed,
                         'noise_sha256':noise_hashes[seed],'input_file':case['input_file'],'input_sha256':case['input_sha256'],
                         'state':state.tolist(),'target_actions':target.tolist(),'valid_target_action_mask':valid.tolist(),
                         'actions':actions.tolist(),'metrics':metrics,'timing_s':{'model':model_end-started,'post':finished-model_end,'model_post':finished-started}}
                    rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
            print(json.dumps({'checkpoint_step':step,'completed_cases':24,'total_predictions_completed':len(rows),'verified_adapter_tensors':80}),flush=True)
    report.update(adapter_verifications=adapter_verifications,rows=rows,summary=summarize_rows(rows))
    write_json(args.output/'result.json',report);write_json(args.output/'summary.json',report['summary'])
    print(json.dumps(report['summary'],ensure_ascii=False),flush=True)

if __name__=='__main__': main()

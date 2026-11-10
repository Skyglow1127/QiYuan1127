#!/usr/bin/env python3
"""Real PI05+020000 LoRA: switch between GPU vision and NPU vision+GPU actions."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
import torch
from common import ROOT, RECORDED, load_policy, stats, write_json
from npu_bridge import NpuClient, NpuPrefix

def summary(values):
    a=np.asarray(values,dtype=np.float64)
    return dict(n=len(a),mean=float(a.mean()),median=float(np.median(a)),min=float(a.min()),max=float(a.max()),each=a.tolist())

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--vision',choices=['gpu','npu','both'],default='both')
    ap.add_argument('--recorded',type=Path,default=RECORDED)
    ap.add_argument('--output-dir',type=Path,default=ROOT/'results/integrated')
    ap.add_argument('--warmup',type=int,default=2)
    ap.add_argument('--samples',type=int,default=3)
    ap.add_argument('--seed',type=int,default=0)
    ap.add_argument('--npu-model',type=Path,default=ROOT/'models/vision_batch1.onnx')
    ap.add_argument('--npu-config',type=Path,default=ROOT/'code/vaiml_tp1.json')
    args=ap.parse_args()
    if args.warmup<1 or args.samples<1: ap.error('warmup and samples must be positive')
    out=args.output_dir.resolve(); out.mkdir(parents=True,exist_ok=False)
    report=dict(command=[sys.executable,*sys.argv],robot_io=False,seed=args.seed,
                warmup_per_path=args.warmup,samples_per_path=args.samples,
                recorded=str(args.recorded),paths=args.vision)
    write_json(out/'report.json',report)
    client=None; bridge=None
    try:
        if args.vision!='gpu':
            print('Starting NPU worker before loading the GPU model.',flush=True)
            client=NpuClient(out,args.npu_model,args.npu_config)
            report['npu_worker_start_seconds']=client.start_seconds
            report['npu_session']=client.session
            print('NPU worker ready; actual NPU execution audit passed.',flush=True)
        policy,base,cfg,batch,post,sample,loading=load_policy(args.recorded)
        report['loading']=loading
        torch.manual_seed(args.seed)
        noise=base.model.sample_noise((1,cfg.chunk_size,cfg.max_action_dim),'cuda')
        np.save(out/'initial_noise.npy',noise.cpu().numpy())
        with torch.inference_mode():
            images,masks=base._preprocess_images(batch)
            pixels=torch.cat(images).contiguous().cpu().numpy()
        np.save(out/'preprocessed_pixels.npy',pixels)
        write_json(out/'input.json',dict(task=sample['task'],camera_order=list(cfg.image_features),
            masks=[x.cpu().tolist() for x in masks],pixels_shape=list(pixels.shape),pixels_dtype=str(pixels.dtype),
            pixels_sha256=hashlib.sha256(pixels.tobytes()).hexdigest(),
            state=sample['observation.state'].cpu().tolist(),
            noise_sha256=hashlib.sha256(noise.cpu().numpy().tobytes()).hexdigest(),
            tokens=batch['observation.language.tokens'].cpu().tolist(),
            language_attention_mask=batch['observation.language.attention_mask'].cpu().tolist()))
        if client: bridge=NpuPrefix(base.model,client)
        paths=['gpu','npu'] if args.vision=='both' else [args.vision]
        times={p:[] for p in paths}; records={p:[] for p in paths}; outputs={p:[] for p in paths}
        npu_feature_samples=[]
        cold={}
        def predict(path):
            if bridge:
                bridge.install() if path=='npu' else bridge.restore()
            before=len(bridge.records) if bridge else 0
            policy.reset(); torch.cuda.synchronize()
            t=time.perf_counter()
            with torch.inference_mode():
                action=post(policy.predict_action_chunk(batch,noise=noise.clone())).detach().cpu()
            torch.cuda.synchronize(); elapsed=time.perf_counter()-t
            assert tuple(action.shape)==(1,50,7) and torch.isfinite(action).all()
            if bridge and path=='npu': assert len(bridge.records)==before+1, 'NPU was not called exactly once'
            return action[0].numpy(),elapsed,(bridge.records[-1].copy() if bridge and path=='npu' else None)
        for p in paths:
            a,t,r=predict(p); cold[p]=dict(seconds=t,vision=r)
            np.save(out/f'{p}_actions_cold.npy',a)
            if p=='npu': np.save(out/'npu_features_cold.npy',bridge.last_features)
            print(f'{p} first complete chunk: {t:.6f} s',flush=True)
        for i in range(args.warmup):
            for p in paths:
                _,t,_=predict(p)
                print(f'{p} warmup {i+1}: {t:.6f} s',flush=True)
        for i in range(args.samples):
            for p in paths:
                a,t,r=predict(p); times[p].append(t); outputs[p].append(a)
                if r: records[p].append(r)
                if p=='npu': npu_feature_samples.append(bridge.last_features.copy())
                print(f'{p} sample {i+1}: {t:.6f} s',flush=True)
        if bridge: bridge.restore()
        report['cold_complete_chunks']=cold
        report['complete_chunk_seconds']={p:summary(v) for p,v in times.items()}
        if client:
            report['npu_stage_seconds']={k:summary([r[k] for r in records['npu']]) for k in records['npu'][0] if k.endswith('_seconds')}
            np.save(out/'npu_features_samples.npy',np.stack(npu_feature_samples))
            np.save(out/'npu_features.npy',npu_feature_samples[0])
            np.save(out/'npu_features_last.npy',npu_feature_samples[-1])
            report['npu_calls_completed']=len(bridge.records)
            report['npu_measured_calls']=records['npu']
            report['npu_feature_repeat_error']=max(
                (stats(npu_feature_samples[0],a) for a in npu_feature_samples),key=lambda v:v['max_abs'])
        # Measure original, sequential dual-camera GPU vision after full-path tests.
        vision_times=[]
        with torch.inference_mode():
            for i in range(args.warmup+args.samples):
                torch.cuda.synchronize(); t=time.perf_counter()
                gpu_features=torch.cat([base.model.paligemma_with_expert.embed_image(x) for x in images])
                torch.cuda.synchronize(); elapsed=time.perf_counter()-t
                if i>=args.warmup: vision_times.append(elapsed)
        np.save(out/'gpu_features.npy',gpu_features.cpu().numpy())
        report['gpu_vision_seconds']=summary(vision_times)
        if client:
            report['feature_error']=stats(gpu_features.cpu().numpy(),npu_feature_samples[0])
            report['feature_error']['by_camera']={name:stats(gpu_features.cpu().numpy()[i],npu_feature_samples[0][i]) for i,name in enumerate(('global','wrist'))}
            report['feature_action_alignment']='npu_features.npy and npu_actions.npy both come from the first measured prediction; all measured features/actions retained by matching sample index'
            report['output_zero_diagnostic']=dict(
                gpu_zero_fraction=float(np.mean(gpu_features.cpu().numpy()==0)),
                npu_zero_fraction=float(np.mean(bridge.last_features==0)),
                npu_zero_fraction_by_256_channels=[float(np.mean(bridge.last_features[:,:,i:i+256]==0)) for i in range(0,2048,256)])
        for p in paths:
            np.save(out/f'{p}_actions_samples.npy',np.stack(outputs[p]))
            np.save(out/f'{p}_actions.npy',outputs[p][0])
            np.savetxt(out/f'{p}_actions.csv',outputs[p][0],delimiter=',',header=','.join(cfg.action_feature_names),comments='')
            write_json(out/f'{p}_actions.json',dict(names=cfg.action_feature_names,units=['degree']*6+['metre'],
                representation='absolute joint targets and gripper width',actions=outputs[p][0].tolist()))
            report[f'{p}_repeat_error']=max(
                (stats(outputs[p][0],a) for a in outputs[p]),key=lambda v:v['max_abs'])
        if len(paths)==2:
            a,b=outputs['gpu'][0],outputs['npu'][0]
            report['action_error']=stats(a,b)
            report['action_error']['per_axis']={name:dict(unit=('degree' if i<6 else 'metre'),**stats(a[:,i],b[:,i]),
                 signed_mean=float((b[:,i]-a[:,i]).mean())) for i,name in enumerate(cfg.action_feature_names)}
            report['action_error']['overall_note']='Mixed units: use per-axis metrics for physical interpretation.'
            np.save(out/'action_difference_npu_minus_gpu.npy',b-a)
            report['speedup_gpu_over_hybrid']=np.median(times['gpu'])/np.median(times['npu'])
        if 'gpu' in outputs and args.recorded==RECORDED and args.seed==0:
            original=json.loads((ROOT/'results/original_gpu_baseline/predicted_actions.json').read_text())
            report['original_gpu_baseline_error']=stats(np.asarray(original['actions']),outputs['gpu'][0])
        numeric_fault=bool(client and report['output_zero_diagnostic']['npu_zero_fraction']>.4
            and report['output_zero_diagnostic']['gpu_zero_fraction']<.001)
        # A numerical regression gate, not a robot/task-success acceptance test.
        # Limits are declared before the repaired full-model measurement.
        if client:
            f=report['feature_error']
            ref=gpu_features.float().cpu().numpy().astype(np.float64)
            relative_rmse=f['rmse']/float(np.sqrt(np.mean(ref**2)))
            numerical_checks=dict(
                finite_features=bool(f['all_finite']),
                no_half_zero_fault=not numeric_fault,
                feature_cosine_at_least_0_9999=f['cosine']>=.9999,
                feature_relative_rmse_at_most_0_02=relative_rmse<=.02,
                repeat_features_exact=report['npu_feature_repeat_error']['max_abs']==0,
                repeat_actions_exact=report['npu_repeat_error']['max_abs']==0)
            report['numerical_regression']=dict(checks=numerical_checks,
                feature_relative_rmse=relative_rmse,passed=all(numerical_checks.values()),
                scope='Single recorded camera pair; engineering numerical regression only. Per-axis action deviations are reported, not accepted against unspecified robot tolerances.')
            numeric_fault=not all(numerical_checks.values())
        report.update(passed=not numeric_fault,execution_passed=True,closed_loop_executed=True,
            numerical_status=('failed_numerical_regression' if numeric_fault else 'numerical_regression_passed; no task-success acceptance claimed'),
            numerical_fault=numeric_fault,action_shape=[50,7],action_names=cfg.action_feature_names,units=['degree']*6+['metre'],
            timing_scope='predict_action_chunk including original image preprocessing, NPU IPC when selected, 10 denoise steps, original postprocessor and CPU action return; tokenizer preprocessor excluded equally',
            npu_timing_scope='session.run includes ORT internal CPU/device copies and NPU synchronization; no claim of kernel-only time')
        write_json(out/'report.json',report)
        print('Offline comparison completed:',out,flush=True)
        if numeric_fault:
            print('NUMERICAL VALIDATION FAILED: inspect numerical_regression and output_zero_diagnostic. Raw features/actions preserved. No robot I/O.',flush=True)
            return 2
        return 0
    except BaseException as e:
        report.update(passed=False,error=f'{type(e).__name__}: {e}')
        write_json(out/'report.json',report)
        raise
    finally:
        if bridge: bridge.restore()
        if client: client.close()

if __name__=='__main__': raise SystemExit(main())

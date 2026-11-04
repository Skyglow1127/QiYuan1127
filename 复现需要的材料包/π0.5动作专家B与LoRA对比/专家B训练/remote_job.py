"""One isolated offline expert-only experiment; preserves existing model A.

Runs smoke, resume check, and a fresh formal run with the official training
entry. No robot/camera/control imports. All large B checkpoints are on the
system mount; best weights are on the data mount.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

BASE=Path('/root/autodl-tmp/piper_pi05/models/pi05_base')
TOKENIZER=Path('/root/autodl-tmp/piper_pi05/models/paligemma_tokenizer')
DATASET=Path('/root/autodl-tmp/lerobot_act/datasets/rad_place_merged_v5')
MODEL_A=Path('/root/autodl-tmp/piper_pi05/runs/pi05_train_v5_r06')
SERVICE=Path('/root/autodl-tmp/piper_pi05/cloud_grasp_v5')
GPU_UUID='GPU-3850da4c-2c9b-ad14-f678-de227040c0d2'
BASE_SHA='0eb11ca9587678c1d2ef8cf32807c29f8ce53a2bfdfc1aa4a4c96f16fca59b0f'
A_SHA='8e064a68c5991f1b1321e726d901968c516ec728971aa4b9eb1fc6d4213f14a1'


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(8*1024**2),b''):h.update(b)
    return h.hexdigest()


def save(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n');tmp.replace(path)


def snapshot_a():
    assets={str(p):{'sha256':sha(p),'bytes':p.stat().st_size}
            for p in sorted(MODEL_A.rglob('*')) if p.is_file() and not p.is_symlink()}
    final=MODEL_A/'checkpoints/020000/pretrained_model/adapter_model.safetensors'
    if assets[str(final)]['sha256']!=A_SHA:raise RuntimeError('Original A adapter hash mismatch')
    links={str(p):os.readlink(p) for p in MODEL_A.rglob('*') if p.is_symlink()}
    return {'files':assets,'symlinks':links,
            'base_stat':{'bytes':(BASE/'model.safetensors').stat().st_size,
                         'mtime_ns':(BASE/'model.safetensors').stat().st_mtime_ns},
            'base_sha256':sha(BASE/'model.safetensors'),
            'deployment_files':{str(SERVICE/n):sha(SERVICE/n) for n in
                ['server.py','cloud_config.json','pi05_low_memory.py']},
            'dataset_meta':{str(DATASET/'meta'/n):sha(DATASET/'meta'/n)
                for n in ['info.json','stats.json','tasks.parquet','merge_manifest.json']}}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--experiment-id',required=True)
    args=parser.parse_args();ident=args.experiment_id
    if not ident.startswith('pi05_expert_only_v6_') or not ident.replace('_','').isalnum():
        raise RuntimeError('Invalid isolated experiment ID')
    root=Path('/root/autodl-tmp/piper_pi05/experiments')/ident
    scratch=Path('/root/pi05_expert_runs')/ident
    entry=Path(__file__).resolve().parent
    if entry!=root/'bin':raise RuntimeError('Entry must be in isolated experiment/bin')
    if (root/'job_status.json').exists() or scratch.exists():
        raise RuntimeError('Refusing to overwrite an existing experiment')
    scratch.mkdir(parents=True)
    state={'experiment_id':ident,'started_utc':datetime.now(timezone.utc).isoformat(),
           'supervisor_pid':os.getpid(),'data_root':str(root),'scratch_root':str(scratch),
           'original_model_A':str(MODEL_A),'robot_operations':False,'batch_size':8,'automatic_A_service_restore':False}
    def status(stage,**kw):
        state.update(stage=stage,updated_utc=datetime.now(timezone.utc).isoformat(),**kw)
        save(root/'job_status.json',state);print(json.dumps(state,ensure_ascii=False),flush=True)
    child=None
    def run(stage,command,env):
        nonlocal child
        log=root/'logs'/f'{stage}.log';log.parent.mkdir(exist_ok=True)
        with log.open('w') as f:
            child=subprocess.Popen(command,env=env,cwd=entry,stdout=f,stderr=subprocess.STDOUT,
                                   start_new_session=True)
            status(stage,child_pid=child.pid,child_log=str(log),command=command)
            code=child.wait()
        if code:raise RuntimeError(f'{stage} exited {code}; inspect {log}')
        child=None
    try:
        status('protecting_model_A')
        before=snapshot_a()
        if before['base_sha256']!=BASE_SHA:raise RuntimeError('Original base hash mismatch')
        save(root/'model_A_before.json',before)
        env=dict(os.environ)
        for key in ['PYTHONPATH','LD_LIBRARY_PATH']:env.pop(key,None)
        env.update(PI05_EXPERIMENT_ROOT=str(scratch),PI05_BASE_MODEL_PATH=str(BASE),
          PI05_TOKENIZER_PATH=str(TOKENIZER),PI05_PYTHON=sys.executable,
          CUDA_VISIBLE_DEVICES=GPU_UUID,HF_HUB_OFFLINE='1',HF_DATASETS_OFFLINE='1',
          PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='8',MKL_NUM_THREADS='8',
          PI05_EVAL_STEPS='0',PI05_SAVE_FREQ='100',PI05_LOG_FREQ='10',PI05_KEEP_CHECKPOINTS='1')
        env.pop('PI05_BEST_MODEL_DIR',None)
        save(root/'environment.json',{'python':sys.executable,'gpu_uuid':GPU_UUID,
          'versions':{n:importlib.metadata.version(n) for n in
             ['torch','lerobot','transformers','peft','accelerate','safetensors']},
          'new_entry_files':{p.name:sha(p) for p in entry.iterdir() if p.is_file()},
          'smoke_environment':{k:v for k,v in env.items() if k.startswith('PI05_') or k=='CUDA_VISIBLE_DEVICES'}})
        run('smoke_100',['bash',str(entry/'train_expert.sh'),str(DATASET),str(scratch/'smoke'),'100'],env)
        smoke=scratch/'smoke';checkpoint=smoke/'checkpoints/000100'
        if not checkpoint.exists():
            checkpoint=Path(os.path.realpath(smoke/'checkpoints/last'))
        audit=json.loads((smoke/'audit/checkpoint_000100.json').read_text())
        if not audit['exact_cpu_tensor_reload_verified'] or not audit['complete_frozen_vlm_sha256_unchanged']:
            raise RuntimeError('Smoke checkpoint integrity failed')
        shutil.copytree(smoke/'audit',root/'smoke_100_audit')
        save(root/'smoke_checkpoint_manifest.json',audit)
        resume=[sys.executable,str(entry/'train_expert_checked.py'),
           '--config_path='+str(checkpoint/'pretrained_model/train_config.json'),
           '--resume=true','--steps=101','--eval_steps=0','--save_freq=101','--log_freq=1']
        run('smoke_resume_101',resume,env)
        resumed=json.loads((smoke/'audit/checkpoint_000101.json').read_text())
        if resumed['step']!=101 or not resumed['exact_cpu_tensor_reload_verified']:
            raise RuntimeError('Smoke resume checkpoint validation failed')
        shutil.copytree(smoke/'audit',root/'smoke_resume_101_audit')
        save(root/'smoke_resume_checkpoint_manifest.json',resumed)
        if snapshot_a()!=before:raise RuntimeError('Original A/base/deployment/data changed during smoke')
        # These files were created by this job and have verified audit receipts.
        # Preserve both manifests/audits, then release only its smoke weights.
        cleanup=smoke/'checkpoints'
        if not cleanup.resolve().is_relative_to(scratch) or cleanup.name!='checkpoints':
            raise RuntimeError('Invalid owned smoke cleanup path')
        shutil.rmtree(cleanup)
        save(root/'smoke_checkpoint_cleanup.json',{'removed_only_owned_path':str(cleanup),
           'reason':'Verified transient smoke weights; free system mount for rolling formal checkpoints',
           'audits_and_hash_manifests_preserved':True,'model_A_untouched':True})
        status('smoke_passed',child_pid=None,smoke_checkpoint_bytes=audit['total_checkpoint_bytes'],
               resume_step_verified=101,model_A_unchanged=True)
        env.update(PI05_EVAL_STEPS='2000',PI05_SAVE_FREQ='2000',PI05_LOG_FREQ='50',
                   PI05_KEEP_CHECKPOINTS='1',PI05_BEST_MODEL_DIR=str(root/'best_model'))
        run('formal_training',['bash',str(entry/'train_expert.sh'),str(DATASET),str(scratch/'train'),'20000'],env)
        final=scratch/'train/checkpoints/020000'
        end=json.loads((final/'training_state/training_step.json').read_text())
        if end['step']!=20000:raise RuntimeError('Formal run did not reach 20000')
        after=snapshot_a();save(root/'model_A_after.json',after)
        if before!=after:raise RuntimeError('Original model A/base/deployment/data was modified')
        shutil.copytree(scratch/'train/audit',root/'formal_audit')
        save(root/'comparison_manifest.json',{'model_A_unchanged':True,'B_final_checkpoint':str(final),
          'B_best_model':str(root/'best_model'),'B_final_step':20000,'B_uses_A_adapter':False,
          'B_uses_PEFT':False,'B_initialization':'original pi05_base, fresh after smoke',
          'B_batch_size':8,'B_seed':1000,'B_peak_lr':2.5e-5,'B_sample_exposure':160000,
          'frozen_modules':'all paligemma VLM','updated_modules':'original gemma_expert and action/time projections',
          'learning_rate_caveat':'Different LR, batch and exposure from A; compare complete training schemes, not isolated method variable',
          'validation_caveat':'Same official batch-mean implementation and held-out episodes, not physical success'})
        status('complete',child_pid=None,final_checkpoint=str(final),model_A_unchanged=True,
               model_A_service_auto_restore_disabled=True,training_complete=True)
    except BaseException as error:
        status('failed',error=type(error).__name__+': '+str(error),traceback=traceback.format_exc(),
               training_complete=False)
        raise


if __name__=='__main__':main()

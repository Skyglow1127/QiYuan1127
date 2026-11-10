"""Reproducible offline ACT pilot. Training never imports robot communication."""
import argparse
import json
import random
import time
from pathlib import Path
import cv2
import numpy as np
import torch
from .act import RGBEncoder,build_policy,loss_terms
from .prepare import sha,read
from .contracts import CAMERAS


def rgb_tensor(row,size):
    images=[]
    for role in CAMERAS:
        c=row['cameras'][role]
        if sha(c['path'])!=c['sha256']:raise ValueError('image changed after preparation')
        bgr=cv2.imread(c['path'])
        if bgr is None:raise ValueError('invalid_image_bytes')
        rgb=cv2.cvtColor(cv2.resize(bgr,(size[1],size[0]),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2RGB)
        images.append(torch.from_numpy(rgb.transpose(2,0,1).copy()).float()/255.)
    return torch.stack(images)


def tensors(rows,stats,target_space='absolute'):
    q=torch.tensor([r['q'] for r in rows]);a=torch.tensor([r['actions'] for r in rows]);pad=torch.tensor([r['is_pad'] for r in rows])
    qm,qs,am,ast=(torch.tensor(stats[k]) for k in ('q_mean','q_std','action_mean','action_std'))
    target=a-q[:,None,:] if target_space=='delta_from_q' else a
    return (q-qm)/qs,(target-am)/ast,pad,q,a


def metrics(pred,gt,pad,q,stats):
    e=(pred-gt).abs();mask=~pad
    perjoint=e[mask].mean(0)
    scale=torch.tensor(stats['action_std'])
    return dict(normalized_l1=float((e/scale)[mask].mean()),arm_mae_deg=float(perjoint[:6].mean()),
        gripper_mae_sdk_raw=float(perjoint[6]),per_joint_mae=perjoint.tolist(),
        first_step_arm_mae_deg=float(e[:,0,:6].mean()),samples=len(q),valid_target_steps=int(mask.sum()))


def train(dataset,output,steps=300,seed=42,batch_size=16,target_space='delta_from_q',feature_cache=None):
    root=Path(dataset).resolve();m=read(root/'manifest.json');cfg=m['config'];out=Path(output).resolve();out.mkdir(parents=True,exist_ok=False)
    if sha(m['examples_file'])!=m['examples_sha256']:raise ValueError('examples_hash_mismatch')
    rows=[json.loads(x) for x in Path(m['examples_file']).read_text().splitlines()]
    torch.set_num_threads(4);cv2.setNumThreads(1);torch.manual_seed(seed);random.seed(seed);np.random.seed(seed)
    torch.use_deterministic_algorithms(True);rng=np.random.default_rng(seed)
    (out/'dataset_manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2))
    cfg=dict(cfg,internal_target_space=target_space)
    if target_space=='delta_from_q':
        residuals=np.asarray([np.asarray(r['actions'])[~np.asarray(r['is_pad'])]-np.asarray(r['q'])
                             for r in rows if r['split']=='train'],dtype=object)
        residuals=np.concatenate(list(residuals)).astype(np.float32)
        m['stats']=dict(m['stats'],action_mean=residuals.mean(0).tolist(),
            action_std=np.maximum(residuals.std(0),.1).tolist(),action_statistics_space=target_space)
    start=time.perf_counter()
    encoder=RGBEncoder(pretrained=not bool(feature_cache));features=[]
    if feature_cache:
        fc=Path(feature_cache).resolve();old=torch.load(fc/'best.pt',weights_only=True,map_location='cpu')
        if old['dataset_manifest_sha256']!=sha(root/'manifest.json'):raise ValueError('feature_cache_dataset_mismatch')
        stored=torch.load(fc/'features.pt',weights_only=True,map_location='cpu')
        if stored['ids']!=[r['id'] for r in rows]:raise ValueError('feature_cache_order_mismatch')
        encoder.load_state_dict(old['encoder']);features=stored['features']
        for r in rows:
            for c in r['cameras'].values():
                if sha(c['path'])!=c['sha256']:raise ValueError('image_changed_after_cache')
    else:
        with torch.inference_mode():
            for lo in range(0,len(rows),16):
                rgb=torch.stack([rgb_tensor(r,cfg['image_size_hw']) for r in rows[lo:lo+16]])
                features.append(encoder(rgb).to(torch.float16))
                if lo%160==0:print(json.dumps(dict(phase='encode',done=min(lo+16,len(rows)),total=len(rows))),flush=True)
        features=torch.cat(features)
    torch.save(dict(features=features,ids=[r['id'] for r in rows]),out/'features.pt')
    # Frozen visual features allow fast CPU updates; no future images enter the model.
    q,a,pad,rawq,rawa=tensors(rows,m['stats'],target_space);ids={s:np.array([i for i,r in enumerate(rows) if r['split']==s]) for s in ('train','val','test')}
    model=build_policy(cfg);opt=torch.optim.AdamW(model.parameters(),lr=1e-4,weight_decay=1e-4)
    am=torch.tensor(m['stats']['action_mean']);ast=torch.tensor(m['stats']['action_std']);history=[];best=float('inf');beststep=None
    def evaluate(index,feature_override=None):
        preds=[];model.eval()
        with torch.inference_mode():
            for lo in range(0,len(index),32):
                ix=index[lo:lo+32];feat=features[ix].float() if feature_override is None else feature_override[lo:lo+len(ix)]
                pred=model(q[ix],feat,None)[0]*ast+am
                if target_space=='delta_from_q':pred=pred+rawq[ix,None,:]
                preds.append(pred)
        p=torch.cat(preds);return metrics(p,rawa[index],pad[index],rawq[index],m['stats'])
    val=ids['val'];tr=ids['train'];baseline={}
    for name,ix in [('train',tr),('val',val)]:
        hold=rawq[ix,None,:].expand(-1,cfg['chunk_size'],-1)
        baseline[name+'_hold_current']=metrics(hold,rawa[ix],pad[ix],rawq[ix],m['stats'])
        mean=am.view(1,1,7).expand_as(hold)
        if target_space=='delta_from_q':mean=mean+rawq[ix,None,:]
        baseline[name+'_train_mean']=metrics(mean,rawa[ix],pad[ix],rawq[ix],m['stats'])
    baseline['untrained_val']=evaluate(val)
    for step in range(1,steps+1):
        model.train();ix=rng.choice(tr,size=min(batch_size,len(tr)),replace=False)
        loss,l1,kl=loss_terms(model,q[ix],features[ix].float(),a[ix],pad[ix])
        if not torch.isfinite(loss):raise ValueError('nonfinite_training_loss')
        opt.zero_grad();loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),1.);opt.step()
        if step==1 or step%25==0 or step==steps:
            vm=evaluate(val);record=dict(step=step,loss=float(loss.detach()),l1=float(l1.detach()),kl=float(kl.detach()),validation=vm)
            history.append(record);print(json.dumps(record),flush=True)
            if vm['normalized_l1']<best:
                best=vm['normalized_l1'];beststep=step
                torch.save(dict(schema='d1-act-pilot-v1',model=model.state_dict(),encoder=encoder.state_dict(),
                    config=cfg,stats=m['stats'],dataset_manifest_sha256=sha(root/'manifest.json'),
                    skill_id=cfg['skill_id'],shadow_only=True,step=step),out/'best.pt')
    checkpoint=torch.load(out/'best.pt',weights_only=True);model.load_state_dict(checkpoint['model']);model.eval()
    results={name:evaluate(ix) for name,ix in ids.items() if len(ix)}
    # Compare input ablations for diagnosis only. Never select a new model from these.
    with torch.inference_mode():
        results['val_zero_vision']=evaluate(val,torch.zeros_like(features[val]).float())
        shuffled=features[val[::-1].copy()].float()
        results['val_shuffled_vision']=evaluate(val,shuffled)
    manifest=dict(schema='d1-act-offline-result-v1',status='shadow_only',scope=cfg['scope'],seed=seed,steps=steps,best_step=beststep,
        config=cfg,train_device='cpu',npu_used=False,gpu_used=False,robot_commands=0,hardware_accessed=False,
        model_parameters=sum(p.numel() for p in model.parameters()),frozen_visual_parameters=sum(p.numel() for p in encoder.parameters()),
        elapsed_s=time.perf_counter()-start,counts=m['counts'],complete_grasp_episodes=0,
        metrics=results,baselines=baseline,selection='minimum deployment-prior validation normalized L1; no physical test',
        model_sha256=sha(out/'best.pt'),dataset_manifest_sha256=sha(root/'manifest.json'),
        source_sha256={str(p.resolve()):sha(p) for p in Path(__file__).parent.glob('*.py')},
        upstream=read(Path(__file__).parent/'vendor/act/upstream.json'),
        runtime=dict(torch=torch.__version__,opencv=cv2.__version__),
        promotion=False,promotion_reason='Lift fragments only; no complete independent closed-loop grasp evaluation; J4 fault unresolved in latest recorded physical run.')
    (out/'learning_curve.json').write_text(json.dumps(history,indent=2)+'\n')
    (out/'summary.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'summary':str(out/'summary.json'),'best_step':beststep,'metrics':results,'baselines':baseline,'elapsed_s':manifest['elapsed_s']}),flush=True)
    return manifest


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',required=True);p.add_argument('--output',required=True)
    p.add_argument('--steps',type=int,default=300);p.add_argument('--seed',type=int,default=42);p.add_argument('--batch-size',type=int,default=16)
    p.add_argument('--target-space',choices=['absolute','delta_from_q'],default='delta_from_q')
    p.add_argument('--feature-cache')
    a=p.parse_args()
    if a.steps<1 or a.batch_size<1:raise ValueError('positive steps and batch size required')
    train(a.dataset,a.output,a.steps,a.seed,a.batch_size,a.target_space,a.feature_cache)
if __name__=='__main__':main()

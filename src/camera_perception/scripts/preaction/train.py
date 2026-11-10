"""Repeatable offline PREACTION training from causal historical image pairs."""
import os
os.environ.setdefault('YOLO_OFFLINE','true')
os.environ.setdefault('YOLO_CONFIG_DIR','/home/aup/Camera/output/preaction_20261023/yolo_settings')
from pathlib import Path
import argparse,json,time,random,shutil,sys
import cv2,numpy as np,torch
from torch.nn import functional as F
from ultralytics import YOLO
from core import (PreAction,SIZE,PARAM_SCALE,YOLO_WEIGHTS,ENCODER_WEIGHTS,sha,box_of,
                  red_mask,current_target,crop_spec,crop,pack_input,warp_tensor)

def prepare(pairs_path,out,yolo_weights):
    manifest=json.loads(Path(pairs_path).read_text()); pairs=manifest['pairs']
    model=YOLO(str(yolo_weights)); rows=[]; arrays={k:[] for k in ('x','y','future_rgb','future_mask','center','current_box','future_box','spec','split','id')}
    for row in pairs:
        current=cv2.imread(row['current_image']); future=cv2.imread(row['future_image'])
        assert sha(row['current_image'])==row['image_t']['sha256']
        assert sha(row['future_image'])==row['image_t1']['sha256']
        mask,selection=current_target(model,current)
        ref=red_mask(future) # Weak target supervision ONLY, never an input/crop selector.
        box=box_of(mask); target=box_of(ref); spec=crop_spec(box)
        cur_rgb=cv2.cvtColor(crop(current,spec),cv2.COLOR_BGR2RGB)
        fut_rgb=cv2.cvtColor(crop(future,spec),cv2.COLOR_BGR2RGB)
        cm=crop(mask,spec,True); fm=crop(ref,spec,True)
        x=pack_input(cur_rgb,cm,row['q_current'],row['delta_q_command'],row['horizon_s'])
        c=(box[:2]+box[2:])/2;tc=(target[:2]+target[2:])/2
        y=np.r_[(tc-c)/spec[2],np.log((target[2:]-target[:2])/(box[2:]-box[:2]))]*PARAM_SCALE
        values=dict(x=x,y=y,future_rgb=fut_rgb.transpose(2,0,1).astype(np.float32)/255,
             future_mask=fm,center=(c-np.array(spec[:2]))/spec[2],current_box=box,
             future_box=target,spec=spec,split=row['split'],id=row['id'])
        for k,v in values.items():arrays[k].append(v)
        rows.append(dict(id=row['id'],split=row['split'],stage=row['stage'],current_selection=selection,
            current_box=box.tolist(),future_weak_box=target.tolist(),crop=spec,
            target_truncated=bool(target[0]<=1 or target[1]<=1 or target[2]>=639 or target[3]>=479),
            label='largest red component in actual future frame; weak, not independent human ground truth'))
    data={k:np.asarray(v,dtype=np.float32 if k not in ('split','id') else str) for k,v in arrays.items()}
    np.savez_compressed(out/'dataset.npz',**data)
    (out/'observations.json').write_text(json.dumps(rows,indent=2))
    shutil.copyfile(pairs_path,out/'pairs.json')
    return data,manifest

def objective(model,x,y,future_rgb,future_mask,centers):
    pred=model(x).reshape(-1,4)
    reg=F.smooth_l1_loss(pred,y)
    warped=warp_tensor(x[:,:4],centers,pred)
    union=torch.maximum(x[:,3:4],future_mask[:,None])
    rgb=(torch.abs(warped[:,:3]-future_rgb)*union).sum()/(union.sum()*3+1e-6)
    silhouette=torch.abs(warped[:,3]-future_mask).mean()
    return reg+.15*rgb+.25*silhouette,pred

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pairs',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--steps',type=int,default=600);p.add_argument('--seed',type=int,default=42)
    p.add_argument('--yolo',type=Path,default=YOLO_WEIGHTS)
    a=p.parse_args(); out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(4);cv2.setNumThreads(1);torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
    torch.use_deterministic_algorithms(True)
    d,manifest=prepare(a.pairs,out,a.yolo)
    tensors={k:torch.from_numpy(d[k]) for k in ('x','y','future_rgb','future_mask','center')}
    train=np.where(d['split']=='train')[0]; val=np.where(d['split']=='val')[0]
    if not len(train) or not len(val):raise ValueError('Train and validation required')
    model=PreAction();opt=torch.optim.AdamW([p for p in model.parameters() if p.requires_grad],lr=.001,weight_decay=.0001)
    logs=[];best=float('inf');best_step=-1;started=time.perf_counter();baseline_val=None
    def inputs(indices,x_override=None):
        return (x_override if x_override is not None else tensors['x'][indices],tensors['y'][indices],
                tensors['future_rgb'][indices],tensors['future_mask'][indices],tensors['center'][indices])
    # No future image, future feedback, or target-derived crop enters the network.
    for step in range(a.steps+1):
        if step:
            model.train(); x=tensors['x'][train].clone()
            gain=float(np.random.uniform(.9,1.1));x[:,:3]=(x[:,:3]*gain).clamp(0,1)
            batch=list(inputs(train,x));batch[2]=(batch[2]*gain).clamp(0,1)
            loss,_=objective(model,*batch);opt.zero_grad();loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(),5.);opt.step()
        if step%10==0:
            model.eval()
            with torch.no_grad():
                tl,_=objective(model,*inputs(train));vl,_=objective(model,*inputs(val))
            logs.append(dict(step=step,train_loss=float(tl),val_loss=float(vl)))
            if step==0:baseline_val=float(vl)
            if step>0 and float(vl)<best-1e-7:
                best=float(vl);best_step=step;torch.save(model.state_dict(),out/'best.pt')
            if step%50==0:print(json.dumps(logs[-1]),flush=True)
            if step>=150 and step-best_step>=120:break
    model.load_state_dict(torch.load(out/'best.pt',weights_only=True));model.eval()
    torch.onnx.export(model,torch.from_numpy(d['x'][:1]),str(out/'preaction.onnx'),
                      input_names=['x'],output_names=['motion'],opset_version=17,dynamo=False)
    np.savez_compressed(out/'calib_train.npz',x=d['x'][train],split=d['split'][train],id=d['id'][train])
    np.savez_compressed(out/'eval_inputs.npz',x=d['x'],split=d['split'],id=d['id'])
    config=dict(seed=a.seed,steps_requested=a.steps,steps_run=step,best_step=best_step,validation_loss=best,
        selection='best trained checkpoint by validation only; persistence compared separately',
        persistence_validation_loss=baseline_val,
        recommended_forecast='trained_candidate' if best<baseline_val else 'persistence',
        train_pairs=len(train),val_pairs=len(val),test_pairs=int(sum(d['split']=='test')),
        input_shape=[1,19,SIZE,SIZE],output_shape=[1,4,1,1],device='cpu',
        model_parameters=sum(p.numel() for p in model.parameters()),trainable_parameters=sum(p.numel() for p in model.parameters() if p.requires_grad),
        frozen_during_prediction_training=['Qwen3.5-2B semantic model','YOLO detector','existing RGB foreground encoder'],
        trained='new action-conditioned translation/scale head; parameter+RGB-warp+silhouette losses',
        action_contract='q at t, planned target minus available q; no future actual feedback input',
        roi_contract='current-frame selection only; same crop applied to future target',
        label_scope='HSV weak labels and actual recorded RGB; no independent-scene validation',
        recording_roots=len({pair.get('episode') for pair in manifest['pairs']}),
        elapsed_training_export_s=time.perf_counter()-started,onnx_sha256=sha(out/'preaction.onnx'),
        yolo_weights=str(a.yolo.resolve()),yolo_sha256=sha(a.yolo),encoder_sha256=sha(ENCODER_WEIGHTS),pairs_sha256=sha(a.pairs),
        production_replacement=False,robot_commands=0)
    (out/'train_config.json').write_text(json.dumps(config,indent=2))
    (out/'learning_curve.json').write_text(json.dumps(logs,indent=2))
    print(json.dumps(config),flush=True)

if __name__=='__main__':main()

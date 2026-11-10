"""Replay saved observations with ACT and OpenCV diagnostics. No SDK/robot I/O."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
import cv2
import numpy as np
import torch
from .act import RGBEncoder,build_policy
from .contracts import ActionChunk,LIMITS
from .prepare import read,sha
from .train import rgb_tensor


def numeric_review(chunk, q, rate):
    """Diagnostic only; deliberately does not claim collision or live readiness."""
    chunk.validate();a=np.asarray(chunk.commands);q=np.asarray(q)
    limits=np.asarray(LIMITS)
    speed=np.diff(a[:,:6],axis=0)*rate
    accel=np.diff(speed,axis=0)*rate
    return dict(finite=True,within_limits=bool(((a>=limits[:,0])&(a<=limits[:,1])).all()),
        first_target_difference_deg=(a[0,:6]-q[:6]).tolist(),
        max_internal_speed_deg_s=float(abs(speed).max()) if speed.size else 0.,
        max_internal_acceleration_deg_s2=float(abs(accel).max()) if accel.size else 0.,
        internal_speed_pass=bool(not speed.size or abs(speed).max()<=.5+1e-5),
        internal_acceleration_pass=bool(not accel.size or abs(accel).max()<=1.+1e-5),
        collision_checked=False,gripper_contact_verified=False,live_eligible=False)


def color_evidence(path):
    bgr=cv2.imread(path);hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    mask=(((hsv[:,:,0]<11)|(hsv[:,:,0]>169))&(hsv[:,:,1]>=70)&(hsv[:,:,2]>=35)).astype('uint8')
    n,labels,stats,centers=cv2.connectedComponentsWithStats(mask,8)
    if n<=1:return dict(red_component_found=False)
    i=1+int(stats[1:,cv2.CC_STAT_AREA].argmax());x,y,w,h,area=stats[i].tolist()
    return dict(red_component_found=area>=40,area_px=area,box_xywh=[x,y,w,h],
        scope='largest red RGB component only; not object identity, depth, grip, or lift success')


class Predictor:
    def __init__(self,checkpoint):
        self.path=Path(checkpoint).resolve();self.sha=sha(self.path)
        c=torch.load(self.path,weights_only=True,map_location='cpu')
        if c.get('schema')!='d1-act-pilot-v1' or not c.get('shadow_only'):raise ValueError('unsupported_checkpoint')
        self.cfg=c['config'];self.stats=c['stats'];self.skill=c['skill_id']
        self.encoder=RGBEncoder(pretrained=False);self.encoder.load_state_dict(c['encoder']);self.encoder.eval()
        self.model=build_policy(self.cfg);self.model.load_state_dict(c['model']);self.model.eval()

    @torch.inference_mode()
    def predict(self,row):
        started=time.perf_counter();rgb=rgb_tensor(row,self.cfg['image_size_hw']).unsqueeze(0)
        # Match the frozen feature precision used during training.
        feat=self.encoder(rgb).to(torch.float16).float()
        q=torch.tensor(row['q']).unsqueeze(0)
        q=(q-torch.tensor(self.stats['q_mean']))/torch.tensor(self.stats['q_std'])
        a=self.model(q,feat,None)[0][0]*torch.tensor(self.stats['action_std'])+torch.tensor(self.stats['action_mean'])
        if self.cfg.get('internal_target_space','absolute')=='delta_from_q':a=a+torch.tensor(row['q'])
        t=row['command_host_monotonic'];rate=self.cfg['rate_hz']
        chunk=ActionChunk(a.tolist(),[t+i/rate for i in range(len(a))],row['id'],self.sha).validate()
        return chunk,(time.perf_counter()-started)*1000


def replay(dataset,checkpoint,output,split='val',count=20):
    if count<1:raise ValueError('count must be positive')
    torch.set_num_threads(4);cv2.setNumThreads(1)
    root=Path(dataset).resolve();m=read(root/'manifest.json')
    if sha(m['examples_file'])!=m['examples_sha256']:raise ValueError('examples changed')
    rows=[json.loads(x) for x in Path(m['examples_file']).read_text().splitlines()];rows=[r for r in rows if r['split']==split]
    if not rows:raise ValueError('empty split')
    ix=np.unique(np.linspace(0,len(rows)-1,min(count,len(rows))).astype(int))
    out=Path(output).resolve();out.mkdir(parents=True,exist_ok=False);predictor=Predictor(checkpoint);records=[]
    ck=torch.load(checkpoint,weights_only=True,map_location='cpu')
    if ck['dataset_manifest_sha256']!=sha(root/'manifest.json'):raise ValueError('checkpoint_dataset_mismatch')
    for index in ix:
        row=rows[index];chunk,ms=predictor.predict(row)
        records.append(dict(observation_id=row['id'],stage=row['stage'],split=split,
            raw_policy_action=asdict(chunk),numerical_review=numeric_review(chunk,row['q'],predictor.cfg['rate_hz']),
            sent_sdk_targets=[],actual_robot_feedback=None,latency_decode_encode_policy_ms=ms,
            opencv={role:color_evidence(c['path']) for role,c in row['cameras'].items()},
            actual_recorded_teacher_first_target=row['actions'][0],q_actual=row['q']))
    (out/'predictions.jsonl').write_text(''.join(json.dumps(x,allow_nan=False)+'\n' for x in records))
    latency=np.array([r['latency_decode_encode_policy_ms'] for r in records]);summary=dict(status='offline_shadow_complete',
        observations=len(records),model_sha256=predictor.sha,source_dataset_sha256=sha(root/'manifest.json'),
        median_decode_encode_policy_ms=float(np.median(latency)),p95_decode_encode_policy_ms=float(np.quantile(latency,.95)),
        numerical_limit_pass=sum(r['numerical_review']['within_limits'] for r in records),
        numerical_velocity_pass=sum(r['numerical_review']['internal_speed_pass'] for r in records),
        numerical_acceleration_pass=sum(r['numerical_review']['internal_acceleration_pass'] for r in records),
        includes_image_hash_read=True,camera_network_time_included=False,physical_closed_loop=False,
        robot_commands=0,hardware_accessed=False,live_eligible=False,npu_used=False)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');return summary


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',required=True);p.add_argument('--checkpoint',required=True);p.add_argument('--output',required=True)
    p.add_argument('--split',choices=['train','val','test'],default='val');p.add_argument('--count',type=int,default=20)
    a=p.parse_args();print(json.dumps(replay(a.dataset,a.checkpoint,a.output,a.split,a.count)))
if __name__=='__main__':main()

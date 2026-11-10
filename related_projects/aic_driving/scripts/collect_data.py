#!/usr/bin/env python3
"""Collect causal noisy observation histories; truth is used only for labels."""
import argparse,json
from collections import Counter
from pathlib import Path
import numpy as np
from aic_drive.types import EpisodeConfig,DetectionPacket,TrackState
from aic_drive.runner import run_episode,make_controller,dump_json,source_manifest
from aic_drive.sensors import TrackHistory
from aic_drive.prediction import AnalyticPredictor
from aic_drive.learning import build_features,FEATURE_SCHEMA

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',required=True);p.add_argument('--episodes-per-scenario',type=int,default=20)
    p.add_argument('--duration',type=float,default=15);p.add_argument('--seed-start',type=int,default=1000)
    a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    n=a.episodes_per_scenario
    if n<20:raise ValueError('Use at least 20 episodes per scenario for four nonempty splits')
    ntrain=int(.6*n);nval=int(.15*n);ncal=max(2,int(.1*n))
    boundaries=np.cumsum([ntrain,nval,ncal,n-ntrain-nval-ncal])
    split_names=['train','validation','calibration','test']
    X,Y,H,IDs,S=[],[],[],[],[]
    records=[];analytic=AnalyticPredictor('ca')
    for si,scenario in enumerate(['follow','brake','cutin']):
        for i in range(n):
            split=split_names[int(np.searchsorted(boundaries,i,side='right'))]
            seed=a.seed_start+si*10000+i
            delay=[0.,.2,.4][i%3]
            eid=f'{scenario}_{seed}'
            cfg=EpisodeConfig(scenario=scenario,seed=seed,duration=a.duration,delay=delay,position_noise=.10,velocity_noise=.10)
            ctrl=make_controller('ca_fixed',target_speed=[23.,25.,27.][i%3])
            summary=run_episode(cfg,'ca_fixed',out/'episodes'/eid,controller=ctrl)
            path=Path(summary['run_dir'])
            rows=[json.loads(line) for line in (path/'trajectory.jsonl').read_text().splitlines()]
            observations=[json.loads(line) for line in (path/'observations.jsonl').read_text().splitlines()]
            truths={}
            for row in rows:
                for st in row['targets']:
                    truths.setdefault(st['track_id'],[]).append((row['time'],st['x'],st['y']))
            truth_arrays={key:np.array(vals) for key,vals in truths.items()}
            history=TrackHistory()
            count=0
            for j,row in enumerate(observations):
                packets=[DetectionPacket(p['captured_at'],p['received_at'],tuple(TrackState(**s) for s in p['tracks'])) for p in row['packets']]
                history.ingest(packets,row['time'])
                if j%3:continue
                for key,samples in history.snapshot().items():
                    if len(samples)<3 or samples[-1].captured_at-samples[0].captured_at<.2-1e-8:continue
                    truth=truth_arrays[key]
                    for future in [.1,.5,1.,1.5,2.]:
                        q=row['time']+future
                        if q>truth[-1,0]+1e-8:continue
                        actual=np.array([np.interp(q,truth[:,0],truth[:,1]),np.interp(q,truth[:,0],truth[:,2])])
                        base=analytic.predict(samples,np.array([q]))[0,:2]
                        X.append(build_features(samples,q));Y.append(actual-base);H.append(q-samples[-1].captured_at);IDs.append(eid);S.append(split);count+=1
            records.append({'episode_id':eid,'split':split,'seed':seed,'scenario':scenario,'samples':count,'source_run':str(path.resolve()),'config_sha256':summary['config_sha256']})
            if (i+1)%5==0:print(json.dumps({'scenario':scenario,'episode':i+1,'of':n,'samples_total':len(X)}),flush=True)
    np.savez_compressed(out/'dataset.npz',features=np.array(X,dtype=np.float32),targets=np.array(Y,dtype=np.float32),horizons=np.array(H,dtype=np.float32),episode_ids=np.array(IDs),split=np.array(S))
    dump_json(out/'metadata.json',{'feature_schema':FEATURE_SCHEMA,'episodes':records,'sample_counts':dict(Counter(S)),
         'collection_provenance':source_manifest(),'arguments':vars(a),'inputs':'online delayed/noisy observation stream only; world coordinates; stable IDs',
         'labels':'future truth position minus CA prediction; never supplied to controller',
         'selection':'every third decision, >=3 observations, >=0.2s history, future within recorded episode',
         'future_offsets_from_decision_s':[.1,.5,1.,1.5,2.],'split_rule':'whole episodes, stratified per scenario; dedicated seed ranges; no shared episode IDs'})
    print(json.dumps({'dataset':str((out/'dataset.npz').resolve()),'counts':dict(Counter(S))},ensure_ascii=False))
if __name__=='__main__':main()

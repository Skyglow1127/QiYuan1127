#!/usr/bin/env python3
"""New data, validation-only model selection, independent calibration and frozen new tests."""
from pathlib import Path
from collections import Counter
import copy,csv,itertools,json,time,zipfile
import numpy as np
from aic_drive.runner import ROOT,run_episode,make_controller,dump_json,source_manifest
from aic_drive.types import EpisodeConfig,TrackState,DetectionPacket
from aic_drive.sensors import TrackHistory
from aic_drive.prediction import AnalyticPredictor
from aic_drive.learning import build_features,FEATURE_SCHEMA,LearnedPredictor,train_model,calibrate_model,_torch,_sha256
from aic_drive.model_v4 import features,SmoothedPredictor,OptimizedPredictor,Net,WINDOWS,EDGES,SCHEMA,predict_residual

def equal_episode(values,ids):
    return float(np.mean([np.mean(values[ids==eid]) for eid in np.unique(ids)]))

def collect(out):
    old=LearnedPredictor(ROOT/'runs/release_v1/model/model.pt');raw=AnalyticPredictor('ca');fits=[SmoothedPredictor(w) for w in WINDOWS]
    arr={k:[] for k in ['x','old_x','y','raw','old','fit','fit_valid','h','ids','split','scenario']};records=[]
    for si,scene in enumerate(['follow','brake','cutin','traffic']):
        for i in range(40):
            split='train' if i<24 else 'validation' if i<30 else 'calibration' if i<34 else 'test'
            seed=2000000+si*10000+i;eid=f'{scene}_{seed}'
            c=EpisodeConfig(scenario=scene,seed=seed,duration=15,delay=[0,.2,.4][(i//4)%3],delay_mode=['fixed','bursty'][(i//2)%2],
                profile=['development','holdout'][i%2],position_noise=[.05,.1,.2][i%3],velocity_noise=[.05,.1,.2][i%3])
            ctrl=make_controller('ca_recovery',calibration_path=ROOT/'runs/release_v1/model/calibration.json',target_speed=[23,25,27][i%3])
            summary=run_episode(c,'ca_recovery',out/'episodes'/eid,controller=ctrl);path=Path(summary['run_dir'])
            rows=[json.loads(l) for l in (path/'trajectory.jsonl').read_text().splitlines()]
            truth={}
            for row in rows:
                for t in row['targets']:truth.setdefault(t['track_id'],[]).append([row['time'],t['x'],t['y']])
            truth={k:np.array(v) for k,v in truth.items()};history=TrackHistory();count=0
            for j,line in enumerate((path/'observations.jsonl').read_text().splitlines()):
                row=json.loads(line);now=row['time'];history.ingest([DetectionPacket(p['captured_at'],p['received_at'],tuple(TrackState(**s) for s in p['tracks'])) for p in row['packets']],now)
                if j%5:continue
                for key,samples in history.snapshot().items():
                    if len(samples)<3 or samples[-1].captured_at-samples[0].captured_at<.2-1e-8:continue
                    q=now+np.array([.1,.3,.5,1,1.5,2,2.5]);q=q[q<=truth[key][-1,0]+1e-8]
                    if not len(q):continue
                    origin=np.array([samples[-1].state.x,samples[-1].state.y])
                    actual=np.column_stack([np.interp(q,truth[key][:,0],truth[key][:,1]),np.interp(q,truth[key][:,0],truth[key][:,2])])-origin
                    xx=features(samples,q);oldxx=np.array([build_features(samples,v) for v in q]);b=raw.predict(samples,q)[:,:2]-origin
                    oldp=old.predict(samples,q)[:,:2]-origin;f=np.stack([p.predict(samples,q)[:,:2]-origin for p in fits],axis=1);fit_valid=np.array([not p.last_degraded for p in fits])
                    for k in range(len(q)):
                        for name,value in [('x',xx[k]),('old_x',oldxx[k]),('y',actual[k]),('raw',b[k]),('old',oldp[k]),('fit',f[k]),('fit_valid',fit_valid),('h',q[k]-samples[-1].captured_at),('ids',eid),('split',split),('scenario',scene)]:arr[name].append(value)
                        count+=1
            records.append({'episode':eid,'split':split,'config':c.to_dict(),'policy':summary['config_sha256'],'samples':count})
            if (i+1)%10==0:print(json.dumps({'collect':scene,'episodes':i+1,'samples':len(arr['h'])}),flush=True)
    data={k:np.asarray(v,dtype=np.float32 if k not in ['ids','split','scenario'] else str) for k,v in arr.items()}
    np.savez_compressed(out/'dataset.npz',**data)
    dump_json(out/'metadata.json',{'feature_schema':SCHEMA,'episodes':records,'sample_counts':dict(Counter(data['split'])),'sampling':'Every fifth decision, received observations only; truth only for future targets','splits':'whole episode; no cross-split episode or target histories'})
    comp=out/'compat';comp.mkdir();np.savez_compressed(comp/'dataset.npz',features=data['old_x'],targets=data['y']-data['raw'],horizons=data['h'],episode_ids=data['ids'],split=data['split'])
    dump_json(comp/'metadata.json',{'feature_schema':FEATURE_SCHEMA})
    return data

def train(data,out):
    torch=_torch();tr=data['split']=='train';val=data['split']=='validation';cal=data['split']=='calibration'
    win_scores=[equal_episode(np.linalg.norm(data['y'][val]-data['fit'][val,i],axis=1),data['ids'][val]) for i in range(3)]
    index=int(np.argmin(win_scores));window=WINDOWS[index];base=data['fit'][:,index];target=data['y']-base
    supported=[float(data['h'][tr].min()),float(data['h'][tr].max())];eligible=(data['fit_valid'][:,index]>0)&(data['h']>=supported[0]-1e-6)&(data['h']<=supported[1]+1e-6)
    tr=tr&eligible
    mean=data['x'][tr].mean(axis=0);scale=data['x'][tr].std(axis=0);scale[scale<1e-6]=1
    inputs=torch.from_numpy(((data['x'][tr]-mean)/scale).astype(np.float32));labels=torch.from_numpy(target[tr].astype(np.float32));h=torch.from_numpy(data['h'][tr]);gate=(h*h/(h*h+.25))[:,None]
    counts=Counter(data['ids'][tr]);weight=np.array([1/counts[e] for e in data['ids'][tr]],dtype=np.float32);weight/=weight.mean();weight=torch.from_numpy(weight)
    states=[];scores=[];models=[];histories=[]
    for seed in [0,1,2]:
        torch.manual_seed(seed);rng=np.random.default_rng(seed);m=Net.build();opt=torch.optim.AdamW(m.parameters(),lr=.001,weight_decay=1e-4);best=float('inf');best_state=None;stale=0;curve=[]
        for epoch in range(1,101):
            m.train();perm=rng.permutation(len(inputs))
            for start in range(0,len(inputs),512):
                ix=perm[start:start+512];opt.zero_grad();pred=m(inputs[ix])*gate[ix]
                loss=(torch.nn.functional.smooth_l1_loss(pred,labels[ix],reduction='none').mean(axis=1)*weight[ix]).mean();loss.backward();torch.nn.utils.clip_grad_norm_(m.parameters(),5.);opt.step()
            pred=predict_residual([m],data['x'][val],mean,scale)*eligible[val,None];score=equal_episode(np.linalg.norm(target[val]-pred,axis=1),data['ids'][val]);curve.append({'epoch':epoch,'validation_episode_error':score})
            if score<best-1e-5:best=score;best_state=copy.deepcopy(m.state_dict());stale=0
            else:stale+=1
            if stale>=12:break
        m.load_state_dict(best_state);m.eval();models.append(m);states.append(best_state);scores.append(best);histories.append(curve);print(json.dumps({'trained_seed':seed,'validation_error':best,'epochs':len(curve)}),flush=True)
    ensemble_score=equal_episode(np.linalg.norm(target[val]-predict_residual(models,data['x'][val],mean,scale)*eligible[val,None],axis=1),data['ids'][val])
    chosen=int(np.argmin(scores+[ensemble_score]));selected=list(range(3)) if chosen==3 else [chosen]
    meta={'schema':SCHEMA,'architecture':[64,96,96,2],'window':window,'window_validation_scores':win_scores,'training_seeds':[0,1,2],'validation_candidates':scores+[ensemble_score],'selected_seed_indices':selected,
        'normalization':'train only','weighted_training':'equal total weight per episode','selection':'validation episode-mean Euclidean error only','supported_horizons':supported,
        'dataset_sha256':_sha256(out.parent/'data/dataset.npz'),'learning_rate':.001,'weight_decay':.0001,'epochs_max':100,'patience':12,'gate':'h²/(h²+0.25)','limitations':['Learned position corrections retain smoothed-CA velocity approximation','Single scripted/IDM layout family; no road validation']}
    meta=json.loads(json.dumps(meta));torch.save({'metadata':meta,'states':[states[i] for i in selected],'mean':mean.tolist(),'scale':scale.tolist()},out/'model.pt');dump_json(out/'selection.json',meta);dump_json(out/'training_curves.json',{'seeds':histories})
    pred=base+predict_residual([models[i] for i in selected],data['x'],mean,scale)*eligible[:,None]
    # Scales use validation only; scalar max-normalized joint quantile uses calibration only.
    calibration={'model_sha256':_sha256(out/'model.pt'),'window':window,'edges':EDGES.tolist(),'fit_split':'calibration','scale_split':'validation','target':'pointwise joint x/y rectangle empirical .95; not trajectory or safety coverage'}
    buckets=np.clip(np.searchsorted(EDGES[1:],data['h'],side='left'),0,len(EDGES)-2)
    for name,positions in [('fit',base),('nn',pred)]:
        error=np.abs(data['y']-positions);rects=[];details=[]
        for b in range(len(EDGES)-1):
            vm=val&(buckets==b);cm=cal&(buckets==b);sc=np.maximum(np.quantile(error[vm if vm.any() else val],.75,axis=0),.05)
            use=cm if cm.sum()>=40 else cal;score=np.max(error[use]/sc,axis=1);q=float(np.quantile(score,.95,method='higher'));rects.append((sc*q).tolist());details.append({'count':int(cm.sum()),'scale':sc.tolist(),'joint_factor':q,'fallback_global':bool(cm.sum()<40)})
        calibration[name]={'rectangles':rects,'buckets':details}
    dump_json(out/'calibration.json',calibration)
    return base,pred,calibration

def main():
    out=ROOT/'runs/model_optimization_v4';out.mkdir(exist_ok=False);(out/'data').mkdir();(out/'models').mkdir();start=time.perf_counter();source=source_manifest()
    dump_json(out/'contract.json',{'episodes_per_family':40,'families':['follow','brake','cutin','traffic'],'episode_splits':[24,6,4,6],'collection_seed_start':2000000,'seed_family_stride':10000,
       'new_model':'64 history features; 96x96; seeds0,1,2 or average selected by validation','analytic_candidates':list(WINDOWS),'success_offline':'>=10% smaller error than frozen old NN and >=5% smaller than fitted CA; judged on new test only',
       'calibration':'validation scales plus independent-calibration .95 max-normalized joint error rectangles','test_policy':'no retuning after test; report common horizons<=2.4 separately','success_online':'No more collisions than fit; progress loss <=1%; jerk increase <=10%; additionally report practical driving benefit separately',
       'closed_loop':'new seeds 3000000..3000002, all four scenarios, delays .2/.4, fixed/bursty, 5 methods =240 episodes; controllers frozen','provenance':source})
    with zipfile.ZipFile(out/'source.zip','w',zipfile.ZIP_DEFLATED) as z:
        for f in source['files']:z.write(ROOT/f,f)
        for f in (ROOT/'tests').glob('test*.py'):z.write(f,str(f.relative_to(ROOT)))
    data=collect(out/'data');print('data complete',flush=True)
    # Refit old architecture on the identical new episode split as a data-only ablation.
    original=train_model(out/'data/compat/dataset.npz',out/'models/v1_retrained',80,0)
    calibrate_model(out/'data/compat/dataset.npz',out/'models/v1_retrained/model.pt',out/'models/v1_retrained/calibration.json')
    base,pred,cal=train(data,out/'models');retrained=LearnedPredictor(out/'models/v1_retrained/model.pt')
    retrained_residual=retrained.residuals(data['old_x']);lo,hi=retrained.supported_horizons;retrained_residual[(data['h']<lo-1e-6)|(data['h']>hi+1e-6)]=0
    methods={'ca':data['raw'],'fitted_ca':base,'old_frozen':data['old'],'old_architecture_retrained':data['raw']+retrained_residual,'optimized':pred}
    test=data['split']=='test';stats={};detail={};buckets=np.clip(np.searchsorted(EDGES[1:],data['h'],side='left'),0,len(EDGES)-2)
    for name,positions in methods.items():
        err=np.linalg.norm(data['y']-positions,axis=1);values=[]
        for e in np.unique(data['ids'][test]):values.append(float(err[test&(data['ids']==e)].mean()))
        stats[name]={'sample_error_m':float(err[test].mean()),'episode_error_m':float(np.mean(values)),'episodes':len(values),'samples':int(test.sum()),
          'common_horizon_episode_error_m':equal_episode(err[test&(data['h']<=2.4+1e-6)],data['ids'][test&(data['h']<=2.4+1e-6)]),
          'per_family':{scene:equal_episode(err[test&(data['scenario']==scene)],data['ids'][test&(data['scenario']==scene)]) for scene in ['follow','brake','cutin','traffic']}}
        detail[name]=values
        if name in ['fitted_ca','optimized']:
            rect=np.array(cal['fit' if name=='fitted_ca' else 'nn']['rectangles'])[buckets];inside=(np.abs(data['y']-positions)<=rect).all(axis=1)
            stats[name]['test_joint_point_coverage']=float(inside[test].mean());stats[name]['test_episode_equal_joint_coverage']=equal_episode(inside[test],data['ids'][test])
    rng=np.random.default_rng(20261024);contrasts={}
    for ref in ['old_frozen','old_architecture_retrained','fitted_ca']:
        dif=np.array(detail['optimized'])-np.array(detail[ref]);boot=rng.choice(dif,(2000,len(dif)),replace=True).mean(axis=1)
        contrasts[ref]={'optimized_minus_reference_m':float(dif.mean()),'episode_bootstrap95':np.quantile(boot,[.025,.975]).tolist()}
    dump_json(out/'offline_results.json',{'methods':stats,'contrasts':contrasts,'test_used_for_selection':False})
    print(json.dumps(stats,ensure_ascii=False),flush=True)
    np.savez_compressed(out/'test_errors.npz',episode_ids=data['ids'][test],horizons=data['h'][test],scenarios=data['scenario'][test],
       **{name:(data['y']-p)[test] for name,p in methods.items()})
    jobs=list(itertools.product(['follow','brake','cutin','traffic'],[.2,.4],['fixed','bursty'],range(3000000,3000003),['ca_recovery','ml_recovery','ml_retrained_recovery','fit_recovery','ml4_recovery']))
    (out/'closed_loop').mkdir()
    with (out/'closed_loop/episodes.csv').open('w',newline='') as f:
        writer=None
        for i,(scene,delay,mode,seed,method) in enumerate(jobs):
            if method in ['ca_recovery','ml_recovery']:model=ROOT/'runs/release_v1/model/model.pt';cali=ROOT/'runs/release_v1/model/calibration.json'
            elif method=='ml_retrained_recovery':model=out/'models/v1_retrained/model.pt';cali=out/'models/v1_retrained/calibration.json'
            else:model=out/'models/model.pt';cali=out/'models/calibration.json'
            result=run_episode(EpisodeConfig(scenario=scene,seed=seed,profile='holdout',delay=delay,delay_mode=mode,position_noise=.1,velocity_noise=.1),method,out/'closed_loop'/f'episode_{i:04d}',model_path=model,calibration_path=cali)
            if writer is None:writer=csv.DictWriter(f,fieldnames=list(result));writer.writeheader()
            writer.writerow(result);f.flush()
            if (i+1)%10==0:print(json.dumps({'closed_loop':i+1,'total':len(jobs)}),flush=True)
    assert source_manifest()['source_sha256']==source['source_sha256'],'Source changed during optimization'
    dump_json(out/'completion.json',{'status':'completed','closed_loop_episodes':len(jobs),'wall_seconds':time.perf_counter()-start,'source_sha256':source['source_sha256']})
if __name__=='__main__':main()

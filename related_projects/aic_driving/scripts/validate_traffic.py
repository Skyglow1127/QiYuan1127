#!/usr/bin/env python3
"""Bounded new-domain validation of frozen controllers in four-actor IDM/MOBIL traffic."""
import csv,itertools,json,time,zipfile
from pathlib import Path
import numpy as np
from aic_drive.runner import ROOT,run_episode,source_manifest,dump_json
from aic_drive.types import EpisodeConfig

def main():
 out=ROOT/'runs/traffic_20261024';out.mkdir(exist_ok=False);source=source_manifest()
 methods=['ca_adaptive','ca_recovery','ml_recovery']
 jobs=list(itertools.product([.2,.4],['fixed','bursty'],range(1210000,1210008),methods))
 dump_json(out/'manifest.json',{'role':'new-domain validation only; no tuning on results','actors':4,
   'actor_model':'highway-env IDM/MOBIL, forward-only speed clamp, maximum |acceleration|6m/s²',
   'planned_episodes':len(jobs),'delays':[.2,.4],'modes':['fixed','bursty'],
   'seeds':list(range(1210000,1210008)),'methods':methods,'duration':30,'model':'release_v1/model/model.pt unchanged','provenance':source})
 with zipfile.ZipFile(out/'source.zip','w',zipfile.ZIP_DEFLATED) as z:
  for f in source['files']:z.write(ROOT/f,f)
  for f in (ROOT/'tests').glob('test*.py'):z.write(f,str(f.relative_to(ROOT)))
 start=time.perf_counter();results=[]
 with (out/'episodes.csv').open('w',newline='') as f:
  writer=None
  for i,(delay,mode,seed,method) in enumerate(jobs):
   result=run_episode(EpisodeConfig(scenario='traffic',seed=seed,delay=delay,delay_mode=mode,
      position_noise=.1,velocity_noise=.1),method,out/f'episode_{i:04d}',
      model_path=ROOT/'runs/release_v1/model/model.pt',calibration_path=ROOT/'runs/release_v1/model/calibration.json')
   result['target_lane_crossings']=0;last={}
   for line in (Path(result['run_dir'])/'trajectory.jsonl').read_text().splitlines():
    for t in json.loads(line)['targets']:
     lane=int(t['y']>=2.)
     if t['track_id'] in last and last[t['track_id']]!=lane:result['target_lane_crossings']+=1
     last[t['track_id']]=lane
   if writer is None:writer=csv.DictWriter(f,fieldnames=list(result));writer.writeheader()
   writer.writerow(result);f.flush();results.append(result)
   if (i+1)%6==0:print(json.dumps({'completed':i+1,'total':len(jobs),'last_collision':result['collision']}),flush=True)
 summary={}
 for method in methods:
  rs=[r for r in results if r['method']==method]
  summary[method]={'episodes':len(rs),'collisions':sum(r['collision'] for r in rs),'offroad':sum(r['offroad'] for r in rs),
    'actor_lane_crossings':sum(r['target_lane_crossings'] for r in rs),
    **{k:float(np.mean([r[k] for r in rs])) for k in ['progress_m','jerk_rms','decision_p95_ms']}}
 contrasts=[]
 for left,right in [('ca_recovery','ca_adaptive'),('ml_recovery','ca_recovery')]:
  blocks=[]
  for seed in range(1210000,1210008):
   def values(m):return np.array([[r['progress_m'],r['jerk_rms']] for r in results if r['seed']==seed and r['method']==m])
   blocks.append((values(left)-values(right)).mean(axis=0))
  d=np.array(blocks);rng=np.random.default_rng(20261024);boot=d[rng.integers(8,size=(2000,8))].mean(axis=1)
  contrasts.append({'left':left,'right':right,'seed_blocks':8,'mean_difference_progress_jerk':d.mean(axis=0).tolist(),
    'bootstrap95_ci':np.quantile(boot,[.025,.975],axis=0).T.tolist()})
 assert source_manifest()['source_sha256']==source['source_sha256'],'Source changed while testing'
 dump_json(out/'results.json',{'methods':summary,'contrasts':contrasts})
 dump_json(out/'completion.json',{'status':'completed','episodes':len(results),'wall_seconds':time.perf_counter()-start,'source_sha256':source['source_sha256']})
 print(json.dumps(summary,ensure_ascii=False),flush=True)
if __name__=='__main__':main()

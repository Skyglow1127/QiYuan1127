#!/usr/bin/env python3
"""Paired evaluation with a frozen manifest and incremental CSV in a fresh directory."""
import argparse,csv,itertools,json
from pathlib import Path
from aic_drive.types import EpisodeConfig
from aic_drive.runner import run_episode,make_controller,source_manifest,dump_json

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',required=True);p.add_argument('--seeds',type=int,default=5);p.add_argument('--seed-start',type=int,default=10000)
    p.add_argument('--scenarios',nargs='+',default=['follow','brake','cutin']);p.add_argument('--methods',nargs='+',default=['ca_fixed','ca_adaptive'])
    p.add_argument('--delays',nargs='+',type=float,default=[0,.2,.4]);p.add_argument('--delay-modes',nargs='+',default=['fixed'])
    p.add_argument('--profile',choices=['development','holdout'],default='development');p.add_argument('--duration',type=float,default=30)
    p.add_argument('--model');p.add_argument('--calibration');p.add_argument('--position-noise',type=float,default=0.);p.add_argument('--velocity-noise',type=float,default=0.)
    p.add_argument('--fixed-margin',type=float,default=2.,help='Longitudinal fixed margin in metres, shared by fixed methods')
    a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    jobs=list(itertools.product(a.scenarios,a.delays,a.delay_modes,range(a.seed_start,a.seed_start+a.seeds),a.methods))
    dump_json(out/'manifest.json',{'arguments':vars(a),'planned_episodes':len(jobs),'provenance':source_manifest(),
          'label':'development evaluation' if a.profile=='development' else 'held-out scenario stress evaluation'})
    with (out/'episodes.csv').open('w',newline='') as f:
        writer=None
        for i,(scenario,delay,mode,seed,method) in enumerate(jobs):
            c=EpisodeConfig(scenario=scenario,seed=seed,delay=delay,delay_mode=mode,profile=a.profile,duration=a.duration,
                            position_noise=a.position_noise,velocity_noise=a.velocity_noise)
            controller=make_controller(method,a.model,a.calibration,fixed_margin=a.fixed_margin)
            result=run_episode(c,method,out/f'episode_{i:05d}',controller=controller,model_path=a.model,calibration_path=a.calibration)
            if writer is None:writer=csv.DictWriter(f,fieldnames=list(result));writer.writeheader()
            writer.writerow(result);f.flush()
            if (i+1)%10==0 or i==0 or i+1==len(jobs):
                print(json.dumps({'completed':i+1,'total':len(jobs),'last':{k:result[k] for k in ['scenario','method','collision','progress_m','wall_seconds']}}),flush=True)
    print(str((out/'episodes.csv').resolve()))
    dump_json(out/'completion.json',{'status':'completed','completed_episodes':len(jobs),'planned_episodes':len(jobs)})
if __name__=='__main__':main()

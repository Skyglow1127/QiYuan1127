#!/usr/bin/env python3
import argparse,json
from aic_drive.types import EpisodeConfig
from aic_drive.runner import run_episode

def main():
    p=argparse.ArgumentParser(description='Run a logged driving simulation')
    p.add_argument('--scenario',choices=['follow','brake','cutin','traffic'],default='brake')
    p.add_argument('--method',default='ca_fixed')
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--delay',type=float,default=.2)
    p.add_argument('--delay-mode',choices=['fixed','bursty'],default='fixed')
    p.add_argument('--duration',type=float,default=30)
    p.add_argument('--profile',choices=['development','holdout'],default='development')
    p.add_argument('--position-noise',type=float,default=0.)
    p.add_argument('--velocity-noise',type=float,default=0.)
    p.add_argument('--model');p.add_argument('--calibration');p.add_argument('--output');p.add_argument('--video',action='store_true')
    a=p.parse_args()
    result=run_episode(EpisodeConfig(scenario=a.scenario,seed=a.seed,duration=a.duration,delay=a.delay,delay_mode=a.delay_mode,
       profile=a.profile,position_noise=a.position_noise,velocity_noise=a.velocity_noise),a.method,a.output,model_path=a.model,calibration_path=a.calibration)
    if a.video:
        from aic_drive.visualization import render_replay,export_run_plot
        result['video']=str(render_replay(result['run_dir']))
        result['plot']=str(export_run_plot(result['run_dir']))
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()

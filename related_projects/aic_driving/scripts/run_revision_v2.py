#!/usr/bin/env python3
"""Frozen v2 holdout validation after development; never overwrite existing runs."""
import subprocess,sys,json,zipfile,time
from pathlib import Path
from aic_drive.runner import ROOT,source_manifest,dump_json

def main():
    out=ROOT/'runs/revision_v2';manifest=source_manifest()
    if (out/'test_contract.json').exists():raise FileExistsError('Choose a new revision; v2 test already frozen')
    dump_json(out/'test_contract.json',{'main':{'seeds':10,'start':410000,'episodes':360},
      'stress':{'seeds':5,'start':510000,'episodes':240},
      'methods':['ca_adaptive','ml_adaptive','ca_smooth','ml_smooth'],
      'acceptance':{'collision_no_increase':True,'progress_loss_max_fraction':.01,'jerk_reduction_min_fraction':.30},
      'smooth':{'acceleration_spacing':.25,'preferred_delta_acceleration':.4,'safety_overrides_comfort':True},
      'selection':'preset passed 72 development episodes; no parameter search after heldout tests',
      'model':'release_v1/model/model.pt, unchanged','provenance':manifest})
    with zipfile.ZipFile(out/'source.zip','w',zipfile.ZIP_DEFLATED) as z:
        for f in manifest['files']:z.write(ROOT/f,f)
        for f in (ROOT/'tests').glob('test*.py'):z.write(f,str(f.relative_to(ROOT)))
    common=['--methods','ca_adaptive','ml_adaptive','ca_smooth','ml_smooth','--position-noise','.1','--velocity-noise','.1',
      '--model',str(ROOT/'runs/release_v1/model/model.pt'),'--calibration',str(ROOT/'runs/release_v1/model/calibration.json')]
    jobs=[('main',['--seeds','10','--seed-start','410000']),
      ('stress',['--seeds','5','--seed-start','510000','--profile','holdout','--delays','.2','.4','--delay-modes','fixed','bursty'])]
    start=time.perf_counter();stages=[]
    for name,args in jobs:
        cmd=[sys.executable,str(ROOT/'scripts/evaluate.py'),'--output',str(out/name),*common,*args]
        print(name+' running',flush=True)
        with (out/(name+'.log')).open('w') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,cwd=ROOT)
        stages.append({'stage':name,'command':cmd,'exit_code':r.returncode})
        dump_json(out/'pipeline.json',{'status':'running' if r.returncode==0 else 'failed','stages':stages})
        if r.returncode:raise RuntimeError(name+' failed; see log')
        print(name+' completed',flush=True)
    r=subprocess.run([sys.executable,str(ROOT/'scripts/revision_report.py'),'--root',str(out)],stdout=(out/'analysis.log').open('w'),check=True)
    assert source_manifest()['source_sha256']==manifest['source_sha256'],'Source changed during evaluation'
    dump_json(out/'pipeline.json',{'status':'completed','stages':stages,'wall_seconds':time.perf_counter()-start,'source_sha256':manifest['source_sha256']})
    print('completed',flush=True)
if __name__=='__main__':main()

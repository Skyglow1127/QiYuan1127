#!/usr/bin/env python3
"""Run selected existing tests only: virtual robots, pure arithmetic and wire codecs."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

ROOT=Path(__file__).resolve().parents[1]
SUITES={
 'd1': ('src/d1', ['tests/test_shared_progress.py','tests/test_feedback_tolerance.py'], []),
 'pi05': ('src/pi05/pi05_cloud_inference_v7/cloud_grasp_act_bridge', ['test_closure_sync.py'], []),
 'fpga': ('hardware/robot_rt_50mhz', ['tests/test_player_reference.py'], []),
 'piper': ('src/piper/deployment', ['test_policy_bounds.py','test_target_rate.py','test_executor.py','test_act_chunk_timing.py'], ['src/piper/official_adapter']),
 'wire': ('src/pi05/pi05_cloud_inference_v7/lerobot_lossless_service', ['test_wire_codec.py'], []),
}

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--suite', choices=['core','all',*SUITES],default='core')
 p.add_argument('--report',type=Path,default=ROOT/'.local/offline_checks.json')
 args=p.parse_args()
 selected=['d1','pi05','fpga'] if args.suite=='core' else list(SUITES) if args.suite=='all' else [args.suite]
 results=[]
 for name in selected:
  cwd,tests,imports=SUITES[name]
  env=os.environ.copy()
  env['PYTHONDONTWRITEBYTECODE']='1';env['PYTEST_DISABLE_PLUGIN_AUTOLOAD']='1'
  env['HF_HUB_OFFLINE']='1';env['TRANSFORMERS_OFFLINE']='1'
  env['PYTHONPATH']=os.pathsep.join([str(ROOT/cwd),*[str(ROOT/x) for x in imports],env.get('PYTHONPATH','')])
  cmd=[sys.executable,'-B','-m','pytest','-q','-p','no:cacheprovider',*tests]
  result=subprocess.run(cmd,cwd=ROOT/cwd,env=env,text=True,capture_output=True,timeout=180)
  print(f'[{name}] {result.returncode}\n{result.stdout}{result.stderr}',flush=True)
  results.append({'suite':name,'tests':tests,'returncode':result.returncode,'stdout':result.stdout,'stderr':result.stderr})
 report={'timestamp_utc':datetime.now(timezone.utc).isoformat(),'python':sys.version.split()[0],
         'scope':'Selected offline tests; no hardware, cloud service or physical grasp validation.',
         'passed':all(r['returncode']==0 for r in results),'results':results}
 args.report.parent.mkdir(parents=True,exist_ok=True)
 args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 return 0 if report['passed'] else 1
if __name__=='__main__':sys.exit(main())

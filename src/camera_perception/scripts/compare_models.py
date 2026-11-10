"""Sequential warm benchmarks on the same RGB-D snapshot; NOT an accuracy ranking."""
import argparse
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--replay',type=Path,default=ROOT/'output/benchmark_fixture/last_session.yml')
    p.add_argument('--config',type=Path,default=ROOT/'grasp_config.yml')
    p.add_argument('--iterations',type=int,default=50)
    p.add_argument('--output',type=Path,default=ROOT/'output/model_comparison')
    p.add_argument('--modes',nargs='+',choices=['segment','fusion','rtdetr','hybrid','tabletop'],
                   default=['segment','fusion','rtdetr','hybrid'])
    a=p.parse_args()
    reports=[]
    for mode in a.modes:
        directory=a.output.resolve()/mode
        subprocess.run([str(ROOT/'build/detect'),'--mode',mode,'--replay',str(a.replay.resolve()),
                        '--config',str(a.config.resolve()),
                        '--headless','--benchmark',str(a.iterations),'--output',str(directory)],check=True)
        report=json.loads((directory/'benchmark.json').read_text())
        result=json.loads((directory/'replay_result.json').read_text())
        report['detections']=[{'class':c['class'],'score':c['confidence'],'source':c['source'],
                              'verification':c['verification'],'status':c['status']} for c in result['candidates']]
        reports.append(report)
    document={'created_unix':time.time(),'platform':platform.platform(),'snapshot':str(a.replay.resolve()),
              'snapshot_metadata_sha256':hashlib.sha256(a.replay.read_bytes()).hexdigest(),
              'config':str(a.config.resolve()),'config_sha256':hashlib.sha256(a.config.read_bytes()).hexdigest(),
              'scope':'same snapshot, sequential CPU timing; no ground truth, not AP or grasp-success evidence',
              'results':reports}
    a.output.mkdir(parents=True,exist_ok=True)
    (a.output/'comparison.json').write_text(json.dumps(document,indent=2)+'\n')
    print(json.dumps(document,indent=2))

if __name__=='__main__':
    main()

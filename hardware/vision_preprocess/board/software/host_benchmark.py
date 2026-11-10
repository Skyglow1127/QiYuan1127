#!/usr/bin/env python3
"""Measure actual board-to-host output delivery for both paths, with no frame upload."""
import argparse,hashlib,json,urllib.parse,urllib.request
from pathlib import Path
from time import perf_counter_ns
import numpy as np

def main():
    p=argparse.ArgumentParser();p.add_argument('--url',default='http://127.0.0.1:18765')
    p.add_argument('--output',type=Path,required=True);p.add_argument('--repeats',type=int,default=20);a=p.parse_args()
    with urllib.request.urlopen(a.url+'/status',timeout=5) as r:status=json.loads(r.read())
    report={'route':'board-local frame -> software or DMA/PL -> raw RGB output -> existing SSH connection -> host',
            'extra_host_to_board_frame_upload':False,'samples':[]}
    for sample in status['samples']:
        for width,height in ((640,480),(320,240),(224,224)):
            digests={}
            for backend in ('software','pl'):
                traces=[]
                for trial in range(a.repeats):
                    url=a.url+'/process?'+urllib.parse.urlencode(dict(sample=sample,width=width,height=height,backend=backend))
                    begin=perf_counter_ns()
                    with urllib.request.urlopen(url,timeout=10) as r:
                        raw=r.read();server_ns=int(r.headers['X-Server-Work-Ns']);stages=json.loads(r.headers['X-VPR-Stages'])
                    owned=np.frombuffer(raw,dtype=np.uint8).copy()
                    end=perf_counter_ns()
                    digest=hashlib.sha256(owned.tobytes()).hexdigest()
                    if trial and digest!=digests[backend]:raise RuntimeError('Frame changed inside frozen-frame benchmark')
                    digests[backend]=digest
                    traces.append({'request_response_and_host_copy_ns':end-begin,'server_work_ns':server_ns,'server_stages':stages,
                        'bytes_received':len(raw)})
                times=[t['request_response_and_host_copy_ns'] for t in traces]
                report['samples'].append({'sample':sample,'output':[width,height],'backend':backend,'sha256':digest,
                    'wall_p50_ms':float(np.median(times)/1e6),'wall_p95_ms':float(np.percentile(times,95)/1e6),'traces':traces})
            if digests['software']!=digests['pl']:raise RuntimeError('Software/PL delivered bytes differ')
    report['status']='PASS';a.output.write_text(json.dumps(report,indent=2)+'\n');print('PASS: '+str(a.output))
if __name__=='__main__':main()

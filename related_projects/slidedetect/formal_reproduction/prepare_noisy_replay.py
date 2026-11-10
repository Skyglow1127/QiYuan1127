"""Connect saved causal-frontend timestamps to the existing RTL arithmetic TB."""
import hashlib,json
from pathlib import Path
import numpy as np
import reproduce as ref

ROOT=Path(__file__).resolve().parent
FOLDER=ROOT/'results/noise_audit/noisy_replay'

def main():
    config=json.loads((ROOT/'noise_audit_protocol.json').read_text())
    report=json.loads((ROOT/'results/noise_audit/causal_frontend.json').read_text())
    assert all(q['cycle_discontinuities_after_guard']==0 for c in report['cases'] for q in c['quality'])
    paths=[FOLDER/(c['name']+'_ticks.txt') for c in config['frontend']['cases']]
    streams=[np.loadtxt(p,dtype=np.int64) for p in paths]
    rows=[]
    for m in [96,208]:
        target=FOLDER/f'noisy_m{m}.txt';checks=0;events=0
        with target.open('w') as f:
            for stream in streams:
                f.write('3 0 0 0 0\n')
                for i,tick in enumerate(stream):
                    if i>=m-1 and (i-m+1)%(m//4)==0:
                        q,s,u=ref.exact(stream[i-m+1:i+1])
                        f.write(f'2 {int(tick):016x} {q:024x} {s:024x} {u:016x}\n');checks+=1
                    else:f.write(f'1 {int(tick):016x} 0 0 0\n')
                    events+=1
        rows.append(dict(m=m,file=target.name,events=events,checks=checks,sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    (FOLDER/'summary.json').write_text(json.dumps(dict(rtl_vectors=rows),indent=2)+'\n')
    binding=dict(scope='Six causal frontend cases feed RTL arithmetic. Analog chain is a software model; asynchronous physical input is not exercised here.',
        streams=[dict(case=p.stem.removesuffix('_ticks'),events=len(v),file=p.name,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p,v in zip(paths,streams)],
        source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in ['prepare_noisy_replay.py','audit_noise.py','noise_audit_protocol.json','reproduce.py']})
    (FOLDER/'binding.json').write_text(json.dumps(binding,indent=2)+'\n')
    print(json.dumps(rows,indent=2))

if __name__=='__main__':main()

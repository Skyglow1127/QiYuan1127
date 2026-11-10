#!/usr/bin/env python3
"""Additional real-valued phase experiments with integer timestamp oracles."""
import json, math
from pathlib import Path
import numpy as np
from reference import ROOT,FC,M,estimate,timestamps

def write_case(fp,ts,stride,meta,profile,case,truth_fn,label,step=256):
    fp.write('3 0 0 0 0\n')
    for i,c in enumerate(ts):
        if i>=M-1 and (i-(M-1))%step==0:
            window=ts[i-M+1:i+1]
            q,s,u=estimate(window,stride)
            center=s/(M*FC);end=float(c)/FC
            fp.write(f'2 {int(c):016x} {q:024x} {s:024x} {u:016x}\n')
            meta.append(dict(profile=profile,case=case,label=label,
                expected_uHz=u,mean_time_s=center,end_time_s=end,
                truth_at_mean_Hz=float(truth_fn(center)),truth_at_end_Hz=float(truth_fn(end))))
        else: fp.write(f'1 {int(c):016x} 0 0 0\n')

def generate():
    out=ROOT/'sim/vectors';meta=[]
    for d in [1,64]:
        p=f'extended{d}'
        with (out/f'{p}.txt').open('w') as fp:
            # Edge times are inverse phase, evaluated in long double.
            N=2*M;k=np.arange(N,dtype=np.longdouble)*d
            alpha=np.longdouble(100);f0=np.longdouble(200000)
            t=2*k/(f0+np.sqrt(f0*f0+2*alpha*k))
            ts=np.ceil(np.longdouble('.49')+t*FC).astype(np.int64)
            write_case(fp,ts,d,meta,p,0,lambda t:200000+100*t,'linear_100Hz_per_s',512)
            # Comparable normalized window length, clearly distinct physical
            # modulation frequencies between the two profiles.
            fm=5. if d==1 else .1;amp=100.
            t=k/f0
            for _ in range(6):
                phase=f0*t+amp/(2*np.pi*fm)*(1-np.cos(2*np.pi*fm*t))
                t-=(phase-k)/(f0+amp*np.sin(2*np.pi*fm*t))
            ts=np.ceil(np.longdouble('.49')+t*FC).astype(np.int64)
            write_case(fp,ts,d,meta,p,1,lambda t:200000+amp*np.sin(2*np.pi*fm*t),f'sine_{fm}Hz_100Hz_amplitude',512)
            # +1 ppm actual clock, estimator still uses nominal FC.
            ts=np.ceil(.49+np.arange(M+512,dtype=np.longdouble)*FC*np.longdouble('1.000001')*d/350000).astype(np.int64)
            write_case(fp,ts,d,meta,p,2,lambda t:350000.,'clock_plus_1ppm',256)
            # Independent near-rational frequencies: exact divisors are
            # adversarial neighbors, not the only grid points.
            fs=list(np.linspace(70000,350000,17))
            fs += [FC/n+offset for n in [286,300,400,500,700,1000,1400] for offset in [-.001,.001]]
            for j,f in enumerate(fs):
                ts=timestamps(f,.23 if j%2 else .77,d,M)
                write_case(fp,ts,d,meta,p,3+j,lambda t,f=f:f,'dense_static',1)
    (out/'extended_metadata.json').write_text(json.dumps(meta,indent=2))
    print('Generated extended tests:',len(meta),'checked outputs')

def report():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=json.loads((ROOT/'sim/vectors/extended_metadata.json').read_text())
    summary=[]
    fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    for col,d in enumerate([1,64]):
        p=f'extended{d}';data=[r for r in rows if r['profile']==p]
        got=np.loadtxt(ROOT/f'build/{p}/results.csv',delimiter=',',skiprows=1,dtype=np.int64,ndmin=2)
        assert len(got)==len(data)
        assert all(int(a[1])==r['expected_uHz'] for a,r in zip(got,data))
        for label in dict.fromkeys(r['label'] for r in data):
            sub=[r for r in data if r['label']==label]
            em=[r['expected_uHz']/1e6-r['truth_at_mean_Hz'] for r in sub]
            ee=[r['expected_uHz']/1e6-r['truth_at_end_Hz'] for r in sub]
            summary.append(dict(profile=p,label=label,outputs=len(sub),
                max_error_vs_mean_Hz=max(map(abs,em)),max_error_vs_end_Hz=max(map(abs,ee))))
        for row,case in enumerate([0,1]):
            sub=[r for r in data if r['case']==case]
            ax=axes[row,col];t=[r['end_time_s'] for r in sub]
            ax.plot(t,[r['truth_at_end_Hz'] for r in sub],label='Input at output time',color='#d66b32')
            ax.plot(t,[r['expected_uHz']/1e6 for r in sub],label='RTL estimate',color='#237d98')
            ax.set_title(f'stride={d}: '+sub[0]['label']);ax.set_xlabel('Newest event time (s)');ax.set_ylabel('Hz');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(ROOT/'reports/dynamic_response.png',dpi=160)
    (ROOT/'reports/extended_summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))

if __name__=='__main__':
    import sys
    generate() if sys.argv[1]=='generate' else report()

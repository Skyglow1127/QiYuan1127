#!/usr/bin/env python3
"""Independent batch LS oracle and reproducible timestamp stimuli.

Direct centered summation is used at every checked output (not the RTL
recurrence). Files are timestamps in ticks; accelerated replay does not model
physical elapsed wall time or the input synchronizer.
"""
from pathlib import Path
import argparse, json, math
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
FC=100_000_000
M=32768
FREQUENCIES=[70000.,100000.,100000.001,123456.789,199999.999,
             200000.,200000.001,333333.333,349999.999,350000.]
PHASES=[.01,.49,.99]

def estimate(ts, stride, clock=FC):
    n=len(ts)
    # Python arbitrary precision; intentionally no recursive formula.
    q=sum((2*j-n+1)*int(c) for j,c in enumerate(ts))
    total=sum(map(int,ts))
    num=clock*stride*(n*(n*n-1)//6)*1_000_000
    rounded=(2*num+q)//(2*q)
    return q,total,rounded

def timestamps(f, phase, stride, count, jitter_ns=0, seed=1234):
    k=np.arange(count,dtype=np.longdouble)
    ideal=np.longdouble(str(phase))+k*np.longdouble(FC*stride)/np.longdouble(str(f))
    if jitter_ns:
        ideal+=np.random.default_rng(seed).normal(0,jitter_ns*1e-9*FC,count)
    return np.ceil(ideal).astype(np.int64)

def generate():
    out=ROOT/'sim/vectors'; out.mkdir(parents=True,exist_ok=True)
    metadata=[]
    for stride in [1,64]:
        name=f'stride{stride}'
        cases=[(f,p,0.) for f in FREQUENCIES for p in PHASES]
        cases += [(70000.,.49,1.),(350000.,.49,1.),(350000.,.49,10.)]
        with (out/f'{name}.txt').open('w') as fp:
            idx=0
            for case,(f,phase,jitter) in enumerate(cases):
                fp.write('3 0 0 0 0\n')
                # 17 windows at offsets of 32 retained events: correlated
                # outputs, NOT 17 independent Monte Carlo trials.
                ts=timestamps(f,phase,stride,M+512,jitter,1234+case)
                for i,c in enumerate(ts):
                    check=i>=M-1 and (i-(M-1))%32==0
                    if check:
                        q,s,u=estimate(ts[i-M+1:i+1],stride)
                        fp.write(f'2 {int(c):016x} {q:024x} {s:024x} {u:016x}\n')
                        metadata.append(dict(profile=name,index=idx,case=case,
                            frequency_Hz=f,phase_ticks=phase,jitter_ns=jitter,
                            expected_uHz=u,error_uHz=u-int(round(f*1e6)),error_Hz=(u-int(round(f*1e6)))/1e6,
                            span_s=float(ts[i]-ts[i-M+1])/FC,
                            observation_end_s=float(ts[i])/FC))
                        idx+=1
                    else: fp.write(f'1 {int(c):016x} 0 0 0\n')
        print(name,len(cases),'cases',idx,'checked outputs')
    (out/'metadata.json').write_text(json.dumps(metadata,indent=2))
    # Faster small-window arithmetic exercise with large offsets and random
    # monotonic periods; verifies initial fill, wrap, reset, and high bits.
    rng=np.random.default_rng(29)
    with (out/'small.txt').open('w') as fp:
        for offset in [0,1<<52,(1<<63)+1000]:
            fp.write('3 0 0 0 0\n'); ts=[]; c=offset
            for i in range(400):
                c+=int(rng.integers(285,1429));ts.append(c)
                if i>=127:
                    q,s,u=estimate(ts[-128:],1)
                    fp.write(f'2 {c:016x} {q:024x} {s:024x} {u:016x}\n')
                else:fp.write(f'1 {c:016x} 0 0 0\n')

def report():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=json.loads((ROOT/'sim/vectors/metadata.json').read_text())
    for r in rows:
        r['error_uHz']=r['expected_uHz']-round(r['frequency_Hz']*1e6)
        r['error_Hz']=r['error_uHz']/1e6
    output=ROOT/'reports';output.mkdir(exist_ok=True)
    summaries=[]
    for profile in ['stride1','stride64']:
        captured=np.loadtxt(ROOT/f'build/{profile}/results.csv',delimiter=',',skiprows=1,
                            dtype=np.int64,ndmin=2)
        expected=[r for r in rows if r['profile']==profile]
        assert len(captured)==len(expected),(profile,len(captured),len(expected))
        assert all(int(a[1])==b['expected_uHz'] for a,b in zip(captured,expected))
        for f in FREQUENCIES:
            subset=[r for r in expected if r['frequency_Hz']==f and r['jitter_ns']==0]
            err=np.array([r['error_Hz'] for r in subset])
            summaries.append(dict(profile=profile,frequency_Hz=f,outputs=len(err),
                max_abs_error_Hz=float(abs(err).max()),rmse_Hz=float(np.sqrt(np.mean(err**2))),
                mean_error_Hz=float(err.mean()),pass_1mHz=bool(max(abs(r['expected_uHz']-round(f*1e6)) for r in subset)<=1000)))
    (output/'accuracy_summary.json').write_text(json.dumps(summaries,indent=2))
    import csv
    with (output/'accuracy_summary.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(summaries[0]));w.writeheader();w.writerows(summaries)
    fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained')
    for ax,profile in zip(axes,['stride1','stride64']):
        data=[r for r in summaries if r['profile']==profile]
        ax.bar(range(len(data)),[d['max_abs_error_Hz']*1000 for d in data],color='#237d98')
        ax.axhline(1,color='#ca4b32',ls='--',label='1 mHz target')
        ax.set_xticks(range(len(data)),[f"{d['frequency_Hz']:.3f}" for d in data],rotation=35,ha='right')
        ax.set_ylabel('Maximum absolute error (mHz)');ax.set_title(profile+' / zero added jitter / phase sweep')
        ax.legend();ax.grid(axis='y',alpha=.25)
    fig.savefig(output/'accuracy.png',dpi=160)
    noise=[]
    for profile in ['stride1','stride64']:
        for j in [1.,10.]:
            data=[r for r in rows if r['profile']==profile and r['jitter_ns']==j]
            if data:noise.append(dict(profile=profile,jitter_ns=j,
                max_abs_error_Hz=max(abs(r['error_Hz']) for r in data),
                note='correlated windows; descriptive only, not a confidence bound'))
    (output/'noise_summary.json').write_text(json.dumps(noise,indent=2))
    pairs=[]
    for profile in ['stride1','stride64']:
        for lo,hi in [(100000.,100000.001),(199999.999,200000.),(200000.,200000.001),(349999.999,350000.)]:
            a=[r for r in rows if r['profile']==profile and r['frequency_Hz']==lo and r['jitter_ns']==0]
            b=[r for r in rows if r['profile']==profile and r['frequency_Hz']==hi and r['jitter_ns']==0]
            delta=[y['expected_uHz']-x['expected_uHz'] for x,y in zip(a,b)]
            pairs.append(dict(profile=profile,lower_Hz=lo,upper_Hz=hi,
                minimum_measured_separation_uHz=min(delta),maximum_measured_separation_uHz=max(delta),
                all_sampled_pairs_between_0p5_and_1p5_mHz=all(500<=v<=1500 for v in delta)))
    (output/'resolution_pairs.json').write_text(json.dumps(pairs,indent=2))
    print(json.dumps(summaries,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['generate','report'])
    args=parser.parse_args()
    generate() if args.action=='generate' else report()

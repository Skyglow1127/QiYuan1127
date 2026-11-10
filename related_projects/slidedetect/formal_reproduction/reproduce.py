#!/usr/bin/env python3
"""Declared-assumption reconstruction of the published SFLS simulation chain.

No author data is synthesized or substituted. Outputs are generated simulations;
unknown analog settings are explicitly retained in paper_config.json.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfilt

ROOT = Path(__file__).resolve().parent
LD = np.longdouble
CFG = json.loads((ROOT/'paper_config.json').read_text())
P = {k:v['value'] for k,v in CFG['paper_values'].items()}
A = CFG['declared_assumptions']
FC = P['clock_hz']
HZ_PER_NT = P['gamma_rad_s_T']/(2*np.pi)*1e-9


def field(t, dynamic=False, drift=0, modulation_phase=0):
    return P['background_nT_fig5'] + drift*t + (P['modulation_nT']*np.sin(2*np.pi*P['modulation_hz']*t+modulation_phase) if dynamic else 0)


def phase_cycles(t, dynamic=False, drift=0, modulation_phase=0):
    base = P['background_nT_fig5']*t + drift*t*t/2
    if dynamic:
        w = 2*np.pi*P['modulation_hz']
        base += P['modulation_nT']*(np.cos(modulation_phase)-np.cos(w*t+modulation_phase))/w
    return base*HZ_PER_NT


def schmitt(t, signal, threshold):
    """Piecewise-linear threshold crossings with stateful +/- hysteresis."""
    pos = np.flatnonzero((signal[:-1]<threshold)&(signal[1:]>=threshold))
    neg = np.flatnonzero((signal[:-1]>-threshold)&(signal[1:]<=-threshold))
    inds = np.concatenate([pos,neg])
    levels = np.concatenate([np.ones(len(pos),dtype=int),-np.ones(len(neg),dtype=int)])
    order = np.argsort(inds)
    inds, levels = inds[order], levels[order]
    state = -1
    edges = []
    for index, level in zip(inds,levels):
        if level == state:
            continue
        state = level
        if level == 1:
            frac = (threshold-signal[index])/(signal[index+1]-signal[index])
            edges.append(t[index]+frac*(t[index+1]-t[index]))
    return np.array(edges)


def analog_record(seed=0,dynamic=False,noisy=True,threshold=None,fs=None,duration=None):
    fs = fs or A['analog_grid_hz']
    duration = duration or A['observation_duration_s']
    threshold = A['comparator_upper_A0'] if threshold is None else threshold
    t = np.arange(int(round(duration*fs))+1,dtype=float)/fs
    signal = np.exp(-t/P['T2_s'])*np.sin(2*np.pi*phase_cycles(t,dynamic))
    if noisy:
        rng = np.random.default_rng(seed)
        burn = int(.02*fs)
        noise = sosfilt(butter(4,A['noise_bandwidth_hz'],fs=fs,output='sos'),rng.normal(size=len(t)+burn))[burn:]
        noise *= (1/np.sqrt(2)*10**(-P['snr_db']/20))/np.std(noise)
        signal += noise
    edges = schmitt(t,signal,threshold)
    cycles = np.rint(phase_cycles(edges,dynamic)).astype(np.int64)
    bad = int(np.sum(np.diff(cycles)!=1))
    ticks = np.ceil(edges*FC).astype(np.int64)
    assert np.all(np.diff(ticks)>0)
    return t, signal, edges, ticks, bad


def direct(ticks, m, hop):
    starts = np.arange(0,len(ticks)-m+1,hop,dtype=int)
    qweight = 2*np.arange(m,dtype=LD)-(m-1)
    freq, mean, first, last = [],[],[],[]
    for start in starts:
        w = ticks[start:start+m].astype(LD)
        q = np.sum(qweight*(w-w[0]))
        freq.append(LD(FC)*(m*(m*m-1)//6)/q)
        mean.append(np.mean(w)/FC)
        first.append(w[0]/FC)
        last.append(w[-1]/FC)
    return dict(start=starts,frequency=np.array(freq,dtype=LD),mean=np.array(mean,dtype=float),
                first=np.array(first,dtype=float),last=np.array(last,dtype=float))


def exact(ticks):
    m = len(ticks)
    q = sum((2*j-m+1)*int(v) for j,v in enumerate(ticks))
    total = sum(map(int,ticks))
    numerator = FC*(m*(m*m-1)//6)*1_000_000
    return q,total,(2*numerator+q)//(2*q)


def exact_checks():
    rng = np.random.default_rng(81)
    checks = 0
    for m in (3,32,64,95,96,128,256,512,1024):
        ticks = [2**62+int(x) for x in np.cumsum(rng.integers(560,3000,4*m))]
        q,s,_ = exact(ticks[:m])
        for i in range(m,len(ticks)):
            old,new=ticks[i-m],ticks[i]
            q=q-2*s+(m+1)*old+(m-1)*new
            s=s-old+new
            rq,rs,_=exact(ticks[i-m+1:i+1])
            assert (q,s)==(rq,rs)
            checks+=1
    return checks


def ideal_dynamic(modulation_phase=0):
    # Exact phase inversion isolates algorithmic smoothing from unknown frontend.
    duration=P['dynamic_duration_s']
    count=int(np.floor(phase_cycles(duration,True,modulation_phase=modulation_phase)))+1
    k=np.arange(count,dtype=LD)
    t=k/(P['background_nT_fig5']*LD(str(HZ_PER_NT)))
    for _ in range(5):
        t-=(phase_cycles(t,True,modulation_phase=modulation_phase)-k)/(field(t,True,modulation_phase=modulation_phase)*HZ_PER_NT)
    phase_residual=float(np.max(abs(phase_cycles(t,True,modulation_phase=modulation_phase)-k)))
    assert phase_residual<1e-9
    ticks=np.ceil(t*FC).astype(np.int64)
    rows,series=[],[]
    paper_rates={32:264,64:132,96:88,128:66,256:33,512:16,1024:8}
    paper_counts={32:128,64:62,96:40,128:29,256:13,512:5,1024:1}
    paper_errors={32:.05,64:.18,96:.38,128:.64,256:1.74,512:1.99,1024:2.00}
    for m in P['dynamic_windows']:
        data=direct(ticks,m,m//4)
        b=data['frequency']/HZ_PER_NT
        mean_truth=field(data['mean'],True,modulation_phase=modulation_phase)
        end_truth=field(data['last'],True,modulation_phase=modulation_phase)
        midpoint=(data['first']+data['last'])/2
        row=dict(m=m,hop=m//4,outputs=len(b),
                 update_hz_nominal=4*P['background_nT_fig5']*HZ_PER_NT/m,
                 span_ms=float(np.mean(data['last']-data['first'])*1000),
                 data_age_ms=float(np.mean(data['last']-data['mean'])*1000),
                 peak_mean_time_error_nT=float(np.max(abs(b-mean_truth))),
                 peak_end_time_error_nT=float(np.max(abs(b-end_truth))),
                 peak_midpoint_time_error_nT=float(np.max(abs(b-field(midpoint,True,modulation_phase=modulation_phase)))),
                 paper_plot_update_hz=paper_rates[m],paper_plot_outputs=paper_counts[m],
                 paper_plot_peak_error_nT=paper_errors[m])
        rows.append(row)
        for i in range(len(b)):
            series.append(dict(m=m,first_s=data['first'][i],last_s=data['last'][i],
                               mean_s=data['mean'][i],estimate_nT=float(b[i]),
                               mean_truth_nT=float(mean_truth[i]),last_truth_nT=float(end_truth[i])))
    return rows,series,phase_residual


def static_sweep():
    windows=A['static_windows']
    values={m:[] for m in windows}
    faults=[]
    for seed in range(A['independent_records']):
        _,_,_,ticks,bad=analog_record(A['seed']+seed)
        faults.append(dict(seed=A['seed']+seed,events=len(ticks),cycle_index_discontinuities=bad))
        # Keep failures in the report. No oracle-based edge removal is performed.
        for m in windows:
            values[m].append(direct(ticks,m,m//2)['frequency']/HZ_PER_NT-P['background_nT_fig5'])
    rows=[]
    for m in windows:
        counts=[len(x) for x in values[m]]
        if len(set(counts))!=1:
            rows.append(dict(m=m,status='event_count_mismatch',counts=counts));continue
        e=np.asarray(values[m],dtype=float)
        temporal_bias=np.mean(e,axis=0)
        var=np.var(e,axis=0,ddof=1)
        # Resample whole independent acquisitions, retaining within-record overlap.
        rng=np.random.default_rng(A['seed']+m)
        samples=e[rng.integers(0,len(e),(500,len(e)))]
        boot_std=np.sqrt(np.mean(np.var(samples,axis=1,ddof=1),axis=1))
        boot_rmse=np.sqrt(np.mean(samples*samples,axis=(1,2)))
        rows.append(dict(m=m,status='reported_with_quality_flags',frames_per_record=e.shape[1],
                         records=e.shape[0],span_ms=(m-1)/(P['background_nT_fig5']*HZ_PER_NT)*1000,
                         rms_across_record_std_nT=float(np.sqrt(np.mean(var))),
                         mean_bias_nT=float(np.mean(e)),
                         rms_bias_nT=float(np.sqrt(np.mean(temporal_bias**2))),
                         total_rmse_nT=float(np.sqrt(np.mean(e**2))),
                         std_bootstrap95_nT=np.quantile(boot_std,[.025,.975]).tolist(),
                         rmse_bootstrap95_nT=np.quantile(boot_rmse,[.025,.975]).tolist()))
    return rows,faults,{str(m):np.asarray(v,dtype=float).tolist() for m,v in values.items()}


def threshold_audit():
    rows=[]
    for h in (0,.02,.05,.1):
        _,_,edges,ticks,bad=analog_record(noisy=False,threshold=h)
        data=direct(ticks,208,104)
        err=data['frequency']/HZ_PER_NT-P['background_nT_fig5']
        rows.append(dict(threshold_A0=h,cycle_index_discontinuities=bad,events=len(edges),
                         mean_bias_nT=float(np.mean(err)),max_abs_bias_nT=float(np.max(abs(err)))))
    convergence=[]
    previous=None
    for fs in (500000,1000000,2000000):
        _,_,edges,_,bad=analog_record(noisy=False,fs=fs)
        difference=None if previous is None else float(np.max(abs(edges-previous))*1e9)
        convergence.append(dict(analog_grid_hz=fs,events=len(edges),
                                max_crossing_change_vs_previous_ns=difference))
        previous=edges
    return rows,convergence


def write_csv(path,rows):
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)


def make_vectors(out,ticks):
    records=[]
    for m in P['dynamic_windows']+[95]:
        target=out/f'vectors_m{m}.txt'
        checks=0;events=0
        rng=np.random.default_rng(m)
        # Analog input plus two high-offset ring-wrap/reset arithmetic stress cases.
        streams=[ticks,np.cumsum(rng.integers(565,1429,4*m)).astype(object)+2**62,
                 np.cumsum(rng.integers(565,1429,4*m)).astype(object)+2**63]
        with target.open('w') as fp:
            for stream in streams:
                fp.write('3 0 0 0 0\n')
                for i,c in enumerate(stream):
                    if i>=m-1 and (i-m+1)%max(1,m//4)==0:
                        q,s,u=exact(stream[i-m+1:i+1])
                        fp.write(f'2 {int(c):016x} {q:024x} {s:024x} {u:016x}\n');checks+=1
                    else: fp.write(f'1 {int(c):016x} 0 0 0\n')
                    events+=1
        records.append(dict(m=m,events=events,checks=checks,file=target.name,
                            sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    return records


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=ROOT/'results')
    args=p.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    checks=exact_checks()
    dynamic,series,residual=ideal_dynamic()
    # Prespecified complete phase grid, preserving the main phase-zero result.
    phase_scan=[]
    for phi in np.linspace(0,2*np.pi,64,endpoint=False):
        phase_rows,_,_=ideal_dynamic(float(phi))
        phase_scan.append(dict(modulation_phase_rad=float(phi),
                               m1024_peak_mean_time_error_nT=phase_rows[-1]['peak_mean_time_error_nT'],
                               outputs=phase_rows[-1]['outputs']))
    static,quality,raw=static_sweep()
    threshold,convergence=threshold_audit()
    t,signal,edges,ticks,bad=analog_record(A['seed'],True,duration=1)
    # Example waveform and actual state transitions, for audit without huge caches.
    window=(t>=.1)&(t<=.102)
    write_csv(out/'frontend_excerpt.csv',[dict(time_s=x,signal=y) for x,y in zip(t[window],signal[window])])
    write_csv(out/'frontend_edges.csv',[dict(edge_s=float(x),tick=int(y)) for x,y in zip(edges,ticks)])
    write_csv(out/'dynamic_summary.csv',dynamic)
    write_csv(out/'dynamic_series.csv',series)
    write_csv(out/'dynamic_phase_sensitivity.csv',phase_scan)
    (out/'static_errors.json').write_text(json.dumps(static,indent=2)+'\n')
    (out/'static_replicates.json').write_text(json.dumps(raw)+'\n')
    report=dict(scope='Published settings plus declared analog assumptions; no claim of author-figure numeric equivalence',
                config_sha256=hashlib.sha256((ROOT/'paper_config.json').read_bytes()).hexdigest(),
                hz_per_nT=HZ_PER_NT,exact_recurrence_checks=checks,
                inverse_phase_residual_cycles=residual,dynamic=dynamic,static=static,
                m1024_phase_sensitivity=phase_scan,
                quality_by_record=quality,threshold_sensitivity=threshold,
                noise_free_grid_convergence=convergence,
                rtl_vectors=make_vectors(out,ticks))
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(exact_checks=checks,static_records=len(quality),
                         bad_records=sum(x['cycle_index_discontinuities']>0 for x in quality),
                         dynamic_windows=len(dynamic),rtl_profiles=len(report['rtl_vectors'])),indent=2))


if __name__=='__main__':main()

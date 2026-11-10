"""Independent SFLS theory, noise, drift, equal-window EP and step diagnostics."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfilt
from scipy.stats import chi2

import reproduce as ref

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'results/simulation_completion'
CFG = json.loads((ROOT / 'simulation_protocol.json').read_text())
LD = np.longdouble
F0 = ref.P['background_nT_fig5'] * ref.HZ_PER_NT
FC = ref.FC


def write(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2) + '\n')


def theory():
    rows = []
    cfg = CFG['theory']
    n, sigma = cfg['trials'], cfg['timing_jitter_s']
    for m in cfg['windows']:
        rng = np.random.default_rng(CFG['seed'] + m)
        j = np.arange(m, dtype=float)
        x = j - (m - 1) / 2
        sxx = float(x @ x)
        noise = rng.normal(0, sigma, (n, m))
        # Direct slope fits, independent of the integer recurrence.
        sf = 1 / (1 / F0 + noise @ x / sxx)
        ep = (m - 1) / ((m - 1) / F0 + noise[:, -1] - noise[:, 0])
        pred_sf = F0**2 * sigma / np.sqrt(sxx)
        pred_ep = F0**2 * sigma * np.sqrt(2) / (m - 1)
        got_sf, got_ep = np.std(sf, ddof=1), np.std(ep, ddof=1)
        ci = np.sqrt((n - 1) * got_sf**2 / chi2.ppf([.9995,.0005],n-1))
        ok = max(abs(got_sf/pred_sf-1),abs(got_ep/pred_ep-1)) < cfg['relative_std_tolerance']
        rows.append(dict(m=m,span_s=(m-1)/F0,sfls_std_hz=float(got_sf),ep_std_hz=float(got_ep),
                         theory_sfls_std_hz=float(pred_sf),theory_ep_std_hz=float(pred_ep),
                         sfls_std_ci999_hz=ci.tolist(),ep_sfls_std_ratio=float(got_ep/got_sf),passed=bool(ok)))
    slope = float(np.polyfit(np.log([x['span_s'] for x in rows]),np.log([x['sfls_std_hz'] for x in rows]),1)[0])
    passed = all(r['passed'] for r in rows) and abs(slope+1.5)<cfg['slope_tolerance']
    write('theory.json',dict(rows=rows,trials=n,slope=slope,passed=passed,
                            scope='Independent Gaussian timing errors, no quantization or FID decay'))
    assert passed, 'Independent theory acceptance failed'
    return dict(passed=passed,slope=slope,profiles=len(rows))


def analog(seed,snr_db,threshold,bandwidth,drift=0):
    cfg=CFG['frontend'];fs=cfg['grid_hz']
    t=np.arange(int(fs*cfg['duration_s'])+1)/fs
    phases=ref.HZ_PER_NT*(ref.P['background_nT_fig5']*t+.5*drift*t*t)
    signal=np.exp(-t/ref.P['T2_s'])*np.sin(2*np.pi*phases)
    rng=np.random.default_rng(seed);burn=int(.02*fs)
    noise=sosfilt(butter(4,bandwidth,fs=fs,output='sos'),rng.normal(size=len(t)+burn))[burn:]
    noise*=10**(-snr_db/20)/np.sqrt(2)/np.std(noise)
    edges=ref.schmitt(t,signal+noise,threshold)
    cycles=np.rint(ref.HZ_PER_NT*(ref.P['background_nT_fig5']*edges+.5*drift*edges**2)).astype(np.int64)
    faults=int(np.count_nonzero(np.diff(cycles)!=1))
    return np.ceil(edges*FC).astype(np.int64),faults


def stats(records,seed):
    counts=[len(v) for v in records]
    values=np.concatenate(records)
    result=dict(records=len(records),counts=counts,mean_bias_nT=float(np.mean(values)),
                rmse_nT=float(np.sqrt(np.mean(values**2))),aligned_record_std_nT=None,std_ci95_nT=None)
    if len(set(counts))==1:
        e=np.asarray(records,dtype=float)
        result['aligned_record_std_nT']=float(np.sqrt(np.mean(np.var(e,axis=0,ddof=1))))
        rng=np.random.default_rng(seed)
        boot=e[rng.integers(0,len(e),(CFG['frontend']['bootstrap_repeats'],len(e)))]
        result['std_ci95_nT']=np.quantile(np.sqrt(np.mean(np.var(boot,axis=1,ddof=1),axis=1)),[.025,.975]).tolist()
    return result


def frontend():
    cfg=CFG['frontend'];cases=[];raw={}
    configs=[(s,h,cfg['main_bandwidth_hz']) for s in cfg['snr_db'] for h in cfg['threshold_A0']]
    configs += [(35,.05,b) for b in cfg['extra_bandwidth_hz']]
    for snr,h,bw in configs:
        label=f'snr{snr}_threshold{h:g}_bw{bw}'
        errors={m:dict(sfls=[],ep=[]) for m in cfg['windows']};quality=[]
        for trial in range(cfg['records']):
            ticks,bad=analog(CFG['seed']+trial,snr,h,bw)
            quality.append(dict(seed=CFG['seed']+trial,events=len(ticks),cycle_discontinuities=bad))
            for m in cfg['windows']:
                data=ref.direct(ticks,m,m//2)
                ep=FC*(m-1)/(ticks[data['start']+m-1]-ticks[data['start']])
                errors[m]['sfls'].append(np.asarray(data['frequency']/ref.HZ_PER_NT-ref.P['background_nT_fig5'],dtype=float))
                errors[m]['ep'].append(ep/ref.HZ_PER_NT-ref.P['background_nT_fig5'])
        rows=[]
        for m in cfg['windows']:
            rows.append(dict(m=m,span_ms=(m-1)/F0*1000,
                             sfls=stats(errors[m]['sfls'],CFG['seed']+m),
                             ep=stats(errors[m]['ep'],CFG['seed']+m)))
        raw[label]={str(m):{key:[x.tolist() for x in val] for key,val in pair.items()} for m,pair in errors.items()}
        cases.append(dict(label=label,snr_db=snr,threshold_A0=h,bandwidth_hz=bw,quality=quality,rows=rows))
        print('Frontend completed:',label,flush=True)
    write('frontend.json',dict(cases=cases,scope='Exploratory assumptions; not author parameter identification'))
    write('frontend_raw.json',raw)
    return dict(cases=len(cases),records_per_case=cfg['records'],
                cycle_discontinuities=sum(q['cycle_discontinuities'] for c in cases for q in c['quality']))


def time_for_cycle(k,rate):
    k=np.asarray(k,dtype=LD);f0=LD(str(F0));fdot=LD(str(rate*ref.HZ_PER_NT))
    return k/f0 if rate==0 else 2*k/(f0+np.sqrt(f0*f0+2*fdot*k))


def drift():
    cfg=CFG['drift'];rows=[]
    for rate in cfg['rates_nT_s']:
        n=int(F0+.5*rate*ref.HZ_PER_NT)+1
        t=time_for_cycle(np.arange(n),rate)
        for m in cfg['windows']:
            data=ref.direct(t*FC,m,m//2)
            phase_center=time_for_cycle(data['start']+LD(m-1)/2,rate)
            truth_phase=LD(str(F0))+LD(str(rate*ref.HZ_PER_NT))*phase_center
            truth_mean=LD(str(F0))+LD(str(rate*ref.HZ_PER_NT))*np.asarray(data['mean'],dtype=LD)
            truth_last=LD(str(F0))+LD(str(rate*ref.HZ_PER_NT))*np.asarray(data['last'],dtype=LD)
            bias=(data['frequency']-truth_phase)/ref.HZ_PER_NT
            predicted=-(LD(str(rate*ref.HZ_PER_NT))**2)*(3*m*m-7)/(40*truth_phase**3)/ref.HZ_PER_NT
            rows.append(dict(rate_nT_s=rate,m=m,span_ms=float(np.mean(data['last']-data['first'])*1000),
                phase_center_mean_bias_nT=float(np.mean(bias)),eq20_mean_bias_nT=float(np.mean(predicted)),
                phase_center_vs_theory_max_nT=float(np.max(abs(bias-predicted))),
                mean_event_rmse_nT=float(np.sqrt(np.mean(((data['frequency']-truth_mean)/ref.HZ_PER_NT)**2))),
                newest_event_rmse_nT=float(np.sqrt(np.mean(((data['frequency']-truth_last)/ref.HZ_PER_NT)**2)))))
    maps=[];raw={};quality=[];rate=cfg['map_rate_nT_s']
    for m in cfg['windows']:raw[m]=dict(center=[],latest=[],times=None)
    for trial in range(cfg['noise_records']):
        ticks,bad=analog(CFG['seed']+trial,35,.05,10000,drift=rate)
        quality.append(dict(seed=CFG['seed']+trial,cycle_discontinuities=bad))
        for m in cfg['windows']:
            data=ref.direct(ticks,m,m//2)
            estimates=np.asarray(data['frequency']/ref.HZ_PER_NT,dtype=float)
            raw[m]['center'].append(estimates-(ref.P['background_nT_fig5']+rate*data['mean']))
            raw[m]['latest'].append(estimates-(ref.P['background_nT_fig5']+rate*data['last']))
            raw[m]['times']=data['mean']
    for m,v in raw.items():
        assert len(set(len(x) for x in v['center']))==1, 'Do not align records with skipped edges'
        ce,le=np.array(v['center']),np.array(v['latest'])
        maps.append(dict(m=m,times_s=v['times'].tolist(),center_rmse_nT=np.sqrt(np.mean(ce**2,axis=0)).tolist(),
                         latest_rmse_nT=np.sqrt(np.mean(le**2,axis=0)).tolist()))
    assert max(x['phase_center_vs_theory_max_nT'] for x in rows)<1e-8
    write('drift.json',dict(ideal_rows=rows,noisy_maps=maps,quality=quality,
                           scope='Declared linear drift 2 nT/s; 35 dB, threshold 0.05 A0, 10 kHz bandwidth; not Fig.8 pixel reproduction'))
    write('drift_raw.json',{str(m):{k:([x.tolist() for x in v] if isinstance(v,list) else v.tolist()) for k,v in pair.items()} for m,pair in raw.items()})
    return dict(theory_bias_passed=True,max_theory_difference_nT=max(x['phase_center_vs_theory_max_nT'] for x in rows))


def wilson(errors,n):
    z=1.959963984540054;p=errors/n;den=1+z*z/n
    center=(p+z*z/(2*n))/den;half=z*np.sqrt(p*(1-p)/n+z*z/(4*n*n))/den
    return [center-half,center+half]


def step():
    cfg=CFG['step'];base=cfg['base_hz'];delta=cfg['delta_hz'];n=cfg['independent_trials_per_level']
    m=int(base*cfg['window_s'])+1;j=np.arange(m,dtype=LD);x=j-(m-1)/2;sxx=np.sum(x*x)
    amplitude_jitter=10**(-35/20)/np.sqrt(2)/(2*np.pi*base)
    conditions=[('quantization_only',0),('independent_20ns',20e-9),('linearized_amplitude_35dB',amplitude_jitter)]
    rows=[];raw={}
    for label,jitter in conditions:
        errors=[]
        for level in (0,1):
            rng=np.random.default_rng(CFG['seed']+level)
            f=LD(str(base))+level*LD(str(delta))
            time=j[None,:]/f+rng.uniform(0,1/FC,(n,1))+rng.normal(0,jitter,(n,m))
            ticks=np.ceil(time*FC)
            freq=FC*sxx/np.sum((ticks-ticks[:,:1])*x,axis=1)
            errors.append(np.array(freq-f,dtype=float))
        a,b=errors;std1,std2=np.std(a,ddof=1),np.std(b,ddof=1)
        failures=int(np.count_nonzero(a>delta/2)+np.count_nonzero(b<=-delta/2))
        delta_mean=delta+np.mean(b)-np.mean(a)
        uncertainty=np.sqrt(std1*std1/n+std2*std2/n)
        rows.append(dict(label=label,timing_jitter_ns=jitter*1e9,m=m,span_ms=(m-1)/base*1000,
                         per_level_trials=n,single_window_std_hz=[float(std1),float(std2)],
                         classification_errors=failures,total_classifications=2*n,
                         error_rate_ci95=wilson(failures,2*n),estimated_mean_step_hz=float(delta_mean),
                         independent_mean_step_ci95_hz=[float(delta_mean-1.96*uncertainty),float(delta_mean+1.96*uncertainty)],
                         independent_records_per_level_for_3sigma=int(np.ceil(9*(std1**2+std2**2)/delta**2))))
        raw[label]=[x.tolist() for x in errors]
    frequencies=base+delta*np.arange(cfg['trace_plateaus'],dtype=LD)
    ends=np.cumsum(frequencies*cfg['plateau_s']);k=np.arange(int(ends[-1]),dtype=LD)
    levels=np.searchsorted(ends,k,side='right');starts=np.r_[LD(0),ends[:-1]]
    times=levels*cfg['plateau_s']+(k-starts[levels])/frequencies[levels]+LD('.37')/FC
    ticks=np.ceil(times*FC);trace=[]
    for now in np.arange(.11,cfg['trace_plateaus']*cfg['plateau_s']-.01,1/cfg['output_hz']):
        end=np.searchsorted(times,now,side='right');w=ticks[end-m:end]
        if len(w)!=m:continue
        estimate=FC*sxx/np.sum((w-w[0])*x)
        trace.append(dict(time_s=float(now),estimate_minus_base_uHz=float((estimate-base)*1e6),
                          truth_minus_base_uHz=float(delta*int(now/cfg['plateau_s'])*1e6)))
    write('step.json',dict(conditions=rows,trace=trace,
                           scope='Independent acquisitions for discrimination; trace is quantization-only and correlated, not instrument data',
                           clock_fractional_error_for_equal_absolute_error=delta/base))
    write('step_raw.json',raw)
    return dict(conditions=len(rows),single_window_resolution_not_assumed=True)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'validation.json').unlink(missing_ok=True)
    (ROOT/'results/reproduction_status.json').unlink(missing_ok=True)
    report=dict(protocol_sha256=hashlib.sha256((ROOT/'simulation_protocol.json').read_bytes()).hexdigest())
    report['theory']=theory();print('Theory passed',flush=True)
    report['frontend']=frontend();report['drift']=drift();report['step']=step()
    report['source_sha256']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ('complete_simulation.py','reproduce.py','paper_config.json')}
    write('validation.json',report);print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()

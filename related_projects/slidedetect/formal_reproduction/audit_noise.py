"""Noise budget, correlated plateau averaging, and causal frontend diagnostics."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.signal import butter, sosfilt, sosfreqz
from scipy.stats import chi2
import reproduce as ref

ROOT=Path(__file__).resolve().parent
CFG=json.loads((ROOT/'noise_audit_protocol.json').read_text())
OUT=ROOT/'results/noise_audit'
LD=np.longdouble
FC=200000000

def write(name,x):
    OUT.mkdir(exist_ok=True,parents=True)
    (OUT/name).write_text(json.dumps(x,indent=2,ensure_ascii=False)+'\n')

def wilson(k,n):
    z=1.959963984540054;p=k/n;den=1+z*z/n
    mid=(p+z*z/(2*n))/den;half=z*np.sqrt(p*(1-p)/n+z*z/(4*n*n))/den
    return [float(mid-half),float(mid+half)]

def budget():
    f=2128.87; rows=[]
    for span,sigma_pT in [(.01,6), (1.,.008)]:
        m=int(round(f*span))+1;sxx=m*(m*m-1)/12
        sf=sigma_pT/1000*ref.HZ_PER_NT
        equivalent=sf*np.sqrt(sxx)/f**2
        rows.append(dict(requested_window_s=span,m=m,actual_span_s=(m-1)/f,
                         paper_std_pT=sigma_pT,frequency_std_uHz=sf*1e6,
                         iid_equivalent_total_jitter_ns=equivalent*1e9,
                         sine_slope_V_s=2*np.pi*f*2.5e-6,
                         equivalent_unfiltered_voltage_rms_pV=2*np.pi*f*2.5e-6*equivalent*1e12))
    result=dict(paper_fig10_endpoints=rows,quantization_uniform_rms_ns=5/np.sqrt(12),
                scope='Reverse budget under iid timing-error linearization only. Actual quantization is correlated; filtered frontend noise is not iid. Voltage conversion is an unfiltered slope diagnostic, not a component specification.',
                important='The 35 dB numerical experiment, Fig.10 bench experiment, and 40 dB Fig.12 comparison are separate conditions.')
    write('budget.json',result);return result

def window_freq(ticks,starts,m):
    # Prefixes are long double to avoid cancellation in long absolute timestamps.
    a=np.asarray(ticks,dtype=LD);a=a-a[:,:1]
    j=np.arange(a.shape[1],dtype=LD)
    sums=np.pad(np.cumsum(a,axis=1),((0,0),(1,0)))
    sums_j=np.pad(np.cumsum(a*j,axis=1),((0,0),(1,0)))
    ends=starts+m
    q=(sums_j[:,ends]-sums_j[:,starts])-(starts+LD(m-1)/2)*(sums[:,ends]-sums[:,starts])
    return LD(FC)*(LD(m)*(m*m-1)/12)/q

def plateau():
    c=CFG['plateau'];m=c['window_events'];h=c['hop_events'];n=c['independent_records']
    f=LD(str(c['base_hz']));delta=LD(str(c['delta_hz']));rows=[];raw={};max_ref=0.
    for duration in c['dwell_seconds']:
        d=LD(str(duration));k=np.arange(int(np.ceil((2*f+delta)*d)),dtype=LD)
        t=np.where(k<f*d,k/f,d+(k-f*d)/(f+delta))
        split=int(np.searchsorted(t,d))
        starts0=np.arange(0,split-m+1,h,dtype=int)
        starts1=np.arange(split,len(t)-m+1,h,dtype=int)
        assert len(starts0) and len(starts1)
        starts=np.r_[starts0,starts1];weights=np.arange(m,dtype=LD)-LD(m-1)/2
        norms=[]
        for ss in (starts0,starts1-split):
            taps=np.zeros(int(ss[-1])+1);taps[ss]=1/len(ss)
            combined=np.convolve(taps,np.asarray(weights/np.sum(weights**2),dtype=float))
            norms.append(float(np.linalg.norm(combined)))
        for sigma_ns in c['jitter_ns']:
            means=[];single=[];rng=np.random.default_rng(CFG['seed'])
            for first in range(0,n,16):
                batch=min(16,n-first)
                phase=rng.uniform(0,1/FC,(batch,1))
                jitter=rng.normal(0,sigma_ns*1e-9,(batch,len(t)))
                ticks=np.ceil((t[None,:]+phase+jitter)*FC)
                assert np.all(np.diff(ticks,axis=1)>0)
                freq=window_freq(ticks,starts,m)
                if first==0:
                    for index in (0,len(starts)-1):
                        w=ticks[0,starts[index]:starts[index]+m]
                        exact=LD(FC)*np.sum(weights**2)/np.sum((w-w[0])*weights)
                        max_ref=max(max_ref,float(abs(exact-freq[0,index])))
                a=np.asarray(freq[:,:len(starts0)]-f,dtype=float)
                b=np.asarray(freq[:,len(starts0):]-(f+delta),dtype=float)
                means.extend(np.column_stack([a.mean(axis=1),b.mean(axis=1)]).tolist())
                single.extend(np.column_stack([a[:,0],b[:,0]]).tolist())
            e=np.array(means);single=np.array(single)
            failures=int(np.count_nonzero(e[:,0]>float(delta)/2)+np.count_nonzero(e[:,1]<=-float(delta)/2))
            diff=float(delta)+e[:,1]-e[:,0]
            # A record shares clock phase across its two plateaus. Use the
            # independent record as the Bernoulli unit for the confidence gate.
            record_failures=int(np.count_nonzero((e[:,0]>float(delta)/2)|(e[:,1]<=-float(delta)/2)))
            ci=wilson(record_failures,n)
            key=f'dwell{duration:g}_jitter{sigma_ns:g}'
            prediction=np.array(norms)*float(f)**2*np.sqrt((sigma_ns*1e-9)**2+(1/FC)**2/12)
            prediction_ok=bool(np.all(abs(e.std(axis=0,ddof=1)/prediction-1)<.20)) if sigma_ns>=20 else None
            # Measure uncertainty from independent full records, never frame count.
            row=dict(key=key,dwell_s=duration,jitter_ns=sigma_ns,records=n,
                     frames_per_plateau=[len(starts0),len(starts1)],
                     window_s=(m-1)/float(f),first_post_step_result_wait_s=float(t[split+m-1]-d),
                     mean_error_uHz=(e.mean(axis=0)*1e6).tolist(),
                     plateau_std_uHz=(e.std(axis=0,ddof=1)*1e6).tolist(),
                     single_frame_std_uHz=(single.std(axis=0,ddof=1)*1e6).tolist(),
                     errors=failures,classifications=2*n,record_failures=record_failures,
                     record_error_ci95=ci,
                     meets_proposed_plateau_gate=ci[1]<.05,
                     measured_step_mean_uHz=float(diff.mean()*1e6),
                     measured_step_std_uHz=float(diff.std(ddof=1)*1e6),
                     mean_step_ci95_uHz=((diff.mean()+np.array([-1,1])*1.96*diff.std(ddof=1)/np.sqrt(n))*1e6).tolist(),
                     overlap_weight_prediction_uHz=(prediction*1e6).tolist(),
                     overlap_prediction_within_20percent=prediction_ok,
                     mean_step_shift_for_differential_clock_ppb={str(ppb):float(float(f)*ppb*1e-9*1e6) for ppb in (.1,.5,1,2,5)})
            rows.append(row);raw[key]={'plateau_error_hz':means,'first_frame_error_hz':single.tolist()}
        print('Plateau done',duration,flush=True)
    assert max_ref<2e-8, ('prefix precision insufficient',max_ref)
    assert all(r['overlap_prediction_within_20percent'] for r in rows if r['jitter_ns']>=20)
    result=dict(rows=rows,independent_direct_max_difference_hz=max_ref,
                interpretation='A plateau mean is an additional operation, not an improvement of the raw 100 ms SFLS output. Waiting time and assumed noise must accompany any pass.')
    write('plateau.json',result);write('plateau_raw.json',raw);return result

def bp_filter(width,fs):
    if width is None:return None
    f0=ref.P['background_nT_fig5']*ref.HZ_PER_NT
    return butter(2,[f0-width/2,f0+width/2],btype='bandpass',fs=fs,output='sos')

def filter_data(sos,x):return x if sos is None else sosfilt(sos,x)

def extract(t,y,threshold,guard):
    edges=ref.schmitt(t,y,threshold)
    retained=edges[edges>=guard]
    cycles=np.rint(ref.phase_cycles(retained)).astype(np.int64)
    return np.ceil(retained*FC).astype(np.int64),dict(total_events=len(edges),retained_events=len(retained),
        discarded_startup_events=int(np.count_nonzero(edges<guard)),
        cycle_discontinuities_after_guard=int(np.count_nonzero(np.diff(cycles)!=1)))

def scheduled(ticks,m,ends):
    stops=np.searchsorted(ticks/FC,ends,side='right')
    starts=stops-m
    assert np.all(starts>=0)
    return np.asarray(window_freq(ticks[None,:],starts,m)[0],dtype=float)

def frontend():
    c=CFG['frontend'];fs=c['grid_hz'];n=int(c['duration_s']*fs)+1;t=np.arange(n)/fs
    clean=np.exp(-t/ref.P['T2_s'])*np.sin(2*np.pi*ref.phase_cycles(t))
    dynamic=np.exp(-t/ref.P['T2_s'])*np.sin(2*np.pi*ref.phase_cycles(t,dynamic=True))
    f0=ref.P['background_nT_fig5']*ref.HZ_PER_NT
    input_sos=butter(4,c['input_noise_lowpass_hz'],fs=fs,output='sos')
    burn=int(c['noise_burn_s']*fs);noise_rms=10**(-c['initial_snr_db']/20)/np.sqrt(2)
    rows=[];raw={}
    for case in c['cases']:
        sos=bp_filter(case['bandwidth_hz'],fs)
        filtered_clean=filter_data(sos,clean)
        ticks_clean,quality_clean=extract(t,filtered_clean,case['threshold_A0'],c['guard_s'])
        targets={m:np.arange(max(.3,c['guard_s']+(m-1)/f0+.02),c['duration_s']-.01,.05) for m in c['windows']}
        base_errors={m:(scheduled(ticks_clean,m,ends)-f0)/ref.HZ_PER_NT for m,ends in targets.items()}
        errors={m:[] for m in c['windows']};quality=[];snrs=[]
        for trial in range(c['independent_records']):
            rng=np.random.default_rng(CFG['seed']+trial)
            noise=sosfilt(input_sos,rng.normal(size=n+burn))[burn:]
            noise*=noise_rms/np.std(noise)
            noise=filter_data(sos,noise)
            if case['snr_location']=='after_bandpass':noise*=noise_rms/np.std(noise[t>=c['guard_s']])
            snrs.append(float(20*np.log10(1/np.sqrt(2)/np.std(noise[t>=c['guard_s']]))))
            ticks,q=extract(t,filtered_clean+noise,case['threshold_A0'],c['guard_s']);quality.append(q)
            if trial==0:
                replay=OUT/'noisy_replay';replay.mkdir(exist_ok=True,parents=True)
                np.savetxt(replay/(case['name']+'_ticks.txt'),ticks,fmt='%d')
            for m,ends in targets.items():errors[m].append(((scheduled(ticks,m,ends)-f0)/ref.HZ_PER_NT).tolist())
        stats=[]
        for m in c['windows']:
            e=np.array(errors[m]);std=np.sqrt(np.mean(np.var(e,axis=0,ddof=1)))
            rng=np.random.default_rng(CFG['seed']+m)
            boot=e[rng.integers(0,len(e),(400,len(e)))]
            intervals=np.quantile(np.sqrt(np.mean(np.var(boot,axis=1,ddof=1),axis=1)),[.025,.975])
            stats.append(dict(m=m,span_ms=(m-1)/f0*1000,frames_per_record=e.shape[1],
                across_record_std_nT=float(std),std_ci95_nT=intervals.tolist(),
                bias_nT=float(e.mean()),rmse_nT=float(np.sqrt(np.mean(e*e))),
                noiseless_rmse_nT=float(np.sqrt(np.mean(base_errors[m]**2))),
                noise_only_rmse_nT=float(np.sqrt(np.mean((e-base_errors[m])**2)))))
        # Deterministic dynamic response: do not subtract the causal filter delay.
        dticks,_=extract(t,filter_data(sos,dynamic),case['threshold_A0'],c['guard_s'])
        dyn=ref.direct(dticks,96,24);est=np.asarray(dyn['frequency']/ref.HZ_PER_NT,dtype=float)
        omega=2*np.pi*10;basis=np.column_stack([np.ones(len(est)),np.sin(omega*dyn['mean']),np.cos(omega*dyn['mean'])])
        fit=np.linalg.lstsq(basis,est,rcond=None)[0]
        response=dict(amplitude_nT=float(np.hypot(fit[1],fit[2])),
                      lag_ms=float(-np.arctan2(fit[2],fit[1])/omega*1000),
                      peak_mean_time_error_nT=float(np.max(abs(est-ref.field(dyn['mean'],True)))),
                      peak_latest_time_error_nT=float(np.max(abs(est-ref.field(dyn['last'],True)))))
        rows.append(dict(**case,postfilter_initial_reference_snr_db=float(np.mean(snrs)),
                         quality=quality,noiseless_quality=quality_clean,statistics=stats,dynamic_M96=response))
        raw[case['name']]=dict(errors_nT={str(m):v for m,v in errors.items()},
                             noiseless_errors_nT={str(m):v.tolist() for m,v in base_errors.items()},
                             scheduled_end_times_s={str(m):v.tolist() for m,v in targets.items()})
        print('Causal frontend done',case['name'],flush=True)
    result=dict(cases=rows,scope='Filter choices, thresholds, SNR location and startup guard are declared engineering assumptions. No author-curve matching claim.')
    write('causal_frontend.json',result);write('causal_frontend_raw.json',raw);return result

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--part',choices=['all','budget','plateau','frontend'],default='all');args=parser.parse_args()
    OUT.mkdir(exist_ok=True,parents=True)
    (OUT/('validation_'+args.part+'.json')).unlink(missing_ok=True)
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in ['audit_noise.py','noise_audit_protocol.json','reproduce.py','paper_config.json']}
    results={}
    for name,fun in [('budget',budget),('plateau',plateau),('frontend',frontend)]:
        if args.part in ('all',name):results[name]=fun()
    assert hashes=={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in hashes}, 'Sources changed during run'
    report=dict(parts=list(results),source_sha256=hashes,
                author_precision_reproduced=False)
    write('validation_'+args.part+'.json',report)

if __name__=='__main__':main()

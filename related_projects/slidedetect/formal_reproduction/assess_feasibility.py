"""Inverse noise budget and held-out, explicitly conditional feasibility tests."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.signal import butter, sosfilt
from scipy.stats import chi2

import reproduce as ref
from audit_noise import window_freq

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'results/feasibility'
CFG = json.loads((ROOT / 'feasibility_protocol.json').read_text())
F0 = ref.P['background_nT_fig5'] * ref.HZ_PER_NT
LD = np.longdouble


def write(name, value):
    OUT.mkdir(exist_ok=True, parents=True)
    (OUT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def sxx(m):
    return m * (m * m - 1) / 12


def targets():
    return json.loads((ROOT / 'results/noise_audit/fig6_digitized.json').read_text())['rows']


def inverse():
    rows = []
    for point in targets():
        requested = point['window_ms'] / 1000
        m = round(F0 * requested) + 1
        sigma = point['STD']['nT'] * ref.HZ_PER_NT * np.sqrt(sxx(m)) / F0**2
        low, high = np.array(point['STD']['pixel_interval_nT']) * ref.HZ_PER_NT * np.sqrt(sxx(m)) / F0**2
        interval_m = [point['STD']['pixel_interval_nT'][0] * ref.HZ_PER_NT * np.sqrt(sxx(m-1)) / F0**2,
                      point['STD']['pixel_interval_nT'][1] * ref.HZ_PER_NT * np.sqrt(sxx(m+1)) / F0**2]
        pred = F0**2 * CFG['inverse']['equivalent_total_iid_jitter_ns'] * 1e-9 / np.sqrt(sxx(m)) / ref.HZ_PER_NT
        rows.append(dict(requested_window_ms=point['window_ms'], m=m, actual_window_ms=(m-1)/F0*1000,
                         digitized_std_nT=point['STD']['nT'], equivalent_total_iid_jitter_ns=sigma*1e9,
                         pixel_only_jitter_interval_ns=[low*1e9, high*1e9],
                         pixel_and_plusminus_one_event_interval_ns=(np.array(interval_m)*1e9).tolist(),
                         slope_equivalent_initial_snr_db=float(20*np.log10(1/(np.sqrt(2)*2*np.pi*F0*sigma))),
                         seven_ns_prediction_nT=pred, seven_ns_inside_pixel_range=bool(low <= 7e-9 <= high)))
    frontend = json.loads((ROOT/'results/noise_audit/causal_frontend.json').read_text())
    case = next(c for c in frontend['cases'] if c['name']=='bp100_h0002')
    old = next(r for r in case['statistics'] if r['m']==208)
    target = next(r for r in rows if r['m']==208)
    ratio = old['across_record_std_nT']/target['digitized_std_nT']
    result = dict(rows=rows, equivalent_iid_jitter_range_ns=[min(r['equivalent_total_iid_jitter_ns'] for r in rows), max(r['equivalent_total_iid_jitter_ns'] for r in rows)],
                  initial_unfiltered_35dB_linearized_jitter_ns=float(10**(-35/20)/np.sqrt(2)/(2*np.pi*F0)*1e9),
                  previous_causal_model_std_ratio=ratio, linear_snr_extrapolation_db=35+20*np.log10(ratio),
                  scope='7 ns describes the observed STD curve under the iid formula. It does not prove physical edge jitter, noise bandwidth, SNR location, threshold, or author settings. The SNR conversion is only a local unfiltered-sine diagnostic.')
    write('inverse_budget.json',result)
    return dict(all_seven_ns_inside_pixel_ranges=all(r['seven_ns_inside_pixel_range'] for r in rows))


def summary(errors, rng):
    e=np.asarray(errors,dtype=float)
    indices=rng.integers(0,len(e),(CFG['frontend']['bootstrap_repeats'],len(e)))
    boot=e[indices]
    std=np.sqrt(np.mean(np.var(e,axis=0,ddof=1)))
    bstd=np.sqrt(np.mean(np.var(boot,axis=1,ddof=1),axis=1))
    brmse=np.sqrt(np.mean(boot*boot,axis=(1,2)))
    return dict(std_nT=float(std),std_ci95_nT=np.quantile(bstd,[.025,.975]).tolist(),
                rmse_nT=float(np.sqrt(np.mean(e*e))),rmse_ci95_nT=np.quantile(brmse,[.025,.975]).tolist(),
                mean_bias_nT=float(e.mean()))


def frontend():
    c=CFG['frontend']; fs=c['grid_hz']; t=np.arange(round(fs*c['duration_s'])+1)/fs
    raw_clean=np.exp(-t/ref.P['T2_s'])*np.sin(2*np.pi*ref.phase_cycles(t))
    sos=butter(c['bandpass_order'],[F0-c['bandpass_width_hz']/2,F0+c['bandpass_width_hz']/2],fs=fs,btype='bandpass',output='sos')
    clean=sosfilt(sos,raw_clean)
    all_clean_edges=ref.schmitt(t,clean,c['threshold_A0'])
    keep=all_clean_edges>=c['guard_s'];clean_edges=all_clean_edges[keep]
    clean_cycles=np.rint(ref.phase_cycles(clean_edges)).astype(int)
    assert np.all(np.diff(clean_cycles)==1)
    # All acquisitions must retain these exact event identities; no bad edge deletion.
    lowpass=butter(4,c['input_noise_lowpass_hz'],fs=fs,output='sos'); burn=round(c['noise_burn_s']*fs)
    windows=sorted(set(round(F0*p['window_ms']/1000)+1 for p in targets()))
    ends={m:np.arange(max(.3,c['guard_s']+(m-1)/F0+.02),c['duration_s']-.01,.05) for m in windows}
    starts={m:np.searchsorted(clean_edges,v,side='right')-m for m,v in ends.items()}
    # Conditioning on a fixed schedule is kept identical between acquisitions.
    clean_freq={m:np.asarray(window_freq((clean_edges*ref.FC)[None,:],ss,m)[0],dtype=float) for m,ss in starts.items()}
    rows=[]; raw={}; calibration=[]
    for snr in c['initial_snr_db']:
        errors={m:[] for m in windows}; rounded={m:[] for m in windows}; projected={m:[] for m in windows}
        edge_errors=[]; analog_errors={m:[] for m in windows}; quality=[]; snrs=[]
        for trial in range(c['independent_records']):
            rng=np.random.default_rng(CFG['seed']+trial)
            noise=sosfilt(lowpass,rng.normal(size=len(t)+burn))[burn:]
            noise*=10**(-snr/20)/np.sqrt(2)/np.std(noise)
            noise=sosfilt(sos,noise)
            snrs.append(float(20*np.log10(1/np.sqrt(2)/np.std(noise[t>=c['guard_s']]))))
            edges=ref.schmitt(t,clean+noise,c['threshold_A0']); edges=edges[edges>=c['guard_s']]
            cycles=np.rint(ref.phase_cycles(edges)).astype(int)
            same=bool(np.array_equal(cycles,clean_cycles))
            faults=int(np.count_nonzero(np.diff(cycles)!=1))
            quality.append(dict(seed=CFG['seed']+trial,events=len(edges),cycle_faults=faults,same_event_ids=same))
            assert same and faults==0,'Report failed quality; never silently delete or reindex events'
            ticks=np.ceil(edges*ref.FC).astype(np.int64)
            # Fixed scheduled windows must select the same events in every record.
            assert all(np.array_equal(np.searchsorted(ticks/ref.FC,ends[m],side='right')-m,starts[m]) for m in windows)
            delta=edges-clean_edges;edge_errors.append(delta)
            for m,ss in starts.items():
                freq=np.asarray(window_freq(ticks[None,:],ss,m)[0],dtype=float)
                analog=np.asarray(window_freq((edges*ref.FC)[None,:],ss,m)[0],dtype=float)
                x=np.arange(m)-(m-1)/2
                linear=np.array([-clean_freq[m][i]**2*np.dot(delta[start:start+m],x)/sxx(m) for i,start in enumerate(ss)])
                errors[m].append(((freq-F0)/ref.HZ_PER_NT).tolist())
                rounded[m].append(((np.floor(freq*1e6+.5)/1e6-F0)/ref.HZ_PER_NT).tolist())
                analog_errors[m].append(((analog-clean_freq[m])/ref.HZ_PER_NT).tolist())
                projected[m].append((linear/ref.HZ_PER_NT).tolist())
            if (trial+1)%12==0:print(f'Frontend SNR {snr}: {trial+1}/{c["independent_records"]}',flush=True)
        ee=np.asarray(edge_errors);variance=np.var(ee,axis=0,ddof=1);case_rows=[]
        for m in windows:
            stat=summary(errors[m],np.random.default_rng(CFG['seed']+m))
            fixed=summary(rounded[m],np.random.default_rng(CFG['seed']+m))
            actual=float(np.sqrt(np.mean(np.var(analog_errors[m],axis=0,ddof=1))))
            pred=float(np.sqrt(np.mean(np.var(projected[m],axis=0,ddof=1))))
            x=np.arange(m)-(m-1)/2
            independent=np.sqrt(np.mean([clean_freq[m][i]**4*np.dot(variance[start:start+m],x*x)/sxx(m)**2/ref.HZ_PER_NT**2 for i,start in enumerate(starts[m])]))
            point=min(targets(),key=lambda p:abs(p['window_ms']-(m-1)/F0*1000))
            # Small x-coordinate correction uses the declared iid formula only.
            equivalent_m=point['window_ms']/1000*F0+1
            std_limit=point['STD']['pixel_interval_nT'][0]*np.sqrt(sxx(equivalent_m)/sxx(m))
            rmse_limit=point['RMSE']['pixel_interval_nT'][0]
            passed=bool(fixed['std_ci95_nT'][1]<std_limit and fixed['rmse_ci95_nT'][1]<rmse_limit)
            case_rows.append(dict(m=m,span_ms=(m-1)/F0*1000,frames_per_record=len(starts[m]),floating=stat,microhertz_output=fixed,
                                  target_plot_window_ms=point['window_ms'],std_lower_reading_limit_nT=std_limit,rmse_lower_reading_limit_nT=rmse_limit,
                                  meets_pointwise_engineering_gate=passed,
                                  actual_analog_noise_std_nT=actual,linear_joint_error_prediction_std_nT=pred,
                                  independence_surrogate_std_nT=float(independent),joint_to_independent_std_ratio=pred/float(independent),
                                  linearization_relative_std_error=abs(pred/actual-1),
                                  noiseless_analog_bias_nT=float(np.mean((clean_freq[m]-F0)/ref.HZ_PER_NT))))
        raw[str(snr)]=dict(errors_nT={str(k):v for k,v in errors.items()},microhertz_errors_nT={str(k):v for k,v in rounded.items()},
                           analog_noise_errors_nT={str(k):v for k,v in analog_errors.items()},projected_errors_nT={str(k):v for k,v in projected.items()})
        # Enough edge statistics to independently recompute the no-covariance surrogate.
        raw[str(snr)]['edge_error_variances_s2']=variance.tolist()
        rows.append(dict(input_snr_db=snr,postfilter_initial_reference_snr_db=float(np.mean(snrs)),quality=quality,rows=case_rows,
                         all_pointwise_engineering_gates_passed=all(r['meets_pointwise_engineering_gate'] for r in case_rows),
                         all_linearizations_passed=all(r['linearization_relative_std_error']<.02 for r in case_rows)))
    raw['clean']=dict(edge_times_s=clean_edges.tolist(),start_indices={str(k):v.tolist() for k,v in starts.items()},
                      frequencies_hz={str(k):v.tolist() for k,v in clean_freq.items()},scheduled_end_times_s={str(k):v.tolist() for k,v in ends.items()})
    write('frontend_raw.json',raw)
    write('frontend.json',dict(cases=rows,scope='Conditional 80 dB case intentionally differs from paper 35 dB. Passing bounds is not matching original curves or demonstrating a buildable front-end. Actual dependence of edge errors is retained in the linear projection.'))
    return dict(cases=[dict(input_snr_db=r['input_snr_db'],gates_passed=r['all_pointwise_engineering_gates_passed'],linearization_passed=r['all_linearizations_passed']) for r in rows])


def bench():
    c=CFG['bench'];rows=[];raw={};n=c['independent_records'];sigma=c['input_jitter_ns']*1e-9
    for span,target in zip(c['windows_s'],c['target_std_pT']):
        m=round(c['base_hz']*span)+1;j=np.arange(m,dtype=LD);x=j-(m-1)/2
        for offset in c['sub_microhertz_offsets']:
            f=LD(str(c['base_hz']))+LD(str(offset))*LD('0.000001')
            rng=np.random.default_rng(CFG['seed']+m)
            errors=[]
            for start in range(0,n,64):
                size=min(64,n-start)
                t=j[None,:]/f+rng.uniform(0,1/ref.FC,(size,1))+rng.normal(0,sigma,(size,m))
                ticks=np.ceil(t*ref.FC)
                freq=LD(ref.FC)*LD(sxx(m))/np.sum((ticks-ticks[:,:1])*x,axis=1)
                errors.extend(np.asarray(freq-f,dtype=float).tolist())
            errors=np.asarray(errors);outputs=[];key=f'window{span:g}_offset{offset:g}'
            for quantum in [None]+c['output_quantums_hz']:
                rounded=errors if quantum is None else np.asarray(np.floor((f+errors.astype(LD))/LD(str(quantum))+LD('.5'))*LD(str(quantum))-f,dtype=float)
                std=float(rounded.std(ddof=1)/ref.HZ_PER_NT*1000)
                # Chi-square interval is diagnostic for analog errors only; rounded errors are not normal.
                ci=(np.sqrt((n-1)*np.var(rounded,ddof=1)/chi2.ppf([.975,.025],n-1))/ref.HZ_PER_NT*1000).tolist() if quantum is None else None
                outputs.append(dict(quantum_hz=quantum,std_pT=std,mean_bias_pT=float(rounded.mean()/ref.HZ_PER_NT*1000),
                                    rmse_pT=float(np.sqrt(np.mean(rounded**2))/ref.HZ_PER_NT*1000),analog_normal_ci95_pT=ci,
                                    observed_std_below_target=bool(std<target)))
            rows.append(dict(key=key,requested_window_s=span,m=m,actual_window_s=(m-1)/float(f),carrier_offset_uHz=offset,
                             target_std_pT=target,outputs=outputs))
            raw[key]=errors.tolist()
    write('bench_raw.json',raw)
    write('bench.json',dict(rows=rows,scope='A declared iid timing model tests estimator precision and output rounding, not the physical instrument. Sub-LSB sweep prevents an integer-centred carrier from giving a misleadingly favourable rounded STD.'))
    return dict(nanohertz_all_observed_std_pass=all(next(o for o in r['outputs'] if o['quantum_hz']==1e-9)['observed_std_below_target'] for r in rows),
                microhertz_all_observed_std_pass=all(next(o for o in r['outputs'] if o['quantum_hz']==1e-6)['observed_std_below_target'] for r in rows))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--part',choices=['all','inverse','frontend','bench'],default='all');a=parser.parse_args()
    sources=['assess_feasibility.py','feasibility_protocol.json','reproduce.py','paper_config.json','audit_noise.py','noise_audit_protocol.json',
             'results/noise_audit/fig6_digitized.json','results/noise_audit/causal_frontend.json']
    hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources}
    OUT.mkdir(parents=True,exist_ok=True);report=OUT/f'validation_{a.part}.json';report.unlink(missing_ok=True)
    results={}
    for name,fn in [('inverse',inverse),('frontend',frontend),('bench',bench)]:
        if a.part in ('all',name):results[name]=fn()
    assert hashes=={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources}
    write(report.name,dict(source_sha256=hashes,results=results,full_paper_reproduced=False))
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()

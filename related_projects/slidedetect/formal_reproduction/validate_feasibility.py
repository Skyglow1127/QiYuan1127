"""Check raw evidence; distinguish conditional feasibility from paper replication."""
import hashlib,json
from pathlib import Path
import numpy as np
import reproduce as ref

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'results/feasibility'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):return json.loads(p.read_text())


def evidence(path):
    data=load(path)
    for name,h in data.get('source_sha256',{}).items():assert sha(ROOT/name)==h,(path,name)
    for name,h in data.get('report_sha256',{}).items():assert sha(path.parent/name)==h,(path,name)
    return data


def main():
    (OUT/'status.json').unlink(missing_ok=True)
    report=evidence(OUT/'validation_all.json');evidence(OUT/'validation_margin.json')
    inverse=load(OUT/'inverse_budget.json')
    assert len(inverse['rows'])==9 and all(r['seven_ns_inside_pixel_range'] for r in inverse['rows'])
    frontend=load(OUT/'frontend.json');raw=load(OUT/'frontend_raw.json')
    for case in frontend['cases']:
        snr=str(case['input_snr_db']);assert len(case['quality'])==48
        assert all(q['cycle_faults']==0 and q['same_event_ids'] for q in case['quality'])
        for row in case['rows']:
            m=row['m']
            for kind,key in [('errors_nT','floating'),('microhertz_errors_nT','microhertz_output')]:
                e=np.asarray(raw[snr][kind][str(m)])
                assert e.shape==(48,row['frames_per_record'])
                assert np.isclose(np.sqrt(np.mean(np.var(e,axis=0,ddof=1))),row[key]['std_nT'],rtol=1e-12)
                assert np.isclose(np.sqrt(np.mean(e*e)),row[key]['rmse_nT'],rtol=1e-12)
            projection=np.asarray(raw[snr]['projected_errors_nT'][str(m)])
            pred=np.sqrt(np.mean(np.var(projection,axis=0,ddof=1)))
            actual=np.sqrt(np.mean(np.var(raw[snr]['analog_noise_errors_nT'][str(m)],axis=0,ddof=1)))
            assert abs(pred/actual-1)<.02
            x=np.arange(m)-(m-1)/2;sxx=float(np.sum(x*x));var=np.asarray(raw[snr]['edge_error_variances_s2'])
            freq=raw['clean']['frequencies_hz'][str(m)];starts=raw['clean']['start_indices'][str(m)]
            independent=np.sqrt(np.mean([freq[i]**4*np.dot(var[s:s+m],x*x)/sxx**2/ref.HZ_PER_NT**2 for i,s in enumerate(starts)]))
            assert np.isclose(independent,row['independence_surrogate_std_nT'],rtol=1e-12)
            output=row['microhertz_output']
            passed=bool(output['std_ci95_nT'][1]<row['std_lower_reading_limit_nT'] and output['rmse_ci95_nT'][1]<row['rmse_lower_reading_limit_nT'])
            assert passed==row['meets_pointwise_engineering_gate']
        assert all(r['meets_pointwise_engineering_gate'] for r in case['rows'])==case['all_pointwise_engineering_gates_passed']
    original=next(c for c in frontend['cases'] if c['input_snr_db']==35)
    conditional=next(c for c in frontend['cases'] if c['input_snr_db']==80)
    assert not original['all_pointwise_engineering_gates_passed'] and conditional['all_pointwise_engineering_gates_passed']
    margin=load(OUT/'bench_margin.json');mraw=load(OUT/'bench_margin_raw.json')
    assert len(margin['rows'])==24
    for row in margin['rows']:
        e=np.array(mraw[row['key']]);assert len(e)==4096
        f=np.longdouble('2128.87')+np.longdouble(str(row['offset_uHz']))*np.longdouble('1e-6')
        for o in row['outputs']:
            unit=o['quantum_hz']
            values=e if unit is None else np.asarray(np.floor((f+e.astype(np.longdouble))/np.longdouble(str(unit))+np.longdouble('.5'))*np.longdouble(str(unit))-f,dtype=float)
            std=values.std(ddof=1)/ref.HZ_PER_NT*1000
            assert np.isclose(std,o['std_pT'],rtol=1e-10,atol=1e-12)
            assert o['meets_pointwise_std_gate']==(o['std_bootstrap_ci95_pT'][1]<row['target_std_pT'])
    selected=[r for r in margin['rows'] if r['jitter_ns']==.5]
    assert all(next(o for o in r['outputs'] if o['quantum_hz']==1e-9)['meets_pointwise_std_gate'] for r in selected)
    grid=load(OUT/'grid_convergence.json');assert all(r['passed'] and r['fraction_of_target_std']<.01 for r in grid['rows'])
    rtlpath=OUT/'precision_rtl/validation.json';rtl=evidence(rtlpath) if rtlpath.exists() else None
    if rtl:
        assert len(rtl['profiles'])==4 and all(p['readback_exact'] and p['fault_checks']==4 for p in rtl['profiles'])
        for p in rtl['profiles']:assert sha(rtlpath.parent/p['file'])==p['sha256']
    sensorpath=OUT/'precision_sensor/validation.json';sensor=evidence(sensorpath) if sensorpath.exists() else None
    if sensor:assert all(p['passed'] and p['m']==96 and p['hop']==24 for p in sensor['profiles'])
    timingpath=OUT/'precision_sensor_timing_hop24/validation.json';timing=evidence(timingpath) if timingpath.exists() else None
    # A failed standalone synchronous-I/O hold check remains visible.
    standalone=evidence(OUT/'nhz_timing/validation.json') if (OUT/'nhz_timing/validation.json').exists() else None
    result=dict(evidence_checks_passed=True,full_paper_reproduced=False,
                paper_fig6_equivalent_iid_jitter_ns=inverse['equivalent_iid_jitter_range_ns'],
                original_35dB_frontend_target_met=False,
                conditional_80dB_frontend=dict(passed=True,records=48,windows=9,scope='Declared idealized frontend; pointwise bootstrap gates; intentionally changes paper input SNR and threshold assumptions'),
                conditional_bench_05ns=dict(passed=True,records_per_case=4096,carrier_offsets=4,windows=[.01,1],output_quantum_hz=1e-9,
                    scope='Stationary iid timing noise plus ceil counter quantization; no physical frontend or clock-drift certification'),
                precision_rtl_outputs=sum(p['checks'] for p in rtl['profiles']) if rtl else None,
                precision_sensor_outputs=sum(p['outputs'] for p in sensor['profiles']) if sensor else None,
                precision_sensor_200mhz_passed=timing['timing_met'] if timing else None,
                standalone_nhz_hold_met=standalone['timing_met'] if standalone else None,
                remaining=['Original Fig.6-8 noise/statistics are underdetermined by the paper','Full author PLL/EKF models unavailable','Physical clock/frontend/bench measurements not performed'],
                evidence_sha256={str(p.relative_to(ROOT)):sha(p) for p in sorted(OUT.rglob('*')) if p.is_file() and 'build' not in p.parts and p.name!='status.json'})
    (OUT/'status.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='evidence_sha256'},indent=2,ensure_ascii=False))


if __name__=='__main__':main()

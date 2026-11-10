"""Verify new evidence, with unmet paper targets retained as unmet."""
import hashlib,json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'results/noise_audit'

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):return json.loads(p.read_text())

def main():
    (OUT/'status.json').unlink(missing_ok=True)
    all_report=OUT/'validation_all.json'
    reports=[all_report] if all_report.exists() else [OUT/f'validation_{part}.json' for part in ['budget','plateau','frontend']]
    for p in reports:
        for name,h in load(p)['source_sha256'].items():assert digest(ROOT/name)==h,(p,name)
    plateau=load(OUT/'plateau.json');raw=load(OUT/'plateau_raw.json')
    assert len(plateau['rows'])==30
    for r in plateau['rows']:
        e=np.array(raw[r['key']]['plateau_error_hz'])
        assert e.shape==(r['records'],2)
        assert np.allclose(e.std(axis=0,ddof=1)*1e6,r['plateau_std_uHz'],rtol=1e-12)
        failures=int(np.count_nonzero((e[:,0]>5e-6)|(e[:,1]<=-5e-6)))
        assert failures==r['record_failures']
        if r['jitter_ns']>=20:assert r['overlap_prediction_within_20percent']
    selected=next(r for r in plateau['rows'] if r['jitter_ns']==20 and r['dwell_s']==5)
    assert selected['meets_proposed_plateau_gate']
    frontend=load(OUT/'causal_frontend.json');fraw=load(OUT/'causal_frontend_raw.json')
    assert len(frontend['cases'])==6
    for c in frontend['cases']:
        assert len(c['quality'])==24
        assert all(q['cycle_discontinuities_after_guard']==0 for q in c['quality'])
        for row in c['statistics']:
            e=np.array(fraw[c['name']]['errors_nT'][str(row['m'])])
            assert np.isclose(np.sqrt(np.mean(np.var(e,axis=0,ddof=1))),row['across_record_std_nT'],rtol=1e-12)
    digitized=load(OUT/'fig6_digitized.json')
    assert digitized['source_sha256']==digest(ROOT/'digitize_fig6.py')
    point=min(digitized['rows'],key=lambda r:abs(r['window_ms']-99.227))
    filtered=next(c for c in frontend['cases'] if c['name']=='bp100_h0002')
    row=next(r for r in filtered['statistics'] if r['m']==208)
    rtl_path=OUT/'noisy_replay/rtl_validation.json';rtl=None
    if rtl_path.exists():
        rtl=load(rtl_path)
        binding=load(OUT/'noisy_replay/binding.json')
        for report in [rtl,binding]:
            for name,h in report['source_sha256'].items():assert digest(ROOT/name)==h,name
        for stream in binding['streams']:assert digest(OUT/'noisy_replay'/stream['file'])==stream['sha256']
        for profile in rtl['profiles']:
            assert profile['readback_exact']
            assert digest(OUT/f'noisy_replay/noisy_m{profile["m"]}.txt')==profile['vector_sha256']
    result=dict(
        noise_audit_checks_passed=True,full_paper_reproduced=False,
        paper_conditions_separated=True,
        plateau_20ns_5s=dict(records=selected['records'],record_failures=selected['record_failures'],
                           record_error_ci95=selected['record_error_ci95'],passed=True,
                           scope='Stationary timing noise, known midpoint, mean over each 5 s plateau. Not a 100 ms raw-output precision claim, not a decaying FID claim.'),
        frontend_35dB_100Hz=dict(window_ms=row['span_ms'],std_nT=row['across_record_std_nT'],std_ci95_nT=row['std_ci95_nT'],
            paper_digitized_std_nT=point['STD']['nT'],paper_pixel_range_nT=point['STD']['pixel_interval_nT'],ratio_to_digitized=float(row['across_record_std_nT']/point['STD']['nT']),
            explains_paper_precision=False,scope='Declared frontend and different/unknown original aggregation; gap diagnosis, not an identical-condition replication.'),
        noisy_rtl_outputs=sum(p['checks'] for p in rtl['profiles']) if rtl else None,
        remaining=['Exact Fig.6-8 original noise, thresholds and statistics','Full PLL/EKF original models','Bench/frontend/clock measurements'],
        evidence_sha256={str(p.relative_to(ROOT)):digest(p) for p in sorted(OUT.rglob('*')) if p.is_file() and 'build' not in p.parts and p.suffix in ('.json','.csv','.txt')})
    (OUT/'status.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='evidence_sha256'},indent=2,ensure_ascii=False))

if __name__=='__main__':main()

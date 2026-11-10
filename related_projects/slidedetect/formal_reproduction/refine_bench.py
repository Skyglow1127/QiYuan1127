"""Analytically motivated margin check; retains the earlier 1 ns failure."""
import hashlib,json
from pathlib import Path
import numpy as np
import reproduce as ref
from assess_feasibility import sxx, summary
from scipy.signal import butter,sosfilt

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'results/feasibility'
LD=np.longdouble
PROTOCOL={
    'seed':2026111111,'independent_records':4096,'jitter_ns':[0,0.5,1],
    'selection':'At 10 ms the inverse iid budget is 1.677 ns total; subtracting uniform 5 ns quantization variance gives about 0.854 ns analog margin. Select 0.5 ns before the follow-up to leave margin, retain 0 and 1 ns controls. Actual ceil quantization is tested rather than assumed independent.',
    'grid_convergence':{'seed':2026111113,'records':12,'fine_hz':2000000,'coarse_hz':1000000,'snr_db':80,
                       'threshold_A0':0.0001,'bandpass_width_hz':100,'guard_s':0.2,'duration_s':1.5,
                       'gate':'RMS difference between unquantized SFLS on 1 and 2 MHz sampled versions of the SAME filtered analog record must be below 1 percent of the Fig.6 lower STD reading at every tested M. Actual clock quantization changes are reported separately.'}}


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def bench():
    c=json.loads((ROOT/'feasibility_protocol.json').read_text())['bench'];n=PROTOCOL['independent_records'];rows=[];raw={}
    for sigma_ns in PROTOCOL['jitter_ns']:
        for span,target in zip(c['windows_s'],c['target_std_pT']):
            m=round(c['base_hz']*span)+1;j=np.arange(m,dtype=LD);x=j-(m-1)/2
            for offset in c['sub_microhertz_offsets']:
                f=LD(str(c['base_hz']))+LD(str(offset))*LD('1e-6');rng=np.random.default_rng(PROTOCOL['seed']+m)
                errors=[]
                for first in range(0,n,64):
                    count=min(64,n-first)
                    t=j[None,:]/f+rng.uniform(0,1/ref.FC,(count,1))+rng.normal(0,sigma_ns*1e-9,(count,m))
                    ticks=np.ceil(t*ref.FC)
                    freq=LD(ref.FC)*LD(sxx(m))/np.sum((ticks-ticks[:,:1])*x,axis=1)
                    errors.extend(np.asarray(freq-f,dtype=float).tolist())
                e=np.asarray(errors);key=f'jitter{sigma_ns}_m{m}_offset{offset}';raw[key]=e.tolist();results=[]
                # Resample independent records. No Gaussian assumption for rounded errors.
                rng=np.random.default_rng(PROTOCOL['seed']+m+1)
                indices=rng.integers(0,n,(1000,n))
                for unit in [None,1e-6,1e-9]:
                    values=e if unit is None else np.asarray(np.floor((f+e.astype(LD))/LD(str(unit))+LD('.5'))*LD(str(unit))-f,dtype=float)
                    pT=values/ref.HZ_PER_NT*1000;boot=pT[indices]
                    std=float(pT.std(ddof=1));ci=np.quantile(boot.std(axis=1,ddof=1),[.025,.975]).tolist()
                    results.append(dict(quantum_hz=unit,std_pT=std,std_bootstrap_ci95_pT=ci,
                                        mean_bias_pT=float(pT.mean()),rmse_pT=float(np.sqrt(np.mean(pT*pT))),
                                        meets_pointwise_std_gate=bool(ci[1]<target)))
                rows.append(dict(key=key,m=m,window_s=span,offset_uHz=offset,jitter_ns=sigma_ns,target_std_pT=target,outputs=results))
        print('Independent margin check jitter ns',sigma_ns,flush=True)
    (OUT/'bench_margin_raw.json').write_text(json.dumps(raw)+'\n')
    (OUT/'bench_margin.json').write_text(json.dumps(dict(protocol=PROTOCOL,rows=rows),indent=2)+'\n')


def grid():
    c=PROTOCOL['grid_convergence'];fs=c['fine_hz'];t=np.arange(round(c['duration_s']*fs)+1)/fs
    f0=ref.P['background_nT_fig5']*ref.HZ_PER_NT
    sos=butter(2,[f0-50,f0+50],btype='bandpass',fs=fs,output='sos');lp=butter(4,10000,fs=fs,output='sos')
    clean=np.exp(-t/1.5)*np.sin(2*np.pi*f0*t);burn=round(.02*fs)
    points=json.loads((ROOT/'results/feasibility/frontend.json').read_text())['cases'][1]['rows']
    differences={r['m']:[] for r in points};qdiff={r['m']:[] for r in points};quality=[]
    for trial in range(c['records']):
        rng=np.random.default_rng(c['seed']+trial);noise=sosfilt(lp,rng.normal(size=len(t)+burn))[burn:]
        noise*=10**(-c['snr_db']/20)/np.sqrt(2)/np.std(noise)
        y=sosfilt(sos,clean+noise)
        fine=ref.schmitt(t,y,c['threshold_A0']);coarse=ref.schmitt(t[::2],y[::2],c['threshold_A0'])
        fine=fine[fine>=c['guard_s']];coarse=coarse[coarse>=c['guard_s']]
        assert np.array_equal(np.rint(fine*f0),np.rint(coarse*f0))
        quality.append(dict(seed=c['seed']+trial,events=len(fine),max_crossing_change_ns=float(np.max(abs(coarse-fine))*1e9)))
        for row in points:
            m=row['m'];a=ref.direct(fine*ref.FC,m,m//2)['frequency'];b=ref.direct(coarse*ref.FC,m,m//2)['frequency']
            qa=ref.direct(np.ceil(fine*ref.FC),m,m//2)['frequency'];qb=ref.direct(np.ceil(coarse*ref.FC),m,m//2)['frequency']
            differences[m].extend(np.asarray((b-a)/ref.HZ_PER_NT,dtype=float).tolist())
            qdiff[m].extend(np.asarray((qb-qa)/ref.HZ_PER_NT,dtype=float).tolist())
    rows=[]
    for point in points:
        m=point['m'];d=np.array(differences[m]);qd=np.array(qdiff[m]);relative=float(np.sqrt(np.mean(d*d))/point['std_lower_reading_limit_nT'])
        rows.append(dict(m=m,unquantized_rms_difference_nT=float(np.sqrt(np.mean(d*d))),
                         quantized_rms_difference_nT=float(np.sqrt(np.mean(qd*qd))),
                         fraction_of_target_std=relative,passed=relative<.01))
    (OUT/'grid_convergence.json').write_text(json.dumps(dict(protocol=c,rows=rows,quality=quality,
        scope='Convergence of event interpolation only. The same 2 MHz filtered realization is subsampled; this does not test every possible continuous-time filter discretization.'),indent=2)+'\n')


def main():
    sources=['refine_bench.py','reproduce.py','assess_feasibility.py','feasibility_protocol.json','results/feasibility/frontend.json']
    hashes={p:digest(ROOT/p) for p in sources};(OUT/'validation_margin.json').unlink(missing_ok=True)
    bench();grid()
    assert hashes=={p:digest(ROOT/p) for p in sources}
    (OUT/'validation_margin.json').write_text(json.dumps(dict(source_sha256=hashes),indent=2)+'\n')


if __name__=='__main__':main()

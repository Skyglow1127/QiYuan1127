"""Replay fine and legacy outputs against independent exact integer answers."""
import csv,hashlib,json,os,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'results/feasibility/precision_rtl'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    subprocess.run(['/usr/bin/python3',str(ROOT/'prepare_precision_replay.py')],check=True)
    spec=json.loads((OUT/'vectors.json').read_text());build=OUT/'build';build.mkdir(exist_ok=True)
    (OUT/'validation.json').unlink(missing_ok=True)
    sources=['rtl/udiv_round.sv','rtl/sfls_core.sv','rtl/sfls_nhz_readout.sv','sim/tb_precision.sv',
             'prepare_precision_replay.py','run_precision_rtl.py']
    hashes={n:sha(ROOT/n) for n in sources};env=dict(os.environ)
    env['XILINX_VIVADO']='/home/aup/Xilinx/2025.2/Vivado';env['PATH']=env['XILINX_VIVADO']+'/bin:'+env['PATH']
    def run(label,args):
        log=OUT/f'{label}.log'
        with log.open('w') as f:subprocess.run(args,cwd=build,env=env,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=240)
        text=log.read_text();assert not re.search('FATAL|ERROR:|Fatal:',text),label
        return text
    run('compile',['xvlog','--sv']+[str(ROOT/n) for n in sources[:4]])
    reports=[]
    for p in spec['profiles']:
        m=p['m'];label=f'm{m}'
        run('elaborate_'+label,['xelab','tb_precision','-s',label,'-debug','typical','-mt','2','-generic_top',f'WINDOW_SIZE={m}'])
        log=run('simulate_'+label,['xsim',label,'-runall','-onerror','quit','-onfinish','quit',
              '-testplusarg',f'VECTORS={OUT/p["file"]}','-testplusarg',f'RESULTS={OUT/label}.csv'])
        match=re.search(r'PASS precision M=(\d+) events=(\d+) checks=(\d+) latency=(\d+) fault_checks=(\d+)',log)
        assert match and tuple(map(int,match.groups()[:3]))==(m,p['events'],p['checks'])
        expected=[]
        for line in (OUT/p['file']).read_text().splitlines():
            op,tick,q,total,u,n=line.split()
            if op=='2':expected.append((int(u,16),int(n,16),int(tick,16),int(total,16)//m))
        with (OUT/f'{label}.csv').open() as f:actual=[tuple(int(r[k]) for k in ['frequency_uHz','frequency_nHz','last_tick','mean_tick']) for r in csv.DictReader(f)]
        assert actual==expected and sha(OUT/p['file'])==p['sha256']
        reports.append(dict(**p,readback_exact=True,latency_cycles=int(match[4]),fault_checks=int(match[5])))
        print(match[0],flush=True)
    assert hashes=={n:sha(ROOT/n) for n in sources}
    result=dict(profiles=reports,source_sha256=hashes,scope='Optional atomic-snapshot nHz readout beside unchanged core; timestamp replay, not integrated sensor-top or board timing certification.',
                report_sha256={str(p.relative_to(OUT)):sha(p) for p in OUT.iterdir() if p.is_file() and p.name!='validation.json'})
    (OUT/'validation.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()

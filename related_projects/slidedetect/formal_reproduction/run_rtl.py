#!/usr/bin/env python3
"""Vivado arithmetic replay of published window sizes and ring-buffer regressions."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results',type=Path,default=ROOT/'results')
    p.add_argument('--vivado',type=Path,default=Path('/home/aup/Xilinx/2025.2/Vivado'))
    p.add_argument('--legacy-vectors',type=Path)
    args=p.parse_args();out=args.results.resolve()
    (out/'rtl_validation.json').unlink(missing_ok=True)
    spec=json.loads((out/'summary.json').read_text())
    profiles=[dict(label=f'm{x["m"]}',m=x['m'],clock=200000000,stride=1,
                   vectors=out/x['file'],events=x['events'],checks=x['checks']) for x in spec['rtl_vectors']]
    if args.legacy_vectors:
        for d in (1,64):
            profiles.append(dict(label=f'legacy_d{d}',m=32768,clock=100000000,stride=d,
                                 vectors=args.legacy_vectors/f'replay_stride{d}.txt',events=99072,checks=15))
    work=out/'build';work.mkdir(exist_ok=True)
    env=dict(os.environ,XILINX_VIVADO=str(args.vivado))
    env['PATH']=str(args.vivado/'bin')+os.pathsep+env.get('PATH','')
    def command(name,argv):
        with (work/f'{name}.log').open('w') as f:
            subprocess.run(argv,cwd=work,env=env,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=180)
        log=(work/f'{name}.log').read_text()
        if re.search('Fatal:|FATAL|ERROR:',log):raise RuntimeError(f'{name}: see {work}')
        return log
    command('compile',['xvlog','--sv',str(ROOT/'rtl/udiv_round.sv'),str(ROOT/'rtl/sfls_core.sv'),str(ROOT/'sim/tb_replay.sv')])
    reports=[]
    for profile in profiles:
        label,m=profile['label'],profile['m']
        command('elaborate_'+label,['xelab','tb_replay','-s',label,'-debug','typical','-mt','2',
                '-generic_top',f'WINDOW_SIZE={m}','-generic_top',f'CLOCK_HZ={profile["clock"]}',
                '-generic_top',f'EDGE_STRIDE={profile["stride"]}'])
        log=command('simulate_'+label,['xsim',label,'-runall','-onerror','quit','-onfinish','quit',
                    '-testplusarg',f'VECTORS={profile["vectors"]}',
                    '-testplusarg',f'RESULTS={work}/{label}.csv'])
        match=re.search(r'^PASS replay stride=(\d+) window=(\d+) clock=(\d+) events=(\d+) checks=(\d+)',log,re.M)
        if not match:raise RuntimeError('Missing PASS '+label)
        got=tuple(map(int,match.groups()))
        assert got==(profile['stride'],m,profile['clock'],profile['events'],profile['checks'])
        expected=[]
        for line in profile['vectors'].read_text().splitlines():
            op,tick,q,total,freq=line.split()
            if op=='2':expected.append((int(freq,16),int(tick,16),int(total,16)//m))
        with (work/f'{label}.csv').open() as f:
            actual=[(int(x['frequency_uHz']),int(x['last_tick']),int(x['mean_tick'])) for x in csv.DictReader(f)]
        assert actual==expected
        (out/f'rtl_{label}.txt').write_text(log)
        (out/f'rtl_{label}.csv').write_bytes((work/f'{label}.csv').read_bytes())
        reports.append(dict(label=label,m=m,clock=profile['clock'],stride=profile['stride'],
                            events=profile['events'],checks=len(expected),readback_exact=True,
                            vector_sha256=hashlib.sha256(profile['vectors'].read_bytes()).hexdigest(),pass_line=match[0]))
        print(match[0],flush=True)
    result=dict(scope='Arithmetic replay only. 200 MHz timestamp scale is not a 200 MHz placed-and-routed timing claim.',
                source_sha256={str(x.relative_to(ROOT)):hashlib.sha256(x.read_bytes()).hexdigest()
                               for x in [ROOT/'rtl/udiv_round.sv',ROOT/'rtl/sfls_core.sv',ROOT/'sim/tb_replay.sv']},
                profiles=reports)
    (out/'rtl_validation.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()

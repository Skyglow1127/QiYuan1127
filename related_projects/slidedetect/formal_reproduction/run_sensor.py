"""Real 200 MHz clock simulation, async edges, scheduling and acquisition reset."""
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parent
OUT=ROOT/'results/sensor';BUILD=ROOT/'build/sensor'
VIVADO=Path('/home/aup/Xilinx/2025.2/Vivado')


def main():
    OUT.mkdir(parents=True,exist_ok=True);BUILD.mkdir(parents=True,exist_ok=True)
    (OUT/'validation.json').unlink(missing_ok=True)
    env=dict(os.environ,XILINX_VIVADO=str(VIVADO));env['PATH']=str(VIVADO/'bin')+os.pathsep+env.get('PATH','')
    sources=['rtl/udiv_round.sv','rtl/sfls_core.sv','rtl/sfls_sensor.sv','sim/tb_sensor.sv']
    def run(label,args):
        with (OUT/f'{label}.log').open('w') as f:
            subprocess.run(args,cwd=BUILD,env=env,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=600)
        text=(OUT/f'{label}.log').read_text()
        if re.search(r'Fatal:|FATAL|ERROR:',text):raise RuntimeError(label)
        return text
    run('compile',['xvlog','--sv',*(str(ROOT/p) for p in sources)])
    profiles=[]
    for hop in (0,24):
        label=f'm96_hop{hop}'
        run('elaborate_'+label,['xelab','tb_sensor','-s',label,'-debug','typical','-mt','2','-generic_top',f'HOP_EVENTS={hop}'])
        log=run('simulate_'+label,['xsim',label,'-runall','-onerror','quit','-onfinish','quit','-testplusarg',f'RESULTS={OUT}/{label}.csv'])
        match=re.search(r'PASS sensor window=(\d+) hop=(\d+) epochs=(\d+) events=(\d+) outputs=(\d+) max_latency=(\d+)',log)
        assert match, label
        m,h,epochs,events,outputs,latency=map(int,match.groups())
        assert (m,h,epochs)==(96,hop,2)
        with (OUT/f'{label}.csv').open() as f:rows=list(csv.DictReader(f))
        assert len(rows)==outputs and all(r['frequency_uHz']==r['reference_uHz'] for r in rows)
        profiles.append(dict(m=m,hop=hop,epochs=epochs,events=events,outputs=outputs,max_latency_cycles=latency,passed=True))
        print(match[0],flush=True)
    result=dict(profiles=profiles,scope='Digital comparator waveform only; no analog noise or physical metastability claim',
                source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources})
    (OUT/'validation.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()

"""Audit integrated digital timing at 200 MHz; never access or program a board."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parent


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--hop',type=int,choices=[0,24],default=24);args=p.parse_args()
    out=ROOT/f'results/sensor_timing_hop{args.hop}';out.mkdir(parents=True,exist_ok=True)
    build=ROOT/f'build/sensor_timing_hop{args.hop}';build.mkdir(parents=True,exist_ok=True)
    (out/'validation.json').unlink(missing_ok=True);(out/'status.txt').unlink(missing_ok=True)
    sources=['rtl/udiv_round.sv','rtl/sfls_core.sv','rtl/sfls_sensor.sv',
             'scripts/sensor_200mhz.xdc','scripts/implement_sensor.tcl','run_sensor_timing.py']
    hashes={n:sha(ROOT/n) for n in sources}
    cmd=['/home/aup/Xilinx/2025.2/Vivado/bin/vivado','-mode','batch','-notrace','-nojournal',
         '-log',str(out/'vivado.log'),'-source',str(ROOT/'scripts/implement_sensor.tcl'),'-tclargs',str(args.hop),str(out)]
    with (out/'console.log').open('w') as log:
        subprocess.run(cmd,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
    console=(out/'console.log').read_text()
    assert 'SENSOR_TIMING_COMPLETE' in console and not re.search('ERROR:|CRITICAL WARNING:',console)
    checks={k:int(v) for k,v in re.findall(r'checking (\w+) \((\d+)\)',(out/'check_timing.rpt').read_text())}
    for key in ('no_clock','unconstrained_internal_endpoints','no_output_delay','partial_input_delay','partial_output_delay','loops','latch_loops'):
        assert checks[key]==0,(key,checks)
    assert checks['no_input_delay']<=1,checks # only the explicitly exempt async comparator input
    for name,h in hashes.items():assert sha(ROOT/name)==h,name
    status=dict(x.split('=',1) for x in (out/'status.txt').read_text().splitlines())
    result=dict(m=96,hop=args.hop,clock_mhz=200,wns_ns=float(status['WNS']),whs_ns=float(status['WHS']),
                timing_met=float(status['WNS'])>=0 and float(status['WHS'])>=0,
                scope='Integrated digital top OOC; physical board clock, pins and analog frontend excluded',
                async_exception='larmor_in to sync1_reg/D only',constraint_checks=checks,source_sha256=hashes,
                report_sha256={f.name:sha(f) for f in out.glob('*.rpt')})
    (out/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()

"""Route the fine readout alone at 200 MHz. No device connection/programming."""
import hashlib,json,re,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'results/feasibility/nhz_timing'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    OUT.mkdir(parents=True,exist_ok=True);build=OUT/'build';build.mkdir(exist_ok=True)
    (OUT/'validation.json').unlink(missing_ok=True);(OUT/'status.txt').unlink(missing_ok=True)
    sources=['rtl/udiv_round.sv','rtl/sfls_nhz_readout.sv','scripts/nhz_200mhz.xdc','scripts/implement_nhz.tcl','run_nhz_timing.py']
    hashes={n:sha(ROOT/n) for n in sources}
    with (OUT/'console.log').open('w') as f:
        subprocess.run(['/home/aup/Xilinx/2025.2/Vivado/bin/vivado','-mode','batch','-notrace','-nojournal',
                        '-log',str(OUT/'vivado.log'),'-source',str(ROOT/'scripts/implement_nhz.tcl'),'-tclargs',str(OUT)],
                       cwd=build,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=600)
    console=(OUT/'console.log').read_text()
    assert 'NHZ_TIMING_COMPLETE' in console and not re.search('ERROR:|CRITICAL WARNING:',console)
    checks={k:int(v) for k,v in re.findall(r'checking (\w+) \((\d+)\)',(OUT/'check_timing.rpt').read_text())}
    for k in ['no_clock','unconstrained_internal_endpoints','no_input_delay','no_output_delay','partial_input_delay','partial_output_delay','loops','latch_loops']:
        assert checks[k]==0,(k,checks)
    status=dict(line.split('=',1) for line in (OUT/'status.txt').read_text().splitlines())
    assert hashes=={n:sha(ROOT/n) for n in sources}
    result=dict(m=2130,clock_mhz=200,wns_ns=float(status['WNS']),whs_ns=float(status['WHS']),
                timing_met=float(status['WNS'])>=0 and float(status['WHS'])>=0,
                scope='Fine readout module only, synchronous input/output budgets. Not timing of a newly integrated full sensor or board.',
                constraint_checks=checks,source_sha256=hashes,report_sha256={p.name:sha(p) for p in OUT.glob('*.rpt')})
    (OUT/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()

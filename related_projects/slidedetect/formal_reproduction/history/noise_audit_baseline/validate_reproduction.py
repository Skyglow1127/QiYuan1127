"""Summarize evidence gates without treating missing author settings as passes."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def load(p):return json.loads((ROOT/p).read_text())


def evidence(p):
    data=load(p)
    for name,h in data.get('source_sha256',{}).items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==h,(p,name)
    for name,h in data.get('report_sha256',{}).items():
        assert hashlib.sha256((ROOT/p).parent.joinpath(name).read_bytes()).hexdigest()==h,(p,name)
    return data


def main():
    (ROOT/'results/reproduction_status.json').unlink(missing_ok=True)
    numeric=evidence('results/simulation_completion/validation.json')
    protocol=load('simulation_protocol.json')
    assert numeric['protocol_sha256']==hashlib.sha256((ROOT/'simulation_protocol.json').read_bytes()).hexdigest()
    rtl=evidence('results/rtl_validation.json');sensor=evidence('results/sensor/validation.json')
    timing=[evidence(f'results/sensor_timing_hop{h}/validation.json') for h in (0,24)]
    assert {32,64,96,128,256,512,1024} <= {p['m'] for p in rtl['profiles']}
    assert all(p['readback_exact'] for p in rtl['profiles'])
    assert numeric['theory']['passed'] and numeric['drift']['theory_bias_passed']
    assert all(p['passed'] for p in sensor['profiles'])
    assert all(p['timing_met'] for p in timing)
    steps=load('results/simulation_completion/step.json')['conditions']
    gates=[
        dict(item='Independent arithmetic / variance / chirp formulas',status='pass'),
        dict(item='M=96 integrated digital input chain, scheduling, timeout, restart',status='pass'),
        dict(item='M=96 digital top 200 MHz OOC timing, hop=0 and 24',status='pass_with_stated_clock_and_interface_contract'),
        dict(item='10 microHz single-window classification, quantization-only',status='pass_under_ideal_assumption',error_ci95=steps[0]['error_rate_ci95']),
        dict(item='10 microHz single-window classification with 20 ns independent jitter',status='does_not_meet_5_percent_error_target',error_ci95=steps[1]['error_rate_ci95']),
        dict(item='Fig.6 exact STD/RMSE values',status='unresolved_author_noise_threshold_statistics'),
        dict(item='Fig.7 all seven numerical labels',status='partial_six_close_large_window_definition_unresolved'),
        dict(item='Fig.8 exact map',status='declared_assumption_diagnostic_only'),
        dict(item='Fig.12 PLL/EKF numerical baseline reproduction',status='not_implemented_complete_author_models_needed'),
        dict(item='Bench and outdoor measurements / physical CDC reliability',status='outside_simulation_claim')]
    result=dict(gates=gates,full_paper_reproduced=False,
                rtl_outputs=sum(p['checks'] for p in rtl['profiles']),
                digital_chain_outputs=sum(p['outputs'] for p in sensor['profiles']),
                theory_time_exponent=numeric['theory']['slope'],
                provisional_discrimination_target='Wilson 95% upper error-rate bound below 5%; our proposed gate, not a paper-specified criterion',
                scope='Passing calculable gates does not certify unknown original settings or physical experiments')
    (ROOT/'results/reproduction_status.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()

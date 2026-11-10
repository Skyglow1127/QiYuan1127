"""Reproducible KR260 core-only 200 MHz place/route audit; no board access."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window', type=int, default=96)
    parser.add_argument('--vivado', default='/home/aup/Xilinx/2025.2/Vivado/bin/vivado')
    args = parser.parse_args()
    if args.window < 2:
        parser.error('--window must be at least 2')
    root = Path(__file__).resolve().parent
    build = root / 'build' / f'timing_m{args.window}'
    out = root / 'results' / f'timing_m{args.window}'
    build.mkdir(parents=True, exist_ok=True)
    out.mkdir(parents=True, exist_ok=True)
    # Invalidate previous summary before starting a fresh tool run.
    summary = out / 'validation.json'
    summary.unlink(missing_ok=True)
    (out / 'status.txt').unlink(missing_ok=True)
    sources = ['rtl/sfls_core.sv', 'rtl/udiv_round.sv',
               'scripts/implement_core.tcl', 'scripts/core_200mhz.xdc', 'run_timing.py']
    source_hashes = {name: digest(root / name) for name in sources}
    command = [args.vivado, '-mode', 'batch', '-notrace', '-nojournal',
               '-log', str(out / 'vivado.log'), '-source',
               str(root / 'scripts/implement_core.tcl'),
               '-tclargs', str(args.window), str(out)]
    started = time.time()
    env = dict(os.environ)
    env['LC_ALL'] = 'C'
    print(f'Running core place/route: M={args.window}, 200 MHz, KR260', flush=True)
    with (out / 'console.log').open('w') as log:
        result = subprocess.run(command, cwd=build, env=env, stdout=log, stderr=subprocess.STDOUT)
    console = (out / 'console.log').read_text(errors='replace')
    if result.returncode or 'CORE_TIMING_COMPLETE ' not in console or 'ERROR:' in console or 'CRITICAL WARNING:' in console:
        raise RuntimeError(f'Vivado execution failed; see {out / "console.log"}')
    timing_check = (out / 'check_timing.rpt').read_text()
    checks = {name: int(value) for name, value in re.findall(r'checking (\w+) \((\d+)\)', timing_check)}
    required = ['no_clock', 'unconstrained_internal_endpoints', 'no_input_delay',
                'no_output_delay', 'partial_input_delay', 'partial_output_delay', 'loops', 'latch_loops']
    if any(name not in checks or checks[name] != 0 for name in required):
        raise RuntimeError(f'Incomplete timing constraints: {checks}')
    if any(digest(root / name) != value for name, value in source_hashes.items()):
        raise RuntimeError('Source changed during tool execution; rerun required')
    status = dict(line.split('=', 1) for line in (out / 'status.txt').read_text().splitlines())
    data = {
        'window_size': args.window, 'clock_mhz': 200, 'part': status['part'],
        'vivado_version': status['Vivado'], 'wns_ns': float(status['WNS']),
        'whs_ns': float(status['WHS']), 'internal_wns_ns': float(status['internal_WNS']),
        'internal_whs_ns': float(status['internal_WHS']),
        'internal_timing_met': float(status['internal_WNS']) >= 0 and float(status['internal_WHS']) >= 0,
        'timing_met': float(status['WNS']) >= 0 and float(status['WHS']) >= 0,
        'execution_succeeded': True, 'elapsed_seconds': round(time.time() - started, 2),
        'scope': 'Out-of-context core, synchronous I/O budget, not board validation',
        'constraint_checks': checks,
        'source_sha256': source_hashes,
        'report_sha256': {p.name: digest(p) for p in sorted(out.glob('*.rpt'))},
    }
    summary.write_text(json.dumps(data, indent=2) + '\n')
    print(json.dumps(data, indent=2), flush=True)


if __name__ == '__main__':
    main()

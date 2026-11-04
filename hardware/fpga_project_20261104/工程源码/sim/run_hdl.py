#!/usr/bin/env python3
"""Run installed standalone Vivado Simulator, with no FPGA/device access.

Example:
  /usr/bin/python3 sim/run_hdl.py --suite all --output-dir /tmp/robot_rt_hdl_new

Only xvlog/xelab/xsim are run. This does not run Vivado synthesis/implementation,
install tools, load a bitstream, write /dev/mem, or connect to CAN/robot devices.
A zero xsim exit code is insufficient: Fatal/Error output or missing PASS fails.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
SUITES={
    'ceil':('tb_ceil_div128_64',['rtl/ceil_div128_64.sv','sim/tb_ceil_div128_64.sv']),
    'quiet':('tb_tick_quiet',['rtl/tick_gen.sv','sim/tb_tick_quiet.sv']),
    'tick':('tb_tick_gen',['rtl/tick_gen.sv','sim/tb_tick_gen.sv']),
    'guard':('tb_axi_guard',['rtl/robot_rt_axi_guard.sv','sim/tb_axi_guard.sv']),
    'math':('tb_player_math',['rtl/player_math.sv','sim/tb_player_math.sv']),
    'gate':('tb_limit_gate',['rtl/limit_gate.sv','sim/tb_limit_gate.sv']),
    'cache':('tb_limits_product_cache',['rtl/limits_product_cache.sv','sim/tb_limits_product_cache.sv']),
    'pipe':('tb_limit_gate_pipe',['rtl/limit_gate.sv','rtl/limit_gate_pipe.sv','sim/tb_limit_gate_pipe.sv']),
    'domain':('tb_limit_gate_pipe_narrow_domain',['rtl/limit_gate.sv','rtl/limit_gate_pipe.sv','sim/tb_limit_gate_pipe_narrow_domain.sv']),
    'bounds':('tb_seed_bounds',['rtl/stationary_seed_gate.sv','sim/tb_seed_bounds.sv']),
    'seed':('tb_stationary_seed_gate',['rtl/limit_gate.sv','rtl/stationary_seed_gate.sv','sim/tb_stationary_seed_gate.sv']),
    'seed_seq':('tb_stationary_seed_gate_seq',['rtl/stationary_seed_gate.sv','rtl/stationary_seed_gate_seq.sv','sim/tb_stationary_seed_gate_seq.sv']),
    'shared':('tb_shared_math_gate',['rtl/player_math.sv','rtl/limit_gate_pipe.sv','rtl/shared_math_gate.sv','sim/tb_shared_math_gate.sv']),
    'top':('tb_top_protocol',['rtl/tick_gen.sv','rtl/player_math.sv','rtl/limit_gate.sv',
                            'rtl/limit_gate_pipe.sv','rtl/shared_math_gate.sv','rtl/stationary_seed_gate_seq.sv','rtl/limits_product_cache.sv',
                            'rtl/ceil_div128_64.sv','rtl/bank_validator.sv','rtl/robot_rt_top.sv','sim/tb_top_protocol.sv'])
}


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--suite',choices=(*SUITES,'all'),default='all')
    parser.add_argument('--vivado-bin',type=Path,default=Path('/home/aup/Xilinx/2025.2/Vivado/bin'))
    parser.add_argument('--output-dir',type=Path,required=True,help='New directory; existing evidence is not overwritten')
    parser.add_argument('--timeout',type=int,default=180,help='Per tool invocation seconds')
    parser.add_argument('--random-math-cases',type=int,default=512,help='Seeded wide integer vectors plus 16 directed arithmetic cases')
    parser.add_argument('--clock-hz',type=int,default=100_000_000,help='Compiled nominal RTL clock profile; not measured f_exec')
    args=parser.parse_args()
    if args.timeout<1 or args.random_math_cases<0:parser.error('positive timeout and nonnegative vector count required')
    if not 1 <= args.clock_hz <= 2_147_483_647:parser.error('clock-hz must fit a positive RTL integer')
    suites=list(SUITES) if args.suite=='all' else [args.suite]
    try:
        for name in ('xvlog','xelab','xsim'):
            if not (args.vivado_bin/name).is_file():raise ValueError(f'missing installed tool {name}; no automatic installation')
        for suite in suites:
            for file in SUITES[suite][1]:
                if not (ROOT/file).is_file():raise ValueError(f'{suite}: source is not ready: {file}')
        args.output_dir.mkdir(parents=True,exist_ok=False)
    except (ValueError,OSError) as exc:parser.exit(2,f'HDL suite: {exc}\n')
    summary={'python':sys.version,'hardware_access':False,'vivado_build_run':False,'synthesis_run':False,'implementation_run':False,
             'physical_frequency_verified':False,'clock_hz':args.clock_hz,'suites':{},'sources_sha256':{}}
    for reference in ('sim/player_reference.py','sim/tick_reference.py'):
        summary['sources_sha256'][reference]=hashlib.sha256((ROOT/reference).read_bytes()).hexdigest()
    failed=False
    for suite in suites:
        top,files=SUITES[suite];work=(args.output_dir/suite).resolve();work.mkdir()
        for file in files:summary['sources_sha256'][file]=hashlib.sha256((ROOT/file).read_bytes()).hexdigest()
        record={'top':top,'commands':[],'pass':False};summary['suites'][suite]=record
        commands=[]
        if suite=='ceil':
            commands.append([sys.executable,str(ROOT/'sim/make_ceil_vectors.py'),'--output',str(work/'ceil_vectors.mem')])
            summary['sources_sha256']['sim/make_ceil_vectors.py']=hashlib.sha256((ROOT/'sim/make_ceil_vectors.py').read_bytes()).hexdigest()
        if suite=='math':
            commands.append([sys.executable,str(ROOT/'sim/make_player_vectors.py'),'--output',str(work/'player_vectors.mem'),
                             '--random-cases',str(args.random_math_cases),'--clock-hz',str(args.clock_hz)])
            summary['sources_sha256']['sim/make_player_vectors.py']=hashlib.sha256((ROOT/'sim/make_player_vectors.py').read_bytes()).hexdigest()
        if suite=='gate':
            commands.append([sys.executable,str(ROOT/'sim/make_gate_vectors.py'),'--output',str(work/'gate_vectors.mem'),
                             '--clock-hz',str(args.clock_hz)])
            summary['sources_sha256']['sim/make_gate_vectors.py']=hashlib.sha256((ROOT/'sim/make_gate_vectors.py').read_bytes()).hexdigest()
        if suite=='bounds':
            generator='sim/make_bounds_vectors.py'
            commands.append([sys.executable,str(ROOT/generator),'--output',str(work/'bounds_vectors.mem'),
                             '--clock-hz',str(args.clock_hz)])
            summary['sources_sha256'][generator]=hashlib.sha256((ROOT/generator).read_bytes()).hexdigest()
        if suite=='seed':
            generator='sim/make_seed_vectors.py'
            commands.append([sys.executable,str(ROOT/generator),'--output',str(work/'seed_vectors.mem'),
                             '--clock-hz',str(args.clock_hz)])
            summary['sources_sha256'][generator]=hashlib.sha256((ROOT/generator).read_bytes()).hexdigest()
        if suite=='seed_seq':
            for kind in ('seed','bounds'):
                generator='sim/make_'+kind+'_vectors.py'
                commands.append([sys.executable,str(ROOT/generator),'--output',str(work/(kind+'_vectors.mem')),
                                 '--clock-hz',str(args.clock_hz)])
                summary['sources_sha256'][generator]=hashlib.sha256((ROOT/generator).read_bytes()).hexdigest()
        if suite in ('cache','pipe'):
            generator='sim/make_cache_vectors.py' if suite=='cache' else 'sim/make_gate_pipe_vectors.py'
            commands.append([sys.executable,str(ROOT/generator),'--output',str(work/(suite+'_vectors.mem')),
                             '--clock-hz',str(args.clock_hz)])
            for source in (generator,'sim/make_gate_vectors.py','sim/tb_limit_gate.sv'):
                summary['sources_sha256'][source]=hashlib.sha256((ROOT/source).read_bytes()).hexdigest()
        commands.extend([
            [str(args.vivado_bin/'xvlog'),'--sv',*[str(ROOT/f) for f in files]],
            [str(args.vivado_bin/'xelab'),top,'-s',suite+'_sim','-mt','off',
             *(['-generic_top','CLOCK_HZ='+str(args.clock_hz)] if suite not in ('guard','ceil') else [])],
            [str(args.vivado_bin/'xsim'),suite+'_sim','-runall',
             *(['-testplusarg','VECTORS='+str(work/(suite+'_vectors.mem'))] if suite in ('gate','cache','pipe','seed','bounds') else []),
             *(['-testplusarg','SEED_VECTORS='+str(work/'seed_vectors.mem'),
                '-testplusarg','BOUNDS_VECTORS='+str(work/'bounds_vectors.mem')] if suite=='seed_seq' else [])]])
        for index,command in enumerate(commands):
            try:
                result=subprocess.run(command,cwd=work,capture_output=True,text=True,timeout=args.timeout)
                output=result.stdout+result.stderr;code=result.returncode
            except subprocess.TimeoutExpired as exc:
                output=f'TIMEOUT after {args.timeout}s\n';code=124
                if exc.stdout:output+=exc.stdout.decode(errors='replace') if isinstance(exc.stdout,bytes) else exc.stdout
                if exc.stderr:output+=exc.stderr.decode(errors='replace') if isinstance(exc.stderr,bytes) else exc.stderr
            logfile=work/f'{index:02d}_{Path(command[0]).name}.txt';logfile.write_text(output)
            error=bool(re.search(r'(?im)^\s*(?:fatal\s*:|error\s*:|error\s*\[)|assertion.*failed|TIMEOUT after',output))
            entry={'argv':command,'returncode':code,'error_in_output':error,'log':str(logfile),
                   'log_sha256':hashlib.sha256(logfile.read_bytes()).hexdigest()}
            record['commands'].append(entry)
            if code or error:
                failed=True;break
            if Path(command[0]).name=='xsim':
                record['pass']=bool(re.search(r'(?m)^PASS\b',output))
                if not record['pass']:failed=True
        if not record['pass']:failed=True
        print(json.dumps({'suite':suite,'pass':record['pass'],'output_directory':str(work)}),flush=True)
    summary['pass']=not failed
    (args.output_dir/'verification.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    if failed:parser.exit(1,'HDL verification failed; see the saved logs, not just process exit codes.\n')


if __name__=='__main__':main()

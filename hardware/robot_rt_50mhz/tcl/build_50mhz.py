#!/usr/bin/python3
"""Build the existing robot_rt design at50MHz, using its split synthesis flow."""
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tcl'))
from compile_children import declaration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-out', type=Path, required=True)
    parser.add_argument('--work', type=Path, required=True)
    parser.add_argument('--artifacts-out', type=Path, required=True)
    parser.add_argument('--resume', action='store_true', help='Continue completed stages in the same build')
    args = parser.parse_args()
    project, work, artifacts = [p.resolve() for p in (args.project_out, args.work, args.artifacts_out)]
    if not args.resume and any(p.exists() for p in (project, work, artifacts)):
        parser.error('Choose new project/work/artifact directories')
    work.mkdir(parents=True, exist_ok=args.resume)
    vivado = Path('/home/aup/Xilinx/2025.2/Vivado/bin/vivado')
    status = {'requested_clock_hz': 50000000, 'stage': 'prepare', 'completed': [], 'pass': False}
    if args.resume:
        status = json.loads((work / 'status.json').read_text())
        status.pop('error', None)

    def save():
        (work / 'status.json').write_text(json.dumps(status, ensure_ascii=False, indent=2) + '\n')

    def run(label, command, cwd=None):
        if label in status['completed']:
            return
        status['stage'] = label
        save()
        print(time.strftime('%Y-%m-%d %H:%M:%S'), label, flush=True)
        logfile = work / (label + '.log')
        with logfile.open('w') as stream:
            result = subprocess.run(command, cwd=cwd or work, stdout=stream, stderr=subprocess.STDOUT)
        if cwd is not None and (Path(cwd) / 'runme.log').is_file():
            shutil.copy2(Path(cwd) / 'runme.log', logfile)
        output = logfile.read_text(errors='replace')
        if result.returncode or re.search(r'(?m)^(?:ERROR:|FATAL:)', output):
            raise RuntimeError(f'{label} failed (rc={result.returncode}): {logfile}')
        status['completed'].append(label)
        save()

    def tool(label, script):
        run(label, [str(vivado), '-mode', 'batch', '-notrace', '-source', str(script),
                    '-log', str(work / (label + '.vivado.log')),
                    '-journal', str(work / (label + '.jou'))])

    def phase(label):
        run(label, [str(vivado), '-mode', 'batch', '-notrace', '-source', str(ROOT / 'tcl/build_50mhz.tcl'),
                    '-log', str(work / (label + '.vivado.log')), '-journal', str(work / (label + '.jou')),
                    '-tclargs', label, str(project), str(work), str(artifacts)])

    try:
        for module in ('player_math', 'limit_gate_pipe'):
            header = declaration((ROOT / 'rtl' / (module + '.sv')).read_text(), module)
            (work / (module + '_stub.sv')).write_text('(* black_box = "true" *)\n' + header + '\nendmodule\n')
        if args.resume and (project / 'addresses.json').exists() and 'prepare' not in status['completed']:
            phase('plan')
            status['completed'].append('prepare')
        else:
            phase('prepare')
        addresses = json.loads((project / 'addresses.json').read_text())
        clock = addresses['clock_hz']
        status['compiled_clock_hz'] = clock
        save()
        for module in ('player_math', 'limit_gate_pipe'):
            directory = work / module
            directory.mkdir(exist_ok=args.resume)
            (directory / 'clock.xdc').write_text('create_clock -name child_ooc_clk -period 20.000 [get_ports clk]\n')
            generic = f'CLOCK_HZ={clock}' + (' STABLE_JOB_INPUTS=1' if module == 'limit_gate_pipe' else '')
            script = directory / 'compile.tcl'
            script.write_text(f'''set_param general.maxThreads 1
create_project -in_memory -part xck26-sfvc784-2LV-c
read_verilog -sv {{{ROOT / 'rtl' / (module + '.sv')}}}
read_xdc {{{directory / 'clock.xdc'}}}
synth_design -top {module} -part xck26-sfvc784-2LV-c -mode out_of_context -generic {{{generic}}} -directive runtimeoptimized
report_utilization -file {{{directory / 'utilization.rpt'}}}
reset_timing -invalid
if {{[llength [get_clocks -quiet *]]}} {{error "Child clock was not removed"}}
write_checkpoint -force {{{directory / (module + '.dcp')}}}
write_verilog -force -mode funcsim {{{directory / (module + '_synth.v')}}}
puts {{PASS CHILD_50MHZ {module} CLOCK_HZ={clock}}}
exit 0
''')
            tool('child_' + module, script)
            if not (directory / (module + '.dcp')).is_file():
                raise RuntimeError('Missing synthesized child ' + module)
        hook = work / 'assemble_children.tcl'
        hook.write_text(f'''# Same bulk-link sequence as the existing partition_bulk_hook.
# A synthesized OOC design is first published, then linked with its children
# in a fresh in-memory project. read_checkpoint -cell is not used here.
write_checkpoint -force -noxdef {{{work / 'core_unfilled.dcp'}}}
close_project
create_project -in_memory -part xck26-sfvc784-2LV-c
read_checkpoint {{{work / 'core_unfilled.dcp'}}}
''' + '\n'.join(f'read_checkpoint {{{work / module / (module + ".dcp")}}}'
                        for module in ('player_math', 'limit_gate_pipe')) + '''
link_design -top robot_rt_bd_core_0 -part xck26-sfvc784-2LV-c -mode out_of_context
if {[llength [get_cells -hier -filter {IS_BLACKBOX == 1}]]} {error "Unfilled core partition"}
reset_timing -invalid
if {[llength [get_clocks -quiet *]]} {error "Temporary OOC clock remained"}
puts {PASS ASSEMBLED_50MHZ_CHILDREN}
''')
        plan = json.loads((work / 'run_plan.json').read_text())
        for name, path in plan.items():
            directory = Path(path)
            if name == 'robot_rt_bd_core_0_synth_1':
                script = directory / 'robot_rt_bd_core_0.tcl'
                original = script.read_text()
                (work / 'core_worker_original.tcl').write_text(original)
                if f'source {{{hook}}}' not in original:
                    patched, count = re.subn(r'(?m)^(synth_design[^\n]*\n)',
                                            lambda m: m[1] + f'source {{{hook}}}\n', original)
                    if count != 1:
                        raise RuntimeError('Expected exactly one core synth_design command')
                    script.write_text(patched)
            run('synth_' + name, ['bash', str(directory / 'runme.sh')], cwd=directory)
        phase('implement')
        status['stage'] = 'complete'
        status['pass'] = True
        print('50MHZ_BUILD_COMPLETE', artifacts, flush=True)
    except BaseException as exc:
        status['error'] = str(exc)
        raise
    finally:
        save()


if __name__ == '__main__':
    main()

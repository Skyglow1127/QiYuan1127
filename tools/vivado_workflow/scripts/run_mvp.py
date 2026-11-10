#!/usr/bin/env python3
"""Reproduce the LED MVP using the installed Ross MCP server and Vivado."""
import json
from pathlib import Path
import re
import shutil
import time
from datetime import datetime

from ross_mcp import ROOT, Ross

PART = "xczu2cg-sbva484-1-e"
PASS_MARKER = "TEST_PASS: divisors=1,3,5 checks=240 reset_restarts=2 clock=100MHz"


def memory_guard(minimum_gib):
    values = {line.split(':')[0]: int(line.split()[1])
              for line in Path('/proc/meminfo').read_text().splitlines()}
    available = values['MemAvailable'] / 1024**2
    print(f"Available RAM: {available:.1f} GiB; required: {minimum_gib} GiB", flush=True)
    if available < minimum_gib:
        raise RuntimeError("Insufficient available RAM; close unused applications and retry. "
                           "No process is killed automatically.")


def unpack(result):
    texts = [c['text'] for c in result.get('content', []) if c.get('type') == 'text']
    if len(texts) != 1:
        raise RuntimeError(f"Unexpected MCP result: {result}")
    try:
        return json.loads(texts[0])
    except json.JSONDecodeError:
        return {'output': texts[0]}


def main():
    memory_guard(8)
    run = ROOT / 'build' / datetime.now().strftime('run_%Y%m%d_%H%M%S')
    run.mkdir(parents=True)
    evidence = run / 'evidence'
    evidence.mkdir()
    session = None
    active = 'setup'
    summary = {'status': 'RUNNING', 'scope': 'RTL simulation, lint and synthesis only',
               'part': PART, 'run_directory': str(run)}
    with Ross() as ross:
        def call(name, args):
            result = unpack(ross.call(name, args))
            if result.get('exit_code', 0) != 0 or result.get('status') in ('error', 'failed'):
                raise RuntimeError(json.dumps(result, ensure_ascii=False))
            return result

        def tcl(command):
            print(f"Vivado: {command.split(';')[0]}", flush=True)
            result = call('vivado_execute', {'session_id': session, 'command': command,
                                            'verbosity': 'full'})
            while result.get('status') == 'running' or result.get('is_command_running'):
                time.sleep(30)
                result = call('vivado_status', {'session_id': session, 'action': 'session'})
                if not result.get('is_command_running'):
                    if result.get('last_completed_code') != 0:
                        raise RuntimeError(f"Vivado command failed or result unknown: {result}")
                    break
            return result.get('output', '')

        def task(identifier, status, description=''):
            call('vivado_todos', {'operation': 'update', 'session_id': session,
                 'id': identifier, 'status': status,
                 **({'reason': description} if status == 'failed' else {'summary': description})})

        try:
            session = call('vivado_start', {'working_dir': str(run),
                           'session_type': 'general', 'gui_mode': False})['session_id']
            summary['session_id'] = session
            summary['session_close_policy'] = 'manual'
            call('vivado_todos', {'operation': 'plan', 'session_id': session,
                 'title': 'Ross LED MVP', 'tasks': [
                     {'id': 'setup', 'content': '创建工程与检查器件'},
                     {'id': 'simulation', 'content': 'XSim 自检仿真'},
                     {'id': 'lint', 'content': 'RTL Lint 与报告解析'},
                     {'id': 'synthesis', 'content': '综合与报告导出'}]})
            task(active, 'active')
            version = tcl('set_param general.maxThreads 2; puts [version -short]').strip()
            summary['vivado_version_output'] = version
            tcl(f'if {{[llength [get_parts -quiet {PART}]] != 1}} {{error "Required part is not installed"}}')
            tcl('; '.join([
                f'create_project led_mvp {{{run}/project}} -part {PART}',
                f'add_files -norecurse {{{ROOT}/rtl/led_blink.sv}}',
                'set_property top led_blink [get_filesets sources_1]',
                f'add_files -fileset sim_1 -norecurse {{{ROOT}/tb/tb_led_blink.sv}}',
                'set_property top tb_led_blink [get_filesets sim_1]',
                f'add_files -fileset constrs_1 -norecurse {{{ROOT}/constraints/clock.xdc}}',
                'set_property target_simulator XSim [current_project]',
                'set_property xsim.simulate.runtime 2us [get_filesets sim_1]',
                'set_property xsim.simulate.log_all_signals true [get_filesets sim_1]',
                f'set_property xsim.simulate.wdb {{{evidence}/led_blink.wdb}} [get_filesets sim_1]',
                'update_compile_order -fileset sources_1',
                'update_compile_order -fileset sim_1']))
            summary['project'] = str(run / 'project/led_mvp.xpr')
            task(active, 'done')
            active = 'simulation'
            task(active, 'active')
            sim_output = tcl('launch_simulation -simset sim_1 -mode behavioral')
            (evidence / 'simulation_command.txt').write_text(sim_output)
            sim = run / 'project/led_mvp.sim/sim_1/behav/xsim'
            logs = {}
            for name in ('compile.log', 'elaborate.log', 'simulate.log'):
                logs[name] = (sim / name).read_text()
                shutil.copy2(sim / name, evidence / name)
                if re.search(r'^\s*(ERROR:|FATAL:|Fatal:)', logs[name], re.M):
                    raise RuntimeError(f"Simulation failure in {name}")
            if logs['simulate.log'].count(PASS_MARKER) != 1:
                raise RuntimeError('Missing unique TEST_PASS marker')
            summary['simulation'] = {'status': 'PASS', 'checks': 240, 'reset_restarts': 2}
            tcl('close_sim -force')
            task(active, 'done', PASS_MARKER)
            active = 'lint'
            task(active, 'active')
            lint = run / 'project/vivado_agentic_ai_reports/rtl-lint'
            tcl(f'file mkdir {{{lint}}}')
            output = tcl(f'synth_design -top led_blink -part {PART} -lint -file {{{lint}/linter.rpt}}')
            (evidence / 'lint_command.txt').write_text(output)
            count = re.search(r'Total of (\d+) linter message', output)
            if not count:
                raise RuntimeError('Missing Vivado lint message count')
            tcl(f'exec python3 {{{ROOT}/scripts/parse_lint_report.py}} {{{lint}/linter.rpt}} {{{lint}/linter.csv}}')
            # Do not group or reinterpret violations here. A nonempty report
            # needs an engineer to follow the Ross lint resolution workflow.
            if int(count[1]) != 0 or (lint / 'linter.csv').read_text().strip():
                raise RuntimeError('Lint needs review: tool count or canonical CSV is nonzero')
            for name in ('linter.rpt', 'linter.csv'):
                shutil.copy2(lint / name, evidence / name)
            summary['lint'] = {'status': 'PASS', 'violations': 0}
            task(active, 'done', '0 lint violations')
            active = 'synthesis'
            task(active, 'active')
            memory_guard(6)
            tcl('launch_runs synth_1 -jobs 1')
            tcl('wait_on_run synth_1')
            status = tcl('puts "SYNTH_STATUS=[get_property STATUS [get_runs synth_1]]"')
            if 'synth_design Complete!' not in status:
                raise RuntimeError(f'Synthesis did not complete: {status}')
            tcl('open_run synth_1')
            tcl(f'report_utilization -file {{{evidence}/utilization.rpt}}; '
                f'report_timing_summary -file {{{evidence}/timing_summary.rpt}}')
            tcl(f'write_checkpoint {{{evidence}/led_blink_synth.dcp}}')
            shutil.copy2(run / 'project/led_mvp.runs/synth_1/runme.log', evidence / 'synthesis.log')
            summary['synthesis'] = {'status': 'PASS'}
            timing = (evidence / 'timing_summary.rpt').read_text()
            summary['timing'] = {
                'stage': 'post_synthesis', 'signoff': 'NOT_PERFORMED',
                'constraints_met': (False if 'Timing constraints are not met.' in timing else
                                    True if 'All user specified timing constraints are met.' in timing else None),
                'board_io_constraints_complete': False,
                'report': str(evidence / 'timing_summary.rpt')}
            task(active, 'done', 'Synthesis, utilization, timing estimate and DCP exported')
            summary['status'] = 'PASS'
        except BaseException as exc:
            summary['status'] = 'FAIL'
            summary['error'] = str(exc)
            if session:
                try:
                    task(active, 'failed', str(exc)[:1000])
                except Exception:
                    pass
            raise
        finally:
            (evidence / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
            (ROOT / 'reports/latest_run.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
            if session:
                print(f"Vivado session {session}: left open for manual closing in Ross Sessions.",
                      flush=True)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()

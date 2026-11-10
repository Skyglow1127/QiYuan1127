#!/usr/bin/env python3
"""Build and debug the KR260 ILA project through the installed Ross MCP bridge."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import shutil
import time

from ross_mcp import ROOT, Ross
from run_mvp import memory_guard, unpack, PASS_MARKER
from vivado_memory_scope import protect_vivado

PART = 'xck26-sfvc784-2LV-c'
LATEST = ROOT / 'reports/kr260_latest.json'
KR_PASS = 'TEST_PASS: KR260 checks=70000 startup_cycles=255 counter_wrap=verified half_period=64 clock=25MHz'


class Flow:
    def __init__(self, ross, state):
        self.ross, self.state = ross, state
        self.run = Path(state['run_directory'])
        self.evidence = self.run / 'evidence'
        self.evidence.mkdir(parents=True, exist_ok=True)

    def save(self):
        text = json.dumps(self.state, ensure_ascii=False, indent=2)
        (self.evidence / 'summary.json').write_text(text)
        LATEST.write_text(text)

    def call(self, name, **args):
        result = unpack(self.ross.call(name, args))
        if result.get('exit_code', 0) != 0 or result.get('status') in ('error', 'failed'):
            raise RuntimeError(json.dumps(result, ensure_ascii=False))
        return result

    def tcl(self, command):
        print('Vivado:', command[:140], flush=True)
        # Ross deduplicates identical writes for two minutes, even when a new
        # simulation/run has changed the state. Tag each intentional invocation
        # so close_sim/wait_on_run cannot silently reuse an earlier completion.
        # GUI transitions have a Ross transport-switch hook and must stay exact.
        if command.strip() not in ('start_gui', 'stop_gui'):
            command += f'; # ross invocation {time.time_ns()}'
        result = self.call('vivado_execute', session_id=self.state['session_id'],
                           command=command, verbosity='full')
        while result.get('status') == 'running' or result.get('is_command_running'):
            time.sleep(30)
            result = self.call('vivado_status', session_id=self.state['session_id'], action='session')
            if not result.get('is_command_running'):
                if result.get('last_completed_code') != 0:
                    raise RuntimeError(f'Command failed or completion unknown: {result}')
                break
        with (self.evidence / 'commands.jsonl').open('a') as out:
            out.write(json.dumps({'command': command, 'result': result}, ensure_ascii=False) + '\n')
        return result.get('output', '')

    def step(self, name, action):
        if name in self.state.setdefault('completed', []):
            return
        self.state['active_step'] = name
        self.save()
        self.call('vivado_todos', operation='update', session_id=self.state['session_id'],
                  id=name, status='active')
        action()
        self.state['completed'].append(name)
        self.save()
        self.call('vivado_todos', operation='update', session_id=self.state['session_id'],
                  id=name, status='done', summary=f'{name} verified; evidence saved')

    def setup(self):
        self.tcl('set_param general.maxThreads 2; puts [version -short]')
        self.tcl(f'set_param board.repoPaths [list {{{ROOT}/boards/Xilinx}}]; '
                 'puts [get_board_parts *kr260*]')
        self.tcl(f'create_project kr260_ila {{{self.run}/project}} -part {PART}')
        self.tcl('; '.join([
            'set_property board_part xilinx.com:kr260_som:part0:1.1 [current_project]',
            f'add_files -norecurse [list {{{ROOT}/rtl/led_blink.sv}} {{{ROOT}/rtl/kr260_ila_top.sv}}]',
            'set_property top kr260_ila_top [get_filesets sources_1]',
            f'add_files -fileset constrs_1 -norecurse [list {{{ROOT}/constraints/kr260_board.xdc}} {{{ROOT}/constraints/kr260_ila_debug.xdc}}]',
            'set_property USED_IN_SYNTHESIS false [get_files kr260_ila_debug.xdc]',
            'set_property PROCESSING_ORDER LATE [get_files kr260_ila_debug.xdc]',
            f'add_files -fileset sim_1 -norecurse {{{ROOT}/tb/tb_kr260_ila.sv}}',
            'set_property top tb_kr260_ila [get_filesets sim_1]',
            'create_fileset -simset sim_baseline',
            f'add_files -fileset sim_baseline -norecurse {{{ROOT}/tb/tb_led_blink.sv}}',
            'set_property top tb_led_blink [get_filesets sim_baseline]',
            'set_property target_simulator XSim [current_project]',
            'update_compile_order -fileset sources_1',
            'update_compile_order -fileset sim_1',
            'update_compile_order -fileset sim_baseline']))
        self.state['project'] = str(self.run / 'project/kr260_ila.xpr')
        self.state['source_sha256'] = {
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [ROOT/'rtl/led_blink.sv', ROOT/'rtl/kr260_ila_top.sv',
                      ROOT/'constraints/kr260_board.xdc', ROOT/'constraints/kr260_ila_debug.xdc',
                      ROOT/'tb/tb_kr260_ila.sv']}

    def simulate(self):
        for simset, runtime, marker in [('sim_baseline', '2us', PASS_MARKER),
                                        ('sim_1', '3ms', KR_PASS)]:
            dest = self.evidence / simset
            dest.mkdir(exist_ok=True)
            self.tcl(f'set_property xsim.simulate.runtime {runtime} [get_filesets {simset}]; '
                     f'set_property xsim.simulate.log_all_signals true [get_filesets {simset}]; '
                     f'set_property xsim.simulate.wdb {{{dest}/waveform.wdb}} [get_filesets {simset}]')
            self.tcl(f'launch_simulation -simset {simset} -mode behavioral')
            sim = self.run / f'project/kr260_ila.sim/{simset}/behav/xsim'
            for filename in ('compile.log', 'elaborate.log', 'simulate.log'):
                log = (sim / filename).read_text()
                shutil.copy2(sim / filename, dest / filename)
                if re.search(r'^\s*(ERROR:|FATAL:|Fatal:)', log, re.M):
                    raise RuntimeError(f'Simulation error in {sim / filename}')
            if (dest/'simulate.log').read_text().count(marker) != 1:
                raise RuntimeError(f'Missing unique test PASS in {simset}')
            self.tcl('close_sim -force')
        self.state['simulation'] = {'status': 'PASS', 'baseline_checks': 240, 'kr260_checks': 70000}

    def synthesize(self):
        memory_guard(3)
        self.tcl('launch_runs synth_1 -jobs 1')
        self.tcl('wait_on_run synth_1')
        output = self.tcl('puts [get_property STATUS [get_runs synth_1]]')
        if 'synth_design Complete!' not in output:
            raise RuntimeError('Synthesis incomplete: ' + output)
        self.state['synthesis'] = 'PASS'

    def implement(self):
        memory_guard(3)
        self.tcl('launch_runs impl_1 -to_step route_design -jobs 1')
        self.tcl('wait_on_run impl_1; # route_design stage')
        output = self.tcl('puts [get_property STATUS [get_runs impl_1]]')
        if 'route_design Complete!' not in output:
            raise RuntimeError('Implementation incomplete: ' + output)
        self.tcl('open_run impl_1')
        self.tcl(f'report_timing_summary -report_unconstrained -file {{{self.evidence}/timing_summary.rpt}}; '
                 f'report_utilization -file {{{self.evidence}/utilization.rpt}}; '
                 f'report_drc -file {{{self.evidence}/drc.rpt}}; '
                 f'report_methodology -file {{{self.evidence}/methodology.rpt}}; '
                 f'report_route_status -file {{{self.evidence}/route_status.rpt}}')
        timing = (self.evidence/'timing_summary.rpt').read_text()
        if 'All user specified timing constraints are met.' not in timing:
            raise RuntimeError('Implementation timing failed; see timing_summary.rpt')
        self.tcl('set violations [get_drc_violations -filter {SEVERITY == Error || SEVERITY == "Critical Warning"}]; '
                 'if {[llength $violations]} {error "DRC blocking violations: $violations"}; '
                 'puts "DRC_BLOCKERS=0"')
        self.tcl(f'write_debug_probes -force {{{self.evidence}/kr260_ila.ltx}}')
        self.state['implementation'] = 'PASS'

    def bitstream(self):
        memory_guard(3)
        self.tcl('launch_runs impl_1 -to_step write_bitstream -jobs 1')
        self.tcl('wait_on_run impl_1; # write_bitstream stage')
        output = self.tcl('puts [get_property STATUS [get_runs impl_1]]')
        if 'write_bitstream Complete!' not in output:
            raise RuntimeError('Bitstream incomplete: ' + output)
        source = self.run/'project/kr260_ila.runs/impl_1/kr260_ila_top.bit'
        dest = self.evidence/'kr260_ila.bit'
        shutil.copy2(source, dest)
        self.state['bitstream'] = str(dest)
        self.state['probes'] = str(self.evidence/'kr260_ila.ltx')
        self.state['artifact_sha256'] = {
            name: hashlib.sha256(Path(self.state[name]).read_bytes()).hexdigest()
            for name in ('bitstream', 'probes')}
        self.state['status'] = 'BUILD_PASS'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume', type=Path, help='Resume an existing build summary.json')
    args = parser.parse_args()
    with Ross() as ross:
        if args.resume:
            state = json.loads(args.resume.read_text())
        else:
            memory_guard(6)
            run = ROOT/'build'/datetime.now().strftime('kr260_ila_%Y%m%d_%H%M%S')
            run.mkdir(parents=True)
            result = unpack(ross.call('vivado_start', {'working_dir': str(run),
                'session_type': 'general', 'gui_mode': False, 'display_mode': 'native'}))
            if 'session_id' not in result:
                raise RuntimeError(str(result))
            state = {'status': 'RUNNING', 'part': PART, 'run_directory': str(run),
                     'session_id': result['session_id'], 'session_close_policy': 'manual',
                     'hardware_capture': 'NOT_PERFORMED'}
            Flow(ross, state).save()
            # The parent session and every subsequently launched build child
            # share this limit. Nothing closes a completed session automatically.
            state['memory_scope'] = protect_vivado(result['process_id'])
            ross.call('vivado_todos', {'operation': 'plan', 'session_id': state['session_id'],
                'title': 'KR260 ILA board verification', 'tasks': [
                    {'id': k, 'content': v} for k,v in [
                        ('setup','配置 KR260 工程、管脚与 ILA'), ('simulation','验证原模块与 KR260 顶层'),
                        ('synthesis','综合'), ('implementation','布局布线与时序检查'),
                        ('bitstream','生成 BIT 与 LTX'), ('hardware','JTAG 下载与真实 ILA 波形验证')]]})
        flow = Flow(ross, state)
        try:
            if args.resume:
                flow.call('vivado_todos', operation='clear', session_id=state['session_id'])
                flow.call('vivado_todos', operation='plan', session_id=state['session_id'],
                          title='KR260 ILA board verification', tasks=[
                    {'id': k, 'content': v} for k,v in [
                        ('setup','配置 KR260 工程、管脚与 ILA'), ('simulation','验证原模块与 KR260 顶层'),
                        ('synthesis','综合'), ('implementation','布局布线与时序检查'),
                        ('bitstream','生成 BIT 与 LTX'), ('hardware','JTAG 下载与真实 ILA 波形验证')]])
                for name in state.get('completed', []):
                    flow.call('vivado_todos', operation='update', session_id=state['session_id'],
                              id=name, status='done', summary='Completed evidence retained')
            state.pop('error', None)
            if 'bitstream' not in state.get('completed', []):
                state['status'] = 'RUNNING'
            flow.save()
            for name, action in [('setup',flow.setup), ('simulation',flow.simulate),
                                 ('synthesis',flow.synthesize), ('implementation',flow.implement),
                                 ('bitstream',flow.bitstream)]:
                flow.step(name, action)
        except BaseException as exc:
            state['status'] = 'FAILED'
            state['error'] = str(exc)
            raise
        finally:
            flow.save()
            print(f"Vivado session left open: {state['session_id']}", flush=True)
    print(json.dumps(state, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

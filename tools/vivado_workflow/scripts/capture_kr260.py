#!/usr/bin/env python3
"""Program the connected KR260 and verify real ILA samples, using Ross MCP."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from ross_mcp import ROOT, Ross
from run_kr260 import Flow, LATEST
from run_mvp import unpack, memory_guard
from vivado_memory_scope import protect_vivado


def analyze_capture(path):
    rows = list(csv.reader(path.open(newline='')))
    header_index = next(i for i,r in enumerate(rows) if any('Sample in Buffer' in c for c in r))
    header = rows[header_index]
    columns = {signal: next(i for i,c in enumerate(header) if signal in c)
               for signal in ('cycle_count', 'blink', 'rst_n')}
    # Vivado ILA exports data probes in hexadecimal by default. Sample indices
    # are decimal. Inspect the radix row and reject unsupported encodings.
    radix = {i: 16 for i in columns.values()}
    samples = []
    for row in rows[header_index+1:]:
        if not row or len(row) != len(header):
            continue
        if not row[0].strip().isdigit():
            for i in columns.values():
                value = row[i].strip().upper()
                if value in ('HEX', 'HEXADECIMAL'): radix[i] = 16
                elif value in ('UNSIGNED', 'SIGNED', 'DEC', 'DECIMAL'): radix[i] = 10
                elif value in ('BIN', 'BINARY'): radix[i] = 2
            continue
        samples.append({'sample': int(row[0]), **{
            signal: int(row[i].strip(), radix[i]) for signal,i in columns.items()}})
    if len(samples) != 1024:
        raise RuntimeError(f'Expected 1024 hardware samples, got {len(samples)}')
    transitions = []
    for i,sample in enumerate(samples):
        if sample['sample'] != i:
            raise RuntimeError(f'Noncontiguous ILA sample index at {i}')
        if sample['rst_n'] != 1:
            raise RuntimeError(f'Reset not released at sample {i}')
        if sample['blink'] != ((sample['cycle_count'] // 64) % 2):
            raise RuntimeError(f'Toggle phase mismatch at sample {i}: {sample}')
        if i:
            previous = samples[i-1]
            if sample['cycle_count'] != (previous['cycle_count']+1) % 65536:
                raise RuntimeError(f'Counter continuity failure at sample {i}')
            if sample['blink'] != previous['blink']:
                transitions.append(i)
    if len(transitions) < 15 or any(b-a != 64 for a,b in zip(transitions,transitions[1:])):
        raise RuntimeError(f'Unexpected toggle spacing: {transitions}')
    return samples, {'status':'PASS', 'sample_count':len(samples),
                     'counter_transitions_checked':len(samples)-1,
                     'toggle_edges':transitions, 'half_period_cycles':64,
                     'nominal_clock_hz':25000000,
                     'first_counter':samples[0]['cycle_count'],
                     'last_counter':samples[-1]['cycle_count']}


def plot_capture(samples, dest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.style.use('dark_background')
    fig, axes = plt.subplots(3, 1, figsize=(13, 6.3), sharex=True,
                             gridspec_kw={'height_ratios':[2,1,1]}, layout='constrained')
    t = [s['sample'] * .04 for s in samples]
    axes[0].step(t, [s['cycle_count'] for s in samples], where='post', color='#65d5ff')
    axes[0].set_ylabel('cycle_count\n16-bit unsigned')
    for ax, signal, color in zip(axes[1:], ['blink','rst_n'], ['#78e7a0','#f6c36c']):
        ax.step(t, [s[signal] for s in samples], where='post', color=color, linewidth=1.7)
        ax.set_ylabel(signal);ax.set_ylim(-.15,1.2);ax.set_yticks([0,1])
    for ax in axes:
        ax.axvline(128*.04,color='#ff7a90',linestyle='--',alpha=.8)
        ax.grid(alpha=.14)
    axes[-1].set_xlabel('Time (us), using the board nominal 25 MHz clock')
    axes[0].set_title('KR260 | Actual JTAG / ILA capture | 1024 samples | verified PASS',loc='left',pad=14)
    fig.savefig(dest, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default='XFL10TJQWDIB', help='Exact KR260 USB/JTAG serial')
    parser.add_argument('--show', action='store_true', help='Show captured data in the existing Vivado session')
    parser.add_argument('--show-only', action='store_true', help='Open saved ILA capture without programming hardware')
    parser.add_argument('--capture-only', action='store_true', help='Capture current matching programmed design without reprogramming')
    args = parser.parse_args()
    # Serial is a selector, never arbitrary Tcl.
    if not args.serial.isalnum():
        raise RuntimeError('JTAG serial must be alphanumeric')
    state = json.loads(LATEST.read_text())
    if 'bitstream' not in state.get('completed', []):
        raise RuntimeError('Finish the KR260 build before programming')
    if not args.show_only:
        for filename, digest in state['source_sha256'].items():
            if hashlib.sha256((ROOT/filename).read_bytes()).hexdigest() != digest:
                raise RuntimeError('Source changed since build: ' + filename)
        for name,digest in state['artifact_sha256'].items():
            if hashlib.sha256(Path(state[name]).read_bytes()).hexdigest() != digest:
                raise RuntimeError('Build artifact changed: ' + name)
    capture = Path(state['run_directory'])/'vivado_agentic_ai_reports/hw-ila-debug'
    capture.mkdir(parents=True, exist_ok=True)
    with Ross() as ross:
        sessions = unpack(ross.call('vivado_list_sessions', {})).get('sessions', {})
        if state['session_id'] not in sessions:
            memory_guard(6)
            started = unpack(ross.call('vivado_start', {'working_dir':state['run_directory'],
                             'session_type':'general','gui_mode':False,'display_mode':'native'}))
            state['session_id'] = started['session_id']
            state['memory_scope'] = protect_vivado(started['process_id'])
            restored = Flow(ross,state)
            restored.save()
            restored.tcl(f'open_project {{{state["project"]}}}')
        flow = Flow(ross, state)
        if args.show or args.show_only:
            current_sessions = unpack(ross.call('vivado_list_sessions', {})).get('sessions', {})
            if not current_sessions.get(state['session_id'], {}).get('is_gui_mode'):
                flow.tcl('start_gui')
                # A Hardware Manager opened in Tcl mode does not acquire its
                # GUI panes automatically. Reopen its context after switching.
                # This leaves the Vivado application and programmed PL running.
                flow.tcl('close_hw_manager')
            flow.tcl('open_hw_manager')
        if args.show_only:
            saved = capture/'capture_data.ila'
            if not saved.is_file():
                raise RuntimeError('No saved hardware capture exists yet')
            flow.tcl(f'display_hw_ila_data [read_hw_ila_data {{{saved}}}]')
            return
        plan = flow.call('vivado_todos', operation='read', session_id=state['session_id'])
        tasks = plan.get('tasks', [])
        hardware_task = next((t for t in tasks if t['id'] == 'hardware'), None)
        if hardware_task is None or hardware_task['status'] not in ('pending', 'active'):
            # Ross permits activation only from pending. Recreate the plan on
            # a later capture, retaining completed build steps in the sidebar.
            tasks = [t for t in tasks if t['id'] != 'hardware']
            flow.call('vivado_todos', operation='clear', session_id=state['session_id'])
            flow.call('vivado_todos', operation='plan', session_id=state['session_id'],
                      title='KR260 ILA board verification',
                      tasks=[{'id':t['id'], 'content':t['content']} for t in tasks] +
                            [{'id':'hardware', 'content':'JTAG 下载与真实 ILA 波形验证'}])
            for task in tasks:
                if task['status'] == 'done':
                    flow.call('vivado_todos', operation='update', session_id=state['session_id'],
                              id=task['id'], status='done', summary=task.get('summary','Evidence retained'))
            hardware_task = None
        if hardware_task is None or hardware_task['status'] == 'pending':
            flow.call('vivado_todos',operation='update',session_id=state['session_id'],id='hardware',status='active')
        try:
            flow.tcl('open_hw_manager')
            server = flow.tcl('if {[llength [get_hw_servers]] == 0} {connect_hw_server -url localhost:3121}; '
                              'refresh_hw_server [lindex [get_hw_servers] 0]; get_hw_servers')
            (capture/'server.txt').write_text(server)
            flow.tcl(f'set targets [get_hw_targets *{args.serial}*]; '
                     'if {[llength $targets] != 1} {error "Expected exactly one requested KR260 JTAG target"}; '
                     'current_hw_target [lindex $targets 0]; '
                     'set_property PARAM.FREQUENCY 5000000 [current_hw_target]; '
                     'if {![get_property IS_OPENED [current_hw_target]]} {open_hw_target}')
            device = flow.tcl('set candidates [get_hw_devices -filter {PART =~ xck26*}]; '
                             'if {[llength $candidates] != 1} {error "Expected exactly one K26 device"}; '
                             'set kr_device [lindex $candidates 0]; current_hw_device $kr_device; '
                             'format "%s\\nPART=%s" [get_hw_devices] [get_property PART $kr_device]')
            (capture/'device.txt').write_text(device)
            flow.tcl(f'set_property PROBES.FILE {{{state["probes"]}}} $kr_device; '
                     f'set_property PROGRAM.FILE {{{state["bitstream"]}}} $kr_device')
            if not args.capture_only:
                flow.tcl('program_hw_devices $kr_device')
            flow.tcl('refresh_hw_device $kr_device; set cores [get_hw_ilas -of_objects $kr_device]; '
                     'if {[llength $cores] != 1} {error "Expected exactly one ILA core"}; '
                     'set kr_ila [lindex $cores 0]; puts "CORE=$kr_ila"; '
                     'puts "DEPTH=[get_property STATIC.MAX_DATA_DEPTH $kr_ila]"; '
                     'puts [get_hw_probes -of_objects $kr_ila]')
            flow.tcl('set_property CONTROL.DATA_DEPTH 1024 $kr_ila; '
                     'set_property CONTROL.TRIGGER_POSITION 128 $kr_ila; '
                     'set_property CONTROL.TRIGGER_CONDITION AND $kr_ila; '
                     'if {[get_property CONTROL.TRIGGER_MODE $kr_ila] ne "BASIC_ONLY"} '
                     '{error "Expected basic-only ILA trigger mode"}')
            # First establish clock/probe aliveness with a bounded immediate capture.
            flow.tcl('run_hw_ila -trigger_now $kr_ila; wait_on_hw_ila -timeout 1 $kr_ila; '
                     f'write_hw_ila_data -force -csv_file {{{capture}/aliveness.csv}} [upload_hw_ila_data $kr_ila]')
            analyze_capture(capture/'aliveness.csv')
            flow.tcl('set blink_probe [get_hw_probes -of_objects $kr_ila -filter {NAME =~ *blink*}]; '
                     'if {[llength $blink_probe] != 1} {error "Blink probe not uniquely identified"}; '
                     'set_property TRIGGER_COMPARE_VALUE eq1\'bR $blink_probe')
            flow.tcl('run_hw_ila $kr_ila; wait_on_hw_ila -timeout 1 $kr_ila; '
                     'set kr_data [upload_hw_ila_data $kr_ila]')
            flow.tcl(f'write_hw_ila_data -force -csv_file {{{capture}/capture_data.csv}} $kr_data; '
                     f'write_hw_ila_data -force -vcd_file {{{capture}/capture_data.vcd}} $kr_data; '
                     f'write_hw_ila_data -force {{{capture}/capture_data.ila}} $kr_data')
            samples, result = analyze_capture(capture/'capture_data.csv')
            if samples[127]['blink'] != 0 or samples[128]['blink'] != 1:
                raise RuntimeError('Expected rising-edge trigger at sample 128')
            metadata = flow.tcl('set kr_meta [list "STATUS=[get_property STATUS.CORE_STATUS $kr_ila]" '
                                '"CORE=$kr_ila" "DEPTH=[get_property CONTROL.DATA_DEPTH $kr_ila]"]; '
                                'foreach p [get_hw_probes -of_objects $kr_ila] {'
                                'lappend kr_meta "$p WIDTH=[get_property PROBE_PORT_BIT_COUNT $p] COMPARE=[get_property TRIGGER_COMPARE_VALUE $p]"}; '
                                'join $kr_meta "\\n"')
            (capture/'metadata.txt').write_text(metadata)
            report = {'metadata':{'skill':'hw-ila-debug','mode':'ila',
                      'timestamp':datetime.now(timezone.utc).isoformat(),'device':device,'core_details':metadata},
                      'ila_config':{'data_depth':1024,'trigger_mode':'BASIC_ONLY','trigger_condition':'AND',
                                    'trigger_position':128,'trigger':'blink rising edge'},
                      'capture':result,'observations':['All 1023 counter steps increment by one modulo 65536.',
                        'Reset is released throughout capture.', 'Blink agrees with elapsed-cycle division by 64.',
                        'Rising-edge trigger occurs at sample 128.'],
                      'bitstream_sha256':hashlib.sha256(Path(state['bitstream']).read_bytes()).hexdigest(),
                      'probes_sha256':hashlib.sha256(Path(state['probes']).read_bytes()).hexdigest()}
            (capture/'report_data.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
            (capture/'REPORT.md').write_text(
                '# KR260 ILA 实测报告\n\n结果：PASS。数据来自通过 USB/JTAG 连接的 KR260，非仿真。\n\n'
                '- 采样深度：1024；时钟：板载标称 25 MHz。\n'
                '- 触发：blink 上升沿；位置：第 128 号样本。\n'
                '- 1023 次计数器相邻样本检查通过，按 16 位模运算递增。\n'
                '- 翻转间隔：64 个采样周期；复位全程已释放。\n\n'
                '绝对时间由板载时钟标称频率换算；未用外部仪器测量振荡器精度。\n\n'
                '原始数据：capture_data.csv；波形：capture_data.vcd、capture_data.ila。\n')
            plot_capture(samples, capture/'waveform.png')
            state['hardware_capture'] = 'PASS'
            state['capture_directory'] = str(capture)
            state['status'] = 'HARDWARE_PASS'
            state['capture_analysis'] = result
            state.pop('hardware_error', None)
            if 'hardware' not in state['completed']:
                state['completed'].append('hardware')
            state['active_step'] = 'complete'
            flow.save()
            flow.call('vivado_todos',operation='update',session_id=state['session_id'],id='hardware',
                      status='done',summary='Actual 1024-sample ILA trace verified; CSV/VCD/ILA and report saved')
        except BaseException as exc:
            state['hardware_capture'] = 'FAILED'
            state['hardware_error'] = str(exc)
            flow.save()
            raise
        if args.show:
            # User explicitly requested seeing waveforms in Vivado.
            flow.tcl('display_hw_ila_data $kr_data')
    print(f'Hardware PASS. Reports and waveform: {capture}')


if __name__ == '__main__':
    main()

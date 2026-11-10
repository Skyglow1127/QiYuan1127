#!/usr/bin/env python3
"""Offline KR260 firmware build through Ross MCP; never programs hardware.
Leaves all Vivado sessions open, even on failure. Single job/two threads.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'tools'))
from ross_mcp import Ross
from vivado_memory_scope import protect_vivado

BASELINE = 'TEST_PASS: divisors=1,3,5 checks=240 reset_restarts=2 clock=100MHz'
CDC_PASS = 'TEST_PASS: FSR_CDC checks=240 reset_windows=2 channels=2 clock=25MHz'
BUS_PASS = 'TEST_PASS: FSR_BUS checks=44 reset_releases=2 axi_reads=16'
REVISION = 'r2_aux_reset_and_afi_20261025'
SKILL = Path('/home/aup/.vscode/extensions/amd.vivado-ai-extension-2026.9.1/resources/plugin-bundled/skills/vivado-ip-configurator')


def memory_guard(minimum_gib):
    values = {line.split(':')[0]: int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()}
    available = values['MemAvailable'] / 1024**2
    print('Available RAM %.1f GiB; required %s GiB' % (available, minimum_gib), flush=True)
    if available < minimum_gib:
        raise RuntimeError('Insufficient available memory; no process is closed automatically.')


def unpack(result):
    texts = [item['text'] for item in result.get('content', []) if item.get('type') == 'text']
    if len(texts) != 1:
        raise RuntimeError('Unexpected Ross response: ' + repr(result))
    try:
        return json.loads(texts[0])
    except json.JSONDecodeError:
        return {'output': texts[0]}


class Build:
    def __init__(self, ross, state):
        self.ross, self.state = ross, state
        self.run = Path(state['run_directory'])
        self.evidence = Path(state.get('evidence_directory', self.run / 'evidence'))
        self.evidence.mkdir(exist_ok=True)

    def save(self):
        (self.evidence / 'summary.json').write_text(json.dumps(self.state, ensure_ascii=False, indent=2))
        (HERE / 'latest_build.json').write_text(json.dumps(self.state, ensure_ascii=False, indent=2))

    def call(self, name, **args):
        result = unpack(self.ross.call(name, args))
        if result.get('exit_code', 0) != 0 or result.get('status') in ('error', 'failed'):
            raise RuntimeError(json.dumps(result, ensure_ascii=False))
        return result

    def tcl(self, command):
        print('Vivado: ' + command[:160], flush=True)
        command += '; # invocation ' + str(time.time_ns())
        result = self.call('vivado_execute', session_id=self.state['session_id'], command=command, verbosity='full')
        with (self.evidence / 'commands.jsonl').open('a') as f:
            f.write(json.dumps({'command': command, 'result': result}, ensure_ascii=False) + '\n')
        while result.get('status') == 'running' or result.get('is_command_running'):
            time.sleep(30)
            result = self.call('vivado_status', session_id=self.state['session_id'], action='session')
            if not result.get('is_command_running'):
                if result.get('last_completed_code') != 0:
                    raise RuntimeError('Vivado command did not succeed: ' + repr(result))
                break
        return result.get('output', '')

    def step(self, name, action):
        if name in self.state.setdefault('completed', []):
            return
        self.state['active_step'] = name
        self.save()
        action()
        self.state['completed'].append(name)
        self.save()

    def setup(self):
        self.tcl('set_param general.maxThreads 2; puts [version -short]')
        log = self.state['startup_receipt'].get('log_file')
        if not log:
            raise RuntimeError('Ross did not report exact session log_file; refusing to guess native evidence log')
        self.tcl('set ::FSR_PROJECT_DIR {%s}; set ::FSR_EVIDENCE_DIR {%s}; set ::FSR_NATIVE_LOG {%s}; set ::FSR_SKILL_DIR {%s}' %
                 (self.run / 'project', self.evidence, log, self.state['skill_directory']))
        self.tcl('source -notrace {%s}' % (HERE / 'create_design.tcl'))
        self.state['project'] = str(self.run / 'project/kr260_fsr.xpr')

    def simulate(self):
        # Direct DUT instances from a BD are not discovered by Vivado's pruned
        # compile order. Register generated wrappers and their actual vendor
        # libraries explicitly; these remain the unmodified generated IP.
        base = self.run / 'project/kr260_fsr.gen/sources_1/bd/fsr_bd/ip'
        wrappers = [base / (name + '/sim/' + name + '.vhd') for name in ('fsr_bd_reset_i_0', 'fsr_bd_fsr_gpio_0')]
        libraries = []
        for wrapper in wrappers:
            match = re.search(r'^LIBRARY ((?:proc_sys_reset|axi_gpio)_v[^;]+);', wrapper.read_text(), re.M)
            if not match:
                raise RuntimeError('Cannot resolve generated IP library: ' + str(wrapper))
            libraries.append(match[1])
        self.tcl('add_files -fileset sim_bus -norecurse [list %s]; set_property xsim.elaborate.xelab.more_options { %s} [get_filesets sim_bus]; update_compile_order -fileset sim_bus' % (' '.join('{%s}' % p for p in wrappers), ' '.join('-L ' + name for name in libraries)))
        for simset, runtime, marker in [('sim_baseline', '2us', BASELINE), ('sim_1', '12us', CDC_PASS), ('sim_bus', '100us', BUS_PASS)]:
            dest = self.evidence / simset
            dest.mkdir(exist_ok=True)
            self.tcl('current_fileset -simset %s; set_property xsim.simulate.runtime %s [get_filesets %s]' % (simset, runtime, simset))
            self.tcl('launch_simulation -simset %s -mode behavioral' % simset)
            source = self.run / ('project/kr260_fsr.sim/%s/behav/xsim' % simset)
            for name in ('compile.log', 'elaborate.log', 'simulate.log'):
                contents = (source / name).read_text()
                shutil.copy2(source / name, dest / name)
                if re.search(r'^\s*(ERROR:|FATAL:|Fatal:)', contents, re.M):
                    raise RuntimeError('Simulation error in ' + str(source / name))
            if (dest / 'simulate.log').read_text().count(marker) != 1:
                raise RuntimeError('Missing unique expected TEST_PASS: ' + marker)
            self.tcl('close_sim -force')
        self.state['simulation'] = {'baseline_checks': 240, 'cdc_checks': 240, 'bus_checks': 44, 'axi_reads': 16, 'reset_releases': 2, 'status': 'PASS'}

    def repair_reset(self):
        """Repair the same BD through the Ross/IP configurator tracked mutation path."""
        memory_guard(6)
        self.tcl('if {[llength [get_designs]]} {close_design}; open_bd_design [get_files fsr_bd.bd]')
        # C_EXT_RESET_HIGH is derived/read-only after connecting pl_resetn0.
        # Its existing value is verified below; only the writable AUX property
        # is changed in a resumed BD.
        self.tcl('set ::FSR_EVIDENCE_DIR {%s}; set reset_config [list CONFIG.C_EXT_RESET_HIGH 0 CONFIG.C_AUX_RESET_HIGH 1]; set receipt [ipcfg::capture [file join $::FSR_EVIDENCE_DIR reset_polarity.[clock clicks].tcldict] [list ipcfg::require reset_i [list CONFIG.C_AUX_RESET_HIGH 1] "Unused auxiliary reset must be inactive at zero" {} "Repair confirmed auxiliary reset polarity defect"] {REQ:pinned+applied}]; puts $receipt; if {[dict get $receipt status] ne "OK"} {error "Reset correction failed"}' % self.evidence)
        self.tcl('set receipt [ipcfg::capture [file join $::FSR_EVIDENCE_DIR validate_reset.[clock clicks].tcldict] {validate_bd_design}]; puts $receipt; if {[dict get $receipt status] ne "OK"} {error "Reset validation failed"}; set bad [ipcfg::verify_stuck reset_i $reset_config]; if {[llength $bad]} {error "Reset polarity did not stick: $bad"}; puts "RESET_POLARITY: ext=0 aux=1"')
        self.tcl('set f [open {%s} w]; foreach p {ext_reset_in aux_reset_in mb_debug_sys_rst dcm_locked} {set pin [get_bd_pins reset_i/$p]; puts $f "$p net=[get_bd_nets -of_objects $pin]"}; puts $f "C_EXT_RESET_HIGH=[get_property CONFIG.C_EXT_RESET_HIGH [get_bd_cells reset_i]] C_AUX_RESET_HIGH=[get_property CONFIG.C_AUX_RESET_HIGH [get_bd_cells reset_i]]"; close $f; save_bd_design; write_bd_tcl -force {%s}; generate_target all [get_files fsr_bd.bd]' % (self.evidence / 'reset_readback.txt', self.evidence / 'fsr_bd_recreate.tcl'))
        self.tcl('set_property USED_IN_SIMULATION true [get_files fsr_bd.bd]; if {![llength [get_filesets sim_bus]]} {create_fileset -simset sim_bus}; add_files -fileset sim_bus -norecurse {%s}; set_property top tb_fsr_bus [get_filesets sim_bus]; update_compile_order -fileset sim_bus; reset_runs synth_1' % (HERE / 'tb/tb_fsr_bus.sv'))
        old = self.run / 'evidence/address_map.json'
        shutil.copy2(old, self.evidence / 'address_map.json')
        self.state['completed'] = ['setup']
        self.state['revision'] = REVISION
        self.state.pop('verification', None)
        self.save()

    def synthesis(self):
        memory_guard(6)
        self.tcl('launch_runs synth_1 -jobs 1')
        self.tcl('wait_on_run synth_1')
        output = self.tcl('puts [get_property STATUS [get_runs synth_1]]')
        if 'synth_design Complete!' not in output:
            raise RuntimeError('Synthesis incomplete: ' + output)
        self.check_run_log('synth_1')

    def check_run_log(self, name):
        log = self.run / ('project/kr260_fsr.runs/%s/runme.log' % name)
        text = log.read_text(errors='replace')
        shutil.copy2(log, self.evidence / (name + '_runme.log'))
        blockers = re.findall(r'^\s*(?:ERROR:|CRITICAL WARNING:|FATAL:).*$', text, re.M)
        if blockers:
            raise RuntimeError('Run has blocking messages: ' + '\n'.join(blockers[:8]))

    def implementation(self):
        memory_guard(6)
        self.tcl('launch_runs impl_1 -to_step route_design -jobs 1')
        self.tcl('wait_on_run impl_1')
        if 'route_design Complete!' not in self.tcl('puts [get_property STATUS [get_runs impl_1]]'):
            raise RuntimeError('Implementation incomplete')
        self.check_run_log('impl_1')
        self.tcl('open_run impl_1')
        self.tcl('set first [get_cells -hier -filter {NAME =~ *sync_i*meta_reg* && IS_SEQUENTIAL}]; set second [get_cells -hier -filter {NAME =~ *sync_i*synced_reg* && IS_SEQUENTIAL}]; if {[llength $first] != 2 || [llength $second] != 2} {error "Expected two independent two-flop CDC chains"}; set f [open {%s} w]; foreach c [concat $first $second] {if {![get_property ASYNC_REG $c]} {error "Missing ASYNC_REG on $c"}; puts $f "$c ASYNC_REG=[get_property ASYNC_REG $c] D=[get_pins -of_objects $c -filter {REF_PIN_NAME == D}]"}; close $f; puts "CDC_IMPLEMENTED_FLOPS=4"' % (self.evidence / 'cdc_flops.txt'))
        self.tcl('report_exceptions -coverage -file {%s}' % (self.evidence / 'exceptions.rpt'))
        self.cdc_timing_evidence()
        self.tcl('report_timing_summary -report_unconstrained -file {%s}; report_drc -file {%s}; report_utilization -file {%s}; report_cdc -details -file {%s}; report_methodology -file {%s}' %
                 tuple(self.evidence / name for name in ['timing_summary.rpt', 'drc.rpt', 'utilization.rpt', 'cdc.rpt', 'methodology.rpt']))
        timing = (self.evidence / 'timing_summary.rpt').read_text()
        if 'All user specified timing constraints are met.' not in timing:
            raise RuntimeError('Timing requirements not met')
        self.tcl('set violations [get_drc_violations -filter {SEVERITY == Error || SEVERITY == "Critical Warning"}]; if {[llength $violations]} {error "Blocking DRC: $violations"}; puts "DRC_BLOCKERS=0"')
        self.tcl('set ps8 [get_cells -hier -filter {REF_NAME == PS8}]; set gpio_cells [get_cells -hier -filter {NAME =~ *fsr_gpio* && IS_SEQUENTIAL}]; if {[llength $ps8] != 1 || [llength $gpio_cells] == 0} {error "PS/AXI GPIO logic missing"}; set f [open {%s} w]; puts $f "PS8_COUNT=[llength $ps8] PS8=$ps8"; puts $f "GPIO_SEQUENTIAL_CELLS=[llength $gpio_cells]"; close $f' % (self.evidence / 'ps_gpio_presence.txt'))
        # Read implemented I/O and synchronizers as evidence; never rely on XDC text alone.
        self.tcl('set f [open {%s} w]; foreach p [get_ports] {puts $f "$p [get_property DIRECTION $p] [get_property PACKAGE_PIN $p] [get_property IOSTANDARD $p]"}; close $f; puts "IO_READBACK_RECORDED"' % (self.evidence / 'io_readback.txt'))

    def cdc_timing_evidence(self):
        self.tcl('set first [get_cells -hier -filter {NAME =~ *sync_i*meta_reg* && IS_SEQUENTIAL}]; set second [get_cells -hier -filter {NAME =~ *sync_i*synced_reg* && IS_SEQUENTIAL}]; set firstq [get_pins -of_objects $first -filter {REF_PIN_NAME == Q}]; set secondd [get_pins -of_objects $second -filter {REF_PIN_NAME == D}]; set paths [get_timing_paths -from $firstq -to $secondd -max_paths 2]; if {[llength $paths] != 2} {error "CDC stages must retain two timed paths"}; foreach p $paths {if {[get_property SLACK $p] < 0} {error "CDC interstage timing failed"}}; puts "CDC_TIMED_STAGE1_TO_STAGE2_PATHS=2"')
        self.tcl('report_timing -from $firstq -to $secondd -max_paths 2 -file {%s}' % (self.evidence / 'cdc_interstage_timing.rpt'))
        self.tcl('write_xdc -force {%s}' % (self.evidence / 'effective_constraints.xdc'))

    def bitstream(self):
        memory_guard(6)
        self.tcl('launch_runs impl_1 -to_step write_bitstream -jobs 1')
        self.tcl('wait_on_run impl_1')
        if 'write_bitstream Complete!' not in self.tcl('puts [get_property STATUS [get_runs impl_1]]'):
            raise RuntimeError('Bitstream incomplete')
        self.check_run_log('impl_1')
        self.tcl('write_hw_platform -fixed -include_bit -force -file {%s}' % (self.evidence / 'kr260-fsr.xsa'))
        self.package()
        self.state['status'] = 'OFFLINE_BUILD_PASS'

    def package(self):
        output = HERE / 'output'
        output.mkdir(exist_ok=True)
        bit = self.run / 'project/kr260_fsr.runs/impl_1/fsr_top.bit'
        shutil.copy2(bit, output / 'fsr_top.bit')
        shutil.copy2(self.evidence / 'kr260-fsr.xsa', output / 'kr260-fsr.xsa')
        shutil.copy2(self.evidence / 'address_map.json', output / 'address_map.json')
        address = json.loads((output / 'address_map.json').read_text())
        if address['base_address'] != 0xA0000000 or address['address_range'] != 65536 or address['axi_hpm0_data_width'] != 128:
            raise RuntimeError('Address/AFI schema mismatch')
        bit_bif = output / 'fsr.bif'
        bit_bif.write_text('all:\n{\n    fsr_top.bit\n}\n')
        vivado = Path(self.state['startup_receipt'].get('vivado_path', '/home/aup/Xilinx/2025.2/Vivado/bin/vivado')).parent
        bootgen, dtc = vivado / 'bootgen', vivado / 'dtc'
        subprocess.run([str(bootgen), '-arch', 'zynqmp', '-image', bit_bif.name, '-process_bitstream', 'bin', '-w'], cwd=output, check=True)
        generated = output / 'fsr_top.bit.bin'
        if not generated.is_file():
            raise RuntimeError('bootgen did not emit fsr_top.bit.bin')
        generated.replace(output / 'kr260-fsr.bit.bin')
        dts = overlay(address)
        (output / 'kr260-fsr.dts').write_text(dts)
        subprocess.run([str(dtc), '-@', '-I', 'dts', '-O', 'dtb', '-o', str(output / 'kr260-fsr.dtbo'), str(output / 'kr260-fsr.dts')], check=True)
        (output / 'shell.json').write_text(json.dumps({'shell_type': 'XRT_FLAT', 'num_slots': '1'}, indent=2) + '\n')
        names = ['fsr_top.bit', 'kr260-fsr.bit.bin', 'kr260-fsr.xsa', 'kr260-fsr.dtbo', 'shell.json', 'address_map.json']
        manifest = {'schema_version': 1, 'status': 'OFFLINE_BUILD_PASS', 'hardware_status': 'NOT_TESTED',
                    'revision': REVISION,
                    'created_utc': datetime.now(timezone.utc).isoformat(), 'design': {'part': 'xck26-sfvc784-2LV-c', **address},
                    'artifacts': {name: hashlib.sha256((output / name).read_bytes()).hexdigest() for name in names},
                    'evidence_directory': str(self.evidence), 'simulation': self.state['simulation'],
                    'source_sha256': self.source_hashes()}
        (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        self.state['output_directory'] = str(output)

    def source_hashes(self):
        names = ['create_design.tcl', 'build_firmware.py', 'boards/SOURCES.json', 'rtl/fsr_top.sv', 'rtl/fsr_sync2.v', 'rtl/led_blink.sv', 'tb/tb_led_blink.sv', 'tb/tb_fsr_sync2.sv', 'tb/tb_fsr_bus.sv', 'constraints/kr260_fsr.xdc', 'tools/ross_mcp.py', 'tools/vivado_memory_scope.py']
        return {name: hashlib.sha256((HERE / name).read_bytes()).hexdigest() for name in names}

    def finalize(self):
        self.state['revision'] = REVISION
        self.state['source_sha256'] = self.source_hashes()
        self.save()
        shutil.copy2(self.state['startup_receipt']['log_file'], self.evidence / 'native_session.log')
        shutil.copytree(self.evidence, HERE / 'evidence', dirs_exist_ok=True)


def overlay(address):
    base, span = address['base_address'], address['address_range']
    return '''/dts-v1/;
/plugin/;
/* Derived from Vivado address readback and board live DT evidence, 2026-10-25.
 * Offline build only: loading and electrical behavior have NOT been validated.
 */
&fpga_full {
    #address-cells = <2>;
    #size-cells = <2>;
    firmware-name = "kr260-fsr.bit.bin";
};
&{/} {
    fsr_clock25: fsr-clock25 {
        compatible = "fixed-clock";
        #clock-cells = <0>;
        clock-frequency = <25000000>;
        clock-output-names = "kr260-c3-25mhz";
    };
};
&amba {
    #address-cells = <2>;
    #size-cells = <2>;
    fsr_afi: fsr-afi {
        compatible = "xlnx,afi-fpga";
        /* AFIFS id14: HPM0/HPM1 data width128 => selectors 2 at bits9:8/11:10.
           Only used PS->PL width register changes; no unrelated HP/HPC writes. */
        config-afi = <14 0xa00>;
        /* AMD SDT generator assigns fabric resets to the AFI driver. */
        resets = <&zynqmp_reset 116>, <&zynqmp_reset 117>, <&zynqmp_reset 118>, <&zynqmp_reset 119>;
    };
    fsr_gpio: gpio@%x {
        compatible = "xlnx,xps-gpio-1.00.a";
        reg = <0 0x%x 0 0x%x>;
        clocks = <&fsr_clock25>;
        clock-names = "s_axi_aclk";
        gpio-controller;
        #gpio-cells = <2>;
        gpio-line-names = "FSR_DO1", "FSR_DO2";
        label = "kr260-fsr-do";
        xlnx,gpio-width = <2>;
        xlnx,all-inputs = <1>;
        xlnx,all-outputs = <0>;
        xlnx,is-dual = <0>;
        xlnx,interrupt-present = <0>;
        xlnx,tri-default = <0xffffffff>;
        status = "okay";
    };
};
''' % (base, base, span)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--rebuild-runs', action='store_true', help='Reset synthesis/implementation in the same session after a source repair')
    parser.add_argument('--repair-reset', action='store_true', help='Apply revision 2 reset polarity fix to the existing session and rerun all simulation/build gates')
    parser.add_argument('--skill', type=Path, default=SKILL)
    args = parser.parse_args()
    if (args.rebuild_runs or args.repair_reset) and not args.resume:
        parser.error('--rebuild-runs/--repair-reset requires --resume')
    with Ross() as ross:
        if args.resume:
            state = json.loads(args.resume.read_text())
            if args.repair_reset:
                state['evidence_directory'] = str(Path(state['run_directory']) / ('evidence_' + REVISION))
        else:
            memory_guard(8)
            run = HERE / 'build' / datetime.now().strftime('kr260_fsr_%Y%m%d_%H%M%S')
            run.mkdir(parents=True)
            receipt = unpack(ross.call('vivado_start', {'working_dir': str(run), 'session_type': 'general', 'gui_mode': False, 'display_mode': 'none'}))
            if 'session_id' not in receipt:
                raise RuntimeError(repr(receipt))
            state = {'status': 'RUNNING', 'run_directory': str(run), 'session_id': receipt['session_id'],
                     'startup_receipt': receipt, 'skill_directory': str(args.skill.resolve()),
                     'hardware_status': 'NOT_TESTED', 'session_close_policy': 'manual'}
            flow = Build(ross, state)
            flow.save()
            state['memory_scope'] = protect_vivado(receipt['process_id'])
            flow.save()
        flow = Build(ross, state)
        try:
            state.pop('error', None)
            state['status'] = 'RUNNING'
            scope = state['memory_scope']
            cgroup = Path(scope['cgroup'])
            if int((cgroup / 'memory.max').read_text()) != 5 * 1024**3 or int((cgroup / 'memory.swap.max').read_text()) != 1024**3:
                raise RuntimeError('Vivado scope readback no longer matches required 5 GiB/1 GiB limits')
            pid = state['startup_receipt']['process_id']
            if str(pid) not in (cgroup / 'cgroup.procs').read_text().split():
                raise RuntimeError('Vivado process no longer belongs to the verified scope')
            if args.repair_reset:
                flow.repair_reset()
            elif args.rebuild_runs:
                flow.tcl('close_design')
                flow.tcl('reset_runs synth_1')
                state['completed'] = [name for name in state.get('completed', []) if name not in ('synthesis', 'implementation', 'bitstream')]
                flow.save()
            for name in ('setup', 'simulate', 'synthesis', 'implementation', 'bitstream'):
                flow.step(name, getattr(flow, name))
            flow.finalize()
        except BaseException as exc:
            state['status'] = 'FAILED'
            state['error'] = str(exc)
            raise
        finally:
            flow.save()
            print('Vivado session left open: ' + state['session_id'], flush=True)
    print(json.dumps(state, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

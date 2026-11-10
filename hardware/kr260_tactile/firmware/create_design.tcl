# Run only through build_firmware.py / Ross MCP. No board programming occurs.
foreach v {FSR_PROJECT_DIR FSR_EVIDENCE_DIR FSR_NATIVE_LOG FSR_SKILL_DIR} {
    if {![info exists ::$v]} {error "Required build driver variable missing: $v"}
}
set fsr_source [file dirname [file normalize [info script]]]
set_param general.maxThreads 2
set_param board.repoPaths [list [file join $fsr_source boards Xilinx]]
if {[catch {current_project} current] || $current eq ""} {create_project kr260_fsr $::FSR_PROJECT_DIR -part xck26-sfvc784-2LV-c}
set_property board_part xilinx.com:kr260_som:part0:1.1 [current_project]
if {[llength [get_files -filter {NAME =~ *fsr_sync2.sv}]]} {remove_files [get_files -filter {NAME =~ *fsr_sync2.sv}]}
add_files -norecurse [list [file join $fsr_source rtl fsr_sync2.v] [file join $fsr_source rtl fsr_top.sv]]
set_property top fsr_top [get_filesets sources_1]
add_files -fileset constrs_1 -norecurse [file join $fsr_source constraints kr260_fsr.xdc]
if {![llength [get_bd_designs fsr_bd]]} {create_bd_design fsr_bd}
source -notrace [file join $::FSR_SKILL_DIR lib ipcfg.tcl]
set ::ipcfg::native_log_path $::FSR_NATIVE_LOG
set py_status [ipcfg::python_status]
if {![string match {PYTHON:OK *} $py_status]} {error $py_status}
puts $py_status
puts [ipcfg::python_exec -c {import argparse, base64, hashlib, json, pathlib, re, sys, typing; print("PYTHON_IMPORTS:OK")}]
proc fsr_capture {name script {expect {}}} {
    if {[info exists ::fsr_done($name)]} {return}
    set path [file join $::FSR_EVIDENCE_DIR "$name.[clock clicks].tcldict"]
    set receipt [ipcfg::capture $path $script $expect]
    puts $receipt
    if {[dict get $receipt status] ne "OK"} {error "FSR_STAGE_FAIL:$name see $path"}
    set ::fsr_done($name) 1
}
# Candidate-family gates before creation, without any part swaps.
foreach pattern {*zynq*ultra* *gpio* *smartconnect* *proc_sys_reset* *constant*} {
    set available [ipcfg::ip_availability [list $pattern]]
    puts $available
    if {![string match {*AVAILABLE:*} $available]} {error "No supported IP for $pattern"}
}
fsr_capture ps_create [list ipcfg::create_cell_cfg xilinx.com:ip:zynq_ultra_ps_e ps {}] {SUCCESS:}
fsr_capture ps_board [list ipcfg::run_automation ps xilinx.com:bd_rule:zynq_ultra_ps_e [list apply_board_preset 1]] {AUTOMATION:OK SKIP:board}
# Preserve the official SOM board preset; override only PL-facing connectivity.
set ps_config [list CONFIG.PSU__USE__M_AXI_GP0 1 CONFIG.PSU__USE__M_AXI_GP1 0 CONFIG.PSU__USE__M_AXI_GP2 0 CONFIG.PSU__MAXIGP0__DATA_WIDTH 128 CONFIG.PSU__USE__IRQ0 0 CONFIG.PSU__USE__IRQ1 0 CONFIG.PSU__USE__FABRIC__RST 1 CONFIG.PSU__NUM_FABRIC_RESETS 1]
fsr_capture ps_require [list ipcfg::require ps $ps_config "Single input-only AXI GPIO via HPM0; C3 external clock" {} "PL connection only; preserve board DDR/MIO settings"] {REQ:pinned+applied}
set gpio_config [list CONFIG.C_GPIO_WIDTH 2 CONFIG.C_ALL_INPUTS 1 CONFIG.C_ALL_OUTPUTS 0 CONFIG.C_IS_DUAL 0 CONFIG.C_INTERRUPT_PRESENT 0]
fsr_capture gpio_create [list ipcfg::create_cell_cfg xilinx.com:ip:axi_gpio fsr_gpio $gpio_config] {SUCCESS:}
fsr_capture bus_create [list ipcfg::create_cell_cfg xilinx.com:ip:smartconnect axi_bus [list CONFIG.NUM_SI 1 CONFIG.NUM_MI 1]] {SUCCESS:}
set reset_config [list CONFIG.C_EXT_RESET_HIGH 0 CONFIG.C_AUX_RESET_HIGH 1]
fsr_capture reset_create [list ipcfg::create_cell_cfg xilinx.com:ip:proc_sys_reset reset_i $reset_config] {SUCCESS:}
fsr_capture one_create [list ipcfg::create_cell_cfg xilinx.com:ip:xlconstant one [list CONFIG.CONST_WIDTH 1 CONFIG.CONST_VAL 1]] {SUCCESS:}
fsr_capture zero_create [list ipcfg::create_cell_cfg xilinx.com:ip:xlconstant zero [list CONFIG.CONST_WIDTH 1 CONFIG.CONST_VAL 0]] {SUCCESS:}
create_bd_cell -type module -reference fsr_sync2 sync_i
set clkp [create_bd_port -dir I -type clk clk_25mhz]
set_property CONFIG.FREQ_HZ 25000000 $clkp
create_bd_port -dir I -from 1 -to 0 fsr_do
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins axi_bus/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_bus/M00_AXI] [get_bd_intf_pins fsr_gpio/S_AXI]
connect_bd_net [get_bd_ports clk_25mhz] [get_bd_pins ps/maxihpm0_fpd_aclk] [get_bd_pins axi_bus/aclk] [get_bd_pins fsr_gpio/s_axi_aclk] [get_bd_pins reset_i/slowest_sync_clk] [get_bd_pins sync_i/clk]
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins reset_i/ext_reset_in]
connect_bd_net [get_bd_pins one/dout] [get_bd_pins reset_i/dcm_locked]
connect_bd_net [get_bd_pins zero/dout] [get_bd_pins reset_i/aux_reset_in] [get_bd_pins reset_i/mb_debug_sys_rst]
connect_bd_net [get_bd_pins reset_i/peripheral_aresetn] [get_bd_pins fsr_gpio/s_axi_aresetn] [get_bd_pins sync_i/resetn] [get_bd_pins axi_bus/aresetn]
connect_bd_net [get_bd_ports fsr_do] [get_bd_pins sync_i/din]
connect_bd_net [get_bd_pins sync_i/dout] [get_bd_pins fsr_gpio/gpio_io_i]
assign_bd_address -offset 0xA0000000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps/Data] [get_bd_addr_segs fsr_gpio/S_AXI/Reg] -force
fsr_capture validate [list validate_bd_design]
foreach {cell config} [list ps $ps_config fsr_gpio $gpio_config reset_i $reset_config] {
    set missing [ipcfg::verify_stuck $cell $config]
    if {[llength $missing]} {error "Readback mismatch $cell: $missing"}
}
if {[get_property LEFT [get_bd_pins fsr_gpio/gpio_io_i]] != 1 || [get_property RIGHT [get_bd_pins fsr_gpio/gpio_io_i]] != 0 || [get_property DIR [get_bd_pins fsr_gpio/gpio_io_i]] ne "I"} {error "Expected two-bit GPIO input interface"}
set seg [get_bd_addr_segs ps/Data/SEG_fsr_gpio_Reg]
set base [get_property OFFSET $seg]
set span [get_property RANGE $seg]
if {[regexp {^([0-9]+)([KMG])$} $span -> number suffix]} {
    set factors [dict create K 1024 M 1048576 G 1073741824]
    set span [expr {$number * [dict get $factors $suffix]}]
}
if {$base != 0xA0000000 || $span != 65536} {error "Unexpected GPIO address readback $base $span"}
set address_file [open [file join $::FSR_EVIDENCE_DIR address_map.json] w]
puts $address_file [format {{"base_address":%d,"address_range":%d,"gpio_width":2,"all_inputs":true,"clock_hz":25000000,"axi_hpm0_data_width":128,"offsets":[0,1],"pins":["AB14","Y14"]}} $base $span]
close $address_file
fsr_capture requirements [list ipcfg::audit_requirements] {REQ:OK REQ:empty}
save_bd_design
write_bd_tcl -force [file join $::FSR_EVIDENCE_DIR fsr_bd_recreate.tcl]
generate_target all [get_files fsr_bd.bd]
set_property synth_checkpoint_mode None [get_files fsr_bd.bd]
# Small flattened synthesis, one job, no concurrent out-of-context IP processes.
set_property USED_IN_SIMULATION true [get_files fsr_bd.bd]
set_property USED_IN_SIMULATION false [get_files fsr_top.sv]
add_files -fileset sim_1 -norecurse [file join $fsr_source tb tb_fsr_sync2.sv]
set_property top tb_fsr_sync2 [get_filesets sim_1]
create_fileset -simset sim_baseline
add_files -fileset sim_baseline -norecurse [list [file join $fsr_source rtl led_blink.sv] [file join $fsr_source tb tb_led_blink.sv]]
set_property top tb_led_blink [get_filesets sim_baseline]
set_property target_simulator XSim [current_project]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
update_compile_order -fileset sim_baseline
create_fileset -simset sim_bus
add_files -fileset sim_bus -norecurse [file join $fsr_source tb tb_fsr_bus.sv]
set_property top tb_fsr_bus [get_filesets sim_bus]
update_compile_order -fileset sim_bus
puts "FSR_SETUP_COMPLETE: GPIO width=2 input-only base=$base range=$span C3=25MHz; hardware NOT tested"

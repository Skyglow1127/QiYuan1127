# Board integration via Ross MCP only. No board connection or programming here.
foreach v {VPR_PROJECT VPR_EVIDENCE VPR_NATIVE_LOG VPR_ROOT VPR_SKILL VPR_HLS_RTL} {
    if {![info exists ::$v]} {error "Missing driver variable $v"}
}
set root $::VPR_ROOT
set_param general.maxThreads 2
set_param board.repoPaths [list [file join $root boards Xilinx]]
create_project kr260_vision_board $::VPR_PROJECT -part xck26-sfvc784-2LV-c
set_property board_part xilinx.com:kr260_som:part0:1.1 [current_project]
add_files -norecurse [list [file join $root vision_preprocess v2 rtl vpr2_stream.sv] [file join $root vision_preprocess v3 rtl rgb888_axis_packer.sv] [file join $root vision_preprocess v3 rtl vpr3_stream.sv] [file join $root vision_preprocess v3 rtl vpr3_axi.sv] [file join $root vision_preprocess board vision_board_top.sv] [file join $root vision_preprocess v3 rtl vision_preprocess_bd.v] [file join $root pressure_delivery program firmware rtl fsr_sync2.v]]
add_files -norecurse [glob [file join $::VPR_HLS_RTL *.v]]
set_property top vision_board_top [get_filesets sources_1]
add_files -fileset constrs_1 -norecurse [file join $root vision_preprocess v3 board kr260_vpr3.xdc]
create_bd_design vision_bd
source -notrace [file join $::VPR_SKILL lib ipcfg.tcl]
set ::ipcfg::native_log_path $::VPR_NATIVE_LOG
puts [ipcfg::python_status]
puts [ipcfg::python_exec -c {import argparse,base64,hashlib,json,pathlib,re,sys,typing; print("PYTHON_IMPORTS:OK")}]
proc vpr_capture {name script {expect {}}} {
    set path [file join $::VPR_EVIDENCE "$name.[clock clicks].tcldict"]
    set receipt [ipcfg::capture $path $script $expect]
    puts $receipt
    if {[dict get $receipt status] ne "OK"} {error "VPR_CONFIG_FAIL:$name see $path"}
}
foreach pattern {*zynq*ultra* *axi_dma* *gpio* *smartconnect* *proc_sys_reset* *xlconstant* *xlconcat* *clk_wiz*} {
    set available [ipcfg::ip_availability [list $pattern]]
    if {![string match {*AVAILABLE:*} $available]} {error "IP unavailable: $pattern"}
}
vpr_capture ps_create [list ipcfg::create_cell_cfg xilinx.com:ip:zynq_ultra_ps_e ps {}] {SUCCESS:}
vpr_capture ps_board [list ipcfg::run_automation ps xilinx.com:bd_rule:zynq_ultra_ps_e [list apply_board_preset 1]] {AUTOMATION:OK SKIP:board}
# Same official SOM preset and HPM0 as current pressure design, plus HP0 and IRQ0.
set ps_config [list CONFIG.PSU__USE__M_AXI_GP0 1 CONFIG.PSU__USE__M_AXI_GP1 0 CONFIG.PSU__USE__M_AXI_GP2 0 CONFIG.PSU__MAXIGP0__DATA_WIDTH 128 CONFIG.PSU__USE__S_AXI_GP2 1 CONFIG.PSU__SAXIGP2__DATA_WIDTH 128 CONFIG.PSU__USE__IRQ0 1 CONFIG.PSU__USE__IRQ1 0 CONFIG.PSU__USE__FABRIC__RST 1 CONFIG.PSU__NUM_FABRIC_RESETS 1]
vpr_capture ps_require [list ipcfg::require ps $ps_config "HPM0 control, HP0 non-coherent DMA DDR, IRQ0 two channels; C3 25MHz via MMCM to 100MHz" {} "Preserve board preset DDR/MIO"] {REQ:pinned+applied}
set gpio_config [list CONFIG.C_GPIO_WIDTH 2 CONFIG.C_ALL_INPUTS 1 CONFIG.C_ALL_OUTPUTS 0 CONFIG.C_IS_DUAL 0 CONFIG.C_INTERRUPT_PRESENT 0]
vpr_capture gpio_create [list ipcfg::create_cell_cfg xilinx.com:ip:axi_gpio fsr_gpio $gpio_config] {SUCCESS:}
vpr_capture ctrl_create [list ipcfg::create_cell_cfg xilinx.com:ip:smartconnect ctrl_bus [list CONFIG.NUM_SI 1 CONFIG.NUM_MI 3]] {SUCCESS:}
vpr_capture data_create [list ipcfg::create_cell_cfg xilinx.com:ip:smartconnect data_bus [list CONFIG.NUM_SI 2 CONFIG.NUM_MI 1]] {SUCCESS:}
set dma_config [list CONFIG.c_include_sg 0 CONFIG.c_include_mm2s 1 CONFIG.c_include_s2mm 1 CONFIG.c_sg_length_width 26 CONFIG.c_addr_width 40 CONFIG.c_m_axi_mm2s_data_width 32 CONFIG.c_m_axi_s2mm_data_width 32 CONFIG.c_m_axis_mm2s_tdata_width 32 CONFIG.c_s_axis_s2mm_tdata_width 32 CONFIG.c_include_mm2s_dre 0 CONFIG.c_include_s2mm_dre 0 CONFIG.c_mm2s_burst_size 16 CONFIG.c_s2mm_burst_size 16]
vpr_capture dma_create [list ipcfg::create_cell_cfg xilinx.com:ip:axi_dma dma $dma_config] {SUCCESS:}
set reset_config [list CONFIG.C_EXT_RESET_HIGH 0 CONFIG.C_AUX_RESET_HIGH 1]
vpr_capture reset_create [list ipcfg::create_cell_cfg xilinx.com:ip:proc_sys_reset reset_i $reset_config] {SUCCESS:}
vpr_capture one_create [list ipcfg::create_cell_cfg xilinx.com:ip:xlconstant one [list CONFIG.CONST_WIDTH 1 CONFIG.CONST_VAL 1]] {SUCCESS:}
vpr_capture zero_create [list ipcfg::create_cell_cfg xilinx.com:ip:xlconstant zero [list CONFIG.CONST_WIDTH 1 CONFIG.CONST_VAL 0]] {SUCCESS:}
vpr_capture irq_create [list ipcfg::create_cell_cfg xilinx.com:ip:xlconcat irq_concat [list CONFIG.NUM_PORTS 8]] {SUCCESS:}
create_bd_cell -type module -reference vision_preprocess_bd resize
create_bd_cell -type module -reference fsr_sync2 sync_i
set clkp [create_bd_port -dir I -type clk clk_25mhz]
set_property CONFIG.FREQ_HZ 25000000 $clkp
create_bd_port -dir I -from 1 -to 0 fsr_do
# PG065: fixed MMCM, C3 25 MHz -> single 100 MHz PL domain. Reset holds until locked.
set clock_config [list CONFIG.PRIM_IN_FREQ 25.000 CONFIG.CLKOUT1_REQUESTED_OUT_FREQ 100.000 CONFIG.PRIM_SOURCE Single_ended_clock_capable_pin CONFIG.USE_RESET false CONFIG.USE_LOCKED true]
vpr_capture clock_create [list ipcfg::create_cell_cfg xilinx.com:ip:clk_wiz clock_i $clock_config] {SUCCESS:}
connect_bd_net [get_bd_ports clk_25mhz] [get_bd_pins clock_i/clk_in1]
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins ctrl_bus/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins ctrl_bus/M00_AXI] [get_bd_intf_pins fsr_gpio/S_AXI]
connect_bd_intf_net [get_bd_intf_pins ctrl_bus/M01_AXI] [get_bd_intf_pins dma/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins ctrl_bus/M02_AXI] [get_bd_intf_pins resize/s_axi]
connect_bd_intf_net [get_bd_intf_pins dma/M_AXIS_MM2S] [get_bd_intf_pins resize/s_axis]
connect_bd_intf_net [get_bd_intf_pins resize/m_axis] [get_bd_intf_pins dma/S_AXIS_S2MM]
connect_bd_intf_net [get_bd_intf_pins dma/M_AXI_MM2S] [get_bd_intf_pins data_bus/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins dma/M_AXI_S2MM] [get_bd_intf_pins data_bus/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins data_bus/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP0_FPD]
connect_bd_net [get_bd_pins clock_i/clk_out1] [get_bd_pins ps/maxihpm0_fpd_aclk] [get_bd_pins ps/saxihp0_fpd_aclk] [get_bd_pins ctrl_bus/aclk] [get_bd_pins data_bus/aclk] [get_bd_pins fsr_gpio/s_axi_aclk] [get_bd_pins dma/s_axi_lite_aclk] [get_bd_pins dma/m_axi_mm2s_aclk] [get_bd_pins dma/m_axi_s2mm_aclk] [get_bd_pins resize/aclk] [get_bd_pins reset_i/slowest_sync_clk] [get_bd_pins sync_i/clk]
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins reset_i/ext_reset_in]
connect_bd_net [get_bd_pins clock_i/locked] [get_bd_pins reset_i/dcm_locked]
connect_bd_net [get_bd_pins zero/dout] [get_bd_pins reset_i/aux_reset_in] [get_bd_pins reset_i/mb_debug_sys_rst]
connect_bd_net [get_bd_pins reset_i/peripheral_aresetn] [get_bd_pins ctrl_bus/aresetn] [get_bd_pins data_bus/aresetn] [get_bd_pins fsr_gpio/s_axi_aresetn] [get_bd_pins dma/axi_resetn] [get_bd_pins resize/aresetn] [get_bd_pins sync_i/resetn]
connect_bd_net [get_bd_ports fsr_do] [get_bd_pins sync_i/din]
connect_bd_net [get_bd_pins sync_i/dout] [get_bd_pins fsr_gpio/gpio_io_i]
connect_bd_net [get_bd_pins dma/mm2s_introut] [get_bd_pins irq_concat/In0]
connect_bd_net [get_bd_pins dma/s2mm_introut] [get_bd_pins irq_concat/In1]
for {set i 2} {$i<8} {incr i} {connect_bd_net [get_bd_pins zero/dout] [get_bd_pins irq_concat/In$i]}
connect_bd_net [get_bd_pins irq_concat/dout] [get_bd_pins ps/pl_ps_irq0]
assign_bd_address -offset 0xA0000000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps/Data] [get_bd_addr_segs fsr_gpio/S_AXI/Reg] -force
assign_bd_address -offset 0xA0040000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps/Data] [get_bd_addr_segs dma/S_AXI_LITE/Reg] -force
assign_bd_address -offset 0xA0050000 -range 0x00010000 -target_address_space [get_bd_addr_spaces ps/Data] [get_bd_addr_segs resize/s_axi/reg0] -force
assign_bd_address
vpr_capture validate [list validate_bd_design]
# Re-pin the declared 32-bit DMA masters after SmartConnect propagation.
vpr_capture dma_width_readback [list ipcfg::require dma $dma_config "32-bit DMA masters and frame streams" {} "Verified VPR1 propagation rule"] {REQ:pinned+applied}
vpr_capture validate [list validate_bd_design]
foreach {cell config} [list ps $ps_config fsr_gpio $gpio_config dma $dma_config reset_i $reset_config clock_i $clock_config] {
    set bad [ipcfg::verify_stuck $cell $config]
    if {[llength $bad]} {error "Readback mismatch $cell: $bad"}
}
if {[get_property CONFIG.DATA_WIDTH [get_bd_intf_pins ps/S_AXI_HP0_FPD]] != 128} {error "HP0 width mismatch"}
if {[get_property CONFIG.TDATA_NUM_BYTES [get_bd_intf_pins dma/M_AXIS_MM2S]] != 4} {error "MM2S not 32-bit"}
if {[get_property CONFIG.TDATA_NUM_BYTES [get_bd_intf_pins dma/S_AXIS_S2MM]] != 4} {error "S2MM not 32-bit"}
set f [open [file join $::VPR_EVIDENCE address_map.txt] w]
foreach s [get_bd_addr_segs -hier] {puts $f "$s OFFSET=[get_property OFFSET $s] RANGE=[get_property RANGE $s]"}
close $f
vpr_capture requirements [list ipcfg::audit_requirements] {REQ:OK REQ:empty}
save_bd_design
write_bd_tcl -force [file join $::VPR_EVIDENCE vision_bd_recreate.tcl]
generate_target all [get_files vision_bd.bd]
set_property synth_checkpoint_mode None [get_files vision_bd.bd]
update_compile_order -fileset sources_1
puts "VPR_BOARD_SETUP_COMPLETE clock=100MHz core=VPR3_RGB888 GPIO=A0000000 DMA=A0040000 VPR=A0050000 HP0=128"

if {[get_property CONFIG.FREQ_HZ [get_bd_pins clock_i/clk_out1]] != 100000000} {error "Clock frequency readback mismatch"}

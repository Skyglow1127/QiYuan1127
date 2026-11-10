# Continue the same BD after adding the Verilog module-reference wrapper.
create_bd_cell -type module -reference fsr_sync2 sync_i
set clkp [create_bd_port -dir I -type clk clk_25mhz]
set_property CONFIG.FREQ_HZ 25000000 $clkp
create_bd_port -dir I -from 1 -to 0 fsr_do
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins ctrl_bus/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins ctrl_bus/M00_AXI] [get_bd_intf_pins fsr_gpio/S_AXI]
connect_bd_intf_net [get_bd_intf_pins ctrl_bus/M01_AXI] [get_bd_intf_pins dma/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins ctrl_bus/M02_AXI] [get_bd_intf_pins resize/s_axi]
connect_bd_intf_net [get_bd_intf_pins dma/M_AXIS_MM2S] [get_bd_intf_pins resize/s_axis]
connect_bd_intf_net [get_bd_intf_pins resize/m_axis] [get_bd_intf_pins dma/S_AXIS_S2MM]
connect_bd_intf_net [get_bd_intf_pins dma/M_AXI_MM2S] [get_bd_intf_pins data_bus/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins dma/M_AXI_S2MM] [get_bd_intf_pins data_bus/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins data_bus/M00_AXI] [get_bd_intf_pins ps/S_AXI_HP0_FPD]
connect_bd_net [get_bd_ports clk_25mhz] [get_bd_pins ps/maxihpm0_fpd_aclk] [get_bd_pins ps/saxihp0_fpd_aclk] [get_bd_pins ctrl_bus/aclk] [get_bd_pins data_bus/aclk] [get_bd_pins fsr_gpio/s_axi_aclk] [get_bd_pins dma/s_axi_lite_aclk] [get_bd_pins dma/m_axi_mm2s_aclk] [get_bd_pins dma/m_axi_s2mm_aclk] [get_bd_pins resize/aclk] [get_bd_pins reset_i/slowest_sync_clk] [get_bd_pins sync_i/clk]
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins reset_i/ext_reset_in]
connect_bd_net [get_bd_pins one/dout] [get_bd_pins reset_i/dcm_locked]
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
foreach {cell config} [list ps $ps_config fsr_gpio $gpio_config dma $dma_config reset_i $reset_config] {
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
puts "VPR_BOARD_SETUP_COMPLETE clock=25MHz core=VPR1 GPIO=A0000000 DMA=A0040000 VPR=A0050000 HP0=128"

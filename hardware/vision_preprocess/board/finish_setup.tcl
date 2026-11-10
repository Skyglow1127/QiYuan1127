set dma_config [lreplace $dma_config [lsearch -exact $dma_config CONFIG.c_m_axi_mm2s_data_width] [expr {[lsearch -exact $dma_config CONFIG.c_m_axi_mm2s_data_width]+1}] CONFIG.c_m_axi_mm2s_data_width 32]
set dma_config [lreplace $dma_config [lsearch -exact $dma_config CONFIG.c_m_axi_s2mm_data_width] [expr {[lsearch -exact $dma_config CONFIG.c_m_axi_s2mm_data_width]+1}] CONFIG.c_m_axi_s2mm_data_width 32]
vpr_capture dma_width_repair [list ipcfg::require dma $dma_config "32-bit frame stream, 32-bit DMA buses; SmartConnect adapts to HP0 128" {} "Resolve IPI width propagation without ignored writes"] {REQ:pinned+applied}
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

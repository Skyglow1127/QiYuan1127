set root [file normalize [file dirname [info script]]]
create_project -force depth_gate_board $root/board_build -part xck26-sfvc784-2LV-c
set_property ip_repo_paths [list $root/depth_gate_hls/solution1/impl/ip] [current_project]
update_ip_catalog
create_bd_design system
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 ps
set_property -dict [list CONFIG.PSU__USE__M_AXI_GP0 {1} CONFIG.PSU__USE__M_AXI_GP1 {0} CONFIG.PSU__USE__M_AXI_GP2 {0} CONFIG.PSU__FPGA_PL0_ENABLE {1} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100}] [get_bd_cells ps]
create_bd_cell -type ip -vlnv xilinx.com:hls:depth_gate:1.0 gate
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 fabric
set_property -dict [list CONFIG.NUM_SI {4} CONFIG.NUM_MI {2}] [get_bd_cells fabric]
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_bram_ctrl:4.1 bram_ctrl
set_property -dict [list CONFIG.DATA_WIDTH {64} CONFIG.SINGLE_PORT_BRAM {1} CONFIG.ECC_TYPE {0}] [get_bd_cells bram_ctrl]
create_bd_cell -type ip -vlnv xilinx.com:ip:blk_mem_gen:8.4 data_bram
set_property -dict [list CONFIG.Memory_Type {Single_Port_RAM} CONFIG.Use_Byte_Write_Enable {true} CONFIG.Byte_Size {8} CONFIG.Write_Width_A {64} CONFIG.Read_Width_A {64} CONFIG.Write_Depth_A {8192} CONFIG.Enable_A {Use_ENA_Pin}] [get_bd_cells data_bram]
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 rst
set_property CONFIG.C_EXT_RESET_HIGH {0} [get_bd_cells rst]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 locked
set_property CONFIG.CONST_VAL {1} [get_bd_cells locked]
connect_bd_intf_net [get_bd_intf_pins ps/M_AXI_HPM0_FPD] [get_bd_intf_pins fabric/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins gate/m_axi_gmem0] [get_bd_intf_pins fabric/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins gate/m_axi_gmem1] [get_bd_intf_pins fabric/S02_AXI]
connect_bd_intf_net [get_bd_intf_pins gate/m_axi_gmem2] [get_bd_intf_pins fabric/S03_AXI]
connect_bd_intf_net [get_bd_intf_pins fabric/M00_AXI] [get_bd_intf_pins gate/s_axi_control]
connect_bd_intf_net [get_bd_intf_pins fabric/M01_AXI] [get_bd_intf_pins bram_ctrl/S_AXI]
connect_bd_intf_net [get_bd_intf_pins bram_ctrl/BRAM_PORTA] [get_bd_intf_pins data_bram/BRAM_PORTA]
connect_bd_net [get_bd_pins ps/pl_clk0] [get_bd_pins ps/maxihpm0_fpd_aclk] [get_bd_pins fabric/aclk] [get_bd_pins bram_ctrl/s_axi_aclk] [get_bd_pins gate/ap_clk] [get_bd_pins rst/slowest_sync_clk]
connect_bd_net [get_bd_pins ps/pl_resetn0] [get_bd_pins rst/ext_reset_in]
connect_bd_net [get_bd_pins locked/dout] [get_bd_pins rst/dcm_locked]
connect_bd_net [get_bd_pins rst/interconnect_aresetn] [get_bd_pins fabric/aresetn]
connect_bd_net [get_bd_pins rst/peripheral_aresetn] [get_bd_pins bram_ctrl/s_axi_aresetn] [get_bd_pins gate/ap_rst_n]
assign_bd_address -offset 0xA0000000 -range 64K -target_address_space [get_bd_addr_spaces ps/Data] [get_bd_addr_segs gate/s_axi_control/Reg] -force
foreach master {ps/Data gate/Data_m_axi_gmem0 gate/Data_m_axi_gmem1 gate/Data_m_axi_gmem2} {
 assign_bd_address -offset 0xA0010000 -range 64K -target_address_space [get_bd_addr_spaces $master] [get_bd_addr_segs bram_ctrl/S_AXI/Mem0] -force
}
validate_bd_design
save_bd_design
generate_target all [get_files $root/board_build/depth_gate_board.srcs/sources_1/bd/system/system.bd]
make_wrapper -files [get_files $root/board_build/depth_gate_board.srcs/sources_1/bd/system/system.bd] -top
add_files -norecurse $root/board_build/depth_gate_board.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]
launch_runs synth_1 -jobs 2
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "Synthesis failed"}
launch_runs impl_1 -to_step write_bitstream -jobs 2
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "Implementation failed"}
open_run impl_1
report_timing_summary -file $root/board_timing.rpt
report_utilization -file $root/board_utilization.rpt
write_hw_platform -fixed -include_bit -force -file $root/depth_gate_board.xsa
exit

set root [file normalize [file join [file dirname [info script]] ..]]
set_param general.maxThreads 2
create_project sfls_kr260 [file join $root build vivado] -part xck26-sfvc784-2LV-c -force
set_property board_part xilinx.com:kr260_som:part0:1.1 [current_project]
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]
add_files [glob [file join $root rtl *.sv]]
set_property top lamor_frequency_meter [current_fileset]
add_files -fileset sim_1 [file join $root sim tb_end_to_end.sv]
set_property top tb_end_to_end [get_filesets sim_1]
set_property xsim.simulate.runtime all [get_filesets sim_1]
set_property xsim.elaborate.debug_level typical [get_filesets sim_1]
create_fileset -simset sim_precision
add_files -fileset sim_precision [file join $root sim tb_end_to_end.sv]
set_property top tb_end_to_end [get_filesets sim_precision]
set_property generic {WINDOW_LOG2=15 EDGE_STRIDE=64} [get_filesets sim_precision]
set_property xsim.simulate.runtime all [get_filesets sim_precision]
set_property -dict [list xsim.simulate.xsim.more_options "-testplusarg RUN_MS=6020 -testplusarg FREQUENCY=349999.999"] [get_filesets sim_precision]
create_fileset -simset sim_arithmetic
add_files -fileset sim_arithmetic [file join $root sim tb_replay.sv]
set_property top tb_replay [get_filesets sim_arithmetic]
set_property generic {WINDOW_LOG2=7 EDGE_STRIDE=1} [get_filesets sim_arithmetic]
set_property xsim.simulate.runtime all [get_filesets sim_arithmetic]
set_property -dict [list xsim.simulate.xsim.more_options "-testplusarg VECTORS=[file join $root sim vectors small.txt]"] [get_filesets sim_arithmetic]
current_fileset -simset [get_filesets sim_1]
add_files -fileset constrs_1 [file join $root scripts core_timing.xdc]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
update_compile_order -fileset sim_precision
update_compile_order -fileset sim_arithmetic
puts "PROJECT_READY=[get_property DIRECTORY [current_project]]"
close_project

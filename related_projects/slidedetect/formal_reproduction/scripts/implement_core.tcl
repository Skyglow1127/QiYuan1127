set root [file normalize [file join [file dirname [info script]] ..]]
if {[llength $argv] != 2} {error "Usage: implement_core.tcl WINDOW_SIZE OUTPUT_DIR"}
set m [lindex $argv 0]
set out [file normalize [lindex $argv 1]]
if {![string is integer -strict $m] || $m < 2} {error "WINDOW_SIZE must be at least 2"}
file mkdir $out
set_param general.maxThreads 2
set part xck26-sfvc784-2LV-c
create_project -in_memory -part $part
read_verilog -sv [file join $root rtl udiv_round.sv]
read_verilog -sv [file join $root rtl sfls_core.sv]
read_xdc [file join $root scripts core_200mhz.xdc]
synth_design -top sfls_core -part $part -mode out_of_context -generic WINDOW_SIZE=$m -generic CLOCK_HZ=200000000 -generic EDGE_STRIDE=1
report_utilization -file [file join $out utilization_synth.rpt]
report_timing_summary -delay_type min_max -report_unconstrained -file [file join $out timing_synth.rpt]
opt_design
place_design
phys_opt_design
route_design
report_utilization -file [file join $out utilization_routed.rpt]
report_timing_summary -delay_type min_max -report_unconstrained -file [file join $out timing_routed.rpt]
report_timing -max_paths 10 -path_type full_clock_expanded -file [file join $out critical_paths.rpt]
report_drc -file [file join $out drc.rpt]
check_timing -verbose -file [file join $out check_timing.rpt]
set setup [get_property SLACK [get_timing_paths -max_paths 1]]
set hold [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
set internal [get_timing_paths -from [all_registers -clock clk] -to [all_registers -clock clk] -max_paths 1]
set internal_setup [get_property SLACK $internal]
set internal_hold [get_property SLACK [get_timing_paths -from [all_registers -clock clk] -to [all_registers -clock clk] -delay_type min -max_paths 1]]
set fp [open [file join $out status.txt] w]
puts $fp "Vivado=[version -short]"
puts $fp "part=$part"
puts $fp "WINDOW_SIZE=$m"
puts $fp "clock_mhz=200"
puts $fp "WNS=$setup"
puts $fp "WHS=$hold"
puts $fp "internal_WNS=$internal_setup"
puts $fp "internal_WHS=$internal_hold"
puts $fp "mode=out_of_context; not a board timing signoff or bitstream"
close $fp
puts "CORE_TIMING_COMPLETE M=$m WNS=$setup WHS=$hold INTERNAL_WNS=$internal_setup INTERNAL_WHS=$internal_hold"
# A timing violation is a measured result, not a tool execution failure.
# The Python runner records timing_met separately from execution success.

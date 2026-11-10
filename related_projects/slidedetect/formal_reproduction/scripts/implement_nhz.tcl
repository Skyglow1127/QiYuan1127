set root [file normalize [file join [file dirname [info script]] ..]]
if {[llength $argv]!=1} {error "Usage: implement_nhz.tcl OUTPUT_DIR"}
set out [file normalize [lindex $argv 0]]
file mkdir $out
set_param general.maxThreads 2
create_project -in_memory -part xck26-sfvc784-2LV-c
read_verilog -sv [file join $root rtl udiv_round.sv]
read_verilog -sv [file join $root rtl sfls_nhz_readout.sv]
read_xdc [file join $root scripts nhz_200mhz.xdc]
synth_design -top sfls_nhz_readout -part xck26-sfvc784-2LV-c -mode out_of_context -generic WINDOW_SIZE=2130
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -delay_type min_max -report_unconstrained -file [file join $out timing_routed.rpt]
report_utilization -file [file join $out utilization.rpt]
report_drc -file [file join $out drc.rpt]
check_timing -verbose -file [file join $out check_timing.rpt]
set setup [get_property SLACK [get_timing_paths -max_paths 1]]
set hold [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
set fp [open [file join $out status.txt] w]
puts $fp "WNS=$setup"
puts $fp "WHS=$hold"
close $fp
puts "NHZ_TIMING_COMPLETE WNS=$setup WHS=$hold"

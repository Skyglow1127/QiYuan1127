set root [file normalize [file join [file dirname [info script]] ..]]
if {[llength $argv]!=2} {error "Usage: implement_sensor.tcl HOP_EVENTS OUTPUT_DIR"}
set hop [lindex $argv 0]
if {$hop ni {0 24}} {error "Only validated M=96 profiles: hop 0 or 24"}
set out [file normalize [lindex $argv 1]]
file mkdir $out
set_param general.maxThreads 2
create_project -in_memory -part xck26-sfvc784-2LV-c
foreach name {udiv_round sfls_core sfls_sensor} {read_verilog -sv [file join $root rtl ${name}.sv]}
read_xdc [file join $root scripts sensor_200mhz.xdc]
synth_design -top sfls_sensor -part xck26-sfvc784-2LV-c -mode out_of_context -generic WINDOW_SIZE=96 -generic HOP_EVENTS=$hop
if {[llength [get_pins -hier -filter {NAME =~ *sync1_reg/D}]]!=1} {error "First-stage CDC exception did not resolve uniquely"}
report_timing_summary -delay_type min_max -report_unconstrained -file [file join $out timing_synth.rpt]
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -delay_type min_max -report_unconstrained -file [file join $out timing_routed.rpt]
report_utilization -file [file join $out utilization.rpt]
report_timing -max_paths 10 -file [file join $out critical_paths.rpt]
report_cdc -details -file [file join $out cdc.rpt]
report_drc -file [file join $out drc.rpt]
check_timing -verbose -file [file join $out check_timing.rpt]
set setup [get_property SLACK [get_timing_paths -max_paths 1]]
set hold [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
set fp [open [file join $out status.txt] w]
puts $fp "Vivado=[version -short]"
puts $fp "WNS=$setup"
puts $fp "WHS=$hold"
puts $fp "HOP_EVENTS=$hop"
close $fp
write_checkpoint -force [file join $out sensor_routed.dcp]
puts "SENSOR_TIMING_COMPLETE HOP=$hop WNS=$setup WHS=$hold"

set root [file normalize [file join [file dirname [info script]] ..]]
set_param general.maxThreads 2
set stride 1
if {[llength $argv]>0} {set stride [lindex $argv 0]}
if {$stride ni {1 64}} {error "Supported stride: 1 or 64"}
set suffix [expr {$stride==1 ? "" : "_precision"}]
set out [file join $root build implementation${suffix}]
file mkdir $out
create_project -in_memory -part xck26-sfvc784-2LV-c
read_verilog -sv [glob [file join $root rtl *.sv]]
read_xdc [file join $root scripts core_timing.xdc]
synth_design -top lamor_frequency_meter -part xck26-sfvc784-2LV-c -mode out_of_context -generic EDGE_STRIDE=$stride
write_checkpoint -force [file join $out synthesized.dcp]
report_utilization -file [file join $root reports utilization_synth${suffix}.rpt]
report_timing_summary -file [file join $root reports timing_synth${suffix}.rpt]
opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force [file join $out routed.dcp]
report_utilization -file [file join $root reports utilization_routed${suffix}.rpt]
report_timing_summary -delay_type min_max -report_unconstrained -file [file join $root reports timing_routed${suffix}.rpt]
report_cdc -details -file [file join $root reports cdc${suffix}.rpt]
report_drc -file [file join $root reports drc${suffix}.rpt]
set path [get_timing_paths -max_paths 1]
set slack [get_property SLACK $path]
set min_slack [get_property SLACK [get_timing_paths -delay_type min -max_paths 1]]
puts "ROUTED_WNS=$slack"
puts "ROUTED_WHS=$min_slack"
write_verilog -force -mode timesim -sdf_anno true [file join $out meter_timesim.v]
write_sdf -force [file join $out meter_timesim.sdf]
set fp [open [file join $root reports implementation_status${suffix}.txt] w]
puts $fp "Vivado=[version -short]"
puts $fp "part=xck26-sfvc784-2LV-c"
puts $fp "mode=out_of_context; clock=100 MHz; uncertainty=0.1 ns"
puts $fp "EDGE_STRIDE=$stride"
puts $fp "WNS=$slack"
puts $fp "WHS=$min_slack"
close $fp
if {$slack<0 || $min_slack<0} {error "Setup or hold timing closure failed"}

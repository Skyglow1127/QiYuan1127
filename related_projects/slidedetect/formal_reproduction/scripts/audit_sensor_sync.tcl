set root [file normalize [file join [file dirname [info script]] ..]]
foreach hop {0 24} {
    set out [file join $root results sensor_timing_hop${hop}]
    open_checkpoint [file join $out sensor_routed.dcp]
    set fp [open [file join $out synchronizer_audit.txt] w]
    foreach name {sync1_reg sync2_reg} {
        set cell [get_cells $name]
        if {[llength $cell]!=1} {error "Missing synchronizer cell: $name"}
        puts $fp "$name REF_NAME=[get_property REF_NAME $cell] ASYNC_REG=[get_property ASYNC_REG $cell] LOC=[get_property LOC $cell]"
        if {![get_property ASYNC_REG $cell]} {error "Missing synchronizer attribute: $name"}
    }
    set sinks [get_pins -of_objects [get_nets -of_objects [get_pins sync1_reg/Q]] -filter {DIRECTION == IN}]
    if {[llength $sinks]!=1 || [get_property NAME $sinks] ne "sync2_reg/D"} {error "Unexpected first-stage synchronizer fanout: $sinks"}
    set setup [get_timing_paths -from [get_pins sync1_reg/Q] -to [get_pins sync2_reg/D] -max_paths 1]
    set hold [get_timing_paths -from [get_pins sync1_reg/Q] -to [get_pins sync2_reg/D] -delay_type min -max_paths 1]
    if {[llength $setup]!=1 || [llength $hold]!=1} {error "Second synchronizer stage is not timed"}
    puts $fp "stage1_to_stage2_setup_slack=[get_property SLACK $setup]"
    puts $fp "stage1_to_stage2_hold_slack=[get_property SLACK $hold]"
    puts $fp "CDC report warning is retained. This structural check does not establish physical MTBF."
    close $fp
    close_design
}
puts "SYNCHRONIZER_STRUCTURE_PASS"

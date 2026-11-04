# Private PHYS_OPT_DESIGN.TCL.POST hook; ENGINEERING CAPACITY ONLY, NOT STA.
# No exit/return-success on failure. An error must propagate to native worker.
namespace eval ::robot_rt_capacity_gate {
    variable source_path [file normalize [info script]]
    proc sha {path} {return [lindex [exec sha256sum -- $path] 0]}
    proc save {path value} {
        set f [open $path {WRONLY CREAT EXCL}]
        try {puts $f [list set robot_rt_capacity_evidence $value]} finally {close $f}
    }
    proc parse_clb {text} {
        set rows {}
        foreach line [split $text \n] {
            set fields [split $line |]
            if {[llength $fields] >= 2 && [string trim [lindex $fields 1]] eq "CLB"} {
                lappend rows $fields
            }
        }
        if {[llength $rows] != 1} {error "CAPACITY_FORMAT: require exactly one actual CLB row"}
        set row [lindex $rows 0]
        if {[llength $row] != 8 || [string trim [lindex $row 0]] ne "" || [string trim [lindex $row 7]] ne ""} {
            error "CAPACITY_FORMAT: malformed CLB column count"
        }
        set used [string trim [lindex $row 2]]
        set avail [string trim [lindex $row 5]]
        foreach n [list $used $avail] {
            if {![regexp {^(0|[1-9][0-9]*)$} $n]} {error "CAPACITY_FORMAT: CLB counts must be decimal integers"}
        }
        if {$avail <= 0 || $used > $avail} {error "CAPACITY_FORMAT: invalid CLB denominator or used count"}
        # Fixed K26 part profile. Never accept an inflated/foreign denominator.
        if {$avail != 14640} {error "CAPACITY_FORMAT: K26 CLB availability differs from14640"}
        return [dict create CLB_used $used CLB_available $avail numerator_percent 95 accepted [expr {100*$used <= 95*$avail}]]
    }
    proc run {} {
        variable source_path
        if {![string equal -nocase [get_property PART [current_design]] xck26-sfvc784-2LV-c] ||
            [get_property TOP [current_design]] ne "robot_rt_bd_wrapper"} {error "CAPACITY_PROFILE: unexpected actual PART/TOP"}
        set out [file join [pwd] robot_rt_capacity_gate]
        if {[file exists $out]} {error "CAPACITY_FRESH: diagnostic directory already exists"}
        file mkdir $out
        set util [file join $out utilization_physopt.rpt]
        set controls [file join $out control_sets_physopt.rpt]
        set dcp [file join $out physopt_capacity_UNACCEPTED.dcp]
        report_utilization -file $util
        report_control_sets -verbose -file $controls
        # POST is after phys_opt_design; do not assume the vendor checkpoint
        # has already been saved. Preserve our own explicitly UNACCEPTED DCP.
        write_checkpoint $dcp
        set f [open $util r];try {set text [read $f]} finally {close $f}
        set result [parse_clb $text]
        dict set result scope ENGINEERING_CAPACITY_ONLY_NOT_ROUTE_OR_STA
        dict set result hook $source_path
        dict set result hook_sha256 [sha $source_path]
        dict set result utilization_sha256 [sha $util]
        dict set result controls_sha256 [sha $controls]
        dict set result diagnostic_dcp $dcp
        dict set result diagnostic_dcp_sha256 [sha $dcp]
        dict set result final_STA_accepted 0
        save [file join $out evidence.tcl] $result
        if {![dict get $result accepted]} {
            return -code error -errorcode {ROBOT_RT CAPACITY_EXCEEDED} "ROBOT_RT_CAPACITY_REJECT: actual CLB [dict get $result CLB_used]/[dict get $result CLB_available] exceeds engineering95percent; route must not start"
        }
        puts "PASS ROBOT_RT_CAPACITY_GATE CLB [dict get $result CLB_used]/[dict get $result CLB_available] ENGINEERING95_ONLY_NOT_STA"
    }
}
# NATIVE_HOOK_ENTRY: no library flag can suppress this call in a worker.
::robot_rt_capacity_gate::run

# Optional partition support; manifests are parsed as literal lists, never sourced.
proc partition_literal {path variable} {
    set fd [open $path r];set text [string trim [read $fd]];close $fd
    if {[catch {llength $text} n] || $n != 3 || [lindex $text 0] ne "set" || [lindex $text 1] ne $variable} {
        error "Expected one literal set $variable: $path"
    }
    return [lindex $text 2]
}
proc partition_sha {path} {return [lindex [exec sha256sum -- $path] 0]}
# V2 semantic contract. All inputs below are literal data; never source/eval them.
proc partition_generic_map {value module expected_gate} {
    if {[catch {llength $value} n] || $n%2} {error "Malformed HDL generic map"}
    set result {}
    foreach {key number} $value {
        if {[dict exists $result $key] || ![regexp {^(0|[1-9][0-9]*)$} $number]} {error "Duplicate/noncanonical generic"}
        dict set result $key $number
    }
    set wanted [dict create CLOCK_HZ 99999001]
    if {$module eq "limit_gate_pipe"} {dict set wanted STABLE_JOB_INPUTS $expected_gate
    } elseif {$module ne "player_math"} {error "Unsupported generic module"}
    if {[lsort [dict keys $result]] ne [lsort [dict keys $wanted]]} {error "Unexpected/missing HDL generic"}
    dict for {key number} $wanted {if {[dict get $result $key] ne $number} {error "HDL generic mismatch: $key"}}
    return $wanted
}
proc partition_parse_synth_command {line module expected_gate directive} {
    if {![string match "Command: synth_design *" $line]} {error "Missing actual synth Command"}
    set text [string range $line 9 end]
    foreach bad [list {$} {[} {]} {;} "\\" "\""] {
        if {[string first $bad $text] >= 0} {error "Dynamic/nonliteral synth Command"}
    }
    if {[catch {llength $text} n] || $n != 11 || [lindex $text 0] ne "synth_design"} {error "Unsupported actual synth Command"}
    set options {}
    foreach {key value} [lrange $text 1 end] {
        if {[dict exists $options $key]} {error "Duplicate synth option"}
        dict set options $key $value
    }
    if {[lsort [dict keys $options]] ne [lsort {-top -part -mode -generic -directive}] ||
        [dict get $options -top] ne $module || [dict get $options -part] ne "xck26-sfvc784-2LV-c" ||
        [dict get $options -mode] ne "out_of_context" || [dict get $options -directive] ne $directive} {
        error "Actual synth profile mismatch"
    }
    set generic_pairs {}
    set generic_names {}
    foreach token [dict get $options -generic] {
        if {![regexp {^([A-Z_][A-Z_0-9]*)=(0|[1-9][0-9]*)$} $token ignored key number] ||
            [dict exists $generic_names $key]} {error "Malformed/duplicate actual synth generic token"}
        dict set generic_names $key 1
        lappend generic_pairs $key $number
    }
    return [partition_generic_map $generic_pairs $module $expected_gate]
}
proc partition_sv_code {text} {
    # Remove comments AND strings so literal diagnostics cannot spoof instances.
    set out "";set state code;set n [string length $text]
    for {set i 0} {$i<$n} {incr i} {
        set c [string index $text $i];set next [string index $text [expr {$i+1}]]
        if {$state eq "line"} {
            if {$c eq "\n"} {set state code;append out "\n"} else {append out " "};continue
        }
        if {$state eq "block"} {
            if {$c eq "*" && $next eq "/"} {set state code;append out "  ";incr i
            } else {append out [expr {$c eq "\n" ? "\n" : " "}]};continue
        }
        if {$state eq "string"} {
            if {$c eq "\\"} {append out "  ";incr i
            } elseif {$c eq "\""} {set state code;append out " "
            } else {append out [expr {$c eq "\n" ? "\n" : " "}]};continue
        }
        if {$c eq "/" && $next eq "/"} {set state line;append out "  ";incr i
        } elseif {$c eq "/" && $next eq "*"} {set state block;append out "  ";incr i
        } elseif {$c eq "\""} {set state string;append out " "
        } else {append out $c}
    }
    if {$state ni {code line}} {error "Unterminated controlled parent source"}
    return $out
}
proc partition_parent_contract {p} {
    set expected [dict get $p expected_gate_stable_job_inputs]
    if {$expected ni {0 1}} {error "Invalid independent parent gate expectation"}
    foreach name {top shared} {
        set e [dict get $p parent_sources $name];set path [dict get $e path]
        if {[partition_sha $path] ne [dict get $e sha256]} {error "Parent semantic source changed: $name"}
        set fd [open $path r];set code($name) [partition_sv_code [read $fd]];close $fd
    }
    set top_matches [regexp -all -inline {shared_math_gate\s*#\s*\(([^;]*?)\)\s+shared_execution\s*\(} $code(top)]
    if {[llength $top_matches] != 2 || ![regexp {^\s*\.CLOCK_HZ\(\s*CLOCK_HZ\s*\)\s*,\s*\.STABLE_CACHE_INPUTS\(\s*([01])\s*\)\s*$} [lindex $top_matches 1] ignored flag] || $flag ne $expected} {
        error "Actual top stable-input constant differs from independent expectation"
    }
    set shared_matches [regexp -all -inline {limit_gate_pipe\s*#\s*\(([^;]*?)\)\s+gate\s*\(} $code(shared)]
    if {[llength $shared_matches] != 2 || ![regexp {^\s*\.CLOCK_HZ\(\s*CLOCK_HZ\s*\)\s*,\s*\.STABLE_JOB_INPUTS\(\s*STABLE_CACHE_INPUTS\s*\)\s*$} [lindex $shared_matches 1]]} {
        error "Shared gate parameter forwarding is not the reviewed literal form"
    }
    if {![regexp {module\s+shared_math_gate\s*#\s*\(\s*parameter\s+integer\s+CLOCK_HZ\s*=\s*100000000\s*,\s*parameter\s+integer\s+STABLE_CACHE_INPUTS\s*=\s*0\s*\)\s*\(} $code(shared)]} {
        error "Shared general parameter defaults changed"
    }
    return $expected
}
proc partition_check_parameters {p} {
    if {[dict get $p schema] ne "robot_rt.partition.v2"} {error "Parameter-bound partition v2 required"}
    set expected [partition_parent_contract $p]
    foreach module {player_math limit_gate_pipe} {
        set entry [dict get $p modules $module]
        set generics [partition_generic_map [dict get $entry hdl_generics] $module $expected]
        set contract [expr {$module eq "player_math" ? "math.exact.v1" : ($expected ? "gate.shared_owner_stable_job.v1" : "gate.snapshot.v1")}]
        if {[dict get $entry input_contract] ne $contract} {error "Input contract mismatch"}
        if {[dict get $entry synth_directive] ni {default runtimeoptimized}} {error "Unsupported child directive"}
        set kind [dict get $entry evidence_kind]
        if {$kind ni {fresh_v2 legacy_math_v1} || ($kind eq "legacy_math_v1" && $module ne "player_math")} {error "Invalid parameter evidence kind"}
        set ev [partition_literal [dict get $entry parameter_evidence] child_parameters]
        if {[dict get $ev schema] ne "robot_rt.child_parameters.v2" || [dict get $ev module] ne $module ||
            [partition_generic_map [dict get $ev hdl_generics] $module $expected] ne $generics ||
            [dict get $ev input_contract] ne $contract || [dict get $ev evidence_kind] ne $kind ||
            [dict get $ev part] ne [dict get $p part] || [dict get $ev vivado_short] ne [dict get $p vivado_short] ||
            [dict get $ev synth_directive] ne [dict get $entry synth_directive]} {error "Parameter evidence/profile differs"}
        foreach key {source dcp compile_script synth_log tool_verification} {
            if {[dict get $ev ${key}_sha256] ne [dict get $entry ${key}_sha256]} {error "Parameter evidence SHA differs: $key"}
        }
        if {[dict exists $entry reuse_origin_manifest] &&
            (![dict exists $ev reuse_origin_manifest_sha256] || [dict get $ev reuse_origin_manifest_sha256] ne [dict get $entry reuse_origin_manifest_sha256])} {
            error "Reuse-origin parameter evidence differs"
        }
        set fd [open [dict get $entry synth_log] r];set log [read $fd];close $fd
        set child_sentinel "PASS CHILD_OOC $module CLOCK_HZ=99999001 PART=xck26-sfvc784-2LV-c CLOCK_NEUTRAL=1"
        set sentinel_count 0
        foreach line [split $log \n] {if {$line eq $child_sentinel} {incr sentinel_count}}
        if {$sentinel_count != 1 || [regexp -line {^(ERROR:|CHILD_ERROR:)} $log]} {error "Incomplete/failed child log"}
        set commands [regexp -all -inline -line {^Command: synth_design[^\n]*} $log]
        if {[llength $commands] != 1 || [partition_parse_synth_command [lindex $commands 0] $module $expected [dict get $entry synth_directive]] ne $generics} {
            error "Actual log does not prove selected parameter map"
        }
    }
}
proc partition_parameter_contract {p} {
    partition_check_parameters $p
    set result [dict create schema robot_rt.parent_parameters.v2 expected_gate_stable_job_inputs [dict get $p expected_gate_stable_job_inputs] parent_sources [dict get $p parent_sources]]
    foreach module {player_math limit_gate_pipe} {
        set entry [dict get $p modules $module]
        foreach key {hdl_generics input_contract evidence_kind source_sha256 dcp_sha256 parameter_evidence_sha256 compile_script_sha256 synth_log_sha256 tool_verification_sha256} {
            dict set result modules $module $key [dict get $entry $key]
        }
    }
    return $result
}

proc partition_entry_files {entry} {
    set keys {source source_snapshot stub dcp tool_stub port_signature compile_script synth_log tool_verification parameter_evidence}
    if {[dict exists $entry reuse_origin_manifest] != [dict exists $entry reuse_origin_manifest_sha256]} {error "Incomplete reuse-origin identity"}
    if {[dict exists $entry reuse_origin_manifest]} {lappend keys reuse_origin_manifest}
    return $keys
}
proc partition_files {p} {
    set files [list [dict get $p manifest_path]]
    foreach module {player_math limit_gate_pipe} {
        foreach key [partition_entry_files [dict get $p modules $module]] {lappend files [dict get $p modules $module $key]}
    }
    dict for {name entry} [dict get $p parent_sources] {lappend files [dict get $entry path]}
    return [lsort -unique $files]
}
proc partition_check_bytes {p} {
    foreach module {player_math limit_gate_pipe} {
        set entry [dict get $p modules $module]
        foreach key [partition_entry_files $entry] {
            set file [dict get $entry $key];set expected [dict get $entry ${key}_sha256]
            if {[file pathtype $file] ne "absolute" || ![file isfile $file] ||
                ![regexp {^[0-9a-f]{64}$} $expected] || [partition_sha $file] ne $expected} {error "Partition input/hash mismatch: $module $key"}
        }
    }
    partition_check_parameters $p
}
proc partition_load {directory repo {expected_gate 0}} {
    set manifest [file join [file normalize $directory] partition_plan.tcl]
    set p [partition_literal $manifest partition_plan]
    if {[dict get $p schema] ne "robot_rt.partition.v2" || [dict get $p clock_hz] != 99999001 ||
        [dict get $p part] ne "xck26-sfvc784-2LV-c" || [dict get $p vivado_short] ne [version -short] ||
        [lsort [dict keys [dict get $p modules]]] ne [lsort {player_math limit_gate_pipe}]} {error "Unsupported partition profile/schema"}
    if {$expected_gate ni {0 1}} {error "Independent parent gate mode must be0 or1"}
    dict set p expected_gate_stable_job_inputs $expected_gate
    foreach {name file} {top robot_rt_top.sv shared shared_math_gate.sv} {
        set path [file normalize [file join $repo rtl $file]]
        dict set p parent_sources $name [dict create path $path sha256 [partition_sha $path]]
    }
    dict set p manifest_path $manifest
    foreach module {player_math limit_gate_pipe} {
        set e [dict get $p modules $module]
        if {[file normalize [dict get $e source]] ne [file normalize [file join $repo rtl ${module}.sv]] ||
            [dict get $e clock_hz] != [dict get $p clock_hz] || [dict get $e clock_neutral] != 1 ||
            [dict get $e dcp_reopen_clock_count] != 0 || [dict get $e source_snapshot_sha256] ne [dict get $e source_sha256]} {
            error "Partition source/profile not the selected repository: $module"
        }
        if {![llength [partition_literal [dict get $e port_signature] child_ports]]} {error "Empty child ports: $module"}
    }
    partition_check_bytes $p
    return $p
}
proc partition_sources {sources p} {
    foreach module {player_math limit_gate_pipe} {
        set source [dict get $p modules $module source];set matches {};set result {}
        foreach file $sources {
            if {[file normalize $file] eq [file normalize $source]} {lappend matches $file} else {lappend result $file}
        }
        if {[llength $matches] != 1} {error "Expected exactly one true $module input"}
        set sources [concat $result [list [dict get $p modules $module stub]]]
    }
    return $sources
}
proc partition_instances {} {
    return {inst/core_impl/shared_execution/math player_math
        inst/core_impl/shared_execution/gate limit_gate_pipe}
}
proc partition_blackboxes {expected} {
    set actual {}
    foreach cell [get_cells -hierarchical -filter {IS_BLACKBOX == 1}] {lappend actual [get_property NAME $cell]}
    if {[lsort $actual] ne [lsort $expected]} {error "Unexpected partition blackboxes: $actual"}
}
proc partition_instance_ports {p} {
    dict for {instance module} [partition_instances] {
        set cells [get_cells -quiet $instance]
        if {[llength $cells] != 1} {error "Missing exact partition instance: $instance"}
        set pins [get_pins -of_objects $cells]
        if {![llength $pins]} {error "Empty partition instance ports: $instance"}
        set refs [get_property REF_PIN_NAME $pins]
        set directions [get_property DIRECTION $pins]
        if {[llength $refs] != [llength $pins] || [llength $directions] != [llength $pins]} {
            error "Incomplete batch partition port readback: $instance"
        }
        set actual {}
        foreach ref $refs direction $directions {lappend actual [list $ref $direction]}
        set expected [partition_literal [dict get $p modules $module port_signature] child_ports]
        if {[lsort $actual] ne [lsort $expected]} {error "Partition port signature differs: $instance"}
    }
}
# Candidate shared check. Defining these procedures runs no Vivado command.
# Exactly two summary headings per category: TOC + underlined body, as emitted
# by the pinned Vivado2025.2 check_timing report. Missing/duplicate counts fail.
proc partition_check_timing_report {text} {
    set counts {}
    foreach section {no_clock constant_clock multiple_clock unconstrained_internal_endpoints loops latch_loops} {
        set header [format {^[ \t]*[0-9]+\.[ \t]+checking[ \t]+%s[ \t]+\(([0-9]+)\)[ \t]*\r?$} $section]
        set rows [regexp -all -inline -line -- $header $text]
        set body [format {^[ \t]*[0-9]+\.[ \t]+checking[ \t]+%s[ \t]+\(([0-9]+)\)[ \t]*\r?\n[ \t]*-+[ \t]*\r?$} $section]
        set body_rows [regexp -all -inline -line -- $body $text]
        if {[llength $rows] != 4 || [llength $body_rows] != 2} {
            error "Timing report must contain one TOC and one underlined body count for $section"
        }
        foreach count [list [lindex $rows 1] [lindex $rows 3] [lindex $body_rows 1]] {
            if {![regexp {^0+$} $count]} {error "Nonzero timing check $section: $count"}
        }
        dict set counts $section 0
    }
    return $counts
}
# Optional bound_dcp enables read-only reuse only for published DCP verification.
# Bulk assembly passes no DCP and keeps the original fresh-only protection.
proc partition_core_timing_checks {xdc fresh_directory {bound_dcp {}}} {
    set reuse [file exists $fresh_directory]
    if {$reuse && $bound_dcp eq ""} {error "Refusing timing report directory overwrite: $fresh_directory"}
    if {$reuse && ![file isdirectory $fresh_directory]} {error "Timing report cache is not a directory: $fresh_directory"}
    set report [file join $fresh_directory check_timing.rpt]
    set binding_path [file join $fresh_directory timing_binding.tcl]
    set dcp_hash {};set xdc_hash {}
    if {$bound_dcp ne ""} {
        foreach path [list $bound_dcp $xdc] {
            if {![file isfile $path] || ![file readable $path] || [file size $path]==0} {error "Missing/empty timing binding input: $path"}
        }
        set dcp_hash [lindex [exec sha256sum -- $bound_dcp] 0]
        set xdc_hash [lindex [exec sha256sum -- $xdc] 0]
        if {$reuse && (![file isfile $binding_path] || ![file isfile $report])} {error "Incomplete timing report cache: $fresh_directory"}
    }
    # Always inspect the newly opened live design, including cache verification.
    if {![llength [get_clocks]]} {read_xdc -no_add $xdc}
    set clocks [get_clocks];set input_clocks [get_clocks -of_objects [get_ports aclk]]
    if {[llength $clocks] != 1 || [llength $input_clocks] != 1 ||
        [get_property NAME [lindex $clocks 0]] ne "core_aclk_ooc" ||
        [lindex $clocks 0] ne [lindex $input_clocks 0] ||
        abs([get_property PERIOD [lindex $clocks 0]]-10.0)>0.000001} {
        error "Core must have sole original 10ns aclk clock"
    }
    # Vivado2025.2/K26 actual FDRE/FDSE probe reports PRIMITIVE_GROUP=REGISTER.
    # Capability subcells are counted, never treated as fabric FF/coverage.
    set capabilities [all_registers -cells]
    set fabric [get_cells -hierarchical -filter {PRIMITIVE_GROUP == REGISTER}]
    if {![llength $fabric]} {error "Partitioned sequential core has no fabric REGISTER cells"}
    set fabric_names [lsort [get_property NAME $fabric]]
    if {[llength $fabric_names] != [llength $fabric]} {error "Incomplete fabric NAME readback"}
    set clocked [all_registers -cells -clock [lindex $clocks 0]]
    set clocked_names {}
    if {[llength $clocked]} {
        set clocked_names [get_property NAME $clocked]
        if {[llength $clocked_names] != [llength $clocked]} {error "Incomplete clocked NAME readback"}
    }
    # A local array preserves exact name-set membership without passing the
    # entire large Tcl dictionary as an argument on every lookup. This may
    # avoid Vivado-specific conversion/tracing cost; that cause is not proven
    # by the ordinary Tcl benchmark, where both implementations are fast.
    array set clocked_present {}
    foreach name $clocked_names {set clocked_present($name) 1}
    foreach name $fabric_names {
        if {![info exists clocked_present($name)]} {error "Fabric register lacks sole core clock: $name"}
    }
    if {!$reuse} {
        file mkdir $fresh_directory
        check_timing -override_defaults {no_clock constant_clock multiple_clock unconstrained_internal_endpoints loops latch_loops} -verbose -file $report
    }
    if {![file isfile $report] || [file size $report] == 0} {error "Timing report missing/empty: $report"}
    set fd [open $report r];set text [read $fd];close $fd
    set counts [partition_check_timing_report $text]
    set report_hash [lindex [exec sha256sum -- $report] 0]
    set result [dict create clock_name core_aclk_ooc clock_period_ns 10.0 \
        fabric_register_count [llength $fabric] all_registers_capability_count [llength $capabilities] \
        fabric_register_names $fabric_names timing_counts $counts timing_report $report \
        timing_report_sha256 $report_hash]
    if {$bound_dcp ne ""} {
        if {[lindex [exec sha256sum -- $bound_dcp] 0] ne $dcp_hash ||
            [lindex [exec sha256sum -- $xdc] 0] ne $xdc_hash} {error "Timing binding inputs changed during verification"}
        set expected [dict create schema robot_rt.partition_timing_binding.v1 \
            bound_dcp_sha256 $dcp_hash xdc_sha256 $xdc_hash report_sha256 $report_hash \
            timing_counts $counts clock_name core_aclk_ooc clock_period_ns 10.0 \
            fabric_register_count [llength $fabric] all_registers_capability_count [llength $capabilities]]
        if {$reuse} {
            # partition_literal parses one literal set; never source/eval binding.
            set binding [partition_literal $binding_path partition_timing_binding]
            if {[lsort [dict keys $binding]] ne [lsort [dict keys $expected]]} {error "Incomplete/unexpected timing binding schema"}
            dict for {key value} $expected {
                if {[dict get $binding $key] ne $value} {error "Timing binding mismatch: $key"}
            }
        } else {
            set fd [open $binding_path {WRONLY CREAT EXCL}]
            puts $fd [list set partition_timing_binding $expected];close $fd
        }
        dict set result timing_binding $binding_path
        dict set result timing_binding_sha256 [lindex [exec sha256sum -- $binding_path] 0]
    }
    return $result
}

proc partition_write_hook {out p support} {
    set directory [file join $out split];file mkdir $directory
    set hook [file join $directory assemble_partitions_insert.tcl]
    set fd [open $hook {WRONLY CREAT EXCL}]
    puts $fd [list source [file normalize $support]]
    puts $fd [list partition_bulk_hook $p [file join $out core_ooc_constraints core_aclk_ooc.xdc] $directory]
    close $fd
    return $hook
}
proc partition_bulk_hook {p xdc directory} {
    set_param general.maxThreads 1
    partition_check_bytes $p
    set part [dict get $p part];set top robot_rt_bd_core_0
    # NAME is Vivado's session label (e.g. design_1), not the synthesized HDL TOP.
    # Actual 2025.2 synth and DCP readback both expose the read-only TOP property.
    set design [current_design]
    set actual_part [get_property PART $design];set actual_top [get_property TOP $design]
    if {![string equal -nocase $actual_part $part] || $actual_top ne $top} {
        error "Unexpected synthesized core identity: PART=$actual_part TOP=$actual_top NAME=[get_property NAME $design]"
    }
    set expected [dict keys [partition_instances]]
    partition_blackboxes $expected
    partition_instance_ports $p
    puts "PARTITION_PRECHECK TWO_BLACKBOXES"
    set parent [file join $directory core_unfilled.dcp]
    write_checkpoint -noxdef $parent
    close_project
    create_project -in_memory -part $part
    read_checkpoint $parent
    foreach module {player_math limit_gate_pipe} {read_checkpoint [dict get $p modules $module dcp]}
    link_design -top $top -part $part -mode out_of_context
    if {![string equal -nocase [get_property PART [current_design]] $part] ||
        ![string equal -nocase [get_property PART [current_project]] $part]} {error "Bulk-linked core part mismatch"}
    partition_blackboxes {}
    partition_instance_ports $p
    dict for {instance module} [partition_instances] {
        if {![llength [get_cells -hierarchical -filter "NAME =~ $instance/* && IS_PRIMITIVE == 1"]]} {error "Empty partition instance: $instance"}
        puts "PARTITION_FILLED $instance $module"
    }
    puts "PARTITION_PARAMETER_BINDING_PASS V2"
    set timing [partition_core_timing_checks $xdc [file join $directory partition_assembly_timing]]
    report_clocks -file [file join $directory partition_core_clocks.rpt]
    set fd [open [file join $directory partition_assembly_proof.tcl] {WRONLY CREAT EXCL}]
    puts $fd [list set partition_assembly_proof [dict create part $part clock_hz [dict get $p clock_hz] clock_period_ns 10.0 \
        fabric_register_count [dict get $timing fabric_register_count] \
        all_registers_capability_count [dict get $timing all_registers_capability_count] \
        timing_counts [dict get $timing timing_counts] timing_report [dict get $timing timing_report] \
        timing_report_sha256 [dict get $timing timing_report_sha256] instances [partition_instances] blackboxes 0 \
        parameter_contract [partition_parameter_contract $p]]]
    close $fd
    # This is only the isolated core's temporary OOC timing view. Final PS/top
    # constraints and signoff remain unchanged; no temporary primary may leak.
    reset_timing -invalid
    if {[llength [get_clocks]]} {error "Temporary core timing survived reset_timing"}
    partition_check_bytes $p
    puts "PARTITION_TIMING_CLEARED CLOCKS=0"
    puts "PARTITION_ASSEMBLY_PASS NO_BLACKBOXES"
    # Return to the original worker's rename_ref / write-copy DCP / end markers.
}
proc partition_check_log {text} {
    foreach sentinel {"PARTITION_PARAMETER_BINDING_PASS V2" "PARTITION_PRECHECK TWO_BLACKBOXES" "PARTITION_TIMING_CLEARED CLOCKS=0" "PARTITION_ASSEMBLY_PASS NO_BLACKBOXES"} {
        if {[lsearch -exact [split $text \n] $sentinel] < 0} {error "Missing actual partition log sentinel: $sentinel"}
    }
    dict for {instance module} [partition_instances] {
        if {[lsearch -exact [split $text \n] "PARTITION_FILLED $instance $module"] < 0} {error "Partition fill not proven: $instance"}
    }
}

# Explicit transformation of Vivado's public fresh worker Tcl, NOT TCL.POST.
# Only classify complete top-level commands; never eval generated text.
proc partition_commands {text} {
    set chunks {};set buffer {};set offset 0
    foreach line [split $text \n] {
        append buffer $line \n
        if {[regexp {(?:^|[^\\])(?:\\\\)*\\$} $line]} {continue}
        if {[info complete $buffer]} {
            lappend chunks [dict create start $offset end [expr {$offset+[string length $buffer]}] text $buffer]
            incr offset [string length $buffer];set buffer {}
        }
    }
    if {[string trim $buffer] ne ""} {error "Incomplete generated core Tcl"}
    return $chunks
}
proc partition_worker_positions {text hook} {
    set synth {};set hooks {};set publish -1
    foreach entry [partition_commands $text] {
        set command [string trim [dict get $entry text]]
        if {[regexp {^synth_design(?:\s|$)} $command]} {
            if {[string first {;} $command] >= 0 ||
                ![regexp {^synth_design\s+-top\s+robot_rt_bd_core_0(?:\s|$)} $command]} {
                error "Unexpected/multiple-statement core synth command"
            }
            lappend synth [dict get $entry end]
        }
        if {$command eq [list source $hook]} {lappend hooks $entry}
        if {$publish < 0 && [regexp {\m(?:rename_ref|write_checkpoint)\M} $command]} {set publish [dict get $entry start]}
    }
    if {[llength $synth] != 1 || $publish < [lindex $synth 0]} {error "Core synthesis/publication boundary is not unique and ordered"}
    return [dict create synth_end [lindex $synth 0] publish_start $publish hooks $hooks]
}
proc partition_insertion_block {hook} {
    return "# ROBOT_RT_EXPLICIT_GENERATED_SCRIPT_INSERTION_BEGIN\n[list source $hook]\n# ROBOT_RT_EXPLICIT_GENERATED_SCRIPT_INSERTION_END\n"
}
proc partition_write_exact {path text} {
    set fd [open $path {WRONLY CREAT EXCL}];puts -nonewline $fd $text;close $fd
}
proc partition_check_worker {run text plan} {
    set hook [dict get $plan partition_hook]
    if {[get_property STEPS.SYNTH_DESIGN.TCL.POST $run] ne ""} {error "Native core POST must remain empty"}
    set position [partition_worker_positions $text $hook]
    if {[llength [dict get $position hooks]] != 1} {error "Expected one explicit assembly source"}
    set block [partition_insertion_block $hook]
    set start [dict get $position synth_end]
    if {[string range $text $start [expr {$start+[string length $block]-1}]] ne $block ||
        $start+[string length $block] > [dict get $position publish_start]} {
        error "Assembly insertion is not immediately after synthesis and before publication/rename"
    }
    foreach module {player_math limit_gate_pipe} {
        set entry [dict get $plan partition modules $module]
        if {[string first [dict get $entry stub] $text] < 0 || [string first [dict get $entry source] $text] >= 0} {
            error "Core script must read only the $module stub"
        }
    }
}
proc partition_insertion_hash_json {record record_file} {
    return [format { {"original_sha256":"%s","generated_sha256":"%s","patched_sha256":"%s","diff_sha256":"%s","hook_sha256":"%s","runme_sha256":"%s","frozen_manifest_sha256":"%s","record_sha256":"%s"} } \
        [dict get $record original_sha256] [dict get $record generated_sha256] [dict get $record patched_sha256] \
        [dict get $record diff_sha256] [dict get $record hook_sha256] [dict get $record runme_sha256] \
        [dict get $record frozen_manifest_sha256] [partition_sha $record_file]]
}
proc partition_patch_worker {out run plan} {
    set hook [dict get $plan partition_hook]
    if {[get_property STEPS.SYNTH_DESIGN.TCL.POST $run] ne ""} {error "Native core POST must be empty before generation transform"}
    set directory [get_property DIRECTORY $run]
    foreach marker {.vivado.begin.rst .vivado.end.rst .vivado.error.rst} {
        if {[file exists [file join $directory $marker]]} {error "Cannot patch a started/completed worker"}
    }
    set script [file join $directory robot_rt_bd_core_0.tcl]
    set fd [open $script r];set original [read $fd];close $fd
    set position [partition_worker_positions $original $hook]
    if {[llength [dict get $position hooks]] || [string first $hook $original] >= 0} {error "Core script already contains assembly hook"}
    set start [dict get $position synth_end];set block [partition_insertion_block $hook]
    set patched "[string range $original 0 [expr {$start-1}]]$block[string range $original $start end]"
    if {[string replace $patched $start [expr {$start+[string length $block]-1}] {}] ne $original} {
        error "Generated-script transform modified bytes beyond insertion"
    }
    partition_check_worker $run $patched $plan
    set split [file join $out split]
    set before [file join $split core_worker_original.tcl];set after [file join $split core_worker_patched.tcl]
    set diff [file join $split core_worker_insertion.diff]
    set record_file [file join $split partition_insertion_record.tcl]
    set hash_file [file join $split partition_insertion_hashes.json]
    foreach file [list $before $after $diff $record_file $hash_file] {
        if {[file exists $file]} {error "Insertion evidence already exists; require fresh worker/output: $file"}
    }
    partition_write_exact $before $original;partition_write_exact $after $patched
    set rc [catch {exec diff -u -- $before $after} delta options]
    if {$rc != 1 || ![dict exists $options -errorcode] || [lindex [dict get $options -errorcode] 2] != 1} {
        error "Expected nonempty generated-script diff with exit1"
    }
    partition_write_exact $diff "$delta\n"
    set fd [open $script w];puts -nonewline $fd $patched;close $fd
    set record [dict create schema robot_rt.generated_insertion.v1 script $script original $before patched $after \
        diff $diff hook $hook runme [file join $directory runme.sh] \
        original_sha256 [partition_sha $before] generated_sha256 [partition_sha $script] \
        patched_sha256 [partition_sha $after] diff_sha256 [partition_sha $diff] \
        hook_sha256 [partition_sha $hook] runme_sha256 [partition_sha [file join $directory runme.sh]] \
        frozen_manifest_sha256 [partition_sha [file join $split frozen_inputs.sha256]]]
    partition_write_exact $record_file "[list set partition_insertion_record $record]\n"
    set fd [open $hash_file {WRONLY CREAT EXCL}]
    puts $fd [partition_insertion_hash_json $record $record_file]
    close $fd
    partition_verify_insertion $out $run $plan
    puts "SPLIT_PARTITION_INSERTED [partition_sha $hash_file]"
}
proc partition_verify_insertion {out run plan} {
    set split [file join $out split]
    set record [partition_literal [file join $split partition_insertion_record.tcl] partition_insertion_record]
    if {[dict get $record schema] ne "robot_rt.generated_insertion.v1" ||
        [dict get $record script] ne [file join [get_property DIRECTORY $run] robot_rt_bd_core_0.tcl] ||
        [dict get $record hook] ne [dict get $plan partition_hook]} {error "Insertion identity mismatch"}
    foreach key {original patched diff hook runme} {
        if {[partition_sha [dict get $record $key]] ne [dict get $record ${key}_sha256]} {error "Insertion evidence changed: $key"}
    }
    if {[partition_sha [dict get $record script]] ne [dict get $record generated_sha256] ||
        [dict get $record generated_sha256] ne [dict get $record patched_sha256] ||
        [partition_sha [file join $split frozen_inputs.sha256]] ne [dict get $record frozen_manifest_sha256]} {
        error "Generated script/frozen source manifest changed since insertion"
    }
    set fd [open [file join $split partition_insertion_hashes.json] r];set hashes [read $fd];close $fd
    if {[string trim $hashes] ne [string trim [partition_insertion_hash_json $record [file join $split partition_insertion_record.tcl]]]} {
        error "Controller insertion JSON and literal records disagree"
    }
    set fd [open [dict get $record script] r];set text [read $fd];close $fd
    partition_check_worker $run $text $plan
    set position [partition_worker_positions $text [dict get $plan partition_hook]]
    set start [dict get $position synth_end];set block [partition_insertion_block [dict get $plan partition_hook]]
    set fd [open [dict get $record original] r];set original [read $fd];close $fd
    if {[string replace $text $start [expr {$start+[string length $block]-1}] {}] ne $original} {error "Insertion no longer sole script change"}
}
proc partition_verify_published {out run plan} {
    set run_dcp [file join [get_property DIRECTORY $run] robot_rt_bd_core_0.dcp]
    set published [file join $out robot_rt.gen sources_1 bd robot_rt_bd ip robot_rt_bd_core_0 robot_rt_bd_core_0.dcp]
    foreach file [list $run_dcp $published] {
        if {![file isfile $file] || [file size $file] == 0} {error "Core publication missing/empty: $file"}
    }
    set dcp_sha [partition_sha $run_dcp]
    if {$dcp_sha ne [partition_sha $published]} {error "Run core DCP differs from generated published DCP"}
    # Inspect the actual managed run, not the hook's log or temporary design.
    open_run [get_property NAME $run]
    set p [dict get $plan partition]
    if {![string equal -nocase [get_property PART [current_design]] [dict get $p part]]} {error "Published core part mismatch"}
    partition_blackboxes {};partition_instance_ports $p
    # Local core inspection only; final PS-derived-clock signoff is untouched.
    set timing [partition_core_timing_checks [file join $out core_ooc_constraints core_aclk_ooc.xdc] \
        [file join $out split partition_readback_timing] $run_dcp]
    set counts {};set fabric_names [dict get $timing fabric_register_names]
    dict for {instance module} [partition_instances] {
        set owned 0
        foreach name $fabric_names {if {[string match ${instance}/* $name]} {incr owned}}
        if {!$owned} {error "Published core lacks real fabric sequential child: $instance"}
        dict set counts $instance $owned
    }
    partition_check_bytes $p
    save_state $out core_partition_readback.tcl [dict create schema robot_rt.partition_readback.v1 \
        run_dcp $run_dcp published_dcp $published dcp_sha256 $dcp_sha blackboxes 0 \
        instance_fabric_register_counts $counts clock_name core_aclk_ooc clock_period_ns 10.0 \
        fabric_register_count [dict get $timing fabric_register_count] \
        all_registers_capability_count [dict get $timing all_registers_capability_count] \
        timing_counts [dict get $timing timing_counts] timing_report [dict get $timing timing_report] \
        timing_report_sha256 [dict get $timing timing_report_sha256] \
        timing_binding [dict get $timing timing_binding] timing_binding_sha256 [dict get $timing timing_binding_sha256] \
        insertion_record_sha256 [partition_sha [file join $out split partition_insertion_record.tcl]] \
        parameter_contract [partition_parameter_contract $p]]
    puts "PARTITION_PUBLISHED_READBACK_PASS TWO_REAL_CHILDREN RUN_AND_PUBLISHED_SHA_EQUAL"
}

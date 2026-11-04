# External scheduling phases. Fresh output required; no board commands.
# --phase prepare|scripts|verify|signoff|bit-scripts|verify-bit|export
# --repo REPO --out NEW_OUT [--library FILE] [--run NAME] [--vivado-root DIR]
proc frozen {out} {
    set f [file join $out split frozen_inputs.sha256]
    if {![file isfile $f]} {error "Missing frozen input manifest"}
    exec sha256sum -c $f
}
proc load_plan {out} {
    source [file join $out split plan.tcl]
    return $split_plan
}
proc current {name} {
    set r [::robot_rt_bd::one [get_runs -quiet $name] "run $name"]
    if {![run_is_current $r]} {error "Run not current Complete/100%/NEEDS_REFRESH=0: $name [get_property STATUS $r]"}
    return $r
}
proc source_set {run} {
    foreach k {SRCSET SOURCE_SET} {
        if {[lsearch -exact [list_property $run] $k]>=0} {return [list $k [get_property $k $run]]}
    }
    error "No readable source-set property for $run"
}
proc same_contract {plan name} {
    set r [get_runs $name];set c [dict get $plan contracts $name]
    if {[source_set $r] ne [dict get $c source_set] || [get_property CONSTRSET $r] ne [dict get $c constrset]} {error "Run source/constraint set changed: $name"}
    if {[dict exists $c core_flatten] &&
        [get_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY $r] ne [dict get $c core_flatten]} {
        error "Core synthesis hierarchy policy changed: $name"
    }
    if {[dict exists $c core_directive] &&
        [get_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE $r] ne [dict get $c core_directive]} {
        error "Core synthesis directive changed: $name"
    }
    if {[dict exists $c native_post_hook] && [get_property STEPS.SYNTH_DESIGN.TCL.POST $r] ne [dict get $c native_post_hook]} {error "Native core POST contract changed: $name"}
    if {[dict exists $c gate_stable_job_inputs] &&
        (![dict exists $plan partition] || [dict get $c gate_stable_job_inputs] ne [dict get $plan partition expected_gate_stable_job_inputs])} {
        error "Frozen parent/partition gate semantics differ"
    }
    if {[dict exists $c capacity_gate]} {capacity_check $r [dict get $c capacity_gate]}
    return $r
}
# Fresh-project engineering capacity hook. This never changes route/STA/bit bodies.
proc capacity_check {run contract} {
    set hook [dict get $contract hook_path]
    if {![file isfile $hook] || [file type $hook] ne "file" ||
        [lindex [exec sha256sum -- $hook] 0] ne [dict get $contract hook_sha256]} {error "Capacity hook source changed"}
    if {[get_property STEPS.PHYS_OPT_DESIGN.TCL.POST $run] ne $hook} {error "Capacity hook run property changed"}
    set enabled [get_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED $run]
    if {![string is boolean -strict $enabled] || !$enabled || $enabled ne [dict get $contract physopt_enabled]} {error "Capacity hook step disabled or changed"}
}
proc capacity_install {run hook} {
    set hook [file normalize $hook]
    if {![file isfile $hook] || [file type $hook] ne "file"} {error "Fresh capacity hook source missing"}
    if {[get_property STEPS.PHYS_OPT_DESIGN.TCL.POST $run] ne ""} {error "Refusing to replace an existing physopt POST hook"}
    set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true $run
    set_property STEPS.PHYS_OPT_DESIGN.TCL.POST $hook $run
    set contract [dict create hook_path $hook hook_sha256 [lindex [exec sha256sum -- $hook] 0] physopt_enabled [get_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED $run]]
    capacity_check $run $contract
    return $contract
}
# Saved state and bounded generated-input snapshots; defining these starts no run.
proc read_literal_set {path variable} {
    set fd [open $path r];set text [string trim [read $fd]];close $fd
    # Parse the one generated Tcl list; never source/evaluate a saved state file.
    if {[catch {llength $text} n] || $n != 3 ||
        [lindex $text 0] ne "set" || [lindex $text 1] ne $variable} {
        error "Expected one literal set $variable in $path"
    }
    return [lindex $text 2]
}
proc equal_state_dict {left right} {
    if {[catch {dict size $left} n] || [catch {dict size $right} m] || $n != $m} {return 0}
    foreach key [dict keys $left] {
        if {![dict exists $right $key] || [dict get $left $key] ne [dict get $right $key]} {return 0}
    }
    return 1
}
proc save_state {out name value} {
    set path [file join $out split $name]
    if {[file exists $path]} {
        if {![file isfile $path]} {error "State path is not a file: $path"}
        set existing [read_literal_set $path split_state]
        if {$existing ne $value && ![equal_state_dict $existing $value]} {
            error "Existing state differs; refusing overwrite: $path"
        }
        return
    }
    set fd [open $path {WRONLY CREAT EXCL}]
    puts $fd [list set split_state $value];close $fd
}
proc snapshot_properties {object keys} {
    set available [list_property $object];set result {}
    foreach key $keys {
        set index [lsearch -nocase -exact $available $key]
        if {$index < 0} {
            lappend result [list $key absent]
        } else {
            lappend result [list $key present [get_property [lindex $available $index] $object]]
        }
    }
    return $result
}
proc snapshot_runtime_file {object} {
    set path [file normalize [get_property NAME $object]]
    if {[string tolower [file extension $path]] in {.dcp .rpt .log .jou .pb .wdb .wcfg}} {return 1}
    # Vivado registers future OOC stub/simulation outputs before they exist.
    # Exclude only these generated, non-build-role products, even after they
    # appear. Real synthesis wrappers and missing build inputs stay checked.
    set props [list_property $object]
    if {[lsearch -exact $props IS_GENERATED] >= 0 && [get_property IS_GENERATED $object] &&
        [regexp -nocase {_(sim_netlist|stub)\.(v|vhdl)$} $path]} {
        set roles [get_property USED_IN $object]
        set build_role [expr {[lsearch -exact $roles synthesis] >= 0 ||
                              [lsearch -exact $roles implementation] >= 0}]
        foreach key {USED_IN_SYNTHESIS USED_IN_IMPLEMENTATION} {
            if {[lsearch -exact $props $key] >= 0 && [get_property $key $object]} {set build_role 1}
        }
        if {!$build_role && ([lsearch -exact $roles simulation] >= 0 ||
                            [lsearch -exact $roles synth_blackbox_stub] >= 0)} {return 1}
    }
    if {[lsearch -nocase -exact [list_property $object] FILE_TYPE] >= 0} {
        if {[regexp -nocase {(^|[[:space:]])(checkpoint|report|log)([[:space:]]|$)} [get_property FILE_TYPE $object]]} {return 1}
    }
    return 0
}
proc snapshot_file {object} {
    set path [file normalize [get_property NAME $object]]
    if {![file isfile $path] || ![file readable $path]} {error "Snapshot input is not a readable file: $path"}
    set keys {FILE_TYPE LIBRARY USED_IN USED_IN_SYNTHESIS USED_IN_IMPLEMENTATION
        USED_IN_SIMULATION IS_ENABLED IS_GLOBAL_INCLUDE PROCESSING_ORDER
        SCOPED_TO_REF SCOPED_TO_CELLS SCOPED_TO_INSTANCE}
    return [dict create path $path sha256 [lindex [exec sha256sum -- $path] 0] properties [snapshot_properties $object $keys]]
}
proc project_input_snapshot {plan} {
    set runs {};set set_names {}
    foreach name [lsort [dict keys [dict get $plan contracts]]] {
        set r [::robot_rt_bd::one [get_runs -quiet $name] "snapshot run $name"]
        set source [source_set $r];set constraint [get_property CONSTRSET $r]
        if {[lindex $source 1] eq "" || $constraint eq ""} {error "Empty run source/constraint set: $name"}
        lappend runs [list $name [dict create source_set $source constrset $constraint]]
        lappend set_names [lindex $source 1] $constraint
    }
    set sets {}
    foreach name [lsort -unique $set_names] {
        set fs [::robot_rt_bd::one [get_filesets -quiet $name] "snapshot fileset $name"]
        set members {}
        # Preserve Vivado's exposed member order after excluding runtime outputs;
        # never sort away a potentially meaningful constraint/source reordering.
        foreach f [get_files -quiet -of_objects $fs] {
            if {![snapshot_runtime_file $f]} {lappend members [snapshot_file $f]}
        }
        set keys {FILESET_TYPE TOP TOP_LIB GENERIC VERILOG_DEFINE INCLUDE_DIRS}
        lappend sets [list $name [dict create properties [snapshot_properties $fs $keys] members $members]]
    }
    # Generated composite inputs may not be direct fileset members. Pin these
    # visible inputs too, without freezing run-status files, reports or new DCPs.
    set generated {};set bd_count 0;set ip_count 0;set wrapper_count 0
    foreach f [get_files -all -quiet] {
        if {[snapshot_runtime_file $f]} {continue}
        set path [file normalize [get_property NAME $f]]
        set ext [string tolower [file extension $path]]
        set is_bd [expr {$ext eq ".bd"}]
        set is_ip [expr {$ext in {.xci .xcix}}]
        set is_wrapper [expr {$ext in {.v .sv .vhd .vhdl} &&
            ([regexp -nocase {_wrapper\.(?:v|sv|vhd|vhdl)$} $path] ||
             [string first /synth/ [string map {\\ /} $path]] >= 0)}]
        if {$is_bd || $is_ip || $is_wrapper} {
            if {[dict exists $generated $path]} {error "Duplicate generated input path: $path"}
            dict set generated $path [snapshot_file $f]
            incr bd_count $is_bd;incr ip_count $is_ip;incr wrapper_count $is_wrapper
        }
    }
    if {!$bd_count || !$ip_count || !$wrapper_count} {
        error "Generated input enumeration incomplete: BD=$bd_count XCI/XCIX=$ip_count wrapper/synth=$wrapper_count"
    }
    set generated_ordered {}
    foreach path [lsort [dict keys $generated]] {lappend generated_ordered [dict get $generated $path]}
    return [dict create schema robot_rt.project_input_snapshot.v1 runs $runs filesets $sets generated $generated_ordered]
}
proc write_project_snapshot {out plan} {
    set value [project_input_snapshot $plan]
    set fd [open [file join $out split project_inputs.tcl] {WRONLY CREAT EXCL}]
    puts $fd [list set split_project_snapshot $value];close $fd
}
proc check_project_snapshot {out plan} {
    set path [file join $out split project_inputs.tcl]
    if {![file isfile $path]} {error "Project input snapshot missing; prepare a fresh output with the reviewed driver"}
    set expected [read_literal_set $path split_project_snapshot]
    set actual [project_input_snapshot $plan]
    if {$actual ne $expected} {error "Project input snapshot changed (membership/order/properties/content); refusing phase"}
}

proc checked_route {out} {
    source [file join $out split route_signoff.tcl]
    set actual [lindex [exec sha256sum [dict get $split_state routed_dcp]] 0]
    if {$actual ne [dict get $split_state routed_sha256]} {error "Routed DCP changed since signoff"}
    set mf [lindex [exec sha256sum [file join $out split frozen_inputs.sha256]] 0]
    if {$mf ne [dict get $split_state frozen_manifest_sha256]} {error "Source/constraint manifest changed since route signoff"}
    return $split_state
}
proc verify_artifacts {run} {
    set d [get_property DIRECTORY $run]
    if {![file isfile [file join $d .vivado.end.rst]] || [llength [glob -nocomplain -directory $d .*.error.rst]]} {error "Missing end or present error marker"}
    if {![llength [glob -nocomplain -directory $d *.dcp]]} {error "Completed run lacks checkpoint"}
    set fd [open [file join $d runme.log]];set log [read $fd];close $fd
    if {[regexp -line {^ERROR:} $log]} {error "Error in external run log"}
}
# Require an explicit literal directive on every actual synth command, not a
# comment or unrelated property line. Vendor Tcl can use its canonical enum or
# the synth_design command's documented lowercase spelling.
proc assert_runtime_core_directive {text} {
    regsub -all {\\\r?\n[ \t]*} $text { } logical
    set commands [regexp -all -inline -line {^[ \t]*(?:Command:[ \t]*)?synth_design[ \t]+-[A-Za-z_][^\r\n]*} $logical]
    if {![llength $commands]} {error "No actual synth_design command for explicit core directive"}
    foreach command $commands {
        set options [regexp -all -inline -- {(^|[ \t])-directive([ \t]|$)} $command]
        if {[llength $options]!=3 ||
            ![regexp -- {(^|[ \t])-directive[ \t]+(?:\{(?:RuntimeOptimized|runtimeoptimized)\}|RuntimeOptimized|runtimeoptimized)([ \t]|$)} $command]} {
            error "Actual core synth command lacks the unique explicit RuntimeOptimized directive"
        }
    }
}
proc assert_serial_synthesis_script {run {explicit_directive 0}} {
    set found 0
    foreach path [glob -nocomplain -directory [get_property DIRECTORY $run] *.tcl] {
        set fd [open $path r];set text [read $fd];close $fd
        if {[string first "synth_design -top" $text] < 0} {continue}
        incr found
        if {$explicit_directive} {assert_runtime_core_directive $text}
        if {![regexp -line {^\s*set_param\s+general\.maxThreads\s+1\s*$} $text]} {
            error "Generated synthesis script lacks explicit maxThreads=1: $path"
        }
        if {[get_property NAME $run] eq "robot_rt_bd_core_0_synth_1" &&
            [get_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY $run] eq "none" &&
            ![regexp -- {-flatten_hierarchy\s+(?:\{none\}|none)(?:\s|$)} $text]} {
            error "Generated core synthesis script lacks frozen -flatten_hierarchy none: $path"
        }
    }
    if {$found != 1} {error "Cannot identify exactly one serial synthesis script"}
}
if {[catch {
    # Global tool parameters are not saved in the project. Every fresh
    # controller must set this before generating an external worker script.
    set_param general.maxThreads 1
    if {[get_param general.maxThreads] != 1} {error "Cannot limit controller threads"}
    set o [dict create phase "" repo "" out "" library "" run "" core_flatten rebuilt core_directive "" partition_dir "" partition_gate_stable_job_inputs "" vivado_root /home/aup/Xilinx/2025.2/Vivado]
    if {[llength $::argv]%2} {error "All arguments are named value pairs"}
    foreach {k v} $::argv {
        set k [string map {- _} [string trimleft $k -]]
        if {![dict exists $o $k]} {error "Unknown option $k"};dict set o $k $v
    }
    set repo [file normalize [dict get $o repo]];set out [file normalize [dict get $o out]]
    set phase [dict get $o phase];set name [dict get $o run]
    if {[dict get $o repo] eq "" || [dict get $o out] eq ""} {error "--repo and --out are required"}
    set lib [dict get $o library]
    if {$lib eq ""} {set lib [file join $repo tcl build.tcl]}
    # --help also makes source of an unpatched old build safe; then require API.
    set ::argv [list --help];set ::robot_rt_build_library 1
    source $lib
    unset ::robot_rt_build_library
    if {![info exists ::robot_rt_build_api_version] || $::robot_rt_build_api_version!=1} {error "Build library API1 not present; review/apply minimal patch first"}
    set ::robot_rt_bd::root $repo
    if {$phase ne "prepare" && [dict get $o core_directive] ne ""} {error "--core-directive is prepare-only"}
    if {[dict get $o core_directive] ni {"" runtimeoptimized}} {error "Unsupported --core-directive; expected runtimeoptimized"}
    if {$phase ne "prepare" && [dict get $o partition_dir] ne ""} {error "--partition-dir is prepare-only"}
    if {[dict get $o partition_gate_stable_job_inputs] ni {"" 0 1}} {error "Invalid independent parent gate mode"}
    if {[dict get $o partition_gate_stable_job_inputs] ne "" && ($phase ne "prepare" || [dict get $o partition_dir] eq "")} {
        error "--partition-gate-stable-job-inputs requires prepare with --partition-dir"
    }
    if {$phase eq "prepare"} {
        # This legacy split driver consumes the explicit99,999,001Hz manifests.
        # The50MHz build uses build_50mhz.py/tcl and its newly compiled children.
        set opts [::robot_rt_bd::inputs [::robot_rt_bd::options --out $out --vivado-root [dict get $o vivado_root] --clock-hz 100000000]]
        set partition {};set partition_extra {};set partition_hook ""
        set partition_support [file join [file dirname [file normalize [info script]]] partition_support.tcl]
        if {[dict get $o partition_dir] ne ""} {
            if {[dict get $o core_flatten] ne "none"} {error "Partition preparation requires --core-flatten none"}
            source $partition_support
            set gate_mode [dict get $o partition_gate_stable_job_inputs]
            if {$gate_mode eq ""} {set gate_mode 0}
            set partition [partition_load [dict get $o partition_dir] $repo $gate_mode]
            dict set opts sources [partition_sources [dict get $opts sources] $partition]
            set partition_extra [concat [partition_files $partition] [list $partition_support]]
        }
        ::robot_rt_bd::create $opts
        set deps [create_ip_run [get_files */robot_rt_bd.bd]]
        set core [::robot_rt_bd::one [get_runs -quiet robot_rt_bd_core_0_synth_1] "core synthesis profile"]
        set hierarchy [dict get $o core_flatten]
        set directive_explicit [expr {[dict get $o core_directive] ne ""}]
        if {$directive_explicit} {
            set_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE RuntimeOptimized $core
            if {[get_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE $core] ne "RuntimeOptimized"} {
                error "Core RuntimeOptimized directive was not accepted canonically"
            }
        }
        # Absence preserves the original setting; its actual value is still frozen.
        set core_directive [get_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE $core]
        if {$core_directive eq ""} {error "Core directive readback is empty"}
        if {$hierarchy ni {rebuilt none}} {error "Invalid core hierarchy policy"}
        set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY $hierarchy $core
        if {[get_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY $core] ne $hierarchy} {
            error "Core synthesis hierarchy policy was not accepted"
        }
        if {$partition ne ""} {
            if {$::robot_rt_bd::actual_clock_hz != [dict get $partition clock_hz]} {error "Partition and actual PS clock mismatch"}
            set partition_hook [partition_write_hook $out $partition $partition_support]
            if {[get_property STEPS.SYNTH_DESIGN.TCL.POST $core] ne ""} {error "Partition profile requires empty native core POST"}
            lappend partition_extra $partition_hook
        }
        prepare_core_ooc_constraints $out $::robot_rt_bd::actual_clock_hz
        if {$partition ne "" && [get_property STEPS.SYNTH_DESIGN.TCL.POST $core] ne ""} {error "Core preflight changed empty native POST contract"}
        set ips {}
        foreach r [get_runs -filter {IS_SYNTHESIS}] {
            if {[get_property NAME $r] ne "synth_1"} {lappend ips [get_property NAME $r]}
        }
        set ips [lsort $ips]
        foreach d $deps {if {[lsearch -exact $ips [get_property NAME [get_runs $d]]]<0} {error "Missing materialized dependency"}}
        set capacity_contract [capacity_install [get_runs impl_1] [file join [file dirname [file normalize [info script]]] capacity_gate.tcl]]
        set contracts [dict create]
        foreach n [concat $ips {synth_1 impl_1}] {
            set r [get_runs $n];dict set contracts $n [dict create source_set [source_set $r] constrset [get_property CONSTRSET $r]]
            if {$n eq "impl_1"} {dict set contracts $n capacity_gate $capacity_contract}
            if {$n eq "robot_rt_bd_core_0_synth_1"} {
                dict set contracts $n core_flatten $hierarchy
                dict set contracts $n core_directive $core_directive
                dict set contracts $n core_directive_explicit $directive_explicit
            }
        }
        if {$partition ne ""} {
            dict set contracts robot_rt_bd_core_0_synth_1 native_post_hook {}
            dict set contracts robot_rt_bd_core_0_synth_1 gate_stable_job_inputs [dict get $partition expected_gate_stable_job_inputs]
        }
        file mkdir [file join $out split]
        set plan [dict create project [file join $out robot_rt.xpr] ips $ips contracts $contracts clock_hz $::robot_rt_bd::actual_clock_hz vivado_root [dict get $o vivado_root]]
        if {$partition ne ""} {dict set plan partition $partition;dict set plan partition_hook $partition_hook}
        set fd [open [file join $out split plan.tcl] {WRONLY CREAT EXCL}]
        puts $fd [list set split_plan $plan];close $fd
        write_project_snapshot $out $plan
        set inputs [concat [dict get $opts sources] [list $lib [file join [file dirname $lib] create_bd.tcl] [file join $out core_ooc_constraints core_aclk_ooc.xdc] [file join $repo constraints robot_rt.xdc] [file normalize [info script]] [file join $out split plan.tcl] [file join $out split project_inputs.tcl] [file join [file dirname [info script]] split_synthesis.py]]]
        set inputs [concat $inputs $partition_extra [list [dict get $capacity_contract hook_path]]]
        set fd [open [file join $out split frozen_inputs.sha256] {WRONLY CREAT EXCL}]
        foreach f $inputs {puts $fd [exec sha256sum $f]};close $fd
        puts "SPLIT_PREPARED $ips"
    } else {
        frozen $out;set plan [load_plan $out]
        if {![dict exists $plan contracts impl_1 capacity_gate]} {error "Fresh capacity-hook preparation required; refusing an old project/plan"}
        if {[dict exists $plan partition]} {source [file join [file dirname [file normalize [info script]]] partition_support.tcl];partition_check_bytes [dict get $plan partition]}
        open_project [dict get $plan project]
        # Every controller checks the frozen core settings, even while handling
        # a different IP run, implementation, signoff or export.
        same_contract $plan robot_rt_bd_core_0_synth_1
        same_contract $plan impl_1
        check_project_snapshot $out $plan
        require_no_active_runs
        if {$phase in {scripts verify}} {
            if {[lsearch -exact [concat [dict get $plan ips] {synth_1 impl_1}] $name]<0} {error "Run is not in frozen plan"}
            set r [same_contract $plan $name]
        }
        switch -- $phase {
            scripts {
                if {[run_is_current $r]} {puts "SPLIT_SKIP_CURRENT $name"} else {
                    if {[get_property STATUS $r] ne "Not started" || [get_property PROGRESS $r] ne "0%"} {error "Fresh unstarted run required; do not reset cached/stale results"}
                    if {$name eq "synth_1" || $name eq "impl_1"} {foreach dep [dict get $plan ips] {current $dep}}
                    if {$name eq "impl_1"} {
                        current synth_1
                        set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED false $r
                        launch_runs $r -to_step route_design -scripts_only
                    } else {
                        if {$name ne "synth_1"} {
                            # Matches the resident serial schedule. A newly added
                            # nested dependency still needs a reviewed DAG/order.
                            foreach dep [dict get $plan ips] {if {$dep eq $name} break;current $dep}
                        }
                        launch_runs $r -scripts_only
                    }
                    if {[get_property IS_SYNTHESIS $r]} {
                        set explicit_directive 0
                        if {$name eq "robot_rt_bd_core_0_synth_1" &&
                            [dict exists $plan contracts $name core_directive_explicit]} {
                            set explicit_directive [dict get $plan contracts $name core_directive_explicit]
                        }
                        assert_serial_synthesis_script $r $explicit_directive
                        if {[dict exists $plan partition] && $name eq "robot_rt_bd_core_0_synth_1"} {
                            partition_patch_worker $out $r $plan
                        }
                    }
                    puts "SPLIT_SCRIPT_READY $name [get_property DIRECTORY $r]"
                }
            }
            verify {
                set r [current $name];verify_artifacts $r;same_contract $plan $name
                if {$name eq "robot_rt_bd_core_0_synth_1"} {
                    set d [get_property DIRECTORY $r];set fd [open [file join $d runme.log]];set log [read $fd];close $fd
                    if {[dict exists $plan contracts $name core_directive_explicit] &&
                        [dict get $plan contracts $name core_directive_explicit]} {
                        assert_runtime_core_directive $log
                    }
                    set xdc [file join $out core_ooc_constraints core_aclk_ooc.xdc]
                    if {[string first "Parsing XDC File \[$xdc\]" $log]<0} {error "Actual core XDC parse not proven"}
                    if {[dict exists $plan contracts $name core_flatten] &&
                        [dict get $plan contracts $name core_flatten] eq "none" &&
                        ![regexp -- {-flatten_hierarchy\s+(?:\{none\}|none)(?:\s|$)} $log]} {
                        error "Actual core log lacks frozen -flatten_hierarchy none"
                    }
                }
                if {[dict exists $plan partition] && $name eq "robot_rt_bd_core_0_synth_1"} {
                    partition_check_log $log
                    partition_verify_insertion $out $r $plan
                    partition_verify_published $out $r $plan
                    check_project_snapshot $out $plan
                }
                if {$name eq "synth_1"} {build_report_synthesis $out}
                save_state $out ${name}.verified.tcl [dict create run $name status [get_property STATUS $r] source_set [source_set $r] constrset [get_property CONSTRSET $r]]
                puts "SPLIT_VERIFIED $name"
            }
            signoff {
                current impl_1;same_contract $plan impl_1
                build_check_route $out [dict get $plan clock_hz]
                set dcp [file join [get_property DIRECTORY [get_runs impl_1]] robot_rt_bd_wrapper_routed.dcp]
                if {![file isfile $dcp]} {error "Routed checkpoint missing"}
                set input_fd [open [file join $out split frozen_inputs.sha256]];set input_text [read $input_fd];close $input_fd
                save_state $out route_signoff.tcl [dict create routed_dcp $dcp routed_sha256 [lindex [exec sha256sum $dcp] 0] frozen_manifest_sha256 [lindex [exec sha256sum [file join $out split frozen_inputs.sha256]] 0] signoff central_build_check_route]
                puts "SPLIT_SIGNOFF_PASSED"
            }
            bit-scripts {
                current impl_1;same_contract $plan impl_1;checked_route $out
                launch_runs impl_1 -next_step -scripts_only
                puts "SPLIT_BIT_SCRIPT_READY [get_property DIRECTORY [get_runs impl_1]]"
            }
            verify-bit {
                set r [current impl_1];same_contract $plan impl_1;checked_route $out;verify_artifacts $r
                if {![string match {*write_bitstream*Complete*} [get_property STATUS $r]] || ![file isfile [file join [get_property DIRECTORY $r] robot_rt_bd_wrapper.bit]]} {error "External bitstream step not proven complete"}
                save_state $out bit_verified.tcl [dict create run_status [get_property STATUS $r]]
            }
            export {
                if {![file isfile [file join $out split bit_verified.tcl]]} {error "External bitstream verification missing"}
                current impl_1;same_contract $plan impl_1;checked_route $out
                build_export_checked_bitstream $out [dict get $plan vivado_root]
            }
            default {error "Unknown phase $phase"}
        }
        frozen $out
    }
    close_project
    puts "PASS SPLIT_PHASE_DONE $phase; controller exits before external runme"
} why options]} {puts stderr "SPLIT_ERROR $why";puts stderr [dict get $options -errorinfo];exit 1}
exit

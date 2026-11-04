# Run manually inside Vivado; no hardware manager or device programming.
set ::robot_rt_bd_library 1
source [file join [file dirname [file normalize [info script]]] create_bd.tcl]
unset ::robot_rt_bd_library
proc build_usage {} {
    puts {Usage: vivado -mode batch -source tcl/build.tcl -tclargs --out NEW_DIRECTORY [--clock-hz 50000000|100000000] [--jobs 1|2] [--vivado-root DIR]
       tclsh tcl/build.tcl --check-inputs | --help
Creates the BD, synthesizes, routes, checks setup/hold and DRC, then writes .bit/.bit.bin.
Default/recommended --jobs 1 explicitly completes IP synthesis runs one at a time before top synthesis.
Timing failures and missing constraints stop before producing deployment bitstreams.}
}
proc run_finished {name} {
    wait_on_run $name
    set run [get_runs $name]
    if {[get_property PROGRESS $run] ne "100%" || [string match -nocase {*error*} [get_property STATUS $run]]} {
        error "Run $name failed: [get_property STATUS $run]"
    }
}
proc run_is_current {run} {
    if {[lsearch -exact [list_property $run] NEEDS_REFRESH] < 0} {
        error "Cannot verify cache freshness for run $run (NEEDS_REFRESH unavailable)"
    }
    return [expr {[get_property PROGRESS $run] eq "100%" &&
        [string match {*Complete*} [get_property STATUS $run]] &&
        ![get_property NEEDS_REFRESH $run]}]
}
proc require_no_active_runs {} {
    foreach run [get_runs] {
        set status [get_property STATUS $run]
        if {[regexp -nocase {running|queued} $status]} {
            error "Serial scheduling found an already-active run $run: $status"
        }
    }
}
proc prepare_core_ooc_constraints {out actual_clock_hz {requested_clock_hz 100000000}} {
    # AMD create_run/save_constraints_as document the run-local CONSTRSET;
    # read_xdc documents -mode out_of_context. Merely tagging a top XDC does
    # not prove a module-reference OOC run will load it. Check the real script.
    if {![string is integer -strict $actual_clock_hz] || $actual_clock_hz < 1 || $actual_clock_hz > 2147483647} {
        error "Invalid actual clock metadata for core OOC constraints: $actual_clock_hz"
    }
    require_no_active_runs
    set run [::robot_rt_bd::one [get_runs -quiet robot_rt_bd_core_0_synth_1] "core OOC synthesis run"]
    if {![get_property IS_SYNTHESIS $run]} { error "Core OOC run is not synthesis" }
    if {[get_property STATUS $run] ne "Not started" || [get_property PROGRESS $run] ne "0%"} {
        error "Core OOC constraints must be prepared before synthesis/cache reuse; refusing to reset a nonfresh run"
    }
    set top_sets [dict create]
    foreach top_name {synth_1 impl_1} {
        set top [::robot_rt_bd::one [get_runs -quiet $top_name] "top run $top_name"]
        dict set top_sets $top_name [get_property CONSTRSET $top]
    }
    set directory [file join [file normalize $out] core_ooc_constraints]
    set fs core_aclk_ooc
    if {[file exists $directory] || [llength [get_filesets -quiet $fs]]} {
        error "Core OOC constraint evidence already exists; choose a fresh project/output"
    }
    file mkdir $directory
    set xdc [file join $directory core_aclk_ooc.xdc]
    # Match the selected PS clock: 20ns for50MHz, 10ns for100MHz.
    # This is only an OOC input-port target, not another top-level PS clock.
    set period [format %.15g [expr {min(1.0e9 / $requested_clock_hz, 1.0e9 / $actual_clock_hz)}]]
    set handle [open $xdc {WRONLY CREAT EXCL}]
    puts $handle "# Core OOC only; actual CLOCK_HZ=$actual_clock_hz, target period ${period}ns."
    puts $handle [format {create_clock -name core_aclk_ooc -period %s [get_ports aclk]} $period]
    close $handle
    create_fileset -constrset $fs
    add_files -fileset $fs -norecurse $xdc
    set file [::robot_rt_bd::one [get_files -of_objects [get_filesets $fs]] "core OOC XDC"]
    set_property USED_IN {synthesis out_of_context} $file
    set_property USED_IN_IMPLEMENTATION false $file
    set_property CONSTRSET $fs $run
    if {[get_property CONSTRSET $run] ne $fs || [get_property USED_IN_IMPLEMENTATION $file]} {
        error "Core OOC constraint set/scope was not accepted"
    }
    foreach top_name {synth_1 impl_1} {
        if {[get_property CONSTRSET [get_runs $top_name]] ne [dict get $top_sets $top_name]} {
            error "Top constraint set changed while preparing core OOC"
        }
        foreach top_file [get_files -of_objects [get_filesets [dict get $top_sets $top_name]]] {
            if {[file normalize [get_property NAME $top_file]] eq $xdc} {
                error "OOC primary clock XDC leaked into a top constraint set"
            }
        }
    }
    # This only materializes Tcl/runme files. No synthesis process is launched.
    launch_runs $run -scripts_only
    set script [file join [get_property DIRECTORY $run] robot_rt_bd_core_0.tcl]
    if {![file readable $script]} { error "Core OOC synthesis script was not generated: $script" }
    set handle [open $script r];set content [read $handle];close $handle
    set read_found 0;set ooc_mode_found 0;set excluded_from_implementation 0;set block ""
    foreach line [split $content "\n"] {
        append block $line "\n"
        if {![info complete $block]} { continue }
        if {[regexp {^\s*read_xdc(?:\s|$)} $block] && [string first $xdc $block] >= 0} {
            set read_found 1
            if {[regexp -- {-mode\s+\{?out_of_context\}?} $block]} { set ooc_mode_found 1 }
        }
        if {[regexp {^\s*set_property\s+used_in_implementation\s+false\s} $block] && [string first $xdc $block] >= 0} {
            set excluded_from_implementation 1
        }
        set block ""
    }
    if {!$read_found || !$ooc_mode_found || !$excluded_from_implementation} {
        error "Generated core OOC Tcl lacks actual read_xdc -mode out_of_context and implementation exclusion for $xdc; synthesis will not launch"
    }
    foreach top_name {synth_1 impl_1} {
        if {[get_property CONSTRSET [get_runs $top_name]] ne [dict get $top_sets $top_name]} {
            error "Top constraint set changed during script generation"
        }
    }
    # scripts_only leaves Vivado's run in Scripts Generated, which cannot be
    # launched again without reset. Preserve the verified script before that
    # cleanup; never reset an active, completed, or checkpoint-bearing run.
    set run_directory [get_property DIRECTORY $run]
    if {[get_property STATUS $run] ne "Scripts Generated" || [get_property PROGRESS $run] ne "0%" ||
        [llength [glob -nocomplain -directory $run_directory *.dcp]]} {
        error "Core OOC preflight may reset only a fresh Scripts Generated/0% run without a DCP"
    }
    set snapshot [file join $directory robot_rt_bd_core_0.preflight.tcl]
    file copy $script $snapshot
    require_no_active_runs
    reset_runs $run
    set run [::robot_rt_bd::one [get_runs -quiet robot_rt_bd_core_0_synth_1] "reset core OOC synthesis run"]
    set file [::robot_rt_bd::one [get_files -of_objects [get_filesets $fs]] "preserved core OOC XDC"]
    set used_in [get_property USED_IN $file]
    if {[get_property STATUS $run] ne "Not started" || [get_property PROGRESS $run] ne "0%" ||
        [get_property CONSTRSET $run] ne $fs ||
        [file normalize [get_property NAME $file]] ne $xdc || ![file readable $xdc] ||
        [lsearch -exact $used_in synthesis] < 0 || [lsearch -exact $used_in out_of_context] < 0 ||
        [lsearch -exact $used_in implementation] >= 0 || [get_property USED_IN_IMPLEMENTATION $file]} {
        error "Core OOC run reset did not preserve the verified OOC-only constraint set/file"
    }
    foreach top_name {synth_1 impl_1} {
        if {[get_property CONSTRSET [get_runs $top_name]] ne [dict get $top_sets $top_name]} {
            error "Top constraint set changed during core preflight reset"
        }
    }
    set handle [open [file join $directory verification.txt] {WRONLY CREAT EXCL}]
    puts $handle "status=script_preflight_passed"
    puts $handle "actual_clock_hz=$actual_clock_hz"
    puts $handle "target_period_ns=$period"
    puts $handle "xdc=$xdc"
    puts $handle "generated_script=$script"
    puts $handle "verified_script_snapshot=$snapshot"
    puts $handle "read_xdc_out_of_context_verified=1"
    puts $handle "excluded_from_implementation=1"
    puts $handle "top_constraint_sets_unchanged=1"
    puts $handle "synthesis_started_by_preflight=0"
    puts $handle "reset_scripts_generated_without_dcp=1"
    puts $handle "post_reset_status=Not started"
    puts $handle "post_reset_progress=0%"
    close $handle
    puts "Core OOC constraint script verified and fresh run reset: ${period}ns on aclk, OOC-only, no top clock override"
}
proc serial_ip_synthesis {out actual_clock_hz {requested_clock_hz 100000000}} {
    # AMD's installed scripts/project/synth_bd.tcl uses this exact BD-file API
    # to materialize its OOC dependencies before launch. Cached synthesized
    # results are not reset; only the core's no-DCP scripts_only state is reset.
    set bd [::robot_rt_bd::one [get_files -quiet */robot_rt_bd.bd] "robot_rt BD file"]
    set dependencies [create_ip_run $bd]
    prepare_core_ooc_constraints $out $actual_clock_hz $requested_clock_hz
    set top [::robot_rt_bd::one [get_runs -quiet synth_1] "top synthesis run"]
    if {![get_property IS_SYNTHESIS $top]} { error "synth_1 is not a synthesis run" }
    set names {}
    # This is the fresh isolated project created immediately above: synth_1 is
    # its sole top synthesis run; all other synthesis runs belong to this BD.
    # Filter by run type, not an *_synth_1 glob that could include wrong runs.
    foreach run [get_runs -quiet -filter {IS_SYNTHESIS}] {
        set name [get_property NAME $run]
        if {$name ne [get_property NAME $top]} { lappend names $name }
    }
    foreach dependency $dependencies {
        set run [::robot_rt_bd::one [get_runs -quiet $dependency] "BD dependency $dependency"]
        if {![get_property IS_SYNTHESIS $run] || [get_property NAME $run] eq [get_property NAME $top]} {
            error "Unexpected non-IP-synthesis dependency: $dependency"
        }
        if {[lsearch -exact $names [get_property NAME $run]] < 0} {
            error "BD dependency missing from synthesis run enumeration: $dependency"
        }
    }
    if {![llength $names]} { error "No OOC IP synthesis runs were materialized for robot_rt BD" }
    puts "Serial IP synthesis schedule: [lsort $names]"
    foreach name [lsort $names] {
        set run [::robot_rt_bd::one [get_runs -quiet $name] "IP synthesis run $name"]
        if {[run_is_current $run]} {
            puts "Reusing current IP synthesis run: $name"
            continue
        }
        if {[get_property PROGRESS $run] eq "100%"} {
            error "Completed IP run $name is stale/unsuccessful; refusing to reset cached results automatically"
        }
        require_no_active_runs
        puts "Starting single IP synthesis run: $name"
        launch_runs $run -jobs 1
        run_finished $name
        if {![run_is_current $run]} { error "IP synthesis run $name is not current after completion" }
    }
    require_no_active_runs
    # Re-enumerate to catch any nested dependency added during IP generation.
    # Never let the later top launch silently schedule an unfinished IP group.
    foreach run [get_runs -quiet -filter {IS_SYNTHESIS}] {
        if {[get_property NAME $run] ne [get_property NAME $top] && ![run_is_current $run]} {
            error "Unfinished/stale IP synthesis dependency remains before top launch: $run"
        }
    }
}
# Shared single-source report/signoff functions for resident and external scheduling.
proc build_report_synthesis {out} {
open_run synth_1
file mkdir [file join $out reports]
report_utilization -file [file join $out reports utilization_synth.rpt]
report_timing_summary -file [file join $out reports timing_synth.rpt]
close_design
}
proc build_check_route {out actual_clock_hz} {
    set expected_period_ns [expr {1.0e9 / $actual_clock_hz}]
    file mkdir [file join $out reports]
open_run impl_1
report_utilization -file [file join $out reports utilization_route.rpt]
report_timing_summary -delay_type min_max -report_unconstrained -check_timing_verbose \
    -file [file join $out reports timing_route.rpt]
check_timing -verbose -file [file join $out reports check_timing.rpt]
report_drc -file [file join $out reports drc_route.rpt]
set clk [get_clocks -quiet -of_objects [get_pins -hier -filter {REF_PIN_NAME == pl_clk0}]]
if {![llength $clk]} { error "PS PL0 generated clock is missing; no fabricated XDC clock fallback" }
foreach c $clk {
    if {abs([get_property PERIOD $c] - $expected_period_ns) > 0.001} {
        error "PL0 period does not match actual ${actual_clock_hz}Hz PS clock"
    }
}
# Native BRAM CLK interface members do not expose scalar FREQ_HZ.
# Prove the implemented BMG A/B clock pins really receive this same
# PS clock through controller/player aliases, with no new clock.
set ram_cells [get_cells -quiet -hier -filter {REF_NAME =~ RAMB* && NAME =~ *chunks*}]
if {![llength $ram_cells]} { error "Implemented chunk BRAM primitives missing" }
set ram_clock_pins [get_pins -quiet -of_objects $ram_cells \
    -filter {REF_PIN_NAME == CLKARDCLK || REF_PIN_NAME == CLKBWRCLK}]
if {![llength $ram_clock_pins]} { error "Cannot identify implemented chunk BRAM A/B clocks" }
foreach p $ram_clock_pins {
    set ram_clocks [get_clocks -quiet -of_objects $p]
    if {[llength $ram_clocks] != 1 || [lsearch -exact $clk [lindex $ram_clocks 0]] < 0} {
        error "Chunk BRAM clock $p does not receive the single PS clock: $ram_clocks"
    }
    if {abs([get_property PERIOD [lindex $ram_clocks 0]] - $expected_period_ns) > 0.001} {
        error "Chunk BRAM clock $p period mismatches actual ${actual_clock_hz}Hz"
    }
}
foreach delay {max min} {
    set paths [get_timing_paths -quiet -delay_type $delay -max_paths 1 -nworst 1]
    if {![llength $paths]} { error "No $delay timing path: timing coverage unproven" }
    set slack [get_property SLACK [lindex $paths 0]]
    if {![string is double -strict $slack] || $slack < 0.0} { error "$delay timing failed: slack=$slack ns" }
}
set fd [open [file join $out reports timing_route.rpt]]; set summary [read $fd]; close $fd
if {![string match {*All user specified timing constraints are met.*} $summary]} {
    error "Routed timing summary does not confirm all setup/hold/pulse-width constraints; inspect report"
}
# Catch unclocked registers/paths; don't suppress UCIO/NSTD or add false paths.
set fd [open [file join $out reports check_timing.rpt]]; set timing [read $fd]; close $fd
foreach section {no_clock multiple_clock unconstrained_internal_endpoints} {
    set pattern [format {checking %s\s+\(([0-9]+)\)} $section]
    if {![regexp -nocase $pattern $timing unused count]} { error "Cannot parse check_timing $section count; inspect report" }
    if {$count != 0} { error "check_timing reports $section=$count; inspect reports" }
}
if {[llength [get_drc_violations -quiet -filter {SEVERITY == Error}]]} { error "Route DRC errors remain" }
    close_design
}
proc build_export_checked_bitstream {out vivado_root} {
    set bit [file join $out robot_rt.bit]
            open_run impl_1
            set generated_bit [file join [get_property DIRECTORY [get_runs impl_1]] robot_rt_bd_wrapper.bit]
            if {![file exists $generated_bit]} { error "Implementation did not produce $generated_bit" }
            file copy $generated_bit $bit
            write_hw_platform -fixed -include_bit -force -file [file join $out robot_rt.xsa]
            set bif [file join $out robot_rt.bif]
            set fd [open $bif w]
            puts $fd {all:
{
    [destination_device=pl] robot_rt.bit
}}; close $fd
            set bootgen [file join $vivado_root bin bootgen]
            if {![file executable $bootgen]} { error "bootgen missing: $bootgen" }
            set previous_dir [pwd]
            try {
                cd $out
                exec $bootgen -arch zynqmp -image robot_rt.bif -process_bitstream bin -w on
            } finally { cd $previous_dir }
            set bin "$bit.bin"
            if {![file exists $bin] || [file size $bin] == 0} { error "bootgen did not create expected $bin" }
            exec env -u PYTHONHOME -u PYTHONPATH /usr/bin/python3 -c {import hashlib,json,pathlib,sys; p=pathlib.Path(sys.argv[1]); names=['robot_rt.bit','robot_rt.bit.bin','robot_rt.xsa','addresses.json']; rows={n:hashlib.sha256((p/n).read_bytes()).hexdigest() for n in names}; (p/'artifacts.sha256.json').write_text(json.dumps({'schema':'robot_rt.build.v1','timing_passed':True,'board_tested':False,'files':rows},indent=2)+'\n')} $out
            puts "Timing passed; generated $bin. Board/PS-platform compatibility remains unverified."
}
set ::robot_rt_build_api_version 1
# Only a deliberate library source skips the command-line entry point.
if {[info exists ::robot_rt_build_library] && $::robot_rt_build_library} { return }

if {[catch {
    set args {}; set jobs 1
    for {set i 0} {$i < [llength $::argv]} {incr i} {
        set arg [lindex $::argv $i]
        if {$arg eq "--jobs"} {
            incr i; set jobs [lindex $::argv $i]
            if {![string is integer -strict $jobs] || $jobs < 1 || $jobs > 2} { error "--jobs must be 1..2 for this host's current memory budget" }
        } else { lappend args $arg }
    }
    set opts [::robot_rt_bd::options {*}$args]
    if {[dict get $opts help]} { build_usage } else {
        set opts [::robot_rt_bd::inputs $opts]
        if {[dict get $opts check]} { puts "Static inputs checked; no Vivado run or board action." } else {
            set out [::robot_rt_bd::create $opts]
            set actual_clock_hz $::robot_rt_bd::actual_clock_hz
            set expected_period_ns [expr {1.0e9 / $actual_clock_hz}]
            if {$jobs == 1} {
                serial_ip_synthesis $out $actual_clock_hz [dict get $opts clock_hz]
            } else {
                # The optional parallel path must not bypass core constraints.
                create_ip_run [::robot_rt_bd::one [get_files -quiet */robot_rt_bd.bd] "robot_rt BD file"]
                prepare_core_ooc_constraints $out $actual_clock_hz [dict get $opts clock_hz]
            }
            launch_runs synth_1 -jobs $jobs
            run_finished synth_1
            build_report_synthesis $out
            set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED false [get_runs impl_1]
            launch_runs impl_1 -to_step route_design -jobs $jobs
            run_finished impl_1
            build_check_route $out $actual_clock_hz
            # Project mode resumes only after the exact same centralized signoff.
            launch_runs impl_1 -next_step -jobs $jobs
            run_finished impl_1
            build_export_checked_bitstream $out [dict get $opts vivado_root]
        }
    }
} why options]} { puts stderr "ERROR: $why"; exit 1 }

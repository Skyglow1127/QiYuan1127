# Same robot_rt RTL/BD and partition boundary as the100MHz project.
# The Python driver runs synthesis workers serially, then this route/export phase.
set repo [file dirname [file dirname [file normalize [info script]]]]
set phase [lindex $argv 0]
set project [file normalize [lindex $argv 1]]
set work [file normalize [lindex $argv 2]]
set artifacts [file normalize [lindex $argv 3]]
set ::robot_rt_build_library 1
source [file join $repo tcl build.tcl]
unset ::robot_rt_build_library
# Synthesis uses the existing single-process setting on this workstation.
set_param general.maxThreads 1
proc write_run_plan {work} {
    set pairs {}
    foreach run [lsort [get_runs -filter {IS_SYNTHESIS}]] {
        if {[get_property NAME $run] ne "synth_1"} {
            lappend pairs [get_property NAME $run] [get_property DIRECTORY $run]
        }
    }
    lappend pairs synth_1 [get_property DIRECTORY [get_runs synth_1]]
    # Vivado exports its bundled PYTHONHOME; system Python needs its own stdlib.
    exec env -u PYTHONHOME -u PYTHONPATH /usr/bin/python3 -c {import json,pathlib,sys; a=sys.argv[2:]; pathlib.Path(sys.argv[1]).write_text(json.dumps(dict(zip(a[::2],a[1::2])),indent=2)+'\n')} [file join $work run_plan.json] {*}$pairs
}
if {[catch {
    switch -- $phase {
        prepare {
            set opts [::robot_rt_bd::inputs [::robot_rt_bd::options --out $project --clock-hz 50000000]]
            # Keep the proven partition boundaries; compile both children at the
            # new CLOCK_HZ. The parent uses their original module declarations.
            set sources {}
            foreach path [dict get $opts sources] {
                set module [file rootname [file tail $path]]
                if {$module in {player_math limit_gate_pipe}} {
                    lappend sources [file join $work ${module}_stub.sv]
                } else {lappend sources $path}
            }
            dict set opts sources $sources
            ::robot_rt_bd::create $opts
            create_ip_run [get_files */robot_rt_bd.bd]
            set core [get_runs robot_rt_bd_core_0_synth_1]
            set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY none $core
            set_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE RuntimeOptimized $core
            prepare_core_ooc_constraints $project $::robot_rt_bd::actual_clock_hz 50000000
            # scripts_only creates worker files. The controller exits before
            # any synthesis worker starts, as in the existing split build.
            launch_runs synth_1 -scripts_only
            write_run_plan $work
            close_project
        }
        plan {
            open_project [file join $project robot_rt.xpr]
            write_run_plan $work
            close_project
        }
        repair-core {
            open_project [file join $project robot_rt.xpr]
            reset_runs synth_1
            reset_runs robot_rt_bd_core_0_synth_1
            launch_runs synth_1 -scripts_only
            write_run_plan $work
            close_project
        }
        implement {
            open_project [file join $project robot_rt.xpr]
            foreach run [get_runs -filter {IS_SYNTHESIS}] {
                if {![run_is_current $run]} {error "Synthesis incomplete: $run [get_property STATUS $run]"}
            }
            open_run synth_1
            if {[llength [get_cells -hier -filter {IS_BLACKBOX == 1}]]} {error "Unfilled synthesis partition"}
            close_design
            set actual [exec env -u PYTHONHOME -u PYTHONPATH /usr/bin/python3 -c {import json,sys; print(json.load(open(sys.argv[1]))['clock_hz'])} [file join $project addresses.json]]
            file mkdir $artifacts
            file copy [file join $project addresses.json] [file join $artifacts addresses.json]
            build_report_synthesis $artifacts
            set_param general.maxThreads 4
            set run [get_runs impl_1]
            set_property STEPS.OPT_DESIGN.ARGS.DIRECTIVE ExploreWithRemap $run
            set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE Explore $run
            set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true $run
            set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE Explore $run
            set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED false $run
            launch_runs impl_1 -to_step route_design -jobs 1
            run_finished impl_1
            open_run impl_1
            report_route_status -file [file join $artifacts route_status.rpt]
            close_design
            build_check_route $artifacts $actual
            launch_runs impl_1 -to_step write_bitstream -jobs 1
            run_finished impl_1
            build_export_checked_bitstream $artifacts /home/aup/Xilinx/2025.2/Vivado
            puts "50MHZ_BITSTREAM_COMPLETE $artifacts"
            close_project
        }
        default {error "Unknown build phase $phase"}
    }
} message options]} {
    puts stderr "ERROR: $message"
    puts stderr [dict get $options -errorinfo]
    exit 1
}
exit 0

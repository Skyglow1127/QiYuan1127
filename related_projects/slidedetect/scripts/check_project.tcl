set root [file normalize [file join [file dirname [info script]] ..]]
set_param general.maxThreads 2
open_project [file join $root build vivado sfls_kr260.xpr]
current_fileset -simset [get_filesets sim_arithmetic]
launch_simulation -simset sim_arithmetic -mode behavioral
catch {close_sim}
current_fileset -simset [get_filesets sim_1]
close_project

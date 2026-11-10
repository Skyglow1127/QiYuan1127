# Run with vivado -mode gui -source scripts/open_results.tcl
set root [file normalize [file join [file dirname [info script]] ..]]
set_param general.maxThreads 2
open_project [file join $root build vivado sfls_kr260.xpr]
open_wave_database [file join $root reports end_to_end.wdb]
create_wave_config
add_wave /tb_end_to_end/dut/larmor_in
add_wave /tb_end_to_end/dut/window_full
add_wave /tb_end_to_end/dut/result_valid
add_wave -radix unsigned /tb_end_to_end/dut/frequency_uHz
add_wave /tb_end_to_end/dut/fault
add_wave /tb_end_to_end/dut/debug_sample_req
add_wave -radix unsigned /tb_end_to_end/dut/result_last_tick
add_wave -radix unsigned /tb_end_to_end/dut/result_mean_tick
save_wave_config [file join $root reports end_to_end.wcfg]
puts "RESULTS_OPENED: 100 MHz RTL, 350 kHz input, 1 kHz outputs, frequency in uHz"

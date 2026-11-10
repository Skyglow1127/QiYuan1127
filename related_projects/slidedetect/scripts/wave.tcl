# Focused wave database: exclude the 2 Mbit history RAM and free-running clock.
log_wave /tb_end_to_end/dut/larmor_in
log_wave /tb_end_to_end/dut/rst
log_wave /tb_end_to_end/dut/frequency_uHz
log_wave /tb_end_to_end/dut/result_valid
log_wave /tb_end_to_end/dut/window_full
log_wave /tb_end_to_end/dut/fault
log_wave /tb_end_to_end/dut/debug_event_valid
log_wave /tb_end_to_end/dut/debug_sample_req
log_wave /tb_end_to_end/dut/result_last_tick
log_wave /tb_end_to_end/dut/result_mean_tick
run all
quit

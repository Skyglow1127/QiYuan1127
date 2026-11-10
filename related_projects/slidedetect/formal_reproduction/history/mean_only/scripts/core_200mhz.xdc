create_clock -name clk -period 5.000 [get_ports clk]
set_clock_uncertainty 0.100 [get_clocks clk]
# Assumed synchronous integration budget, not board pin constraints.
# There are no asynchronous sensor pins in this timestamp-only core.
set_input_delay -clock clk -max 1.000 [get_ports {rst event_valid event_tick[*] sample_req}]
set_input_delay -clock clk -min 0.200 [get_ports {rst event_valid event_tick[*] sample_req}]
set_output_delay -clock clk -max 1.000 [all_outputs]
set_output_delay -clock clk -min 0.200 [all_outputs]

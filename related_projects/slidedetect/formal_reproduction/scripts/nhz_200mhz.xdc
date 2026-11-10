create_clock -name clk -period 5.000 [get_ports clk]
set_clock_uncertainty 0.100 [get_clocks clk]
# Synchronous module integration assumptions, not board electrical constraints.
set_input_delay -clock clk -max 1.000 [get_ports {rst snapshot_valid snapshot_q[*]}]
set_input_delay -clock clk -min 0.200 [get_ports {rst snapshot_valid snapshot_q[*]}]
set_output_delay -clock clk -max 1.000 [all_outputs]
set_output_delay -clock clk -min 0.200 [all_outputs]

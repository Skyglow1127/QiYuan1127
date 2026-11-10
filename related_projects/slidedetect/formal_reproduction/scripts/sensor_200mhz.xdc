create_clock -name clk -period 5.000 [get_ports clk]
set_clock_uncertainty 0.100 [get_clocks clk]
set_input_delay -clock clk -max 1.000 [get_ports rst]
set_input_delay -clock clk -min 0.200 [get_ports rst]
set_output_delay -clock clk -max 1.000 [all_outputs]
set_output_delay -clock clk -min 0.200 [all_outputs]
# The comparator signal is asynchronous; only its first synchronizer stage
# is exempt. The second stage and the whole timestamp path remain timed.
set_false_path -from [get_ports larmor_in] -to [get_pins -hier -filter {NAME =~ *sync1_reg/D}]

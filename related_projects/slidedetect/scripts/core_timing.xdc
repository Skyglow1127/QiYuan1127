# OOC core timing only. No board pinout/bitstream is claimed by this project.
create_clock -name clk -period 10.000 [get_ports clk]
set_clock_uncertainty 0.100 [get_clocks clk]
# Only the asynchronous input -> FIRST synchronizer stage is excluded.
set_false_path -from [get_ports larmor_in] -to [get_pins -hier -filter {NAME =~ *sync1_reg/D}]
set_input_delay -clock clk -max 2.0 [get_ports rst]
# Synchronous integration contract: rst launched by upstream logic arrives
# 0.5..2.0 ns after clk. This is NOT an asynchronous board reset pin constraint.
set_input_delay -clock clk -min 0.5 [get_ports rst]
set_output_delay -clock clk -max 2.0 [all_outputs]
# Integration contract includes 0.2 ns minimum external data-path delay.
# OOC ports have no physical sink/pad routes; this is not a board hold signoff.
set_output_delay -clock clk -min 0.2 [all_outputs]

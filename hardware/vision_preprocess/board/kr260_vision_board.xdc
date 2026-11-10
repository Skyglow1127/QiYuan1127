# Official vendored board pin evidence: boards/SOURCES.json.
# C3=HPA_CLK0P_CLK, 25 MHz; A12=active-low fan enable, existing repository mapping.
set_property PACKAGE_PIN C3 [get_ports clk_25mhz]
set_property IOSTANDARD LVCMOS18 [get_ports clk_25mhz]
create_clock -name board_clk_25mhz -period 40.000 [get_ports clk_25mhz]
set_property PACKAGE_PIN A12 [get_ports fan_en_b]
set_property IOSTANDARD LVCMOS33 [get_ports fan_en_b]
set_property DRIVE 4 [get_ports fan_en_b]
set_property SLEW SLOW [get_ports fan_en_b]
# User-confirmed four-wire wiring; these two ports are INPUT only.
# J21.11 -> SOM240_2 B58 -> AB14; J21.12 -> SOM240_2 A54 -> Y14.
set_property PACKAGE_PIN AB14 [get_ports {fsr_do[0]}]
set_property PACKAGE_PIN Y14 [get_ports {fsr_do[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {fsr_do[*]}]
# No internal pullups; the physical module defines the electrical bias.
# False-path only to the first synchronizer stage. Stage1->stage2 remains timed.
set fsr_first_stage [get_cells -hier -filter {NAME =~ *sync_i*meta_reg*}]
set_false_path -from [get_ports {fsr_do[*]}] -to [get_pins -of_objects $fsr_first_stage -filter {REF_PIN_NAME == D}]

# Sources: boards/SOURCES.json and AMD kria-vitis-platforms KR260 pin_timing.xdc.
# HPA_CLK0P_CLK -> SOM240_1 A6 -> K26 C3. Carrier specifies LVCMOS18.
set_property PACKAGE_PIN C3 [get_ports clk_25mhz]
set_property IOSTANDARD LVCMOS18 [get_ports clk_25mhz]
create_clock -name board_clk_25mhz -period 40.000 [get_ports clk_25mhz]

# Fan enable is a static active-low control, not a synchronous data interface.
set_property PACKAGE_PIN A12 [get_ports fan_en_b]
set_property IOSTANDARD LVCMOS33 [get_ports fan_en_b]
set_property DRIVE 4 [get_ports fan_en_b]
set_property SLEW SLOW [get_ports fan_en_b]

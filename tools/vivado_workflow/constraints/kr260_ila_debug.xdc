# Implementation-only netlist insertion. All three probes share the 25 MHz clock.
create_debug_core u_ila ila
set_property C_DATA_DEPTH 1024 [get_debug_cores u_ila]
set_property C_TRIGIN_EN false [get_debug_cores u_ila]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila]
set_property C_ADV_TRIGGER false [get_debug_cores u_ila]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila]
set_property C_EN_STRG_QUAL false [get_debug_cores u_ila]
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila]
set_property ALL_PROBE_SAME_MU_CNT 1 [get_debug_cores u_ila]
set_property port_width 1 [get_debug_ports u_ila/clk]
connect_debug_port u_ila/clk [get_nets clk]
set_property port_width 16 [get_debug_ports u_ila/probe0]
connect_debug_port u_ila/probe0 [get_nets [list {cycle_count[0]} {cycle_count[1]} {cycle_count[2]} {cycle_count[3]} {cycle_count[4]} {cycle_count[5]} {cycle_count[6]} {cycle_count[7]} {cycle_count[8]} {cycle_count[9]} {cycle_count[10]} {cycle_count[11]} {cycle_count[12]} {cycle_count[13]} {cycle_count[14]} {cycle_count[15]}]]
create_debug_port u_ila probe
set_property port_width 1 [get_debug_ports u_ila/probe1]
connect_debug_port u_ila/probe1 [get_nets blink]
create_debug_port u_ila probe
set_property port_width 1 [get_debug_ports u_ila/probe2]
connect_debug_port u_ila/probe2 [get_nets rst_n]
set_property C_CLK_INPUT_FREQ_HZ 25000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk]

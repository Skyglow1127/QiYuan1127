open_project -reset depth_gate_rtl
set_top depth_gate
add_files depth_gate.cpp
add_files -tb test_depth_gate_rtl.cpp
open_solution -reset solution1 -flow_target vivado
set_part {xck26-sfvc784-2LV-c}
create_clock -period 10 -name default
csynth_design
cosim_design -rtl verilog -trace_level none
exit

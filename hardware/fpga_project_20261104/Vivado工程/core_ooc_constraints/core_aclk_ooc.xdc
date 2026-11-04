# Core OOC only; actual CLOCK_HZ=49999500, target period 20ns.
create_clock -name core_aclk_ooc -period 20 [get_ports aclk]

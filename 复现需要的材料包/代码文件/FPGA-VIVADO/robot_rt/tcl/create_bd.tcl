# From-scratch KR260 DESC_ONLY project. No device connection or CAN backend.
# Standalone: vivado -mode batch -source tcl/create_bd.tcl -tclargs --out /tmp/rt_bd
# Static input check only: tclsh tcl/create_bd.tcl --check-inputs
namespace eval ::robot_rt_bd {
    variable here [file dirname [file normalize [info script]]]
    variable root [file dirname $here]
    variable board_part "xilinx.com:kr260_som:part0:1.1"
    variable part "xck26-sfvc784-2LV-c"
    variable actual_clock_hz 0
}
proc ::robot_rt_bd::usage {} {
    puts {Usage: create_bd.tcl [--out NEW_DIRECTORY] [--vivado-root DIRECTORY] [--clock-hz 50000000|100000000] [--check-inputs] [--help]
Creates only a new DESC_ONLY project; never runs synthesis or touches a board.
Default output: robot_rt/build/kr260_desc_only. Existing projects are rejected.
--check-inputs reads local board/preset/IP/RTL inputs with ordinary tclsh.}
}
proc ::robot_rt_bd::options {args} {
    variable root
    set vr /home/aup/Xilinx/2025.2/Vivado
    if {[info exists ::env(XILINX_VIVADO)]} { set vr $::env(XILINX_VIVADO) }
    set opts [dict create out [file join $root build kr260_desc_only] vivado_root $vr clock_hz 50000000 check 0 help 0]
    while {[llength $args]} {
        set key [lindex $args 0]; set args [lrange $args 1 end]
        switch -- $key {
            --help { dict set opts help 1 }
            --check-inputs { dict set opts check 1 }
            --clock-hz {
                if {![llength $args] || [lindex $args 0] ni {50000000 100000000}} {
                    error "--clock-hz must be 50000000 or 100000000"
                }
                dict set opts clock_hz [lindex $args 0]
                set args [lrange $args 1 end]
            }
            --out - --vivado-root {
                if {![llength $args]} { error "$key needs a value" }
                set dest [expr {$key eq "--out" ? "out" : "vivado_root"}]
                dict set opts $dest [file normalize [lindex $args 0]]
                set args [lrange $args 1 end]
            }
            default { error "Unknown option: $key" }
        }
    }
    return $opts
}
proc ::robot_rt_bd::inputs {opts} {
    variable root
    set vr [dict get $opts vivado_root]
    set repo [file join $vr data xhub boards XilinxBoardStore boards Xilinx]
    set board [file join $repo kr260_som 1.1 board.xml]
    set preset [file join $repo kr260_som 1.1 preset.xml]
    foreach p [list $board $preset \
        [file join $vr data ip xilinx axi_bram_ctrl_v4_1 component.xml] \
        [file join $vr data ip xilinx blk_mem_gen_v8_4 component.xml]] {
        if {![file readable $p]} { error "Required installed board/preset/IP file missing: $p" }
    }
    set fd [open $board]; set xml [read $fd]; close $fd
    if {![string match {*part_name="xck26-sfvc784-2LV-c"*} $xml] ||
        ![string match {*preset_proc="mpsoc_preset_vsom"*} $xml]} {
        error "Board part/preset does not match the verified KR260 SOM 1.1 input"
    }
    set fd [open $preset]; set xml [read $fd]; close $fd
    if {![string match {*preset_proc_name="mpsoc_preset_vsom"*} $xml] ||
        ![regexp {name="CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ" value="100"} $xml]} {
        error "Verified 100MHz mpsoc_preset_vsom is unavailable; no generic PS fallback"
    }
    set sources [lsort [concat [glob -nocomplain [file join $root rtl *.sv]] \
        [glob -nocomplain [file join $root rtl *.v]]]]
    foreach name {robot_rt_top.sv robot_rt_axi_guard.sv robot_rt_top_bd.v robot_rt_axi_guard_bd.v tick_gen.sv player_math.sv stationary_seed_gate_seq.sv} {
        if {![file readable [file join $root rtl $name]]} { error "RTL not ready: $root/rtl/$name" }
    }
    if {![file readable [file join $root constraints robot_rt.xdc]]} { error "Missing robot_rt.xdc" }
    dict set opts board_repo $repo
    dict set opts board_file $board
    dict set opts preset_file $preset
    dict set opts sources $sources
    return $opts
}
proc ::robot_rt_bd::one {objects description} {
    if {[llength $objects] != 1} { error "Expected exactly one $description; found: $objects" }
    return [lindex $objects 0]
}
proc ::robot_rt_bd::pin {name} { return [one [get_bd_pins -quiet $name] "pin $name"] }
proc ::robot_rt_bd::intf {name} { return [one [get_bd_intf_pins -quiet $name] "interface $name"] }
proc ::robot_rt_bd::wire {args} {
    set pins {}; foreach name $args { lappend pins [pin $name] }
    connect_bd_net {*}$pins
}
proc ::robot_rt_bd::constant {name width value} {
    set c [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 $name]
    set_property -dict [list CONFIG.CONST_WIDTH $width CONFIG.CONST_VAL $value] $c
    return $c
}
proc ::robot_rt_bd::mapped {space offset bytes} {
    set matches {}
    foreach s [get_bd_addr_segs -quiet -of_objects $space] {
        set off [get_property OFFSET $s]; set size [get_property RANGE $s]
        if {$off ne "" && $size ne "" && $off == $offset && $size == $bytes} { lappend matches $s }
    }
    return [one $matches "mapped segment at $offset size $bytes"]
}
proc ::robot_rt_bd::exclusive_interface {a b} {
    set ap [intf $a]; set bp [intf $b]
    set net [one [get_bd_intf_nets -quiet -of_objects $ap] "net of $a"]
    set attached [lsort [get_bd_intf_pins -quiet -of_objects $net]]
    if {$attached ne [lsort [list $ap $bp]]} {
        error "Expected exclusive interface $a <-> $b; found $attached"
    }
}
proc ::robot_rt_bd::create {opts} {
    variable root; variable board_part; variable part; variable actual_clock_hz
    if {![llength [info commands create_project]]} { error "Use Vivado for project creation, or tclsh --check-inputs" }
    set out [dict get $opts out]
    if {[file exists $out]} { error "Output already exists; choose a new --out directory: $out" }
    set_param general.maxThreads 1
    set_param board.repoPaths [list [dict get $opts board_repo]]
    one [get_board_parts -quiet $board_part] "installed board part $board_part"
    create_project robot_rt $out -part $part
    set_property BOARD_PART $board_part [current_project]
    set_property target_language Verilog [current_project]
    add_files -norecurse [dict get $opts sources]
    add_files -fileset constrs_1 -norecurse [file join $root constraints robot_rt.xdc]
    update_compile_order -fileset sources_1
    create_bd_design robot_rt_bd
    set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.5 ps]
    apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} $ps
    set requested_clock_hz [dict get $opts clock_hz]
    set requested_mhz [expr {$requested_clock_hz / 1000000}]
    # Keep board DDR/MIO preset. Only HPM0 and PL clock/reset are selected here.
    set_property -dict [list CONFIG.PSU__USE__M_AXI_GP0 1 CONFIG.PSU__USE__M_AXI_GP1 0 \
        CONFIG.PSU__USE__M_AXI_GP2 0 CONFIG.PSU__MAXIGP0__DATA_WIDTH 32 \
        CONFIG.PSU__FPGA_PL0_ENABLE 1 CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ $requested_mhz \
        CONFIG.PSU__USE__FABRIC__RST 1 CONFIG.PSU__NUM_FABRIC_RESETS 1] $ps
    # Keep the board reference/PLL preset. Read its derived frequency instead
    # of assuming the requested integer frequency. CLOCK_HZ drives DDS timing
    # and trajectory arithmetic together, so f_exec and physical units agree.
    set actual_clock_hz [get_property CONFIG.FREQ_HZ [pin ps/pl_clk0]]
    if {![string is wideinteger -strict $actual_clock_hz] || $actual_clock_hz <= 0} {
        error "PS PL0 did not provide a valid actual CONFIG.FREQ_HZ: $actual_clock_hz"
    }
    set sc [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect:1.0 fabric]
    set_property -dict [list CONFIG.NUM_SI 1 CONFIG.NUM_MI 2 CONFIG.NUM_CLKS 1] $sc
    set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 resets]
    # proc_sys_reset bd/bd.tcl marks C_EXT_RESET_HIGH propagate-only.
    # The ACTIVE_LOW PS pl_resetn0 pin supplies its polarity; assert it after
    # validation instead of trying to set this read-only derived parameter.
    set_property CONFIG.C_AUX_RESET_HIGH 1 $rst
    # Vivado module-reference requires Verilog-2001 at the reference boundary.
    # Thin .v wrappers instantiate the unchanged SystemVerilog implementation.
    set core [create_bd_cell -type module -reference robot_rt_top_bd core]
    set_property CONFIG.CLOCK_HZ $actual_clock_hz $core
    set guard [create_bd_cell -type module -reference robot_rt_axi_guard_bd guard]
    set_property CONFIG.BASE_ADDR 0xA0020000 $guard
    foreach name {core/aclk guard/aclk} {
        set_property CONFIG.FREQ_HZ $actual_clock_hz [pin $name]
    }
    foreach name {core/s_axi_cfg guard/s_axi_chunk guard/m_axi_bram} {
        set_property CONFIG.FREQ_HZ $actual_clock_hz [intf $name]
    }
    set ctrl [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_bram_ctrl:4.1 bram_ctrl]
    # Controller BD propagation derives ID_WIDTH from its AXI master and
    # MEM_DEPTH from the assigned address range; both are read-only here.
    set_property -dict [list CONFIG.DATA_WIDTH 32 CONFIG.PROTOCOL AXI4 \
        CONFIG.SINGLE_PORT_BRAM 1 CONFIG.USE_ECC 0 CONFIG.READ_LATENCY 1] $ctrl
    set ram [create_bd_cell -type ip -vlnv xilinx.com:ip:blk_mem_gen:8.4 chunks]
    set_property -dict [list CONFIG.Memory_Type True_Dual_Port_RAM CONFIG.use_bram_block BRAM_Controller \
        CONFIG.Use_Byte_Write_Enable true CONFIG.Byte_Size 8 \
        CONFIG.Write_Width_A 32 CONFIG.Read_Width_A 32 CONFIG.Write_Depth_A 32768 \
        CONFIG.Write_Width_B 32 CONFIG.Read_Width_B 32 CONFIG.Enable_A Use_ENA_Pin CONFIG.Enable_B Use_ENB_Pin \
        CONFIG.Use_RSTA_Pin true CONFIG.Use_RSTB_Pin true CONFIG.Reset_Type SYNC \
        CONFIG.Register_PortA_Output_of_Memory_Primitives false CONFIG.Register_PortA_Output_of_Memory_Core false \
        CONFIG.Register_PortB_Output_of_Memory_Primitives false CONFIG.Register_PortB_Output_of_Memory_Core false \
        CONFIG.ECC false] $ram
    constant zero 1 0; constant one 1 1
    connect_bd_intf_net [intf ps/M_AXI_HPM0_FPD] [intf fabric/S00_AXI]
    connect_bd_intf_net [intf fabric/M00_AXI] [intf core/s_axi_cfg]
    connect_bd_intf_net [intf fabric/M01_AXI] [intf guard/s_axi_chunk]
    connect_bd_intf_net [intf guard/m_axi_bram] [intf bram_ctrl/S_AXI]
    connect_bd_intf_net [intf bram_ctrl/BRAM_PORTA] [intf chunks/BRAM_PORTA]
    connect_bd_intf_net [intf core/PLAYER_BRAM] [intf chunks/BRAM_PORTB]
    wire ps/pl_clk0 ps/maxihpm0_fpd_aclk fabric/aclk resets/slowest_sync_clk core/aclk guard/aclk bram_ctrl/s_axi_aclk
    wire ps/pl_resetn0 resets/ext_reset_in
    wire one/dout resets/dcm_locked core/backend_ready
    wire zero/dout resets/aux_reset_in resets/mb_debug_sys_rst core/hard_fault
    wire resets/peripheral_aresetn core/aresetn guard/aresetn bram_ctrl/s_axi_aresetn
    wire resets/interconnect_aresetn fabric/aresetn
    wire core/bank_writable guard/bank_writable
    wire guard/write_busy core/write_busy
    # Constants are simulation/descriptor-only readiness, not a wired hard-fault or CAN backend.
    # Deliberately do not export debug, descriptor, disable_req or motion_permit to PL pins.
    set ps_space [one [get_bd_addr_spaces -quiet ps/Data] "PS data address space"]
    set cfg_seg [one [get_bd_addr_segs -quiet -of_objects [intf core/s_axi_cfg]] "core cfg segment"]
    set guard_seg [one [get_bd_addr_segs -quiet -of_objects [intf guard/s_axi_chunk]] "guard slave segment"]
    set mem_seg [one [get_bd_addr_segs -quiet -of_objects [intf bram_ctrl/S_AXI]] "BRAM slave segment"]
    assign_bd_address -offset 0xA0000000 -range 0x1000 -target_address_space $ps_space $cfg_seg -force
    assign_bd_address -offset 0xA0020000 -range 0x20000 -target_address_space $ps_space $guard_seg -force
    set guard_space [one [get_bd_addr_spaces -quiet -of_objects [intf guard/m_axi_bram]] "guard master address space"]
    assign_bd_address -offset 0xA0020000 -range 0x20000 -target_address_space $guard_space $mem_seg -force
    validate_bd_design
    if {[get_property CONFIG.FREQ_HZ [pin ps/pl_clk0]] != $actual_clock_hz ||
        [get_property CONFIG.CLOCK_HZ $core] != $actual_clock_hz} {
        error "Actual PS clock changed during validation or did not reach arithmetic"
    }
    foreach name {core/aclk guard/aclk bram_ctrl/s_axi_aclk fabric/aclk} {
        if {[get_property CONFIG.FREQ_HZ [pin $name]] != $actual_clock_hz} {
            error "$name clock metadata does not match actual PS clock $actual_clock_hz"
        }
    }
    # BRAM CLK members have no FREQ_HZ property. They carry the complete native
    # interface clock; aclk explicitly associates both interfaces, and the
    # wrapper passes aclk through unchanged. Check the real RAM clock after
    # implementation too, rather than inventing a separate clock constraint.
    set associated [split [get_property CONFIG.ASSOCIATED_BUSIF [pin core/aclk]] :]
    if {[lsearch -exact $associated PLAYER_BRAM] < 0 || [lsearch -exact $associated s_axi_cfg] < 0} {
        error "Core input clock must associate both cfg and native player BRAM interfaces"
    }
    foreach name {core/s_axi_cfg guard/s_axi_chunk guard/m_axi_bram bram_ctrl/S_AXI fabric/M00_AXI fabric/M01_AXI} {
        if {[get_property CONFIG.FREQ_HZ [intf $name]] != $actual_clock_hz} {
            error "$name AXI clock metadata does not match actual PS clock $actual_clock_hz"
        }
    }
    exclusive_interface bram_ctrl/BRAM_PORTA chunks/BRAM_PORTA
    exclusive_interface core/PLAYER_BRAM chunks/BRAM_PORTB
    if {[get_property CONFIG.POLARITY [pin resets/ext_reset_in]] ne "ACTIVE_LOW" ||
        [get_property CONFIG.C_EXT_RESET_HIGH $rst] != 0} {
        error "PS active-low reset polarity did not propagate to proc_sys_reset"
    }
    mapped $ps_space 0xA0000000 0x1000
    mapped $ps_space 0xA0020000 0x20000
    mapped $guard_space 0xA0020000 0x20000
    # Fail on changes that would invalidate the byte addressing or latency contract.
    foreach {prop expected} {CONFIG.Memory_Type True_Dual_Port_RAM CONFIG.Enable_32bit_Address true CONFIG.Write_Depth_A 32768 CONFIG.READ_LATENCY_A 1 CONFIG.READ_LATENCY_B 1} {
        if {[get_property $prop $ram] ne $expected} { error "BMG $prop unexpectedly [get_property $prop $ram], expected $expected" }
    }
    foreach {prop expected} {CONFIG.SINGLE_PORT_BRAM 1 CONFIG.MEM_DEPTH 32768 CONFIG.ID_WIDTH 1 CONFIG.READ_LATENCY 1} {
        if {[get_property $prop $ctrl] != $expected} { error "Controller $prop unexpectedly [get_property $prop $ctrl], expected $expected" }
    }
    if {[get_property LEFT [pin chunks/addrb]] != 31} { error "Expected 32-bit byte-addressed BMG port B" }
    save_bd_design
    generate_target all [get_files */robot_rt_bd.bd]
    set wrapper [make_wrapper -files [get_files */robot_rt_bd.bd] -top]
    add_files -norecurse $wrapper
    set_property top robot_rt_bd_wrapper [current_fileset]
    update_compile_order -fileset sources_1
    set fd [open [file join $out addresses.json] w]
    puts $fd "\{\n  \"schema\": \"robot_rt.addresses.v1\",\n  \"status\": \"generated_design_not_board_verified\",\n  \"board_part\": \"$board_part\",\n  \"part\": \"$part\",\n  \"clock_hz\": $actual_clock_hz,\n  \"requested_clock_hz\": $requested_clock_hz,\n  \"clock_source\": \"ps/pl_clk0\",\n  \"backend\": \"DESC_ONLY\",\n  \"cfg_base\": 2684354560,\n  \"cfg_bytes\": 4096,\n  \"chunk_base\": 2684485632,\n  \"chunk_bytes\": 131072,\n  \"expected_id\": 1381257292\n\}"
    close $fd
    set fd [open [file join $out address_map.txt] w]
    foreach space [list $ps_space $guard_space] {
        foreach s [get_bd_addr_segs -quiet -of_objects $space] {
            puts $fd "$space $s offset=[get_property OFFSET $s] range=[get_property RANGE $s]"
        }
    }
    close $fd
    puts "Created DESC_ONLY project: $out (not synthesized or board tested)"
    return $out
}
if {![info exists ::robot_rt_bd_library]} {
    if {[catch {
        set opts [::robot_rt_bd::options {*}$::argv]
        if {[dict get $opts help]} { ::robot_rt_bd::usage } else {
            set opts [::robot_rt_bd::inputs $opts]
            if {[dict get $opts check]} { puts "Static board/preset/IP/RTL inputs exist; Vivado BD not executed." } else { ::robot_rt_bd::create $opts }
        }
    } why options]} { puts stderr "ERROR: $why"; exit 1 }
}

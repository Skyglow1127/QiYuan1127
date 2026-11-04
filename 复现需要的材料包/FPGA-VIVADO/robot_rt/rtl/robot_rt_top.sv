`timescale 1ns/1ps
// DESC_ONLY implementation. Never connects to CAN or grants physical enable.
// Functional RTL and standalone simulation are not synthesis/timing evidence.
module robot_rt_top #(
    parameter integer CLOCK_HZ = 100000000
) (
    (* X_INTERFACE_INFO="xilinx.com:signal:clock:1.0 aclk CLK", X_INTERFACE_PARAMETER="ASSOCIATED_BUSIF s_axi_cfg, ASSOCIATED_RESET aresetn" *) input logic aclk,
    (* X_INTERFACE_INFO="xilinx.com:signal:reset:1.0 aresetn RST", X_INTERFACE_PARAMETER="POLARITY ACTIVE_LOW" *) input logic aresetn,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg AWADDR", X_INTERFACE_PARAMETER="PROTOCOL AXI4LITE, DATA_WIDTH 32, ADDR_WIDTH 12, HAS_PROT 1, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1" *) input logic [11:0] s_axi_cfg_awaddr,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg AWPROT" *) input logic [2:0] s_axi_cfg_awprot,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg AWVALID" *) input logic s_axi_cfg_awvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg AWREADY" *) output logic s_axi_cfg_awready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg WDATA" *) input logic [31:0] s_axi_cfg_wdata,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg WSTRB" *) input logic [3:0] s_axi_cfg_wstrb,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg WVALID" *) input logic s_axi_cfg_wvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg WREADY" *) output logic s_axi_cfg_wready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg BRESP" *) output logic [1:0] s_axi_cfg_bresp,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg BVALID" *) output logic s_axi_cfg_bvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg BREADY" *) input logic s_axi_cfg_bready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg ARADDR" *) input logic [11:0] s_axi_cfg_araddr,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg ARPROT" *) input logic [2:0] s_axi_cfg_arprot,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg ARVALID" *) input logic s_axi_cfg_arvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg ARREADY" *) output logic s_axi_cfg_arready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg RDATA" *) output logic [31:0] s_axi_cfg_rdata,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg RRESP" *) output logic [1:0] s_axi_cfg_rresp,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg RVALID" *) output logic s_axi_cfg_rvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_cfg RREADY" *) input logic s_axi_cfg_rready,
    output logic [16:0] b_addr,
    (* X_INTERFACE_INFO="xilinx.com:signal:clock:1.0 b_clk CLK" *) output wire b_clk, output logic b_en,
    output wire b_rst, output wire [31:0] b_din, output wire [3:0] b_we,
    input logic [31:0] b_dout,
    output logic [1:0] bank_writable, input logic [1:0] write_busy,
    input logic hard_fault, backend_ready,
    output wire disable_req, motion_permit, output wire dbg_tick, output logic dbg_desc,
    output logic tx_desc_valid, output logic [767:0] tx_desc_data
);
    initial begin
        if(CLOCK_HZ<=0) $fatal(1,"CLOCK_HZ must be a positive signed 32-bit integer");
    end
    localparam [2:0] DISARMED=0,ARMED=1,RUN=2,RAMP=3,HOLD=4,FAULT=5;
    localparam [2:0] FREE=0,VALIDATING=1,READY=2,ACTIVE=3,REJECTED=4;
    localparam [3:0] C_IDLE=0,C_CRC=1,C_CHECK=2,C_ACTIVATE=3,C_BANK_WAIT=4,C_BANK_SETTLE=5,C_DISARM=6,C_SEED_CRC=7,C_ARM_CHECK=8,C_BANK_SEED_WAIT=9;
    localparam [1:0] S_NONE=0,S_CONFIG=1,S_BANK=2,S_ARM=3;
    // Sparse physical storage; preserve the full legacy read-only address view.
    // Invalid/unmapped addresses never alias slot0. No read or bus cycle added.
    localparam integer SHADOW_WORDS = 130;
    wire [31:0] shadow[0:1023];
    logic [31:0] shadow_words[0:SHADOW_WORDS-1],cfg[0:1023];
    function automatic integer shadow_slot(input integer byte_address);
        integer relative;
        begin
            shadow_slot=-1;
            relative=byte_address-'h100;
            if ((byte_address & 3)==0) begin
                if(byte_address>='h30 && byte_address<='h70)
                    shadow_slot=(byte_address-'h30)>>2;
                else if(byte_address>='h80 && byte_address<'ha0)
                    shadow_slot=17+((byte_address-'h80)>>2);
                else if(byte_address>='h100 && byte_address<'h2c0 && (relative & 63)<48)
                    shadow_slot=25+((relative>>6)*12)+((relative & 63)>>2);
                else if(byte_address>='h400 && byte_address<='h448)
                    shadow_slot=109+((byte_address-'h400)>>2);
                else if(byte_address=='h460) shadow_slot=128;
                else if(byte_address=='h464) shadow_slot=129;
            end
        end
    endfunction
    generate for(genvar shadow_word=0;shadow_word<1024;shadow_word=shadow_word+1) begin : shadow_view
        localparam integer SLOT=shadow_slot(shadow_word*4);
        if(SLOT>=0 && SLOT<SHADOW_WORDS) begin : present
            assign shadow[shadow_word]=shadow_words[SLOT];
        end else begin : absent
            assign shadow[shadow_word]=32'd0;
        end
    end endgenerate
    logic [2:0] state, bank_state[0:1];
    logic configured, tick_commit, dds_tick,dds_error,dds_quiet;
    logic [63:0] cycle,tick_index,origin;
    logic [3:0] fault_code,reject_code,cmd_state;
    logic [63:0] fault_cycle,last_nonce,last_accepted,last_kick_seq,last_kick_cycle;
    logic kick_valid,stop_pending;
    logic [63:0] stop_started;
    logic hard_meta,hard_sync;
    logic [31:0] cfg_crc_acc;
    logic [7:0] cfg_crc_index;
    // CONFIG-only wide arithmetic. P0 captures immutable shadow operands/tag;
    // P1 computes full96 products while the original131word CRC keeps running.
    logic cfg_rate_prepare,cfg_rate_fresh;
    logic [63:0] cfg_rate_num_snapshot,cfg_rate_den_snapshot,cfg_rate_nonce_snapshot;
    logic [31:0] cfg_rate_epoch_snapshot,cfg_rate_crc_snapshot;
    logic [95:0] cfg_rate_threshold,cfg_rate_max_gap;
    logic cmd_bank;
    logic [1:0] settle_count;
    logic aw_saved,w_saved;
    logic [11:0] aw_address;
    logic [31:0] w_value;
    logic [3:0] w_strobes;
    logic [63:0] pending_sequence;
    logic [447:0] lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,seed_q;
    logic [223:0] seed_q16,seed_wire;
    logic seed_allow,seed_clamp,seed_bounds_valid;
    // One sequential seed/bounds service, independently owned by a command.
    // These input muxes end at E0 snapshot registers inside the service; no
    // cmd_state mux propagates through a combinational quantization gate.
    logic [447:0] seed_seq_lower,seed_seq_upper,seed_seq_vmax,seed_seq_amax,seed_seq_jmax,seed_seq_dq;
    logic seed_seq_start,seed_seq_busy,seed_seq_done,seed_seq_allow,seed_seq_clamp,seed_seq_bounds;
    logic [223:0] seed_seq_q16,seed_seq_wire;
    logic seed_seq_resetn;
    logic [1:0] seed_owner;
    logic [5:0] seed_age;
    logic [447:0] seed_q_tag;
    logic [31:0] seed_epoch_tag,seed_crc_tag,seed_cert_epoch,cfg_seed_epoch,cfg_seed_crc;
    logic [63:0] seed_nonce_tag,seed_cert_nonce,cfg_seed_nonce;
    logic cfg_seed_done,seed_cert_done,bank_needs_seed;
    logic [63:0] bank_parent_sequence;
    logic cache_busy,cache_done,cache_valid;
    logic [31:0] cached_cfg_epoch;
    logic [1343:0] cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c;
    logic [63:0] bseq[0:1],bprev[0:1],bfirstcycle[0:1],bfirsttick[0:1],bendcycle[0:1],bendtick[0:1],bbrakeend[0:1],bdeadline[0:1];
    logic [31:0] berror[0:1];
    logic [7:0] bnormal[0:1],bbrake[0:1];
    logic [447:0] bfirstq[0:1],bfirstv[0:1],bfirsta[0:1],bendq[0:1],bendv[0:1],benda[0:1];
    logic [223:0] bendwire[0:1];
    logic v_start,v_bank,v_busy,v_done,v_accept,v_req,v_grant,v_valid;
    logic [16:0] v_addr; logic [3:0] v_reject;
    logic [63:0] vs,vprev,vfirstcycle,vfirsttick,vendcycle,vendtick,vbrakeend,vobs,vprod,vdeadline;
    logic [31:0] verror; logic [7:0] vn,vb;
    logic [447:0] vfirstq,vfirstv,vfirsta,vendq,vendv,venda;
    logic [223:0] vendwire;
    logic [63:0] parent_seq,parent_cycle,parent_tick;
    logic [447:0] parent_q,parent_v,parent_a;
    logic [223:0] parent_wire;
    logic val_resetn;
    logic vx_math_req,vx_gate_req,vx_math_grant,vx_math_done,vx_math_overflow;
    logic vx_gate_grant,vx_gate_done,vx_allow,vx_clamp,vx_cache_valid;
    logic [31:0] vx_dt;
    logic [447:0] vx_q0,vx_v0,vx_a0,vx_j,vx_lower,vx_upper,vx_vmax,vx_amax,vx_jmax,vx_dq;
    logic [447:0] vx_q1,vx_v1,vx_a1;
    logic [223:0] vx_previous_wire,vx_q16,vx_wire;
    logic shared_collision,shared_protocol_fault;

    logic pf_active,pf_bank,cur_valid,cur_bank;
    logic [7:0] pf_record_index,cur_record_index;
    logic [6:0] pf_issued,pf_received;
    // cur_record is filled only while invalid; word63 atomically publishes it.
    logic [2047:0] cur_record;
    logic pf_req,pf_grant,pf_valid;
    logic [16:0] pf_addr;
    logic rd_pending,rd_owner;
    logic [6:0] rd_word;
    logic active_bank;
    logic [447:0] committed_q,committed_v,committed_a;
    logic [223:0] previous_wire;
    logic pipe_active,pipe_math_ready,pipe_gate_ready,pipe_bad;
    logic [6:0] pipe_age;
    logic [63:0] pipe_cycle,pipe_tick;
    logic [31:0] pipe_dt;
    logic [447:0] pipe_q,pipe_v,pipe_a,pipe_j;
    logic [2:0] pipe_mode;
    logic math_start,math_busy,math_done,math_overflow;
    logic [447:0] out_q,out_v,out_a;
    wire [447:0] out_q_latched,out_v_latched,out_a_latched;
    logic gate_allow,gate_clamp,gate_busy,gate_done;
    logic [1343:0] qp0_terms,qp1_terms,qp2_terms,qp3_terms;
    logic [1343:0] vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms;
    logic [223:0] gate_q16,gate_wire;
    logic [767:0] packed_desc,last_desc,snap_desc;
    logic [63:0] emit_count,reject_count,clamp_count,starve_count,backpressure_count;
    logic [63:0] latency_min,latency_max;
    logic [63:0] snapshots[0:10];
    // Explicit continuous array references are essential here. A port
    // expression cf64(constant) can be folded by simulators without tracking
    // the function's hidden dependency on the global cfg array.
    wire[63:0] live_rate_num={cfg['h44>>2],cfg['h40>>2]};
    wire[63:0] live_rate_den={cfg['h4c>>2],cfg['h48>>2]};
    wire[63:0] live_nonce={cfg['h34>>2],cfg['h30>>2]};
    wire[63:0] live_source_age={cfg['h5c>>2],cfg['h58>>2]};
    wire[63:0] live_stop_timeout={cfg['h6c>>2],cfg['h68>>2]};
    integer i;

    function automatic logic[63:0] sh64(input integer offset);
        sh64={shadow[(offset>>2)+1],shadow[offset>>2]};
    endfunction
    function automatic logic[63:0] cf64(input integer offset);
        cf64={cfg[(offset>>2)+1],cfg[offset>>2]};
    endfunction
    function automatic logic[31:0] crc_word(input logic[31:0] crc,input logic[31:0] data);
        logic[31:0] c; integer b;
        begin
            c=crc;
            for(b=0;b<32;b=b+1) if(c[0]^data[b]) c=(c>>1)^32'hedb88320; else c=c>>1;
            crc_word=c;
        end
    endfunction
    function automatic integer crc_offset(input integer index);
        if(index<17) crc_offset='h30+index*4;
        else if(index<129) crc_offset='h100+(index-17)*4;
        else crc_offset='h460+(index-129)*4;
    endfunction
    function automatic logic is_static(input integer addr);
        is_static=(addr>='h30&&addr<='h70)||(addr>='h100&&addr<'h2c0)||(addr>='h460&&addr<'h468);
    endfunction
    function automatic logic is_shadow(input integer addr);
        integer rel;
        begin
            rel=(addr-'h100)%64;
            is_shadow=(addr>='h30&&addr<='h70)||(addr>='h80&&addr<'ha0)||
                (addr>='h100&&addr<'h2c0&&rel<48)||
                (addr>='h400&&addr<='h448)||(addr=='h460)||(addr=='h464);
        end
    endfunction
    function automatic logic signed[127:0] sext64(input logic[63:0] value);
        sext64={{64{value[63]}},value};
    endfunction
    function automatic logic signed[127:0] ceil_shift(input logic signed[127:0] value,input integer bits);
        ceil_shift=-((-value)>>>bits);
    endfunction
    function automatic logic static_valid;
        logic[95:0] t,mg;
        logic okay;
        begin
            // Same complete96 operands/values, now registered BEFORE compare.
            t=cfg_rate_threshold;
            mg=cfg_rate_max_gap;
            okay=sh64('h30)!=0&&sh64('h30)!=last_nonce&&shadow['h38>>2]!=0&&shadow['h3c>>2]==0&&
                sh64('h40)!=0&&sh64('h48)!=0&&t[95:64]==0&&sh64('h40)<=(t[63:0]>>8)&&t<=mg&&
                sh64('h50)>0&&sh64('h50)<=64'h7fffffffffffffff&&sh64('h58)>0&&sh64('h58)<=64'h7fffffffffffffff&&
                sh64('h60)<=64'hffffffff&&sh64('h68)>0&&sh64('h68)<=64'h7fffffffffffffff&&
                shadow['h460>>2]==32'h10001&&shadow['h464>>2]>0&&shadow['h464>>2]<=65535;
            // A previous CONFIG's products never satisfy a fresh admission.
            if(cfg_rate_fresh!==1'b1||cfg_rate_num_snapshot!==sh64('h40)||
               cfg_rate_den_snapshot!==sh64('h48)||cfg_rate_nonce_snapshot!==sh64('h30)||
               cfg_rate_epoch_snapshot!==shadow['h38>>2]||cfg_rate_crc_snapshot!==shadow['h70>>2]) okay=0;
            // Only a certificate produced for this accepted CONFIG may pass.
            if(cfg_seed_done!==1'b1||seed_bounds_valid!==1'b1||
               cfg_seed_epoch!==shadow['h38>>2]||cfg_seed_nonce!==sh64('h30)||
               cfg_seed_crc!==shadow['h70>>2]) okay=0;
            for(integer a=0;a<7;a=a+1) begin
                for(integer k=2;k<6;k=k+1) if($signed(sh64('h100+a*64+k*8))<=0) okay=0;
                for(integer k=12;k<16;k=k+1) if(shadow[('h100+a*64)/4+k]!=0) okay=0;
            end
            static_valid=okay;
        end
    endfunction

    assign b_clk=aclk;
    assign b_rst=!aresetn;
    assign b_din=32'd0;
    assign b_we=4'd0;
    assign disable_req=1'b1;
    assign motion_permit=1'b0;
    assign dbg_tick=dds_tick;
    assign s_axi_cfg_awready=!aw_saved&&!s_axi_cfg_bvalid&&(cmd_state==C_IDLE);
    assign s_axi_cfg_wready=!w_saved&&!s_axi_cfg_bvalid&&(cmd_state==C_IDLE);
    assign s_axi_cfg_arready=!s_axi_cfg_rvalid;
    assign val_resetn=aresetn&&(cmd_state!=C_DISARM);
    assign math_start=pipe_active&&(pipe_age==3);
    assign seed_seq_resetn=aresetn&&(cmd_state!=C_DISARM)&&!hard_sync&&(state!=FAULT);
    always_comb begin
        for(integer a=0;a<7;a=a+1) begin
            lower[64*a+:64]=cf64('h100+a*64);upper[64*a+:64]=cf64('h108+a*64);
            seed_seq_lower[64*a+:64]=(seed_owner==S_CONFIG)?sh64('h100+a*64):lower[64*a+:64];
            seed_seq_upper[64*a+:64]=(seed_owner==S_CONFIG)?sh64('h108+a*64):upper[64*a+:64];
            seed_seq_vmax[64*a+:64]=(seed_owner==S_CONFIG)?sh64('h110+a*64):cf64('h110+a*64);
            seed_seq_amax[64*a+:64]=(seed_owner==S_CONFIG)?sh64('h118+a*64):cf64('h118+a*64);
            seed_seq_jmax[64*a+:64]=(seed_owner==S_CONFIG)?sh64('h120+a*64):cf64('h120+a*64);
            seed_seq_dq[64*a+:64]=(seed_owner==S_CONFIG)?sh64('h128+a*64):cf64('h128+a*64);
            velocity_limit[64*a+:64]=cf64('h110+a*64);acceleration_limit[64*a+:64]=cf64('h118+a*64);
            jerk_limit[64*a+:64]=cf64('h120+a*64);dq_max[64*a+:64]=cf64('h128+a*64);
            seed_q[64*a+:64]=sh64('h410+a*8);
        end
        bank_writable={(bank_state[1]==FREE),(bank_state[0]==FREE)};
        parent_seq=0;parent_cycle=0;parent_tick=0;parent_q=seed_q_tag;parent_v=0;parent_a=0;parent_wire=seed_wire;
        if(bank_state[!v_bank]==READY||bank_state[!v_bank]==ACTIVE) begin
            parent_seq=bseq[!v_bank];parent_cycle=bendcycle[!v_bank];parent_tick=bendtick[!v_bank];
            parent_q=bendq[!v_bank];parent_v=bendv[!v_bank];parent_a=benda[!v_bank];parent_wire=bendwire[!v_bank];
        end
        pf_req=pf_active&&(pf_issued<64);
        pf_addr={pf_bank,16'd0}+17'd256+({9'd0,pf_record_index}<<8)+({10'd0,pf_issued}<<2);
        pf_grant=pf_req;v_grant=v_req&&!pf_req;
        b_en=pf_req||v_req;b_addr=pf_req?pf_addr:v_addr;
        pf_valid=rd_pending&&rd_owner;v_valid=rd_pending&&!rd_owner;
    end
    always_ff @(posedge aclk) begin
        if(!aresetn) begin rd_pending<=0;rd_owner<=0;rd_word<=0;end
        else begin rd_pending<=b_en;rd_owner<=pf_req;rd_word<=pf_issued;end
    end
    tick_gen #(.CLOCK_HZ(CLOCK_HZ)) dds(.clk(aclk),.resetn(aresetn),.config_commit(tick_commit),.enable(configured),
        .rate_num(live_rate_num),.rate_den(live_rate_den),.tick(dds_tick),.tick_index(tick_index),
        .cycle(cycle),.origin(origin),.config_error(dds_error),.quiet_window(dds_quiet));
    // Cache preparation starts with the original DDS commit edge. Its extra
    // configuration response latency does not move DDS origin or source time.
    limits_product_cache #(.CLOCK_HZ(CLOCK_HZ)) config_products(.clk(aclk),.resetn(aresetn),.start(tick_commit),
        .cfg_epoch(cfg['h38>>2]),.lower(lower),.upper(upper),.dq_max(dq_max),
        .velocity_limit(velocity_limit),.acceleration_limit(acceleration_limit),
        .busy(cache_busy),.done(cache_done),.cache_valid(cache_valid),.rejected(),.cached_cfg_epoch(cached_cfg_epoch),
        .cached_lower_qden(cached_lower_qden),.cached_upper_qden(cached_upper_qden),.cached_dq_qden(cached_dq_qden),
        .cached_vmax_vden(cached_vmax_vden),.cached_amax_c(cached_amax_c));
    stationary_seed_gate_seq #(.CLOCK_HZ(CLOCK_HZ)) seed_encoder(.clk(aclk),.resetn(seed_seq_resetn),.start(seed_seq_start),
        .seed_q(seed_q),.lower(seed_seq_lower),.upper(seed_seq_upper),.velocity_limit(seed_seq_vmax),
        .acceleration_limit(seed_seq_amax),.jerk_limit(seed_seq_jmax),.dq_max(seed_seq_dq),
        .busy(seed_seq_busy),.done(seed_seq_done),.allow(seed_seq_allow),.clamp(seed_seq_clamp),
        .bounds_valid(seed_seq_bounds),.q16(seed_seq_q16),.wire_values(seed_seq_wire));
    bank_validator #(.CLOCK_HZ(CLOCK_HZ),.EXTERNAL_EXECUTION(1)) validator(.clk(aclk),.resetn(val_resetn),.start(v_start),.bank(v_bank),
        .now_cycle(cycle),.rate_origin(origin),.rate_num(live_rate_num),.rate_den(live_rate_den),
        .expected_sequence(pending_sequence),.last_accepted_sequence(last_accepted),.session_nonce(live_nonce),
        .source_max_age(live_source_age),.stop_timeout(live_stop_timeout),.config_epoch(cfg['h38>>2]),
        .config_crc(cfg['h70>>2]),.max_map_error(cfg['h60>>2]),.expected_previous(parent_seq),
        .parent_end_cycle(parent_cycle),.parent_end_tick(parent_tick),.parent_q(parent_q),.parent_v(parent_v),.parent_a(parent_a),.parent_wire(parent_wire),
        .lower(lower),.upper(upper),.velocity_limit(velocity_limit),.acceleration_limit(acceleration_limit),.jerk_limit(jerk_limit),.dq_max(dq_max),
        .cache_valid(cache_valid),.cached_config_epoch(cached_cfg_epoch),
        .cached_lower_qden(cached_lower_qden),.cached_upper_qden(cached_upper_qden),.cached_dq_qden(cached_dq_qden),
        .cached_vmax_vden(cached_vmax_vden),.cached_amax_c(cached_amax_c),
        .mem_req(v_req),.mem_addr(v_addr),.mem_grant(v_grant),.mem_valid(v_valid),.mem_data(b_dout),
        .busy(v_busy),.done(v_done),.accepted(v_accept),.reject_code(v_reject),.bank_sequence(vs),.previous_sequence(vprev),
        .normal_count(vn),.brake_count(vb),.first_cycle(vfirstcycle),.first_tick(vfirsttick),.normal_end_cycle(vendcycle),.normal_end_tick(vendtick),
        .brake_end_cycle(vbrakeend),.observation_cycle(vobs),.produced_cycle(vprod),.normal_deadline(vdeadline),.map_error(verror),
        .first_q(vfirstq),.first_v(vfirstv),.first_a(vfirsta),.normal_end_q(vendq),.normal_end_v(vendv),.normal_end_a(venda),.normal_end_wire(vendwire),
        .exec_math_req(vx_math_req),.exec_gate_req(vx_gate_req),.exec_dt_cycles(vx_dt),
        .exec_q0(vx_q0),.exec_v0(vx_v0),.exec_a0(vx_a0),.exec_j(vx_j),
        .exec_lower(vx_lower),.exec_upper(vx_upper),.exec_velocity_limit(vx_vmax),
        .exec_acceleration_limit(vx_amax),.exec_jerk_limit(vx_jmax),.exec_dq_max(vx_dq),
        .exec_previous_wire(vx_previous_wire),.exec_cache_valid(vx_cache_valid),
        .exec_math_grant(vx_math_grant),.exec_math_done(vx_math_done),.exec_math_overflow(vx_math_overflow),
        .exec_q1(vx_q1),.exec_v1(vx_v1),.exec_a1(vx_a1),
        .exec_gate_grant(vx_gate_grant),.exec_gate_done(vx_gate_done),.exec_gate_allow(vx_allow),.exec_gate_clamp(vx_clamp),
        .exec_q16(vx_q16),.exec_wire(vx_wire));
    // A pending registered DDS tick and the full112-clock runtime window
    // reserve the engine. Wide DDS lookahead independently protects the next tick.
    shared_math_gate #(.CLOCK_HZ(CLOCK_HZ),.STABLE_CACHE_INPUTS(1)) shared_execution(
        .clk(aclk),.resetn(val_resetn),.rt_cancel(!pipe_active),.val_cancel(!val_resetn),
        .val_slot_admit(configured&&dds_quiet&&!dds_tick&&!pipe_active),
        .rt_math_req(math_start),
        .rt_gate_req(pipe_active&&math_done&&!math_overflow),
        .rt_dt_cycles(pipe_dt),
        .rt_q0(pipe_q),
        .rt_v0(pipe_v),
        .rt_a0(pipe_a),
        .rt_j(pipe_j),
        .rt_lower(lower),
        .rt_upper(upper),
        .rt_velocity_limit(velocity_limit),
        .rt_acceleration_limit(acceleration_limit),
        .rt_jerk_limit(jerk_limit),
        .rt_dq_max(dq_max),
        .rt_previous_wire(previous_wire),
        .rt_previous_wire_valid(1'b1),
        .rt_cache_valid(cache_valid&&(cached_cfg_epoch==cfg['h38>>2])),
        .rt_cached_lower_qden(cached_lower_qden),
        .rt_cached_upper_qden(cached_upper_qden),
        .rt_cached_dq_qden(cached_dq_qden),
        .rt_cached_vmax_vden(cached_vmax_vden),
        .rt_cached_amax_c(cached_amax_c),
        .rt_math_grant(),
        .rt_gate_grant(),
        .rt_math_busy(math_busy),
        .rt_math_done(math_done),
        .rt_math_overflow(math_overflow),
        .rt_q1(out_q),
        .rt_v1(out_v),
        .rt_a1(out_a),
        .rt_q1_latched(out_q_latched),
        .rt_v1_latched(out_v_latched),
        .rt_a1_latched(out_a_latched),
        .rt_gate_busy(gate_busy),
        .rt_gate_done(gate_done),
        .rt_allow(gate_allow),
        .rt_clamp(gate_clamp),
        .rt_q16(gate_q16),
        .rt_wire_values(gate_wire),
        .val_math_req(vx_math_req),
        .val_gate_req(vx_gate_req),
        .val_dt_cycles(vx_dt),
        .val_q0(vx_q0),
        .val_v0(vx_v0),
        .val_a0(vx_a0),
        .val_j(vx_j),
        .val_lower(vx_lower),
        .val_upper(vx_upper),
        .val_velocity_limit(vx_vmax),
        .val_acceleration_limit(vx_amax),
        .val_jerk_limit(vx_jmax),
        .val_dq_max(vx_dq),
        .val_previous_wire(vx_previous_wire),
        .val_previous_wire_valid(1'b1),
        .val_cache_valid(vx_cache_valid),
        .val_cached_lower_qden(cached_lower_qden),
        .val_cached_upper_qden(cached_upper_qden),
        .val_cached_dq_qden(cached_dq_qden),
        .val_cached_vmax_vden(cached_vmax_vden),
        .val_cached_amax_c(cached_amax_c),
        .val_math_grant(vx_math_grant),
        .val_gate_grant(vx_gate_grant),
        .val_math_busy(),
        .val_math_done(vx_math_done),
        .val_math_overflow(vx_math_overflow),
        .val_q1(vx_q1),
        .val_v1(vx_v1),
        .val_a1(vx_a1),
        .val_gate_busy(),
        .val_gate_done(vx_gate_done),
        .val_allow(vx_allow),
        .val_clamp(vx_clamp),
        .val_q16(vx_q16),
        .val_wire_values(vx_wire),
        .busy(),.owner_is_validator(),.owner_age(),
        .runtime_collision(shared_collision),.protocol_fault(shared_protocol_fault));

    always_comb begin : encode_descriptor
        logic[31:0] flags;
        logic[63:0] payload;
        flags=0;if(pipe_mode==RAMP)flags[0]=1;if(pipe_mode==HOLD)flags[1]=1;if(gate_clamp)flags[2]=1;
        packed_desc=0;packed_desc[0+:64]=emit_count+1;packed_desc[64+:64]=pipe_cycle;
        packed_desc[128+:64]=pipe_tick;packed_desc[192+:32]=cfg['h38>>2];packed_desc[224+:32]=flags;
        for(integer frame=0;frame<4;frame=frame+1) begin
            packed_desc[(32+frame*16)*8+:16]=(frame==0)?16'h155:(frame==1)?16'h156:(frame==2)?16'h157:16'h159;
            packed_desc[(34+frame*16)*8+:8]=8'd8;
            payload=0;
            if(frame<3) begin
                for(integer k=0;k<4;k=k+1) begin
                    payload[k*8+:8]=gate_wire[(frame*2)*32+24-k*8+:8];
                    payload[(k+4)*8+:8]=gate_wire[(frame*2+1)*32+24-k*8+:8];
                end
            end else begin
                for(integer k=0;k<4;k=k+1) payload[k*8+:8]=gate_wire[6*32+24-k*8+:8];
                payload[32+:8]=cfg['h464>>2][15:8];payload[40+:8]=cfg['h464>>2][7:0];
                payload[48+:8]=cfg['h460>>2][7:0];payload[56+:8]=0;
            end
            packed_desc[(36+frame*16)*8+:64]=payload;
        end
    end

    // The control process below owns all session/bank/committed state. A fault
    // wins after command processing and never commits rejected candidate state.
    always_ff @(posedge aclk) begin : control
        logic[3:0] requested_fault;
        logic[31:0] command,read_value;
        logic read_ok,bus_bad,disarm_now,stop_now,stop_expired;
        logic arm_checked_bank,arm_guards_bad,arm_encoding_bad;
        logic[63:0] now_edge,record_start,record_end,record_tick,record_kind;
        logic[127:0] time_wide;
        logic[31:0] crc_data;
        integer addr,read_addr,which,offset,word,shadow_write_slot;
        if(!aresetn) begin
            for(i=0;i<SHADOW_WORDS;i=i+1) shadow_words[i]<=0;
            for(i=0;i<1024;i=i+1) cfg[i]<=0;
            for(i=0;i<2;i=i+1) begin
                bank_state[i]<=FREE;bseq[i]<=0;bprev[i]<=0;bfirstcycle[i]<=0;bfirsttick[i]<=0;bendcycle[i]<=0;bendtick[i]<=0;bbrakeend[i]<=0;bdeadline[i]<=0;berror[i]<=0;
                bnormal[i]<=0;bbrake[i]<=0;bfirstq[i]<=0;bfirstv[i]<=0;bfirsta[i]<=0;bendq[i]<=0;bendv[i]<=0;benda[i]<=0;bendwire[i]<=0;
            end
            for(i=0;i<11;i=i+1)snapshots[i]<=0;
            state<=DISARMED;configured<=0;tick_commit<=0;fault_code<=0;reject_code<=0;fault_cycle<=0;
            last_nonce<=0;last_accepted<=0;last_kick_seq<=0;last_kick_cycle<=0;kick_valid<=0;stop_pending<=0;stop_started<=0;
            hard_meta<=0;hard_sync<=0;cmd_state<=C_IDLE;cmd_bank<=0;settle_count<=0;pending_sequence<=0;
            cfg_crc_acc<=32'hffffffff;cfg_crc_index<=0;aw_saved<=0;w_saved<=0;aw_address<=0;w_value<=0;w_strobes<=0;
            s_axi_cfg_bvalid<=0;s_axi_cfg_bresp<=0;s_axi_cfg_rvalid<=0;s_axi_cfg_rresp<=0;s_axi_cfg_rdata<=0;
            v_start<=0;v_bank<=0;pf_active<=0;pf_bank<=0;pf_record_index<=0;pf_issued<=0;pf_received<=0;cur_record<=0;cur_valid<=0;cur_bank<=0;cur_record_index<=0;active_bank<=0;
            committed_q<=0;committed_v<=0;committed_a<=0;previous_wire<=0;pipe_active<=0;pipe_age<=0;pipe_math_ready<=0;pipe_gate_ready<=0;pipe_bad<=0;pipe_cycle<=0;pipe_tick<=0;pipe_dt<=0;pipe_q<=0;pipe_v<=0;pipe_a<=0;pipe_j<=0;pipe_mode<=DISARMED;
            tx_desc_valid<=0;tx_desc_data<=0;dbg_desc<=0;last_desc<=0;snap_desc<=0;
            emit_count<=0;reject_count<=0;clamp_count<=0;starve_count<=0;backpressure_count<=0;latency_min<=0;latency_max<=0;
            seed_seq_start<=0;seed_owner<=S_NONE;seed_age<=0;seed_q_tag<=0;
            cfg_rate_prepare<=0;cfg_rate_fresh<=0;
            cfg_rate_num_snapshot<=0;cfg_rate_den_snapshot<=0;cfg_rate_nonce_snapshot<=0;
            cfg_rate_epoch_snapshot<=0;cfg_rate_crc_snapshot<=0;
            cfg_rate_threshold<=0;cfg_rate_max_gap<=0;
            seed_epoch_tag<=0;seed_crc_tag<=0;seed_nonce_tag<=0;seed_cert_epoch<=0;seed_cert_nonce<=0;
            cfg_seed_done<=0;cfg_seed_epoch<=0;cfg_seed_nonce<=0;cfg_seed_crc<=0;
            seed_cert_done<=0;seed_allow<=0;seed_clamp<=0;seed_bounds_valid<=0;seed_q16<=0;seed_wire<=0;
            bank_needs_seed<=0;bank_parent_sequence<=0;
        end else begin
            requested_fault=0;now_edge=cycle+1;
            // One guard expression serves initial admission and final completion.
            // Final selection is the latched bank, never a newly READY alternative.
            arm_checked_bank=(cmd_state==C_ARM_CHECK||cmd_state==C_SEED_CRC)?cmd_bank:
                             ((bank_state[0]==READY&&bprev[0]==0)?1'b0:1'b1);
            time_wide={64'd0,sh64('h400)}+cf64('h58);
            arm_guards_bad=!configured||cache_valid!==1'b1||cached_cfg_epoch!==cfg['h38>>2]||fault_code!=0||state!=DISARMED||!kick_valid||bank_state[arm_checked_bank]!=READY||bprev[arm_checked_bank]!=0||
                sh64('h90)!=bfirsttick[arm_checked_bank]||bfirsttick[arm_checked_bank]<=tick_index||sh64('h408)==0||sh64('h400)>now_edge||time_wide<=now_edge||time_wide>128'hffffffffffffffff||
                bfirstq[arm_checked_bank]!==seed_q||bfirstv[arm_checked_bank]!==448'd0||bfirsta[arm_checked_bank]!==448'd0||
                {64'd0,now_edge}+berror[arm_checked_bank]>=bdeadline[arm_checked_bank]||
                {1'b0,now_edge}>={1'b0,last_kick_cycle}+{1'b0,cf64('h50)}||
                hard_sync||cycle==64'hffffffffffffffff||dds_error||shared_collision||shared_protocol_fault;
            // Initial admission checks every non-encoding guard immediately;
            // only a NEW snapshot may satisfy encoding at final completion.
            arm_encoding_bad=seed_cert_done!==1'b1||seed_allow!==1'b1||seed_q_tag!==seed_q||
                seed_cert_epoch!==cfg['h38>>2]||seed_cert_nonce!==live_nonce;
            disarm_now=aw_saved&&w_saved&&!s_axi_cfg_bvalid&&cmd_state==C_IDLE&&aw_address=='h010&&w_value==128&&w_strobes==4'hf;
            stop_now=aw_saved&&w_saved&&!s_axi_cfg_bvalid&&cmd_state==C_IDLE&&aw_address=='h010&&w_value==4&&w_strobes==4'hf&&(state==RUN||state==ARMED)&&{64'd0,now_edge}+{64'd0,cf64('h68)}<=128'hffffffffffffffff;
            stop_expired=stop_pending&&(state==ARMED||state==RUN||state==RAMP)&&
                {1'b0,now_edge}>={1'b0,stop_started}+{1'b0,cf64('h68)}&&!disarm_now&&cmd_state!=C_DISARM;
            hard_meta<=hard_fault;hard_sync<=hard_meta;
            tick_commit<=0;v_start<=0;tx_desc_valid<=0;dbg_desc<=0;seed_seq_start<=0;
            if(cfg_rate_prepare)begin
                cfg_rate_threshold<={32'd0,cfg_rate_den_snapshot}*{64'd0,CLOCK_HZ[31:0]};
                cfg_rate_max_gap<=({32'd0,cfg_rate_num_snapshot}<<32)-{32'd0,cfg_rate_num_snapshot};
                cfg_rate_prepare<=0;cfg_rate_fresh<=1;
            end
            if(seed_owner!=S_NONE)begin
                if(seed_seq_done===1'b1)begin
                    if(seed_owner==S_CONFIG)begin
                        cfg_seed_done<=1;seed_bounds_valid<=seed_seq_bounds;
                        cfg_seed_epoch<=seed_epoch_tag;cfg_seed_nonce<=seed_nonce_tag;cfg_seed_crc<=seed_crc_tag;
                    end else begin
                        seed_cert_done<=1;seed_allow<=seed_seq_allow;seed_clamp<=seed_seq_clamp;
                        seed_q16<=seed_seq_q16;seed_wire<=seed_seq_wire;
                        seed_cert_epoch<=seed_epoch_tag;seed_cert_nonce<=seed_nonce_tag;
                    end
                    seed_owner<=S_NONE;seed_age<=0;
                end else if(seed_age==6'd63)begin
                    // Internal fixed-latency violation rejects; no grace to any
                    // source/lease/tick deadline and no fabricated certificate.
                    if(seed_owner==S_CONFIG)begin cfg_seed_done<=1;seed_bounds_valid<=0;end
                    else begin seed_cert_done<=1;seed_allow<=0;seed_clamp<=0;seed_q16<=0;seed_wire<=0;end
                    seed_owner<=S_NONE;
                end else seed_age<=seed_age+1;
            end
            if(s_axi_cfg_bvalid&&s_axi_cfg_bready)s_axi_cfg_bvalid<=0;
            if(s_axi_cfg_rvalid&&s_axi_cfg_rready)s_axi_cfg_rvalid<=0;
            if(s_axi_cfg_awvalid&&s_axi_cfg_awready)begin aw_saved<=1;aw_address<=s_axi_cfg_awaddr;end
            if(s_axi_cfg_wvalid&&s_axi_cfg_wready)begin w_saved<=1;w_value<=s_axi_cfg_wdata;w_strobes<=s_axi_cfg_wstrb;end
            if(s_axi_cfg_arvalid&&s_axi_cfg_arready) begin
                read_addr=s_axi_cfg_araddr;read_value=0;read_ok=1;
                case(read_addr)
                    'h000:read_value=32'h5254504c;'h004:read_value=32'h00010000;'h008:read_value=1;'h00c:read_value=state;
                    'h014:read_value={23'd0,configured,1'b0,bank_state[1],1'b0,bank_state[0]};
                    'h018:read_value=fault_code;'h01c:read_value=reject_code;
                    'h020:read_value=cycle[31:0];'h024:read_value=cycle[63:32];'h028:read_value=tick_index[31:0];'h02c:read_value=tick_index[63:32];
                    'h0a0:read_value=origin[31:0];'h0a4:read_value=origin[63:32];
                    'h0a8:read_value=CLOCK_HZ[31:0];'h0ac:read_value=32'd0;
                    default:begin
                        if(is_shadow(read_addr))read_value=shadow[read_addr>>2];
                        else if(read_addr>='h500&&read_addr<'h558) begin offset=read_addr-'h500;read_value=snapshots[offset/8][(offset%8)*8+:32];end
                        else if(read_addr>='h600&&read_addr<'h660)read_value=snap_desc[(read_addr-'h600)*8+:32];
                        else read_ok=0;
                    end
                endcase
                if(read_addr%4!=0)read_ok=0;
                s_axi_cfg_rdata<=read_ok?read_value:32'd0;s_axi_cfg_rresp<=read_ok?2'b00:2'b10;s_axi_cfg_rvalid<=1;
            end
            if(aw_saved&&w_saved&&!s_axi_cfg_bvalid&&cmd_state==C_IDLE)begin
                aw_saved<=0;w_saved<=0;s_axi_cfg_bvalid<=1;s_axi_cfg_bresp<=0;addr=aw_address;
                if(addr%4!=0) s_axi_cfg_bresp<=2'b10;
                else if(addr=='h010)begin
                    command=w_value;
                    if(w_strobes!=4'hf||command==0||(command&(command-1))!=0) s_axi_cfg_bresp<=2'b10;
                    else case(command)
                        1:if(state!=DISARMED||bank_state[0]!=FREE||bank_state[1]!=FREE||fault_code!=0||hard_sync||seed_owner!=S_NONE||seed_seq_busy) s_axi_cfg_bresp<=2'b10;
                          else begin
                            configured<=0;cmd_state<=C_CRC;cfg_crc_acc<=32'hffffffff;cfg_crc_index<=0;s_axi_cfg_bvalid<=0;
                            cfg_rate_prepare<=1;cfg_rate_fresh<=0;
                            cfg_rate_num_snapshot<=sh64('h40);cfg_rate_den_snapshot<=sh64('h48);
                            cfg_rate_nonce_snapshot<=sh64('h30);cfg_rate_epoch_snapshot<=shadow['h38>>2];cfg_rate_crc_snapshot<=shadow['h70>>2];
                            seed_seq_start<=1;seed_owner<=S_CONFIG;seed_age<=0;seed_q_tag<=seed_q;
                            seed_epoch_tag<=shadow['h38>>2];seed_nonce_tag<=sh64('h30);seed_crc_tag<=shadow['h70>>2];
                            cfg_seed_done<=0;seed_bounds_valid<=0;seed_cert_done<=0;
                          end
                        2:begin
                            // Every original non-CRC guard is checked now too;
                            // an initially future/invalid source cannot become
                            // eligible merely by waiting through the CRC pass.
                            if(arm_guards_bad!==1'b0||seed_owner!=S_NONE||seed_seq_busy) s_axi_cfg_bresp<=2'b10;
                            else begin
                                cmd_bank<=arm_checked_bank;cfg_crc_acc<=32'hffffffff;cfg_crc_index<=0;
                                cmd_state<=C_SEED_CRC;s_axi_cfg_bvalid<=0;
                                seed_seq_start<=1;seed_owner<=S_ARM;seed_age<=0;seed_q_tag<=seed_q;
                                seed_epoch_tag<=cfg['h38>>2];seed_nonce_tag<=live_nonce;seed_crc_tag<=cfg['h70>>2];
                                seed_cert_done<=0;seed_allow<=0;seed_clamp<=0;seed_q16<=0;seed_wire<=0;
                            end
                        end
                        4:if((state==RUN||state==ARMED)&&{64'd0,now_edge}+{64'd0,cf64('h68)}<=128'hffffffffffffffff)begin stop_pending<=1;if(!stop_pending)stop_started<=now_edge;end else s_axi_cfg_bresp<=2'b10;
                        8:if(state==DISARMED&&bank_state[0]==FREE&&bank_state[1]==FREE&&!hard_sync)begin fault_code<=0;fault_cycle<=0;end else s_axi_cfg_bresp<=2'b10;
                        16:begin
                            snapshots[0]<=cycle;snapshots[1]<=tick_index;snapshots[2]<=emit_count;snapshots[3]<=reject_count;snapshots[4]<=clamp_count;snapshots[5]<=starve_count;snapshots[6]<=backpressure_count;
                            snapshots[7]<=latency_min;snapshots[8]<=latency_max;snapshots[9]<=emit_count;snapshots[10]<=fault_cycle;snap_desc<=last_desc;
                        end
                        32,64:begin
                            which=command==64;
                            if(!configured||cache_valid!==1'b1||cached_cfg_epoch!==cfg['h38>>2]||bank_state[which]!=FREE||v_busy||v_start||fault_code!=0||hard_sync||seed_owner!=S_NONE||seed_seq_busy)begin reject_code<=1;s_axi_cfg_bresp<=2'b10;end
                            else begin
                                bank_state[which]<=VALIDATING;cmd_bank<=which;v_bank<=which;pending_sequence<=sh64(which?'h88:'h80);cmd_state<=C_BANK_WAIT;s_axi_cfg_bvalid<=0;
                                bank_needs_seed<=!(bank_state[!which]==READY||bank_state[!which]==ACTIVE);
                                bank_parent_sequence<=(bank_state[!which]==READY||bank_state[!which]==ACTIVE)?bseq[!which]:64'd0;
                                if(!(bank_state[!which]==READY||bank_state[!which]==ACTIVE))begin
                                    seed_seq_start<=1;seed_owner<=S_BANK;seed_age<=0;seed_q_tag<=seed_q;
                                    seed_epoch_tag<=cfg['h38>>2];seed_nonce_tag<=live_nonce;seed_crc_tag<=cfg['h70>>2];
                                    seed_cert_done<=0;seed_allow<=0;seed_clamp<=0;seed_q16<=0;seed_wire<=0;
                                end
                            end
                        end
                        128:begin configured<=0;pipe_active<=0;cur_valid<=0;pf_active<=0;state<=DISARMED;cmd_state<=C_DISARM;s_axi_cfg_bvalid<=0;
                            seed_owner<=S_NONE;seed_seq_start<=0;seed_cert_done<=0;cfg_seed_done<=0;seed_allow<=0;seed_bounds_valid<=0;
                            cfg_rate_prepare<=0;cfg_rate_fresh<=0;
                        end
                        256:if(!configured||sh64('h98)<=last_kick_seq||{64'd0,now_edge}+{64'd0,cf64('h50)}>128'hffffffffffffffff) s_axi_cfg_bresp<=2'b10;
                            else begin last_kick_seq<=sh64('h98);last_kick_cycle<=now_edge;kick_valid<=1;end
                        512,1024:begin
                            which=command==1024;
                            if(bank_state[which]==REJECTED||(state==DISARMED&&bank_state[which]==READY))bank_state[which]<=FREE;
                            else s_axi_cfg_bresp<=2'b10;
                        end
                        default:s_axi_cfg_bresp<=2'b10;
                    endcase
                end else if(!is_shadow(addr)||(is_static(addr)&&configured))s_axi_cfg_bresp<=2'b10;
                else begin
                    shadow_write_slot=shadow_slot(addr);
                    // Defensive range guard: invalid -1 is never an array index.
                    if(shadow_write_slot<0 || shadow_write_slot>=SHADOW_WORDS) s_axi_cfg_bresp<=2'b10;
                    else for(integer b=0;b<4;b=b+1)
                        if(w_strobes[b])shadow_words[shadow_write_slot][b*8+:8]<=w_value[b*8+:8];
                end
            end
            case(cmd_state)
                C_CRC,C_SEED_CRC:begin
                    // One shared word CRC expression, not two independently
                    // unrolled instances. CONFIG's canonical sequence is unchanged.
                    word=(cmd_state==C_CRC)?(crc_offset(cfg_crc_index)>>2):(('h400>>2)+cfg_crc_index);
                    crc_data=(cmd_state==C_CRC&&word==('h70>>2))?0:shadow[word];
                    cfg_crc_acc<=crc_word(cfg_crc_acc,crc_data);
                    if(cmd_state==C_CRC&&cfg_crc_index==130)cmd_state<=C_CHECK;
                    else if(cmd_state==C_SEED_CRC&&cfg_crc_index==17)cmd_state<=C_ARM_CHECK;
                    else cfg_crc_index<=cfg_crc_index+1;
                end
                C_ARM_CHECK:begin
                    // CRC finished E18, encoding normally publishes E36. The
                    // final CURRENT edge rechecks every original ARM predicate.
                    which=cmd_bank;
                    if(arm_guards_bad!==1'b0)begin cmd_state<=C_IDLE;s_axi_cfg_bvalid<=1;s_axi_cfg_bresp<=2'b10;end
                    else if(seed_cert_done===1'b1)begin
                        cmd_state<=C_IDLE;s_axi_cfg_bvalid<=1;s_axi_cfg_bresp<=0;
                        if(arm_encoding_bad!==1'b0||(~cfg_crc_acc)!==shadow['h448>>2])s_axi_cfg_bresp<=2'b10;
                        else begin
                            state<=ARMED;active_bank<=which;bank_state[which]<=ACTIVE;committed_q<=seed_q;committed_v<=0;committed_a<=0;previous_wire<=seed_wire;
                            pf_active<=1;pf_bank<=which;pf_record_index<=0;pf_issued<=0;pf_received<=0;cur_valid<=0;stop_pending<=0;
                        end
                    end else if(seed_owner!=S_ARM)begin cmd_state<=C_IDLE;s_axi_cfg_bvalid<=1;s_axi_cfg_bresp<=2'b10;end
                end
                C_CHECK:begin
                    if(hard_sync||fault_code!=0||state!=DISARMED||cycle==64'hffffffffffffffff||shared_collision||shared_protocol_fault)begin
                        reject_code<=4;s_axi_cfg_bresp<=2'b10;s_axi_cfg_bvalid<=1;cmd_state<=C_IDLE;
                        cfg_rate_prepare<=0;cfg_rate_fresh<=0;
                    end else if(cfg_seed_done!==1'b1&&seed_owner==S_CONFIG)begin end
                    else if((~cfg_crc_acc)!==shadow['h70>>2]||static_valid()!==1'b1)begin
                        reject_code<=4;s_axi_cfg_bresp<=2'b10;s_axi_cfg_bvalid<=1;cmd_state<=C_IDLE;
                        cfg_rate_prepare<=0;cfg_rate_fresh<=0;
                    end else begin
                        cfg_rate_prepare<=0;cfg_rate_fresh<=0;
                        for(integer w=0;w<1024;w=w+1)if(is_static(w*4))cfg[w]<=shadow[w];
                        tick_commit<=1;last_nonce<=sh64('h30);last_accepted<=0;last_kick_seq<=0;kick_valid<=0;emit_count<=0;reject_count<=0;clamp_count<=0;starve_count<=0;backpressure_count<=0;latency_min<=0;latency_max<=0;last_desc<=0;snap_desc<=0;
                        cmd_state<=C_ACTIVATE;
                    end
                end
                C_ACTIVATE:if(cache_done)begin
                    cmd_state<=C_IDLE;s_axi_cfg_bvalid<=1;
                    if(cache_valid===1'b1&&cached_cfg_epoch===cfg['h38>>2])begin configured<=1;s_axi_cfg_bresp<=0;end
                    else begin configured<=0;reject_code<=4;s_axi_cfg_bresp<=2'b10;end
                end
                C_BANK_WAIT:if(!write_busy[cmd_bank])begin settle_count<=0;cmd_state<=C_BANK_SETTLE;end
                C_BANK_SETTLE:if(write_busy[cmd_bank])cmd_state<=C_BANK_WAIT;
                    else if(settle_count==1)begin
                        if(fault_code!=0||hard_sync||!configured||cache_valid!==1'b1||cached_cfg_epoch!==cfg['h38>>2])begin
                            bank_state[cmd_bank]<=REJECTED;reject_code<=4;reject_count<=reject_count+1;
                            cmd_state<=C_IDLE;s_axi_cfg_bvalid<=1;s_axi_cfg_bresp<=2'b10;
                        end else if(bank_needs_seed)cmd_state<=C_BANK_SEED_WAIT;
                        else if(parent_seq!==bank_parent_sequence||parent_seq==0)begin
                            // A parent that retired during drain is not replaced
                            // by an old seed result. Conservative requeue only.
                            bank_state[cmd_bank]<=REJECTED;reject_code<=8;reject_count<=reject_count+1;
                            cmd_state<=C_IDLE;s_axi_cfg_bvalid<=1;s_axi_cfg_bresp<=2'b10;
                        end else begin v_start<=1;cmd_state<=C_IDLE;s_axi_cfg_bvalid<=1;s_axi_cfg_bresp<=0;end
                    end else settle_count<=settle_count+1;
                C_BANK_SEED_WAIT:begin
                    if(fault_code!=0||hard_sync||!configured||cache_valid!==1'b1||cached_cfg_epoch!==cfg['h38>>2]||
                       (seed_owner!=S_BANK&&seed_cert_done!==1'b1))begin
                        bank_state[cmd_bank]<=REJECTED;reject_code<=4;reject_count<=reject_count+1;
                        cmd_state<=C_IDLE;s_axi_cfg_bvalid<=1;s_axi_cfg_bresp<=2'b10;
                    end else if(seed_cert_done===1'b1)begin
                        cmd_state<=C_IDLE;s_axi_cfg_bvalid<=1;s_axi_cfg_bresp<=0;
                        if(seed_allow!==1'b1||seed_q_tag!==seed_q||seed_cert_epoch!==cfg['h38>>2]||
                           seed_cert_nonce!==live_nonce||parent_seq!==64'd0)begin
                            bank_state[cmd_bank]<=REJECTED;reject_code<=7;reject_count<=reject_count+1;s_axi_cfg_bresp<=2'b10;
                        end else v_start<=1;
                    end
                end
                C_DISARM:if(!rd_pending&&write_busy==0)begin
                    bank_state[0]<=FREE;bank_state[1]<=FREE;kick_valid<=0;stop_pending<=0;cmd_state<=C_IDLE;s_axi_cfg_bvalid<=1;s_axi_cfg_bresp<=0;
                end
                default:;
            endcase
            if(v_done&&cmd_state!=C_DISARM)begin
                if(!v_accept)begin bank_state[v_bank]<=REJECTED;reject_code<=v_reject;reject_count<=reject_count+1;end
                else begin
                    bank_state[v_bank]<=READY;last_accepted<=vs;bseq[v_bank]<=vs;bprev[v_bank]<=vprev;bnormal[v_bank]<=vn;bbrake[v_bank]<=vb;
                    bfirstcycle[v_bank]<=vfirstcycle;bfirsttick[v_bank]<=vfirsttick;bendcycle[v_bank]<=vendcycle;bendtick[v_bank]<=vendtick;bbrakeend[v_bank]<=vbrakeend;bdeadline[v_bank]<=vdeadline;berror[v_bank]<=verror;
                    bfirstq[v_bank]<=vfirstq;bfirstv[v_bank]<=vfirstv;bfirsta[v_bank]<=vfirsta;bendq[v_bank]<=vendq;bendv[v_bank]<=vendv;benda[v_bank]<=venda;bendwire[v_bank]<=vendwire;
                end
            end
            if(pf_grant)pf_issued<=pf_issued+1;
            if(pf_valid&&pf_active&&!disarm_now&&cmd_state!=C_DISARM)begin
                cur_record[rd_word*32+:32]<=b_dout;pf_received<=pf_received+1;
                if(rd_word==63)begin
                    cur_valid<=1;cur_bank<=pf_bank;cur_record_index<=pf_record_index;pf_active<=0;
                end
            end
            // Priority request calculation uses this physical edge's free cycle.
            if(hard_sync)requested_fault=1;
            else if((state==ARMED||state==RUN||state==RAMP||state==HOLD)&&(!kick_valid||{1'b0,now_edge}>={1'b0,last_kick_cycle}+{1'b0,cf64('h50)}))requested_fault=2;
            else if(cycle==64'hffffffffffffffff)requested_fault=3;
            else if(configured&&dds_error)requested_fault=4;
            else if(shared_collision||shared_protocol_fault)requested_fault=10;
            if(dds_tick&&(state==ARMED||state==RUN||state==RAMP||state==HOLD)&&requested_fault==0&&!disarm_now&&cmd_state!=C_DISARM)begin
                if(pipe_active)requested_fault=10;
                else if(state==HOLD)begin
                    pipe_active<=1;pipe_age<=1;pipe_math_ready<=0;pipe_gate_ready<=0;pipe_bad<=0;pipe_cycle<=cycle;pipe_tick<=tick_index;pipe_dt<=cycle-pipe_cycle;
                    pipe_q<=committed_q;pipe_v<=0;pipe_a<=0;pipe_j<=0;pipe_mode<=HOLD;
                end else if(state==ARMED&&tick_index<bfirsttick[active_bank])begin end
                else if(!cur_valid||cur_bank!=active_bank)requested_fault=5;
                else begin
                    record_start=cur_record[0+:64];record_end=cur_record[64+:64];record_tick=cur_record[128+:64];record_kind=cur_record[192+:32];
                    if(record_tick!=tick_index||record_end!=cycle||record_end<=record_start||record_end-record_start>64'hffffffff)requested_fault=5;
                    else if(record_kind==1&&{64'd0,record_end}+128'd112+berror[active_bank]>=bdeadline[active_bank])requested_fault=6;
                    else begin
                        pipe_active<=1;pipe_age<=1;pipe_math_ready<=0;pipe_gate_ready<=0;pipe_bad<=0;pipe_cycle<=cycle;pipe_tick<=tick_index;pipe_dt<=record_end-record_start;
                        for(integer a=0;a<7;a=a+1)begin pipe_q[64*a+:64]<=cur_record[(32+a*32)*8+:64];pipe_v[64*a+:64]<=cur_record[(40+a*32)*8+:64];pipe_a[64*a+:64]<=cur_record[(48+a*32)*8+:64];pipe_j[64*a+:64]<=cur_record[(56+a*32)*8+:64];end
                        pipe_mode<=record_kind==2?RAMP:RUN;state<=record_kind==2?RAMP:RUN;cur_valid<=0;
                        if(cur_record_index==bnormal[active_bank]-1)begin
                            which=!active_bank;
                            if(!(stop_pending||stop_now)&&bank_state[which]==READY&&bprev[which]==bseq[active_bank]&&{64'd0,cycle}+berror[which]<bdeadline[which])begin
                                bank_state[active_bank]<=FREE;bank_state[which]<=ACTIVE;active_bank<=which;pf_bank<=which;pf_record_index<=0;
                            end else begin
                                pf_bank<=active_bank;pf_record_index<=cur_record_index+1;if(!(stop_pending||stop_now))starve_count<=starve_count+1;state<=RAMP;
                            end
                            pf_active<=1;pf_issued<=0;pf_received<=0;
                        end else if(cur_record_index+1<bnormal[active_bank]+bbrake[active_bank])begin
                            pf_active<=1;pf_bank<=active_bank;pf_record_index<=cur_record_index+1;pf_issued<=0;pf_received<=0;
                        end
                    end
                end
            end
            if(pipe_active&&!disarm_now&&cmd_state!=C_DISARM)begin
                pipe_age<=pipe_age+1;
                if(math_done)begin pipe_math_ready<=1;pipe_bad<=math_overflow;end
                if(gate_done)pipe_gate_ready<=1;
                if(pipe_age==111)begin
                    pipe_active<=0;
                    if(requested_fault==0)begin
                        if(pipe_math_ready!==1'b1||pipe_gate_ready!==1'b1||pipe_bad!==1'b0||gate_allow!==1'b1||cache_valid!==1'b1||cached_cfg_epoch!==cfg['h38>>2])requested_fault=4;
                        else if(!backend_ready)begin requested_fault=7;backpressure_count<=backpressure_count+1;end
                        else if(pipe_q!=committed_q||pipe_v!=committed_v||pipe_a!=committed_a)requested_fault=8;
                        else if(pipe_mode==RAMP&&pipe_cycle==bbrakeend[active_bank]&&(out_v_latched!=0||out_a_latched!=0))requested_fault=8;
                        else if(stop_expired)requested_fault=9;
                        else if(emit_count==64'hffffffffffffffff)requested_fault=10;
                        else begin
                            tx_desc_data<=packed_desc;tx_desc_valid<=1;dbg_desc<=1;last_desc<=packed_desc;emit_count<=emit_count+1;
                            if(emit_count==0||now_edge-pipe_cycle<latency_min)latency_min<=now_edge-pipe_cycle;
                            if(emit_count==0||now_edge-pipe_cycle>latency_max)latency_max<=now_edge-pipe_cycle;
                            committed_q<=out_q_latched;committed_v<=out_v_latched;committed_a<=out_a_latched;previous_wire<=gate_wire;
                            if(gate_clamp)clamp_count<=clamp_count+1;
                            if(pipe_mode==RAMP&&pipe_cycle==bbrakeend[active_bank])state<=HOLD;
                        end
                    end
                end
            end
            // Resolve the lower-priority timeout after arithmetic/record/source
            // faults, but block candidate state above before any commit.
            if(stop_expired&&(requested_fault==0||requested_fault>9))requested_fault=9;
            if(requested_fault!=0)begin
                if(fault_code==0)begin fault_code<=requested_fault;fault_cycle<= (cycle == 64'hffffffffffffffff) ? cycle : now_edge;end
                state<=FAULT;pipe_active<=0;cur_valid<=0;pf_active<=0;tx_desc_valid<=0;dbg_desc<=0;
                seed_seq_start<=0;seed_owner<=S_NONE;seed_cert_done<=0;cfg_seed_done<=0;seed_allow<=0;seed_bounds_valid<=0;
                cfg_rate_prepare<=0;cfg_rate_fresh<=0;
            end
        end
    end
endmodule

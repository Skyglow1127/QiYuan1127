`timescale 1ns/1ps
module tb_top_protocol #(parameter integer CLOCK_HZ=100000000);
    logic aclk=0;always #5 aclk=~aclk;
    logic aresetn=0;
    logic[11:0] s_axi_cfg_awaddr=0,s_axi_cfg_araddr=0;
    logic[2:0] s_axi_cfg_awprot=0,s_axi_cfg_arprot=0;
    logic s_axi_cfg_awvalid=0,s_axi_cfg_wvalid=0,s_axi_cfg_bready=1,s_axi_cfg_arvalid=0,s_axi_cfg_rready=1;
    wire s_axi_cfg_awready,s_axi_cfg_wready,s_axi_cfg_bvalid,s_axi_cfg_arready,s_axi_cfg_rvalid;
    logic[31:0] s_axi_cfg_wdata=0;logic[3:0]s_axi_cfg_wstrb=15;
    wire[31:0]s_axi_cfg_rdata;wire[1:0]s_axi_cfg_bresp,s_axi_cfg_rresp;
    wire[16:0] b_addr;wire b_clk,b_rst,b_en;wire[31:0]b_din;wire[3:0]b_we;logic[31:0]b_dout=0;
    wire[1:0]bank_writable;logic[1:0]write_busy=0;
    logic hard_fault=0,backend_ready=1;
    wire disable_req,motion_permit,dbg_tick,dbg_desc,tx_desc_valid;wire[767:0]tx_desc_data;
    robot_rt_top #(.CLOCK_HZ(CLOCK_HZ)) dut(.*);
    // PRIVATE STABLE_JOB_INPUTS=1 producer contract monitor, all full-width data.
    wire [24863:0] stable_gate_data;
    assign stable_gate_data={
        dut.shared_execution.gate.q0,dut.shared_execution.gate.v0,dut.shared_execution.gate.a0,
        dut.shared_execution.gate.j,dut.shared_execution.gate.q1,dut.shared_execution.gate.v1,
        dut.shared_execution.gate.a1,dut.shared_execution.gate.lower,dut.shared_execution.gate.upper,
        dut.shared_execution.gate.velocity_limit,dut.shared_execution.gate.acceleration_limit,dut.shared_execution.gate.jerk_limit,
        dut.shared_execution.gate.dq_max,dut.shared_execution.gate.previous_wire,dut.shared_execution.gate.qp0_terms,
        dut.shared_execution.gate.qp1_terms,dut.shared_execution.gate.qp2_terms,dut.shared_execution.gate.qp3_terms,
        dut.shared_execution.gate.vp0_terms,dut.shared_execution.gate.vp1_terms,dut.shared_execution.gate.vp2_terms,
        dut.shared_execution.gate.ap0_terms,dut.shared_execution.gate.ap1_terms,dut.shared_execution.gate.cached_lower_qden,
        dut.shared_execution.gate.cached_upper_qden,dut.shared_execution.gate.cached_dq_qden,dut.shared_execution.gate.cached_vmax_vden,
        dut.shared_execution.gate.cached_amax_c
    };
    logic stable_gate_active=0;
    logic [24863:0] stable_gate_snapshot;
    integer stable_gate_jobs=0,stable_gate_edges=0,stable_gate_completed=0;
    integer stable_gate_cancel_edges=0,stable_gate_reset_jobs=0;
    initial begin
        if(dut.shared_execution.gate.STABLE_JOB_INPUTS!==1)
            $fatal(1,"PRIVATE top monitor requires STABLE_JOB_INPUTS=1");
    end

    logic density_mon_active=0;
    logic [6719:0] density_mon_products;
    logic [31:0] density_mon_epoch;
    logic density_mon_valid;
    integer density_mon_edges=0,density_mon_jobs=0;

    logic[31:0]mem[0:32767],settings[0:1023];
    // Independent architectural address oracle: explicit regmap RW_SHADOW
    // words. Never call DUT is_shadow/shadow_slot or copy its range formula.
    localparam logic [11:0] SHADOW_ADDR [0:129] = '{
        12'h030, 12'h034, 12'h038, 12'h03c, 12'h040, 12'h044, 12'h048, 12'h04c,
        12'h050, 12'h054, 12'h058, 12'h05c, 12'h060, 12'h064, 12'h068, 12'h06c,
        12'h070, 12'h080, 12'h084, 12'h088, 12'h08c, 12'h090, 12'h094, 12'h098,
        12'h09c, 12'h100, 12'h104, 12'h108, 12'h10c, 12'h110, 12'h114, 12'h118,
        12'h11c, 12'h120, 12'h124, 12'h128, 12'h12c, 12'h140, 12'h144, 12'h148,
        12'h14c, 12'h150, 12'h154, 12'h158, 12'h15c, 12'h160, 12'h164, 12'h168,
        12'h16c, 12'h180, 12'h184, 12'h188, 12'h18c, 12'h190, 12'h194, 12'h198,
        12'h19c, 12'h1a0, 12'h1a4, 12'h1a8, 12'h1ac, 12'h1c0, 12'h1c4, 12'h1c8,
        12'h1cc, 12'h1d0, 12'h1d4, 12'h1d8, 12'h1dc, 12'h1e0, 12'h1e4, 12'h1e8,
        12'h1ec, 12'h200, 12'h204, 12'h208, 12'h20c, 12'h210, 12'h214, 12'h218,
        12'h21c, 12'h220, 12'h224, 12'h228, 12'h22c, 12'h240, 12'h244, 12'h248,
        12'h24c, 12'h250, 12'h254, 12'h258, 12'h25c, 12'h260, 12'h264, 12'h268,
        12'h26c, 12'h280, 12'h284, 12'h288, 12'h28c, 12'h290, 12'h294, 12'h298,
        12'h29c, 12'h2a0, 12'h2a4, 12'h2a8, 12'h2ac, 12'h400, 12'h404, 12'h408,
        12'h40c, 12'h410, 12'h414, 12'h418, 12'h41c, 12'h420, 12'h424, 12'h428,
        12'h42c, 12'h430, 12'h434, 12'h438, 12'h43c, 12'h440, 12'h444, 12'h448,
        12'h460, 12'h464
    };
    logic [31:0] expected_shadow [0:129];
    logic shadow_axi_only=0;
    integer checks=0,descriptors=0;
    logic[63:0] bench_edges=0,last_dbg_edge=0,last_dbg_index=0;
    logic[63:0] origin_value;
    logic[63:0] cache_origin_expected;
    logic[63:0] case_numerator=CLOCK_HZ;
    integer shared_val_jobs=0,shared_val_jobs_running=0,shared_wait_cycles=0;
    logic shared_val_inflight=0;
    time shared_val_e0;
    localparam logic[63:0] FIRST=64;
    always_ff @(posedge aclk)if(b_en&&!b_rst)b_dout<=mem[b_addr>>2];

    task automatic check(input logic condition,input string message);
        begin checks=checks+1;if(condition!==1'b1)$fatal(1,"%s cycle=%0d state=%0d fault=%0d reject=%0d",message,dut.cycle,dut.state,dut.fault_code,dut.reject_code);end
    endtask
    task automatic reset_core;
        begin
            @(negedge aclk);aresetn=0;s_axi_cfg_awvalid=0;s_axi_cfg_wvalid=0;s_axi_cfg_arvalid=0;write_busy=0;hard_fault=0;backend_ready=1;
            repeat(4)@(posedge aclk);#1;
            check(disable_req&&!motion_permit&&!tx_desc_valid,"reset must remain DESC_ONLY disabled");
            @(negedge aclk);aresetn=1;descriptors=0;
        end
    endtask
    task automatic wr(input logic[11:0] address,input logic[31:0] value,input logic[1:0] expected,input logic[3:0] strobes=4'hf);
        begin
            @(negedge aclk);s_axi_cfg_awaddr=address;s_axi_cfg_wdata=value;s_axi_cfg_wstrb=strobes;s_axi_cfg_awvalid=1;s_axi_cfg_wvalid=1;
            do @(posedge aclk);while(!(s_axi_cfg_awready&&s_axi_cfg_wready));
            @(negedge aclk);s_axi_cfg_awvalid=0;s_axi_cfg_wvalid=0;
            do begin @(posedge aclk);#1;end while(!s_axi_cfg_bvalid);
            check(s_axi_cfg_bresp==expected,$sformatf("AXI write response addr=%h value=%h actual=%h expected=%h",address,value,s_axi_cfg_bresp,expected));
            @(negedge aclk);
        end
    endtask
    task automatic rd(input logic[11:0] address,output logic[31:0] value);
        begin
            @(negedge aclk);s_axi_cfg_araddr=address;s_axi_cfg_arvalid=1;
            do @(posedge aclk);while(!s_axi_cfg_arready);
            @(negedge aclk);s_axi_cfg_arvalid=0;
            while(!s_axi_cfg_rvalid)begin @(posedge aclk);#1;end
            check(s_axi_cfg_rresp==0,"AXI read response");value=s_axi_cfg_rdata;
            @(negedge aclk);
        end
    endtask
    function automatic logic listed_shadow(input logic [11:0] address);
        begin
            listed_shadow=0;
            for(integer n=0;n<130;n=n+1)if(address==SHADOW_ADDR[n])listed_shadow=1;
        end
    endfunction
    task automatic shadow_reset_view_zero;
        begin
            for(integer word_index=0;word_index<1024;word_index=word_index+1)
                check(dut.shadow[word_index]===32'd0,$sformatf("reset full1024 shadow view word=%0d",word_index));
        end
    endtask
    task automatic shadow_readback_all(input string phase);
        logic [31:0] actual;
        begin
            for(integer n=0;n<130;n=n+1)begin
                rd(SHADOW_ADDR[n],actual);
                check(actual===expected_shadow[n],$sformatf("%s AXI shadow/alias addr=%h actual=%h expected=%h",phase,SHADOW_ADDR[n],actual,expected_shadow[n]));
            end
        end
    endtask
    task automatic shadow_axi_exhaustive;
        logic [31:0] payload,actual;
        logic [3:0] mask;
        integer rejected;
        begin
            shadow_axi_only=1;
            shadow_reset_view_zero();
            for(integer n=0;n<130;n=n+1)begin
                for(integer prior=0;prior<n;prior=prior+1)
                    check(SHADOW_ADDR[n]!=SHADOW_ADDR[prior],"independent address list has no duplicates");
                expected_shadow[n]=32'h91e30000^(32'h01010101*(n+1));
                wr(SHADOW_ADDR[n],expected_shadow[n],0);
            end
            shadow_readback_all("full-word distinct pattern");
            for(integer byte_index=0;byte_index<4;byte_index=byte_index+1)begin
                mask=4'b0001<<byte_index;
                for(integer n=0;n<130;n=n+1)begin
                    payload=32'h6b94d20f^(32'h01020409*(n+1))^(32'h10101010*byte_index);
                    wr(SHADOW_ADDR[n],payload,0,mask);
                    expected_shadow[n][byte_index*8+:8]=payload[byte_index*8+:8];
                    rd(SHADOW_ADDR[n],actual);
                    check(actual===expected_shadow[n],$sformatf("single-byte WSTRB addr=%h mask=%h actual=%h expected=%h",SHADOW_ADDR[n],mask,actual,expected_shadow[n]));
                end
                shadow_readback_all($sformatf("cross-register after byte%0d",byte_index));
            end
            for(integer n=0;n<130;n=n+1)begin
                wr(SHADOW_ADDR[n],~expected_shadow[n],0,4'b0000);
                rd(SHADOW_ADDR[n],actual);
                check(actual===expected_shadow[n],$sformatf("zero WSTRB cannot change addr=%h",SHADOW_ADDR[n]));
            end
            shadow_readback_all("zero-mask cross-register");
            rejected=0;
            // Entire aligned 12-bit aperture. Do not write command0x010:
            // the following full legacy scenarios retain all command tests.
            for(integer address=0;address<4096;address=address+4)begin
                if(address!=12'h010&&!listed_shadow(address[11:0]))begin
                    wr(address[11:0],32'hd7a50000^address,2);
                    rejected=rejected+1;
                    // Every rejected bus write must preserve all130 words,
                    // using bus reads rather than sparse-storage internals.
                    shadow_readback_all($sformatf("rejected write%h",address[11:0]));
                end
            end
            check(rejected==893,"all893 nonshadow/noncommand aligned writes rejected");
            shadow_axi_only=0;
            reset_core();
            shadow_reset_view_zero();
            $display("PASS shadow_axi_exhaustive legal_words=130 invalid_words=893 command_writes=0");
        end
    endtask
    task automatic skew_write;
        begin
            @(negedge aclk);s_axi_cfg_awaddr='h40;s_axi_cfg_awvalid=1;
            do @(posedge aclk);while(!s_axi_cfg_awready);
            @(negedge aclk);s_axi_cfg_awvalid=0;
            repeat(3)begin @(posedge aclk);#1;check(!s_axi_cfg_bvalid,"AW alone cannot complete a write");end
            @(negedge aclk);s_axi_cfg_wdata=123;s_axi_cfg_wvalid=1;
            do @(posedge aclk);while(!s_axi_cfg_wready);
            @(negedge aclk);s_axi_cfg_wvalid=0;
            do begin @(posedge aclk);#1;end while(!s_axi_cfg_bvalid);
            check(s_axi_cfg_bresp==0&&dut.shadow['h40/4]==123,"independent AW/W channel capture");
            @(negedge aclk);
        end
    endtask
    function automatic logic[31:0] crc_word(input logic[31:0] crc,input logic[31:0] data);
        logic[31:0]c;
        begin c=crc;for(integer b=0;b<32;b=b+1)if(c[0]^data[b])c=(c>>1)^32'hedb88320;else c=c>>1;crc_word=c;end
    endfunction
    task automatic put64(input integer address,input logic[63:0] value);
        begin settings[address/4]=value[31:0];settings[address/4+1]=value[63:32];end
    endtask
    task automatic mput64(input integer address,input logic[63:0] value);
        begin mem[address/4]=value[31:0];mem[address/4+1]=value[63:32];end
    endtask
    // Independent analytical edge oracle, never reads the DUT phase logic.
    function automatic logic[63:0] tick_offset(input logic[63:0] index);
        logic[127:0] n,d,product;
        begin
            n={64'd0,settings['h44/4],settings['h40/4]};
            d={64'd0,settings['h4c/4],settings['h48/4]};
            product={64'd0,index}*d*{96'd0,CLOCK_HZ[31:0]};
            tick_offset=(product+n-1)/n;
        end
    endfunction
    task automatic initialize_settings(input logic[63:0] denominator,input logic[63:0] lease);
        logic[31:0]c;
        begin
            for(integer w=0;w<1024;w=w+1)settings[w]=0;
            put64('h30,1);settings['h38/4]=1;put64('h40,case_numerator);put64('h48,denominator);
            put64('h50,lease);put64('h58,1000000);put64('h60,0);put64('h68,1000000);
            for(integer a=0;a<7;a=a+1)begin
                put64('h100+a*64,a==6?64'd0:-64'd42949672960);
                put64('h108+a*64,a==6?64'd429496729:64'd42949672960);
                put64('h110+a*64,64'd42949672960);put64('h118+a*64,64'd42949672960);put64('h120+a*64,64'd42949672960);put64('h128+a*64,64'd4294967296);
                put64('h410+a*8,a==6?64'd214748365:64'd0);
            end
            put64('h400,0);put64('h408,1);settings['h460/4]='h10001;settings['h464/4]=1000;
            c='1;for(integer w='h400/4;w<'h448/4;w=w+1)c=crc_word(c,settings[w]);settings['h448/4]=~c;
            c='1;for(integer w='h30/4;w<'h74/4;w=w+1)c=crc_word(c,settings[w]);
            for(integer w='h100/4;w<'h2c0/4;w=w+1)c=crc_word(c,settings[w]);
            for(integer w='h460/4;w<'h468/4;w=w+1)c=crc_word(c,settings[w]);settings['h70/4]=~c;
        end
    endtask
    task automatic write_settings;
        begin
            for(integer address='h30;address<='h70;address=address+4)wr(address,settings[address/4],0);
            for(integer a=0;a<7;a=a+1)for(integer w=0;w<12;w=w+1)wr('h100+a*64+w*4,settings[('h100+a*64)/4+w],0);
            for(integer address='h400;address<='h448;address=address+4)wr(address,settings[address/4],0);
            wr('h460,settings['h460/4],0);wr('h464,settings['h464/4],0);
        end
    endtask
    task automatic read_origin;
        logic[31:0]lo,hi;
        begin
            rd('ha0,lo);rd('ha4,hi);origin_value={hi,lo};check(origin_value!=0,"origin is global commit cycle");
            check(dut.dds.numerator===case_numerator&&dut.dds.threshold===({settings['h4c/4],settings['h48/4]}*{32'd0,CLOCK_HZ[31:0]}),"DDS receives exact committed rate, without unknown bits");
            check(!$isunknown({dut.lower,dut.upper,dut.velocity_limit,dut.acceleration_limit,dut.jerk_limit,dut.dq_max,dut.seed_q,dut.seed_wire}),"configured limits and seed must be known numeric values");
            check(dut.cache_valid&&dut.cached_cfg_epoch==dut.cfg['h38/4],"configuration acknowledgement requires complete matching PL cache");
            check(origin_value==cache_origin_expected,"cache preparation must not retime original DDS origin");
        end
    endtask
    task automatic bank_image(input logic early_expiry,input logic corrupt_payload);
        logic[31:0]c;
        begin
            for(integer w=0;w<16384;w=w+1)mem[w]=0;
            mem[0]='h4b425452;mem[1]=1;mem[2]=256;mem[3]=256;
            mput64(16,1);mput64(24,1);mem[8]=1;mem[9]=2;mem[10]=2;
            mput64(48,origin_value);mput64(56,origin_value);
            mput64(64,early_expiry?origin_value+tick_offset(FIRST)+100:origin_value+1000000);
            mput64(72,FIRST);mput64(80,origin_value+tick_offset(FIRST-1));mput64(88,0);mem[26]=settings['h70/4];
            for(integer r=0;r<4;r=r+1)begin
                mput64(256+r*256,origin_value+tick_offset(FIRST+r-1));
                mput64(264+r*256,origin_value+tick_offset(FIRST+r));mput64(272+r*256,FIRST+r);mem[(280+r*256)/4]=r<2?1:2;
                mput64(256+r*256+32+6*32,214748365);
            end
            c='1;for(integer w=64;w<320;w=w+1)c=crc_word(c,mem[w]);mem[24]=~c;
            c='1;for(integer w=0;w<64;w=w+1)c=crc_word(c,(w==25)?0:mem[w]);mem[25]=~c;
            if(corrupt_payload)mem[(256+3*256+56)/4]=1;
        end
    endtask
    task automatic wait_bank(input logic[2:0] expected,input logic[3:0] reason);
        integer waited;
        begin waited=0;while(dut.bank_state[0]==1&&waited<10000)begin @(posedge aclk);#1;waited=waited+1;end
            check(dut.bank_state[0]==expected,"bank validation result");if(reason!=0)check(dut.reject_code==reason,"bank rejection code");
        end
    endtask
    task automatic successor_image;
        logic[31:0] c;
        integer base;
        begin
            base=16384;
            for(integer w=0;w<16384;w=w+1)mem[base+w]=mem[w];
            mem[base+4]=2;mem[base+5]=0;mem[base+22]=1;mem[base+23]=0;
            mem[base+18]=FIRST+2;mem[base+19]=0;
            {mem[base+21],mem[base+20]}=origin_value+tick_offset(FIRST+1);
            for(integer r=0;r<4;r=r+1)begin
                {mem[base+(256+r*256)/4+1],mem[base+(256+r*256)/4]}=origin_value+tick_offset(FIRST+r+1);
                {mem[base+(264+r*256)/4+1],mem[base+(264+r*256)/4]}=origin_value+tick_offset(FIRST+r+2);
                {mem[base+(272+r*256)/4+1],mem[base+(272+r*256)/4]}=FIRST+r+2;
            end
            c='1;for(integer w=64;w<320;w=w+1)c=crc_word(c,mem[base+w]);mem[base+24]=~c;mem[base+25]=0;
            c='1;for(integer w=0;w<64;w=w+1)c=crc_word(c,mem[base+w]);mem[base+25]=~c;
        end
    endtask
    // PRIVATE additive tests; original body and112 assertions are retained.
    localparam integer SQ_NORMAL=1,SQ_STALE=2,SQ_LEASE=3,SQ_SOURCE=4,SQ_HARD=5,SQ_RESET=6,SQ_CRC=7;
    bit sq_suite=0,sq_aw_seen=0,sq_w_seen=0,sq_dispatched=0,sq_watch=0;
    logic[11:0]sq_aw;
    logic[31:0]sq_w;
    integer sq_mode=SQ_NORMAL,sq_kind=0,sq_phase=0,sq_cases=0,sq_cfg_bounds=1;
    logic[63:0]sq_clock=0,sq_p0=0,sq_kick=0,sq_first=0,sq_observation=0,sq_deadline=0;
    bit sq_p0_seen=0,sq_cycle_read;
    // P0 derives from external AW/W handshakes, not DUT command state/age.
    always @(posedge aclk)begin : seq_external_monitor
        if(!aresetn)begin
            sq_clock=0;sq_aw_seen=0;sq_w_seen=0;sq_dispatched=0;sq_watch=0;sq_p0_seen=0;sq_kick=0;
        end else begin
            sq_clock=sq_clock+1;
            sq_cycle_read=s_axi_cfg_arvalid&&s_axi_cfg_arready&&s_axi_cfg_araddr=='h20;
            if(s_axi_cfg_bvalid&&s_axi_cfg_bready)begin sq_aw_seen=0;sq_w_seen=0;sq_dispatched=0;end
            if(sq_aw_seen&&sq_w_seen&&!sq_dispatched)begin
                sq_dispatched=1;
                if(sq_aw=='h10&&sq_w==256)sq_kick=sq_clock;
                if(sq_suite&&sq_aw=='h10&&(sq_w==1||sq_w==32||sq_w==2))begin
                    sq_watch=1;sq_kind=sq_w;sq_p0=sq_clock;
                    if(sq_kind==2)begin
                        sq_p0_seen=1;
                        check({settings['h404/4],settings['h400/4]}<=sq_clock,"ARM seed not future at external P0");
                        check(sq_clock<({settings['h404/4],settings['h400/4]}+{settings['h5c/4],settings['h58/4]}),"ARM source valid at external P0");
                        check(sq_kick!=0&&sq_clock<sq_kick+{settings['h54/4],settings['h50/4]},"ARM lease valid at external P0");
                        check(sq_clock<sq_deadline&&sq_deadline<=sq_observation+{settings['h5c/4],settings['h58/4]},"ARM bank deadline and source window initially valid");
                        check(origin_value+tick_offset(sq_first)>sq_clock+128,"ARM first tick initially has prefetch lead");
                        check({settings['h414/4],settings['h410/4]}==64'd4294967296&&{mem[73],mem[72]}==64'd4294967296,"independent seed and first-record q seam");
                        if(sq_mode==SQ_LEASE)check(sq_kick+32-sq_clock>0&&sq_kick+32-sq_clock<39,"lease really expires inside wait");
                        if(sq_mode==SQ_SOURCE)check(65536-sq_clock>0&&65536-sq_clock<39,"source really expires inside wait");
                    end
                end
            end
            if(s_axi_cfg_awvalid&&s_axi_cfg_awready)begin sq_aw_seen=1;sq_aw=s_axi_cfg_awaddr;end
            if(s_axi_cfg_wvalid&&s_axi_cfg_wready)begin sq_w_seen=1;sq_w=s_axi_cfg_wdata;end
            #1;
            if(sq_suite&&sq_cycle_read)check(s_axi_cfg_rvalid&&s_axi_cfg_rdata==sq_clock-1,"AXI CYCLE read matches independent reset-edge count");
            if(sq_watch)begin
                sq_phase=sq_clock-sq_p0;
                if(sq_phase==0||sq_phase==1||sq_phase==37||sq_phase==38||sq_phase==39)
                    $display("SEQ_PHASE kind=%0d mode=%0d P%0d cycle=%0d busy=%b done=%b cert=%b BVALID=%b",
                        sq_kind,sq_mode,sq_phase,sq_clock,dut.seed_seq_busy,dut.seed_seq_done,dut.seed_cert_done,s_axi_cfg_bvalid);
                if(sq_phase==0)begin
                    check(dut.seed_seq_start&&!s_axi_cfg_bvalid,"P0 launches fresh service without response");
                    if(sq_kind==1)check(dut.seed_owner==1&&!dut.cfg_seed_done&&!dut.seed_bounds_valid,"CONFIG clears prior bounds certificate");
                    else check(!dut.seed_cert_done&&!dut.seed_allow&&dut.seed_q16==0&&dut.seed_wire==0,"new encoding clears old allow/data");
                    if(sq_kind==2)check(dut.cmd_state==7&&dut.state==0,"ARM actually admitted into wait");
                end
                if(sq_phase==1)check(dut.seed_seq_busy&&!dut.seed_seq_done&&!s_axi_cfg_bvalid,"P1 snapshot accepted");
                if(sq_kind==1&&sq_phase<132)
                    check(!s_axi_cfg_bvalid&&!dut.configured&&!dut.tick_commit,"CONFIG cannot bypass original131-word CRC");
                if((sq_kind==32||(sq_kind==2&&(sq_mode==SQ_NORMAL||sq_mode==SQ_CRC)))&&sq_phase<39)begin
                    check(!s_axi_cfg_bvalid,"fresh certificate command cannot acknowledge before P39");
                    if(sq_kind==32)check(!dut.v_start,"first COMMIT cannot start validator with stale encoding");
                    else check(dut.state==0&&!dut.pf_active&&!tx_desc_valid,"ARM cannot publish side effects before fresh certificate");
                end
                if(sq_kind==2&&sq_mode==SQ_STALE&&sq_phase<64)
                    check(!s_axi_cfg_bvalid&&dut.state==0&&!dut.pf_active,"old allow cannot bypass missing-done watchdog");
                if(!s_axi_cfg_bvalid)
                    check(!s_axi_cfg_awready&&!s_axi_cfg_wready,"non-IDLE service freezes AXI writes");
                if((sq_kind!=2||sq_mode==SQ_NORMAL||sq_mode==SQ_CRC)&&sq_phase==37)
                    check(dut.seed_seq_done&&!dut.seed_seq_busy&&!s_axi_cfg_bvalid,"P37 service publishes before top consumes");
                if((sq_kind!=2||sq_mode==SQ_NORMAL||sq_mode==SQ_CRC)&&sq_phase==38)begin
                    check(!dut.seed_seq_done&&!s_axi_cfg_bvalid,"P38 NBA certificate cannot be consumed early");
                    if(sq_kind==1)check(dut.cfg_seed_done&&dut.seed_bounds_valid==sq_cfg_bounds,"fresh CONFIG bounds stored");
                    else check(dut.seed_cert_done&&dut.seed_allow&&dut.seed_q_tag[63:0]==64'd4294967296&&
                        dut.seed_wire[31:0]==1000&&dut.seed_wire[192+:32]==50003,"fresh q and wire certificate stored");
                end
                if(sq_phase==39)begin
                    if(sq_kind==1)check(!s_axi_cfg_bvalid&&!dut.configured,"CONFIG still waits original131 CRC words");
                    else if(sq_kind==32)check(s_axi_cfg_bvalid&&s_axi_cfg_bresp==0&&dut.v_start&&dut.parent_seq==0&&
                        dut.parent_q[63:0]==64'd4294967296&&dut.parent_wire[31:0]==1000,"P39 first COMMIT uses fresh parent");
                    else if(sq_mode==SQ_NORMAL)check(s_axi_cfg_bvalid&&s_axi_cfg_bresp==0&&dut.state==1&&
                        dut.previous_wire[31:0]==1000&&dut.previous_wire[192+:32]==50003,"P39 ARM uses fresh encoding");
                    else if(sq_mode==SQ_CRC)check(s_axi_cfg_bvalid&&s_axi_cfg_bresp==2&&dut.state==0&&!dut.pf_active,"bad CRC rejects after new encoding");
                    else if(sq_mode==SQ_STALE)check(!s_axi_cfg_bvalid&&!dut.seed_cert_done&&dut.state==0,"old allow cannot substitute new done");
                end
                if(s_axi_cfg_bvalid)sq_watch=0;
            end
        end
    end
    task automatic sq_crcs;
        logic[31:0]c;
        begin
            c='1;for(integer w='h400/4;w<'h448/4;w=w+1)c=crc_word(c,settings[w]);settings['h448/4]=~c;
            settings['h70/4]=0;c='1;
            for(integer w='h30/4;w<'h74/4;w=w+1)c=crc_word(c,settings[w]);
            for(integer w='h100/4;w<'h2c0/4;w=w+1)c=crc_word(c,settings[w]);
            for(integer w='h460/4;w<'h468/4;w=w+1)c=crc_word(c,settings[w]);settings['h70/4]=~c;
        end
    endtask
    task automatic sq_seed(input logic[63:0]cycle_value,input logic[63:0]sequence_value);
        begin
            put64('h410,64'd4294967296);put64('h400,cycle_value);put64('h408,sequence_value);sq_crcs();
            for(integer address='h400;address<='h448;address=address+4)wr(address,settings[address/4],0);
        end
    endtask
    task automatic sq_bank(input logic[63:0]first_tick,input logic[63:0]observation);
        logic[31:0]c;
        begin
            bank_image(0,0);sq_first=first_tick;sq_observation=observation;
            sq_deadline=observation+{settings['h5c/4],settings['h58/4]};
            mput64(48,observation);mput64(56,observation);mput64(64,sq_deadline);
            mput64(72,first_tick);mput64(80,origin_value+tick_offset(first_tick-1));
            for(integer r=0;r<4;r=r+1)begin
                mput64(256+r*256,origin_value+tick_offset(first_tick+r-1));
                mput64(264+r*256,origin_value+tick_offset(first_tick+r));mput64(272+r*256,first_tick+r);
                mput64(256+r*256+32,64'd4294967296);
            end
            c='1;for(integer w=64;w<320;w=w+1)c=crc_word(c,mem[w]);mem[24]=~c;mem[25]=0;
            c='1;for(integer w=0;w<64;w=w+1)c=crc_word(c,mem[w]);mem[25]=~c;
            check(origin_value+tick_offset(first_tick+1)+112<sq_deadline,"fixture normal outputs fit deadline before COMMIT");
            check(sq_observation<=sq_clock&&sq_clock<sq_deadline,"fixture is already produced before COMMIT");
        end
    endtask
    task automatic sq_prefix(input logic[63:0]lease,input logic[63:0]source_age,input bit late_first);
        logic[31:0]current;
        logic[63:0]first_tick;
        begin
            reset_core();case_numerator=CLOCK_HZ;sq_mode=SQ_NORMAL;sq_cfg_bounds=1;
            initialize_settings(256,lease);put64('h58,source_age);sq_crcs();write_settings();wr('h10,1,0);read_origin();
            if(late_first)repeat(2048)begin @(posedge aclk);#1;end
            sq_seed(0,11);first_tick=late_first?((source_age+512-origin_value+255)/256):FIRST;
            rd('h20,current);sq_bank(first_tick,current);
            wr('h80,1,0);wr('h84,0,0);wr('h10,32,0);wait_bank(2,0);
            check(dut.seed_cert_done&&dut.seed_allow&&dut.seed_wire[31:0]==1000,"first COMMIT completed new encoding");
            put64('h90,first_tick);wr('h90,settings['h90/4],0);wr('h94,settings['h94/4],0);
        end
    endtask
    task automatic sq_kick_now;
        begin put64('h98,1);wr('h98,1,0);wr('h9c,0,0);wr('h10,256,0);end
    endtask
    task automatic sq_arm_begin;
        begin
            sq_p0_seen=0;
            @(negedge aclk);s_axi_cfg_awaddr='h10;s_axi_cfg_wdata=2;s_axi_cfg_wstrb=15;s_axi_cfg_awvalid=1;s_axi_cfg_wvalid=1;
            do @(posedge aclk);while(!(s_axi_cfg_awready&&s_axi_cfg_wready));
            @(negedge aclk);s_axi_cfg_awvalid=0;s_axi_cfg_wvalid=0;
        end
    endtask
    task automatic sq_arm_finish(input logic[1:0]expected);
        integer waited;
        begin
            waited=0;do begin @(posedge aclk);#1;waited=waited+1;end while(!s_axi_cfg_bvalid&&waited<128);
            check(sq_p0_seen,"ARM waited after independently valid P0");check(s_axi_cfg_bvalid&&s_axi_cfg_bresp==expected,"bounded ARM response");
            @(negedge aclk);
        end
    endtask
    task automatic sq_no_ghost(input logic[31:0]fault_expected);
        logic[31:0]value;
        begin
            repeat(40)begin @(posedge aclk);#1;check(!tx_desc_valid&&!dut.pf_active&&dut.state!=1&&dut.state!=2,"no ghost ARM/prefetch/output");end
            rd('h18,value);check(value==fault_expected,"external fault cause after rejection/cancel");
            rd('h14,value);check((value&7)==2,"rejected ARM leaves READY bank immutable");
        end
    endtask
    task automatic seq_extra_scenarios;
        logic[31:0]v;
        logic[63:0]old_origin,seed_cycle;
        begin
            sq_suite=1;
            $display("STAGE SEQ CONFIG invalid seed and valid bounds");
            reset_core();initialize_settings(256,1000000);put64('h410,64'h8000000000000000);sq_crcs();write_settings();
            sq_cfg_bounds=1;wr('h10,1,0);read_origin();
            check(dut.configured&&dut.cfg_seed_done&&dut.seed_bounds_valid&&!dut.seed_seq_allow,"invalid seed cannot veto valid CONFIG bounds");sq_cases=sq_cases+1;
            $display("STAGE SEQ CONFIG unrepresentable Q16 bounds");
            reset_core();initialize_settings(256,1000000);put64('h100,1);put64('h108,65535);sq_crcs();write_settings();
            sq_cfg_bounds=0;wr('h10,1,2);check(!dut.configured&&!dut.cache_busy&&dut.origin==0&&dut.cfg['h38/4]==0,"bad bounds cannot partially CONFIG");sq_cases=sq_cases+1;
            $display("STAGE SEQ fresh COMMIT and rewritten-sequence ARM");
            sq_prefix(1000000,1000000,0);old_origin=origin_value;rd('h20,v);seed_cycle=v;
            sq_seed(seed_cycle,12);sq_kick_now();sq_mode=SQ_NORMAL;sq_arm_begin();sq_arm_finish(0);
            check(dut.seed_q_tag[63:0]==64'd4294967296&&dut.origin==old_origin,"fresh ARM retains q and original DDS origin");sq_cases=sq_cases+1;
            $display("STAGE SEQ bad seed CRC after fresh encoding");
            sq_prefix(1000000,1000000,0);rd('h20,v);sq_seed(v,12);wr('h448,settings['h448/4]^1,0);
            sq_kick_now();sq_mode=SQ_CRC;sq_arm_begin();sq_arm_finish(2);sq_no_ghost(0);sq_cases=sq_cases+1;
            $display("STAGE SEQ old allow cannot replace missing fresh done");
            sq_prefix(1000000,1000000,0);rd('h20,v);sq_seed(v,12);check(dut.seed_cert_done&&dut.seed_allow,"old actual COMMIT allow exists");
            sq_kick_now();sq_mode=SQ_STALE;force dut.seed_seq_done=1'b0;sq_arm_begin();sq_arm_finish(2);release dut.seed_seq_done;
            sq_no_ghost(0);sq_cases=sq_cases+1;
            $display("STAGE SEQ valid lease expires inside wait");
            sq_prefix(32,1000000,0);sq_kick_now();sq_mode=SQ_LEASE;sq_arm_begin();sq_arm_finish(2);
            check(sq_clock>=sq_kick+32&&sq_clock<1000000&&sq_clock<sq_deadline&&sq_clock<origin_value+tick_offset(sq_first),"only lease expired");
            sq_no_ghost(0);sq_cases=sq_cases+1;
            $display("STAGE SEQ valid source expires inside wait");
            sq_prefix(1000000,65536,1);old_origin=origin_value;
            while(sq_clock<65536-40)begin @(posedge aclk);#1;end
            sq_kick_now();sq_mode=SQ_SOURCE;sq_arm_begin();sq_arm_finish(2);
            check(sq_clock>=65536&&sq_clock<sq_kick+1000000&&sq_clock<sq_deadline&&
                sq_clock<origin_value+tick_offset(sq_first)&&dut.origin==old_origin,"only source expired without origin refresh");
            rd('h400,v);check(v==0,"expired seed timestamp not renewed");sq_no_ghost(0);sq_cases=sq_cases+1;
            $display("STAGE SEQ hardfault cancels busy encoding");
            sq_prefix(1000000,1000000,0);sq_kick_now();sq_mode=SQ_HARD;sq_arm_begin();wait(sq_p0_seen);
            repeat(10)@(negedge aclk);check(dut.seed_seq_busy,"hardfault injected during actual busy");hard_fault=1;sq_arm_finish(2);
            check(dut.seed_owner==0&&!dut.seed_cert_done&&!dut.seed_allow,"hardfault cancels ownership and validity");sq_no_ghost(1);sq_cases=sq_cases+1;
            $display("STAGE SEQ reset cancels busy ARM transaction");
            sq_prefix(1000000,1000000,0);sq_kick_now();sq_mode=SQ_RESET;sq_arm_begin();wait(sq_p0_seen);
            repeat(10)@(negedge aclk);check(dut.seed_seq_busy,"reset injected during actual busy");aresetn=0;
            repeat(4)@(posedge aclk);#1;
            check(!dut.seed_seq_busy&&!dut.seed_seq_done&&!dut.seed_cert_done&&!dut.seed_allow&&!s_axi_cfg_bvalid&&dut.state==0,"reset cancels outstanding transaction");
            @(negedge aclk);aresetn=1;
            repeat(40)begin @(posedge aclk);#1;check(!tx_desc_valid&&!dut.pf_active&&dut.state==0&&!dut.seed_seq_done,"no reset ghost event");end
            sq_cases=sq_cases+1;check(sq_cases==9,"all nine additive scenario groups reached");
            sq_suite=0;sq_mode=SQ_NORMAL;case_numerator=CLOCK_HZ;
            $display("PASS SEQ_TOP_INCREMENT_SCENARIOS cases=%0d; ORIGINAL FULL SUITE FOLLOWS",sq_cases);
        end
    endtask

    // Observe actual grant/response edges, independent of the shared owner's
    // internal age counter. A live validator transaction cannot hold the
    // engine for the128-clock DDS reservation horizon.
    always @(posedge aclk) begin
        if(!dut.val_resetn)shared_val_inflight=0;
        else begin
            if(dut.vx_math_grant)begin
                check(!shared_val_inflight,"one outstanding validator math job");
                shared_val_inflight=1;shared_val_e0=$time;
                shared_val_jobs=shared_val_jobs+1;
                if(dut.state>=1&&dut.state<=4)shared_val_jobs_running=shared_val_jobs_running+1;
                check(dut.dds_quiet&&!dut.dds_tick&&!dut.pipe_active,"grant excludes pending tick and full runtime reservation");
            end
            if(dut.vx_math_req&&!dut.vx_math_grant)shared_wait_cycles=shared_wait_cycles+1;
            if(shared_val_inflight)begin
                if(dut.vx_gate_done)begin
                    check($time-shared_val_e0==960,"validator gate consumed96 clocks after math E0");
                    shared_val_inflight=0;
                end else if(dut.vx_math_done&&dut.vx_math_overflow)shared_val_inflight=0;
                else check($time-shared_val_e0<=990,"validator whole execution stays below100 clocks");
            end
            check(!dut.shared_collision&&!dut.shared_protocol_fault,"shared engine cannot collide or stall during valid top flow");
        end
    end
    always @(posedge aclk)begin
        #1;
        bench_edges=bench_edges+1;
        if(aresetn)begin
            if(shadow_axi_only)check(!dut.configured&&dut.state==0&&dut.cmd_state==0&&!dut.tick_commit&&!dut.v_start&&!tx_desc_valid&&descriptors==0,"shadow AXI exercise cannot configure or execute commands");
            if(dut.tick_commit)cache_origin_expected=dut.cycle+1;
            if(dut.cache_busy)check(!dut.configured&&!s_axi_cfg_bvalid,"cache prepares before configuration response, without publishing ready");
            if(dbg_tick)begin last_dbg_edge=bench_edges;last_dbg_index=dut.tick_index;end
            check(disable_req&&!motion_permit,"DESC_ONLY cannot grant physical permit");
            if(dut.v_start)check(!$isunknown({dut.live_rate_num,dut.live_rate_den,dut.live_nonce,dut.live_source_age,dut.live_stop_timeout,dut.cfg['h38/4],dut.cfg['h70/4],dut.parent_seq,dut.parent_cycle,dut.parent_tick,dut.parent_q,dut.parent_v,dut.parent_a,dut.parent_wire}),"validator configuration and parent snapshot cannot contain unknown bits");
            if(dut.validator.state==9)check(!$isunknown({dut.validator.rec_start,dut.validator.rec_end,dut.validator.rec_tick,dut.validator.rq,dut.validator.rv,dut.validator.ra,dut.validator.rj,dut.validator.end_q,dut.validator.end_v,dut.validator.end_a,dut.validator.gate_allow,dut.validator.gate_wire}),"validated record, math endpoint and final gate cannot contain unknown bits");
            if(dut.v_done&&dut.v_accept)check(!$isunknown({dut.vs,dut.vprev,dut.vfirstcycle,dut.vfirsttick,dut.vendcycle,dut.vendtick,dut.vbrakeend,dut.vobs,dut.vprod,dut.vdeadline,dut.verror,dut.vn,dut.vb,dut.vfirstq,dut.vfirstv,dut.vfirsta,dut.vendq,dut.vendv,dut.venda,dut.vendwire}),"accepted bank metadata cannot contain unknown bits");
            if(tx_desc_valid)begin
                descriptors=descriptors+1;
                check(dut.cycle-tx_desc_data[64+:64]==112,"tick-to-descriptor exact112 clocks");
                check(dut.pipe_gate_ready&&dut.cache_valid&&dut.cached_cfg_epoch==dut.cfg['h38/4],"descriptor requires this job's completed gate and matching cache");
                check(tx_desc_data[64+:64]==origin_value+tick_offset(tx_desc_data[128+:64]),"descriptor timestamp agrees with independent rational DDS oracle");
                check(bench_edges-last_dbg_edge==112&&tx_desc_data[128+:64]==last_dbg_index,"external dbg_tick to valid edge is112 clocks");
                check(tx_desc_data[0+:64]==descriptors,"successful descriptor sequence");
                check(tx_desc_data[256+:16]==16'h155&&tx_desc_data[(32+3*16)*8+:16]==16'h159,"CAN IDs hexadecimal");
                check(tx_desc_data[(36+3*16)*8+:32]==32'h53c30000,"gripper big-endian50003um");
            end
        end
    end
    initial begin : scenarios
        logic[31:0]value;
        logic[63:0] original_stop_started;
        integer jobs_before,desc_before;
        for(integer w=0;w<32768;w=w+1)mem[w]=0;
        seq_extra_scenarios(); // PRIVATE additive prefix; original body below is byte-identical.
        $display("STAGE exhaustive shadow AXI aperture/WSTRB/no-alias");reset_core();shadow_axi_exhaustive();
        $display("STAGE reset/identity/skew");reset_core();rd(0,value);check(value=='h5254504c,"identity");rd('ha8,value);check(value==CLOCK_HZ,"nominal clock low word");rd('hac,value);check(value==0,"nominal clock high word");wr('ha8,1,2);wr('hac,1,2);skew_write();
        $display("STAGE invalid config cycle=%0d",dut.cycle);
        initialize_settings(0,1000000);write_settings();wr('h10,1,2);check(!dut.configured,"invalid denominator hardware reject");
        $display("STAGE valid config cycle=%0d",dut.cycle);
        initialize_settings(256,1000000);write_settings();wr('h10,1,0);read_origin();
        $display("STAGE corrupt bank origin=%0d cycle=%0d",origin_value,dut.cycle);
        bank_image(0,1);wr('h80,1,0);wr('h84,0,0);wr('h10,32,0);wait_bank(4,3);
        check(!bank_writable[0],"rejected bank remains frozen");wr('h10,512,0);
        $display("STAGE expired bank cycle=%0d",dut.cycle);bank_image(1,0);wr('h10,32,0);wait_bank(4,6);wr('h10,512,0);
        $display("STAGE valid bank/busy freeze cycle=%0d",dut.cycle);
        bank_image(0,0);write_busy[0]=1;
        fork
            begin wr('h10,32,0);end
            begin repeat(12)@(posedge aclk);#1;check(!bank_writable[0],"COMMIT freezes before old write drains");@(negedge aclk);write_busy[0]=0;end
        join
        wait_bank(2,0);check(!bank_writable[0],"READY bank immutable");
        $display("STAGE successor cycle=%0d",dut.cycle);successor_image();wr('h88,2,0);wr('h8c,0,0);wr('h10,64,0);
        while(dut.bank_state[1]==1)begin @(posedge aclk);#1;end
        check(dut.bank_state[1]==2,"successor fully validated READY");
        wr('h98,1,0);wr('h9c,0,0);wr('h10,256,0);wr('h90,FIRST,0);wr('h94,0,0);
        force dut.cache_valid=1'b0;wr('h10,2,2);check(dut.state==0&&descriptors==0,"ARM rejects absent cache");
        // SV release of a procedurally assigned flop retains the forced value
        // until its next assignment. Restore the already-published valid bit
        // explicitly before removing this simulation-only fault injection.
        force dut.cache_valid=1'b1;@(negedge aclk);release dut.cache_valid;
        wr('h10,2,0);
        $display("STAGE armed/STOP collision cycle=%0d target=%0d",dut.cycle,origin_value+tick_offset(FIRST+1));
        wait(dut.cycle==origin_value+tick_offset(FIRST+1)-1);
        wr('h10,4,0);
        check(dut.active_bank==0&&dut.stop_pending,"same-edge STOP overrides READY successor");
        while(descriptors<6&&dut.fault_code==0)@(posedge aclk);
        #1;check(descriptors>=6&&dut.state==4&&dut.fault_code==0&&dut.bank_state[1]==2,"minimum256-cycle RUN/brake/HOLD and no automatic successor resume");
        wr('h10,16,0);rd('h510,value);check(value>=6,"snapshot emit count");rd('h600,value);check(value>=6,"snapshot descriptor is read-only data");
        rd('h538,value);check(value==112,"measured minimum latency snapshot");rd('h540,value);check(value==112,"measured maximum latency snapshot");
        $display("STAGE HOLD/backpressure cycle=%0d",dut.cycle);backend_ready=0;while(dut.fault_code==0)@(posedge aclk);#1;check(dut.fault_code==7&&!tx_desc_valid,"backpressure faults without output");
        $display("STAGE noninteger256.5-cycle DDS");
        reset_core();case_numerator=2*CLOCK_HZ;initialize_settings(513,1000000);write_settings();wr('h10,1,0);read_origin();bank_image(0,0);
        check(tick_offset(FIRST+1)-tick_offset(FIRST)==257&&tick_offset(FIRST+2)-tick_offset(FIRST+1)==256,"noninteger fixture alternates257/256 clocks");
        wr('h80,1,0);wr('h84,0,0);wr('h10,32,0);wait_bank(2,0);wr('h98,1,0);wr('h10,256,0);wr('h90,FIRST,0);wr('h10,2,0);
        while(descriptors<6&&dut.fault_code==0)@(posedge aclk);
        #1;check(descriptors>=6&&dut.state==4&&dut.fault_code==0,"noninteger DDS RUN/brake/HOLD has no missed tick");
        $display("STAGE background validation shares engine with active HOLD at minimum noninteger tick gap");
        successor_image();jobs_before=shared_val_jobs_running;desc_before=descriptors;
        wr('h88,2,0);wr('h8c,0,0);wr('h10,64,0);
        while(dut.bank_state[1]==1&&dut.fault_code==0)begin @(posedge aclk);#1;end
        check(dut.bank_state[1]==2&&dut.fault_code==0,"concurrent background bank completes without disturbing HOLD");
        check(shared_val_jobs_running-jobs_before==4&&descriptors-desc_before>=6,"all four certificate jobs overlap continued runtime output");
        check(shared_wait_cycles>0,"validator request held across unavailable slots");
        $display("STAGE stale completed gate cannot replace missing fresh done");
        // The old gate's allow is true in HOLD. Suppress only the next done:
        // the per-job readiness latch must stop output at the fixed deadline.
        @(negedge aclk);force dut.gate_done=1'b0;
        while(dut.fault_code==0)@(posedge aclk);#1;
        check(dut.fault_code==4&&!tx_desc_valid,"fresh gate completion is mandatory even when old allow was true");
        release dut.gate_done;
        $display("STAGE lease cycle=%0d",dut.cycle);
        reset_core();case_numerator=CLOCK_HZ;initialize_settings(256,1000);write_settings();wr('h10,1,0);read_origin();bank_image(0,0);
        wr('h80,1,0);wr('h84,0,0);wr('h10,32,0);wait_bank(2,0);wr('h98,1,0);wr('h10,256,0);wr('h90,FIRST,0);wr('h10,2,0);
        wr('h10,4,0);original_stop_started=dut.stop_started;repeat(8)@(posedge aclk);wr('h10,4,0);
        check(dut.stop_started==original_stop_started,"repeated STOP cannot renew timeout");
        while(dut.fault_code==0)@(posedge aclk);#1;
        check(dut.fault_code==2&&descriptors==0,"lease expires independently before next execution tick");
        reset_core();hard_fault=1;repeat(5)@(posedge aclk);#1;check(dut.fault_code==1&&disable_req&&!motion_permit,"hard fault locks even while DISARMED");
        $display("PASS CACHE_LIFETIME_REACHABLE_TRACE jobs=%0d edges=%0d",density_mon_jobs,density_mon_edges);
        if(stable_gate_jobs==0 || stable_gate_edges==0)
            $fatal(1,"PRIVATE stable gate monitor was not exercised");
        $display("PASS STABLE_GATE_JOB_DATA jobs=%0d edges=%0d completed=%0d cancel_edges=%0d reset_jobs=%0d width=24864 controls_excluded=1",stable_gate_jobs,stable_gate_edges,stable_gate_completed,stable_gate_cancel_edges,stable_gate_reset_jobs);
        $display("PASS tb_top_protocol checks=%0d clock_hz=%0d design_only=1 descriptor_only=1 shared_jobs=%0d concurrent_jobs=%0d waiting_cycles=%0d",checks,CLOCK_HZ,shared_val_jobs,shared_val_jobs_running,shared_wait_cycles);$finish;
    end
    // Budget includes >116000 independent AXI readbacks in the new aperture test.
    initial begin #10000000;
        $display("DDS configured=%0b internal=%0b phase=%0d num=%0d threshold=%0d tick=%0b index=%0d firsttick=%0d origin=%0d record_start=%0d record_end=%0d record_tick=%0d",dut.configured,dut.dds.configured,dut.dds.phase,dut.dds.numerator,dut.dds.threshold,dut.dds_tick,dut.tick_index,dut.bfirsttick[dut.active_bank],dut.origin,dut.cur_record[0+:64],dut.cur_record[64+:64],dut.cur_record[128+:64]);
        $fatal(1,"top timeout cycle=%0d state=%0d cmd=%0d aw=%0b w=%0b bv=%0b rv=%0b bank0=%0d bank1=%0d validator=%0d pf=%0b cur=%0b fault=%0d",dut.cycle,dut.state,dut.cmd_state,dut.aw_saved,dut.w_saved,s_axi_cfg_bvalid,s_axi_cfg_rvalid,dut.bank_state[0],dut.bank_state[1],dut.validator.state,dut.pf_active,dut.cur_valid,dut.fault_code);
    end

    // Private simulation-only check of the proved PL cache lifetime contract.
    always @(posedge aclk) begin
        if(dut.val_resetn!==1'b1 || dut.shared_execution.owner_cancel===1'b1)
            density_mon_active=0;
        else if(dut.shared_execution.accept_rt===1'b1 || dut.shared_execution.accept_val===1'b1) begin
            density_mon_active=1;
            density_mon_products={dut.cached_lower_qden,dut.cached_upper_qden,dut.cached_dq_qden,dut.cached_vmax_vden,dut.cached_amax_c};
            density_mon_epoch=dut.cached_cfg_epoch;density_mon_valid=dut.cache_valid;
            density_mon_jobs=density_mon_jobs+1;
        end else if(dut.shared_execution.owner_valid===1'b1 && density_mon_active) begin
            if({dut.cached_lower_qden,dut.cached_upper_qden,dut.cached_dq_qden,dut.cached_vmax_vden,dut.cached_amax_c}!==density_mon_products ||
               dut.cached_cfg_epoch!==density_mon_epoch || dut.cache_valid!==density_mon_valid)
                $fatal(1,"private PL cache lifetime contract changed within live owner job");
            density_mon_edges=density_mon_edges+1;
        end else density_mon_active=0;
    end
    // PRIVATE monitor: cancellation does NOT terminate the data obligation.
    // End only when the registered gate.done is observed or on a synchronous
    // reset edge. Pre-edge sampling avoids NBA races; ownership holds producer
    // data through the done-consumption edge, before any next math grant.
    always @(posedge aclk) begin
        if(dut.shared_execution.gate.resetn!==1'b1) begin
            if(stable_gate_active)stable_gate_reset_jobs=stable_gate_reset_jobs+1;
            stable_gate_active=0;
        end else begin
            if(stable_gate_active) begin
                if(stable_gate_data!==stable_gate_snapshot)
                    $fatal(1,"PRIVATE stable gate DATA changed within accepted job, including cancel/drain");
                stable_gate_edges=stable_gate_edges+1;
                if(dut.shared_execution.owner_cancel!==1'b0)
                    stable_gate_cancel_edges=stable_gate_cancel_edges+1;
                if(dut.shared_execution.gate.done===1'b1) begin
                    stable_gate_active=0;
                    stable_gate_completed=stable_gate_completed+1;
                end
            end
            if(dut.shared_execution.gate.start===1'b1 && dut.shared_execution.gate.busy===1'b0) begin
                if(stable_gate_active)$fatal(1,"PRIVATE overlapping stable gate job");
                stable_gate_snapshot=stable_gate_data;
                stable_gate_active=1;
                stable_gate_jobs=stable_gate_jobs+1;
            end
        end
    end
endmodule

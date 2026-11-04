`timescale 1ns/1ps
// Explicitly SYNTHETIC protocol-only tests using the actual exact arithmetic.
// No CAN/device/hardware event or acceptance claim. Run by root with real RTL.
module tb_shared_math_gate;
    parameter integer CLOCK_HZ=99999001;
    logic clk;
    logic resetn;
    logic rt_cancel;
    logic val_cancel;
    logic val_slot_admit;
    logic rt_math_req;
    logic rt_gate_req;
    logic [31:0] rt_dt_cycles;
    logic [447:0] rt_q0;
    logic [447:0] rt_v0;
    logic [447:0] rt_a0;
    logic [447:0] rt_j;
    logic [447:0] rt_lower;
    logic [447:0] rt_upper;
    logic [447:0] rt_velocity_limit;
    logic [447:0] rt_acceleration_limit;
    logic [447:0] rt_jerk_limit;
    logic [447:0] rt_dq_max;
    logic [223:0] rt_previous_wire;
    logic rt_previous_wire_valid;
    logic rt_cache_valid;
    logic [1343:0] rt_cached_lower_qden;
    logic [1343:0] rt_cached_upper_qden;
    logic [1343:0] rt_cached_dq_qden;
    logic [1343:0] rt_cached_vmax_vden;
    logic [1343:0] rt_cached_amax_c;
    logic rt_math_grant;
    logic rt_gate_grant;
    logic rt_math_busy;
    logic rt_math_done;
    logic rt_math_overflow;
    logic [447:0] rt_q1;
    logic [447:0] rt_v1;
    logic [447:0] rt_a1;
    wire [447:0] rt_q1_latched,rt_v1_latched,rt_a1_latched;
    logic rt_gate_busy;
    logic rt_gate_done;
    logic rt_allow;
    logic rt_clamp;
    logic [223:0] rt_q16;
    logic [223:0] rt_wire_values;
    logic val_math_req;
    logic val_gate_req;
    logic [31:0] val_dt_cycles;
    logic [447:0] val_q0;
    logic [447:0] val_v0;
    logic [447:0] val_a0;
    logic [447:0] val_j;
    logic [447:0] val_lower;
    logic [447:0] val_upper;
    logic [447:0] val_velocity_limit;
    logic [447:0] val_acceleration_limit;
    logic [447:0] val_jerk_limit;
    logic [447:0] val_dq_max;
    logic [223:0] val_previous_wire;
    logic val_previous_wire_valid;
    logic val_cache_valid;
    logic [1343:0] val_cached_lower_qden;
    logic [1343:0] val_cached_upper_qden;
    logic [1343:0] val_cached_dq_qden;
    logic [1343:0] val_cached_vmax_vden;
    logic [1343:0] val_cached_amax_c;
    logic val_math_grant;
    logic val_gate_grant;
    logic val_math_busy;
    logic val_math_done;
    logic val_math_overflow;
    logic [447:0] val_q1;
    logic [447:0] val_v1;
    logic [447:0] val_a1;
    logic val_gate_busy;
    logic val_gate_done;
    logic val_allow;
    logic val_clamp;
    logic [223:0] val_q16;
    logic [223:0] val_wire_values;
    logic busy;
    logic owner_is_validator;
    logic [7:0] owner_age;
    logic runtime_collision;
    logic protocol_fault;
    always #5 clk=~clk;
    integer cycle_count=0,checks=0;
    integer job_cycle;
    logic auto_val_gate;
    localparam logic signed[191:0] C=CLOCK_HZ;
    localparam logic signed[191:0] QDEN=6*C*C*C, VDEN=2*C*C;
    shared_math_gate #(.CLOCK_HZ(CLOCK_HZ)) dut (.*);
    assign rt_gate_req=rt_math_done&&!rt_math_overflow;
    always @(posedge clk) begin
        cycle_count<=cycle_count+1;
        if(!resetn||val_cancel)val_gate_req<=0;
        else if(auto_val_gate&&val_math_done&&!val_math_overflow)val_gate_req<=1;
        else if(val_gate_grant)val_gate_req<=0;
    end
    // BEGIN LATCHED_OUTPUT_MONITOR: pre-edge done/data, post-NBA saved outputs.
    // The original endpoint assertions below independently check Fraction goldens.
    logic [1343:0] expected_rt_latched = '0;
    integer latched_samples=0,latched_resets=0,latched_fresh=0;
    integer latched_gate_holds=0,latched_validator_holds=0;
    always @(posedge clk) begin
        if(resetn===1'b0) begin
            expected_rt_latched='0;
            latched_resets=latched_resets+1;
        end else if(resetn===1'b1) begin
            if(rt_math_done===1'b1) begin
                expected_rt_latched={rt_q1,rt_v1,rt_a1};
                latched_fresh=latched_fresh+1;
            end
            if(rt_gate_done===1'b1)latched_gate_holds=latched_gate_holds+1;
            if(val_math_done===1'b1)latched_validator_holds=latched_validator_holds+1;
        end
        // Exactly the consumer edge: producer done/raw was sampled above;
        // the shared saved registers and top fresh-ready flag update by NBA.
        #1;
        if(resetn===1'b0||resetn===1'b1) begin
            latched_samples=latched_samples+1;
            if({rt_q1_latched,rt_v1_latched,rt_a1_latched}!==expected_rt_latched)
                $fatal(1,"latched runtime outputs stale/overwritten/reset mismatch cycle %0d",cycle_count);
        end
    end
    // END LATCHED_OUTPUT_MONITOR
    task automatic ck(input logic condition,input string label);
        begin checks=checks+1;if(condition!==1'b1)$fatal(1,"FAIL %s cycle %0d",label,cycle_count);end
    endtask
    task automatic next_edge;begin @(posedge clk);#1;end endtask
    task automatic reset_all;
        begin
            @(negedge clk);resetn=0;rt_math_req=0;val_math_req=0;
            rt_cancel=0;val_cancel=0;val_slot_admit=1;auto_val_gate=1;
            #1;ck(!rt_math_grant&&!val_math_grant&&!rt_gate_grant&&!val_gate_grant&&
                !rt_math_done&&!val_math_done&&!rt_gate_done&&!val_gate_done,"reset assertion immediately suppresses all events");
            repeat(2)next_edge();
            ck(!busy&&!rt_math_done&&!val_math_done&&!rt_gate_done&&!val_gate_done,"reset clears events");
            ck(!protocol_fault&&!runtime_collision,"reset clears faults");
            @(negedge clk);resetn=1;
        end
    endtask
    task automatic configure(input logic client,input logic[63:0] qvalue);
        logic[447:0] q,lo,hi,lim;
        logic[1343:0] lq,hq,dq,vv,aa;
        begin
            for(integer a=0;a<7;a=a+1)begin
                q[64*a+:64]=qvalue;lo[64*a+:64]=-64'sd4294967296;
                hi[64*a+:64]=64'sd4294967296;lim[64*a+:64]=64'sd4294967296;
                lq[192*a+:192]=-192'sd4294967296*QDEN;
                hq[192*a+:192]=192'sd4294967296*QDEN;
                dq[192*a+:192]=192'sd4294967296*QDEN;
                vv[192*a+:192]=192'sd4294967296*VDEN;
                aa[192*a+:192]=192'sd4294967296*C;
            end
            if(client==1'b0)begin
                rt_dt_cycles=32'd256;
                rt_q0=q;
                rt_v0=448'd0;
                rt_a0=448'd0;
                rt_j=448'd0;
                rt_lower=lo;
                rt_upper=hi;
                rt_velocity_limit=lim;
                rt_acceleration_limit=lim;
                rt_jerk_limit=lim;
                rt_dq_max=lim;
                rt_previous_wire=224'd0;
                rt_previous_wire_valid=1'b0;
                rt_cache_valid=1'b1;
                rt_cached_lower_qden=lq;
                rt_cached_upper_qden=hq;
                rt_cached_dq_qden=dq;
                rt_cached_vmax_vden=vv;
                rt_cached_amax_c=aa;
            end
            else if(client==1'b1)begin
                val_dt_cycles=32'd256;
                val_q0=q;
                val_v0=448'd0;
                val_a0=448'd0;
                val_j=448'd0;
                val_lower=lo;
                val_upper=hi;
                val_velocity_limit=lim;
                val_acceleration_limit=lim;
                val_jerk_limit=lim;
                val_dq_max=lim;
                val_previous_wire=224'd0;
                val_previous_wire_valid=1'b0;
                val_cache_valid=1'b1;
                val_cached_lower_qden=lq;
                val_cached_upper_qden=hq;
                val_cached_dq_qden=dq;
                val_cached_vmax_vden=vv;
                val_cached_amax_c=aa;
            end
        end
    endtask
    task automatic start_job(input logic client);
        begin
            @(negedge clk);
            if(client)val_math_req=1;else rt_math_req=1;
            #1;ck(client?val_math_grant:rt_math_grant,"same-edge combinational math grant");
            next_edge();job_cycle=cycle_count;
            if(client)val_math_req=0;else rt_math_req=0;
        end
    endtask
    task automatic wait_math(input logic client);
        begin
            while(!(client?val_math_done:rt_math_done))next_edge();
            ck(cycle_count-job_cycle==78,"math done remains E78");
            ck(!(client?val_math_overflow:rt_math_overflow),"finite stationary exact math");
        end
    endtask
    task automatic wait_gate(input logic client,input integer expected_cycle);
        begin
            while(!(client?val_gate_done:rt_gate_done))next_edge();
            ck(cycle_count-job_cycle==expected_cycle,"gate done exact edge");
            ck(client?val_allow:rt_allow,"valid synthetic certificate");
            next_edge();ck(!busy,"ownership released one edge after done");
        end
    endtask
    task automatic check_quarter(input logic client,input logic half_value);
        logic[31:0] code,wire_joint,wire_gripper;
        begin
            code=half_value?32'd32768:32'd16384;
            wire_joint=half_value?32'd500:32'd250;
            wire_gripper=half_value?32'd500000:32'd250000;
            for(integer a=0;a<7;a=a+1)begin
                ck((client?val_q16[32*a+:32]:rt_q16[32*a+:32])==code,"Q16 by owner");
                ck((client?val_wire_values[32*a+:32]:rt_wire_values[32*a+:32])==(a==6?wire_gripper:wire_joint),"CAN unit by axis");
            end
        end
    endtask
    // Exact rational oracle constants generated with Python Fraction for one
    // second, distinct q0=axis/64, and nonzero v/a/j for both owners.
    task automatic moving_job(input logic client);
        logic signed[63:0] expected_q[0:6];
        logic signed[31:0] expected_code[0:6],expected_wire[0:6];
        begin
            configure(client,0);
            if(client)begin
                expected_q='{-64'sd425022805,-64'sd357913941,-64'sd290805077,-64'sd223696213,-64'sd156587349,-64'sd89478485,-64'sd22369621};
                expected_code='{-32'sd6485,-32'sd5461,-32'sd4437,-32'sd3413,-32'sd2389,-32'sd1365,-32'sd341};
                expected_wire='{-32'sd99,-32'sd83,-32'sd68,-32'sd52,-32'sd36,-32'sd21,-32'sd5203};
                val_dt_cycles=CLOCK_HZ;
                for(integer a=0;a<7;a=a+1)begin
                    val_q0[64*a+:64]=a*64'sd67108864;
                    val_v0[64*a+:64]=-64'sd536870912;
                    val_a0[64*a+:64]=64'sd268435456;
                    val_j[64*a+:64]=-64'sd134217728;
                end
            end else begin
                expected_q='{64'sd1386916523,64'sd1454025387,64'sd1521134251,64'sd1588243115,64'sd1655351979,64'sd1722460843,64'sd1789569707};
                expected_code='{32'sd21163,32'sd22187,32'sd23211,32'sd24235,32'sd25259,32'sd26283,32'sd27307};
                expected_wire='{32'sd323,32'sd339,32'sd354,32'sd370,32'sd385,32'sd401,32'sd416672};
                rt_dt_cycles=CLOCK_HZ;
                for(integer a=0;a<7;a=a+1)begin
                    rt_q0[64*a+:64]=a*64'sd67108864;
                    rt_v0[64*a+:64]=64'sd1073741824;
                    rt_a0[64*a+:64]=64'sd536870912;
                    rt_j[64*a+:64]=64'sd268435456;
                end
            end
            start_job(client);
            @(negedge clk);configure(client,64'h7fffffffffffffff);
            wait_math(client);
            for(integer a=0;a<7;a=a+1)begin
                ck((client?val_q1[64*a+:64]:rt_q1[64*a+:64])===expected_q[a],"moving axis endpoint by owner");
                ck((client?val_v1[64*a+:64]:rt_v1[64*a+:64])===(client?-64'sd335544320:64'sd1744830464),"moving velocity port");
                ck((client?val_a1[64*a+:64]:rt_a1[64*a+:64])===(client?64'sd134217728:64'sd805306368),"moving acceleration port");
            end
            wait_gate(client,client?95:94);
            for(integer a=0;a<7;a=a+1)begin
                ck((client?val_q16[32*a+:32]:rt_q16[32*a+:32])===expected_code[a],"moving Q16 exact rational oracle");
                ck((client?val_wire_values[32*a+:32]:rt_wire_values[32*a+:32])===expected_wire[a],"moving wire exact rational oracle");
            end
            ck(!(client?val_clamp:rt_clamp),"moving trajectory needs no clamp");
        end
    endtask
    initial begin
        clk=0;resetn=0;rt_math_req=0;val_math_req=0;val_gate_req=0;
        rt_cancel=0;val_cancel=0;val_slot_admit=0;auto_val_gate=1;
        configure(0,64'd1073741824);configure(1,64'd2147483648);
        reset_all();
        // RT context snapshot remains valid despite all accepted-client raw,
        // bound, cache, and previous-wire inputs being disturbed while busy.
        start_job(0);
        @(negedge clk);configure(0,64'h7fffffffffffffff);rt_lower='1;rt_cache_valid=0;rt_previous_wire='1;
        val_cancel=1;
        wait_math(0);wait_gate(0,94);check_quarter(0,0);
        ck(!protocol_fault&&!runtime_collision,"validator cancel does not touch RT");
        val_cancel=0;
        configure(1,64'd2147483648);start_job(1);
        @(negedge clk);configure(1,64'h8000000000000000);val_upper='0;val_cache_valid=0;val_previous_wire='1;
        wait_math(1);wait_gate(1,95);check_quarter(1,1);check_quarter(0,0);
        ck(!protocol_fault&&!runtime_collision,"normal owner schedule");
        // Same-cycle requests have RT priority; there is no implicit queue.
        configure(0,64'd1073741824);configure(1,64'd2147483648);
        @(negedge clk);rt_math_req=1;val_math_req=1;#1;
        ck(rt_math_grant&&!val_math_grant,"runtime priority");
        next_edge();job_cycle=cycle_count;rt_math_req=0;val_math_req=0;
        wait_math(0);wait_gate(0,94);
        // Explicit admission false blocks validation without consuming request.
        val_slot_admit=0;@(negedge clk);val_math_req=1;#1;
        ck(!val_math_grant&&!busy,"no validator admission");
        repeat(4)next_edge();ck(!busy,"denied request not queued");val_math_req=0;val_slot_admit=1;
        // Overflow math is E78, no gate, release E79.
        configure(1,64'd2147483648);val_dt_cycles=0;start_job(1);
        while(!val_math_done)next_edge();
        ck(cycle_count-job_cycle==78&&val_math_overflow,"overflow edge preserved");
        next_edge();ck(!busy&&!val_gate_done,"overflow releases without gate");
        moving_job(0);moving_job(1);
        // Cancel at every stage across both jobs. Cancelled client emits no
        // future done, does not need gate_req after math, and always drains.
        for(integer who=0;who<2;who=who+1)begin
            for(integer phase=0;phase<=95;phase=phase+1)begin
                reset_all();configure(who!=0,who?64'd2147483648:64'd1073741824);
                start_job(who!=0);
                repeat(phase)next_edge();
                @(negedge clk);if(who)val_cancel=1;else rt_cancel=1;
                repeat(100)begin
                    next_edge();ck(who?(!val_math_done&&!val_gate_done):(!rt_math_done&&!rt_gate_done),"cancel suppresses events");
                end
                ck(!busy,"cancel drains ownership");
                ck(!protocol_fault,"cancel drains before owner watchdog");
            end
        end
        // Reset at every in-flight offset must immediately invalidate output.
        for(integer phase=0;phase<=95;phase=phase+1)begin
            reset_all();configure(1,64'd2147483648);start_job(1);
            repeat(phase)next_edge();reset_all();
            repeat(3)next_edge();ck(!busy&&!val_math_done&&!val_gate_done,"reset no stale completion");
        end
        // Busy collision cannot preempt the validator.
        reset_all();configure(1,64'd2147483648);start_job(1);
        repeat(10)next_edge();@(negedge clk);rt_math_req=1;#1;ck(!rt_math_grant,"busy collision cannot grant");
        next_edge();rt_math_req=0;ck(runtime_collision&&protocol_fault&&owner_is_validator,"busy collision sticky");
        wait_math(1);wait_gate(1,95);check_quarter(1,1);
        // Missing gate request raises fault E104, keeps ownership; cancellation
        // then releases safely. This is a client protocol fault, not a queue.
        reset_all();configure(1,64'd2147483648);auto_val_gate=0;start_job(1);
        repeat(103)next_edge();ck(busy&&!protocol_fault,"no early owner watchdog");
        next_edge();ck(busy&&protocol_fault,"owner watchdog E104");
        val_cancel=1;next_edge();ck(!busy,"cancel stalled ready owner");
        // BEGIN LATCHED_OUTPUT_COVERAGE: require actual original-test exercise.
        if(latched_fresh<1||latched_gate_holds<1||latched_validator_holds<1||latched_resets<1)
            $fatal(1,"latched monitor coverage missing fresh=%0d gate=%0d validator=%0d reset=%0d",latched_fresh,latched_gate_holds,latched_validator_holds,latched_resets);
        $display("PASS LATCHED_RUNTIME_OUTPUTS samples=%0d fresh=%0d gate_hold=%0d validator_hold=%0d reset=%0d",latched_samples,latched_fresh,latched_gate_holds,latched_validator_holds,latched_resets);
        // END LATCHED_OUTPUT_COVERAGE
        $display("PASS SHARED_MATH_GATE checks=%0d RT_MATH78 RT_GATE94 VAL_GATE95 MOVING_JOBS2",checks);
        $finish;
    end
    initial begin #10000000;$fatal(1,"shared protocol timeout");end
endmodule

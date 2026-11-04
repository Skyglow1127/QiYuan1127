`timescale 1ns/1ps
// Targeted functional test only. No STA or physical-frequency conclusion.
module tb_tick_quiet #(parameter integer CLOCK_HZ=99999001);
    logic clk=0;always #5 clk=~clk;
    logic resetn=0,config_commit=0,enable=0;
    logic [63:0] rate_num=0,rate_den=0;
    wire tick,config_error,quiet_window;
    wire [63:0] tick_index,cycle,origin;
    tick_gen #(.CLOCK_HZ(CLOCK_HZ)) dut(.*);

    // The driver marks each directed rate legal/illegal independently of the
    // DUT's input_valid implementation. Enabled commit is always rejected.
    logic directed_rate_legal=0,runtime_armed=0;
    logic ref_configured=0,ref_error=0,ref_tick=0;
    logic [63:0] ref_phase=0,ref_num=0,ref_threshold=0;
    logic [63:0] ref_cycle=0,ref_origin=0,ref_index=0;
    logic [127:0] step_sum,wide_threshold;
    logic pre_tick,pre_quiet,pre_error;
    integer checks=0,promise_remaining=0,promises=0;
    integer pipe_remaining=0,val_remaining=0,val_grants=0,rt_reservations=0;
    integer pending_tick_aliases=0,weak_rule_collisions=0,equality_denials=0,wide_horizons=0;
    integer val_before_first_tick=0;
    logic old_pipe,old_val,runtime_due,val_grant,weak_grant;
    logic [31:0] requests=32'h9357a20d;
    logic val_pending=1;

    task automatic check(input logic condition,input string message);
        begin
            checks=checks+1;
            if(condition!==1'b1)$fatal(1,"%s cycle=%0d phase=%0d num=%0d threshold=%0d",message,cycle,ref_phase,ref_num,ref_threshold);
        end
    endtask

    // Independent lookahead uses repeated DDS steps, not P+128*N or DUT phase.
    // It detects a raw crossing even when external enable currently masks tick.
    function automatic logic future_128_empty(
        input logic configured,input logic[63:0] phase_value,
        input logic[63:0] numerator_value,input logic[63:0] threshold_value);
        logic[127:0] p,s;
        logic hit;
        begin
            p={64'd0,phase_value};hit=0;
            for(integer n=0;n<128;n=n+1)begin
                s=p+{64'd0,numerator_value};
                if(s>={64'd0,threshold_value})begin
                    hit=1;p=s-{64'd0,threshold_value};
                end else p=s;
            end
            future_128_empty=configured&&!hit;
        end
    endfunction

    always @(posedge clk)begin
        // Pre-edge values are exactly what a synchronous arbiter samples.
        pre_tick=tick;pre_quiet=quiet_window;pre_error=config_error;
        old_pipe=(pipe_remaining>0);old_val=(val_remaining>0);
        runtime_due=pre_tick&&enable&&runtime_armed&&!pre_error;
        val_grant=enable&&!pre_error&&pre_quiet&&!pre_tick&&!old_pipe&&!old_val&&val_pending;
        // Deliberately wrong rule omits the pending registered tick guard.
        // This is only a counterexample observer, never a DUT/model grant.
        weak_grant=enable&&!pre_error&&pre_quiet&&!old_pipe&&!old_val&&val_pending;

        if(!resetn)begin
            ref_configured=0;ref_error=0;ref_tick=0;
            ref_phase=0;ref_num=0;ref_threshold=0;
            ref_cycle=0;ref_origin=0;ref_index=0;
            promise_remaining=0;pipe_remaining=0;val_remaining=0;val_pending=1;
        end else begin
            // Conceptual top reservation: tick captured at externalT+1,
            // pipe remains reserved through externalT+112. A99 VAL job drains
            // without preemption. No production datapath is replaced by this.
            if(config_commit&&!enable&&directed_rate_legal)begin
                pipe_remaining=0;val_remaining=0;val_pending=1;
            end else if(!enable||pre_error)begin
                pipe_remaining=0;val_remaining=0;
            end else begin
                check(!(old_pipe&&old_val),"positive top reservation never overlaps RT/VAL");
                if(runtime_due)begin
                    check(!old_val&&!old_pipe,"runtime tick never queues behind admission");
                    rt_reservations=rt_reservations+1;
                end
                if(runtime_due&&weak_grant)weak_rule_collisions=weak_rule_collisions+1;
                if(pre_tick&&pre_quiet)pending_tick_aliases=pending_tick_aliases+1;
                if(pipe_remaining>0)pipe_remaining=pipe_remaining-1;
                if(val_remaining>0)val_remaining=val_remaining-1;
                if(runtime_due)pipe_remaining=111;
                if(val_grant)begin
                    check(!runtime_due&&!old_pipe&&!old_val,"admission respects pending/active RT reservation");
                    val_remaining=99;val_pending=0;val_grants=val_grants+1;
                    if(ref_index==0)val_before_first_tick=val_before_first_tick+1;
                end
                // One held request, never a runtime queue. Deterministic varied
                // request arrivals exercise waiting inside and outside windows.
                requests={requests[30:0],requests[31]^requests[21]^requests[1]^requests[0]};
                if(requests[2:0]==0)val_pending=1;
                check(!(pipe_remaining>0&&val_remaining>0),"positive grant update stays mutually exclusive");
            end

            // Every sampled quiet promise is checked against actual future
            // registered tick pulses for128 successive edges. Valid re-commit
            // starts a new epoch, so old promise/reservation history is cleared.
            if(config_commit&&!enable&&directed_rate_legal)promise_remaining=0;
            else if(pre_quiet===1'b1)begin promise_remaining=128;promises=promises+1;end

            ref_tick=0;
            if(ref_cycle==64'hffffffffffffffff)begin ref_configured=0;ref_error=1;end
            else begin
                ref_cycle=ref_cycle+1;
                if(ref_configured)begin
                    step_sum={64'd0,ref_phase}+{64'd0,ref_num};
                    if(step_sum>={64'd0,ref_threshold})begin
                        ref_phase=step_sum-{64'd0,ref_threshold};
                        ref_index=ref_index+1;ref_tick=enable;
                    end else ref_phase=step_sum[63:0];
                end
                if(config_commit)begin
                    if(enable||!directed_rate_legal)ref_error=1;
                    else begin
                        wide_threshold={64'd0,rate_den}*CLOCK_HZ;
                        check(wide_threshold[127:64]==0,"directed valid threshold fits64");
                        ref_num=rate_num;ref_threshold=wide_threshold[63:0];
                        ref_configured=1;ref_phase=0;ref_index=0;ref_origin=ref_cycle;
                        ref_error=0;ref_tick=0;
                    end
                end
            end
        end
        #1;
        check({tick,config_error,tick_index,cycle,origin}==={ref_tick,ref_error,ref_index,ref_cycle,ref_origin},"cyclewise independent DDS oracle");
        check(dut.configured===ref_configured,"quiet extension/enable cannot mutate internal configured");
        check({dut.phase,dut.numerator,dut.threshold}==={ref_phase,ref_num,ref_threshold},"cyclewise internal DDS state is unchanged");
        check(quiet_window===future_128_empty(ref_configured,ref_phase,ref_num,ref_threshold),"quiet agrees with128 independent future steps");
        if(resetn&&promise_remaining>0)begin
            check(tick===1'b0,"historical quiet promise forbids actual next128 ticks");
            promise_remaining=promise_remaining-1;
        end
        if(ref_configured)begin
            step_sum={64'd0,ref_phase}+({64'd0,ref_num}<<7);
            if(step_sum=={64'd0,ref_threshold})begin
                check(!quiet_window,"equality boundary must deny quiet");equality_denials=equality_denials+1;
            end
            if(step_sum[127:64]!=0)begin
                check(!quiet_window,"horizon above64bits cannot wrap into quiet");wide_horizons=wide_horizons+1;
            end
        end
    end

    task automatic reset_dut;
        begin
            @(negedge clk);resetn=0;enable=0;config_commit=0;runtime_armed=0;
            repeat(4)@(negedge clk);
            check(!tick&&!quiet_window&&tick_index==0&&origin==0,"reset clears pending tick and quiet epoch");
            resetn=1;repeat(2)@(negedge clk);
        end
    endtask
    task automatic configure(input logic[63:0] n,d,input logic rate_legal,input logic enabled);
        begin
            @(negedge clk);rate_num=n;rate_den=d;directed_rate_legal=rate_legal;
            enable=enabled;config_commit=1;
            @(negedge clk);config_commit=0;
            check(config_error==(!rate_legal||enabled),"directed commit accept/reject");
        end
    endtask
    task automatic run_edges(input integer count);
        begin repeat(count)@(negedge clk);end
    endtask
    initial begin : scenarios
        logic[63:0] old_num,old_origin,large_den,large_num;
        logic[127:0] large_threshold;
        integer first_tick_grants;
        reset_dut();
        configure(0,1,0,0);run_edges(5);check(!quiet_window&&!dut.configured,"invalid initial rate creates no window");

        $display("STAGE quiet integer minimum256");
        configure(CLOCK_HZ,256,1,0);@(negedge clk);enable=1;runtime_armed=1;
        run_edges(256*12);
        check(pending_tick_aliases>0&&weak_rule_collisions>0,"negative proof: quiet&&pending tick defeats weak grant rule");
        check(equality_denials>0,"integer exact128-step boundary exercised");

        $display("STAGE quiet noninteger minimum256.5");
        configure(64'd2*CLOCK_HZ,513,1,0);@(negedge clk);enable=1;
        run_edges(257*12);
        // enable gates only pulses. Internal phase/index/config continue.
        @(negedge clk);enable=0;run_edges(600);@(negedge clk);enable=1;run_edges(600);

        $display("STAGE rejected commits retain original DDS state");
        old_num=ref_num;old_origin=origin;
        configure(1,1,1,1);check(ref_num==old_num&&origin==old_origin,"enabled commit must not retime");
        configure(1,0,0,0);check(ref_num==old_num&&origin==old_origin,"zero denominator must not retime");
        configure(1,100,0,0);check(ref_num==old_num,"gap aboveUINT32 rejected");
        configure(CLOCK_HZ,1,0,0);check(ref_num==old_num,"gap below256 rejected");
        configure(1,64'hffffffffffffffff,0,0);check(ref_num==old_num,"threshold overflow rejected");

        $display("STAGE long firsttick allows preARM work");
        configure(1,1,1,0);first_tick_grants=val_before_first_tick;
        @(negedge clk);enable=1;runtime_armed=0;run_edges(700);
        check(tick_index==0&&val_before_first_tick>first_tick_grants,"quiet admits validation before a distant first tick");
        while(val_remaining<40)@(negedge clk);
        runtime_armed=1;run_edges(300);check(tick_index==0,"ARM does not create early DDS tick");

        $display("STAGE128bit horizon nearUINT64 threshold");
        large_den=64'hffffffffffffffff/CLOCK_HZ;
        large_threshold={64'd0,large_den}*CLOCK_HZ;large_num=large_threshold[63:0]>>8;
        configure(large_num,large_den,1,0);@(negedge clk);enable=1;
        run_edges(257*8);check(wide_horizons>0,"wide horizon overflow case reached");

        $display("STAGE recommit and reset clear registered pending tick");
        do @(negedge clk);while(!tick);
        enable=0;runtime_armed=0;rate_num=CLOCK_HZ;rate_den=256;directed_rate_legal=1;config_commit=1;
        @(negedge clk);config_commit=0;
        check(!tick&&tick_index==0&&quiet_window,"valid recommit clears pending event/history without changing firsttick rule");
        enable=1;runtime_armed=1;run_edges(800);
        do @(negedge clk);while(!tick);
        reset_dut();check(!dut.configured&&!quiet_window&&!tick,"reset clears configured and pending promise");

        check(promises>128&&val_grants>0&&rt_reservations>0,"positive reservation/future-history proofs exercised");
        $display("PASS tb_tick_quiet checks=%0d promises=%0d val=%0d rt=%0d negative_weak_collisions=%0d wide=%0d design_only=1",checks,promises,val_grants,rt_reservations,weak_rule_collisions,wide_horizons);
        $finish;
    end
    initial begin #2000000;$fatal(1,"tb_tick_quiet timeout");end
endmodule

`timescale 1ns/1ps
// Axis-pipeline candidate test; no synthesis or hardware claim.
module tb_limit_gate_pipe #(
    parameter integer CLOCK_HZ=100000000
);
    logic clk=0; always #5 clk=~clk;
    logic resetn=0,start=0;
    logic [447:0] q0,v0,a0,j,q1,v1,a1,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max;
    logic [1343:0] qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms,cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c;
    logic [31:0] dt_cycles;
    logic [223:0] previous_wire;
    logic previous_wire_valid,terms_valid,cache_valid,math_overflow;
    wire old_allow,old_clamp,busy,done,allow,clamp;
    wire [223:0] old_q16,old_wire,q16,wire_values;
    logic expected_old_allow,expected_old_clamp;
    logic [223:0] expected_old_q16,expected_old_wire;
    logic [24899:0] saved_inputs;
    integer fd,count,rc,index,latency,resets=0;
    string vector_path;
    limit_gate #(.CLOCK_HZ(CLOCK_HZ)) legacy(
        .q0(q0),
        .v0(v0),
        .a0(a0),
        .j(j),
        .q1(q1),
        .v1(v1),
        .a1(a1),
        .lower(lower),
        .upper(upper),
        .velocity_limit(velocity_limit),
        .acceleration_limit(acceleration_limit),
        .jerk_limit(jerk_limit),
        .dq_max(dq_max),
        .dt_cycles(dt_cycles),
        .previous_wire(previous_wire),
        .previous_wire_valid(previous_wire_valid),
        .allow(old_allow),.clamp(old_clamp),.q16(old_q16),.wire_values(old_wire));
    limit_gate_pipe #(.CLOCK_HZ(CLOCK_HZ)) dut(
        .clk(clk),
        .resetn(resetn),
        .start(start),
        .q0(q0),
        .v0(v0),
        .a0(a0),
        .j(j),
        .q1(q1),
        .v1(v1),
        .a1(a1),
        .lower(lower),
        .upper(upper),
        .velocity_limit(velocity_limit),
        .acceleration_limit(acceleration_limit),
        .jerk_limit(jerk_limit),
        .dq_max(dq_max),
        .dt_cycles(dt_cycles),
        .previous_wire(previous_wire),
        .previous_wire_valid(previous_wire_valid),
        .qp0_terms(qp0_terms),
        .qp1_terms(qp1_terms),
        .qp2_terms(qp2_terms),
        .qp3_terms(qp3_terms),
        .vp0_terms(vp0_terms),
        .vp1_terms(vp1_terms),
        .vp2_terms(vp2_terms),
        .ap0_terms(ap0_terms),
        .ap1_terms(ap1_terms),
        .cached_lower_qden(cached_lower_qden),
        .cached_upper_qden(cached_upper_qden),
        .cached_dq_qden(cached_dq_qden),
        .cached_vmax_vden(cached_vmax_vden),
        .cached_amax_c(cached_amax_c),
        .terms_valid(terms_valid),
        .cache_valid(cache_valid),
        .math_overflow(math_overflow),
        .busy(busy),.done(done),.allow(allow),.clamp(clamp),.q16(q16),.wire_values(wire_values));

    task automatic abort_check(input integer abort_after);
        begin
            @(negedge clk);
            {q0,v0,a0,j,q1,v1,a1,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,dt_cycles,previous_wire,previous_wire_valid,qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms,cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c,terms_valid,cache_valid,math_overflow}=saved_inputs;
            start=1;
            @(posedge clk); #1;
            if (busy !== 1 || done !== 0) $fatal(1,"reset-case start failed");
            repeat(abort_after) begin
                @(posedge clk);#1;
                if(done !== 0 || busy !== 1) $fatal(1,"early done before reset at phase%0d",abort_after);
            end
            @(negedge clk);start=0;resetn=0;
            @(posedge clk);#1;
            if(busy !== 0 || done !== 0 || allow !== 0 || clamp !== 0 || q16 !== '0 || wire_values !== '0)
                $fatal(1,"reset did not clear state/outputs at phase%0d",abort_after);
            @(negedge clk);resetn=1;
            repeat(17) begin
                @(posedge clk);#1;
                if(busy !== 0 || done !== 0 || allow !== 0 || clamp !== 0 || q16 !== '0 || wire_values !== '0)
                    $fatal(1,"reset allowed stale completion at phase%0d",abort_after);
            end
            resets=resets+1;
        end
    endtask
    task automatic run_case(input integer number);
        logic want_allow,want_clamp;
        logic [223:0] want_q16,want_wire;
        begin
            #1;
            if(old_allow !== expected_old_allow || old_clamp !== expected_old_clamp ||
               old_q16 !== expected_old_q16 || old_wire !== expected_old_wire)
                $fatal(1,"legacy vs independent Python mismatch row%0d allow%0b/%0b q%h/%h wire%h/%h",
                    number,old_allow,expected_old_allow,old_q16,expected_old_q16,old_wire,expected_old_wire);
            want_allow=expected_old_allow && terms_valid && cache_valid && !math_overflow;
            want_clamp=want_allow ? expected_old_clamp : 0;
            want_q16=want_allow ? expected_old_q16 : '0;
            want_wire=want_allow ? expected_old_wire : '0;
            @(negedge clk);start=1;
            @(posedge clk);#1;
            if(busy !== 1 || done !== 0 || allow !== 0 || clamp !== 0 || q16 !== '0 || wire_values !== '0)
                $fatal(1,"start handshake/clear row%0d",number);
            // Poison every external job field before E1. Held/repeated start
            // through E15 must never replace the accepted E0 snapshot.
            @(negedge clk);start=1;{q0,v0,a0,j,q1,v1,a1,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,dt_cycles,previous_wire,previous_wire_valid,qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms,cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c,terms_valid,cache_valid,math_overflow}='x;
            for(latency=1;latency<=15;latency=latency+1) begin
                @(posedge clk);#1;
                if(latency<15 && (busy !== 1 || done !== 0 || allow !== 0 || clamp !== 0 || q16 !== '0 || wire_values !== '0))
                    $fatal(1,"early result row%0d latency%0d",number,latency);
                if(latency<15) begin
                    @(negedge clk);start=1;
                    if(latency[0]) {q0,v0,a0,j,q1,v1,a1,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,dt_cycles,previous_wire,previous_wire_valid,qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms,cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c,terms_valid,cache_valid,math_overflow}='1;
                    else {q0,v0,a0,j,q1,v1,a1,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,dt_cycles,previous_wire,previous_wire_valid,qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms,cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c,terms_valid,cache_valid,math_overflow}='x;
                end
            end
            if(done !== 1 || busy !== 0 || allow !== want_allow || clamp !== want_clamp || q16 !== want_q16 || wire_values !== want_wire)
                $fatal(1,"pipe result row%0d E15 allow%0b/%0b clamp%0b/%0b q%h/%h wire%h/%h",
                    number,allow,want_allow,clamp,want_clamp,q16,want_q16,wire_values,want_wire);
            @(negedge clk);start=0;
            @(posedge clk);#1;
            if(done !== 0 || busy !== 0)$fatal(1,"done pulse/ignored busy-start row%0d",number);
        end
    endtask
    // Analytic stationary case: q32=65536 gives q16=1 on all axes.
    // Deg wire RNE(1000/65536)=0; gripper RNE(1000000/65536)=15.
    // This explicitly tests the unit sideband across all overlapping tokens.
    task automatic load_axis_units_case(input logic bad_last_axis);
        logic signed [191:0] c, qden, vden;
        logic signed [63:0] bound, one;
        begin
            c=CLOCK_HZ; qden=192'sd6*c*c*c; vden=192'sd2*c*c;
            one=64'sd4294967296; bound=64'sd42949672960;
            q0='0;v0='0;a0='0;j='0;q1='0;v1='0;a1='0;
            lower='0;upper='0;velocity_limit='0;acceleration_limit='0;jerk_limit='0;dq_max='0;
            qp0_terms='0;qp1_terms='0;qp2_terms='0;qp3_terms='0;
            vp0_terms='0;vp1_terms='0;vp2_terms='0;ap0_terms='0;ap1_terms='0;
            cached_lower_qden='0;cached_upper_qden='0;cached_dq_qden='0;
            cached_vmax_vden='0;cached_amax_c='0;
            previous_wire='0;previous_wire_valid=1;dt_cycles=1;
            terms_valid=1;cache_valid=1;math_overflow=0;
            for(integer a=0;a<7;a=a+1) begin
                q0[64*a+:64]=64'd65536;q1[64*a+:64]=64'd65536;
                lower[64*a+:64]=-bound;upper[64*a+:64]=bound;
                velocity_limit[64*a+:64]=one;acceleration_limit[64*a+:64]=one;
                jerk_limit[64*a+:64]=one;dq_max[64*a+:64]=one;
                qp0_terms[192*a+:192]=192'sd65536*qden;
                cached_lower_qden[192*a+:192]=-192'sd42949672960*qden;
                cached_upper_qden[192*a+:192]=192'sd42949672960*qden;
                cached_dq_qden[192*a+:192]=192'sd4294967296*qden;
                cached_vmax_vden[192*a+:192]=192'sd4294967296*vden;
                cached_amax_c[192*a+:192]=192'sd4294967296*c;
            end
            expected_old_allow=1;expected_old_clamp=0;
            expected_old_q16={7{32'd1}};expected_old_wire='0;
            expected_old_wire[192+:32]=32'd15;
            if(bad_last_axis) begin
                // 2m previous is in [-10m,10m], but exceeds DQ=1m.
                // Only axis6 fails, at S6/E13; E14 must see that failure.
                previous_wire[192+:32]=32'd2000000;
                expected_old_allow=0;expected_old_q16='0;expected_old_wire='0;
            end
        end
    endtask
    initial begin
        {q0,v0,a0,j,q1,v1,a1,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,dt_cycles,previous_wire,previous_wire_valid,qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms,cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c,terms_valid,cache_valid,math_overflow}='0;
        repeat(3)@(posedge clk);
        @(negedge clk);resetn=1;
        if(!$value$plusargs("VECTORS=%s",vector_path))$fatal(1,"provide +VECTORS path");
        fd=$fopen(vector_path,"r");if(!fd)$fatal(1,"cannot read vectors");
        rc=$fscanf(fd,"%d\n",count);if(rc!=1 || count<1)$fatal(1,"bad vector count");
        for(index=0;index<count;index=index+1)begin
            rc=$fscanf(fd,"%h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h\n",q0,v0,a0,j,q1,v1,a1,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,dt_cycles,previous_wire,previous_wire_valid,expected_old_allow,expected_old_clamp,expected_old_q16,expected_old_wire,qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms,cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c,terms_valid,cache_valid,math_overflow);
            if(rc!=37)$fatal(1,"row%0d malformed fields%0d",index,rc);
            if(index==0)begin
                saved_inputs={q0,v0,a0,j,q1,v1,a1,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,dt_cycles,previous_wire,previous_wire_valid,qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms,cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c,terms_valid,cache_valid,math_overflow};
                for(integer k=0;k<15;k=k+1)abort_check(k);
                {q0,v0,a0,j,q1,v1,a1,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,dt_cycles,previous_wire,previous_wire_valid,qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms,cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c,terms_valid,cache_valid,math_overflow}=saved_inputs;
            end
            run_case(index);
        end
        $fclose(fd);
        load_axis_units_case(0);run_case(-101);
        load_axis_units_case(1);run_case(-102);
        $display("PASS limit_gate_pipe vectors=%0d reset_phases=%0d latency=15 directed_axis=2 clock_hz=%0d; Python+legacy vectors plus analytic unit/last-axis cases; no STA/hardware claim",count,resets,CLOCK_HZ);
        $finish;
    end
endmodule

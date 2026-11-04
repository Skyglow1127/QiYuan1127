`timescale 1ns/1ps
// Pure combinational design-only gate test, never a robot/CAN test.
// Optional +VECTORS=FILE: each whitespace-separated row contains 13 hex
// 448-bit buses (q0 v0 a0 j q1 v1 a1 lower upper velocity acceleration jerk dq),
// dt32, previous_wire224, previous_valid, expected_allow, expected_clamp,
// expected_q16_224, expected_wire_224. Goldens must come from player_reference.
module tb_limit_gate #(
    parameter integer CLOCK_HZ = 100000000
);
    logic [447:0] q0,v0,a0,j,q1,v1,a1,lower,upper;
    logic [447:0] velocity_limit,acceleration_limit,jerk_limit,dq_max;
    logic [31:0] dt_cycles;
    logic [223:0] previous_wire;
    logic previous_wire_valid;
    wire allow,clamp;
    wire [223:0] q16,wire_values;
    wire seed_allow,seed_clamp;
    wire [223:0] seed_q16,seed_wire;
    logic [223:0] expected_q16,expected_wire;
    logic expected_allow,expected_clamp;
    integer tests=0,axis,fd,rc;
    string vector_path;
    limit_gate #(.CLOCK_HZ(CLOCK_HZ)) dut(.*);
    limit_gate #(.CLOCK_HZ(CLOCK_HZ),.STATIONARY(1)) seed_specialization(
        .allow(seed_allow),.clamp(seed_clamp),.q16(seed_q16),.wire_values(seed_wire),.*);
    task automatic base;
        begin
            q0='0;v0='0;a0='0;j='0;q1='0;v1='0;a1='0;
            lower='0;upper='0;velocity_limit='0;acceleration_limit='0;jerk_limit='0;dq_max='0;
            previous_wire='0;previous_wire_valid=1;
            expected_q16='0;expected_wire='0;dt_cycles=32'd1000000;
            for (integer k=0;k<7;k=k+1) begin
                lower[64*k+:64]=-64'sd42949672960;
                upper[64*k+:64]=64'sd42949672960;
                velocity_limit[64*k+:64]=64'sd42949672960;
                acceleration_limit[64*k+:64]=64'sd42949672960;
                jerk_limit[64*k+:64]=64'sd42949672960;
                dq_max[64*k+:64]=64'sd4294967296;
            end
        end
    endtask
    task automatic check(input string name,input logic gold_allow,input logic gold_clamp,
        input logic [223:0] gold_q,input logic [223:0] gold_wire);
        begin
            #1;
            tests=tests+1;
            if (allow !== gold_allow || clamp !== gold_clamp || q16 !== gold_q || wire_values !== gold_wire)
                $fatal(1,"%s case=%0d allow=%b/%b clamp=%b/%b q16=%h/%h wire=%h/%h",
                    name,tests,allow,gold_allow,clamp,gold_clamp,q16,gold_q,wire_values,gold_wire);
            if(v0==='0&&a0==='0&&j==='0&&q1===q0&&v1==='0&&a1==='0)begin
                if(seed_allow!==gold_allow||seed_clamp!==gold_clamp||seed_q16!==gold_q||seed_wire!==gold_wire)
                    $fatal(1,"stationary specialization differs from independent golden case=%0d",tests);
            end else if(seed_allow!==1'b0||seed_clamp!==1'b0||seed_q16!=='0||seed_wire!=='0)
                $fatal(1,"stationary specialization accepted a moving segment case=%0d",tests);
        end
    endtask
    initial begin
        base();check("stationary",1,0,'0,'0);
        base();dt_cycles=0;check("zero dt",0,0,'0,'0);
        base();j[0+:64]=64'sd42949672961;check("jerk",0,0,'0,'0);
        base();j[0+:64]=64'h8000000000000000;check("abs INT64_MIN",0,0,'0,'0);
        base();q0[0+:64]=64'sd47244640256;q1[0+:64]=64'sd47244640256;
        check("true physical overflow",0,0,'0,'0);
        base();dt_cycles=32'hffffffff;check("max dt stationary",1,0,'0,'0);
        base();dt_cycles=CLOCK_HZ;
        lower[0+:64]=-64'sd4294967296;upper[0+:64]=64'sd4294967296;
        v0[0+:64]=64'sd42949672960;a0[0+:64]=-64'sd85899345920;
        v1[0+:64]=-64'sd42949672960;a1[0+:64]=-64'sd85899345920;
        acceleration_limit[0+:64]=64'sd85899345920;dq_max[0+:64]=64'sd42949672960;
        check("interior excursion with legal endpoints",0,0,'0,'0);
        base();q0[384+:64]=64'sd429496729;q1[384+:64]=64'sd429496729;
        lower[384+:64]=0;upper[384+:64]=64'sd429496729;
        previous_wire_valid=0;expected_q16[192+:32]=32'd6553;expected_wire[192+:32]=32'd99991;
        check("nongrid upper inward",1,1,expected_q16,expected_wire);
        base();q0[384+:64]=-64'sd429496729;q1[384+:64]=-64'sd429496729;
        lower[384+:64]=-64'sd429496729;upper[384+:64]=0;
        previous_wire_valid=0;expected_q16[192+:32]=-32'sd6553;expected_wire[192+:32]=-32'sd99991;
        check("negative nongrid lower inward",1,1,expected_q16,expected_wire);
        base();q0[384+:64]=64'sd1;q1[384+:64]=64'sd1;
        lower[384+:64]=64'sd1;upper[384+:64]=64'sd4294967296;
        previous_wire_valid=0;expected_q16[192+:32]=32'd1;expected_wire[192+:32]=32'd15;
        check("positive nongrid lower inward",1,1,expected_q16,expected_wire);
        base();q0[0+:64]=-64'sd32768;q1[0+:64]=-64'sd32768;
        check("negative half to even zero",1,0,'0,'0);
        base();q0[0+:64]=-64'sd98304;q1[0+:64]=-64'sd98304;expected_q16[0+:32]=-32'sd2;
        check("negative odd half to even minus2",1,0,expected_q16,'0);
        base();q0[0+:64]=64'sd268435456;q1[0+:64]=64'sd268435456;
        expected_q16[0+:32]=32'd4096;expected_wire[0+:32]=32'd62;
        check("CAN positive half even",1,0,expected_q16,expected_wire);
        base();q0[0+:64]=64'sd805306368;q1[0+:64]=64'sd805306368;
        expected_q16[0+:32]=32'd12288;expected_wire[0+:32]=32'd188;
        check("CAN positive odd half",1,0,expected_q16,expected_wire);
        base();q0[0+:64]=-64'sd268435456;q1[0+:64]=-64'sd268435456;
        expected_q16[0+:32]=-32'sd4096;expected_wire[0+:32]=-32'sd62;
        check("CAN negative half even",1,0,expected_q16,expected_wire);
        base();q0[0+:64]=64'sd2555904;q1[0+:64]=64'sd2555904;
        lower[0+:64]=0;upper[0+:64]=64'sd2576980;expected_q16[0+:32]=32'd39;
        check("CAN inward maximum one wire step",1,1,expected_q16,'0);
        base();q0[0+:64]=64'sd65536;q1[0+:64]=64'sd65536;
        dq_max[0+:64]=64'sd65536;previous_wire[0+:32]=32'd1;
        check("decoded previous CAN DQ",0,0,'0,'0);
        base();q0[0+:64]=64'sd32767;q1[0+:64]=64'sd32769;
        v0[0+:64]=64'sd200;v1[0+:64]=64'sd200;dq_max[0+:64]=64'sd4;
        check("Q16 crosses rounding boundary DQ",0,0,'0,'0);
        base();lower[0+:64]=64'sd1;upper[0+:64]=64'sd2;q0[0+:64]=64'sd1;q1[0+:64]=64'sd1;
        check("empty Q16 domain",0,0,'0,'0);
        base();lower[0+:64]=64'sd429497;upper[0+:64]=64'sd858993;
        q0[0+:64]=64'sd644245;q1[0+:64]=64'sd644245;
        check("empty CAN domain",0,0,'0,'0);
        base();lower[0+:64]=-64'sd171798691840000;upper[0+:64]=64'sd171798691840000;
        check("Q16 overflowing domain",0,0,'0,'0);
        base();velocity_limit[0+:64]=0;check("zero bound invalid",0,0,'0,'0);
        if ($value$plusargs("VECTORS=%s",vector_path)) begin
            fd=$fopen(vector_path,"r");
            if (!fd) $fatal(1,"Cannot open golden vectors: %s",vector_path);
            while (!$feof(fd)) begin
                rc=$fscanf(fd,"%h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h\n",
                    q0,v0,a0,j,q1,v1,a1,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,
                    dt_cycles,previous_wire,previous_wire_valid,expected_allow,expected_clamp,expected_q16,expected_wire);
                if (rc==20) check("integer golden vector",expected_allow,expected_clamp,expected_q16,expected_wire);
                else if (!$feof(fd)) $fatal(1,"Malformed vector row: fields=%0d",rc);
            end
            $fclose(fd);
        end
        $display("PASS limit_gate %0d cases clock_hz=%0d",tests,CLOCK_HZ);
        $finish;
    end
endmodule

`timescale 1ns/1ps
// Synthetic-only native-product domain rejection checks; not hardware evidence.
module tb_limit_gate_pipe_narrow_domain #(
    parameter integer CLOCK_HZ = 99999001
);
    logic clk=0;
    always #5 clk=~clk;
    logic resetn=0,start=0;
    logic terms_valid=1,cache_valid=1,math_overflow=0;
    logic [31:0] dt_cycles=1;
    logic [447:0] q0,v0,a0,j,q1,v1,a1,lower,upper;
    logic [447:0] velocity_limit,acceleration_limit,jerk_limit,dq_max;
    logic [223:0] previous_wire;
    logic previous_wire_valid=1;
    logic [1343:0] qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms;
    logic [1343:0] cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c;
    wire busy,done,allow,clamp;
    wire [223:0] q16,wire_values;
    wire legacy_allow,legacy_clamp;
    wire [223:0] legacy_q16,legacy_wire;
    localparam logic signed [191:0] C=CLOCK_HZ;
    localparam logic signed [191:0] QDEN=192'sd6*C*C*C;
    localparam logic signed [191:0] VDEN=192'sd2*C*C;
    localparam logic signed [191:0] LO=-192'sd42949672960;
    localparam logic signed [191:0] HI=192'sd42949672960;
    localparam logic signed [191:0] DQ=192'sd4294967296;
    integer checks=0,jobs=0,edge_index,axis;
    limit_gate_pipe #(.CLOCK_HZ(CLOCK_HZ)) dut(.*);
    limit_gate #(.CLOCK_HZ(CLOCK_HZ)) legacy(
        .q0(q0),.v0(v0),.a0(a0),.j(j),.q1(q1),.v1(v1),.a1(a1),.dt_cycles(dt_cycles),
        .lower(lower),.upper(upper),.velocity_limit(velocity_limit),.acceleration_limit(acceleration_limit),
        .jerk_limit(jerk_limit),.dq_max(dq_max),.previous_wire(previous_wire),.previous_wire_valid(previous_wire_valid),
        .allow(legacy_allow),.clamp(legacy_clamp),.q16(legacy_q16),.wire_values(legacy_wire));
    task automatic check(input logic condition,input string message);
        begin
            checks=checks+1;
            if(condition !== 1'b1) $fatal(1,"narrow domain check failed: %s job=%0d",message,jobs);
        end
    endtask
    task automatic base;
        begin
            q0='0;v0='0;a0='0;j='0;q1='0;v1='0;a1='0;previous_wire='0;
            qp0_terms='0;qp1_terms='0;qp2_terms='0;qp3_terms='0;
            vp0_terms='0;vp1_terms='0;vp2_terms='0;ap0_terms='0;ap1_terms='0;
            terms_valid=1;cache_valid=1;math_overflow=0;previous_wire_valid=1;dt_cycles=1;
            for(axis=0;axis<7;axis=axis+1) begin
                lower[64*axis+:64]=LO[63:0];upper[64*axis+:64]=HI[63:0];
                dq_max[64*axis+:64]=DQ[63:0];
                velocity_limit[64*axis+:64]=HI[63:0];acceleration_limit[64*axis+:64]=HI[63:0];
                jerk_limit[64*axis+:64]=HI[63:0];
                cached_lower_qden[192*axis+:192]=LO*QDEN;
                cached_upper_qden[192*axis+:192]=HI*QDEN;
                cached_dq_qden[192*axis+:192]=DQ*QDEN;
                cached_vmax_vden[192*axis+:192]=HI*VDEN;
                cached_amax_c[192*axis+:192]=HI*C;
            end
        end
    endtask
    // mode1: positive non-sign-extended term; mode2: corrupt cache high sign;
    // mode3: unknown high bit. Low162 values and flags are still legal/valid.
    task automatic run_case(input integer mode);
        logic expected_allow;
        begin
            jobs=jobs+1;expected_allow=(mode==0);
            @(negedge clk);base();
            if(mode==1) qp0_terms[162]=1'b1;
            if(mode==2) cached_dq_qden[191]=1'b1;
            if(mode==3) qp0_terms[162]=1'bx;
            #1;
            check(legacy_allow && !legacy_clamp && legacy_q16==='0 && legacy_wire==='0,
                  "raw stationary record is legal according to unchanged legacy gate");
            check(terms_valid===1'b1 && cache_valid===1'b1 && math_overflow===1'b0,
                  "rejection cannot be attributed to valid/overflow flags");
            start=1;
            @(posedge clk);#1;
            check(busy && !done && !allow && !clamp,"E0 accepted without early decision");
            check(dut.bad_job === !expected_allow,"E0 native-product high-extension check itself detects corruption");
            @(negedge clk);start=0;
            for(edge_index=1;edge_index<=15;edge_index=edge_index+1) begin
                @(posedge clk);#1;
                if(edge_index<15) check(busy && !done && !allow && q16==='0 && wire_values==='0,
                                       "all jobs retain original E15 publication");
                else check(!busy && done && allow===expected_allow && !clamp && q16==='0 && wire_values==='0,
                           "E15 allow/reject without leaking truncated legal outputs");
            end
            @(posedge clk);#1;check(!done && !busy,"done remains a one-edge pulse");
        end
    endtask
    initial begin
        base();repeat(3) @(posedge clk);
        @(negedge clk);resetn=1;
        run_case(0);run_case(1);run_case(2);run_case(3);
        $display("PASS narrow_product_domain jobs=%0d checks=%0d clock_hz=%0d E15; all flags valid, legacy raw legal",jobs,checks,CLOCK_HZ);
        $finish;
    end
endmodule

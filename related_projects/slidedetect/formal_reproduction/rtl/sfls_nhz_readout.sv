`timescale 1ns/1ps
// Optional fine readout of an ATOMIC centered-sum snapshot from sfls_core.
// Q = sum((2*j-M+1)*timestamp_j). No changes to the validated legacy core.
// Assert snapshot_valid only when ready. Invalid requests cause a sticky fault.
module sfls_nhz_readout #(
    parameter integer WINDOW_SIZE=96,
    parameter integer CLOCK_HZ=200000000,
    parameter integer EDGE_STRIDE=1,
    parameter integer ACC_W=96
)(
    input wire clk, rst,
    input wire snapshot_valid,
    input wire signed [ACC_W-1:0] snapshot_q,
    output wire ready,
    output reg result_valid,
    output reg [63:0] frequency_nHz,
    output reg fault
);
    localparam [ACC_W-1:0] M_W=WINDOW_SIZE;
    localparam [ACC_W-1:0] K=M_W*(M_W*M_W-1)/6;
    localparam [ACC_W-1:0] NUMERATOR=K*CLOCK_HZ*EDGE_STRIDE*1000000000;
    reg start;
    reg [ACC_W-1:0] denominator;
    wire busy, done, divide_by_zero;
    wire [ACC_W-1:0] quotient;
    assign ready=!(busy || start || fault);
    udiv_round #(.W(ACC_W)) divider(
        .clk(clk),.rst(rst),.start(start),.numerator(NUMERATOR),.denominator(denominator),
        .busy(busy),.done(done),.divide_by_zero(divide_by_zero),.quotient(quotient));
    always @(posedge clk) begin
        if(rst) begin
            start<=0;denominator<=0;result_valid<=0;frequency_nHz<=0;fault<=0;
        end else begin
            start<=0;result_valid<=0;
            if(snapshot_valid) begin
                if(!ready || snapshot_q<=0) fault<=1;
                else begin denominator<=$unsigned(snapshot_q);start<=1;end
            end
            if(done) begin
                if(divide_by_zero || |quotient[ACC_W-1:64]) fault<=1;
                else if(!fault && !(snapshot_valid && (!ready || snapshot_q<=0))) begin
                    frequency_nHz<=quotient[63:0];result_valid<=1;
                end
            end
        end
    end
endmodule

`timescale 1ns/1ps
// Unsigned restoring divider. Fixed W-cycle latency, round to nearest.
module udiv_round #(parameter integer W=96)(
    input wire clk, input wire rst,
    input wire start, input wire [W-1:0] numerator, denominator,
    output reg busy, output reg done, output reg divide_by_zero,
    output reg [W-1:0] quotient
);
    reg [W-1:0] dividend, divisor, q;
    reg [W:0] remainder;
    reg [$clog2(W+1)-1:0] left;
    wire [W:0] trial = {remainder[W-1:0],dividend[W-1]};
    wire take_bit = trial >= {1'b0,divisor};
    wire [W:0] next_rem = take_bit ? trial-{1'b0,divisor} : trial;
    wire [W-1:0] next_q = {q[W-2:0],take_bit};
    always @(posedge clk) begin
        if(rst) begin
            busy<=0; done<=0; divide_by_zero<=0; quotient<=0;
            dividend<=0; divisor<=0; q<=0; remainder<=0; left<=0;
        end else begin
            done<=0;
            if(start && !busy) begin
                divide_by_zero <= denominator==0;
                if(denominator==0) begin quotient<=0; done<=1; end
                else begin
                    busy<=1; dividend<=numerator; divisor<=denominator;
                    q<=0; remainder<=0; left<=W;
                end
            end else if(busy) begin
                dividend<={dividend[W-2:0],1'b0};
                remainder<=next_rem; q<=next_q; left<=left-1'b1;
                if(left==1) begin
                    quotient <= next_q + ((next_rem<<1)>={1'b0,divisor});
                    busy<=0; done<=1;
                end
            end
        end
    end
endmodule

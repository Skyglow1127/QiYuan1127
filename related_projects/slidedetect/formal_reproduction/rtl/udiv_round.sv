`timescale 1ns/1ps
// Unsigned restoring divider. W iterations plus one output/rounding cycle.
// ROUND_NEAREST=0 preserves the integer floor needed for mean timestamps.
module udiv_round #(parameter integer W=96, parameter integer ROUND_NEAREST=1)(
    input wire clk, input wire rst,
    input wire start, input wire [W-1:0] numerator, denominator,
    output reg busy, output reg done, output reg divide_by_zero,
    output reg [W-1:0] quotient
);
    reg [W-1:0] dividend, divisor, q;
    reg [W:0] remainder;
    reg [$clog2(W+1)-1:0] left;
    reg finish_pending;
    wire [W:0] trial = {remainder[W-1:0],dividend[W-1]};
    wire take_bit = trial >= {1'b0,divisor};
    wire [W:0] next_rem = take_bit ? trial-{1'b0,divisor} : trial;
    wire [W-1:0] next_q = {q[W-2:0],take_bit};
    always @(posedge clk) begin
        if(rst) begin
            busy<=0; done<=0; divide_by_zero<=0; quotient<=0;
            dividend<=0; divisor<=0; q<=0; remainder<=0; left<=0; finish_pending<=0;
        end else begin
            done<=0;
            if(start && !busy) begin
                divide_by_zero <= denominator==0;
                if(denominator==0) begin quotient<=0; done<=1; end
                else begin
                    busy<=1; dividend<=numerator; divisor<=denominator;
                    q<=0; remainder<=0; left<=W; finish_pending<=0;
                end
            end else if(busy && finish_pending) begin
                // Round registered division results rather than cascading
                // compare/subtract/round through the final iteration.
                quotient <= q + (ROUND_NEAREST && ((remainder<<1)>={1'b0,divisor}));
                busy<=0; done<=1; finish_pending<=0;
            end else if(busy) begin
                dividend<={dividend[W-2:0],1'b0};
                remainder<=next_rem; q<=next_q; left<=left-1'b1;
                if(left==1) begin
                    finish_pending<=1;
                end
            end
        end
    end
endmodule

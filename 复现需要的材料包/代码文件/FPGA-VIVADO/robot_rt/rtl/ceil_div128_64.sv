`timescale 1ns/1ps
// Shared exact admission-only divider; no transport or enable I/O.
// start E0 accepts only when !busy. E1..E128 divide; E129 rounds;
// E130 asserts done and checks origin + ceil against the uint64 domain.
module ceil_div128_64 (
    input logic clk, resetn, start,
    input logic [127:0] dividend,
    input logic [63:0] divisor, origin,
    output logic busy, done, invalid, absolute_overflow,
    output logic [127:0] ceil_quotient,
    output logic [63:0] absolute_cycle
);
    localparam [1:0] IDLE=0,DIVIDE=1,ROUND=2,FINISH=3;
    logic [1:0] state;
    logic [127:0] dividend_shift,quotient;
    logic [63:0] remainder_work,divisor_work,origin_work;
    logic [7:0] steps;
    logic invalid_work;
    wire [64:0] trial={remainder_work,dividend_shift[127]};
    wire subtract=trial>={1'b0,divisor_work};
    wire [64:0] reduced=subtract ? trial-{1'b0,divisor_work} : trial;
    wire [128:0] rounded={1'b0,quotient}+(remainder_work!=0);
    wire [64:0] absolute_low={1'b0,ceil_quotient[63:0]}+{1'b0,origin_work};
    always_ff @(posedge clk) begin
        if(!resetn) begin
            state<=IDLE;busy<=0;done<=0;invalid<=0;absolute_overflow<=0;
            ceil_quotient<=0;absolute_cycle<=0;dividend_shift<=0;quotient<=0;
            remainder_work<=0;divisor_work<=1;origin_work<=0;steps<=0;invalid_work<=0;
        end else begin
            done<=0;
            case(state)
                IDLE:if(start)begin
                    busy<=1;invalid<=0;absolute_overflow<=0;absolute_cycle<=0;ceil_quotient<=0;
                    dividend_shift<=dividend;quotient<=0;remainder_work<=0;steps<=128;
                    // Keep deterministic latency and arithmetic even for bad D.
                    invalid_work<=divisor==0;divisor_work<=divisor==0?64'd1:divisor;
                    origin_work<=origin;state<=DIVIDE;
                end
                DIVIDE:begin
                    remainder_work<=reduced[63:0];
                    quotient<={quotient[126:0],subtract};
                    dividend_shift<={dividend_shift[126:0],1'b0};steps<=steps-1;
                    if(steps==1)state<=ROUND;
                end
                ROUND:begin
                    // With positive D, ceil(X/D)<=X<=UINT128_MAX. A carry
                    // here is an internal arithmetic fault, never truncation.
                    invalid_work<=invalid_work||rounded[128];
                    ceil_quotient<=invalid_work||rounded[128]?128'd0:rounded[127:0];
                    state<=FINISH;
                end
                FINISH:begin
                    invalid<=invalid_work;
                    absolute_overflow<=!invalid_work&&((ceil_quotient[127:64]!=0)||absolute_low[64]);
                    absolute_cycle<=!invalid_work&&(ceil_quotient[127:64]==0)&&!absolute_low[64]?absolute_low[63:0]:64'd0;
                    busy<=0;done<=1;state<=IDLE;
                end
                default:begin state<=IDLE;busy<=0;invalid<=1;done<=1;absolute_cycle<=0;ceil_quotient<=0;end
            endcase
        end
    end
endmodule

`timescale 1ns / 1ps
`default_nettype none

// A 100 MHz clock and the default divisor produce a 1 Hz LED square wave.
// rst_n asserts asynchronously; the system must release it synchronously.
// Contract: HALF_PERIOD_CYCLES >= 1. Toggle after exactly that many rising edges.
module led_blink #(
    parameter integer HALF_PERIOD_CYCLES = 50_000_000
) (
    input  wire clk,
    input  wire rst_n,
    output reg  led
);
    localparam integer COUNT_WIDTH = (HALF_PERIOD_CYCLES <= 1)
                                    ? 1 : $clog2(HALF_PERIOD_CYCLES);
    localparam [COUNT_WIDTH-1:0] LAST_COUNT = COUNT_WIDTH'(HALF_PERIOD_CYCLES - 1);
    reg [COUNT_WIDTH-1:0] count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= {COUNT_WIDTH{1'b0}};
            led   <= 1'b0;
        end else if (count == LAST_COUNT) begin
            count <= {COUNT_WIDTH{1'b0}};
            led   <= ~led;
        end else begin
            count <= count + 1'b1;
        end
    end
endmodule

`default_nettype wire

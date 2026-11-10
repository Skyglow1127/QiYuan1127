`timescale 1ns / 1ps
`default_nettype none
module fsr_top (
    input wire clk_25mhz,
    input wire [1:0] fsr_do,
    output wire fan_en_b
);
    // BD clock port is buffered automatically by Vivado's top-level synthesis.
    fsr_bd system_i (.clk_25mhz(clk_25mhz), .fsr_do(fsr_do));
    assign fan_en_b = 1'b0;
endmodule
`default_nettype wire

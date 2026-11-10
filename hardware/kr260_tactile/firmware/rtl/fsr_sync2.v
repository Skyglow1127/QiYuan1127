`timescale 1ns / 1ps
`default_nettype none
// Two independent asynchronous digital comparator signals, not a coherent bus.
module fsr_sync2 (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME clk, ASSOCIATED_RESET resetn" *)
    input wire clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 resetn RST" *)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME resetn, POLARITY ACTIVE_LOW" *)
    input wire resetn,
    input wire [1:0] din,
    output wire [1:0] dout
);
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0] meta = 2'b00;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0] synced = 2'b00;
    always @(posedge clk) begin
        if (!resetn) begin
            meta <= 2'b00;
            synced <= 2'b00;
        end else begin
            meta <= din;
            synced <= meta;
        end
    end
    assign dout = synced;
endmodule
`default_nettype wire

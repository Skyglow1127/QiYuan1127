`timescale 1ns / 1ps
`default_nettype none

// KR260 carrier HPA_CLK0P_CLK: 25 MHz, K26 pin C3, LVCMOS18.
// No PS clocks or software initialization are required by this PL design.
module kr260_ila_top (
    input  wire clk_25mhz,
    output wire fan_en_b
);
    wire clk_input;
    wire clk;
    IBUF input_clock (.I(clk_25mhz), .O(clk_input));
    BUFG global_clock (.I(clk_input), .O(clk));

    // FPGA configuration initializes this register. Release reset on the
    // local clock only, after 255 complete clock edges.
    reg [7:0] startup_count = 8'd0;
    (* MARK_DEBUG = "TRUE" *) reg rst_n = 1'b0;
    always @(posedge clk) begin
        if (!(&startup_count)) startup_count <= startup_count + 1'b1;
        if (startup_count == 8'd254) rst_n <= 1'b1;
    end

    (* MARK_DEBUG = "TRUE" *) reg [15:0] cycle_count = 16'd0;
    (* MARK_DEBUG = "TRUE" *) wire blink;
    always @(posedge clk)
        if (!rst_n) cycle_count <= 16'd0;
        else cycle_count <= cycle_count + 1'b1;

    // 64 cycles per half-period: 2.56 us at 25 MHz, easily visible in ILA.
    led_blink #(.HALF_PERIOD_CYCLES(64)) u_blink (
        .clk(clk), .rst_n(rst_n), .led(blink)
    );

    // Keep the carrier fan enabled (active low); no external LED required.
    assign fan_en_b = 1'b0;
endmodule

`default_nettype wire

`timescale 1ns / 1ps
`default_nettype none
module tb_kr260_ila;
    reg clk_25mhz = 1'b0;
    wire fan_en_b;
    integer edges = 0;
    integer elapsed = 0;
    integer checked = 0;
    kr260_ila_top dut (.clk_25mhz(clk_25mhz), .fan_en_b(fan_en_b));
    always #20 clk_25mhz = ~clk_25mhz;

    // The oracle uses elapsed input clock cycles, independent of DUT state.
    always @(posedge clk_25mhz) begin
        edges = edges + 1;
        if (edges > 255) elapsed = elapsed + 1;
        #1;
        if (fan_en_b !== 1'b0) $fatal(1, "Fan must remain enabled");
        if (dut.rst_n !== (edges >= 255))
            $fatal(1, "Startup reset release mismatch edge=%0d", edges);
        if (dut.cycle_count !== (elapsed % 65536))
            $fatal(1, "Counter mismatch elapsed=%0d actual=%0d", elapsed, dut.cycle_count);
        if (dut.blink !== ((elapsed / 64) % 2))
            $fatal(1, "Blink mismatch elapsed=%0d actual=%b", elapsed, dut.blink);
        checked = checked + 1;
        if (checked == 70000) begin
            $display("TEST_PASS: KR260 checks=70000 startup_cycles=255 counter_wrap=verified half_period=64 clock=25MHz");
            $finish;
        end
    end
    initial begin
        #3000000;
        $fatal(1, "WATCHDOG_TIMEOUT: KR260 simulation exceeded 3 ms");
    end
endmodule
`default_nettype wire

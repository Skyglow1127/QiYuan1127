`timescale 1ns / 1ps
`default_nettype none

module blink_checker #(
    parameter integer HALF_PERIOD = 1
) (
    input wire clk,
    input wire rst_n,
    output wire led
);
    integer cycles_since_reset = 0;
    integer checked_cycles = 0;
    reg expected;

    led_blink #(.HALF_PERIOD_CYCLES(HALF_PERIOD)) dut (
        .clk(clk), .rst_n(rst_n), .led(led)
    );

    // Independent oracle: derive the waveform from elapsed clock cycles.
    // Checking every cycle catches early/late toggles as well as wrong polarity.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycles_since_reset = 0;
            #1;
            if (led !== 1'b0)
                $fatal(1, "RESET_FAIL divisor=%0d led=%b", HALF_PERIOD, led);
        end else begin
            cycles_since_reset = cycles_since_reset + 1;
            expected = ((cycles_since_reset / HALF_PERIOD) % 2) != 0;
            #1;
            if (led !== expected)
                $fatal(1, "WAVEFORM_FAIL divisor=%0d cycle=%0d expected=%b actual=%b",
                       HALF_PERIOD, cycles_since_reset, expected, led);
            checked_cycles = checked_cycles + 1;
        end
    end
endmodule

module tb_led_blink;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    wire led_1, led_3, led_5;
    always #5 clk = ~clk;

    blink_checker #(.HALF_PERIOD(1)) check_1 (clk, rst_n, led_1);
    blink_checker #(.HALF_PERIOD(3)) check_3 (clk, rst_n, led_3);
    blink_checker #(.HALF_PERIOD(5)) check_5 (clk, rst_n, led_5);

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1'b1;
        repeat (17) @(negedge clk);
        #2 rst_n = 1'b0;
        #2;
        if ({led_1, led_3, led_5} !== 3'b000)
            $fatal(1, "Asynchronous reset did not clear all LEDs between clock edges");
        repeat (2) @(negedge clk);
        rst_n = 1'b1;
        repeat (43) @(negedge clk);
        #2 rst_n = 1'b0;
        #2;
        if ({led_1, led_3, led_5} !== 3'b000)
            $fatal(1, "Repeated asynchronous reset failed");
        repeat (2) @(negedge clk);
        rst_n = 1'b1;
        repeat (20) @(negedge clk);
        #1;
        if (check_1.checked_cycles != 80 || check_3.checked_cycles != 80 ||
            check_5.checked_cycles != 80)
            $fatal(1, "Not all required checks completed");
        $display("TEST_PASS: divisors=1,3,5 checks=240 reset_restarts=2 clock=100MHz");
        $finish;
    end

    initial begin
        #2000;
        $fatal(1, "WATCHDOG_TIMEOUT: simulation exceeded 2 us");
    end
endmodule

`default_nettype wire

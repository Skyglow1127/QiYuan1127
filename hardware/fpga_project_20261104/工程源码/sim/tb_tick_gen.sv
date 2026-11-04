`timescale 1ns/1ps
module tb_tick_gen #(parameter integer CLOCK_HZ=100000000);
    logic clk = 0;
    always #5 clk = ~clk;
    logic resetn = 0, config_commit = 0, enable = 0;
    logic [63:0] rate_num = 0, rate_den = 0;
    wire tick, config_error, quiet_window;
    wire [63:0] tick_index, cycle, origin;
    tick_gen #(.CLOCK_HZ(CLOCK_HZ)) dut(.*);
    integer checks = 0;
    localparam logic[63:0] MAX_DEN=64'hffffffffffffffff/CLOCK_HZ;
    localparam logic[63:0] MAX_THRESHOLD=MAX_DEN*CLOCK_HZ;
    localparam logic[63:0] CARRY_NUM=(MAX_THRESHOLD>>8)-1;

    task automatic check(input logic condition, input string message);
        begin
            checks = checks + 1;
            if (condition !== 1'b1) $fatal(1, "%s (cycle=%0d tick_index=%0d)", message, cycle, tick_index);
        end
    endtask

    task automatic reset_core;
        begin
            @(negedge clk); resetn = 0; enable = 0; config_commit = 0;
            repeat (3) @(posedge clk);
            #1;
            check(cycle == 0 && tick_index == 0 && origin == 0 && !tick && !config_error, "hardware reset state");
            @(negedge clk); resetn = 1;
        end
    endtask

    task automatic configure(input logic [63:0] n, input logic [63:0] d, input logic good);
        logic [63:0] previous_origin;
        begin
            @(negedge clk); enable = 0; rate_num = n; rate_den = d;
            previous_origin = origin;
            config_commit = 1;
            @(posedge clk); #1;
            check(config_error == !good, "hardware configuration validity");
            if (good) begin
                check(origin == cycle && tick_index == 0 && dut.phase == 0 && !tick, "commit origin and phase");
            end else check(origin == previous_origin, "invalid configuration must retain origin");
            @(negedge clk); config_commit = 0; enable = good;
        end
    endtask

    function automatic logic [63:0] expected_relative(input logic [63:0] index, input logic [63:0] n, input logic [63:0] d);
        logic [127:0] value;
        begin
            value = {64'd0, index} * d * {96'd0,CLOCK_HZ[31:0]};
            expected_relative = (value + n - 1) / n;
        end
    endfunction

    task automatic await_tick(input logic [63:0] index, input logic [63:0] n, input logic [63:0] d);
        logic [63:0] target;
        begin
            target = origin + expected_relative(index, n, d);
            while (cycle < target) begin
                @(posedge clk); #1;
                if (cycle < target) check(!tick, "no early tick");
            end
            check(tick && tick_index == index && cycle == target, "exact analytical event edge");
            @(negedge clk);
        end
    endtask

    // Verification-only jump to the exact pre-event mathematical state. The
    // next actual RTL clock then checks 65-bit DDS behavior at a long horizon,
    // without simulating 10^17 otherwise idle clock edges.
    task automatic jump_before_tick(input logic [63:0] index, input logic [63:0] n, input logic [63:0] d);
        logic [63:0] relative_cycle;
        logic [127:0] residual;
        begin
            relative_cycle = expected_relative(index, n, d) - 1;
            residual = {64'd0, relative_cycle} * n - ({64'd0, index}-1) * d * {96'd0,CLOCK_HZ[31:0]};
            @(negedge clk);
            dut.cycle = origin + relative_cycle;
            dut.tick_index = index - 1;
            dut.phase = residual[63:0];
            @(posedge clk); #1;
            check(tick && tick_index == index && cycle == origin + relative_cycle + 1, "long-horizon event");
            check(dut.phase < n, "long-horizon phase error strictly below one clock");
        end
    endtask

    initial begin
        reset_core();
        configure(20, 1, 1);
        await_tick(1, 20, 1);
        check(cycle-origin == (CLOCK_HZ+19)/20, "20Hz exact first tick at selected clock");
        jump_before_tick(64'd1000000000000, 20, 1);
        reset_core();
        configure(275, 2, 1);
        await_tick(1, 275, 2);
        await_tick(2, 275, 2);
        await_tick(3, 275, 2);
        check(dut.phase < 275, "137.5Hz phase error bound");
        jump_before_tick(64'd1000000000000, 275, 2);
        reset_core();
        configure(CLOCK_HZ, 256, 1);
        await_tick(1, CLOCK_HZ, 256);
        @(negedge clk); enable = 0;
        repeat (300) @(posedge clk);
        #1; check(!tick && tick_index == 2, "enable never pauses phase/index");
        @(negedge clk); enable = 1;
        await_tick(3, CLOCK_HZ, 256);
        check(cycle-origin == 768, "enable never reorigins DDS");
        @(negedge clk); config_commit = 1; rate_num = 20; rate_den = 1;
        @(posedge clk); #1; check(config_error && dut.numerator == CLOCK_HZ, "active reconfiguration rejected atomically");
        @(negedge clk); config_commit = 0;
        configure(0, 1, 0);
        configure(1, 0, 0);
        configure(CLOCK_HZ+1, 256, 0); // ceil first gap256, but floor gap255.
        configure(CLOCK_HZ+1, 1, 0);
        configure(1, MAX_DEN+1, 0); // uint64 threshold overflow.
        configure(1, 64'hffffffff/CLOCK_HZ+1, 0); // ceil gap4_300_000_000 exceeds uint32.
        configure(1, 64'hffffffff/CLOCK_HZ, 1);
        jump_before_tick(1, 1, 64'hffffffff/CLOCK_HZ);
        reset_core();
        configure(CARRY_NUM, MAX_DEN, 1);
        jump_before_tick(1, CARRY_NUM, MAX_DEN);
        check(dut.phase == 128'd257*CARRY_NUM-MAX_THRESHOLD, "65-bit carry survives sum beyond uint64");
        jump_before_tick(64'd1000000, CARRY_NUM, MAX_DEN);
        @(negedge clk); dut.cycle = 64'hffffffffffffffff;
        @(posedge clk); #1;
        check(cycle == 64'hffffffffffffffff && config_error && !tick, "cycle overflow saturates and faults");
        reset_core();
        $display("PASS tb_tick_gen checks=%0d clock_hz=%0d design_only=1 physical_frequency_validated=0", checks,CLOCK_HZ);
        $finish;
    end
    initial begin
        #200000000;
        $fatal(1, "testbench timeout");
    end
endmodule

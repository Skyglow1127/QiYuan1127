`timescale 1ns/1ps
// Rational DDS, explicit nominal CLOCK_HZ. No physical execution-rate claim.
// Config checks are synthesizable constant multiply/shift/subtract/compare.
// Timing and resource closure require the selected build reports.
// enable must be configured, never RUN/ARM. It gates only tick pulses; phase
// and tick_index continue through a deasserted enable until a new commit/reset.
module tick_gen #(
    parameter integer CLOCK_HZ = 100000000
) (
    input  logic        clk,
    input  logic        resetn,
    input  logic        config_commit,
    input  logic        enable,
    input  logic [63:0] rate_num,
    input  logic [63:0] rate_den,
    output logic        tick,
    output logic [63:0] tick_index,
    output logic [63:0] cycle,
    output logic [63:0] origin,
    output logic        config_error,
    // True only when no DDS crossing can occur in the next128 clocks.
    // Consumers must separately reserve a tick already registered on tick.
    output wire         quiet_window
);
    initial begin
        if(CLOCK_HZ<=0) $fatal(1,"CLOCK_HZ must be a positive signed 32-bit integer");
    end
    localparam logic [63:0] U64_MAX = 64'hffff_ffff_ffff_ffff;
    logic [63:0] phase;
    logic [63:0] numerator;
    logic [63:0] threshold;
    logic        configured;
    wire [95:0] threshold_wide = {32'b0, rate_den} * {64'd0, CLOCK_HZ[31:0]};
    // ceil(T/N)<=2^32-1 iff T<=N*(2^32-1), with a 96-bit product.
    wire [95:0] maximum_gap_product = ({32'b0, rate_num} << 32) - {32'b0, rate_num};
    wire input_valid = rate_num != 0 && rate_den != 0 &&
                       threshold_wide[95:64] == 0 &&
                       rate_num <= (threshold_wide[63:0] >> 8) &&
                       threshold_wide <= maximum_gap_product;
    wire [64:0] phase_sum = {1'b0, phase} + {1'b0, numerator};
    wire [127:0] background_horizon = {64'd0,phase} + ({64'd0,numerator} << 7);
    assign quiet_window = configured && (background_horizon < {64'd0,threshold});

    always_ff @(posedge clk) begin
        if (!resetn) begin
            tick <= 1'b0;
            tick_index <= 64'd0;
            cycle <= 64'd0;
            origin <= 64'd0;
            phase <= 64'd0;
            numerator <= 64'd0;
            threshold <= 64'd0;
            configured <= 1'b0;
            config_error <= 1'b0;
        end else begin
            tick <= 1'b0;
            if (cycle == U64_MAX) begin
                // Saturate, do not wrap the time domain or continue emitting.
                configured <= 1'b0;
                config_error <= 1'b1;
            end else begin
                cycle <= cycle + 64'd1;
                if (configured) begin
                    if (phase_sum >= {1'b0, threshold}) begin
                        phase <= phase_sum - {1'b0, threshold};
                        if (tick_index == U64_MAX) begin
                            configured <= 1'b0;
                            config_error <= 1'b1;
                        end else begin
                            tick_index <= tick_index + 64'd1;
                            tick <= enable;
                        end
                    end else begin
                        phase <= phase_sum[63:0];
                    end
                end
                if (config_commit) begin
                    if (enable || !input_valid) begin
                        // Reject atomically; retain the prior rate and origin.
                        config_error <= 1'b1;
                    end else begin
                        numerator <= rate_num;
                        threshold <= threshold_wide[63:0];
                        configured <= 1'b1;
                        phase <= 64'd0;
                        tick_index <= 64'd0;
                        // The origin is this edge's newly incremented CYCLE.
                        origin <= cycle + 64'd1;
                        config_error <= 1'b0;
                        tick <= 1'b0;
                    end
                end
            end
        end
    end
endmodule

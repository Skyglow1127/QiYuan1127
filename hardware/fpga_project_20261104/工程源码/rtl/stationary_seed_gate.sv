`timescale 1ns/1ps
// OPTIONAL raw49 width candidate only. Exact two-state specialization of limit_gate(STATIONARY=1)
// for q0=q1=seed_q, v0=a0=j=v1=a1=0, dt=1, previous_wire_valid=0.
// Q32.32 signed64 in; signed32 Q16.16 and mdeg/um out. Axis0 is low.
// Unknown input bits additionally fail closed; legacy gate is not X-safe.
module stationary_seed_gate #(
    parameter integer CLOCK_HZ = 100000000
) (
    input logic [447:0] seed_q, lower, upper,
    input logic [447:0] velocity_limit, acceleration_limit, jerk_limit, dq_max,
    output logic allow, clamp, bounds_valid,
    output logic [223:0] q16, wire_values
);
    generate if (CLOCK_HZ <= 0) begin : invalid_clock
        initial $fatal(1, "CLOCK_HZ must be a positive signed integer");
    end endgenerate
    logic [6:0] good_lane, clamp_lane, bounds_lane;
    logic [223:0] q_candidate, wire_candidate;
    for (genvar axis=0; axis<7; axis=axis+1) begin : lane
        localparam logic signed [33:0] QMIN = -34'sd2147483648;
        localparam logic signed [33:0] QMAX = 34'sd2147483647;
        localparam logic signed [38:0] WMIN = -39'sd2147483648;
        localparam logic signed [38:0] WMAX = 39'sd2147483647;
        wire signed [63:0] raw_q = seed_q[64*axis +:64];
        wire signed [48:0] q = raw_q[48:0];
        wire signed [63:0] raw_lo = lower[64*axis +:64];
        wire signed [48:0] lo = raw_lo[48:0];
        wire signed [63:0] raw_hi = upper[64*axis +:64];
        wire signed [48:0] hi = raw_hi[48:0];
        wire signed [63:0] vl = velocity_limit[64*axis +:64];
        wire signed [63:0] al = acceleration_limit[64*axis +:64];
        wire signed [63:0] jl = jerk_limit[64*axis +:64];
        wire signed [63:0] dq = dq_max[64*axis +:64];
        logic signed [33:0] code_lo, code_hi, nearest_q, code;
        logic signed [34:0] code_correction;
        logic signed [49:0] q_decoded;
        logic signed [69:0] scaled_lo, scaled_hi;
        logic signed [53:0] scaled_code;
        logic signed [38:0] wire_lo, wire_hi, nearest_wire, encoded;
        logic signed [39:0] wire_correction;
        logic signed [70:0] wire_decoded;
        logic lane_ok, raw_fits, bounds_ok;
        // These are compile-time branches: no runtime 192-bit multiplier or
        // variable unit selector. raw49*unit fits signed70; code34*unit
        // fits signed54. Raw49 narrowing is guarded independently below.
        if (axis == 6) begin : metres
            always_comb begin
                scaled_lo = $signed({{21{lo[48]}},lo}) * 70'sd1000000;
                scaled_hi = $signed({{21{hi[48]}},hi}) * 70'sd1000000;
                scaled_code = $signed({{20{code[33]}},code}) * 54'sd1000000;
            end
        end else begin : degrees
            always_comb begin
                scaled_lo = $signed({{21{lo[48]}},lo}) * 70'sd1000;
                scaled_hi = $signed({{21{hi[48]}},hi}) * 70'sd1000;
                scaled_code = $signed({{20{code[33]}},code}) * 54'sd1000;
            end
        end
        always_comb begin
            // Signed floor + unsigned nonnegative low remainder is exact for
            // negative numbers too; high-bit rejection precedes any acceptance.
            raw_fits = (raw_q[63:49] == {15{raw_q[48]}}) &&
                       (raw_lo[63:49] == {15{raw_lo[48]}}) &&
                       (raw_hi[63:49] == {15{raw_hi[48]}});
            code_lo = $signed({lo[48],lo[48:16]}) +
                      ((lo[15:0] != 16'd0) ? 34'sd1 : 34'sd0);
            code_hi = $signed({hi[48],hi[48:16]});
            nearest_q = $signed({q[48],q[48:16]});
            if (q[15:0] > 16'd32768 ||
                (q[15:0] == 16'd32768 && nearest_q[0]))
                nearest_q = nearest_q + 34'sd1;
            code = nearest_q < code_lo ? code_lo :
                   (nearest_q > code_hi ? code_hi : nearest_q);
            code_correction = $signed({code[33],code}) -
                              $signed({nearest_q[33],nearest_q});
            q_decoded = $signed({{16{code[33]}},code}) <<< 16;
            wire_lo = $signed({scaled_lo[69],scaled_lo[69:32]}) +
                      ((scaled_lo[31:0] != 32'd0) ? 39'sd1 : 39'sd0);
            wire_hi = $signed({scaled_hi[69],scaled_hi[69:32]});
            // scaled_code is signed54; divide-by-65536 floor is signed38.
            // Extend to39 before RNE increment, retaining the positive carry.
            nearest_wire = $signed({scaled_code[53],scaled_code[53:16]});
            if (scaled_code[15:0] > 16'd32768 ||
                (scaled_code[15:0] == 16'd32768 && nearest_wire[0]))
                nearest_wire = nearest_wire + 39'sd1;
            encoded = nearest_wire < wire_lo ? wire_lo :
                      (nearest_wire > wire_hi ? wire_hi : nearest_wire);
            wire_correction = $signed({encoded[38],encoded}) -
                              $signed({nearest_wire[38],nearest_wire});
            wire_decoded = $signed({{32{encoded[38]}},encoded}) <<< 32;
            // Pure bounds certificate: independent of seed_q and derivative
            // values. Reuses the exact raw64 guards/quantized interval nodes.
            bounds_ok = ((raw_lo[63:49] == {15{raw_lo[48]}}) &&
                         (raw_hi[63:49] == {15{raw_hi[48]}}) && lo < hi &&
                         code_lo <= code_hi && code_lo >= QMIN && code_hi <= QMAX &&
                         wire_lo <= wire_hi && wire_lo >= WMIN && wire_hi <= WMAX);
            bounds_lane[axis] = (bounds_ok === 1'b1) &&
                ((^{raw_lo,raw_hi}) !== 1'bx);
            lane_ok = (raw_fits && lo < hi && vl > 64'sd0 && al > 64'sd0 &&
                       jl > 64'sd0 && dq > 64'sd0 && q >= lo && q <= hi &&
                       code_lo <= code_hi && code_lo >= QMIN && code_hi <= QMAX &&
                       code_correction >= -35'sd1 && code_correction <= 35'sd1 &&
                       q_decoded >= $signed({lo[48],lo}) &&
                       q_decoded <= $signed({hi[48],hi}) &&
                       wire_lo <= wire_hi && wire_lo >= WMIN && wire_hi <= WMAX &&
                       wire_correction >= -40'sd1 && wire_correction <= 40'sd1 &&
                       wire_decoded >= $signed({scaled_lo[69],scaled_lo}) &&
                       wire_decoded <= $signed({scaled_hi[69],scaled_hi}));
            good_lane[axis] = (lane_ok === 1'b1) &&
                ((^{raw_q,raw_lo,raw_hi,vl,al,jl,dq}) !== 1'bx);
            clamp_lane[axis] = (code != nearest_q) || (encoded != nearest_wire);
            q_candidate[32*axis +:32] = code[31:0];
            wire_candidate[32*axis +:32] = encoded[31:0];
        end
    end
    always_comb begin
        bounds_valid = ((&bounds_lane) === 1'b1);
        allow=1'b0; clamp=1'b0; q16='0; wire_values='0;
        if ((&good_lane) === 1'b1) begin
            allow=1'b1;
            clamp=|clamp_lane;
            q16=q_candidate;
            wire_values=wire_candidate;
        end
    end
endmodule

`timescale 1ns/1ps
// PRIVATE CANDIDATE. No top integration or timing/resource claim.
// E0 accepts start; E1..35 execute seven axes, five phases per axis;
// E36 publishes all outputs atomically. Busy start is ignored.
module stationary_seed_gate_seq #(
    parameter integer CLOCK_HZ = 100000000
) (
    input logic clk, resetn, start,
    input logic [447:0] seed_q, lower, upper,
    input logic [447:0] velocity_limit, acceleration_limit, jerk_limit, dq_max,
    output logic busy, done,
    output logic allow, clamp, bounds_valid,
    output logic [223:0] q16, wire_values
);
    generate if (CLOCK_HZ <= 0) begin : invalid_clock
        initial $fatal(1, "CLOCK_HZ must be a positive signed integer");
    end endgenerate

    localparam logic signed [33:0] QMIN = -34'sd2147483648;
    localparam logic signed [33:0] QMAX = 34'sd2147483647;
    localparam logic signed [38:0] WMIN = -39'sd2147483648;
    localparam logic signed [38:0] WMAX = 39'sd2147483647;

    // Full raw64 snapshots: neither narrowing nor FROZEN_INPUTS is trusted.
    logic [447:0] q_hold, lo_hold, hi_hold, vl_hold, al_hold, jl_hold, dq_hold;
    logic [2:0] axis_index, phase;
    logic [6:0] good_axes, clamp_axes, bounds_axes;
    logic [223:0] q_candidates, wire_candidates;

    wire signed [63:0] raw_q = q_hold[64*axis_index +:64];
    wire signed [63:0] raw_lo = lo_hold[64*axis_index +:64];
    wire signed [63:0] raw_hi = hi_hold[64*axis_index +:64];
    wire signed [63:0] vl = vl_hold[64*axis_index +:64];
    wire signed [63:0] al = al_hold[64*axis_index +:64];
    wire signed [63:0] jl = jl_hold[64*axis_index +:64];
    wire signed [63:0] dq = dq_hold[64*axis_index +:64];
    wire signed [48:0] q = raw_q[48:0];
    wire signed [48:0] lo = raw_lo[48:0];
    wire signed [48:0] hi = raw_hi[48:0];

    logic signed [33:0] qlo_next, qhi_next, qnearest_next;
    logic signed [33:0] code_lo, code_hi, nearest_q, code;
    logic signed [33:0] code_next;
    logic signed [34:0] q_correction_next;
    logic signed [49:0] q_decoded_next;
    logic q_guard, q_interval_guard, bounds_raw_guard;
    logic lane_known, bounds_known;
    logic q_quant_guard;
    logic signed [69:0] scaled_lo, scaled_hi;
    logic signed [53:0] scaled_code;
    logic signed [38:0] wire_lo, wire_hi, nearest_wire, encoded;
    logic signed [38:0] wire_lo_next, wire_hi_next, nearest_wire_next, encoded_next;
    logic signed [39:0] wire_correction;
    logic signed [70:0] wire_decoded;
    logic wire_interval_guard;
    logic good_now, bounds_now, clamp_now;

    // Exactly one 49x21 signed multiplication expression is shared by the
    // lower, upper and Q-code scales. Both positive unit literals fit signed21.
    // The code34 sign extends to49; its exact product fits signed54. Keep the
    // physical raw49 scales as signed70; all signed guards remain unshortened.
    logic signed [48:0] mul_operand;
    logic signed [20:0] unit_factor;
    logic signed [69:0] mul_product;
    always_comb begin
        unit_factor = (axis_index == 3'd6) ? 21'sd1000000 : 21'sd1000;
        mul_operand = 49'sd0;
        case (phase)
            3'd1: mul_operand = lo;
            3'd2: mul_operand = hi;
            3'd3: mul_operand = $signed({{15{code[33]}},code});
            default: mul_operand = 49'sd0;
        endcase
        mul_product = $signed(mul_operand) * $signed(unit_factor);
    end

    always_comb begin
        // Floor plus nonnegative binary remainder works for both signs.
        qlo_next = $signed({lo[48],lo[48:16]}) +
                   ((lo[15:0] != 16'd0) ? 34'sd1 : 34'sd0);
        qhi_next = $signed({hi[48],hi[48:16]});
        qnearest_next = $signed({q[48],q[48:16]});
        if (q[15:0] > 16'd32768 ||
            (q[15:0] == 16'd32768 && qnearest_next[0]))
            qnearest_next = qnearest_next + 34'sd1;

        code_next = nearest_q < code_lo ? code_lo :
                    (nearest_q > code_hi ? code_hi : nearest_q);
        q_correction_next = $signed({code_next[33],code_next}) -
                            $signed({nearest_q[33],nearest_q});
        q_decoded_next = $signed({{16{code_next[33]}},code_next}) <<< 16;

        wire_lo_next = $signed({scaled_lo[69],scaled_lo[69:32]}) +
                       ((scaled_lo[31:0] != 32'd0) ? 39'sd1 : 39'sd0);
        wire_hi_next = $signed({scaled_hi[69],scaled_hi[69:32]});
        nearest_wire_next = $signed({scaled_code[53],scaled_code[53:16]});
        if (scaled_code[15:0] > 16'd32768 ||
            (scaled_code[15:0] == 16'd32768 && nearest_wire_next[0]))
            nearest_wire_next = nearest_wire_next + 39'sd1;
        encoded_next = nearest_wire_next < wire_lo ? wire_lo :
                       (nearest_wire_next > wire_hi ? wire_hi : nearest_wire_next);
        wire_correction = $signed({encoded[38],encoded}) -
                          $signed({nearest_wire[38],nearest_wire});
        wire_decoded = $signed({{32{encoded[38]}},encoded}) <<< 32;

        // q/derivative values cannot influence this certificate, even as X/Z.
        bounds_now = (bounds_raw_guard && q_interval_guard &&
                      wire_interval_guard) === 1'b1;
        good_now = (lane_known && q_guard && q_interval_guard &&
                    q_quant_guard && wire_interval_guard &&
                    wire_correction >= -40'sd1 && wire_correction <= 40'sd1 &&
                    wire_decoded >= $signed({scaled_lo[69],scaled_lo}) &&
                    wire_decoded <= $signed({scaled_hi[69],scaled_hi})) === 1'b1;
        clamp_now = ((code != nearest_q) || (encoded != nearest_wire)) === 1'b1;
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            busy<=0; done<=0; allow<=0; clamp<=0; bounds_valid<=0;
            q16<='0; wire_values<='0;
            q_hold<='0; lo_hold<='0; hi_hold<='0; vl_hold<='0;
            al_hold<='0; jl_hold<='0; dq_hold<='0;
            axis_index<=0; phase<=0; good_axes<=0; clamp_axes<=0; bounds_axes<=0;
            q_candidates<='0; wire_candidates<='0;
            code_lo<=0; code_hi<=0; nearest_q<=0; code<=0;
            q_guard<=0; q_interval_guard<=0; bounds_raw_guard<=0;
            lane_known<=0; bounds_known<=0; q_quant_guard<=0;
            scaled_lo<=0; scaled_hi<=0; scaled_code<=0;
            wire_lo<=0; wire_hi<=0; nearest_wire<=0; encoded<=0;
            wire_interval_guard<=0;
        end else begin
            done<=0;
            if (!busy) begin
                if (start === 1'b1) begin
                    q_hold<=seed_q; lo_hold<=lower; hi_hold<=upper;
                    vl_hold<=velocity_limit; al_hold<=acceleration_limit;
                    jl_hold<=jerk_limit; dq_hold<=dq_max;
                    axis_index<=0; phase<=1; busy<=1;
                    good_axes<=0; clamp_axes<=0; bounds_axes<=0;
                    q_candidates<='0; wire_candidates<='0;
                    allow<=0; clamp<=0; bounds_valid<=0; q16<='0; wire_values<='0;
                end
            end else begin
                case (phase)
                    3'd1: begin
                        code_lo<=qlo_next; code_hi<=qhi_next; nearest_q<=qnearest_next;
                        scaled_lo<=mul_product;
                        lane_known<=((^{raw_q,raw_lo,raw_hi,vl,al,jl,dq}) !== 1'bx);
                        bounds_known<=((^{raw_lo,raw_hi}) !== 1'bx);
                        bounds_raw_guard<=((raw_lo[63:49] == {15{raw_lo[48]}}) &&
                                           (raw_hi[63:49] == {15{raw_hi[48]}}) &&
                                           lo < hi) === 1'b1;
                        q_guard<=((raw_q[63:49] == {15{raw_q[48]}}) &&
                                  (raw_lo[63:49] == {15{raw_lo[48]}}) &&
                                  (raw_hi[63:49] == {15{raw_hi[48]}}) &&
                                  lo < hi && q >= lo && q <= hi &&
                                  vl > 64'sd0 && al > 64'sd0 &&
                                  jl > 64'sd0 && dq > 64'sd0) === 1'b1;
                        q_interval_guard<=(qlo_next<=qhi_next &&
                                           qlo_next>=QMIN && qhi_next<=QMAX) === 1'b1;
                        phase<=2;
                    end
                    3'd2: begin
                        code<=code_next; scaled_hi<=mul_product;
                        q_quant_guard<=(q_correction_next >= -35'sd1 &&
                                        q_correction_next <= 35'sd1 &&
                                        q_decoded_next >= $signed({lo[48],lo}) &&
                                        q_decoded_next <= $signed({hi[48],hi})) === 1'b1;
                        phase<=3;
                    end
                    3'd3: begin
                        // code34 x unit21 is exact signed54. The shared70
                        // product is narrowed only at this proved bound.
                        scaled_code<=mul_product[53:0];
                        wire_lo<=wire_lo_next; wire_hi<=wire_hi_next;
                        wire_interval_guard<=(wire_lo_next<=wire_hi_next &&
                                              wire_lo_next>=WMIN && wire_hi_next<=WMAX) === 1'b1;
                        phase<=4;
                    end
                    3'd4: begin
                        nearest_wire<=nearest_wire_next; encoded<=encoded_next;
                        phase<=5;
                    end
                    3'd5: begin
                        good_axes[axis_index]<=good_now;
                        bounds_axes[axis_index]<=bounds_now && bounds_known;
                        clamp_axes[axis_index]<=clamp_now;
                        q_candidates[32*axis_index+:32]<=code[31:0];
                        wire_candidates[32*axis_index+:32]<=encoded[31:0];
                        if (axis_index==3'd6) phase<=6;
                        else begin axis_index<=axis_index+1'b1; phase<=1; end
                    end
                    3'd6: begin
                        busy<=0; done<=1; phase<=0;
                        bounds_valid<=((&bounds_axes) === 1'b1);
                        if ((&good_axes) === 1'b1) begin
                            allow<=1; clamp<=|clamp_axes;
                            q16<=q_candidates; wire_values<=wire_candidates;
                        end else begin
                            allow<=0; clamp<=0; q16<='0; wire_values<='0;
                        end
                    end
                    default: begin
                        // Unreachable two-state control: fail closed.
                        busy<=0; done<=1; phase<=0;
                        allow<=0; clamp<=0; bounds_valid<=0; q16<='0; wire_values<='0;
                    end
                endcase
            end
        end
    end
endmodule

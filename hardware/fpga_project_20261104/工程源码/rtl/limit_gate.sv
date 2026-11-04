`timescale 1ns/1ps
// Conservative whole-segment certificate and two-stage integer wire gate.
// Pure combinational logic; caller must hold inputs stable and reject the
// upstream player_math overflow. This module makes no 100MHz timing claim.
// Axes 0..5: deg, axis 6: m. Input fields: signed64 Q32.32, axis0 in low bits.
// Output q16: signed32 Q16.16; wire_values: signed32 mdeg / gripper um.
// Full products/sums fit signed192 for int64 inputs, uint32 dt and a positive
// signed-integer CLOCK_HZ. This also preserves q0 +/- DQ without wrap.
module limit_gate #(
    parameter integer CLOCK_HZ = 100000000,
    // Specialize only the seed call. Reject nonstationary inputs rather than
    // treating this parameter as a trusted bypass for a general segment.
    parameter integer STATIONARY = 0
) (
    input logic [447:0] q0, v0, a0, j, q1, v1, a1,
    input logic [31:0] dt_cycles,
    input logic [447:0] lower, upper,
    input logic [447:0] velocity_limit, acceleration_limit, jerk_limit, dq_max,
    input logic [223:0] previous_wire,
    input logic previous_wire_valid,
    output logic allow,
    output logic clamp,
    output logic [223:0] q16,
    output logic [223:0] wire_values
);
    localparam logic signed [191:0] C = CLOCK_HZ;
    localparam logic signed [191:0] C2 = C * C;
    localparam logic signed [191:0] C3 = C2 * C;
    localparam logic signed [191:0] QDEN = 192'sd6 * C3;
    localparam logic signed [191:0] VDEN = 192'sd2 * C2;
    localparam logic signed [191:0] S16 = 192'sd65536;
    localparam logic signed [191:0] S32 = 192'sd4294967296;
    localparam logic signed [191:0] I32LO = -192'sd2147483648;
    localparam logic signed [191:0] I32HI = 192'sd2147483647;

    generate if (CLOCK_HZ <= 0) begin : invalid_clock_hz
        initial $fatal(1, "CLOCK_HZ must be a positive signed integer");
    end endgenerate

    function automatic logic signed [191:0] wide64(input logic signed [63:0] value);
        wide64 = {{128{value[63]}}, value};
    endfunction
    function automatic logic signed [191:0] wide32(input logic signed [31:0] value);
        wide32 = {{160{value[31]}}, value};
    endfunction
    function automatic logic signed [191:0] magnitude(input logic signed [191:0] value);
        magnitude = value < 0 ? -value : value;
    endfunction
    function automatic logic signed [191:0] ceil_shift(
        input logic signed [191:0] value, input integer bits);
        ceil_shift = -((-value) >>> bits);
    endfunction
    function automatic logic signed [191:0] rne_shift16(input logic signed [191:0] value);
        logic signed [191:0] mag, whole, remainder;
        begin
            mag = magnitude(value);
            whole = mag >>> 16;
            remainder = mag & 192'hffff;
            if (remainder > 192'sd32768 || (remainder == 192'sd32768 && whole[0]))
                whole = whole + 192'sd1;
            rne_shift16 = value < 0 ? -whole : whole;
        end
    endfunction
    function automatic logic ratio_inside(
        input logic signed [191:0] numerator,
        input logic signed [191:0] minimum,
        input logic signed [191:0] maximum,
        input logic signed [191:0] denominator);
        // Bounds are integer Q32.32. This is exactly equivalent to requiring
        // floor(control_point)>=minimum and ceil(control_point)<=maximum.
        ratio_inside = numerator >= minimum*denominator && numerator <= maximum*denominator;
    endfunction

    integer axis, point;
    logic signed [191:0] d, d2, d3;
    logic signed [191:0] qs, vs, acs, js, qe, ve, ae;
    logic signed [191:0] lo, hi, vmax, amax, jmax, dq;
    logic signed [191:0] qlo, qhi;
    logic signed [191:0] qb [0:3];
    logic signed [191:0] vb [0:2];
    logic signed [191:0] ab [0:1];
    logic signed [191:0] qcode_lo, qcode_hi, current_code, next_code, nearest;
    logic signed [191:0] scale, wire_lo, wire_hi, encoded, previous;
    always_comb begin
        allow = 1'b1;
        clamp = 1'b0;
        q16 = '0;
        wire_values = '0;
        d = $signed({160'b0,dt_cycles});
        d2 = d*d;
        d3 = d2*d;
        // Fixed nonzero denominators make all following operations defined even
        // on rejected dt/config inputs. There is no data-dependent division.
        if (dt_cycles == 0) allow = 1'b0;
        for (axis = 0; axis < 7; axis = axis + 1) begin
            qs = wide64($signed(q0[64*axis +: 64]));
            vs = wide64($signed(v0[64*axis +: 64]));
            acs = wide64($signed(a0[64*axis +: 64]));
            js = wide64($signed(j[64*axis +: 64]));
            qe = wide64($signed(q1[64*axis +: 64]));
            ve = wide64($signed(v1[64*axis +: 64]));
            ae = wide64($signed(a1[64*axis +: 64]));
            lo = wide64($signed(lower[64*axis +: 64]));
            hi = wide64($signed(upper[64*axis +: 64]));
            vmax = wide64($signed(velocity_limit[64*axis +: 64]));
            amax = wide64($signed(acceleration_limit[64*axis +: 64]));
            jmax = wide64($signed(jerk_limit[64*axis +: 64]));
            dq = wide64($signed(dq_max[64*axis +: 64]));
            if (lo >= hi || vmax <= 0 || amax <= 0 || jmax <= 0 || dq <= 0)
                allow = 1'b0;
            if (magnitude(js) > jmax) allow = 1'b0;
            qlo = lo > qs-dq ? lo : qs-dq;
            qhi = hi < qs+dq ? hi : qs+dq;
            // Exact cubic/quadratic/linear Bernstein numerators, common
            // denominators 6*C^3, 2*C^2 and C respectively.
            if (STATIONARY) begin
                for (point=0;point<4;point=point+1) qb[point]=0;
                for (point=0;point<3;point=point+1) vb[point]=0;
                for (point=0;point<2;point=point+1) ab[point]=0;
                if (vs!=0 || acs!=0 || js!=0 || qe!=qs || ve!=0 || ae!=0 || qs<lo || qs>hi)
                    allow=1'b0;
            end else begin
            qb[0] = qs*QDEN;
            qb[1] = qs*QDEN + vs*(192'sd2*d*C2);
            qb[2] = qs*QDEN + vs*(192'sd4*d*C2) + acs*(d2*C);
            qb[3] = qs*QDEN + vs*(192'sd6*d*C2) + acs*(192'sd3*d2*C) + js*d3;
            vb[0] = vs*VDEN;
            vb[1] = vs*VDEN + acs*(d*C);
            vb[2] = vs*VDEN + acs*(192'sd2*d*C) + js*d2;
            ab[0] = acs*C;
            ab[1] = acs*C + js*d;
            for (point = 0; point < 4; point = point + 1)
                if (!ratio_inside(qb[point],qlo,qhi,QDEN)) allow = 1'b0;
            for (point = 0; point < 3; point = point + 1)
                if (!ratio_inside(vb[point],-vmax,vmax,VDEN)) allow = 1'b0;
            for (point = 0; point < 2; point = point + 1)
                if (!ratio_inside(ab[point],-amax,amax,C)) allow = 1'b0;
            end
            // The caller supplies the single-RNE mathematical endpoint. Its
            // int64 overflow is upstream; actual rounded values are gated here.
            if (qe < qlo || qe > qhi || magnitude(ve) > vmax || magnitude(ae) > amax)
                allow = 1'b0;
            qcode_lo = ceil_shift(lo,16);
            qcode_hi = hi >>> 16;
            if (qcode_lo > qcode_hi || qcode_lo < I32LO || qcode_hi > I32HI)
                allow = 1'b0;
            nearest = rne_shift16(qe);
            next_code = nearest < qcode_lo ? qcode_lo : (nearest > qcode_hi ? qcode_hi : nearest);
            if (magnitude(next_code-nearest) > 192'sd1) allow = 1'b0;
            if (next_code != nearest) clamp = 1'b1;
            nearest = rne_shift16(qs);
            current_code = nearest < qcode_lo ? qcode_lo : (nearest > qcode_hi ? qcode_hi : nearest);
            if (magnitude(current_code-nearest) > 192'sd1) allow = 1'b0;
            if (magnitude(next_code-current_code)*S16 > dq) allow = 1'b0;
            if (next_code*S16 < lo || next_code*S16 > hi) allow = 1'b0;
            q16[32*axis +: 32] = next_code[31:0];

            scale = axis == 6 ? 192'sd1000000 : 192'sd1000;
            wire_lo = ceil_shift(lo*scale,32);
            wire_hi = (hi*scale) >>> 32;
            if (wire_lo > wire_hi || wire_lo < I32LO || wire_hi > I32HI)
                allow = 1'b0;
            nearest = rne_shift16(next_code*scale);
            encoded = nearest < wire_lo ? wire_lo : (nearest > wire_hi ? wire_hi : nearest);
            if (magnitude(encoded-nearest) > 192'sd1) allow = 1'b0;
            if (encoded != nearest) clamp = 1'b1;
            if (encoded*S32 < lo*scale || encoded*S32 > hi*scale) allow = 1'b0;
            if (previous_wire_valid) begin
                previous = wide32($signed(previous_wire[32*axis +: 32]));
                if (previous*S32 < lo*scale || previous*S32 > hi*scale ||
                    magnitude(encoded-previous)*S32 > dq*scale) allow = 1'b0;
            end else previous = 192'sd0;
            wire_values[32*axis +: 32] = encoded[31:0];
        end
        if (!allow) begin
            q16 = '0;
            wire_values = '0;
            clamp = 1'b0;
        end
    end
endmodule

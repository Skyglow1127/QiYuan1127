`timescale 1ns/1ps
// Candidate axis pipeline: same certificate/encoding, fixed E15 completion.
// No hardware timing/resource claim. Axes0..5 deg, axis6 m; axis0 is low bits.
// Accept only while !busy. Default0 freezes all inputs at E0. Mode1 requires
// the shared-owner stable producer contract through gate completion/reset.
// E0 checks/previous-valid, S0 axis registers and E15 publication stay unchanged.
// Six arithmetic stages finish axis6 at E13; E14 reduces; E15 publishes.
// Cache and terms must be PL-produced and bound to the same validated record,
// CLOCK_HZ and active config epoch. Valid bits are not PS-supplied certificates.
module limit_gate_pipe #(
    parameter integer CLOCK_HZ = 100000000,
    parameter integer STABLE_JOB_INPUTS = 0
) (
    input logic clk, resetn, start,
    input logic terms_valid, cache_valid, math_overflow,
    input logic [31:0] dt_cycles,
    input logic [447:0] q0, v0, a0, j, q1, v1, a1,
    input logic [447:0] lower, upper,
    input logic [447:0] velocity_limit, acceleration_limit, jerk_limit, dq_max,
    input logic [223:0] previous_wire,
    input logic previous_wire_valid,
    // Each packed field contains seven signed192 exact, unrounded products.
    input logic [1343:0] qp0_terms, qp1_terms, qp2_terms, qp3_terms,
    input logic [1343:0] vp0_terms, vp1_terms, vp2_terms, ap0_terms, ap1_terms,
    // Unscaled cached cross-products, computed by PL from validated raw limits.
    input logic [1343:0] cached_lower_qden, cached_upper_qden, cached_dq_qden,
    input logic [1343:0] cached_vmax_vden, cached_amax_c,
    output logic busy, done, allow, clamp,
    output logic [223:0] q16, wire_values
);
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
    // Every arithmetic/compare below explicitly widens register operands first.
    // In particular, shifts/subtractions must never inherit a narrow result size.
    function automatic logic signed [191:0] wide162(input logic signed [161:0] value);
        wide162 = {{30{value[161]}}, value};
    endfunction
    function automatic logic signed [191:0] wide166(input logic signed [165:0] value);
        wide166 = {{26{value[165]}}, value};
    endfunction
    function automatic logic signed [191:0] wide65(input logic signed [64:0] value);
        wide65 = {{127{value[64]}}, value};
    endfunction
    function automatic logic signed [191:0] wide49(input logic signed [48:0] value);
        wide49 = {{143{value[48]}}, value};
    endfunction
    function automatic logic signed [191:0] wide85(input logic signed [84:0] value);
        wide85 = {{107{value[84]}}, value};
    endfunction
    function automatic logic signed [191:0] wide54(input logic signed [53:0] value);
        wide54 = {{138{value[53]}}, value};
    endfunction
    function automatic logic packed_fits162(input logic [1343:0] values);
        logic [191:0] value;
        integer lane;
        begin
            packed_fits162=1'b1;
            for(lane=0;lane<7;lane=lane+1) begin
                value=values[192*lane+:192];
                // Ordinary == followed by !==1 also rejects X/Z sign/high bits.
                if ((value[191:162] == {30{value[161]}}) !== 1'b1)
                    packed_fits162=1'b0;
            end
        end
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
    function automatic logic signed [191:0] times_three(input logic signed [191:0] value);
        times_three = value + (value <<< 1);
    endfunction
    function automatic logic signed [191:0] inward(
        input logic signed [191:0] value, minimum, maximum);
        inward = value < minimum ? minimum : (value > maximum ? maximum : value);
    endfunction

    logic [3:0] phase;
    logic bad_job, decision_good, decision_clamp;
    logic [6:0] bad_axis, clamp_axis, seen_axis;
    logic [6:0] token_valid, token_bad, token_clamp;
    logic [2:0] token_axis [0:6];
    logic previous_valid_reg;
    logic [223:0] previous_reg, candidate_q16, candidate_wire;
    logic [447:0] qs_reg, vs_reg, as_reg, js_reg, qe_reg, ve_reg, ae_reg;
    logic [447:0] lower_reg, upper_reg, vmax_reg, amax_reg, jmax_reg, dq_reg;
    logic signed [161:0] qp [0:6][0:3], vp [0:6][0:2], ap [0:6][0:1];
    logic signed [161:0] cache_lo [0:6], cache_hi [0:6], cache_dq [0:6];
    logic signed [161:0] cache_v [0:6], cache_a [0:6];
    logic signed [165:0] nq[0:3], rq[0:3];
    logic signed [165:0] nv[0:2], na[0:1];
    logic signed [165:0] hull_lo, hull_hi, hull_dq;
    logic signed [165:0] hull_v, hull_a;
    logic signed [64:0] qlo, qhi;
    logic signed [48:0] code_lo, code_hi;
    logic signed [48:0] next_nearest, current_nearest;
    logic signed [48:0] next_code, current_code;
    logic signed [84:0] scaled_lo, scaled_hi, scaled_dq;
    logic signed [53:0] wire_lo, wire_hi;
    logic signed [84:0] scaled_next;
    logic signed [53:0] wire_nearest, encoded;
    logic signed [48:0] next_nearest_s3, current_nearest_s3;
    logic signed [53:0] wire_lo_s4, wire_hi_s4;
    logic signed [84:0] scaled_lo_s3, scaled_hi_s3, scaled_dq_s3;
    logic signed [84:0] scaled_lo_s4, scaled_hi_s4, scaled_dq_s4;
    logic signed [84:0] scaled_lo_s5, scaled_hi_s5, scaled_dq_s5;
    logic [831:0] raw_token [0:3];
    logic [3:0] unit_token;
    logic [31:0] previous_token [0:5];
    logic signed [161:0] qp_s0 [0:3], vp_s0 [0:2], ap_s0 [0:1];
    logic signed [161:0] cache_lo_s0,cache_hi_s0,cache_dq_s0,cache_v_s0,cache_a_s0;
    integer inject_axis;
    // Blocking local scratch only; no scratch value is carried between edges.
    logic signed [191:0] qs, vs, acs, js, qe, ve, ae, lo, hi, vmax, amax, jmax, dq;
    logic signed [191:0] previous, nearest_tmp;
    logic axis_ok;
    integer axis, point;
    integer axis_s1,axis_s2,axis_s3,axis_s4,axis_s5,axis_s6;

    // Constant modes elaborate exactly one owner per view. Default0 is the
    // original E0 capture, including absence of a data reset load. The local
    // snapshot_axis is not shared with the main arithmetic process.
    generate if(STABLE_JOB_INPUTS==0) begin : snapshot_job_inputs
        always_ff @(posedge clk) begin
            if(!resetn) begin
                // Original data context had no reset assignments.
            end else if(!busy) begin
                if(start) begin
                    qs_reg <= q0; vs_reg <= v0; as_reg <= a0; js_reg <= j;
                    qe_reg <= q1; ve_reg <= v1; ae_reg <= a1;
                    lower_reg <= lower; upper_reg <= upper;
                    vmax_reg <= velocity_limit; amax_reg <= acceleration_limit;
                    jmax_reg <= jerk_limit; dq_reg <= dq_max;
                    previous_reg <= previous_wire;
                    for (integer snapshot_axis = 0; snapshot_axis < 7; snapshot_axis = snapshot_axis + 1) begin
                        qp[snapshot_axis][0] <= $signed(qp0_terms[192*snapshot_axis +: 162]);
                        qp[snapshot_axis][1] <= $signed(qp1_terms[192*snapshot_axis +: 162]);
                        qp[snapshot_axis][2] <= $signed(qp2_terms[192*snapshot_axis +: 162]);
                        qp[snapshot_axis][3] <= $signed(qp3_terms[192*snapshot_axis +: 162]);
                        vp[snapshot_axis][0] <= $signed(vp0_terms[192*snapshot_axis +: 162]);
                        vp[snapshot_axis][1] <= $signed(vp1_terms[192*snapshot_axis +: 162]);
                        vp[snapshot_axis][2] <= $signed(vp2_terms[192*snapshot_axis +: 162]);
                        ap[snapshot_axis][0] <= $signed(ap0_terms[192*snapshot_axis +: 162]);
                        ap[snapshot_axis][1] <= $signed(ap1_terms[192*snapshot_axis +: 162]);
                        cache_lo[snapshot_axis] <= $signed(cached_lower_qden[192*snapshot_axis +: 162]);
                        cache_hi[snapshot_axis] <= $signed(cached_upper_qden[192*snapshot_axis +: 162]);
                        cache_dq[snapshot_axis] <= $signed(cached_dq_qden[192*snapshot_axis +: 162]);
                        cache_v[snapshot_axis] <= $signed(cached_vmax_vden[192*snapshot_axis +: 162]);
                        cache_a[snapshot_axis] <= $signed(cached_amax_c[192*snapshot_axis +: 162]);
                    end
                end
            end
        end
    end else if(STABLE_JOB_INPUTS==1) begin : stable_job_input_views
        // ONLY the top/shared PL-owner contract permits these direct aliases.
        // E0 packed_fits162/XZ/valid checks below are deliberately not bypassed.
        assign qs_reg=q0;
        assign vs_reg=v0;
        assign as_reg=a0;
        assign js_reg=j;
        assign qe_reg=q1;
        assign ve_reg=v1;
        assign ae_reg=a1;
        assign lower_reg=lower;
        assign upper_reg=upper;
        assign vmax_reg=velocity_limit;
        assign amax_reg=acceleration_limit;
        assign jmax_reg=jerk_limit;
        assign dq_reg=dq_max;
        assign previous_reg=previous_wire;
        for(genvar stable_axis=0;stable_axis<7;stable_axis=stable_axis+1) begin : axis_views
            assign qp[stable_axis][0]=$signed(qp0_terms[192*stable_axis+:162]);
            assign qp[stable_axis][1]=$signed(qp1_terms[192*stable_axis+:162]);
            assign qp[stable_axis][2]=$signed(qp2_terms[192*stable_axis+:162]);
            assign qp[stable_axis][3]=$signed(qp3_terms[192*stable_axis+:162]);
            assign vp[stable_axis][0]=$signed(vp0_terms[192*stable_axis+:162]);
            assign vp[stable_axis][1]=$signed(vp1_terms[192*stable_axis+:162]);
            assign vp[stable_axis][2]=$signed(vp2_terms[192*stable_axis+:162]);
            assign ap[stable_axis][0]=$signed(ap0_terms[192*stable_axis+:162]);
            assign ap[stable_axis][1]=$signed(ap1_terms[192*stable_axis+:162]);
            assign cache_lo[stable_axis]=$signed(cached_lower_qden[192*stable_axis+:162]);
            assign cache_hi[stable_axis]=$signed(cached_upper_qden[192*stable_axis+:162]);
            assign cache_dq[stable_axis]=$signed(cached_dq_qden[192*stable_axis+:162]);
            assign cache_v[stable_axis]=$signed(cached_vmax_vden[192*stable_axis+:162]);
            assign cache_a[stable_axis]=$signed(cached_amax_c[192*stable_axis+:162]);
        end
    end else begin : invalid_stable_job_inputs
        initial $fatal(1,"STABLE_JOB_INPUTS must be0 or1");
    end endgenerate

    always_ff @(posedge clk) begin
        if (!resetn) begin
            busy <= 0; done <= 0; allow <= 0; clamp <= 0;
            q16 <= '0; wire_values <= '0; phase <= 0;
            bad_job <= 0; bad_axis <= '0; clamp_axis <= '0; seen_axis <= '0;
            token_valid <= '0; token_bad <= '0; token_clamp <= '0;
            for (axis=0;axis<7;axis=axis+1) token_axis[axis] <= 0;
            decision_good <= 0; decision_clamp <= 0;
            candidate_q16 <= '0; candidate_wire <= '0;
        end else begin
            done <= 0;
            if (!busy) begin
                if (start) begin
                    busy <= 1; phase <= 1; allow <= 0; clamp <= 0;
                    q16 <= '0; wire_values <= '0;
                    bad_job <= ((dt_cycles > 0) !== 1'b1) ||
                               (terms_valid !== 1'b1) || (cache_valid !== 1'b1) ||
                               (math_overflow !== 1'b0) ||
                               !packed_fits162(qp0_terms) ||
                               !packed_fits162(qp1_terms) ||
                               !packed_fits162(qp2_terms) ||
                               !packed_fits162(qp3_terms) ||
                               !packed_fits162(vp0_terms) ||
                               !packed_fits162(vp1_terms) ||
                               !packed_fits162(vp2_terms) ||
                               !packed_fits162(ap0_terms) ||
                               !packed_fits162(ap1_terms) ||
                               !packed_fits162(cached_lower_qden) ||
                               !packed_fits162(cached_upper_qden) ||
                               !packed_fits162(cached_dq_qden) ||
                               !packed_fits162(cached_vmax_vden) ||
                               !packed_fits162(cached_amax_c);
                    bad_axis <= '0; clamp_axis <= '0; seen_axis <= '0;
                    token_valid <= '0; token_bad <= '0; token_clamp <= '0;
                    decision_good <= 0; decision_clamp <= 0;
                    candidate_q16 <= '0; candidate_wire <= '0;
                    previous_valid_reg <= previous_wire_valid;
                end
            end else begin
                phase <= phase + 1'b1;
                // S0 is a separate registered selector; it does no gate arithmetic.
                token_valid[0] <= (phase>=1 && phase<=7);
                if(phase>=1 && phase<=7) begin
                    token_axis[0] <= phase[2:0]-3'd1;
                    inject_axis=phase-1;
                    raw_token[0] <= {dq_reg[64*inject_axis+:64],jmax_reg[64*inject_axis+:64],amax_reg[64*inject_axis+:64],
                        vmax_reg[64*inject_axis+:64],upper_reg[64*inject_axis+:64],lower_reg[64*inject_axis+:64],
                        ae_reg[64*inject_axis+:64],ve_reg[64*inject_axis+:64],qe_reg[64*inject_axis+:64],
                        js_reg[64*inject_axis+:64],as_reg[64*inject_axis+:64],vs_reg[64*inject_axis+:64],qs_reg[64*inject_axis+:64]};
                    previous_token[0] <= previous_reg[32*inject_axis+:32];
                    unit_token[0] <= (inject_axis==6);
                    for(integer n=0;n<4;n=n+1) qp_s0[n] <= qp[inject_axis][n];
                    for(integer n=0;n<3;n=n+1) vp_s0[n] <= vp[inject_axis][n];
                    for(integer n=0;n<2;n=n+1) ap_s0[n] <= ap[inject_axis][n];
                    cache_lo_s0<=cache_lo[inject_axis];cache_hi_s0<=cache_hi[inject_axis];cache_dq_s0<=cache_dq[inject_axis];
                    cache_v_s0<=cache_v[inject_axis];cache_a_s0<=cache_a[inject_axis];

                    token_bad[0] <= 0; token_clamp[0] <= 0;
                end
                for(integer t=1;t<7;t=t+1) begin
                    token_valid[t] <= token_valid[t-1];
                    token_axis[t] <= token_axis[t-1];
                end
                for(integer r=1;r<4;r=r+1) begin
                    raw_token[r] <= raw_token[r-1];
                    unit_token[r] <= unit_token[r-1];
                end
                for(integer r=1;r<6;r=r+1) previous_token[r] <= previous_token[r-1];
                // S1: axis a executes at E(2+a).
                if(token_valid[0] === 1'b1) begin
                    axis_s1=token_axis[0];
                            qs=wide64($signed(raw_token[0][0+:64]));
                            vs=wide64($signed(raw_token[0][64+:64]));
                            acs=wide64($signed(raw_token[0][128+:64]));
                            js=wide64($signed(raw_token[0][192+:64]));
                            lo=wide64($signed(raw_token[0][448+:64]));
                            hi=wide64($signed(raw_token[0][512+:64]));
                            vmax=wide64($signed(raw_token[0][576+:64]));
                            amax=wide64($signed(raw_token[0][640+:64]));
                            jmax=wide64($signed(raw_token[0][704+:64]));
                            dq=wide64($signed(raw_token[0][768+:64]));
                            axis_ok=(lo < hi && vmax > 0 && amax > 0 && jmax > 0 && dq > 0 &&
                                     magnitude(js) <= jmax && qs >= lo && qs <= hi &&
                                     magnitude(vs) <= vmax && magnitude(acs) <= amax &&
                                     wide162(cache_lo_s0) < wide162(cache_hi_s0) && wide162(cache_dq_s0) > 0 &&
                                     wide162(cache_v_s0) > 0 && wide162(cache_a_s0) > 0);
                            token_bad[1] <= token_bad[0] | (axis_ok !== 1'b1);
                            qlo <= lo > qs-dq ? lo : qs-dq;
                            qhi <= hi < qs+dq ? hi : qs+dq;
                            hull_lo <= times_three(wide162(cache_lo_s0));
                            hull_hi <= times_three(wide162(cache_hi_s0));
                            hull_dq <= times_three(wide162(cache_dq_s0));
                            hull_v <= wide162(cache_v_s0) <<< 1;
                            hull_a <= wide162(cache_a_s0);
                            nq[0] <= times_three(wide162(qp_s0[0]));
                            nq[1] <= times_three(wide162(qp_s0[0]))+wide162(qp_s0[1]);
                            nq[2] <= times_three(wide162(qp_s0[0]))+(wide162(qp_s0[1]) <<< 1)+wide162(qp_s0[2]);
                            nq[3] <= times_three(wide162(qp_s0[0])+wide162(qp_s0[1])+wide162(qp_s0[2])+wide162(qp_s0[3]));
                            rq[0] <= 192'sd0;
                            rq[1] <= wide162(qp_s0[1]);
                            rq[2] <= (wide162(qp_s0[1]) <<< 1)+wide162(qp_s0[2]);
                            rq[3] <= times_three(wide162(qp_s0[1])+wide162(qp_s0[2])+wide162(qp_s0[3]));
                            nv[0] <= wide162(vp_s0[0]) <<< 1;
                            nv[1] <= (wide162(vp_s0[0]) <<< 1)+wide162(vp_s0[1]);
                            nv[2] <= (wide162(vp_s0[0])+wide162(vp_s0[1])+wide162(vp_s0[2])) <<< 1;
                            na[0] <= wide162(ap_s0[0]);
                            na[1] <= wide162(ap_s0[0])+wide162(ap_s0[1]);
                            token_clamp[1] <= token_clamp[0];
                end
                // S2: axis a executes at E(3+a).
                if(token_valid[1] === 1'b1) begin
                    axis_s2=token_axis[1];
                            axis_ok=1'b1;
                            for (point=0;point<4;point=point+1)
                                if ((wide166(nq[point]) >= wide166(hull_lo) && wide166(nq[point]) <= wide166(hull_hi) &&
                                     wide166(rq[point]) >= -wide166(hull_dq) && wide166(rq[point]) <= wide166(hull_dq)) !== 1'b1) axis_ok=1'b0;
                            for (point=0;point<3;point=point+1)
                                if ((wide166(nv[point]) >= -wide166(hull_v) && wide166(nv[point]) <= wide166(hull_v)) !== 1'b1) axis_ok=1'b0;
                            for (point=0;point<2;point=point+1)
                                if ((wide166(na[point]) >= -wide166(hull_a) && wide166(na[point]) <= wide166(hull_a)) !== 1'b1) axis_ok=1'b0;
                            qe=wide64($signed(raw_token[1][256+:64]));
                            ve=wide64($signed(raw_token[1][320+:64]));
                            ae=wide64($signed(raw_token[1][384+:64]));
                            lo=wide64($signed(raw_token[1][448+:64]));
                            hi=wide64($signed(raw_token[1][512+:64]));
                            vmax=wide64($signed(raw_token[1][576+:64]));
                            amax=wide64($signed(raw_token[1][640+:64]));
                            dq=wide64($signed(raw_token[1][768+:64]));
                            if ((qe >= wide65(qlo) && qe <= wide65(qhi) && magnitude(ve) <= vmax && magnitude(ae) <= amax) !== 1'b1) axis_ok=0;
                            token_bad[2] <= token_bad[1] | !axis_ok;
                            code_lo <= ceil_shift(lo,16);
                            code_hi <= hi >>> 16;
                            next_nearest <= rne_shift16(qe);
                            current_nearest <= rne_shift16(wide64($signed(raw_token[1][0+:64])));
                            // Remaining constant scale multipliers concern wire
                            // encoding, not duplicate polynomial arithmetic.
                            if(unit_token[1]) begin
                                scaled_lo <= lo*192'sd1000000; scaled_hi <= hi*192'sd1000000; scaled_dq <= dq*192'sd1000000;
                            end else begin
                                scaled_lo <= lo*192'sd1000; scaled_hi <= hi*192'sd1000; scaled_dq <= dq*192'sd1000;
                            end
                            token_clamp[2] <= token_clamp[1];
                end
                // S3: axis a executes at E(4+a).
                if(token_valid[2] === 1'b1) begin
                    axis_s3=token_axis[2];
                            axis_ok=(wide49(code_lo) <= wide49(code_hi) && wide49(code_lo) >= I32LO && wide49(code_hi) <= I32HI);
                            token_bad[3] <= token_bad[2] | (axis_ok !== 1'b1);
                            next_code <= inward(wide49(next_nearest),wide49(code_lo),wide49(code_hi));
                            current_code <= inward(wide49(current_nearest),wide49(code_lo),wide49(code_hi));
                            wire_lo <= ceil_shift(wide85(scaled_lo),32);
                            wire_hi <= wide85(scaled_hi) >>> 32;
                            token_clamp[3] <= token_clamp[2];
                            next_nearest_s3 <= next_nearest; current_nearest_s3 <= current_nearest;
                            scaled_lo_s3 <= scaled_lo; scaled_hi_s3 <= scaled_hi; scaled_dq_s3 <= scaled_dq;
                end
                // S4: axis a executes at E(5+a).
                if(token_valid[3] === 1'b1) begin
                    axis_s4=token_axis[3];
                            lo=wide64($signed(raw_token[3][448+:64]));
                            hi=wide64($signed(raw_token[3][512+:64]));
                            dq=wide64($signed(raw_token[3][768+:64]));
                            axis_ok=(magnitude(wide49(next_code)-wide49(next_nearest_s3)) <= 192'sd1 &&
                                     magnitude(wide49(current_code)-wide49(current_nearest_s3)) <= 192'sd1 &&
                                     (magnitude(wide49(next_code)-wide49(current_code)) <<< 16) <= dq &&
                                     (wide49(next_code) <<< 16) >= lo && (wide49(next_code) <<< 16) <= hi &&
                                     wide54(wire_lo) <= wide54(wire_hi) && wide54(wire_lo) >= I32LO && wide54(wire_hi) <= I32HI);
                            token_bad[4] <= token_bad[3] | (axis_ok !== 1'b1);
                            token_clamp[4] <= token_clamp[3] | (wide49(next_code) != wide49(next_nearest_s3));
                            candidate_q16[32*axis_s4+:32] <= next_code[31:0];
                            if(unit_token[3]) scaled_next <= wide49(next_code)*192'sd1000000;
                            else scaled_next <= wide49(next_code)*192'sd1000;
                            wire_lo_s4 <= wire_lo; wire_hi_s4 <= wire_hi;
                            scaled_lo_s4 <= scaled_lo_s3; scaled_hi_s4 <= scaled_hi_s3; scaled_dq_s4 <= scaled_dq_s3;
                end
                // S5: axis a executes at E(6+a).
                if(token_valid[4] === 1'b1) begin
                    axis_s5=token_axis[4];
                            nearest_tmp=rne_shift16(wide85(scaled_next));
                            wire_nearest <= nearest_tmp;
                            encoded <= inward(nearest_tmp,wide54(wire_lo_s4),wide54(wire_hi_s4));
                            token_bad[5] <= token_bad[4]; token_clamp[5] <= token_clamp[4];
                            scaled_lo_s5 <= scaled_lo_s4; scaled_hi_s5 <= scaled_hi_s4; scaled_dq_s5 <= scaled_dq_s4;
                end
                // S6: axis a executes at E(7+a).
                if(token_valid[5] === 1'b1) begin
                    axis_s6=token_axis[5];
                            axis_ok=(magnitude(wide54(encoded)-wide54(wire_nearest)) <= 192'sd1 &&
                                     (wide54(encoded) <<< 32) >= wide85(scaled_lo_s5) &&
                                     (wide54(encoded) <<< 32) <= wide85(scaled_hi_s5));
                            if (previous_valid_reg === 1'b1) begin
                                previous=wide32($signed(previous_token[5]));
                                if (((previous <<< 32) >= wide85(scaled_lo_s5) && (previous <<< 32) <= wide85(scaled_hi_s5) &&
                                     (magnitude(wide54(encoded)-previous) <<< 32) <= wide85(scaled_dq_s5)) !== 1'b1) axis_ok=0;
                            end else if (previous_valid_reg !== 1'b0) axis_ok=0;
                            token_bad[6] <= token_bad[5] | (axis_ok !== 1'b1);
                            token_clamp[6] <= token_clamp[5] | (wide54(encoded) != wide54(wire_nearest));
                            candidate_wire[32*axis_s6+:32] <= encoded[31:0];
                            bad_axis[axis_s6] <= token_bad[5] | (axis_ok !== 1'b1);
                            clamp_axis[axis_s6] <= token_clamp[5] | (wide54(encoded) != wide54(wire_nearest));
                            seen_axis[axis_s6] <= 1'b1;
                end
                // Last axis writes its final S6 result at E13. NBA state is
                // complete before E14 reduces; no last-axis result is omitted.
                if(phase==14) begin
                    decision_good <= !bad_job && !(|bad_axis) && (seen_axis==7'h7f);
                    decision_clamp <= |clamp_axis;
                end
                if(phase==15) begin
                    busy <= 0; done <= 1; phase <= 0;
                    token_valid <= '0;
                    allow <= decision_good;
                    if(decision_good === 1'b1) begin
                        clamp <= decision_clamp;
                        q16 <= candidate_q16; wire_values <= candidate_wire;
                    end else begin
                        allow <= 0; clamp <= 0; q16 <= '0; wire_values <= '0;
                    end
                end
            end
        end
    end
endmodule

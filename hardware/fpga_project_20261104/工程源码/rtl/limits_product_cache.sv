`timescale 1ns/1ps
// PL-only configuration products. No hardware resource/timing claim.
// E0 accepts/snapshots start while idle; E1..E35 register one input pair/edge;
// E2..E36 register two exact partial products; E3..E37 capture their sum.
// E38 atomically publishes all five groups and pulses done. No early valid.
// Each input axis is signed64 Q32.32; axis0 occupies the low packed bits.
// Positive integer CLOCK_HZ is elaboration-time, never a PS-supplied product.
// cfg_epoch must identify the PL's immutable active raw configuration; callers
// must also compare cached_cfg_epoch with their active epoch before gate start.
module limits_product_cache #(
    parameter integer CLOCK_HZ = 100000000
) (
    input logic clk, resetn, start,
    input logic [31:0] cfg_epoch,
    input logic [447:0] lower, upper, dq_max,
    input logic [447:0] velocity_limit, acceleration_limit,
    output logic busy, done, rejected, cache_valid,
    output logic [31:0] cached_cfg_epoch,
    output logic [1343:0] cached_lower_qden, cached_upper_qden, cached_dq_qden,
    output logic [1343:0] cached_vmax_vden, cached_amax_c
);
    // Positive signed32 CLOCK_HZ implies C<2^31, 6*C^3<2^96, hence
    // every coefficient fits positive signed98. Full signed64*98 is signed162.
    // Invalid CLOCK_HZ still elaborates, but every request is rejected at E38.
    localparam logic CLOCK_OK = CLOCK_HZ > 0;
    localparam logic signed [97:0] C = CLOCK_OK ? CLOCK_HZ : 98'sd1;
    localparam logic signed [97:0] C2 = C*C;
    localparam logic signed [97:0] C3 = C2*C;
    localparam logic signed [97:0] QDEN = 98'sd6*C3;
    localparam logic signed [97:0] VDEN = 98'sd2*C2;
    localparam logic signed [95:0] I32LO = -96'sd2147483648;
    localparam logic signed [95:0] I32HI = 96'sd2147483647;

    function automatic logic raw_limits_valid(
        input logic [447:0] lo_all, hi_all, dq_all, vmax_all, amax_all);
        logic signed [63:0] lo, hi, dq, vmax, amax;
        logic signed [95:0] lo_wide, hi_wide, code_lo, code_hi;
        integer axis;
        begin
            raw_limits_valid = 1'b1;
            for (axis=0; axis<7; axis=axis+1) begin
                lo=$signed(lo_all[64*axis+:64]); hi=$signed(hi_all[64*axis+:64]);
                dq=$signed(dq_all[64*axis+:64]); vmax=$signed(vmax_all[64*axis+:64]);
                amax=$signed(amax_all[64*axis+:64]);
                lo_wide={{32{lo[63]}},lo}; hi_wide={{32{hi[63]}},hi};
                // Negation is widened first, including INT64_MIN.
                code_lo=-((-lo_wide) >>> 16); code_hi=hi_wide >>> 16;
                if ((lo<hi && dq>0 && vmax>0 && amax>0 &&
                     code_lo<=code_hi && code_lo>=I32LO && code_hi<=I32HI) !== 1'b1)
                    raw_limits_valid=1'b0;
            end
        end
    endfunction

    logic [447:0] lower_reg, upper_reg, dq_reg, vmax_reg, amax_reg;
    logic [31:0] epoch_reg;
    logic bad_job, publish_pending;
    logic [2:0] group_index, axis_index;
    // Store only the exact 162-bit product; sign extension occurs at publish.
    logic [1133:0] scratch_lower, scratch_upper, scratch_dq, scratch_vmax, scratch_amax;
    logic signed [63:0] mul_lhs;
    logic signed [97:0] mul_rhs;
    // Register the FULL selected signed64/signed98 pair and its destination.
    // Then register exact low32-unsigned/high32-signed partial products. Both
    // valid stages clear at reset/E0; unreset data registers cannot leak jobs.
    logic signed [63:0] mul_lhs_reg;
    logic signed [97:0] mul_rhs_reg;
    logic product_valid, partial_valid;
    logic [2:0] product_group, product_axis, partial_group, partial_axis;
    logic signed [130:0] mul_low_reg;
    logic signed [129:0] mul_high_reg;
    wire signed [130:0] mul_low = $signed({1'b0,mul_lhs_reg[31:0]}) * mul_rhs_reg;
    wire signed [129:0] mul_high = $signed(mul_lhs_reg[63:32]) * mul_rhs_reg;
    wire signed [161:0] mul_low_extended = {{31{mul_low_reg[130]}},mul_low_reg};
    wire signed [161:0] mul_high_extended = {{32{mul_high_reg[129]}},mul_high_reg};
    logic signed [161:0] mul_product;
    integer axis;

    // One muxed input pair feeds TWO explicit partial multipliers. Other '*'
    // above are elaboration-time constants; packed-index factors are wiring.
    always_comb begin
        mul_lhs=64'sd0;
        mul_rhs=QDEN;
        case (group_index)
            0: mul_lhs=$signed(lower_reg[64*axis_index+:64]);
            1: mul_lhs=$signed(upper_reg[64*axis_index+:64]);
            2: mul_lhs=$signed(dq_reg[64*axis_index+:64]);
            3: begin mul_lhs=$signed(vmax_reg[64*axis_index+:64]); mul_rhs=VDEN; end
            4: begin mul_lhs=$signed(amax_reg[64*axis_index+:64]); mul_rhs=C; end
            default: begin mul_lhs=64'sd0; mul_rhs=98'sd1; end
        endcase
    end
    // Extend to FULL162 BEFORE shifting. x=unsigned_low32+(signed_high32<<32)
    // holds for every signed64 x, including INT64_MIN; no product bit is lost.
    assign mul_product=mul_low_extended+(mul_high_extended <<< 32);

    always_ff @(posedge clk) begin
        if (!resetn) begin
            busy<=0; done<=0; rejected<=0; cache_valid<=0;
            cached_cfg_epoch<=0; epoch_reg<=0; bad_job<=0;
            publish_pending<=0; group_index<=0; axis_index<=0;
            product_valid<=0; partial_valid<=0;
            product_group<=0; product_axis<=0; partial_group<=0; partial_axis<=0;
            lower_reg<='0; upper_reg<='0; dq_reg<='0; vmax_reg<='0; amax_reg<='0;
            scratch_lower<='0; scratch_upper<='0; scratch_dq<='0;
            scratch_vmax<='0; scratch_amax<='0;
            cached_lower_qden<='0; cached_upper_qden<='0; cached_dq_qden<='0;
            cached_vmax_vden<='0; cached_amax_c<='0;
        end else begin
            done<=0;
            if (!busy) begin
                if (start === 1'b1) begin
                    busy<=1; rejected<=0; cache_valid<=0; cached_cfg_epoch<=0;
                    epoch_reg<=cfg_epoch;
                    lower_reg<=lower; upper_reg<=upper; dq_reg<=dq_max;
                    vmax_reg<=velocity_limit; amax_reg<=acceleration_limit;
                    bad_job<=!CLOCK_OK || !raw_limits_valid(lower,upper,dq_max,velocity_limit,acceleration_limit) ||
                             ((^cfg_epoch) === 1'bx);
                    group_index<=0; axis_index<=0; publish_pending<=0;
                    product_valid<=0; partial_valid<=0;
                    scratch_lower<='0; scratch_upper<='0; scratch_dq<='0;
                    scratch_vmax<='0; scratch_amax<='0;
                    cached_lower_qden<='0; cached_upper_qden<='0; cached_dq_qden<='0;
                    cached_vmax_vden<='0; cached_amax_c<='0;
                end else if (cache_valid && ((cfg_epoch == cached_cfg_epoch) !== 1'b1)) begin
                    // An epoch change invalidates the published cache; returning
                    // to an old epoch does not resurrect it without a new start.
                    cache_valid<=0; cached_cfg_epoch<=0;
                    cached_lower_qden<='0; cached_upper_qden<='0; cached_dq_qden<='0;
                    cached_vmax_vden<='0; cached_amax_c<='0;
                end
            end else begin
                // A changed/unknown epoch at any busy edge permanently rejects
                // this job, even if it changes back before publication.
                if ((cfg_epoch == epoch_reg) !== 1'b1) bad_job<=1;
                // E2..E36 partial capture uses the PRIOR E1..E35 input pair
                // and tag. E3..E37 scratch capture uses PRIOR partial registers.
                // Nonblocking writes cannot mix neighboring axes or groups.
                partial_valid<=product_valid;
                if (product_valid) begin
                    mul_low_reg<=mul_low;mul_high_reg<=mul_high;
                    partial_group<=product_group;partial_axis<=product_axis;
                end
                if (partial_valid) begin
                    case (partial_group)
                        0: scratch_lower[162*partial_axis+:162]<=mul_product;
                        1: scratch_upper[162*partial_axis+:162]<=mul_product;
                        2: scratch_dq[162*partial_axis+:162]<=mul_product;
                        3: scratch_vmax[162*partial_axis+:162]<=mul_product;
                        4: scratch_amax[162*partial_axis+:162]<=mul_product;
                        default: bad_job<=1;
                    endcase
                end
                if (!publish_pending) begin
                    mul_lhs_reg<=mul_lhs;mul_rhs_reg<=mul_rhs;
                    product_group<=group_index;product_axis<=axis_index;
                    product_valid<=1;
                    if (axis_index == 6) begin
                        axis_index<=0;
                        if (group_index == 4) publish_pending<=1;
                        else group_index<=group_index+1'b1;
                    end else axis_index<=axis_index+1'b1;
                end else begin
                    product_valid<=0;
                    // E36 forms last partials; E37 stores their full sum.
                    // E38 publishes only after BOTH valid stages have drained.
                    if (!product_valid && !partial_valid) begin
                        busy<=0; done<=1; publish_pending<=0;
                        if (bad_job === 1'b0 && ((cfg_epoch == epoch_reg) === 1'b1)) begin
                            cache_valid<=1; rejected<=0; cached_cfg_epoch<=epoch_reg;
                            for (axis=0; axis<7; axis=axis+1) begin
                                cached_lower_qden[192*axis+:192]<={{30{scratch_lower[162*axis+161]}},scratch_lower[162*axis+:162]};
                                cached_upper_qden[192*axis+:192]<={{30{scratch_upper[162*axis+161]}},scratch_upper[162*axis+:162]};
                                cached_dq_qden[192*axis+:192]<={{30{scratch_dq[162*axis+161]}},scratch_dq[162*axis+:162]};
                                cached_vmax_vden[192*axis+:192]<={{30{scratch_vmax[162*axis+161]}},scratch_vmax[162*axis+:162]};
                                cached_amax_c[192*axis+:192]<={{30{scratch_amax[162*axis+161]}},scratch_amax[162*axis+:162]};
                            end
                        end else begin
                            cache_valid<=0; rejected<=1; cached_cfg_epoch<=0;
                            cached_lower_qden<='0; cached_upper_qden<='0; cached_dq_qden<='0;
                            cached_vmax_vden<='0; cached_amax_c<='0;
                        end
                    end
                end
            end
        end
    end
endmodule

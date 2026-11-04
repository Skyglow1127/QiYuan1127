`timescale 1ns/1ps
// DESIGN-ONLY exact Q32.32 constant-jerk arithmetic. No transport or enable I/O.
// start is accepted only when !busy. Inputs are captured on that edge E0.
// done pulses on E78; invalid dt/overflow have the same latency and zero outputs.
// Every axis occupies [64*axis +: 64], in the surrounding protocol's axis order.
module player_math #(
    parameter integer CLOCK_HZ = 100000000
) (
    input  logic         clk,
    input  logic         resetn,
    input  logic         start,
    input  logic [31:0]  dt_cycles,
    input  logic [447:0] q0,
    input  logic [447:0] v0,
    input  logic [447:0] a0,
    input  logic [447:0] j,
    output logic         busy,
    output logic         done,
    output logic [447:0] q1,
    output logic [447:0] v1,
    output logic [447:0] a1,
    output logic         overflow,
    // Read-only exact products for the PL certificate pipeline. They are
    // consumed only with this instance's done and !overflow, never PS input.
    output wire [1343:0] qp0_terms, qp1_terms, qp2_terms, qp3_terms,
    output wire [1343:0] vp0_terms, vp1_terms, vp2_terms, ap0_terms, ap1_terms
);
    // Positive signed-integer clock metadata; exact products extend to 192 bits.
    // No reference/PLL setting or physical execution rate is selected here.
    localparam logic signed [191:0] C = CLOCK_HZ;
    localparam logic signed [191:0] C2 = C * C;
    localparam logic signed [191:0] C3 = C2 * C;
    localparam logic signed [191:0] Q_DEN = 192'sd6 * C3;
    localparam logic signed [191:0] V_DEN = 192'sd2 * C2;
    localparam logic [191:0] POS_MAX = {128'b0, 64'h7fffffffffffffff};
    localparam logic [191:0] NEG_MAG_MAX = {128'b0, 64'h8000000000000000};

    generate if (CLOCK_HZ <= 0) begin : invalid_clock_hz
        initial $fatal(1, "CLOCK_HZ must be a positive signed integer");
    end endgenerate

    logic [6:0] phase;
    logic [31:0] d_reg;
    logic [63:0] d_squared;
    logic [95:0] d_cubed;
    logic signed [63:0] q_reg [0:6];
    logic signed [63:0] v_reg [0:6];
    logic signed [63:0] a_reg [0:6];
    logic signed [63:0] j_reg [0:6];
    // Exact all-profile bounds for C in 1..2^31-1 and d in 0..2^32-1.
    // Positive coefficient magnitudes need 97/97/96/64/64/32 bits;
    // the extra leading sign bit must remain zero after E3.
    logic signed [97:0] k_qv, k_qa;
    logic signed [96:0] k_qj;
    logic signed [64:0] k_va, k_vj;
    logic signed [32:0] k_aj;
    logic signed [191:0] qp0 [0:6], qp1 [0:6], qp2 [0:6], qp3 [0:6];
    logic signed [191:0] vp0 [0:6], vp1 [0:6], vp2 [0:6];
    logic signed [191:0] ap0 [0:6], ap1 [0:6];
    logic signed [191:0] numerator [0:20];
    // Prepared low66 dividend and seed remainder per q/v/a lane. The remainder
    // storage becomes the exact final remainder at its original slot boundary.
    // N >= D*2^64 is already guaranteed to overflow either signed64 sign, and
    // is explicitly flagged (never silently truncated) before initializing 0.
    logic [65:0] dividend [0:20], quotient [0:20];
    logic [95:0] remainder [0:20];
    logic [20:0] negative, early_bad, round_bad;
    logic [63:0] rounded [0:20];
    logic invalid_dt;
    integer axis;

    // One exact frontend multiplier bank per axis; division uses 64 binary steps.
    // All legal coefficients fit positive signed98. Split the signed65 operand
    // into unsigned low32 (zero-extended signed33) and signed high33. Both
    // full partial products are registered, then sign-extended/recombined at
    // signed163 before exact signed192 extension. No result bits are dropped.
    logic signed [97:0] mul_coefficient;
    logic signed [64:0] mul_operand [0:6];
    // Original E1..E9 register selected mux operands. E2..E10 register complete
    // 33x98/33x98 partials from the PRIOR operand edge. E3..E11 capture the
    // recombined product from the PRIOR partial edge. E10 does not relaunch;
    // E11 now launches reciprocalN0 while consuming the LAST frontend term.
    // No data-register reset is needed: every first capture has two preceding
    // pipeline launches in this job; reset/start control itself is unchanged.
    // All full-width products and original certificate outputs are retained.
    logic signed [97:0] mul_coefficient_reg;
    logic signed [64:0] mul_operand_reg [0:6];
    wire signed [32:0] mul_low_operand [0:6];
    wire signed [32:0] mul_high_operand [0:6];
    wire signed [130:0] mul_low_product [0:6];
    wire signed [130:0] mul_high_product [0:6];
    logic signed [130:0] mul_low_product_reg [0:6];
    logic signed [130:0] mul_high_product_reg [0:6];
    wire signed [162:0] mul_low_product163 [0:6], mul_high_product163 [0:6];
    wire signed [162:0] mul_product [0:6];
    wire signed [191:0] mul_product192 [0:6];
    always_comb begin
        mul_coefficient = 98'sd0;
        case (phase)
            7'd1: mul_coefficient = C[97:0];
            7'd2: mul_coefficient = V_DEN[97:0];
            7'd3: mul_coefficient = Q_DEN[97:0];
            7'd4: mul_coefficient = k_qv;
            7'd5: mul_coefficient = k_qa;
            7'd6: mul_coefficient = {{1{k_qj[96]}}, k_qj};
            7'd7: mul_coefficient = {{33{k_va[64]}}, k_va};
            7'd8: mul_coefficient = {{33{k_vj[64]}}, k_vj};
            7'd9: mul_coefficient = {{65{k_aj[32]}}, k_aj};
            default: mul_coefficient = 98'sd0;
        endcase
        for (integer pick_axis = 0; pick_axis < 7; pick_axis = pick_axis + 1) begin
            mul_operand[pick_axis] = 65'sd0;
            case (phase)
                7'd1, 7'd5, 7'd7: mul_operand[pick_axis] = $signed({a_reg[pick_axis][63],a_reg[pick_axis]});
                7'd2, 7'd4: mul_operand[pick_axis] = $signed({v_reg[pick_axis][63],v_reg[pick_axis]});
                7'd3: mul_operand[pick_axis] = $signed({q_reg[pick_axis][63],q_reg[pick_axis]});
                7'd6, 7'd8, 7'd9: mul_operand[pick_axis] = $signed({j_reg[pick_axis][63],j_reg[pick_axis]});
                default: mul_operand[pick_axis] = 65'sd0;
            endcase
        end
    end
    generate for (genvar mul_axis = 0; mul_axis < 7; mul_axis = mul_axis + 1) begin : state_multiplier
        assign mul_low_operand[mul_axis] = $signed({1'b0, mul_operand_reg[mul_axis][31:0]});
        assign mul_high_operand[mul_axis] = $signed(mul_operand_reg[mul_axis][64:32]);
        assign mul_low_product[mul_axis] = mul_low_operand[mul_axis] * mul_coefficient_reg;
        assign mul_high_product[mul_axis] = mul_high_operand[mul_axis] * mul_coefficient_reg;
        assign mul_low_product163[mul_axis] = {{32{mul_low_product_reg[mul_axis][130]}}, mul_low_product_reg[mul_axis]};
        assign mul_high_product163[mul_axis] = {{32{mul_high_product_reg[mul_axis][130]}}, mul_high_product_reg[mul_axis]};
        // Shift only AFTER 163-bit sign extension. A shift in 131 bits would
        // discard necessary high bits for negative/full-width input states.
        assign mul_product[mul_axis] = mul_low_product163[mul_axis] +
                                      (mul_high_product163[mul_axis] <<< 32);
        assign mul_product192[mul_axis] = {{29{mul_product[mul_axis][162]}}, mul_product[mul_axis]};
        assign qp0_terms[192*mul_axis+:192] = qp0[mul_axis];
        assign qp1_terms[192*mul_axis+:192] = qp1[mul_axis];
        assign qp2_terms[192*mul_axis+:192] = qp2[mul_axis];
        assign qp3_terms[192*mul_axis+:192] = qp3[mul_axis];
        assign vp0_terms[192*mul_axis+:192] = vp0[mul_axis];
        assign vp1_terms[192*mul_axis+:192] = vp1[mul_axis];
        assign vp2_terms[192*mul_axis+:192] = vp2[mul_axis];
        assign ap0_terms[192*mul_axis+:192] = ap0[mul_axis];
        assign ap1_terms[192*mul_axis+:192] = ap1[mul_axis];
    end endgenerate

    function automatic logic [191:0] denominator(input integer index);
        case (index % 3)
            0: denominator = Q_DEN;
            1: denominator = V_DEN;
            default: denominator = C;
        endcase
    endfunction

    // For accepted |N| < D*2^64, the seed R=|N|>>64 is <D.
    // Each unsigned97 subtraction consumes one low dividend bit. Since
    // -D <= (2R+b-D) < D and D fits96, bit96 is the exact borrow sign.
    wire [96:0] div_shift [0:20], div_difference [0:20];
    generate for (genvar di=0;di<21;di=di+1) begin : binary_division
        localparam logic [191:0] DENOM = denominator(di);
        assign div_shift[di] = {remainder[di],dividend[di][63]};
        assign div_difference[di] = div_shift[di] - {1'b0,DENOM[95:0]};
    end endgenerate

    function automatic logic [64:0] reduce_rne (
        input logic [191:0] magnitude_quotient,
        input logic [191:0] exact_remainder,
        input logic [191:0] divisor,
        input logic is_negative
    );
        logic [191:0] magnitude, twice_remainder;
        logic bad;
        logic [63:0] value;
        begin
            magnitude = magnitude_quotient;
            twice_remainder = exact_remainder << 1;
            if ((twice_remainder > divisor) ||
                ((twice_remainder == divisor) && magnitude[0]))
                magnitude = magnitude + 192'd1;
            bad = is_negative ? (magnitude > NEG_MAG_MAX) : (magnitude > POS_MAX);
            value = is_negative ? ((~magnitude[63:0]) + 64'd1) : magnitude[63:0];
            reduce_rne = {bad, value};
        end
    endfunction

    always_ff @(posedge clk) begin
        if (!resetn) begin
            busy <= 1'b0;
            done <= 1'b0;
            overflow <= 1'b0;
            phase <= 7'd0;
            q1 <= '0;
            v1 <= '0;
            a1 <= '0;
            invalid_dt <= 1'b0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (start) begin
                    busy <= 1'b1;
                    overflow <= 1'b0;
                    phase <= 7'd1;
                    d_reg <= dt_cycles;
                    invalid_dt <= (dt_cycles == 0);
                    for (axis = 0; axis < 7; axis = axis + 1) begin
                        q_reg[axis] <= $signed(q0[64*axis +: 64]);
                        v_reg[axis] <= $signed(v0[64*axis +: 64]);
                        a_reg[axis] <= $signed(a0[64*axis +: 64]);
                        j_reg[axis] <= $signed(j[64*axis +: 64]);
                    end
                end
            end else begin
                phase <= phase + 7'd1;
                // Launch all nine operations using the existing phase mux.
                // k_* updated at E3 is first sampled here at E4 (v*k_qv term).
                if ((phase >= 7'd1) && (phase <= 7'd9)) begin
                    mul_coefficient_reg <= mul_coefficient;
                    for (axis = 0; axis < 7; axis = axis + 1)
                        mul_operand_reg[axis] <= mul_operand[axis];
                end
                // Complete partial products are a distinct pipeline stage;
                // the case below consumes their PRIOR edge's registered values.
                if ((phase >= 7'd2) && (phase <= 7'd10)) begin
                    for (axis = 0; axis < 7; axis = axis + 1) begin
                        mul_low_product_reg[axis] <= mul_low_product[axis];
                        mul_high_product_reg[axis] <= mul_high_product[axis];
                    end
                end
                case (phase)
                    7'd1: begin
                        // E1 launch a*C; coefficient preparation stays at E1..3.
                        d_squared <= {32'b0, d_reg} * {32'b0, d_reg};
                    end
                    7'd2: begin
                        // E2 locks a*C partials; launch v*V_DEN.
                        d_cubed <= {32'b0, d_squared} * {64'b0, d_reg};
                    end
                    7'd3: begin
                        k_qv <= (192'sd6 * C2) * $signed({160'b0, d_reg});
                        k_qa <= (192'sd3 * C) * $signed({128'b0, d_squared});
                        k_qj <= $signed({96'b0, d_cubed});
                        k_va <= (192'sd2 * C) * $signed({160'b0, d_reg});
                        k_vj <= $signed({128'b0, d_squared});
                        k_aj <= $signed({160'b0, d_reg});
                        for (axis = 0; axis < 7; axis = axis + 1)
                            ap0[axis] <= mul_product192[axis];
                    end
                    7'd4: begin
                        for (axis = 0; axis < 7; axis = axis + 1)
                            vp0[axis] <= mul_product192[axis];
                    end
                    7'd5: begin
                        for (axis = 0; axis < 7; axis = axis + 1)
                            qp0[axis] <= mul_product192[axis];
                    end
                    7'd6: begin
                        for (axis = 0; axis < 7; axis = axis + 1)
                            qp1[axis] <= mul_product192[axis];
                    end
                    7'd7: begin
                        for (axis = 0; axis < 7; axis = axis + 1)
                            qp2[axis] <= mul_product192[axis];
                    end
                    7'd8: begin
                        for (axis = 0; axis < 7; axis = axis + 1)
                            qp3[axis] <= mul_product192[axis];
                    end
                    7'd9: begin
                        for (axis = 0; axis < 7; axis = axis + 1) begin
                            vp1[axis] <= mul_product192[axis];
                        end
                    end
                    7'd10: begin
                        for (axis = 0; axis < 7; axis = axis + 1)
                            vp2[axis] <= mul_product192[axis];
                    end
                    7'd11: begin
                        for (axis = 0; axis < 7; axis = axis + 1)
                            ap1[axis] <= mul_product192[axis];
                    end
                    7'd78: begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        phase <= '0;
                        overflow <= invalid_dt | (|round_bad);
                        if (invalid_dt | (|round_bad)) begin
                            q1 <= '0;
                            v1 <= '0;
                            a1 <= '0;
                        end else begin
                            for (axis = 0; axis < 7; axis = axis + 1) begin
                                q1[64*axis +: 64] <= rounded[3*axis];
                                v1[64*axis +: 64] <= rounded[3*axis+1];
                                a1[64*axis +: 64] <= rounded[3*axis+2];
                            end
                        end
                    end
                    default: begin end
                endcase

            end
        end
    end
    // Each lane owns its registers through constant indices. Keep the original
    // E10/E12 seeding, 64 division steps, E77 RNE and E78 publication schedule.
    generate for(genvar dl=0;dl<21;dl=dl+1) begin : division_lane
        localparam integer AXIS=dl/3;
        localparam integer KIND=dl%3;
        localparam integer SEED_PHASE=(KIND==0)?10:12;
        localparam integer FIRST_PHASE=(KIND==0)?11:13;
        localparam integer LAST_PHASE=(KIND==0)?74:76;
        localparam logic [191:0] DEN=denominator(dl);
        wire [191:0] raw=(KIND==2)?(ap0[AXIS]+ap1[AXIS]):numerator[dl];
        wire [191:0] magnitude=raw[191]?(~raw+192'd1):raw;
        wire too_large=(magnitude >= (DEN << 64));
        wire [64:0] rne=reduce_rne({126'b0,quotient[dl]},
                                 {96'b0,remainder[dl]},DEN,negative[dl]);
        always_ff @(posedge clk) begin
            if(!resetn) begin
                negative[dl]<=0;early_bad[dl]<=0;round_bad[dl]<=0;
            end else if(!busy) begin
                if(start) begin early_bad[dl]<=0;round_bad[dl]<=0;end
            end else begin
                if(KIND==0 && phase==7'd9)
                    numerator[dl]<=qp0[AXIS]+qp1[AXIS]+qp2[AXIS]+qp3[AXIS];
                if(KIND==1 && phase==7'd11)
                    numerator[dl]<=vp0[AXIS]+vp1[AXIS]+vp2[AXIS];
                if(phase==SEED_PHASE) begin
                    negative[dl]<=raw[191];early_bad[dl]<=too_large;
                    dividend[dl]<=too_large?66'd0:{2'b0,magnitude[63:0]};
                    remainder[dl]<=too_large?96'd0:(magnitude>>64);
                    quotient[dl]<=0;
                end
                if(phase>=FIRST_PHASE && phase<=LAST_PHASE) begin
                    dividend[dl]<={2'b0,dividend[dl][62:0],1'b0};
                    quotient[dl]<={quotient[dl][64:0],!div_difference[dl][96]};
                    remainder[dl]<=div_difference[dl][96]?div_shift[dl][95:0]:div_difference[dl][95:0];
                end
                if(phase==7'd77) begin
                    round_bad[dl]<=early_bad[dl]|rne[64];rounded[dl]<=rne[63:0];
                end
            end
        end
    end endgenerate

endmodule

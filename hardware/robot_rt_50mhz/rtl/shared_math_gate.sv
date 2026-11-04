`timescale 1ns/1ps
// Resource-only CANDIDATE. No hardware timing/resource/area claim.
// One exact math + one certificate gate, with no queue. A math grant is
// combinational before accepted E0 and is passed directly to player_math.start.
// Raw/limit/previous-wire and cache-valid freeze at E0. Cache products also
// freeze by default. STABLE_CACHE_INPUTS=1 is ONLY for the proven PL-global
// immutable configuration cache used by robot_rt_top; never a PS certificate.
// Requests must be known 0/1. Runtime has priority; validator also needs slot.
// The scheduler MUST reserve >=100 cycles until the next runtime math request.
// Normal runtime: math done E78, gate accept E79, gate done E94, release E95.
// Normal validator FSM: gate accept E80, gate done E95, release E96.
// Done/raw-result bypass preserves these edges; saved per-client results persist
// while the other client runs. Consumer observes the usual synchronous pulse.
// Cancelling either owner suppresses its events and drains the child currently
// in flight. Cancellation never resets either child or the unrelated owner.
// Any new runtime request while owned is a sticky collision. A stuck owner on
// E104 raises protocol_fault but is not silently aborted; parent must cancel.
module shared_math_gate #(
    parameter integer CLOCK_HZ=100000000,
    parameter integer STABLE_CACHE_INPUTS=0
) (
    input logic clk,
    input logic resetn,
    input logic rt_cancel,
    input logic val_cancel,
    input logic val_slot_admit,
    input logic rt_math_req,
    input logic rt_gate_req,
    input logic [31:0] rt_dt_cycles,
    input logic [447:0] rt_q0,
    input logic [447:0] rt_v0,
    input logic [447:0] rt_a0,
    input logic [447:0] rt_j,
    input logic [447:0] rt_lower,
    input logic [447:0] rt_upper,
    input logic [447:0] rt_velocity_limit,
    input logic [447:0] rt_acceleration_limit,
    input logic [447:0] rt_jerk_limit,
    input logic [447:0] rt_dq_max,
    input logic [223:0] rt_previous_wire,
    input logic rt_previous_wire_valid,
    input logic rt_cache_valid,
    input logic [1343:0] rt_cached_lower_qden,
    input logic [1343:0] rt_cached_upper_qden,
    input logic [1343:0] rt_cached_dq_qden,
    input logic [1343:0] rt_cached_vmax_vden,
    input logic [1343:0] rt_cached_amax_c,
    output logic rt_math_grant,
    output logic rt_gate_grant,
    output logic rt_math_busy,
    output logic rt_math_done,
    output logic rt_math_overflow,
    output logic [447:0] rt_q1,
    output logic [447:0] rt_v1,
    output logic [447:0] rt_a1,
    // Register-only late-consumption view; no additional clock stage.
    // Validity is supplied by the caller fresh-job ready/cancel guards.
    output wire [447:0] rt_q1_latched,
    output wire [447:0] rt_v1_latched,
    output wire [447:0] rt_a1_latched,
    output logic rt_gate_busy,
    output logic rt_gate_done,
    output logic rt_allow,
    output logic rt_clamp,
    output logic [223:0] rt_q16,
    output logic [223:0] rt_wire_values,
    input logic val_math_req,
    input logic val_gate_req,
    input logic [31:0] val_dt_cycles,
    input logic [447:0] val_q0,
    input logic [447:0] val_v0,
    input logic [447:0] val_a0,
    input logic [447:0] val_j,
    input logic [447:0] val_lower,
    input logic [447:0] val_upper,
    input logic [447:0] val_velocity_limit,
    input logic [447:0] val_acceleration_limit,
    input logic [447:0] val_jerk_limit,
    input logic [447:0] val_dq_max,
    input logic [223:0] val_previous_wire,
    input logic val_previous_wire_valid,
    input logic val_cache_valid,
    input logic [1343:0] val_cached_lower_qden,
    input logic [1343:0] val_cached_upper_qden,
    input logic [1343:0] val_cached_dq_qden,
    input logic [1343:0] val_cached_vmax_vden,
    input logic [1343:0] val_cached_amax_c,
    output logic val_math_grant,
    output logic val_gate_grant,
    output logic val_math_busy,
    output logic val_math_done,
    output logic val_math_overflow,
    output logic [447:0] val_q1,
    output logic [447:0] val_v1,
    output logic [447:0] val_a1,
    output logic val_gate_busy,
    output logic val_gate_done,
    output logic val_allow,
    output logic val_clamp,
    output logic [223:0] val_q16,
    output logic [223:0] val_wire_values,
    output logic busy,
    output logic owner_is_validator,
    output logic [7:0] owner_age,
    output logic runtime_collision,
    output logic protocol_fault
);
    logic [31:0] ctx_dt_cycles;
    logic [447:0] ctx_q0;
    logic [447:0] ctx_v0;
    logic [447:0] ctx_a0;
    logic [447:0] ctx_j;
    logic [447:0] ctx_lower;
    logic [447:0] ctx_upper;
    logic [447:0] ctx_velocity_limit;
    logic [447:0] ctx_acceleration_limit;
    logic [447:0] ctx_jerk_limit;
    logic [447:0] ctx_dq_max;
    logic [223:0] ctx_previous_wire;
    logic ctx_previous_wire_valid;
    logic ctx_cache_valid;
    logic [1343:0] ctx_cached_lower_qden;
    logic [1343:0] ctx_cached_upper_qden;
    logic [1343:0] ctx_cached_dq_qden;
    logic [1343:0] ctx_cached_vmax_vden;
    logic [1343:0] ctx_cached_amax_c;
    // Enabled only when BOTH client cache-product ports are tied to the same
    // PL cache, stable for the entire owner job. Use rt_* as canonical wires.
    // ctx_cache_valid and all original raw/high-bit/epoch guards are unchanged.
    wire [1343:0] gate_cached_lower_qden,gate_cached_upper_qden,gate_cached_dq_qden;
    wire [1343:0] gate_cached_vmax_vden,gate_cached_amax_c;
    generate if(STABLE_CACHE_INPUTS==1) begin : global_stable_cache
        assign gate_cached_lower_qden=rt_cached_lower_qden;
        assign gate_cached_upper_qden=rt_cached_upper_qden;
        assign gate_cached_dq_qden=rt_cached_dq_qden;
        assign gate_cached_vmax_vden=rt_cached_vmax_vden;
        assign gate_cached_amax_c=rt_cached_amax_c;
    end else begin : snapshotted_cache
        assign gate_cached_lower_qden=ctx_cached_lower_qden;
        assign gate_cached_upper_qden=ctx_cached_upper_qden;
        assign gate_cached_dq_qden=ctx_cached_dq_qden;
        assign gate_cached_vmax_vden=ctx_cached_vmax_vden;
        assign gate_cached_amax_c=ctx_cached_amax_c;
    end endgenerate
    initial begin
        if(STABLE_CACHE_INPUTS!=0 && STABLE_CACHE_INPUTS!=1)
            $fatal(1,"STABLE_CACHE_INPUTS must be0 or1");
    end
    logic owner_valid, discarded, math_ready, gate_started;
    logic owner_cancel, owner_gate_req, accept_rt, accept_val;
    logic shared_math_start, shared_gate_start;
    logic m_busy,m_done,m_overflow,g_busy,g_done,g_allow,g_clamp;
    logic [447:0] m_q1,m_v1,m_a1;
    logic [223:0] g_q16,g_wire;
    wire [1343:0] qp0_terms;
    wire [1343:0] qp1_terms;
    wire [1343:0] qp2_terms;
    wire [1343:0] qp3_terms;
    wire [1343:0] vp0_terms;
    wire [1343:0] vp1_terms;
    wire [1343:0] vp2_terms;
    wire [1343:0] ap0_terms;
    wire [1343:0] ap1_terms;
    logic [447:0] rt_q1_saved;
    logic [447:0] rt_v1_saved;
    logic [447:0] rt_a1_saved;
    logic rt_overflow_saved;
    logic rt_allow_saved;
    logic rt_clamp_saved;
    logic [223:0] rt_q16_saved;
    logic [223:0] rt_wire_saved;
    logic [447:0] val_q1_saved;
    logic [447:0] val_v1_saved;
    logic [447:0] val_a1_saved;
    logic val_overflow_saved;
    logic val_allow_saved;
    logic val_clamp_saved;
    logic [223:0] val_q16_saved;
    logic [223:0] val_wire_saved;

    // Deliberately no rt_math_done mux on these late-stage outputs.
    assign rt_q1_latched=rt_q1_saved;
    assign rt_v1_latched=rt_v1_saved;
    assign rt_a1_latched=rt_a1_saved;

    always_comb begin
        busy=owner_valid;
        owner_cancel=owner_is_validator ? val_cancel : rt_cancel;
        owner_gate_req=owner_is_validator ? val_gate_req : rt_gate_req;
        accept_rt=(resetn===1'b1) && !owner_valid && !m_busy && !g_busy &&
                  (rt_math_req===1'b1) && (rt_cancel===1'b0);
        accept_val=(resetn===1'b1) && !owner_valid && !m_busy && !g_busy &&
                   (rt_math_req===1'b0) && (val_math_req===1'b1) &&
                   (val_slot_admit===1'b1) && (val_cancel===1'b0);
        rt_math_grant=accept_rt;
        val_math_grant=accept_val;
        shared_math_start=accept_rt||accept_val;
        // raw m_done permits runtime gate E79; a register-only readiness test
        // here would delay gate acceptance by one edge.
        shared_gate_start=(resetn===1'b1) && owner_valid && !discarded && (owner_cancel===1'b0) &&
            (owner_gate_req===1'b1) && !gate_started && !g_busy &&
            (m_done||math_ready) && (m_overflow===1'b0);
        rt_gate_grant=shared_gate_start&&!owner_is_validator;
        val_gate_grant=shared_gate_start&&owner_is_validator;
        rt_math_busy=owner_valid&&!owner_is_validator&&m_busy;
        val_math_busy=owner_valid&&owner_is_validator&&m_busy;
        rt_gate_busy=owner_valid&&!owner_is_validator&&g_busy;
        val_gate_busy=owner_valid&&owner_is_validator&&g_busy;
        rt_math_done=(resetn===1'b1)&&owner_valid&&!owner_is_validator&&
            !discarded&&(rt_cancel===1'b0)&&m_done;
        val_math_done=(resetn===1'b1)&&owner_valid&&owner_is_validator&&
            !discarded&&(val_cancel===1'b0)&&m_done;
        rt_gate_done=(resetn===1'b1)&&owner_valid&&!owner_is_validator&&
            !discarded&&(rt_cancel===1'b0)&&g_done;
        val_gate_done=(resetn===1'b1)&&owner_valid&&owner_is_validator&&
            !discarded&&(val_cancel===1'b0)&&g_done;
        rt_q1=rt_math_done ? m_q1 : rt_q1_saved;
        rt_v1=rt_math_done ? m_v1 : rt_v1_saved;
        rt_a1=rt_math_done ? m_a1 : rt_a1_saved;
        rt_math_overflow=rt_math_done ? m_overflow : rt_overflow_saved;
        rt_allow=rt_gate_done ? g_allow : rt_allow_saved;
        rt_clamp=rt_gate_done ? g_clamp : rt_clamp_saved;
        rt_q16=rt_gate_done ? g_q16 : rt_q16_saved;
        rt_wire_values=rt_gate_done ? g_wire : rt_wire_saved;
        val_q1=val_math_done ? m_q1 : val_q1_saved;
        val_v1=val_math_done ? m_v1 : val_v1_saved;
        val_a1=val_math_done ? m_a1 : val_a1_saved;
        val_math_overflow=val_math_done ? m_overflow : val_overflow_saved;
        val_allow=val_gate_done ? g_allow : val_allow_saved;
        val_clamp=val_gate_done ? g_clamp : val_clamp_saved;
        val_q16=val_gate_done ? g_q16 : val_q16_saved;
        val_wire_values=val_gate_done ? g_wire : val_wire_saved;
    end

    player_math #(.CLOCK_HZ(CLOCK_HZ)) math (
        .clk(clk),
        .resetn(resetn),
        .start(shared_math_start),
        .dt_cycles(accept_val ? val_dt_cycles : rt_dt_cycles),
        .q0(accept_val ? val_q0 : rt_q0),
        .v0(accept_val ? val_v0 : rt_v0),
        .a0(accept_val ? val_a0 : rt_a0),
        .j(accept_val ? val_j : rt_j),
        .busy(m_busy),
        .done(m_done),
        .q1(m_q1),
        .v1(m_v1),
        .a1(m_a1),
        .overflow(m_overflow),
        .qp0_terms(qp0_terms),
        .qp1_terms(qp1_terms),
        .qp2_terms(qp2_terms),
        .qp3_terms(qp3_terms),
        .vp0_terms(vp0_terms),
        .vp1_terms(vp1_terms),
        .vp2_terms(vp2_terms),
        .ap0_terms(ap0_terms),
        .ap1_terms(ap1_terms)
    );
    limit_gate_pipe #(.CLOCK_HZ(CLOCK_HZ),.STABLE_JOB_INPUTS(STABLE_CACHE_INPUTS)) gate (
        .clk(clk),
        .resetn(resetn),
        .start(shared_gate_start),
        .terms_valid(m_done||math_ready),
        .math_overflow(m_overflow),
        .dt_cycles(ctx_dt_cycles),
        .q0(ctx_q0),
        .v0(ctx_v0),
        .a0(ctx_a0),
        .j(ctx_j),
        .lower(ctx_lower),
        .upper(ctx_upper),
        .velocity_limit(ctx_velocity_limit),
        .acceleration_limit(ctx_acceleration_limit),
        .jerk_limit(ctx_jerk_limit),
        .dq_max(ctx_dq_max),
        .previous_wire(ctx_previous_wire),
        .previous_wire_valid(ctx_previous_wire_valid),
        .cache_valid(ctx_cache_valid),
        .cached_lower_qden(gate_cached_lower_qden),
        .cached_upper_qden(gate_cached_upper_qden),
        .cached_dq_qden(gate_cached_dq_qden),
        .cached_vmax_vden(gate_cached_vmax_vden),
        .cached_amax_c(gate_cached_amax_c),
        .q1(m_q1),
        .v1(m_v1),
        .a1(m_a1),
        .qp0_terms(qp0_terms),
        .qp1_terms(qp1_terms),
        .qp2_terms(qp2_terms),
        .qp3_terms(qp3_terms),
        .vp0_terms(vp0_terms),
        .vp1_terms(vp1_terms),
        .vp2_terms(vp2_terms),
        .ap0_terms(ap0_terms),
        .ap1_terms(ap1_terms),
        .busy(g_busy),
        .done(g_done),
        .allow(g_allow),
        .clamp(g_clamp),
        .q16(g_q16),
        .wire_values(g_wire)
    );

    always_ff @(posedge clk) begin
        if(!resetn) begin
            owner_valid<=0; owner_is_validator<=0; owner_age<=0;
            discarded<=0; math_ready<=0; gate_started<=0;
            runtime_collision<=0; protocol_fault<=0;
            rt_q1_saved<=0;rt_v1_saved<=0;rt_a1_saved<=0;rt_overflow_saved<=0;
            rt_allow_saved<=0;rt_clamp_saved<=0;rt_q16_saved<=0;rt_wire_saved<=0;
            val_q1_saved<=0;val_v1_saved<=0;val_a1_saved<=0;val_overflow_saved<=0;
            val_allow_saved<=0;val_clamp_saved<=0;val_q16_saved<=0;val_wire_saved<=0;
            // Context has no reset load: valid/ownership and both child resets
            // prevent consumption until every bit is captured by an accepted E0.
        end else begin
            if((rt_math_req===1'b1)&&owner_valid&&(rt_cancel===1'b0)) begin
                runtime_collision<=1'b1; protocol_fault<=1'b1;
            end
            if(shared_math_start) begin
                owner_valid<=1'b1; owner_is_validator<=accept_val;
                owner_age<=0; discarded<=0; math_ready<=0; gate_started<=0;
                ctx_dt_cycles<=accept_val ? val_dt_cycles : rt_dt_cycles;
                ctx_q0<=accept_val ? val_q0 : rt_q0;
                ctx_v0<=accept_val ? val_v0 : rt_v0;
                ctx_a0<=accept_val ? val_a0 : rt_a0;
                ctx_j<=accept_val ? val_j : rt_j;
                ctx_lower<=accept_val ? val_lower : rt_lower;
                ctx_upper<=accept_val ? val_upper : rt_upper;
                ctx_velocity_limit<=accept_val ? val_velocity_limit : rt_velocity_limit;
                ctx_acceleration_limit<=accept_val ? val_acceleration_limit : rt_acceleration_limit;
                ctx_jerk_limit<=accept_val ? val_jerk_limit : rt_jerk_limit;
                ctx_dq_max<=accept_val ? val_dq_max : rt_dq_max;
                ctx_previous_wire<=accept_val ? val_previous_wire : rt_previous_wire;
                ctx_previous_wire_valid<=accept_val ? val_previous_wire_valid : rt_previous_wire_valid;
                ctx_cache_valid<=accept_val ? val_cache_valid : rt_cache_valid;
                if(STABLE_CACHE_INPUTS==0) begin
                    ctx_cached_lower_qden<=accept_val ? val_cached_lower_qden : rt_cached_lower_qden;
                    ctx_cached_upper_qden<=accept_val ? val_cached_upper_qden : rt_cached_upper_qden;
                    ctx_cached_dq_qden<=accept_val ? val_cached_dq_qden : rt_cached_dq_qden;
                    ctx_cached_vmax_vden<=accept_val ? val_cached_vmax_vden : rt_cached_vmax_vden;
                    ctx_cached_amax_c<=accept_val ? val_cached_amax_c : rt_cached_amax_c;
                end
            end else if(owner_valid) begin
                if(owner_age!=8'hff)owner_age<=owner_age+1'b1;
                if(owner_age>=8'd103)protocol_fault<=1'b1;
                if(owner_cancel!==1'b0)discarded<=1'b1;
                if(shared_gate_start)gate_started<=1'b1;
                if(m_done) begin
                    math_ready<=1'b1;
                    if(m_overflow!==1'b0)owner_valid<=1'b0;
                end
                if(g_done)owner_valid<=1'b0;
                // Once cancelled, do not demand a gate request after math.
                // An already started gate drains before ownership is released.
                if((discarded||(owner_cancel!==1'b0)) && !g_busy &&
                   (m_done||(!m_busy&&math_ready)))owner_valid<=1'b0;
            end
            if(rt_math_done) begin
                rt_q1_saved<=m_q1;rt_v1_saved<=m_v1;rt_a1_saved<=m_a1;
                rt_overflow_saved<=m_overflow;
            end
            if(rt_gate_done) begin
                rt_allow_saved<=g_allow;rt_clamp_saved<=g_clamp;
                rt_q16_saved<=g_q16;rt_wire_saved<=g_wire;
            end
            if(val_math_done) begin
                val_q1_saved<=m_q1;val_v1_saved<=m_v1;val_a1_saved<=m_a1;
                val_overflow_saved<=m_overflow;
            end
            if(val_gate_done) begin
                val_allow_saved<=g_allow;val_clamp_saved<=g_clamp;
                val_q16_saved<=g_q16;val_wire_saved<=g_wire;
            end
        end
    end
endmodule

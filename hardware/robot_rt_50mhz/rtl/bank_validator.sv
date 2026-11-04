`timescale 1ns/1ps
// Frozen-bank validator. All checks are hardware logic; no PS-supplied accept bit.
// Background arithmetic/division resource and timing are NOT synthesized/proven.
module bank_validator #(
    parameter integer CLOCK_HZ = 100000000,
    parameter integer EXTERNAL_EXECUTION = 0
) (
    input logic clk, resetn, start, bank,
    input logic [63:0] now_cycle, rate_origin, rate_num, rate_den,
    input logic [63:0] expected_sequence, last_accepted_sequence,
    input logic [63:0] session_nonce, source_max_age, stop_timeout,
    input logic [31:0] config_epoch, config_crc, max_map_error,
    input logic [63:0] expected_previous, parent_end_cycle, parent_end_tick,
    input logic [447:0] parent_q, parent_v, parent_a,
    input logic [223:0] parent_wire,
    input logic [447:0] lower, upper, velocity_limit, acceleration_limit, jerk_limit, dq_max,
    input logic cache_valid,
    input logic [31:0] cached_config_epoch,
    input logic [1343:0] cached_lower_qden, cached_upper_qden, cached_dq_qden,
    input logic [1343:0] cached_vmax_vden, cached_amax_c,
    output logic mem_req,
    output logic [16:0] mem_addr,
    input logic mem_grant, mem_valid,
    input logic [31:0] mem_data,
    output logic busy, done, accepted,
    output logic [3:0] reject_code,
    output logic [63:0] bank_sequence, previous_sequence,
    output logic [7:0] normal_count, brake_count,
    output logic [63:0] first_cycle, first_tick, normal_end_cycle, normal_end_tick, brake_end_cycle,
    output logic [63:0] observation_cycle, produced_cycle, normal_deadline,
    output logic [31:0] map_error,
    output logic [447:0] first_q, first_v, first_a, normal_end_q, normal_end_v, normal_end_a,
    output logic [223:0] normal_end_wire,
    // Optional PL-local shared execution channel. No PS acceptance input.
    // Requests/context remain stable until grant; result registers belong to
    // this validator and must survive the shared engine's next owner.
    output wire exec_math_req, exec_gate_req,
    output wire [31:0] exec_dt_cycles,
    output wire [447:0] exec_q0, exec_v0, exec_a0, exec_j,
    output wire [447:0] exec_lower, exec_upper, exec_velocity_limit,
    output wire [447:0] exec_acceleration_limit, exec_jerk_limit, exec_dq_max,
    output wire [223:0] exec_previous_wire,
    output wire exec_cache_valid,
    input logic exec_math_grant, exec_math_done, exec_math_overflow,
    input logic [447:0] exec_q1, exec_v1, exec_a1,
    input logic exec_gate_grant, exec_gate_done, exec_gate_allow, exec_gate_clamp,
    input logic [223:0] exec_q16, exec_wire
);
    initial begin
        if(CLOCK_HZ<=0) $fatal(1,"CLOCK_HZ must be a positive signed 32-bit integer");
    end
    typedef enum logic[4:0] {IDLE,HREQ,HWAIT,HCHECK,RREQ,RWAIT,RCHECK,MSTART,MWAIT,GCHECK,FINISH,FAIL,DIV_REQ,DIV_WAIT,H_TICK_CHECK,R_TICK_CHECK,GSTART,GWAIT,THRESHOLD_PREP,DIV_PRODUCT} state_t;
    state_t state;
    logic bank_reg;
    logic [31:0] h[0:63], r[0:63];
    logic [6:0] word_index;
    logic [7:0] record_index, total_count;
    logic [31:0] header_crc, payload_crc;
    logic [63:0] next_start, next_tick;
    logic [447:0] chain_q,chain_v,chain_a;
    logic [223:0] chain_wire;
    logic[63:0] num_reg,den_reg,origin_reg,sequence_reg,last_sequence_reg,nonce_reg,age_reg,timeout_reg,parent_sequence_reg,parent_cycle_reg,parent_tick_reg;
    logic[31:0] epoch_reg,config_crc_reg,max_error_reg;
    logic[447:0] lower_reg,upper_reg,velocity_reg,acceleration_reg,jerk_reg,dq_reg;
    logic math_start, math_busy, math_done, math_overflow;
    logic [447:0] rq,rv,ra,rj, end_q,end_v,end_a;
    logic [63:0] rec_start,rec_end,rec_tick;
    logic [31:0] rec_dt;
    logic gate_allow,gate_clamp;
    logic gate_busy,gate_done,gate_start;
    logic [223:0] gate_q16,gate_wire;
    logic [1343:0] qp0_terms,qp1_terms,qp2_terms,qp3_terms;
    logic [1343:0] vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms;
    // One shared admission-only exact divider. Original state encodings0..17
    // are retained; appended18/19 prepare full products on separate edges.
    // Active player timing is outside this module.
    logic div_record_mode,div_previous;
    logic div_start,div_busy,div_done,div_invalid,div_absolute_overflow;
    logic [63:0] div_index,div_absolute,div_end_absolute,div_start_absolute;
    logic [127:0] div_dividend,div_ceil;
    // Keep the COMPLETE threshold (including overflow high64) per bank job.
    // E0 captures den_reg; THRESHOLD_PREP captures den_reg*CLOCK_HZ at E1.
    // DIV_PRODUCT later captures index64*threshold64 after HCHECK validated it.
    // Neither multiplication cascades combinationally into the other or into
    // the divider's E0 input register. No precision or deadline is changed.
    logic [127:0] threshold_wide;
    wire [127:0] threshold_product={64'd0,den_reg}*{96'd0,CLOCK_HZ[31:0]};
    wire [127:0] index_product={64'd0,div_index}*{64'd0,threshold_wide[63:0]};
    integer i;

    function automatic logic[63:0] hu64(input integer w);
        hu64={h[w+1],h[w]};
    endfunction
    function automatic logic[63:0] ru64(input integer w);
        ru64={r[w+1],r[w]};
    endfunction
    function automatic logic[31:0] crc_word(input logic[31:0] crc,input logic[31:0] data);
        logic[31:0] c; integer b;
        begin
            c=crc;
            for(b=0;b<32;b=b+1) begin
                if(c[0]^data[b]) c=(c>>1)^32'hedb88320;
                else c=c>>1;
            end
            crc_word=c;
        end
    endfunction
    always_comb begin
        mem_req=(state==HREQ)||(state==RREQ);
        mem_addr={bank_reg,16'd0} + ((state==HREQ)?17'd0:17'd256+({9'd0,record_index}<<8)) + ({10'd0,word_index}<<2);
        rec_start=ru64(0); rec_end=ru64(2); rec_tick=ru64(4);
        rec_dt=rec_end-rec_start;
        rq='0;rv='0;ra='0;rj='0;
        for(integer a=0;a<7;a=a+1) begin
            rq[64*a+:64]=ru64(8+a*8);
            rv[64*a+:64]=ru64(10+a*8);
            ra[64*a+:64]=ru64(12+a*8);
            rj[64*a+:64]=ru64(14+a*8);
        end
        math_start=state==MSTART;
        gate_start=state==GSTART;
        div_index=div_record_mode?rec_tick:hu64(18);
        if(div_previous)div_index=div_index-64'd1;
        // HCHECK/RCHECK require index>0 and HCHECK rejects threshold high64
        // before DIV_PRODUCT. That edge saves the exact unsigned128 product.
        div_start=(state==DIV_REQ)&&!div_busy;
    end
    ceil_div128_64 admission_divider(.clk(clk),.resetn(resetn),.start(div_start),
        .dividend(div_dividend),.divisor(num_reg),.origin(origin_reg),
        .busy(div_busy),.done(div_done),.invalid(div_invalid),
        .absolute_overflow(div_absolute_overflow),.ceil_quotient(div_ceil),
        .absolute_cycle(div_absolute));
    assign exec_math_req=math_start;
    assign exec_gate_req=gate_start;
    assign exec_dt_cycles=rec_dt;
    assign exec_q0=rq;assign exec_v0=rv;assign exec_a0=ra;assign exec_j=rj;
    assign exec_lower=lower_reg;assign exec_upper=upper_reg;
    assign exec_velocity_limit=velocity_reg;assign exec_acceleration_limit=acceleration_reg;
    assign exec_jerk_limit=jerk_reg;assign exec_dq_max=dq_reg;
    assign exec_previous_wire=chain_wire;
    assign exec_cache_valid=cache_valid&&(cached_config_epoch==epoch_reg);
    generate if(EXTERNAL_EXECUTION==0) begin : internal_execution
    player_math #(.CLOCK_HZ(CLOCK_HZ)) validator_math(.clk(clk),.resetn(resetn),.start(math_start),.dt_cycles(rec_dt),
        .q0(rq),.v0(rv),.a0(ra),.j(rj),.busy(math_busy),.done(math_done),
        .q1(end_q),.v1(end_v),.a1(end_a),.overflow(math_overflow),
        .qp0_terms(qp0_terms),.qp1_terms(qp1_terms),.qp2_terms(qp2_terms),.qp3_terms(qp3_terms),
        .vp0_terms(vp0_terms),.vp1_terms(vp1_terms),.vp2_terms(vp2_terms),.ap0_terms(ap0_terms),.ap1_terms(ap1_terms));
    limit_gate_pipe #(.CLOCK_HZ(CLOCK_HZ)) validator_gate(.clk(clk),.resetn(resetn),.start(gate_start),
        .terms_valid(1'b1),.math_overflow(math_overflow),.cache_valid(cache_valid&&(cached_config_epoch==epoch_reg)),
        .qp0_terms(qp0_terms),.qp1_terms(qp1_terms),.qp2_terms(qp2_terms),.qp3_terms(qp3_terms),
        .vp0_terms(vp0_terms),.vp1_terms(vp1_terms),.vp2_terms(vp2_terms),.ap0_terms(ap0_terms),.ap1_terms(ap1_terms),
        .cached_lower_qden(cached_lower_qden),.cached_upper_qden(cached_upper_qden),.cached_dq_qden(cached_dq_qden),
        .cached_vmax_vden(cached_vmax_vden),.cached_amax_c(cached_amax_c),
        .q0(rq),.v0(rv),.a0(ra),.j(rj),.q1(end_q),.v1(end_v),.a1(end_a),
        .dt_cycles(rec_dt),.lower(lower_reg),.upper(upper_reg),.velocity_limit(velocity_reg),
        .acceleration_limit(acceleration_reg),.jerk_limit(jerk_reg),.dq_max(dq_reg),
        .previous_wire(chain_wire),.previous_wire_valid(1'b1),.allow(gate_allow),.clamp(gate_clamp),
        .q16(gate_q16),.wire_values(gate_wire),.busy(gate_busy),.done(gate_done));
    end else begin : external_execution
        always_comb begin
            math_busy=1'b0;gate_busy=1'b0;
            math_done=exec_math_done;math_overflow=exec_math_overflow;
            end_q=exec_q1;end_v=exec_v1;end_a=exec_a1;
            gate_done=exec_gate_done;gate_allow=exec_gate_allow;gate_clamp=exec_gate_clamp;
            gate_q16=exec_q16;gate_wire=exec_wire;
            qp0_terms='0;qp1_terms='0;qp2_terms='0;qp3_terms='0;
            vp0_terms='0;vp1_terms='0;vp2_terms='0;ap0_terms='0;ap1_terms='0;
        end
    end endgenerate

    always_ff @(posedge clk) begin
        if(!resetn) begin
            state<=IDLE;busy<=0;done<=0;accepted<=0;reject_code<=0;
            bank_reg<=0;word_index<=0;record_index<=0;total_count<=0;
            header_crc<=32'hffffffff;payload_crc<=32'hffffffff;
            bank_sequence<=0;previous_sequence<=0;normal_count<=0;brake_count<=0;
            first_cycle<=0;first_tick<=0;normal_end_cycle<=0;normal_end_tick<=0;brake_end_cycle<=0;
            observation_cycle<=0;produced_cycle<=0;normal_deadline<=0;map_error<=0;
            first_q<=0;first_v<=0;first_a<=0;normal_end_q<=0;normal_end_v<=0;normal_end_a<=0;normal_end_wire<=0;
            chain_q<=0;chain_v<=0;chain_a<=0;chain_wire<=0;next_start<=0;next_tick<=0;
            div_record_mode<=0;div_previous<=0;div_end_absolute<=0;div_start_absolute<=0;
            threshold_wide<=0;div_dividend<=0;
        end else begin
            done<=0;
            case(state)
                IDLE: if(start) begin
                    busy<=1;accepted<=0;reject_code<=0;bank_reg<=bank;
                    word_index<=0;record_index<=0;header_crc<=32'hffffffff;payload_crc<=32'hffffffff;
                    chain_q<=parent_q;chain_v<=parent_v;chain_a<=parent_a;chain_wire<=parent_wire;
                    num_reg<=rate_num;den_reg<=rate_den;origin_reg<=rate_origin;sequence_reg<=expected_sequence;last_sequence_reg<=last_accepted_sequence;nonce_reg<=session_nonce;
                    age_reg<=source_max_age;timeout_reg<=stop_timeout;epoch_reg<=config_epoch;config_crc_reg<=config_crc;max_error_reg<=max_map_error;
                    parent_sequence_reg<=expected_previous;parent_cycle_reg<=parent_end_cycle;parent_tick_reg<=parent_end_tick;
                    lower_reg<=lower;upper_reg<=upper;velocity_reg<=velocity_limit;acceleration_reg<=acceleration_limit;jerk_reg<=jerk_limit;dq_reg<=dq_max;
                    state<=THRESHOLD_PREP;
                end
                THRESHOLD_PREP:begin
                    threshold_wide<=threshold_product;state<=HREQ;
                end
                HREQ: if(mem_grant) state<=HWAIT;
                HWAIT: if(mem_valid) begin
                    h[word_index]<=mem_data;
                    header_crc<=crc_word(header_crc,(word_index==25)?32'd0:mem_data);
                    if(word_index==63) state<=HCHECK;
                    else begin word_index<=word_index+1;state<=HREQ;end
                end
                HCHECK: begin : validate_header
                    logic reserved_bad;
                    logic[127:0] obs,prod,deadline,err,now128,age;
                    reserved_bad=h[11]!=0;
                    for(integer k=28;k<64;k=k+1) if(h[k]!=0) reserved_bad=1;
                    obs={64'd0,hu64(12)};prod={64'd0,hu64(14)};deadline={64'd0,hu64(16)};
                    err={96'd0,h[27]};now128={64'd0,now_cycle};age={64'd0,age_reg};
                    if(h[0]!=32'h4b425452||h[1]!=1||h[2]!=256||h[3]!=256||reserved_bad) begin reject_code<=2;state<=FAIL;end
                    else if((~header_crc)!=h[25]) begin reject_code<=3;state<=FAIL;end
                    else if(hu64(6)!=nonce_reg||h[8]!=epoch_reg||h[26]!=config_crc_reg||cache_valid!==1'b1||cached_config_epoch!==epoch_reg) begin reject_code<=4;state<=FAIL;end
                    else if(hu64(4)==0||hu64(4)!=sequence_reg||hu64(4)<=last_sequence_reg||hu64(22)!=parent_sequence_reg) begin reject_code<=5;state<=FAIL;end
                    else if(h[9]==0||h[10]==0||{1'b0,h[9]}+{1'b0,h[10]}>255) begin reject_code<=10;state<=FAIL;end
                    else if(num_reg==0||den_reg==0||threshold_wide[127:64]!=0||h[27]>max_error_reg||obs<err||prod<err||obs+err>prod-err||prod+err>now128||now128+err>=deadline||obs-err+age>128'hffffffffffffffff||deadline>obs-err+age||deadline>128'hffffffffffffffff) begin reject_code<=6;state<=FAIL;end
                    else if(hu64(18)==0) begin reject_code<=6;state<=FAIL;end
                    else begin div_record_mode<=0;div_previous<=0;state<=DIV_PRODUCT;end
                end
                DIV_PRODUCT:begin
                    div_dividend<=index_product;state<=DIV_REQ;
                end
                DIV_REQ:if(!div_busy)state<=DIV_WAIT;
                DIV_WAIT:if(div_done)begin
                    if(div_invalid!==1'b0||div_absolute_overflow!==1'b0)begin reject_code<=6;state<=FAIL;end
                    else if(!div_previous)begin
                        div_end_absolute<=div_absolute;div_previous<=1;state<=DIV_PRODUCT;
                    end else begin
                        div_start_absolute<=div_absolute;
                        state<=div_record_mode?R_TICK_CHECK:H_TICK_CHECK;
                    end
                end
                H_TICK_CHECK:begin
                    if(div_start_absolute!==hu64(20))begin reject_code<=6;state<=FAIL;end
                    else if(parent_sequence_reg!=0&&(hu64(20)!=parent_cycle_reg||hu64(18)!=parent_tick_reg+1||parent_tick_reg==64'hffffffffffffffff)) begin reject_code<=8;state<=FAIL;end
                    else begin
                        bank_sequence<=hu64(4);previous_sequence<=hu64(22);
                        normal_count<=h[9][7:0];brake_count<=h[10][7:0];total_count<=h[9]+h[10];
                        observation_cycle<=hu64(12);produced_cycle<=hu64(14);normal_deadline<=hu64(16);map_error<=h[27];
                        first_cycle<=hu64(20);first_tick<=hu64(18);next_start<=hu64(20);next_tick<=hu64(18);
                        word_index<=0;state<=RREQ;
                    end
                end
                RREQ: if(mem_grant) state<=RWAIT;
                RWAIT: if(mem_valid) begin
                    r[word_index]<=mem_data;payload_crc<=crc_word(payload_crc,mem_data);
                    if(word_index==63) state<=RCHECK;
                    else begin word_index<=word_index+1;state<=RREQ;end
                end
                RCHECK: begin
                    if(r[7]!=0||r[6]!=((record_index<normal_count)?1:2)) begin reject_code<=2;state<=FAIL;end
                    else if(rec_tick==0||rec_tick!=next_tick||rec_start!=next_start||rec_end<=rec_start||rec_end-rec_start>64'hffffffff) begin reject_code<=6;state<=FAIL;end
                    else begin div_record_mode<=1;div_previous<=0;state<=DIV_PRODUCT;end
                end
                R_TICK_CHECK:begin : validate_record_times_and_chain
                    logic[127:0] delayed;
                    delayed={64'd0,rec_end}+128'd112+map_error;
                    if(div_end_absolute!==rec_end||div_start_absolute!==rec_start)begin reject_code<=6;state<=FAIL;end
                    else if(record_index<normal_count&&(delayed>=normal_deadline||delayed>128'hffffffffffffffff)) begin reject_code<=6;state<=FAIL;end
                    else if(rq!=chain_q||rv!=chain_v||ra!=chain_a) begin reject_code<=8;state<=FAIL;end
                    else begin
                        if(record_index==0) begin first_q<=rq;first_v<=rv;first_a<=ra;end
                        state<=MSTART;
                    end
                end
                MSTART: if(EXTERNAL_EXECUTION==0||exec_math_grant===1'b1)state<=MWAIT;
                MWAIT: if(math_done) begin
                    if(math_overflow!==1'b0) begin reject_code<=7;state<=FAIL;end
                    else state<=GSTART;
                end
                GSTART:if(EXTERNAL_EXECUTION==0||exec_gate_grant===1'b1)state<=GWAIT;
                GWAIT:if(gate_done)state<=GCHECK;
                GCHECK: begin
                    if(gate_allow!==1'b1) begin reject_code<=7;state<=FAIL;end
                    else if(record_index==total_count-1&&(end_v!='0||end_a!='0)) begin reject_code<=9;state<=FAIL;end
                    else begin
                        chain_q<=end_q;chain_v<=end_v;chain_a<=end_a;chain_wire<=gate_wire;
                        next_start<=rec_end;next_tick<=rec_tick+1;
                        if(record_index==normal_count-1) begin
                            normal_end_cycle<=rec_end;normal_end_tick<=rec_tick;
                            normal_end_q<=end_q;normal_end_v<=end_v;normal_end_a<=end_a;normal_end_wire<=gate_wire;
                        end
                        if(record_index==total_count-1) begin brake_end_cycle<=rec_end;state<=FINISH;end
                        else if(rec_tick==64'hffffffffffffffff) begin reject_code<=6;state<=FAIL;end
                        else begin record_index<=record_index+1;word_index<=0;state<=RREQ;end
                    end
                end
                FINISH: begin
                    if((~payload_crc)!=h[24]) begin reject_code<=3;state<=FAIL;end
                    else if(brake_end_cycle<first_cycle||brake_end_cycle-first_cycle>timeout_reg||{64'd0,now_cycle}+map_error>=normal_deadline) begin reject_code<=6;state<=FAIL;end
                    else begin accepted<=1;busy<=0;done<=1;state<=IDLE;end
                end
                FAIL: begin accepted<=0;busy<=0;done<=1;state<=IDLE;end
                default: state<=IDLE;
            endcase
        end
    end
endmodule

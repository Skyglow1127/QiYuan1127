`timescale 1ns/1ps
// Timestamp-only SFLS engine. Time units are counter ticks. Each retained
// event represents EDGE_STRIDE physical rising edges. Arbitrary M >= 2.
// No real arithmetic, simulation-only delay, or CPU is used here.
module sfls_core #(
    parameter integer WINDOW_LOG2=15,
    parameter integer EDGE_STRIDE=1,
    parameter integer CLOCK_HZ=100000000,
    parameter integer ACC_W=96,
    parameter integer WINDOW_SIZE=(1<<WINDOW_LOG2)
)(
    input wire clk, input wire rst,
    input wire event_valid, input wire [63:0] event_tick,
    output wire event_ready,
    input wire sample_req,
    output reg result_valid,
    output reg [63:0] frequency_uHz,
    output reg [63:0] result_last_tick, result_mean_tick,
    output reg full,
    output reg fault,
    output wire compute_busy,
    output wire signed [ACC_W-1:0] debug_q, debug_sum
);
    localparam integer M=WINDOW_SIZE;
    localparam integer ADDR_W=(M>2)?$clog2(M):1;
    localparam [ACC_W-1:0] M_W=M;
    localparam [ACC_W-1:0] K=M_W*(M_W*M_W-1)/6;
    localparam [ACC_W-1:0] NUMERATOR=K*CLOCK_HZ*EDGE_STRIDE*1000000;
    (* ram_style="block" *) reg [63:0] history[0:M-1];
    reg [ADDR_W-1:0] ptr;
    reg [ADDR_W:0] count;
    reg [1:0] state;
    reg [63:0] incoming, outgoing, newest;
    reg signed [ACC_W-1:0] sum, q;
    reg signed [ACC_W-1:0] fill_product, full_delta;
    wire signed [ACC_W-1:0] in_ext={{(ACC_W-64){1'b0}},incoming};
    wire signed [ACC_W-1:0] old_ext={{(ACC_W-64){1'b0}},outgoing};
    // Constants preserve the exact centered-index recurrence for odd/even M.
    // Non-power-of-two multiplication needs configuration-specific timing validation.
    localparam signed [ACC_W-1:0] M_PLUS=M+1, M_MINUS=M-1;
    assign event_ready=state==0 && !fault;
    assign debug_q=q; assign debug_sum=sum;
    reg div_start;
    reg [ACC_W-1:0] denominator;
    wire div_busy, div_done, div_zero;
    wire [ACC_W-1:0] div_result;
    reg pending;
    reg [63:0] snap_last;
    reg [ACC_W-1:0] mean_numerator;
    wire mean_busy, mean_done, mean_zero;
    wire [ACC_W-1:0] mean_result;
    assign compute_busy=div_busy|div_start|pending;
    udiv_round #(.W(ACC_W)) divider(
        .clk(clk),.rst(rst),.start(div_start),.numerator(NUMERATOR),
        .denominator(denominator),.busy(div_busy),.done(div_done),
        .divide_by_zero(div_zero),.quotient(div_result));
    // Both dividers start from the same atomic snapshot and have W+1-cycle
    // latency. This removes the 96-bit combinational divide-by-M path.
    udiv_round #(.W(ACC_W),.ROUND_NEAREST(0)) mean_divider(
        .clk(clk),.rst(rst),.start(div_start),.numerator(mean_numerator),
        .denominator(M_W),.busy(mean_busy),.done(mean_done),
        .divide_by_zero(mean_zero),.quotient(mean_result));
    always @(posedge clk) begin
        if(rst) begin
            ptr<=0; count<=0; state<=0; incoming<=0; outgoing<=0;
            newest<=0; sum<=0; q<=0; fill_product<=0; full_delta<=0; full<=0; fault<=0;
            div_start<=0; denominator<=0; pending<=0; snap_last<=0; mean_numerator<=0;
            result_valid<=0; frequency_uHz<=0; result_last_tick<=0; result_mean_tick<=0;
        end else begin
            div_start<=0; result_valid<=0;
            if(event_valid && !event_ready) fault<=1;
            case(state)
                0: if(event_valid && event_ready) begin
                    if(count!=0 && event_tick<=newest) fault<=1;
                    incoming<=event_tick;
                    outgoing<=history[ptr];
                    state<=1;
                end
                1: begin
                    // Initial fill maintains centered Q for the current count.
                    fill_product<=in_ext*$signed({1'b0,count});
                    // Capture the constant products before the accumulator
                    // update, separating BRAM output from the Q adder chain.
                    full_delta<=old_ext*M_PLUS+in_ext*M_MINUS;
                    state<=2;
                end
                2: begin
                    history[ptr]<=incoming;
                    ptr<=(ptr==M-1)?0:ptr+1'b1;
                    newest<=incoming;
                    if(!full) begin
                        q<=q-sum+fill_product; sum<=sum+in_ext;
                        count<=count+1'b1;
                        if(count==M-1) full<=1;
                    end else begin
                        q<=q-(sum<<<1)+full_delta;
                        sum<=sum-old_ext+in_ext;
                    end
                    state<=0;
                end
                default: state<=0;
            endcase
            // Snapshot pre-edge statistics atomically, even during an update.
            if(sample_req && full && !fault) begin
                if(compute_busy || q<=0) fault<=1;
                else begin
                    denominator<=q; div_start<=1; pending<=1;
                    snap_last<=newest; mean_numerator<=$unsigned(sum);
                end
            end
            if(div_done) begin
                pending<=0;
                if(div_zero || |div_result[ACC_W-1:64] || !mean_done || mean_zero || |mean_result[ACC_W-1:64]) fault<=1;
                else if(!fault) begin
                    frequency_uHz<=div_result[63:0];
                    result_last_tick<=snap_last; result_mean_tick<=mean_result[63:0];
                    result_valid<=1;
                end
            end
        end
    end
endmodule

`timescale 1ns/1ps
// Conditioned digital comparator input; physical clock/pins are supplied by
// the eventual board design. rst starts an independent acquisition epoch.
module sfls_sensor #(
    parameter integer CLOCK_HZ=200000000,
    parameter integer WINDOW_SIZE=96,
    parameter integer OUTPUT_HZ=1000,
    parameter integer HOP_EVENTS=0, // 0: fixed cadence; >0: event-driven hop
    parameter integer TIMEOUT_CYCLES=CLOCK_HZ/100
)(
    input wire clk, rst, larmor_in,
    output wire [63:0] frequency_uHz, result_last_tick, result_mean_tick,
    output wire result_valid, window_full, fault, input_timeout,
    output wire debug_event_valid, debug_sample_req,
    output wire [63:0] debug_event_tick
);
    localparam integer OUTPUT_CYCLES=CLOCK_HZ/OUTPUT_HZ;
    localparam integer OUT_W=(OUTPUT_CYCLES>1)?$clog2(OUTPUT_CYCLES):1;
    localparam integer COUNT_W=$clog2(WINDOW_SIZE+1);
    localparam integer HOP_W=(HOP_EVENTS>1)?$clog2(HOP_EVENTS):1;
    (* ASYNC_REG="TRUE" *) reg sync1, sync2;
    reg delayed;
    reg [63:0] ticks;
    reg [OUT_W-1:0] output_count;
    reg [$clog2(TIMEOUT_CYCLES+1)-1:0] silence;
    reg timeout_sticky;
    reg [COUNT_W-1:0] fill_count;
    reg [HOP_W-1:0] hop_count;
    reg [2:0] request_pipe;
    wire rising=sync2 && !delayed;
    wire selected=rising && !timeout_sticky;
    wire fixed_request=output_count==OUTPUT_CYCLES-1;
    wire hop_request=selected && ((fill_count==WINDOW_SIZE-1) ||
                                 (fill_count==WINDOW_SIZE && hop_count==HOP_EVENTS-1));
    wire sample_req=(HOP_EVENTS==0)?fixed_request:request_pipe[2];
    wire core_valid, core_fault, ready, busy;
    assign fault=core_fault|timeout_sticky;
    assign input_timeout=timeout_sticky;
    assign result_valid=core_valid && !fault;
    assign debug_event_valid=selected;
    assign debug_event_tick=ticks;
    assign debug_sample_req=sample_req;
    always @(posedge clk) begin
        if(rst) begin
            sync1<=0;sync2<=0;delayed<=0;ticks<=0;output_count<=0;
            silence<=0;timeout_sticky<=0;fill_count<=0;hop_count<=0;request_pipe<=0;
        end else begin
            sync1<=larmor_in;sync2<=sync1;delayed<=sync2;
            ticks<=ticks+1'b1;
            output_count<=fixed_request?0:output_count+1'b1;
            request_pipe<={request_pipe[1:0],hop_request};
            if(rising) silence<=0;
            else if(silence<TIMEOUT_CYCLES) silence<=silence+1'b1;
            else timeout_sticky<=1;
            if(selected) begin
                if(fill_count<WINDOW_SIZE) begin
                    fill_count<=fill_count+1'b1;hop_count<=0;
                end else if(HOP_EVENTS>0) begin
                    hop_count<=(hop_count==HOP_EVENTS-1)?0:hop_count+1'b1;
                end
            end
        end
    end
    sfls_core #(.CLOCK_HZ(CLOCK_HZ),.WINDOW_SIZE(WINDOW_SIZE),.EDGE_STRIDE(1)) core(
        .clk(clk),.rst(rst),.event_valid(selected),.event_tick(ticks),.event_ready(ready),
        .sample_req(sample_req),.result_valid(core_valid),.frequency_uHz(frequency_uHz),
        .result_last_tick(result_last_tick),.result_mean_tick(result_mean_tick),
        .full(window_full),.fault(core_fault),.compute_busy(busy),.debug_q(),.debug_sum());
endmodule

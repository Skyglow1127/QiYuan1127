`timescale 1ns/1ps
// Digital, already-conditioned Larmor input. clk is a calibrated timebase.
module lamor_frequency_meter #(
    parameter integer CLOCK_HZ=100000000,
    parameter integer OUTPUT_HZ=1000,
    parameter integer WINDOW_LOG2=15,
    parameter integer EDGE_STRIDE=1,
    parameter integer TIMEOUT_CYCLES=CLOCK_HZ/100 // 10 ms without physical edge
)(
    input wire clk, input wire rst, input wire larmor_in,
    output wire [63:0] frequency_uHz,
    output wire [63:0] result_last_tick, result_mean_tick,
    output wire result_valid, output wire window_full,
    output wire fault,
    output wire input_timeout,
    output wire debug_event_valid,
    output wire [63:0] debug_event_tick,
    output wire debug_sample_req
);
    localparam integer OUTPUT_CYCLES=CLOCK_HZ/OUTPUT_HZ;
    (* ASYNC_REG="TRUE" *) reg sync1,sync2;
    reg delayed;
    wire rising=sync2 && !delayed;
    reg [63:0] ticks;
    reg [$clog2(OUTPUT_CYCLES)-1:0] output_count;
    reg [$clog2(TIMEOUT_CYCLES+1)-1:0] silence;
    localparam integer STRIDE_W=(EDGE_STRIDE>1)?$clog2(EDGE_STRIDE):1;
    reg [STRIDE_W-1:0] edge_count;
    reg seen_input, timeout_sticky;
    wire selected=rising && edge_count==0;
    wire ready,core_fault,busy,core_valid;
    wire sample_req=output_count==OUTPUT_CYCLES-1;
    assign debug_event_valid=selected;
    assign debug_event_tick=ticks;
    assign debug_sample_req=sample_req;
    assign input_timeout=timeout_sticky;
    assign fault=core_fault|timeout_sticky;
    assign result_valid=core_valid && !fault;
    always @(posedge clk) begin
        if(rst) begin
            sync1<=0; sync2<=0; delayed<=0; ticks<=0; output_count<=0;
            edge_count<=0; silence<=0; seen_input<=0; timeout_sticky<=0;
        end else begin
            sync1<=larmor_in; sync2<=sync1; delayed<=sync2;
            ticks<=ticks+1'b1;
            if(sample_req) output_count<=0; else output_count<=output_count+1'b1;
            if(rising) begin
                seen_input<=1; silence<=0;
                if(edge_count==EDGE_STRIDE-1) edge_count<=0;
                else edge_count<=edge_count+1'b1;
            end else if(silence<TIMEOUT_CYCLES) silence<=silence+1'b1;
            else timeout_sticky<=1;
        end
    end
    sfls_core #(.WINDOW_LOG2(WINDOW_LOG2),.EDGE_STRIDE(EDGE_STRIDE),
                .CLOCK_HZ(CLOCK_HZ)) core(
        .clk(clk),.rst(rst),.event_valid(selected && !timeout_sticky),
        .event_tick(ticks),.event_ready(ready),.sample_req(sample_req),
        .result_valid(core_valid),.frequency_uHz(frequency_uHz),
        .result_last_tick(result_last_tick),.result_mean_tick(result_mean_tick),
        .full(window_full),.fault(core_fault),.compute_busy(busy),
        .debug_q(),.debug_sum());
endmodule

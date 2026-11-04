`timescale 1ns/1ps
// V3 wraps the proven V2/HLS datapath. Counters remain pixels, not packed beats.
// The V2 core may finish before the last packed beat leaves; keep the outer
// transaction active until BOTH the input/core and packed output have finished.
module vpr3_stream (
    input wire aclk, aresetn, cfg_valid, abort_frame, output wire cfg_ready,
    input wire [15:0] cfg_in_width,cfg_in_height,cfg_out_width,cfg_out_height,cfg_resize_width,cfg_resize_height,
    input wire [7:0] cfg_pad,input wire cfg_bypass,
    input wire [31:0] s_axis_tdata,input wire [3:0] s_axis_tkeep,input wire s_axis_tlast,s_axis_tvalid,output wire s_axis_tready,
    output wire [31:0] m_axis_tdata,output wire [3:0] m_axis_tkeep,output wire m_axis_tlast,m_axis_tvalid,input wire m_axis_tready,
    output reg busy,done,output wire [3:0] error_flags,output wire [31:0] input_count,output_count
);
    reg bypass_q, core_done_q, output_done_q;
    wire core_cfg_ready, core_busy, core_done;
    wire [31:0] pixel_data, packed_data;
    wire [3:0] pixel_keep, packed_keep;
    wire pixel_valid,pixel_last,pixel_ready,core_input_ready;
    wire packed_valid,packed_last,packer_input_ready,packer_done;
    assign cfg_ready = aresetn && !busy && core_cfg_ready && !abort_frame;
    wire accept_cfg = cfg_valid && cfg_ready;
    wire enabled = busy && !abort_frame && aresetn;
    assign s_axis_tready = enabled && core_input_ready;
    assign pixel_ready = enabled && (bypass_q ? m_axis_tready : packer_input_ready);
    assign m_axis_tdata = bypass_q ? pixel_data : packed_data;
    assign m_axis_tkeep = bypass_q ? pixel_keep : packed_keep;
    assign m_axis_tlast = bypass_q ? pixel_last : packed_last;
    assign m_axis_tvalid = enabled && (bypass_q ? pixel_valid : packed_valid);
    wire output_last_accepted = m_axis_tvalid && m_axis_tready && m_axis_tlast;
    always @(posedge aclk) begin
        if (!aresetn) begin
            busy <= 0; done <= 0; bypass_q <= 0; core_done_q <= 0; output_done_q <= 0;
        end else begin
            done <= 0;
            if (abort_frame) begin
                busy <= 0; done <= 1; core_done_q <= 0; output_done_q <= 0;
            end else if (accept_cfg) begin
                busy <= 1; bypass_q <= cfg_bypass; core_done_q <= 0; output_done_q <= 0;
            end else if (busy) begin
                if (core_done) core_done_q <= 1;
                if (output_last_accepted) output_done_q <= 1;
                // Invalid geometry produces no output packet. The V2 error
                // and done event are authoritative for that rejected command.
                if ((core_done_q || core_done) &&
                    (output_done_q || output_last_accepted || error_flags[0])) begin
                    busy <= 0; done <= 1;
                end
            end
        end
    end
    vpr2_stream pixel_core (
        .aclk(aclk),.aresetn(aresetn),.cfg_valid(accept_cfg),.cfg_ready(core_cfg_ready),.abort_frame(abort_frame),
        .cfg_in_width(cfg_in_width),.cfg_in_height(cfg_in_height),.cfg_out_width(cfg_out_width),.cfg_out_height(cfg_out_height),
        .cfg_resize_width(cfg_resize_width),.cfg_resize_height(cfg_resize_height),.cfg_pad(cfg_pad),.cfg_bypass(cfg_bypass),
        .s_axis_tdata(s_axis_tdata),.s_axis_tkeep(s_axis_tkeep),.s_axis_tlast(s_axis_tlast),
        .s_axis_tvalid(s_axis_tvalid && enabled),.s_axis_tready(core_input_ready),
        .m_axis_tdata(pixel_data),.m_axis_tkeep(pixel_keep),.m_axis_tlast(pixel_last),.m_axis_tvalid(pixel_valid),.m_axis_tready(pixel_ready),
        .busy(core_busy),.done(core_done),.error_flags(error_flags),.input_count(input_count),.output_count(output_count)
    );
    rgb888_axis_packer output_packer (
        .aclk(aclk),.aresetn(aresetn),.clear(abort_frame || accept_cfg),
        .s_axis_tdata(pixel_data),.s_axis_tvalid(pixel_valid && enabled && !bypass_q),.s_axis_tlast(pixel_last),.s_axis_tready(packer_input_ready),
        .m_axis_tdata(packed_data),.m_axis_tkeep(packed_keep),.m_axis_tvalid(packed_valid),.m_axis_tlast(packed_last),
        .m_axis_tready(m_axis_tready && enabled && !bypass_q),.frame_done(packer_done)
    );
endmodule

`timescale 1ns/1ps
// Compact one RGBX pixel per input beat into little-endian RGB888 byte packets.
// Seven bytes of storage allow accepting a pixel while a full beat is stalled.
// A packet boundary is retained until its last output beat has been accepted.
module rgb888_axis_packer (
    input wire aclk, aresetn, clear,
    input wire [31:0] s_axis_tdata,
    input wire s_axis_tvalid, s_axis_tlast,
    output wire s_axis_tready,
    output wire [31:0] m_axis_tdata,
    output wire [3:0] m_axis_tkeep,
    output wire m_axis_tvalid, m_axis_tlast,
    input wire m_axis_tready,
    output reg frame_done
);
    reg [55:0] bytes_q, bytes_next;
    reg [3:0] count_q, count_next;
    reg ending_q, ending_next;
    wire enabled = aresetn && !clear;
    assign m_axis_tvalid = enabled && ((count_q >= 4) || (ending_q && count_q != 0));
    assign m_axis_tdata = bytes_q[31:0];
    assign m_axis_tlast = ending_q && count_q <= 4;
    assign m_axis_tkeep = count_q >= 4 ? 4'hf : (4'hf >> (4-count_q));
    wire pop = m_axis_tvalid && m_axis_tready;
    assign s_axis_tready = enabled && !ending_q && ((count_q <= 4) || pop);
    wire push = s_axis_tvalid && s_axis_tready;
    always @* begin
        bytes_next = bytes_q;
        count_next = count_q;
        ending_next = ending_q;
        if (pop) begin
            bytes_next = bytes_q >> 32;
            count_next = count_q >= 4 ? count_q-4 : 0;
            if (m_axis_tlast) ending_next = 0;
        end
        if (push) begin
            bytes_next = bytes_next | ({32'b0,s_axis_tdata[23:0]} << (count_next*8));
            count_next = count_next + 3;
            if (s_axis_tlast) ending_next = 1;
        end
    end
    always @(posedge aclk) begin
        if (!aresetn || clear) begin
            bytes_q <= 0;
            count_q <= 0;
            ending_q <= 0;
            frame_done <= 0;
        end else begin
            bytes_q <= bytes_next;
            count_q <= count_next;
            ending_q <= ending_next;
            frame_done <= pop && m_axis_tlast;
        end
    end
endmodule

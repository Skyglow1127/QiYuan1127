`timescale 1ns/1ps
// AXI4-Lite single outstanding transaction. AW and W may arrive independently.
module vpr3_axi #(
    parameter integer MAX_WIDTH=1920, MAX_HEIGHT=1080
)(
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI:S_AXIS:M_AXIS, ASSOCIATED_RESET aresetn" *)
    input wire aclk, input wire aresetn,
    input wire [7:0] s_axi_awaddr, input wire s_axi_awvalid, output wire s_axi_awready,
    input wire [31:0] s_axi_wdata, input wire [3:0] s_axi_wstrb,
    input wire s_axi_wvalid, output wire s_axi_wready,
    output reg [1:0] s_axi_bresp, output reg s_axi_bvalid, input wire s_axi_bready,
    input wire [7:0] s_axi_araddr, input wire s_axi_arvalid, output wire s_axi_arready,
    output reg [31:0] s_axi_rdata, output reg [1:0] s_axi_rresp,
    output reg s_axi_rvalid, input wire s_axi_rready,
    input wire [31:0] s_axis_tdata, input wire [3:0] s_axis_tkeep,
    input wire s_axis_tvalid, output wire s_axis_tready, input wire s_axis_tlast,
    output wire [31:0] m_axis_tdata, output wire [3:0] m_axis_tkeep,
    output wire m_axis_tvalid, input wire m_axis_tready, output wire m_axis_tlast
);
    localparam [31:0] MAX_SIZE=(MAX_HEIGHT<<16)|MAX_WIDTH;
    reg aw_pending, w_pending;
    reg [7:0] aw_addr;
    reg [31:0] w_data, in_size, out_size, mode, content_size, pad_value;
    reg [3:0] w_strb;
    reg start, abort_frame, done_sticky;
    wire cfg_ready, busy, done;
    wire [3:0] errors;
    wire [31:0] inputs, outputs;
    assign s_axi_awready=aresetn && !aw_pending && !s_axi_bvalid;
    assign s_axi_wready=aresetn && !w_pending && !s_axi_bvalid;
    assign s_axi_arready=aresetn && !s_axi_rvalid;
    function automatic [31:0] merge_bytes(input [31:0] old_word, new_word, input [3:0] strobe);
        integer k;
        begin
            merge_bytes=old_word;
            for(k=0;k<4;k=k+1) if(strobe[k]) merge_bytes[k*8+:8]=new_word[k*8+:8];
        end
    endfunction
    always @(posedge aclk) begin
        if (!aresetn) begin
            aw_pending<=0; w_pending<=0; aw_addr<=0; w_data<=0; w_strb<=0;
            in_size<=32'h01e00280; out_size<=32'h02800280; mode<=0; content_size<=32'h01e00280; pad_value<=114;
            start<=0; abort_frame<=0; done_sticky<=0;
            s_axi_bvalid<=0; s_axi_bresp<=0; s_axi_rvalid<=0; s_axi_rresp<=0; s_axi_rdata<=0;
        end else begin
            start<=0; abort_frame<=0;
            if(done) done_sticky<=1;
            if(s_axi_bvalid && s_axi_bready) s_axi_bvalid<=0;
            if(s_axi_rvalid && s_axi_rready) s_axi_rvalid<=0;
            if(s_axi_awvalid && s_axi_awready) begin aw_pending<=1; aw_addr<=s_axi_awaddr; end
            if(s_axi_wvalid && s_axi_wready) begin w_pending<=1; w_data<=s_axi_wdata; w_strb<=s_axi_wstrb; end
            if(aw_pending && w_pending && !s_axi_bvalid) begin
                aw_pending<=0; w_pending<=0; s_axi_bvalid<=1; s_axi_bresp<=0;
                if(aw_addr[1:0]!=0) s_axi_bresp<=2'b10;
                else case(aw_addr)
                    8'h00: if(w_strb[0]) begin
                        if(w_data[1]) abort_frame<=1;
                        else if(w_data[0]) begin
                            if(cfg_ready && !start) begin start<=1; done_sticky<=0; end
                            else s_axi_bresp<=2'b10;
                        end
                        if(w_data[2]) done_sticky<=0;
                    end
                    8'h08: in_size<=merge_bytes(in_size,w_data,w_strb);
                    8'h0c: out_size<=merge_bytes(out_size,w_data,w_strb);
                    8'h10: mode<=merge_bytes(mode,w_data,w_strb);
                    8'h28: content_size<=merge_bytes(content_size,w_data,w_strb);
                    8'h2c: pad_value<=merge_bytes(pad_value,w_data,w_strb);
                    default: s_axi_bresp<=2'b10;
                endcase
            end
            if(s_axi_arvalid && s_axi_arready) begin
                s_axi_rvalid<=1; s_axi_rresp<=0;
                if(s_axi_araddr[1:0]!=0) begin s_axi_rdata<=0; s_axi_rresp<=2'b10; end
                else case(s_axi_araddr)
                    8'h00: s_axi_rdata<=0;
                    8'h04: s_axi_rdata<={16'b0,4'b0,errors,6'b0,done_sticky,busy};
                    8'h08: s_axi_rdata<=in_size;
                    8'h0c: s_axi_rdata<=out_size;
                    8'h10: s_axi_rdata<=mode;
                    8'h14: s_axi_rdata<={28'b0,errors};
                    8'h18: s_axi_rdata<=inputs;
                    8'h1c: s_axi_rdata<=outputs;
                    8'h20: s_axi_rdata<=MAX_SIZE;
                    8'h24: s_axi_rdata<=32'h56505233;
                    8'h28: s_axi_rdata<=content_size;
                    8'h2c: s_axi_rdata<=pad_value;
                    8'h30: s_axi_rdata<=32'h02800280;
                    8'h34: s_axi_rdata<=32'h00000001; // bit 0: compact RGB888 normal output
                    default: begin s_axi_rdata<=0; s_axi_rresp<=2'b10; end
                endcase
            end
        end
    end
    vpr3_stream core (
        .aclk(aclk),.aresetn(aresetn),.cfg_valid(start),.cfg_ready(cfg_ready),
        .cfg_in_width(in_size[15:0]),.cfg_in_height(in_size[31:16]),
        .cfg_out_width(out_size[15:0]),.cfg_out_height(out_size[31:16]),
        .cfg_resize_width(content_size[15:0]),.cfg_resize_height(content_size[31:16]),
        .cfg_pad(pad_value[7:0]),.cfg_bypass(mode[0]),.abort_frame(abort_frame),
        .s_axis_tdata(s_axis_tdata),.s_axis_tkeep(s_axis_tkeep),.s_axis_tvalid(s_axis_tvalid),
        .s_axis_tready(s_axis_tready),.s_axis_tlast(s_axis_tlast),
        .m_axis_tdata(m_axis_tdata),.m_axis_tkeep(m_axis_tkeep),.m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready),.m_axis_tlast(m_axis_tlast),
        .busy(busy),.done(done),.error_flags(errors),.input_count(inputs),.output_count(outputs));
endmodule

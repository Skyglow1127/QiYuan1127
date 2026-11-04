`timescale 1ns/1ps
// From-scratch bank-ownership firewall. No TX/robot device interface.
// One outstanding write and one read. Legal accepted bursts drain even after
// bank_writable closes; COMMIT waits for write_busy then two RAM settle clocks.
module robot_rt_axi_guard #(
    parameter logic [31:0] BASE_ADDR = 32'hA0020000
)(
    (* X_INTERFACE_INFO="xilinx.com:signal:clock:1.0 aclk CLK", X_INTERFACE_PARAMETER="ASSOCIATED_BUSIF s_axi_chunk:m_axi_bram, ASSOCIATED_RESET aresetn" *) input wire aclk,
    (* X_INTERFACE_INFO="xilinx.com:signal:reset:1.0 aresetn RST", X_INTERFACE_PARAMETER="POLARITY ACTIVE_LOW" *) input wire aresetn,
    input wire [1:0] bank_writable,
    output wire [1:0] write_busy,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWID", X_INTERFACE_PARAMETER="PROTOCOL AXI4, DATA_WIDTH 32, ADDR_WIDTH 32, ID_WIDTH 1, HAS_BURST 1, HAS_LOCK 1, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1" *) input wire s_axi_chunk_awid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWADDR" *) input wire [31:0] s_axi_chunk_awaddr,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWLEN" *) input wire [7:0] s_axi_chunk_awlen,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWSIZE" *) input wire [2:0] s_axi_chunk_awsize,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWBURST" *) input wire [1:0] s_axi_chunk_awburst,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWLOCK" *) input wire s_axi_chunk_awlock,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWCACHE" *) input wire [3:0] s_axi_chunk_awcache,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWPROT" *) input wire [2:0] s_axi_chunk_awprot,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWQOS" *) input wire [3:0] s_axi_chunk_awqos,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWVALID" *) input wire s_axi_chunk_awvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk AWREADY" *) output wire s_axi_chunk_awready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk WDATA" *) input wire [31:0] s_axi_chunk_wdata,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk WSTRB" *) input wire [3:0] s_axi_chunk_wstrb,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk WLAST" *) input wire s_axi_chunk_wlast,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk WVALID" *) input wire s_axi_chunk_wvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk WREADY" *) output wire s_axi_chunk_wready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk BID" *) output wire s_axi_chunk_bid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk BRESP" *) output wire [1:0] s_axi_chunk_bresp,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk BVALID" *) output wire s_axi_chunk_bvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk BREADY" *) input wire s_axi_chunk_bready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARID" *) input wire s_axi_chunk_arid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARADDR" *) input wire [31:0] s_axi_chunk_araddr,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARLEN" *) input wire [7:0] s_axi_chunk_arlen,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARSIZE" *) input wire [2:0] s_axi_chunk_arsize,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARBURST" *) input wire [1:0] s_axi_chunk_arburst,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARLOCK" *) input wire s_axi_chunk_arlock,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARCACHE" *) input wire [3:0] s_axi_chunk_arcache,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARPROT" *) input wire [2:0] s_axi_chunk_arprot,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARQOS" *) input wire [3:0] s_axi_chunk_arqos,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARVALID" *) input wire s_axi_chunk_arvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk ARREADY" *) output wire s_axi_chunk_arready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk RID" *) output wire s_axi_chunk_rid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk RDATA" *) output wire [31:0] s_axi_chunk_rdata,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk RRESP" *) output wire [1:0] s_axi_chunk_rresp,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk RLAST" *) output wire s_axi_chunk_rlast,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk RVALID" *) output wire s_axi_chunk_rvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 s_axi_chunk RREADY" *) input wire s_axi_chunk_rready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWID", X_INTERFACE_PARAMETER="PROTOCOL AXI4, DATA_WIDTH 32, ADDR_WIDTH 32, ID_WIDTH 1, HAS_BURST 1, HAS_LOCK 1, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1" *) output wire m_axi_bram_awid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWADDR" *) output wire [31:0] m_axi_bram_awaddr,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWLEN" *) output wire [7:0] m_axi_bram_awlen,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWSIZE" *) output wire [2:0] m_axi_bram_awsize,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWBURST" *) output wire [1:0] m_axi_bram_awburst,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWLOCK" *) output wire m_axi_bram_awlock,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWCACHE" *) output wire [3:0] m_axi_bram_awcache,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWPROT" *) output wire [2:0] m_axi_bram_awprot,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWQOS" *) output wire [3:0] m_axi_bram_awqos,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWVALID" *) output wire m_axi_bram_awvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram AWREADY" *) input wire m_axi_bram_awready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram WDATA" *) output wire [31:0] m_axi_bram_wdata,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram WSTRB" *) output wire [3:0] m_axi_bram_wstrb,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram WLAST" *) output wire m_axi_bram_wlast,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram WVALID" *) output wire m_axi_bram_wvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram WREADY" *) input wire m_axi_bram_wready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram BID" *) input wire m_axi_bram_bid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram BRESP" *) input wire [1:0] m_axi_bram_bresp,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram BVALID" *) input wire m_axi_bram_bvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram BREADY" *) output wire m_axi_bram_bready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARID" *) output wire m_axi_bram_arid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARADDR" *) output wire [31:0] m_axi_bram_araddr,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARLEN" *) output wire [7:0] m_axi_bram_arlen,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARSIZE" *) output wire [2:0] m_axi_bram_arsize,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARBURST" *) output wire [1:0] m_axi_bram_arburst,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARLOCK" *) output wire m_axi_bram_arlock,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARCACHE" *) output wire [3:0] m_axi_bram_arcache,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARPROT" *) output wire [2:0] m_axi_bram_arprot,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARQOS" *) output wire [3:0] m_axi_bram_arqos,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARVALID" *) output wire m_axi_bram_arvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram ARREADY" *) input wire m_axi_bram_arready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram RID" *) input wire m_axi_bram_rid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram RDATA" *) input wire [31:0] m_axi_bram_rdata,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram RRESP" *) input wire [1:0] m_axi_bram_rresp,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram RLAST" *) input wire m_axi_bram_rlast,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram RVALID" *) input wire m_axi_bram_rvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 m_axi_bram RREADY" *) output wire m_axi_bram_rready
);
    localparam [2:0] WI=0, WA=1, WD=2, WB=3, WL=4, WE=5;
    logic [2:0] ws;
    logic [31:0] wa;
    logic [7:0] wlen, wi;
    logic wid, wbank, winrange, badlast;
    logic [2:0] wsize, wprot;
    logic [1:0] wburst;
    logic wlock;
    logic [3:0] wcache,wqos;
    logic [32:0] woff,woff_end,roff,roff_end;
    logic wok,rok;
    always_comb begin
        woff={1'b0,s_axi_chunk_awaddr}-{1'b0,BASE_ADDR};
        woff_end=woff+(({25'b0,s_axi_chunk_awlen}+33'd1)<<2)-1;
        roff={1'b0,s_axi_chunk_araddr}-{1'b0,BASE_ADDR};
        roff_end=roff+(({25'b0,s_axi_chunk_arlen}+33'd1)<<2)-1;
        wok=(s_axi_chunk_awaddr>=BASE_ADDR && woff_end<33'd131072 &&
            !s_axi_chunk_awaddr[1:0] && s_axi_chunk_awsize==3'd2 &&
            s_axi_chunk_awburst==2'b01 && !s_axi_chunk_awlock &&
            s_axi_chunk_awlen<=8'd63 && woff[16:12]==woff_end[16:12] &&
            bank_writable[woff[16]]);
        rok=(s_axi_chunk_araddr>=BASE_ADDR && roff_end<33'd131072 &&
            !s_axi_chunk_araddr[1:0] && s_axi_chunk_arsize==3'd2 &&
            s_axi_chunk_arburst==2'b01 && !s_axi_chunk_arlock &&
            s_axi_chunk_arlen<=8'd63 && roff[16:12]==roff_end[16:12]);
    end
    assign write_busy=(ws==WI || !winrange)?2'b00:(wbank?2'b10:2'b01);
    assign s_axi_chunk_awready=(ws==WI);
    assign m_axi_bram_awvalid=(ws==WA);
    assign m_axi_bram_awid=wid;
    assign m_axi_bram_awaddr=wa;
    assign m_axi_bram_awlen=wlen;
    assign m_axi_bram_awsize=wsize;
    assign m_axi_bram_awburst=wburst;
    assign m_axi_bram_awlock=wlock;
    assign m_axi_bram_awcache=wcache;
    assign m_axi_bram_awprot=wprot;
    assign m_axi_bram_awqos=wqos;
    assign m_axi_bram_wvalid=(ws==WD) && s_axi_chunk_wvalid;
    assign m_axi_bram_wdata=s_axi_chunk_wdata;
    assign m_axi_bram_wstrb=s_axi_chunk_wstrb;
    assign m_axi_bram_wlast=(wi==wlen); // Never propagate malformed early WLAST.
    assign s_axi_chunk_wready=(ws==WL) || ((ws==WD) && m_axi_bram_wready);
    assign s_axi_chunk_bvalid=(ws==WE) || ((ws==WB) && m_axi_bram_bvalid);
    assign s_axi_chunk_bid=(ws==WE)?wid:m_axi_bram_bid;
    assign s_axi_chunk_bresp=(ws==WE || badlast)?2'b10:m_axi_bram_bresp;
    assign m_axi_bram_bready=(ws==WB) && s_axi_chunk_bready;
    always_ff @(posedge aclk) begin
        if(!aresetn) begin
            ws<=WI; wa<=0;wlen<=0;wi<=0;wid<=0;wbank<=0;winrange<=0;badlast<=0;
            wsize<=0;wburst<=0;wlock<=0;wcache<=0;wprot<=0;wqos<=0;
        end else case(ws)
            WI: if(s_axi_chunk_awvalid) begin
                wa<=s_axi_chunk_awaddr;wlen<=s_axi_chunk_awlen;wi<=0;wid<=s_axi_chunk_awid;
                wsize<=s_axi_chunk_awsize;wburst<=s_axi_chunk_awburst;wlock<=s_axi_chunk_awlock;
                wcache<=s_axi_chunk_awcache;wprot<=s_axi_chunk_awprot;wqos<=s_axi_chunk_awqos;
                wbank<=woff[16];winrange<=s_axi_chunk_awaddr>=BASE_ADDR && woff<131072;
                badlast<=0; ws<=wok?WA:WL;
            end
            WA: if(m_axi_bram_awready) ws<=WD;
            WD: if(s_axi_chunk_wvalid && s_axi_chunk_wready) begin
                badlast<=badlast || (s_axi_chunk_wlast!=(wi==wlen));
                if(wi==wlen) ws<=WB; else wi<=wi+1'b1;
            end
            WB: if(m_axi_bram_bvalid && s_axi_chunk_bready) ws<=WI;
            WL: if(s_axi_chunk_wvalid) begin
                if(wi==wlen) ws<=WE; else wi<=wi+1'b1;
            end
            WE: if(s_axi_chunk_bready) ws<=WI;
            default: ws<=WI;
        endcase
    end
    localparam [1:0] RI=0,RA=1,RR=2,RE=3;
    logic [1:0] rs;
    logic [31:0] ra;
    logic [7:0] rlen,ri;
    logic rid,rlock;
    logic [2:0] rsize,rprot;
    logic [1:0] rburst;
    logic [3:0] rcache,rqos;
    assign s_axi_chunk_arready=(rs==RI);
    assign m_axi_bram_arvalid=(rs==RA);
    assign m_axi_bram_arid=rid;
    assign m_axi_bram_araddr=ra;
    assign m_axi_bram_arlen=rlen;
    assign m_axi_bram_arsize=rsize;
    assign m_axi_bram_arburst=rburst;
    assign m_axi_bram_arlock=rlock;
    assign m_axi_bram_arcache=rcache;
    assign m_axi_bram_arprot=rprot;
    assign m_axi_bram_arqos=rqos;
    assign s_axi_chunk_rvalid=(rs==RE) || ((rs==RR) && m_axi_bram_rvalid);
    assign s_axi_chunk_rid=(rs==RE)?rid:m_axi_bram_rid;
    assign s_axi_chunk_rdata=(rs==RE)?32'd0:m_axi_bram_rdata;
    assign s_axi_chunk_rresp=(rs==RE)?2'b10:m_axi_bram_rresp;
    assign s_axi_chunk_rlast=(rs==RE)?(ri==rlen):m_axi_bram_rlast;
    assign m_axi_bram_rready=(rs==RR) && s_axi_chunk_rready;
    always_ff @(posedge aclk) begin
        if(!aresetn) begin rs<=RI;ra<=0;rlen<=0;ri<=0;rid<=0;rlock<=0;rsize<=0;
            rprot<=0;rburst<=0;rcache<=0;rqos<=0;end
        else case(rs)
            RI: if(s_axi_chunk_arvalid) begin
                ra<=s_axi_chunk_araddr;rlen<=s_axi_chunk_arlen;ri<=0;rid<=s_axi_chunk_arid;
                rlock<=s_axi_chunk_arlock;rsize<=s_axi_chunk_arsize;rprot<=s_axi_chunk_arprot;
                rburst<=s_axi_chunk_arburst;rcache<=s_axi_chunk_arcache;rqos<=s_axi_chunk_arqos;
                rs<=rok?RA:RE;
            end
            RA: if(m_axi_bram_arready) rs<=RR;
            RR: if(m_axi_bram_rvalid && s_axi_chunk_rready && m_axi_bram_rlast) rs<=RI;
            RE: if(s_axi_chunk_rready) begin if(ri==rlen) rs<=RI; else ri<=ri+1'b1;end
            default: rs<=RI;
        endcase
    end
endmodule

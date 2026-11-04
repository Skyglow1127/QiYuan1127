`timescale 1ns/1ps
module tb_axi_guard;
  logic aclk=0;always #5 aclk=~aclk;
  logic aresetn=0;logic [1:0] bank_writable=0;wire[1:0]write_busy;
  logic s_axi_chunk_awid=0;
  logic [31:0] s_axi_chunk_awaddr=0;
  logic [7:0] s_axi_chunk_awlen=0;
  logic [2:0] s_axi_chunk_awsize=0;
  logic [1:0] s_axi_chunk_awburst=0;
  logic s_axi_chunk_awlock=0;
  logic [3:0] s_axi_chunk_awcache=0;
  logic [2:0] s_axi_chunk_awprot=0;
  logic [3:0] s_axi_chunk_awqos=0;
  logic s_axi_chunk_awvalid=0;
  logic s_axi_chunk_awready=0;
  logic [31:0] s_axi_chunk_wdata=0;
  logic [3:0] s_axi_chunk_wstrb=0;
  logic s_axi_chunk_wlast=0;
  logic s_axi_chunk_wvalid=0;
  logic s_axi_chunk_wready=0;
  logic s_axi_chunk_bid=0;
  logic [1:0] s_axi_chunk_bresp=0;
  logic s_axi_chunk_bvalid=0;
  logic s_axi_chunk_bready=0;
  logic s_axi_chunk_arid=0;
  logic [31:0] s_axi_chunk_araddr=0;
  logic [7:0] s_axi_chunk_arlen=0;
  logic [2:0] s_axi_chunk_arsize=0;
  logic [1:0] s_axi_chunk_arburst=0;
  logic s_axi_chunk_arlock=0;
  logic [3:0] s_axi_chunk_arcache=0;
  logic [2:0] s_axi_chunk_arprot=0;
  logic [3:0] s_axi_chunk_arqos=0;
  logic s_axi_chunk_arvalid=0;
  logic s_axi_chunk_arready=0;
  logic s_axi_chunk_rid=0;
  logic [31:0] s_axi_chunk_rdata=0;
  logic [1:0] s_axi_chunk_rresp=0;
  logic s_axi_chunk_rlast=0;
  logic s_axi_chunk_rvalid=0;
  logic s_axi_chunk_rready=0;
  logic m_axi_bram_awid=0;
  logic [31:0] m_axi_bram_awaddr=0;
  logic [7:0] m_axi_bram_awlen=0;
  logic [2:0] m_axi_bram_awsize=0;
  logic [1:0] m_axi_bram_awburst=0;
  logic m_axi_bram_awlock=0;
  logic [3:0] m_axi_bram_awcache=0;
  logic [2:0] m_axi_bram_awprot=0;
  logic [3:0] m_axi_bram_awqos=0;
  logic m_axi_bram_awvalid=0;
  logic m_axi_bram_awready=0;
  logic [31:0] m_axi_bram_wdata=0;
  logic [3:0] m_axi_bram_wstrb=0;
  logic m_axi_bram_wlast=0;
  logic m_axi_bram_wvalid=0;
  logic m_axi_bram_wready=0;
  logic m_axi_bram_bid=0;
  logic [1:0] m_axi_bram_bresp=0;
  logic m_axi_bram_bvalid=0;
  logic m_axi_bram_bready=0;
  logic m_axi_bram_arid=0;
  logic [31:0] m_axi_bram_araddr=0;
  logic [7:0] m_axi_bram_arlen=0;
  logic [2:0] m_axi_bram_arsize=0;
  logic [1:0] m_axi_bram_arburst=0;
  logic m_axi_bram_arlock=0;
  logic [3:0] m_axi_bram_arcache=0;
  logic [2:0] m_axi_bram_arprot=0;
  logic [3:0] m_axi_bram_arqos=0;
  logic m_axi_bram_arvalid=0;
  logic m_axi_bram_arready=0;
  logic m_axi_bram_rid=0;
  logic [31:0] m_axi_bram_rdata=0;
  logic [1:0] m_axi_bram_rresp=0;
  logic m_axi_bram_rlast=0;
  logic m_axi_bram_rvalid=0;
  logic m_axi_bram_rready=0;
  robot_rt_axi_guard dut(.*);
  integer aw_seen=0,w_seen=0,ar_seen=0;
  logic rd_active=0;logic[7:0]rd_index=0,rd_len=0;logic rd_id=0;
  always @(posedge aclk) begin
    if(!aresetn) begin m_axi_bram_bvalid<=0;rd_active<=0;end
    else begin
      if(m_axi_bram_awvalid&&m_axi_bram_awready)begin aw_seen<=aw_seen+1;m_axi_bram_bid<=m_axi_bram_awid;end
      if(m_axi_bram_wvalid&&m_axi_bram_wready)begin
        w_seen<=w_seen+1;
        if(m_axi_bram_wlast)m_axi_bram_bvalid<=1;
      end
      if(m_axi_bram_bvalid&&m_axi_bram_bready)m_axi_bram_bvalid<=0;
      if(m_axi_bram_arvalid&&m_axi_bram_arready)begin
        ar_seen<=ar_seen+1;rd_active<=1;rd_index<=0;rd_len<=m_axi_bram_arlen;rd_id<=m_axi_bram_arid;
      end
      if(rd_active&&m_axi_bram_rready)begin
        if(rd_index==rd_len)rd_active<=0;else rd_index<=rd_index+1;
      end
    end
  end
  always_comb begin
    m_axi_bram_rvalid=rd_active;m_axi_bram_rid=rd_id;
    m_axi_bram_rdata=32'h42000000+rd_index;m_axi_bram_rlast=rd_index==rd_len;m_axi_bram_rresp=0;
  end
  task automatic aw(input logic[31:0]addr,input integer n);
    @(negedge aclk);s_axi_chunk_awaddr=addr;s_axi_chunk_awlen=n-1;s_axi_chunk_awvalid=1;
    do @(posedge aclk);while(!s_axi_chunk_awready);
    @(negedge aclk);s_axi_chunk_awvalid=0;
  endtask
  task automatic wd(input integer n,input bit malformed=0);
    for(integer k=0;k<n;k=k+1)begin
      @(negedge aclk);s_axi_chunk_wvalid=1;s_axi_chunk_wdata=32'hab000000+k;
      s_axi_chunk_wlast=(k==n-1)||(malformed&&k==0);
      do @(posedge aclk);while(!s_axi_chunk_wready);
      @(negedge aclk);s_axi_chunk_wvalid=0;
    end
  endtask
  task automatic response(input logic[1:0]expected);
    @(negedge aclk);s_axi_chunk_bready=1;
    do @(posedge aclk);while(!s_axi_chunk_bvalid);
    if(s_axi_chunk_bresp!==expected)$fatal(1,"B response expected %d got %d",expected,s_axi_chunk_bresp);
    @(negedge aclk);s_axi_chunk_bready=0;
    if(write_busy!==0)$fatal(1,"busy did not drain through B");
  endtask
  task automatic read_burst(input logic[31:0]addr,input integer n,input bit error);
    @(negedge aclk);s_axi_chunk_araddr=addr;s_axi_chunk_arlen=n-1;s_axi_chunk_arvalid=1;
    do @(posedge aclk);while(!s_axi_chunk_arready);
    @(negedge aclk);s_axi_chunk_arvalid=0;s_axi_chunk_rready=1;
    for(integer k=0;k<n;k=k+1)begin
      do @(posedge aclk);while(!s_axi_chunk_rvalid);
      if(s_axi_chunk_rresp !== (error?2'b10:2'b00))$fatal(1,"read response");
      if(s_axi_chunk_rlast !== (k==n-1))$fatal(1,"read LAST");
      if(s_axi_chunk_rdata !== (error?32'b0:32'h42000000+k))$fatal(1,"read data");
      @(negedge aclk);
    end
    s_axi_chunk_rready=0;
  endtask
  initial begin
    s_axi_chunk_awsize=2;s_axi_chunk_awburst=1;s_axi_chunk_wstrb=15;
    s_axi_chunk_arsize=2;s_axi_chunk_arburst=1;
    m_axi_bram_awready=1;m_axi_bram_wready=1;m_axi_bram_arready=1;
    repeat(4)@(negedge aclk);aresetn=1;bank_writable=3;
    aw(32'hA0020010,4);
    bank_writable=0; // legal AW already accepted: drain despite owner closure.
    wd(4);repeat(4)@(negedge aclk);
    if(write_busy!==1 || aw_seen!=1 || w_seen!=4)$fatal(1,"accepted write drain");
    response(0);
    aw(32'hA0030000,1);wd(1);response(2);
    if(aw_seen!=1 || w_seen!=4)$fatal(1,"locked write forwarded");
    bank_writable=3;
    aw(32'hA002fffc,2);wd(2);response(2);
    aw(32'hA0020ffc,2);wd(2);response(2); // AXI4 4KiB boundary.
    if(aw_seen!=1)$fatal(1,"cross boundary write forwarded");
    aw(32'hA0020020,2);wd(2,1);response(2); // normalized WLAST, error returned.
    if(aw_seen!=2 || w_seen!=6)$fatal(1,"malformed WLAST normalization");
    aw(32'hA0020002,1);wd(1);response(2); // unaligned.
    read_burst(32'hA002fffc,2,1);
    read_burst(32'hA0020020,3,0);
    if(ar_seen!=1)$fatal(1,"invalid read forwarded");
    @(negedge aclk);aresetn=0;
    repeat(2)@(negedge aclk);if(write_busy!==0)$fatal(1,"reset busy");
    $display("PASS tb_axi_guard: frozen bank, outstanding drain, boundaries, WLAST, readback, reset");
    $finish;
  end
  initial begin #200000;$fatal(1,"timeout");end
endmodule

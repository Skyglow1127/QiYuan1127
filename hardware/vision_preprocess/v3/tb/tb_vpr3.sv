`timescale 1ns/1ps
module tb_vpr3;
 reg clk=0;always #5 clk=~clk;
 reg resetn=0;reg [7:0] awaddr=0,araddr=0;reg awvalid=0,wvalid=0,bready=0,arvalid=0,rready=0;
 reg [31:0] wdata=0;reg [3:0] wstrb=15;wire awready,wready,bvalid,arready,rvalid;wire [1:0] bresp,rresp;wire [31:0] rdata;
 reg [31:0] sd=0;reg [3:0] sk=15;reg sv=0,sl=0,mr=0;wire sr,mv,ml;wire [31:0] md;wire [3:0] mk;
 integer cycle=0,checks=0,frames=0,k,which,manifest,scanned,iw,ih,rw,rh,ow,oh,pad,expected_cases=14;
 integer cw,ch,crw,crh,cpad,output_file=0;reg [31:0] value;
 reg [31:0] src[0:2073599],gold[0:409599];string vectors,path,name,actual_directory;
 reg hold_final=0,held=0;reg[31:0] held_data;reg[3:0] held_keep;reg held_last;
 vpr3_axi dut(.aclk(clk),.aresetn(resetn),.s_axi_awaddr(awaddr),.s_axi_awvalid(awvalid),.s_axi_awready(awready),
 .s_axi_wdata(wdata),.s_axi_wstrb(wstrb),.s_axi_wvalid(wvalid),.s_axi_wready(wready),.s_axi_bresp(bresp),.s_axi_bvalid(bvalid),.s_axi_bready(bready),
 .s_axi_araddr(araddr),.s_axi_arvalid(arvalid),.s_axi_arready(arready),.s_axi_rdata(rdata),.s_axi_rresp(rresp),.s_axi_rvalid(rvalid),.s_axi_rready(rready),
 .s_axis_tdata(sd),.s_axis_tkeep(sk),.s_axis_tlast(sl),.s_axis_tvalid(sv),.s_axis_tready(sr),
 .m_axis_tdata(md),.m_axis_tkeep(mk),.m_axis_tlast(ml),.m_axis_tvalid(mv),.m_axis_tready(mr));
 always @(negedge clk)begin cycle=cycle+1;mr=resetn&&(cycle%7!=0)&&(cycle%11!=0)&&!(hold_final&&mv&&ml);end
 task check(input bit ok,input string msg);begin checks=checks+1;if(!ok)$fatal(1,"TEST_FAIL VPR3 %s cycle=%0d case=%s",msg,cycle,name);end endtask
 // Abort explicitly terminates AXIS validity. Other stalls must retain payload.
 always @(posedge clk)begin
  if(!resetn||dut.abort_frame)held=0;
  else begin
   if(held)check(mv&&md===held_data&&mk===held_keep&&ml===held_last,"output stable under backpressure");
   held=mv&&!mr;held_data=md;held_keep=mk;held_last=ml;
  end
 end
 task wr_response(input [7:0] address,input [31:0] data,input [1:0] response);begin
  fork
   begin @(negedge clk);awaddr=address;awvalid=1;do @(posedge clk);while(!awready);@(negedge clk);awvalid=0;end
   begin repeat(2)@(negedge clk);wdata=data;wvalid=1;do @(posedge clk);while(!wready);@(negedge clk);wvalid=0;end
  join
  bready=1;do @(posedge clk);while(!bvalid);check(bresp==response,"AXIL write response");@(negedge clk);bready=0;
 end endtask
 task wr(input[7:0]address,input[31:0]data);begin wr_response(address,data,0);end endtask
 task rd(input [7:0] address,output [31:0] data);begin
  @(negedge clk);araddr=address;arvalid=1;do @(posedge clk);while(!arready);@(negedge clk);arvalid=0;rready=1;
  do @(posedge clk);while(!rvalid);data=rdata;check(rresp==0,"AXIL read response");@(negedge clk);rready=0;
 end endtask
 task status(input [3:0] errors,input integer ni,no);integer tries;reg[31:0] v;begin
  tries=0;v=0;while((v&3)!=2&&tries<100)begin rd(4,v);tries=tries+1;end
  check((v&3)==2,"frame completes");rd('h14,v);check(v==errors,"error flags");rd('h18,v);check(v==ni,"input counter");
  rd('h1c,v);if(no>=0)check(v==no,"output counter stays pixels");else check(v<50176,"aborted partial border count");
 end endtask
 task configure(input integer iw,ih,rw,rh,ow,oh,pad,input bit bypass);begin
  cw=ow;ch=oh;crw=rw;crh=rh;cpad=pad;
  wr(8,(ih<<16)|iw);wr(12,(oh<<16)|ow);wr('h28,(rh<<16)|rw);wr('h2c,pad);wr('h10,bypass);wr(0,1);
 end endtask
 task packet(input integer ni,no,input bit exact,bad);integer i,j,c,d,sum,maxerr,bytes,beats,n,offset,pixel_index,channel;reg[31:0] p,v;reg[3:0] expected_keep;begin
  sum=0;maxerr=0;bytes=exact?no*4:no*3;beats=(bytes+3)/4;hold_final=1;
  fork
   begin
    for(i=0;i<ni;i=i+1)begin
     @(negedge clk);if(i%5==0)begin sv=0;@(negedge clk);end
     sd=src[i];sk=(bad&&i==0)?3:15;sl=bad?0:(i==ni-1);sv=1;
     do @(posedge clk);while(!sr);
    end
    // A following frame offered without a fresh cfg must never enter the core.
    @(negedge clk);sd=32'hcafef00d;sv=1;sl=0;sk=15;
    repeat(4)begin @(posedge clk);check(!sr,"no unconfigured next-frame beat admitted");end
    @(negedge clk);sv=0;sl=0;sk=15;
   end
   begin
    for(j=0;j<beats;j=j+1)begin
     do @(posedge clk);while(!(mv&&mr));p=md;n=bytes-j*4;if(n>4)n=4;
     expected_keep=4'hf>>(4-n);if(exact&&bad&&j==0)expected_keep=3;
     check(mk===expected_keep,"precise byte TKEEP");check(ml==(j==beats-1),"packed TLAST");
     if(exact)check(p===gold[j],"bypass full 32-bit exact");
     else begin
      for(c=0;c<n;c=c+1)begin
       offset=j*4+c;pixel_index=offset/3;channel=offset%3;
       if(output_file!=0)$fwrite(output_file,"%c",p[c*8+:8]);
       if(pixel_index/cw<(ch-crh)/2||pixel_index/cw>=(ch-crh)/2+crh||pixel_index%cw<(cw-crw)/2||pixel_index%cw>=(cw-crw)/2+crw)
        check(p[c*8+:8]===cpad[7:0],"exact border byte");
       d=int'(p[c*8+:8])-int'(gold[pixel_index][channel*8+:8]);if(d<0)d=-d;sum=sum+d;if(d>maxerr)maxerr=d;
       check(d<=2,"bilinear byte error <= 2");
      end
      for(c=n;c<4;c=c+1)check(p[c*8+:8]==0,"zero unused tail bytes");
     end
    end
   end
   begin
    // Hold the final beat, even after the HLS core may have completed. The
    // application-visible busy/done and cfg readiness must include this tail.
    do @(negedge clk);while(!(mv&&ml&&!mr));
    rd(4,v);check(v[0]&&!v[1],"busy until final packed beat accepted");
    wr_response(0,1,2'b10);check(mv&&ml,"new start cannot overwrite stalled tail");
    repeat(4)@(negedge clk);hold_final=0;
   end
  join
  if(!exact)check(sum<=no*3/2,"bilinear MAE <= 0.5");
  status(bad?6:0,ni,no);frames=frames+1;
  $display("FRAME_PASS VPR3 frame=%0d case=%s pixels=%0d bytes=%0d max_error=%0d sum_error=%0d",frames,name,no,bytes,maxerr,sum);
 end endtask
 task abort_packed_tail;integer i;reg[31:0] v;begin
  name="abort_packed_tail";hold_final=1;configure(16,16,16,16,17,17,114,0);
  fork
   begin
    for(i=0;i<256;i=i+1)begin
     @(negedge clk);sd=32'h0055aa11;sv=1;sl=i==255;
     do @(posedge clk);while(!sr);
    end
    @(negedge clk);sv=0;sl=0;
   end
   begin
    do @(negedge clk);while(!(mv&&ml&&!mr));
    repeat(8)@(negedge clk);rd(4,v);check(v[0]&&!v[1],"tail-pending frame busy before abort");
    wr(0,2);hold_final=0;
   end
  join
  status(8,256,289);repeat(8)@(negedge clk);check(!mv,"abort clears packed tail");
 end endtask
 initial begin
  if(!$value$plusargs("VECTOR_DIR=%s",vectors))$fatal(1,"VECTOR_DIR missing");
  if($value$plusargs("EXPECTED_CASES=%d",expected_cases))begin end
  if($value$plusargs("OUTPUT_DIR=%s",actual_directory))begin end
  repeat(8)@(negedge clk);resetn=1;repeat(8)@(negedge clk);
  rd('h24,value);check(value=='h56505233,"VPR3 tag");rd('h20,value);check(value=='h04380780,"max input");
  rd('h30,value);check(value=='h02800280,"max canvas");rd('h34,value);check(value==1,"compact RGB888 capability");
  configure(15,25,16,16,16,16,114,0);status(1,0,0);
  configure(640,480,160,120,641,640,114,0);status(1,0,0);
  configure(640,480,159,120,224,224,0,0);status(1,0,0);
  configure(33,25,224,170,224,224,114,0);repeat(30)@(negedge clk);wr(0,2);status(8,0,-1);repeat(8)@(negedge clk);
  abort_packed_tail();
  manifest=$fopen({vectors,"/cases.txt"},"r");check(manifest!=0,"frozen vector manifest");which=0;
  while(!$feof(manifest))begin
   scanned=$fscanf(manifest,"%s %d %d %d %d %d %d %d\n",name,iw,ih,rw,rh,ow,oh,pad);
   if(scanned==8)begin
    path={vectors,"/",name};$readmemh({path,".input.hex"},src,0,iw*ih-1);$readmemh({path,".expected.hex"},gold,0,ow*oh-1);
    if(actual_directory!="")begin output_file=$fopen({actual_directory,"/",name,".actual.rgb"},"wb");check(output_file!=0,"open actual RGB output");end
    configure(iw,ih,rw,rh,ow,oh,pad,0);packet(iw*ih,ow*oh,0,0);which=which+1;
    if(output_file!=0)begin $fclose(output_file);output_file=0;end
   end else if(scanned!=-1)$fatal(1,"Malformed vector manifest record %0d",scanned);
  end
  $fclose(manifest);check(which==expected_cases,"complete frozen matrix");
  // Full HLS + wrapper coverage of every normal-output pixel-count residue mod4.
  for(ow=16;ow<=19;ow=ow+1)begin
   name="constant_tail";
   for(k=0;k<ow*17;k=k+1)begin src[k]=32'hc555aa11;gold[k]=32'h0055aa11;end
   configure(ow,17,ow,17,ow,17,114,0);packet(ow*17,ow*17,0,0);
  end
  for(k=0;k<825;k=k+1)begin src[k]=32'ha5ff0055^(k*32'h1234567);gold[k]=src[k];end
  name="bypass_exact";configure(33,25,33,25,33,25,114,1);packet(825,825,1,0);
  name="bypass_errors";configure(33,25,33,25,33,25,114,1);packet(825,825,1,1);
  check(frames==expected_cases+6,"all complete frames");
  $display("TEST_PASS VPR3_WRAPPER frames=%0d checks=%0d vector_cases=%0d",frames,checks,which);$finish;
 end
 initial begin #200000000;$fatal(1,"TEST_FAIL VPR3 watchdog");end
endmodule

`timescale 1ns/1ps
module tb_rgb888_packer;
    reg clk=0; always #5 clk=~clk;
    reg resetn=0,clear=0,sv=0,sl=0,mr=0;
    reg [31:0] sd=0;
    wire sr,mv,ml,frame_done; wire [31:0] md; wire [3:0] mk;
    integer cycle=0,checks=0,frames=0,length,case_index;
    reg allow_output=1,held=0;
    reg [31:0] held_data;reg [3:0] held_keep;reg held_last;
    rgb888_axis_packer dut(.aclk(clk),.aresetn(resetn),.clear(clear),
        .s_axis_tdata(sd),.s_axis_tvalid(sv),.s_axis_tlast(sl),.s_axis_tready(sr),
        .m_axis_tdata(md),.m_axis_tkeep(mk),.m_axis_tvalid(mv),.m_axis_tlast(ml),.m_axis_tready(mr),.frame_done(frame_done));
    task check(input bit ok,input string msg);begin
        checks=checks+1;if(!ok)$fatal(1,"TEST_FAIL PACKER %s cycle=%0d",msg,cycle);
    end endtask
    function automatic [31:0] pixel(input integer index);
        pixel=32'ha5000000|(((index*79+37)&255)<<16)|(((index*53+19)&255)<<8)|((index*31+7)&255);
    endfunction
    always @(negedge clk)begin
        cycle=cycle+1;mr=resetn&&!clear&&allow_output&&(cycle%7!=0)&&(cycle%11!=0);
    end
    always @(posedge clk)begin
        if(!resetn||clear)held=0;
        else begin
            if(held)check(mv&&md===held_data&&mk===held_keep&&ml===held_last,"stable VALID/DATA/KEEP/LAST while stalled");
            held=mv&&!mr;held_data=md;held_keep=mk;held_last=ml;
        end
    end
    task packet(input integer count,input integer gaps);integer i,j,b,n,offset;reg [31:0] p;reg[3:0] expected_keep;begin
        fork
            begin
                for(i=0;i<count;i=i+1)begin
                    @(negedge clk);
                    if(gaps!=0&&i%gaps==0)begin sv=0;repeat(2)@(negedge clk);end
                    sd=pixel(i);sv=1;sl=i==count-1;
                    do @(posedge clk);while(!sr);
                end
                @(negedge clk);sv=0;sl=0;
            end
            begin
                for(j=0;j<(count*3+3)/4;j=j+1)begin
                    do @(posedge clk);while(!(mv&&mr));
                    n=count*3-j*4;if(n>4)n=4;
                    expected_keep=4'hf>>(4-n);
                    check(mk===expected_keep,"tail KEEP");check(ml==((j+1)*4>=count*3),"packet LAST");
                    for(b=0;b<n;b=b+1)begin
                        offset=j*4+b;p=pixel(offset/3);
                        check(md[b*8+:8]===p[(offset%3)*8+:8],"byte order and payload");
                    end
                    for(b=n;b<4;b=b+1)check(md[b*8+:8]==0,"unused tail bytes zero");
                end
                @(negedge clk);check(frame_done,"last beat completion pulse");
            end
        join
        repeat(3)@(posedge clk);check(!mv,"packet has no extra beat");
        frames=frames+1;
    end endtask
    task abort_buffer(input integer pixels,input bit end_packet);integer i;begin
        @(negedge clk);allow_output=0;mr=0;
        for(i=0;i<pixels;i=i+1)begin
            @(negedge clk);sd=pixel(i+100);sv=1;sl=end_packet&&i==pixels-1;
            do @(posedge clk);while(!sr);
        end
        @(negedge clk);sv=0;sl=0;
        repeat(4)@(negedge clk);
        if(end_packet)check(mv&&ml,"abort with stalled final tail");
        clear=1;@(negedge clk);clear=0;allow_output=1;
        repeat(3)@(posedge clk);check(!mv&&!frame_done,"abort discards all buffered bytes");
        packet(9,2);
    end endtask
    initial begin
        repeat(5)@(negedge clk);resetn=1;
        for(length=1;length<=33;length=length+1)packet(length,length%5);
        // Empty, partial (3-byte), full+partial (6-byte), and final-tail aborts.
        abort_buffer(0,0);abort_buffer(1,0);abort_buffer(2,0);abort_buffer(1,1);
        check(frames==37,"all packer cases completed");
        $display("TEST_PASS VPR3_PACKER frames=%0d checks=%0d",frames,checks);$finish;
    end
    initial begin #1000000;$fatal(1,"TEST_FAIL PACKER watchdog");end
endmodule

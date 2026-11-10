`timescale 1ns/1fs
// Targeted timing-netlist check: real asynchronous input, startup and cadence.
// Long-window frequency accuracy is checked separately in RTL regressions.
module tb_postroute;
    reg clk=0;always #5 clk=~clk;
    reg rst=1,signal_in=0;
    wire valid,full,fault,timeout_flag,ev,req;
    wire [63:0] f,last,mean,tick;
    lamor_frequency_meter dut(.clk(clk),.rst(rst),.larmor_in(signal_in),
        .frequency_uHz(f),.result_last_tick(last),.result_mean_tick(mean),
        .result_valid(valid),.window_full(full),.fault(fault),.input_timeout(timeout_flag),
        .debug_event_valid(ev),.debug_event_tick(tick),.debug_sample_req(req));
    integer i,events=0,requests=0;
    real target;
    reg [63:0] prev;
    initial begin
        // Xilinx glbl GSR lasts 100 ns. Start after it is released.
        #200;@(negedge clk);rst=0;
        for(i=0;i<32;i=i+1)begin
            target=303.7+i*(1e9/350000.0);
            #(target-$realtime);signal_in=1;
            #800;signal_in=0;
        end
        #1000;
        if(events!=32 || requests!=0 || fault || valid || full)
            $fatal(1,"Postroute mismatch events=%0d requests=%0d fault=%b valid=%b full=%b",events,requests,fault,valid,full);
        $display("PASS postroute SDF 32 asynchronous edges and startup (scheduler checked in full RTL)");$finish;
    end
    always @(negedge clk)begin
        if(!rst)begin
            if(ev===1'b1)begin
                if(events>0 && tick-prev!=285 && tick-prev!=286)
                    $fatal(1,"Postroute timestamp interval wrong %0d",tick-prev);
                prev=tick;events=events+1;
                if(events%8==0)$display("Postroute progress: %0d edges at %0t",events,$time);
            end
            if(req===1'b1)requests=requests+1;
            if(fault===1'b1)$fatal(1,"Postroute fault");
        end
    end
    initial begin #2000000;$fatal(1,"Postroute watchdog");end
endmodule

`timescale 1ns/1ps
module tb_faults;
    reg clk=0;always #5 clk=~clk;
    reg rst=1,ev=0,req=0;
    reg [63:0] tick=0;
    wire ready,valid,full,fault,busy;
    wire [63:0] freq;
    sfls_core #(.WINDOW_LOG2(3)) dut(.clk(clk),.rst(rst),.event_valid(ev),
        .event_tick(tick),.event_ready(ready),.sample_req(req),.result_valid(valid),
        .frequency_uHz(freq),.result_last_tick(),.result_mean_tick(),
        .full(full),.fault(fault),.compute_busy(busy),.debug_q(),.debug_sum());
    task reset_core;begin
        rst=1;ev=0;req=0;repeat(4)@(negedge clk);rst=0;
        if(fault || full || valid)$fatal(1,"Reset failed");
    end endtask
    task put(input [63:0] t);begin
        tick=t;ev=1;@(negedge clk);ev=0;repeat(3)@(negedge clk);
    end endtask
    integer i;
    initial begin
        reset_core();req=1;@(negedge clk);req=0;repeat(110)@(negedge clk);
        if(valid || fault || full)$fatal(1,"Premature output");
        put(100);put(100);if(!fault)$fatal(1,"Duplicate timestamp not rejected");
        reset_core();put(200);put(100);if(!fault)$fatal(1,"Reverse timestamp not rejected");
        reset_core();tick=100;ev=1;repeat(2)@(negedge clk);ev=0;
        if(!fault)$fatal(1,"Overrun not detected");
        reset_core();for(i=0;i<8;i=i+1)put(1000+i*1000);
        if(!full)$fatal(1,"Window not full");
        req=1;repeat(3)@(negedge clk);req=0;
        if(!fault)$fatal(1,"Overlapping request not detected");
        repeat(110)@(negedge clk);if(valid)$fatal(1,"Invalid result leaked after fault");
        reset_core();for(i=0;i<8;i=i+1)put(1000+i*1000);
        req=1;@(negedge clk);req=0;wait(valid);@(negedge clk);
        if(freq!=64'd100000000000 || fault)$fatal(1,"Recovery result wrong freq=%0d fault=%b q=%0d sum=%0d",freq,fault,dut.q,dut.sum);
        $display("PASS faults premature, duplicate, reverse, overrun, busy, reset_recovery");$finish;
    end
    initial begin #1000000;$fatal(1,"Fault-test watchdog");end
endmodule

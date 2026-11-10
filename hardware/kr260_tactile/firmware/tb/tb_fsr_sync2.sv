`timescale 1ns / 1ps
module tb_fsr_sync2;
    reg clk=0;
    always #20 clk=~clk;
    reg resetn=0;
    reg [1:0] din=0;
    wire [1:0] dout;
    fsr_sync2 dut(.clk(clk),.resetn(resetn),.din(din),.dout(dout));
    reg [1:0] expected_stage1=0, expected_stage2=0;
    integer checks=0;
    integer i;
    always @(posedge clk) begin
        expected_stage2 = resetn ? expected_stage1 : 2'b00;
        expected_stage1 = resetn ? din : 2'b00;
        #1;
        if (dout !== expected_stage2) $fatal(1,"CDC mismatch check=%0d expected=%b got=%b",checks,expected_stage2,dout);
        checks=checks+1;
    end
    initial begin
        // Exercise all states, independent transitions, and reset during activity.
        // Inputs change 7ns after falling edges, deliberately asynchronous to sampling.
        for (i=0; i<240; i=i+1) begin
            @(negedge clk); #7;
            resetn = !(i<3 || (i>=109 && i<114));
            din[0] = ((i/3)%2);
            din[1] = ((i/7)%2);
        end
        if (checks != 240) $fatal(1,"Wrong check count %0d",checks);
        $display("TEST_PASS: FSR_CDC checks=240 reset_windows=2 channels=2 clock=25MHz");
        $finish;
    end
    initial begin #20000; $fatal(1,"CDC timeout"); end
endmodule

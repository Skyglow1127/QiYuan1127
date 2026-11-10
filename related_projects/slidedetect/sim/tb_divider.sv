`timescale 1ns/1ps
module tb_divider;
    reg clk=0;always #5 clk=~clk;
    reg rst=1,start=0;
    reg [95:0] a=0,b=1;
    wire busy,done,zero;
    wire [95:0] result;
    udiv_round dut(.clk(clk),.rst(rst),.start(start),.numerator(a),.denominator(b),
                  .busy(busy),.done(done),.divide_by_zero(zero),.quotient(result));
    reg [95:0] expected,remnant;
    reg [96:0] twice_rem;
    integer i,n;
    task check(input [95:0] x,input [95:0] y);begin
        a=x;b=y;
        if(y!=0)begin
            remnant=x%y;twice_rem={1'b0,remnant}<<1;
            expected=x/y+(twice_rem>={1'b0,y});
        end
        start=1;@(negedge clk);start=0;n=0;
        while(!done && n<100)begin @(negedge clk);n=n+1;end
        if(!done || (y==0 && !zero) || (y!=0 && (zero || result!==expected)))
            $fatal(1,"Divider mismatch a=%h b=%h got=%h expected=%h",x,y,result,expected);
        @(negedge clk);
    end endtask
    initial begin
        repeat(4)@(negedge clk);rst=0;
        check(0,1);check(1,2);check(3,2);check({96{1'b1}},1);
        check(96'hffffffffffffffffffffffff,96'h800000000000000000000000);
        check(42,0);
        for(i=0;i<500;i=i+1)check({$urandom,$urandom,$urandom},{$urandom,$urandom,$urandom}|96'd1);
        $display("PASS divider 506 cases including rounding and divide_by_zero");$finish;
    end
    initial begin #1000000;$fatal(1,"Divider watchdog");end
endmodule

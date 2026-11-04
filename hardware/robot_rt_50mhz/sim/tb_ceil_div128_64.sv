`timescale 1ns/1ps
module tb_ceil_div128_64;
    logic clk=0;always #5 clk=~clk;
    logic resetn=0,start=0;
    logic[127:0] dividend=0;
    logic[63:0] divisor=0,origin=0;
    wire busy,done,invalid,absolute_overflow;
    wire[127:0] ceil_quotient;
    wire[63:0] absolute_cycle;
    ceil_div128_64 dut(.*);
    integer checks=0,test_index=0,reset_cases=0;
    integer descriptor,vector_count,fields,dummy;
    logic[127:0] input_dividend,expected_ceil;
    logic[63:0] input_divisor,input_origin,expected_absolute;
    integer expected_invalid,expected_overflow;
    string vector_path;

    task automatic check(input logic condition,input string message);
        begin
            checks=checks+1;
            if(condition!==1'b1)$fatal(1,"%s vector=%0d busy=%b done=%b invalid=%b overflow=%b",message,test_index,busy,done,invalid,absolute_overflow);
        end
    endtask

    task automatic abort_at(input integer phase);
        begin
            @(negedge clk);start=1;dividend='1;divisor=3;origin=7;
            @(posedge clk);#1;check(busy&&!done,"reset case starts");
            @(negedge clk);start=0;
            repeat(phase)begin @(posedge clk);#1;check(busy&&!done,"reset case not completed before abort");end
            @(negedge clk);resetn=0;start=1;dividend=0;divisor=0;origin='1;
            @(posedge clk);#1;
            check(!busy&&!done&&!invalid&&!absolute_overflow&&ceil_quotient==0&&absolute_cycle==0,"synchronous reset clears all outputs and ignores start");
            @(negedge clk);resetn=1;start=0;
            repeat(132)begin @(posedge clk);#1;check(!busy&&!done,"reset abort cannot deliver a stale completion");end
            reset_cases=reset_cases+1;
        end
    endtask

    task automatic run_vector;
        integer latency;
        begin
            @(negedge clk);dividend=input_dividend;divisor=input_divisor;origin=input_origin;start=1;
            @(posedge clk);#1;
            check(busy===1'b1&&done===1'b0,"E0 accepts a request");
            check(invalid===1'b0&&absolute_overflow===1'b0&&absolute_cycle===64'd0,"E0 clears previous result flags");
            for(latency=1;latency<=130;latency=latency+1)begin
                // Perturb ALL external operands on every busy clock. Repeated
                // starts include the final DIVIDE, ROUND and FINISH edges.
                @(negedge clk);
                dividend={input_origin^64'hfedcba9876543210,input_divisor^64'h0123456789abcdef}^{96'd0,latency[31:0]};
                divisor=(latency%3==0)?64'hffffffffffffffff:64'd0;
                origin=~input_origin^{32'd0,latency[31:0]};
                start=(latency%17==0)||(latency>=127);
                @(posedge clk);#1;
                if(latency<130)check(busy===1'b1&&done===1'b0,"no early busy drop or done");
                else check(busy===1'b0&&done===1'b1,"done exactly at E130");
            end
            if(ceil_quotient!==expected_ceil||absolute_cycle!==expected_absolute)begin
                $display("input X=%h D=%h origin=%h",input_dividend,input_divisor,input_origin);
                $display("ceil got=%h expected=%h; absolute got=%h expected=%h",ceil_quotient,expected_ceil,absolute_cycle,expected_absolute);
                $fatal(1,"unsigned integer arithmetic mismatch vector=%0d",test_index);
            end
            check(invalid===expected_invalid[0]&&absolute_overflow===expected_overflow[0],"exact invalid and uint64 overflow flags");
            @(negedge clk);start=0;dividend=0;divisor=1;origin=0;
            @(posedge clk);#1;
            check(!busy&&!done,"done is one clock pulse, ignored busy starts cannot restart");
            check(ceil_quotient===expected_ceil&&absolute_cycle===expected_absolute,"completed values remain stable while idle");
        end
    endtask

    initial begin
        vector_path="ceil_vectors.mem";
        dummy=$value$plusargs("VECTORS=%s",vector_path);
        descriptor=$fopen(vector_path,"r");
        if(descriptor==0)$fatal(1,"cannot open vectors: %s",vector_path);
        fields=$fscanf(descriptor,"%d\n",vector_count);
        if(fields!=1||vector_count<1||vector_count>1000000)$fatal(1,"invalid vector count");
        repeat(4)@(posedge clk);#1;
        check(!busy&&!done&&!invalid&&!absolute_overflow&&ceil_quotient==0&&absolute_cycle==0,"hardware reset state");
        @(negedge clk);resetn=1;
        abort_at(1);abort_at(64);abort_at(128);abort_at(129);
        for(test_index=0;test_index<vector_count;test_index=test_index+1)begin
            fields=$fscanf(descriptor,"%h %h %h %h %h %d %d\n",input_dividend,input_divisor,input_origin,expected_ceil,expected_absolute,expected_invalid,expected_overflow);
            if(fields!=7||expected_invalid<0||expected_invalid>1||expected_overflow<0||expected_overflow>1)$fatal(1,"malformed vector %0d fields=%0d",test_index,fields);
            run_vector();
        end
        fields=$fscanf(descriptor,"%h",input_dividend);
        if(fields==1)$fatal(1,"extra data after declared vectors");
        $fclose(descriptor);
        $display("PASS ceil_div128_64 vectors=%0d latency=130 cycles reset_cases=%0d checks=%0d design_only=1",vector_count,reset_cases,checks);
        $finish;
    end
    initial begin #10000000;$fatal(1,"ceil divider testbench timeout");end
endmodule

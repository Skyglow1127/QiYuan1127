`timescale 1ns/1ps
module tb_replay;
    parameter integer WINDOW_LOG2=15;
    parameter integer EDGE_STRIDE=1;
    reg clk=0;always #5 clk=~clk;
    reg rst=1,event_valid=0,sample_req=0;
    reg [63:0] event_tick=0;
    wire ready,valid,full,fault,busy;
    wire [63:0] freq,last_tick,mean_tick;
    wire signed [95:0] q,sum;
    sfls_core #(.WINDOW_LOG2(WINDOW_LOG2),.EDGE_STRIDE(EDGE_STRIDE)) dut(
        .clk(clk),.rst(rst),.event_valid(event_valid),.event_tick(event_tick),
        .event_ready(ready),.sample_req(sample_req),.result_valid(valid),
        .frequency_uHz(freq),.result_last_tick(last_tick),.result_mean_tick(mean_tick),
        .full(full),.fault(fault),.compute_busy(busy),.debug_q(q),.debug_sum(sum));
    integer fd,fo,rc,op,checks=0,events=0,cycles;
    reg [63:0] tick,expected_freq;
    reg [95:0] expected_q,expected_sum;
    string vector_path,result_path;
    initial begin
        if(!$value$plusargs("VECTORS=%s",vector_path)) $fatal(1,"Missing VECTORS");
        if(!$value$plusargs("RESULTS=%s",result_path)) result_path="results.csv";
        fd=$fopen(vector_path,"r");fo=$fopen(result_path,"w");
        if(!fd || !fo) $fatal(1,"Cannot open replay files");
        $fwrite(fo,"index,frequency_uHz,last_tick,mean_tick\n");
        repeat(4) @(negedge clk);rst=0;
        while(!$feof(fd)) begin
            rc=$fscanf(fd,"%d %h %h %h %h\n",op,tick,expected_q,expected_sum,expected_freq);
            if(rc!=5) $fatal(1,"Malformed vector record %0d",rc);
            if(op==3) begin
                rst=1;repeat(3) @(negedge clk);rst=0;
                if(full || fault || valid) $fatal(1,"Reset state incorrect");
            end else begin
                if(!ready) $fatal(1,"Unexpected backpressure");
                event_tick=tick;event_valid=1;
                @(negedge clk);event_valid=0;
                repeat(3) @(negedge clk);
                events=events+1;
                if(fault) $fatal(1,"Unexpected engine fault, event %0d",events);
                if(op==2) begin
                    if(!full || q!==expected_q || sum!==expected_sum)
                        $fatal(1,"Statistics mismatch event %0d Q=%h expected=%h sum=%h expected=%h",events,q,expected_q,sum,expected_sum);
                    sample_req=1;@(negedge clk);sample_req=0;cycles=0;
                    while(!valid && cycles<110) begin @(negedge clk);cycles=cycles+1;end
                    if(!valid || fault || freq!==expected_freq || last_tick!==tick || mean_tick!==(expected_sum>>WINDOW_LOG2))
                        $fatal(1,"Result mismatch index %0d got=%0d expected=%0d fault=%b",checks,freq,expected_freq,fault);
                    $fwrite(fo,"%0d,%0d,%0d,%0d\n",checks,freq,last_tick,mean_tick);
                    checks=checks+1;
                    @(negedge clk);
                end
            end
        end
        $display("PASS replay stride=%0d log2=%0d events=%0d checks=%0d",EDGE_STRIDE,WINDOW_LOG2,events,checks);
        $fclose(fd);$fclose(fo);$finish;
    end
    initial begin #1000000000;$fatal(1,"Replay watchdog");end
endmodule

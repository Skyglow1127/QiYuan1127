`timescale 1ns/1ps
module tb_precision;
    parameter integer WINDOW_SIZE=96;
    reg clk=0;always #2.5 clk=~clk;
    reg rst=1,event_valid=0,sample_req=0;
    reg [63:0] event_tick=0;
    wire ready,valid,full,fault,busy;
    wire [63:0] ufreq,last_tick,mean_tick;
    wire signed [95:0] q,sum;
    sfls_core #(.WINDOW_SIZE(WINDOW_SIZE),.CLOCK_HZ(200000000)) core(
        .clk(clk),.rst(rst),.event_valid(event_valid),.event_tick(event_tick),.event_ready(ready),
        .sample_req(sample_req),.result_valid(valid),.frequency_uHz(ufreq),
        .result_last_tick(last_tick),.result_mean_tick(mean_tick),.full(full),.fault(fault),
        .compute_busy(busy),.debug_q(q),.debug_sum(sum));
    reg inject=0,manual_valid=0;
    reg signed [95:0] manual_q=0;
    wire nready,nvalid,nfault;
    wire [63:0] nfreq;
    sfls_nhz_readout #(.WINDOW_SIZE(WINDOW_SIZE)) fine(
        .clk(clk),.rst(rst),.snapshot_valid(inject?manual_valid:sample_req),
        .snapshot_q(inject?manual_q:q),.ready(nready),.result_valid(nvalid),.frequency_nHz(nfreq),.fault(nfault));
    integer fd,fo,rc,op,checks=0,events=0,cycles,max_latency=0;
    reg [63:0] tick,expected_u,expected_n;
    reg [95:0] expected_q,expected_sum;
    string vectors,results;
    task reset_all;
        begin rst=1;manual_valid=0;sample_req=0;repeat(3) @(negedge clk);rst=0;@(negedge clk);end
    endtask
    initial begin
        if(!$value$plusargs("VECTORS=%s",vectors)) $fatal(1,"Missing vectors");
        if(!$value$plusargs("RESULTS=%s",results)) $fatal(1,"Missing results");
        fd=$fopen(vectors,"r");fo=$fopen(results,"w");
        if(!fd || !fo) $fatal(1,"Cannot open replay files");
        $fwrite(fo,"frequency_uHz,frequency_nHz,last_tick,mean_tick,latency_cycles\n");
        reset_all;
        while(!$feof(fd)) begin
            rc=$fscanf(fd,"%d %h %h %h %h %h\n",op,tick,expected_q,expected_sum,expected_u,expected_n);
            if(rc!=6) $fatal(1,"Malformed input");
            if(op==3) reset_all;
            else begin
                if(!ready) $fatal(1,"Core not ready");
                event_tick=tick;event_valid=1;@(negedge clk);event_valid=0;repeat(3) @(negedge clk);events=events+1;
                if(fault || nfault) $fatal(1,"Unexpected fault");
                if(op==2) begin
                    if(!full || !nready || q!==expected_q || sum!==expected_sum) $fatal(1,"Snapshot statistics mismatch");
                    sample_req=1;@(negedge clk);sample_req=0;cycles=0;
                    while(!valid && cycles<110) begin @(negedge clk);cycles=cycles+1;end
                    if(!valid || !nvalid || fault || nfault || ufreq!==expected_u || nfreq!==expected_n || last_tick!==tick || mean_tick!==(expected_sum/WINDOW_SIZE))
                        $fatal(1,"Precision mismatch u=%0d/%0d n=%0d/%0d",ufreq,expected_u,nfreq,expected_n);
                    if(cycles>max_latency) max_latency=cycles;
                    $fwrite(fo,"%0d,%0d,%0d,%0d,%0d\n",ufreq,nfreq,last_tick,mean_tick,cycles);
                    checks=checks+1;@(negedge clk);
                end
            end
        end
        // Reject invalid denominator, busy overwrite, and output overflow; reset recovers.
        inject=1;reset_all;manual_q=0;manual_valid=1;@(negedge clk);manual_valid=0;
        if(!nfault || nvalid) $fatal(1,"Zero snapshot was accepted");
        reset_all;manual_q=-1;manual_valid=1;@(negedge clk);manual_valid=0;
        if(!nfault || nvalid) $fatal(1,"Negative snapshot was accepted");
        reset_all;manual_q=96'd1000000000000;manual_valid=1;repeat(2) @(negedge clk);manual_valid=0;
        if(!nfault) $fatal(1,"Busy overwrite not rejected");
        repeat(105) begin @(negedge clk);if(nvalid) $fatal(1,"Faulted request produced output");end
        reset_all;manual_q=1;manual_valid=1;@(negedge clk);manual_valid=0;
        repeat(105) begin @(negedge clk);if(nvalid) $fatal(1,"Overflow produced valid output");end
        if(!nfault) $fatal(1,"Overflow not reported");
        reset_all;manual_q=expected_q;manual_valid=1;@(negedge clk);manual_valid=0;
        // expected_q may be zero on the last non-request vector; use a known valid snapshot.
        if(expected_q==0) begin reset_all;manual_q=96'd1000000000000;manual_valid=1;@(negedge clk);manual_valid=0;end
        cycles=0;while(!nvalid && cycles<110) begin @(negedge clk);cycles=cycles+1;end
        if(!nvalid || nfault || !nready) $fatal(1,"Reset did not restore fine readout");
        $display("PASS precision M=%0d events=%0d checks=%0d latency=%0d fault_checks=4",WINDOW_SIZE,events,checks,max_latency);
        $fclose(fd);$fclose(fo);$finish;
    end
    initial begin #1000000000;$fatal(1,"Watchdog");end
endmodule

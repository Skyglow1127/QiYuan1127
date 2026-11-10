`timescale 1ns/1fs
// Real clock simulation of asynchronous input -> synchronizer -> full RTL.
// Stimulus is scheduled from absolute phase, never by a rounded repeated period.
module tb_end_to_end;
    parameter integer WINDOW_LOG2=15;
    parameter integer EDGE_STRIDE=1;
    localparam integer M=1<<WINDOW_LOG2;
    localparam [95:0] MW=M;
    localparam [95:0] NUMERATOR=MW*(MW*MW-1)/6*100000000*EDGE_STRIDE*1000000;
    reg clk=0;always #5 clk=~clk;
    reg rst=1,signal_in=0;
    wire valid,full,fault,timeout_flag,event_valid,sample_req;
    wire [63:0] freq,last_tick,mean_tick,event_tick;
    lamor_frequency_meter #(.WINDOW_LOG2(WINDOW_LOG2),.EDGE_STRIDE(EDGE_STRIDE)) dut(
        .clk(clk),.rst(rst),.larmor_in(signal_in),.frequency_uHz(freq),
        .result_last_tick(last_tick),.result_mean_tick(mean_tick),
        .result_valid(valid),.window_full(full),.fault(fault),.input_timeout(timeout_flag),
        .debug_event_valid(event_valid),.debug_event_tick(event_tick),.debug_sample_req(sample_req));
    real frequency=350000.0,run_ms=115.0,phase_ns=103.7,target_ns,origin;
    integer edge_number=0,fd,checks=0,retained=0;
    reg [63:0] ring[0:M-1];
    integer wp=0,n=0,j;
    reg [1:0] commit_delay=0;
    reg [63:0] delay0,delay1;
    reg signed [95:0] reference_q,reference_sum,term;
    reg [95:0] expected;
    reg [63:0] expected_last,expected_mean,previous_result_tick;
    reg pending=0,had_result=0;
    integer latency=0;
    string output_path;
    initial begin
        if($value$plusargs("FREQUENCY=%f",frequency)) begin end
        if($value$plusargs("RUN_MS=%f",run_ms)) begin end
        if(!$value$plusargs("RESULTS=%s",output_path))output_path="end_to_end.csv";
        fd=$fopen(output_path,"w");
        if(!fd)$fatal(1,"Cannot open output");
        $fwrite(fd,"time_ns,frequency_uHz,reference_uHz,last_tick,mean_tick,latency_cycles\n");
        repeat(5)@(negedge clk);rst=0;origin=$realtime+phase_ns;
        while($realtime<run_ms*1e6) begin
            target_ns=origin+edge_number*(1e9/frequency);
            #(target_ns-$realtime);signal_in=1;
            #(0.3e9/frequency);signal_in=0;
            edge_number=edge_number+1;
        end
        #10000;
        if(checks<3)$fatal(1,"Too few valid results %0d",checks);
        if(fault)$fatal(1,"Unexpected end-to-end fault");
        // Stop the signal: sticky timeout must invalidate outputs.
        #11000000;
        if(!timeout_flag || !fault || valid)$fatal(1,"Missing input-loss fault");
        rst=1;repeat(4)@(negedge clk);
        if(fault || full || valid)$fatal(1,"Fault did not reset");
        $display("PASS end_to_end frequency=%0.9f stride=%0d events=%0d outputs=%0d",frequency,EDGE_STRIDE,retained,checks);
        $fclose(fd);$finish;
    end
    always @(posedge clk) begin
        if(rst)begin
            wp=0;n=0;commit_delay=0;pending=0;had_result=0;
        end else begin
            if(pending)latency=latency+1;
            // The reference snapshot is computed from the committed FIFO,
            // before the simultaneous commit, matching the interface contract.
            if(sample_req && n==M && !fault)begin
                if(pending)$fatal(1,"Overlapping calculation");
                reference_q=0;reference_sum=0;
                for(j=0;j<M;j=j+1)begin
                    term=$signed({1'b0,ring[(wp+j)%M]});
                    reference_q=reference_q+(2*j-M+1)*term;
                    reference_sum=reference_sum+term;
                end
                expected=(2*NUMERATOR+reference_q)/(2*reference_q);
                expected_last=ring[(wp+M-1)%M];
                expected_mean=reference_sum>>WINDOW_LOG2;
                pending=1;latency=0;
            end
            if(commit_delay[1])begin
                ring[wp]=delay1;wp=(wp+1)%M;
                if(n<M)n=n+1;
            end
            delay1=delay0;
            if(event_valid)begin delay0=event_tick;retained=retained+1;end
            commit_delay={commit_delay[0],event_valid};
        end
    end
    always @(negedge clk)begin
        if(!rst && valid)begin
            if(!pending || freq!==expected[63:0] || last_tick!==expected_last || mean_tick!==expected_mean)
                $fatal(1,"End-to-end oracle mismatch got=%0d expected=%0d",freq,expected);
            if(had_result && event_tick-previous_result_tick!=100000)
                $fatal(1,"Output is not exactly 1 kHz");
            if(latency>105)$fatal(1,"Calculation exceeded latency budget");
            $fwrite(fd,"%0.0f,%0d,%0d,%0d,%0d,%0d\n",$realtime,freq,expected,last_tick,mean_tick,latency);
            pending=0;had_result=1;previous_result_tick=event_tick;checks=checks+1;
        end
    end
    // A real-valued delay avoids 32-bit unsized-integer truncation in XSim.
    initial begin #(36.0*1e9);$fatal(1,"End-to-end watchdog");end
endmodule

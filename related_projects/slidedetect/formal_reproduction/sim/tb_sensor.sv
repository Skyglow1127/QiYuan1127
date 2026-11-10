`timescale 1ns/1fs
module tb_sensor;
    parameter integer WINDOW_SIZE=96;
    parameter integer HOP_EVENTS=0;
    localparam integer CLOCK_HZ=200000000;
    localparam [95:0] M=WINDOW_SIZE;
    localparam [95:0] NUMERATOR=M*(M*M-1)/6*CLOCK_HZ*1000000;
    reg clk=0;always #2.5 clk=~clk;
    reg rst=1,signal_in=0;
    wire valid,full,fault,timeout_flag,event_valid,sample_req;
    wire [63:0] freq,last_tick,mean_tick,event_tick;
    sfls_sensor #(.WINDOW_SIZE(WINDOW_SIZE),.HOP_EVENTS(HOP_EVENTS)) dut(
        .clk(clk),.rst(rst),.larmor_in(signal_in),.frequency_uHz(freq),
        .result_last_tick(last_tick),.result_mean_tick(mean_tick),.result_valid(valid),
        .window_full(full),.fault(fault),.input_timeout(timeout_flag),
        .debug_event_valid(event_valid),.debug_event_tick(event_tick),.debug_sample_req(sample_req));
    reg [63:0] ring[0:WINDOW_SIZE-1],scheduled[0:2047];
    integer wp=0,n=0,j,checks=0,events=0,epoch=0,scheduled_n=0,observed_n=0;
    integer fd,commit_count=0,previous_request_events=0,epoch_checks=0;
    integer latency=0,max_latency=0;
    reg [1:0] commit_delay=0;
    reg [63:0] delay0,delay1,previous_result_tick;
    reg signed [95:0] reference_q,reference_sum,term;
    reg [95:0] expected;
    reg [63:0] expected_last,expected_mean;
    reg pending=0,had_result=0;
    real first_active_ns,origin,target_ns;
    string path;
    task acquire(input real frequency,input real duration_ms);
        integer k;
        begin
            epoch=epoch+1;scheduled_n=0;observed_n=0;epoch_checks=0;
            repeat(4) @(negedge clk);
            rst=0;first_active_ns=$realtime+2.5;origin=$realtime+103.7;k=0;
            while(k/frequency<duration_ms/1000.0) begin
                target_ns=origin+k*1e9/frequency;
                #(target_ns-$realtime);
                scheduled[scheduled_n]=$rtoi($ceil((target_ns-first_active_ns)/5.0))+2;
                scheduled_n=scheduled_n+1;
                signal_in=1;#(0.3e9/frequency);signal_in=0;k=k+1;
            end
            #10000;
            if(epoch_checks<2 || observed_n!=scheduled_n || fault)
                $fatal(1,"Acquisition failed checks=%0d events=%0d/%0d",epoch_checks,observed_n,scheduled_n);
        end
    endtask
    initial begin
        if(!$value$plusargs("RESULTS=%s",path))path="sensor.csv";
        fd=$fopen(path,"w");if(!fd)$fatal(1,"Cannot open results");
        $fwrite(fd,"epoch,time_ns,frequency_uHz,reference_uHz,last_tick,mean_tick,latency_cycles\n");
        acquire(2128.87,70.0);
        #11000000;
        if(!timeout_flag || !fault || valid)$fatal(1,"Input-loss timeout did not invalidate output");
        signal_in=1;#100;signal_in=0;#100;
        if(!fault)$fatal(1,"Input return must not clear sticky epoch fault");
        @(negedge clk);rst=1;
        repeat(4)@(negedge clk);
        if(fault || full || valid)$fatal(1,"Epoch reset did not clear state");
        acquire(2128.87001,65.0);
        $display("PASS sensor window=%0d hop=%0d epochs=%0d events=%0d outputs=%0d max_latency=%0d",WINDOW_SIZE,HOP_EVENTS,epoch,events,checks,max_latency);
        $fclose(fd);$finish;
    end
    always @(posedge clk) begin
        if(rst)begin
            wp=0;n=0;commit_delay=0;pending=0;had_result=0;commit_count=0;previous_request_events=0;
        end else begin
            if(pending)latency=latency+1;
            if(sample_req && n==WINDOW_SIZE && !fault)begin
                if(pending)$fatal(1,"Overlapping calculations");
                if(HOP_EVENTS>0 && previous_request_events!=0 && commit_count-previous_request_events!=HOP_EVENTS)
                    $fatal(1,"Incorrect event hop");
                if(HOP_EVENTS>0 && previous_request_events==0 && commit_count!=WINDOW_SIZE)
                    $fatal(1,"First hop request is not at first full window");
                previous_request_events=commit_count;
                reference_q=0;reference_sum=0;
                for(j=0;j<WINDOW_SIZE;j=j+1)begin
                    term=$signed({1'b0,ring[(wp+j)%WINDOW_SIZE]});
                    reference_q=reference_q+(2*j-WINDOW_SIZE+1)*term;
                    reference_sum=reference_sum+term;
                end
                expected=(2*NUMERATOR+reference_q)/(2*reference_q);
                expected_last=ring[(wp+WINDOW_SIZE-1)%WINDOW_SIZE];
                expected_mean=reference_sum/WINDOW_SIZE;pending=1;latency=0;
            end
            if(commit_delay[1])begin
                ring[wp]=delay1;wp=(wp+1)%WINDOW_SIZE;
                if(n<WINDOW_SIZE)n=n+1;
                commit_count=commit_count+1;
            end
            delay1=delay0;
            if(event_valid)begin
                if(observed_n>=scheduled_n || event_tick!==scheduled[observed_n])
                    $fatal(1,"Independent edge timestamp mismatch got=%0d expected=%0d",event_tick,scheduled[observed_n]);
                observed_n=observed_n+1;delay0=event_tick;events=events+1;
            end
            commit_delay={commit_delay[0],event_valid};
        end
    end
    always @(negedge clk)begin
        if(!rst && valid)begin
            if(!pending || freq!==expected[63:0] || last_tick!==expected_last || mean_tick!==expected_mean)
                $fatal(1,"Full-chain result mismatch got=%0d expected=%0d",freq,expected);
            if(HOP_EVENTS==0 && had_result && event_tick-previous_result_tick!=200000)
                $fatal(1,"Output cadence is not 1 kHz");
            if(latency>105)$fatal(1,"Latency exceeded budget");
            if(latency>max_latency)max_latency=latency;
            $fwrite(fd,"%0d,%0.0f,%0d,%0d,%0d,%0d,%0d\n",epoch,$realtime,freq,expected,last_tick,mean_tick,latency);
            pending=0;had_result=1;previous_result_tick=event_tick;checks=checks+1;epoch_checks=epoch_checks+1;
        end
    end
    initial begin #(1.0e9);$fatal(1,"Watchdog");end
endmodule

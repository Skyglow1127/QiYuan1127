`timescale 1ns/1ps
module tb_player_math #(
    parameter integer CLOCK_HZ = 100000000
);
    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic resetn = 1'b0, start = 1'b0;
    logic [31:0] dt_cycles = '0;
    logic [447:0] q0 = '0, v0 = '0, a0 = '0, j = '0;
    logic busy, done, overflow;
    logic [447:0] q1, v1, a1;
    wire [1343:0] qp0_terms,qp1_terms,qp2_terms,qp3_terms,vp0_terms,vp1_terms,vp2_terms,ap0_terms,ap1_terms;
    logic signed [191:0] c,d,qs,vs,acs,js;
    logic [31:0] input_dt;
    logic [447:0] input_q, input_v, input_a, input_j;
    logic [447:0] expected_q, expected_v, expected_a;
    integer expected_overflow;
    integer descriptor, vector_count, test_index, fields, latency, dummy;
    string vector_path;

`ifdef ROBOT_RT_POST_SYNTH
    player_math
`else
    player_math #(.CLOCK_HZ(CLOCK_HZ))
`endif
    dut(.clk(clk), .resetn(resetn), .start(start), .dt_cycles(dt_cycles),
        .q0(q0), .v0(v0), .a0(a0), .j(j), .busy(busy), .done(done),
        .q1(q1), .v1(v1), .a1(a1), .overflow(overflow),
        .qp0_terms(qp0_terms),.qp1_terms(qp1_terms),.qp2_terms(qp2_terms),.qp3_terms(qp3_terms),
        .vp0_terms(vp0_terms),.vp1_terms(vp1_terms),.vp2_terms(vp2_terms),.ap0_terms(ap0_terms),.ap1_terms(ap1_terms));

    initial begin
        vector_path = "player_vectors.mem";
        dummy = $value$plusargs("VECTORS=%s", vector_path);
        descriptor = $fopen(vector_path, "r");
        if (descriptor == 0) $fatal(1, "Cannot open vectors: %s", vector_path);
        fields = $fscanf(descriptor, "%d\n", vector_count);
        if (fields != 1 || vector_count < 1) $fatal(1, "Invalid vector count");
        // Include the simulator's 100ns global startup reset for netlist runs.
        repeat (20) @(posedge clk);
        @(negedge clk); resetn = 1'b1;
        for (test_index = 0; test_index < vector_count; test_index = test_index + 1) begin
            fields = $fscanf(descriptor, "%h %h %h %h %h %h %h %h %d\n",
                input_dt, input_q, input_v, input_a, input_j, expected_q, expected_v, expected_a, expected_overflow);
            if (fields != 9) $fatal(1, "Malformed vector %0d: fields=%0d", test_index, fields);
            @(negedge clk);
            dt_cycles = input_dt; q0 = input_q; v0 = input_v; a0 = input_a; j = input_j;
            start = 1'b1;
            @(posedge clk); #1;
            if (busy !== 1'b1 || done !== 1'b0) $fatal(1, "Start handshake vector %0d", test_index);
            @(negedge clk); start = 1'b0;
            latency = 0;
            while (done !== 1'b1) begin
                @(posedge clk); #1;
                latency = latency + 1;
                if (latency > 100) $fatal(1, "Timeout vector %0d", test_index);
                if (latency < 78 && busy !== 1'b1) $fatal(1, "Early busy drop vector %0d", test_index);
                // Exercise ignored starts and input latching while the unit is busy.
                if (latency == 10) begin start = 1'b1; dt_cycles = 0; q0 = '1; end
                if (latency == 11) start = 1'b0;
            end
            if (latency != 78 || busy !== 1'b0) $fatal(1, "Latency mismatch vector %0d: %0d", test_index, latency);
            if (overflow !== expected_overflow[0]) $fatal(1, "Overflow vector %0d got=%b expected=%b", test_index, overflow, expected_overflow[0]);
            if (q1 !== expected_q || v1 !== expected_v || a1 !== expected_a) begin
                $display("q got=%h expected=%h", q1, expected_q);
                $display("v got=%h expected=%h", v1, expected_v);
                $display("a got=%h expected=%h", a1, expected_a);
                $fatal(1, "Arithmetic mismatch vector %0d", test_index);
            end
            // Independently verify the new read-only product bus wiring for
            // nonzero signed states too, using full-width testbench products.
            c=CLOCK_HZ;d=$signed({160'd0,input_dt});
            for(integer a=0;a<7;a=a+1)begin
                qs=$signed(input_q[64*a+:64]);vs=$signed(input_v[64*a+:64]);
                acs=$signed(input_a[64*a+:64]);js=$signed(input_j[64*a+:64]);
                if($signed(qp0_terms[192*a+:192])!==qs*192'sd6*c*c*c ||
                   $signed(qp1_terms[192*a+:192])!==vs*192'sd6*d*c*c ||
                   $signed(qp2_terms[192*a+:192])!==acs*192'sd3*d*d*c ||
                   $signed(qp3_terms[192*a+:192])!==js*d*d*d ||
                   $signed(vp0_terms[192*a+:192])!==vs*192'sd2*c*c ||
                   $signed(vp1_terms[192*a+:192])!==acs*192'sd2*d*c ||
                   $signed(vp2_terms[192*a+:192])!==js*d*d ||
                   $signed(ap0_terms[192*a+:192])!==acs*c ||
                   $signed(ap1_terms[192*a+:192])!==js*d)
                    $fatal(1,"Read-only exact term wiring vector=%0d axis=%0d",test_index,a);
            end
            @(posedge clk); #1;
            if (done !== 1'b0) $fatal(1, "done is not a pulse");
        end
        $fclose(descriptor);
`ifdef ROBOT_RT_POST_SYNTH
        $display("PASS player_math vectors=%0d latency=78 cycles clock_hz=%0d; post-synthesis functional simulation", vector_count,CLOCK_HZ);
`else
        $display("PASS player_math vectors=%0d latency=78 cycles clock_hz=%0d; RTL simulation", vector_count,CLOCK_HZ);
`endif
        $finish;
    end
endmodule

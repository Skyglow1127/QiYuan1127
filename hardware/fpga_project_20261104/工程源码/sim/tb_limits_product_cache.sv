`timescale 1ns/1ps
// Synthetic-only integer goldens from make_cache_vectors.py; no hardware data.
// +VECTORS=/new/path/cache.mem and xelab -generic_top CLOCK_HZ=<same integer>.
module tb_limits_product_cache #(
    parameter integer CLOCK_HZ = 100000000,
    parameter integer PUBLISH_EDGE = 38
);
    logic clk=0;
    always #5 clk=~clk;
    logic resetn=0,start=0;
    logic [31:0] cfg_epoch=0;
    logic [447:0] lower=0,upper=0,dq_max=0,velocity_limit=0,acceleration_limit=0;
    wire busy,done,rejected,cache_valid;
    wire [31:0] cached_cfg_epoch;
    wire [1343:0] cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c;
    limits_product_cache #(.CLOCK_HZ(CLOCK_HZ)) dut(.*);
    logic [447:0] in_lo,in_hi,in_dq,in_v,in_a;
    logic [1343:0] exp_lo,exp_hi,exp_dq,exp_v,exp_a;
    logic [31:0] in_epoch;
    integer expected_valid,fd,read_count,jobs=0,checks=0,edge_index;
    string vector_path;
    integer epoch_fault_edge=10,epoch_fault_kind=0;
    integer raw_unknown_group=0,raw_unknown_kind=0;
    integer reset_cases=0,epoch_cases=0,unknown_raw_cases=0,unknown_epoch_cases=0;

    task automatic check(input logic condition,input string message);
        begin
            checks=checks+1;
            if(condition !== 1'b1) $fatal(1,"cache check failed: %s clock=%0d job=%0d",message,CLOCK_HZ,jobs);
        end
    endtask
    task automatic check_zero;
        begin
            check(({cached_lower_qden,cached_upper_qden,cached_dq_qden,cached_vmax_vden,cached_amax_c} === '0),"public products must be zero before publish/on reject");
        end
    endtask
    // mode0 keeps the original numeric vector job. mode1 changes epoch for
    // exactly the selected busy edge (known mismatch, X or Z) then restores.
    // mode2 introduces X/Z in one raw input group at E0; mode3 is X/Z epoch E0.
    task automatic run_job(input integer mode);
        logic wanted_valid;
        begin
            jobs=jobs+1;
            wanted_valid=(expected_valid != 0) && mode==0;
            @(negedge clk);
            start=1;cfg_epoch=in_epoch;
            lower=in_lo;upper=in_hi;dq_max=in_dq;velocity_limit=in_v;acceleration_limit=in_a;
            if(mode==2)begin
                case(raw_unknown_group)
                    0:lower=raw_unknown_kind ? 'z : 'x;
                    1:upper=raw_unknown_kind ? 'z : 'x;
                    2:dq_max=raw_unknown_kind ? 'z : 'x;
                    3:velocity_limit=raw_unknown_kind ? 'z : 'x;
                    4:acceleration_limit=raw_unknown_kind ? 'z : 'x;
                endcase
            end
            if(mode==3)cfg_epoch=epoch_fault_kind==1 ? 32'hxxxxxxxx : 32'hzzzzzzzz;
            @(posedge clk);#1;
            check(busy && !done && !cache_valid,"E0 accepted, never early valid");
            check_zero();
            check(rejected===1'b0&&cached_cfg_epoch===0,"new E0 clears prior verdict and epoch");
            for(edge_index=1;edge_index<=PUBLISH_EDGE;edge_index=edge_index+1) begin
                @(negedge clk);
                start=(edge_index==8 || edge_index==15 || edge_index==PUBLISH_EDGE);
                lower='x;upper='0;dq_max='0;velocity_limit='x;acceleration_limit='0;
                cfg_epoch=in_epoch;
                if(mode==1&&edge_index==epoch_fault_edge)begin
                    case(epoch_fault_kind)
                        0:cfg_epoch=in_epoch+1'b1;
                        1:cfg_epoch=32'hxxxxxxxx;
                        2:cfg_epoch=32'hzzzzzzzz;
                    endcase
                end
                @(posedge clk);#1;
                if(edge_index<PUBLISH_EDGE) begin
                    check(busy && !done && !cache_valid,"all preparation/drain edges busy without publication");
                    check(cached_cfg_epoch===0,"public epoch stays zero before atomic publish");
                    check_zero();
                end else begin
                    check(!busy && done,$sformatf("done exactly E%0d",PUBLISH_EDGE));
                    check(cache_valid === wanted_valid,"valid/reject matches synthetic golden");
                    check(rejected === !wanted_valid,"rejected status");
                    if(wanted_valid) begin
                        check(cached_cfg_epoch===in_epoch,"snapshot epoch published with products");
                        check(cached_lower_qden===exp_lo,"lower exact signed192");
                        check(cached_upper_qden===exp_hi,"upper exact signed192");
                        check(cached_dq_qden===exp_dq,"dq exact signed192");
                        check(cached_vmax_vden===exp_v,"vmax exact signed192");
                        check(cached_amax_c===exp_a,"amax exact signed192");
                    end else begin
                        check(cached_cfg_epoch===0,"reject cannot publish epoch");
                        check_zero();
                    end
                end
            end
            @(negedge clk);start=0;cfg_epoch=in_epoch;
            @(posedge clk);#1;check(!done && !busy,"done is a one-edge pulse and publish-edge start ignored");
            check(cache_valid===wanted_valid,"valid stable after job");
        end
    endtask


    // reset_at0 presents reset and start together at E0; reset_at1..publish
    // first accepts a valid job, then resets BEFORE the specified busy edge.
    task automatic abort_at(input integer reset_edge);
        begin
            @(negedge clk);cfg_epoch=in_epoch;
            lower=in_lo;upper=in_hi;dq_max=in_dq;velocity_limit=in_v;acceleration_limit=in_a;
            start=1;resetn=(reset_edge==0)?1'b0:1'b1;
            if(reset_edge!=0)begin
                @(posedge clk);#1;
                check(busy&&!done&&!cache_valid,"reset case really accepted a fresh valid raw snapshot");check_zero();
                for(integer edge_before=1;edge_before<reset_edge;edge_before=edge_before+1)begin
                    @(negedge clk);start=1;lower='x;upper='z;dq_max=0;velocity_limit=0;acceleration_limit=0;
                    @(posedge clk);#1;
                    check(busy&&!done&&!cache_valid,"reset target not reached/published early");check_zero();
                end
                @(negedge clk);resetn=0;start=1;
            end
            @(posedge clk);#1;
            check(!busy&&!done&&!cache_valid&&!rejected&&cached_cfg_epoch===0,"synchronous reset overrides all stages/publication and start");check_zero();
            @(negedge clk);resetn=1;start=0;cfg_epoch=in_epoch;
            repeat(PUBLISH_EDGE+2)begin
                @(posedge clk);#1;
                check(!busy&&!done&&!cache_valid&&!rejected&&cached_cfg_epoch===0,"reset job cannot publish or restart later");check_zero();
            end
            reset_cases=reset_cases+1;
        end
    endtask

    initial begin
        if(!$value$plusargs("VECTORS=%s",vector_path)) $fatal(1,"supply +VECTORS=synthetic_file");
        repeat(2) @(posedge clk);
        @(negedge clk);resetn=1;
        fd=$fopen(vector_path,"r");
        if(!fd) $fatal(1,"cannot open synthetic vectors");
        while(!$feof(fd)) begin
            read_count=$fscanf(fd,"%d %h %h %h %h %h %h %h %h %h %h %h\n",
                expected_valid,in_epoch,in_lo,in_hi,in_dq,in_v,in_a,exp_lo,exp_hi,exp_dq,exp_v,exp_a);
            if(read_count==12) run_job(0);
            else if(read_count!=-1) $fatal(1,"invalid synthetic vector row count=%0d",read_count);
        end
        $fclose(fd);
        // Reload the original raw first row for protocol adversaries (bad clocks still reject).
        fd=$fopen(vector_path,"r");
        read_count=$fscanf(fd,"%d %h %h %h %h %h %h %h %h %h %h %h\n",
                expected_valid,in_epoch,in_lo,in_hi,in_dq,in_v,in_a,exp_lo,exp_hi,exp_dq,exp_v,exp_a);
        $fclose(fd);
        check(read_count==12,"first synthetic row exists");
        run_job(1); // Mid-job epoch excursion permanently rejects.
        run_job(2); // Unknown raw limits reject, never publish X products.
        run_job(0);
        @(negedge clk);cfg_epoch=in_epoch+1'b1;
        @(posedge clk);#1;check(!cache_valid,"idle epoch change invalidates cache");check_zero();
        @(negedge clk);cfg_epoch=in_epoch;
        @(posedge clk);#1;check(!cache_valid,"old epoch does not resurrect invalidated cache");
        // Every busy edge is tested, including both drain edges and the same
        // publication edge whose current cfg_epoch must be checked directly.
        for(integer kind=0;kind<3;kind=kind+1)begin
            epoch_fault_kind=kind;
            for(integer fault_edge=1;fault_edge<=PUBLISH_EDGE;fault_edge=fault_edge+1)begin
                epoch_fault_edge=fault_edge;run_job(1);epoch_cases=epoch_cases+1;
            end
        end
        for(integer kind=0;kind<2;kind=kind+1)begin
            raw_unknown_kind=kind;
            for(integer group=0;group<5;group=group+1)begin
                raw_unknown_group=group;run_job(2);unknown_raw_cases=unknown_raw_cases+1;
            end
            epoch_fault_kind=kind+1;run_job(3);unknown_epoch_cases=unknown_epoch_cases+1;
        end
        // Every edge E0..publish, including reset priority at final publication.
        for(integer phase=0;phase<=PUBLISH_EDGE;phase=phase+1)abort_at(phase);
        epoch_fault_kind=0;raw_unknown_kind=0;raw_unknown_group=0;
        run_job(0); // A fresh valid job must still work after all reset cases.
        check(reset_cases==PUBLISH_EDGE+1&&epoch_cases==3*PUBLISH_EDGE&&unknown_raw_cases==10&&unknown_epoch_cases==2,"all targeted protocol phases actually exercised");
        $display("PASS limits_product_cache clock=%0d synthetic_jobs=%0d checks=%0d done_edge=%0d reset_edges=%0d epoch_edges=%0d raw_unknown=%0d epoch_unknown_e0=%0d",CLOCK_HZ,jobs,checks,PUBLISH_EDGE,reset_cases,epoch_cases,unknown_raw_cases,unknown_epoch_cases);
        $finish;
    end
endmodule

module tb_cache_actual; tb_limits_product_cache #(.CLOCK_HZ(99999001)) run(); endmodule
module tb_cache_nominal; tb_limits_product_cache #(.CLOCK_HZ(100000000)) run(); endmodule
module tb_cache_maximum; tb_limits_product_cache #(.CLOCK_HZ(2147483647)) run(); endmodule
module tb_cache_zero; tb_limits_product_cache #(.CLOCK_HZ(0)) run(); endmodule
module tb_cache_negative; tb_limits_product_cache #(.CLOCK_HZ(-1)) run(); endmodule

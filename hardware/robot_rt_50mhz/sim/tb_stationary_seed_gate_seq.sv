// Private fallback test only. No HDL execution is implied by this file.
module tb_stationary_seed_gate_seq #(parameter integer CLOCK_HZ=99999001);
    timeunit 1ns; timeprecision 1ps;
    logic clk=0,resetn=0,start=0;
    always #5 clk=~clk;
    logic [447:0] seed_q,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max;
    logic busy,done,allow,clamp,bounds_valid;
    logic [223:0] q16,wire_values;
    stationary_seed_gate_seq #(.CLOCK_HZ(CLOCK_HZ)) dut(.*);
    // A separate frozen view prevents hostile live-input changes affecting the
    // existing combinational reference while the sequential DUT is busy.
    logic [447:0] ref_seed_q,ref_lower,ref_upper,ref_velocity_limit;
    logic [447:0] ref_acceleration_limit,ref_jerk_limit,ref_dq_max;
    wire ref_allow,ref_clamp,ref_bounds_valid;
    wire [223:0] ref_q16,ref_wire_values;
    stationary_seed_gate #(.CLOCK_HZ(CLOCK_HZ)) reference_comb(
        .seed_q(ref_seed_q),.lower(ref_lower),.upper(ref_upper),
        .velocity_limit(ref_velocity_limit),.acceleration_limit(ref_acceleration_limit),
        .jerk_limit(ref_jerk_limit),.dq_max(ref_dq_max),
        .allow(ref_allow),.clamp(ref_clamp),.bounds_valid(ref_bounds_valid),
        .q16(ref_q16),.wire_values(ref_wire_values));
    wire [450:0] response={bounds_valid,allow,clamp,q16,wire_values};
    integer seed_count=0,bounds_count=0,unknown_count=0,independent_count=0;
    integer bounds_unknown_count=0,reset_count=0,completed_jobs=0;
    integer fd,rc,n,row,ea,ec,axis,field,variant,kind;
    logic [223:0] expected_q16,expected_wire;
    logic [896:0] bounds_word;
    string seed_path,bounds_path;
    task automatic legal_inputs;
        begin
            seed_q=0;
            for(integer a=0;a<7;a=a+1)begin
                lower[64*a+:64]=(a==6)?64'sd0:-64'sd42949672960;
                upper[64*a+:64]=(a==6)?64'sd4294967296:64'sd42949672960;
                velocity_limit[64*a+:64]=64'sd4294967296;
                acceleration_limit[64*a+:64]=64'sd4294967296;
                jerk_limit[64*a+:64]=64'sd4294967296;
                dq_max[64*a+:64]=64'sd4294967296;
            end
        end
    endtask
    task automatic freeze_reference;
        begin
            ref_seed_q=seed_q;ref_lower=lower;ref_upper=upper;
            ref_velocity_limit=velocity_limit;ref_acceleration_limit=acceleration_limit;
            ref_jerk_limit=jerk_limit;ref_dq_max=dq_max;
        end
    endtask
    task automatic poison_inputs(input integer edge_number);
        begin
            if(edge_number%2)begin
                seed_q='x;lower='z;upper='x;velocity_limit='z;
                acceleration_limit='x;jerk_limit='z;dq_max='x;
            end else begin
                seed_q='1;lower='0;upper='0;velocity_limit='0;
                acceleration_limit='0;jerk_limit='0;dq_max='0;
            end
        end
    endtask
    task automatic require_cleared(input string label);
        begin
            if(response!==451'd0)$fatal(1,"not atomic/cleared %s response=%h",label,response);
        end
    endtask
    // Negative expectations select the frozen existing comb result; otherwise
    // independent Fraction vectors or explicit fail-closed checks supply them.
    task automatic run_job(input integer expected_allow,input integer expected_clamp,
        input integer expected_bounds,input logic[223:0] expected_q,
        input logic[223:0] expected_w,input string label);
        logic[450:0] wanted,comb;
        begin
            @(negedge clk);freeze_reference();#1;
            comb={ref_bounds_valid,ref_allow,ref_clamp,ref_q16,ref_wire_values};
            wanted={(expected_bounds<0)?ref_bounds_valid:expected_bounds[0],
                    (expected_allow<0)?ref_allow:expected_allow[0],
                    (expected_clamp<0)?ref_clamp:expected_clamp[0],
                    (expected_allow<0)?ref_q16:expected_q,
                    (expected_allow<0)?ref_wire_values:expected_w};
            if(comb!==wanted)$fatal(1,"existing comb != independent expectation %s",label);
            if(busy!==0||done!==0)$fatal(1,"not idle before new job %s",label);
            start=1;
            @(posedge clk);#1;
            if(busy!==1||done!==0)$fatal(1,"E0 accept failed %s",label);
            require_cleared({label," E0 old-result invalidation"});
            for(integer e=1;e<=35;e=e+1)begin
                @(negedge clk);poison_inputs(e);start=e[0];
                @(posedge clk);#1;
                if(busy!==1||done!==0)$fatal(1,"early done/not busy E%0d %s",e,label);
                require_cleared(label);
            end
            // E36 still starts busy; asserted start must not accept a new job.
            @(negedge clk);poison_inputs(37);start=1;
            @(posedge clk);#1;
            if(busy!==0||done!==1||response!==wanted)
                $fatal(1,"E36 mismatch %s busy=%b done=%b got=%h wanted=%h",label,busy,done,response,wanted);
            for(integer hold_edge=0;hold_edge<2;hold_edge=hold_edge+1)begin
                @(negedge clk);start=0;poison_inputs(hold_edge);
                @(posedge clk);#1;
                if(busy!==0||done!==0||response!==wanted)
                    $fatal(1,"done pulse/idle holding/ignored E36 start %s",label);
            end
            completed_jobs=completed_jobs+1;
        end
    endtask
    task automatic reset_at_edge(input integer cancel_edge);
        begin
            @(negedge clk);legal_inputs();start=1;
            if(cancel_edge==0)resetn=0;
            @(posedge clk);#1;
            if(cancel_edge!=0)begin
                if(busy!==1||done!==0)$fatal(1,"reset test did not start");
                for(integer e=1;e<cancel_edge;e=e+1)begin
                    @(negedge clk);poison_inputs(e);start=1;
                    @(posedge clk);#1;
                    if(busy!==1||done!==0)$fatal(1,"reset test early completion E%0d",e);
                    require_cleared("reset pending job");
                end
                @(negedge clk);resetn=0;start=1;
                @(posedge clk);#1;
            end
            if(busy!==0||done!==0)$fatal(1,"synchronous reset failed edge%0d",cancel_edge);
            require_cleared("reset edge");
            @(negedge clk);resetn=1;start=0;poison_inputs(1);
            repeat(40)begin
                @(posedge clk);#1;
                if(busy!==0||done!==0)$fatal(1,"aborted job reappeared edge%0d",cancel_edge);
                require_cleared("after reset");
            end
            reset_count=reset_count+1;
        end
    endtask
    initial begin
        legal_inputs();freeze_reference();
        repeat(2)@(posedge clk);
        #1;if(busy!==0||done!==0)$fatal(1,"initial reset control");require_cleared("initial reset");
        @(negedge clk);resetn=1;
        if(!$value$plusargs("SEED_VECTORS=%s",seed_path)||
           !$value$plusargs("BOUNDS_VECTORS=%s",bounds_path))
            $fatal(1,"explicit +SEED_VECTORS and +BOUNDS_VECTORS required");
        fd=$fopen(seed_path,"r");if(!fd)$fatal(1,"cannot read seed vectors");
        rc=$fscanf(fd,"%d\n",n);if(rc!=1||n!=1406)$fatal(1,"expected original 1406 seed vectors");
        for(row=0;row<n;row=row+1)begin
            rc=$fscanf(fd,"%h %h %h %h %h %h %h %d %d %h %h\n",
                seed_q,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,
                ea,ec,expected_q16,expected_wire);
            if(rc!=11)$fatal(1,"bad seed row %0d",row);
            run_job(ea,ec,-1,expected_q16,expected_wire,$sformatf("seed %0d",row));
            seed_count=seed_count+1;
        end
        $fclose(fd);
        fd=$fopen(bounds_path,"r");if(!fd)$fatal(1,"cannot read bounds vectors");
        while(!$feof(fd))begin
            rc=$fscanf(fd,"%h\n",bounds_word);
            if(rc==1)begin
                legal_inputs();lower=bounds_word[447:0];upper=bounds_word[895:448];
                run_job(-1,-1,{31'd0,bounds_word[896]},0,0,$sformatf("bounds %0d",bounds_count));
                bounds_count=bounds_count+1;
            end else if(!$feof(fd))$fatal(1,"malformed bounds vector");
        end
        $fclose(fd);if(bounds_count!=3367)$fatal(1,"expected original 3367 bounds vectors");
        for(kind=0;kind<2;kind=kind+1)
        for(field=0;field<7;field=field+1)
        for(axis=0;axis<7;axis=axis+1)begin
            legal_inputs();
            case(field)
                0:seed_q[64*axis]=(kind==0)?1'bx:1'bz;
                1:lower[64*axis]=(kind==0)?1'bx:1'bz;
                2:upper[64*axis]=(kind==0)?1'bx:1'bz;
                3:velocity_limit[64*axis]=(kind==0)?1'bx:1'bz;
                4:acceleration_limit[64*axis]=(kind==0)?1'bx:1'bz;
                5:jerk_limit[64*axis]=(kind==0)?1'bx:1'bz;
                6:dq_max[64*axis]=(kind==0)?1'bx:1'bz;
            endcase
            run_job(0,0,(field==1||field==2)?0:1,0,0,"single X/Z full input");
            unknown_count=unknown_count+1;
        end
        for(axis=0;axis<7;axis=axis+1)begin
            for(field=0;field<5;field=field+1)
            for(variant=0;variant<3;variant=variant+1)begin
                legal_inputs();
                case(field)
                    0:seed_q[64*axis+:64]=(variant==0)?64'd0:(variant==1)?64'bx:64'bz;
                    1:velocity_limit[64*axis+:64]=(variant==0)?64'd0:(variant==1)?64'bx:64'bz;
                    2:acceleration_limit[64*axis+:64]=(variant==0)?64'd0:(variant==1)?64'bx:64'bz;
                    3:jerk_limit[64*axis+:64]=(variant==0)?64'd0:(variant==1)?64'bx:64'bz;
                    4:dq_max[64*axis+:64]=(variant==0)?64'd0:(variant==1)?64'bx:64'bz;
                endcase
                if(field==0&&variant==0)run_job(-1,-1,1,0,0,"bounds independent legal zero seed");
                else run_job(0,0,1,0,0,"bounds independent invalid seed/limits");
                independent_count=independent_count+1;
            end
            legal_inputs();seed_q[64*axis+:64]=64'h8000000000000000;
            run_job(0,0,1,0,0,"bounds independent int64-min seed");
            independent_count=independent_count+1;
        end
        for(axis=0;axis<7;axis=axis+1)
        for(field=0;field<2;field=field+1)
        for(variant=0;variant<3;variant=variant+1)begin
            legal_inputs();
            if(field==0)begin
                if(variant==0)lower[64*axis]=1'bx;
                else if(variant==1)lower[64*axis+63]=1'bx;
                else lower[64*axis+:64]=64'bz;
            end else begin
                if(variant==0)upper[64*axis]=1'bx;
                else if(variant==1)upper[64*axis+63]=1'bx;
                else upper[64*axis+:64]=64'bz;
            end
            run_job(0,0,0,0,0,"bounds low/high X/Z fail closed");
            bounds_unknown_count=bounds_unknown_count+1;
        end
        for(integer cancel_edge=0;cancel_edge<=36;cancel_edge=cancel_edge+1)reset_at_edge(cancel_edge);
        legal_inputs();run_job(-1,-1,1,0,0,"fresh success after all resets");
        if(seed_count!=1406||bounds_count!=3367||unknown_count!=98||
           independent_count!=112||bounds_unknown_count!=42||reset_count!=37||completed_jobs!=5026)
            $fatal(1,"test accounting mismatch");
        $display("PASS stationary_seed_seq seed=%0d bounds=%0d unknown=%0d independent=%0d bounds_unknown=%0d reset=%0d jobs=%0d done_edge=36 clock=%0d",
            seed_count,bounds_count,unknown_count,independent_count,bounds_unknown_count,reset_count,completed_jobs,CLOCK_HZ);
        $finish;
    end
    initial begin #10000000;$fatal(1,"sequential seed TB timeout");end
endmodule

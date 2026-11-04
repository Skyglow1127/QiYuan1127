`timescale 1ns/1ps
// Independent Fraction vectors plus direct X/Z and input-independence checks.
// Prepared only; an HDL result exists only after an actual root-controlled run.
module tb_seed_bounds #(parameter integer CLOCK_HZ=100000000);
    logic [447:0] seed_q,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max;
    logic allow,clamp,bounds_valid;
    logic [223:0] q16,wire_values;
    stationary_seed_gate #(.CLOCK_HZ(CLOCK_HZ)) dut(.*);
    logic [896:0] vector_word;
    string vectors;
    integer fd,rc,axis,field,variant,binary_vectors,independence_checks,unknown_checks;
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
    task automatic require_independent(input logic expected_allow);
        begin
            #1;
            if(bounds_valid!==1'b1||allow!==expected_allow)
                $fatal(1,"independence axis%0d field%0d variant%0d bounds%0b allow%0b expected%0b",axis,field,variant,bounds_valid,allow,expected_allow);
            if(!expected_allow&&(clamp!==1'b0||q16!==224'd0||wire_values!==224'd0))
                $fatal(1,"invalid seed/derivative did not fail closed");
            independence_checks=independence_checks+1;
        end
    endtask
    initial begin
        binary_vectors=0;independence_checks=0;unknown_checks=0;
        if(!$value$plusargs("VECTORS=%s",vectors))$fatal(1,"explicit +VECTORS path required");
        fd=$fopen(vectors,"r");if(!fd)$fatal(1,"cannot read vectors");
        legal_inputs();
        while(!$feof(fd))begin
            rc=$fscanf(fd,"%h\n",vector_word);
            if(rc==1)begin
                lower=vector_word[447:0];upper=vector_word[895:448];#1;
                if(bounds_valid!==vector_word[896])$fatal(1,"Fraction vector%0d mismatch expected%0b got%0b",binary_vectors,vector_word[896],bounds_valid);
                binary_vectors=binary_vectors+1;
            end else if(!$feof(fd))$fatal(1,"malformed bounds vector file");
        end
        $fclose(fd);if(binary_vectors<2000)$fatal(1,"too few directed/random vectors");
        // Pure bounds output must not depend on q, v, a, j, or DQ. Zero q is
        // legal here, while X/Z/out-of-range q and nonpositive limits reject
        // the original allow output without contaminating bounds_valid.
        for(axis=0;axis<7;axis=axis+1)begin
            for(field=0;field<5;field=field+1)begin
                for(variant=0;variant<3;variant=variant+1)begin
                    legal_inputs();
                    case(field)
                        0:seed_q[64*axis+:64]=(variant==0)?64'd0:(variant==1)?64'bx:64'bz;
                        1:velocity_limit[64*axis+:64]=(variant==0)?64'd0:(variant==1)?64'bx:64'bz;
                        2:acceleration_limit[64*axis+:64]=(variant==0)?64'd0:(variant==1)?64'bx:64'bz;
                        3:jerk_limit[64*axis+:64]=(variant==0)?64'd0:(variant==1)?64'bx:64'bz;
                        4:dq_max[64*axis+:64]=(variant==0)?64'd0:(variant==1)?64'bx:64'bz;
                    endcase
                    require_independent(field==0&&variant==0);
                end
            end
            legal_inputs();seed_q[64*axis+:64]=64'h8000000000000000;require_independent(1'b0);
        end
        // Unknown low fractions, discarded high bits and Z bounds all reject.
        for(axis=0;axis<7;axis=axis+1)begin
            for(field=0;field<2;field=field+1)begin
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
                    #1;
                    if(bounds_valid!==1'b0||allow!==1'b0||clamp!==1'b0||q16!==224'd0||wire_values!==224'd0)
                        $fatal(1,"unknown bounds not rejected axis%0d field%0d variant%0d",axis,field,variant);
                    unknown_checks=unknown_checks+1;
                end
            end
        end
        $display("PASS seed_bounds binary_vectors=%0d independence_checks=%0d unknown_checks=%0d clock_hz=%0d",binary_vectors,independence_checks,unknown_checks,CLOCK_HZ);
        $finish;
    end
    initial begin #100000;$fatal(1,"seed bounds timeout");end
endmodule

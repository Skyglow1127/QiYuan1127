`timescale 1ns/1ps
module tb_stationary_seed_gate #(
    parameter integer CLOCK_HZ=99999001
);
    logic [447:0] seed_q,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max;
    wire allow,clamp,golden_allow,golden_clamp;
    wire [223:0] q16,wire_values,golden_q16,golden_wire;
    logic bounds_valid; // New pure bounds output; original TB assertions unchanged.
    stationary_seed_gate #(.CLOCK_HZ(CLOCK_HZ)) dut(.*);
    limit_gate #(.CLOCK_HZ(CLOCK_HZ),.STATIONARY(1)) golden(
        .q0(seed_q),.q1(seed_q),.v0(448'd0),.a0(448'd0),.j(448'd0),
        .v1(448'd0),.a1(448'd0),.dt_cycles(32'd1),
        .lower(lower),.upper(upper),.velocity_limit(velocity_limit),
        .acceleration_limit(acceleration_limit),.jerk_limit(jerk_limit),.dq_max(dq_max),
        .previous_wire(224'd0),.previous_wire_valid(1'b0),
        .allow(golden_allow),.clamp(golden_clamp),.q16(golden_q16),.wire_values(golden_wire));
    integer fd,n,status,index,ea,ec,axis,field,kind;
    integer checks=0, unknown_checks=0, legacy_unknown_allow=0;
    logic [223:0] eq16,ewire;
    string path;
    task automatic set_base;
        integer a;
        begin
            for(a=0;a<7;a=a+1) begin
                seed_q[64*a+:64]=64'd0;
                lower[64*a+:64]=(a==6)?64'sd0:-64'sd42949672960;
                upper[64*a+:64]=(a==6)?64'sd429496729:64'sd42949672960;
                velocity_limit[64*a+:64]=64'sd4294967296;
                acceleration_limit[64*a+:64]=64'sd4294967296;
                jerk_limit[64*a+:64]=64'sd4294967296;
                dq_max[64*a+:64]=64'sd1;
            end
        end
    endtask
    initial begin
        path="seed_vectors.mem";
        if($value$plusargs("VECTORS=%s",path)) begin end
        fd=$fopen(path,"r"); if(!fd) $fatal(1,"cannot open vectors %s",path);
        status=$fscanf(fd,"%d\n",n); if(status!=1 || n<1) $fatal(1,"bad vector count");
        for(index=0;index<n;index=index+1) begin
            status=$fscanf(fd,"%h %h %h %h %h %h %h %d %d %h %h\n",
                seed_q,lower,upper,velocity_limit,acceleration_limit,jerk_limit,dq_max,
                ea,ec,eq16,ewire);
            if(status!=11) $fatal(1,"bad vector row %0d fields=%0d",index,status);
            #1;
            if({allow,clamp,q16,wire_values} !== {ea[0],ec[0],eq16,ewire})
                $fatal(1,"candidate mismatch row=%0d allow=%b/%b clamp=%b/%b q=%h/%h wire=%h/%h",
                    index,allow,ea[0],clamp,ec[0],q16,eq16,wire_values,ewire);
            if({golden_allow,golden_clamp,golden_q16,golden_wire} !== {ea[0],ec[0],eq16,ewire})
                $fatal(1,"baseline mismatch row=%0d",index);
            checks=checks+1;
        end
        $fclose(fd);
        // Exact equivalence is asserted only on binary inputs. Legacy RTL can
        // allow X/Z because its if(comparison) skips unknown conditions. The
        // candidate deliberately rejects any X/Z bit, including raw low bits.
        for(kind=0;kind<2;kind=kind+1)
        for(field=0;field<7;field=field+1)
        for(axis=0;axis<7;axis=axis+1) begin
            set_base();
            case(field)
                0:seed_q[64*axis]=(kind==0)?1'bx:1'bz;
                1:lower[64*axis]=(kind==0)?1'bx:1'bz;
                2:upper[64*axis]=(kind==0)?1'bx:1'bz;
                3:velocity_limit[64*axis]=(kind==0)?1'bx:1'bz;
                4:acceleration_limit[64*axis]=(kind==0)?1'bx:1'bz;
                5:jerk_limit[64*axis]=(kind==0)?1'bx:1'bz;
                6:dq_max[64*axis]=(kind==0)?1'bx:1'bz;
            endcase
            #1;
            if({allow,clamp,q16,wire_values} !== 450'd0)
                $fatal(1,"unknown not fail-closed kind=%0d field=%0d axis=%0d",kind,field,axis);
            if(golden_allow === 1'b1) legacy_unknown_allow=legacy_unknown_allow+1;
            unknown_checks=unknown_checks+1;
        end
        $display("PASS stationary_seed binary_vectors=%0d unknown_checks=%0d legacy_unknown_allow=%0d clock=%0d",
            checks,unknown_checks,legacy_unknown_allow,CLOCK_HZ);
        $finish;
    end
endmodule

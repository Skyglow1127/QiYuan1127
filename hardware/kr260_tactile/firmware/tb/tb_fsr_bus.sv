`timescale 1ns / 1ps
`default_nettype none

// Integration regression: use the generated reset and AXI GPIO IP, not models.
// In particular, aux_reset_in=0 must be inactive. The former generated
// C_AUX_RESET_HIGH=0 configuration fails the first reset-release deadline.
module tb_fsr_bus;
    localparam integer EXPECTED_CHECKS = 44;
    localparam integer EXPECTED_READS = 16;
    localparam integer AXI_TIMEOUT_CYCLES = 100;
    localparam integer RESET_TIMEOUT_CYCLES = 256;

    reg clk = 1'b0;
    always #20 clk = ~clk; // Actual design clock: 25 MHz.

    reg ext_resetn = 1'b0;
    reg [1:0] din = 2'b00;
    wire [1:0] synced;
    wire [0:0] peripheral_aresetn;
    wire [0:0] interconnect_aresetn;
    wire [0:0] peripheral_reset;
    wire [0:0] bus_struct_reset;
    wire mb_reset;

    reg [8:0] awaddr = 9'h000;
    reg awvalid = 1'b0;
    wire awready;
    reg [31:0] wdata = 32'h00000000;
    reg [3:0] wstrb = 4'h0;
    reg wvalid = 1'b0;
    wire wready;
    wire [1:0] bresp;
    wire bvalid;
    reg bready = 1'b0;
    reg [8:0] araddr = 9'h000;
    reg arvalid = 1'b0;
    wire arready;
    wire [31:0] rdata;
    wire [1:0] rresp;
    wire rvalid;
    reg rready = 1'b0;

    integer checks = 0;
    integer axi_reads = 0;
    integer reset_releases = 0;
    integer epoch;
    integer sample;
    reg [1:0] expected_do;

    fsr_bd_reset_i_0 reset_i (
        .slowest_sync_clk(clk),
        .ext_reset_in(ext_resetn),
        .aux_reset_in(1'b0),
        .mb_debug_sys_rst(1'b0),
        .dcm_locked(1'b1),
        .mb_reset(mb_reset),
        .bus_struct_reset(bus_struct_reset),
        .peripheral_reset(peripheral_reset),
        .interconnect_aresetn(interconnect_aresetn),
        .peripheral_aresetn(peripheral_aresetn)
    );

    fsr_sync2 sync_i (
        .clk(clk), .resetn(peripheral_aresetn[0]),
        .din(din), .dout(synced)
    );

    fsr_bd_fsr_gpio_0 gpio_i (
        .s_axi_aclk(clk), .s_axi_aresetn(peripheral_aresetn[0]),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid),
        .s_axi_awready(awready), .s_axi_wdata(wdata),
        .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid),
        .s_axi_wready(wready), .s_axi_bresp(bresp),
        .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid),
        .s_axi_arready(arready), .s_axi_rdata(rdata),
        .s_axi_rresp(rresp), .s_axi_rvalid(rvalid),
        .s_axi_rready(rready), .gpio_io_i(synced)
    );

    task automatic check(input logic condition, input string description);
        begin
            if (condition !== 1'b1)
                $fatal(1, "FSR_BUS check %0d failed: %s at %0t",
                       checks + 1, description, $time);
            checks = checks + 1;
        end
    endtask

    task automatic check_reset_asserted;
        integer cycles;
        begin
            cycles = 0;
            while ((peripheral_aresetn[0] !== 1'b0 ||
                    interconnect_aresetn[0] !== 1'b0) &&
                   cycles < RESET_TIMEOUT_CYCLES) begin
                @(posedge clk);
                #1;
                cycles = cycles + 1;
            end
            // Allow the synchronizer's synchronous reset to take effect.
            repeat (8) @(posedge clk);
            #1;
            check(peripheral_aresetn[0] === 1'b0,
                  "peripheral reset asserted with ext_resetn low");
            check(interconnect_aresetn[0] === 1'b0,
                  "interconnect reset asserted with ext_resetn low");
            check(synced === 2'b00, "CDC registers cleared during reset");
        end
    endtask

    task automatic check_reset_released;
        integer cycles;
        begin
            cycles = 0;
            while ((peripheral_aresetn[0] !== 1'b1 ||
                    interconnect_aresetn[0] !== 1'b1) &&
                   cycles < RESET_TIMEOUT_CYCLES) begin
                @(posedge clk);
                #1;
                cycles = cycles + 1;
            end
            check(peripheral_aresetn[0] === 1'b1,
                  "peripheral reset releases within 256 cycles, aux=0");
            check(interconnect_aresetn[0] === 1'b1,
                  "interconnect reset releases within 256 cycles, aux=0");
            reset_releases = reset_releases + 1;
        end
    endtask

    // Linux's GPIO driver writes the direction register at offset 0x4.
    // The actual IP is built ALL_INPUTS=1 and exposes no GPIO output ports.
    task automatic write_tri_inputs;
        integer cycles;
        bit aw_done;
        bit w_done;
        bit response_seen;
        begin
            cycles = 0;
            aw_done = 0;
            w_done = 0;
            response_seen = 0;
            @(negedge clk);
            awaddr = 9'h004;
            awvalid = 1'b1;
            wdata = 32'hffffffff;
            wstrb = 4'hf;
            wvalid = 1'b1;
            bready = 1'b0;
            while (!(aw_done && w_done) && cycles < AXI_TIMEOUT_CYCLES) begin
                @(posedge clk);
                cycles = cycles + 1;
                if (awvalid === 1'b1 && awready === 1'b1) aw_done = 1;
                if (wvalid === 1'b1 && wready === 1'b1) w_done = 1;
                @(negedge clk);
                if (aw_done) awvalid = 1'b0;
                if (w_done) wvalid = 1'b0;
            end
            if (!(aw_done && w_done))
                $fatal(1, "FSR_BUS AXI write address/data timeout");
            // BREADY stays low until BRESP has been inspected, preventing
            // an early response from being consumed between task phases.
            while (!response_seen && cycles < AXI_TIMEOUT_CYCLES) begin
                @(posedge clk);
                cycles = cycles + 1;
                if (bvalid === 1'b1) response_seen = 1;
            end
            if (!response_seen)
                $fatal(1, "FSR_BUS AXI write response timeout");
            check(bresp === 2'b00, "TRI write returns AXI OKAY");
            @(negedge clk);
            bready = 1'b1;
            @(posedge clk);
            @(negedge clk);
            bready = 1'b0;
        end
    endtask

    task automatic read_gpio(input logic [1:0] expected);
        integer cycles;
        bit address_seen;
        bit response_seen;
        begin
            cycles = 0;
            address_seen = 0;
            response_seen = 0;
            @(negedge clk);
            araddr = 9'h000;
            arvalid = 1'b1;
            rready = 1'b0;
            while (!address_seen && cycles < AXI_TIMEOUT_CYCLES) begin
                @(posedge clk);
                cycles = cycles + 1;
                if (arready === 1'b1) address_seen = 1;
            end
            if (!address_seen)
                $fatal(1, "FSR_BUS AXI read address timeout, read=%0d",
                       axi_reads + 1);
            @(negedge clk);
            arvalid = 1'b0;
            while (!response_seen && cycles < AXI_TIMEOUT_CYCLES) begin
                @(posedge clk);
                cycles = cycles + 1;
                if (rvalid === 1'b1) response_seen = 1;
            end
            if (!response_seen)
                $fatal(1, "FSR_BUS AXI read response timeout, read=%0d",
                       axi_reads + 1);
            check(rresp === 2'b00, "DATA read returns AXI OKAY");
            check(rdata === {30'b0, expected},
                  $sformatf("DATA read expected=%08x actual=%08x",
                            {30'b0, expected}, rdata));
            @(negedge clk);
            rready = 1'b1;
            @(posedge clk);
            @(negedge clk);
            rready = 1'b0;
            axi_reads = axi_reads + 1;
        end
    endtask

    initial begin
        for (epoch = 0; epoch < 2; epoch = epoch + 1) begin
            check_reset_asserted();
            @(negedge clk);
            ext_resetn = 1'b1;
            check_reset_released();
            write_tri_inputs();
            for (sample = 0; sample < 8; sample = sample + 1) begin
                expected_do = (epoch == 0) ? (sample % 4) : (3 - sample % 4);
                @(negedge clk);
                din = expected_do;
                repeat (6) @(posedge clk);
                #1;
                read_gpio(expected_do);
            end
            if (epoch == 0) begin
                @(negedge clk);
                // Keep a nonzero external input during the second reset.
                din = 2'b11;
                ext_resetn = 1'b0;
            end
        end
        if (checks != EXPECTED_CHECKS || axi_reads != EXPECTED_READS ||
            reset_releases != 2)
            $fatal(1, "FSR_BUS incomplete checks=%0d reads=%0d releases=%0d",
                   checks, axi_reads, reset_releases);
        $display("TEST_PASS: FSR_BUS checks=%0d reset_releases=%0d axi_reads=%0d",
                 checks, reset_releases, axi_reads);
        $finish;
    end

    initial begin
        #200000;
        $fatal(1, "FSR_BUS global watchdog expired");
    end
endmodule
`default_nettype wire

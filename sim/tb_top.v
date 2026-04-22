`timescale 1ns / 1ps

module tb_top;

    // Simulation Parameters
    localparam integer CLK_PERIOD_NS = 20; // 50MHz
    localparam integer SIM_SEC_CYCLES_50M = 10_000; // 50MHz cycles per sim second
    localparam integer SIM_SEC_CYCLES_100M = SIM_SEC_CYCLES_50M * 2; // 100MHz cycles per sim second
    
    // Derived real time per sim second = 10000 * 40ns = 400us
    
    reg clk;
    reg rst_n;
    reg gps_pps;
    
    wire uart_rx = 1'b1;
    wire uart_tx;
    wire dpps_out;
    wire spi_sclk;
    wire spi_mosi;
    wire spi_miso;
    wire spi_cs_n;

    // Clock Generation (50MHz)
    initial begin
        clk = 0;
        forever #(CLK_PERIOD_NS/2) clk = ~clk;
    end

    // Reset Generation
    initial begin
        rst_n = 0;
        #1000; // 1us
        rst_n = 1;
    end

    // Instantiate TOP
    top #(
        .GPS_LOST_THRESHOLD(SIM_SEC_CYCLES_50M * 2), // 2 seconds threshold
        .CLK_CYCLES_PER_SEC(SIM_SEC_CYCLES_50M)     // 50MHz cycles per second
    ) u_top (
        .clk(clk),
        .rst_n(rst_n),
        .gps_pps(gps_pps),
        .spi_miso(spi_miso),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .dpps_out(dpps_out),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_cs_n(spi_cs_n)
    );

    // Flash Model
    spi_flash_model #(
        .MEM_BYTES(65536)
    ) u_flash_model (
        .sys_clk(clk),
        .cs_n(spi_cs_n),
        .sclk(spi_sclk),
        .mosi(spi_mosi),
        .miso(spi_miso)
    );

    // PPS Generation (Ideal)
    reg ideal_pps;
    integer sec_cnt;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sec_cnt <= 0;
            ideal_pps <= 0;
        end else begin
            if (sec_cnt >= SIM_SEC_CYCLES_50M - 1) begin
                sec_cnt <= 0;
                ideal_pps <= 1;
            end else begin
                sec_cnt <= sec_cnt + 1;
                ideal_pps <= 0;
            end
        end
    end

    // GPS PPS Control (Simulate Loss)
    reg gps_enable;
    always @(*) begin
        gps_pps = gps_enable ? ideal_pps : 1'b0;
    end

    // Test Sequence
    initial begin
        gps_enable = 1;
        
        // Wait for lock and initial training
        $display("Waiting for reset...");
        wait(rst_n == 1);
        
        $display("System running...");
        
        // Wait for a few seconds (sim seconds)
        repeat(5) @(posedge ideal_pps);
        $display("PPS active, observing behavior...");
        
        // Simulate GPS Loss
        $display("Simulating GPS Loss...");
        gps_enable = 0;
        
        repeat(5) @(posedge ideal_pps);
        
        $display("Simulating GPS Recovery...");
        gps_enable = 1;
        
        repeat(5) @(posedge ideal_pps);
        
        $finish;
    end
    
    // Monitor
    initial begin
        $monitor("Time=%t rst=%b gps=%b dpps=%b state=%h", $time, rst_n, gps_pps, dpps_out, u_top.sys_state);
    end

endmodule

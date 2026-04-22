// SPI Flash功能验证Testbench
// 测试功能：验证timekeeping_core生成的差值数据通过FIFO写入SPI Flash的完整流程
// 设计作者：TraeAI
// 设计日期：2026-01-19
// 版本号：v1.0
//
// 测试组件：
// 1. 系统时钟和复位生成
// 2. GPS PPS信号生成
// 3. DPPS偏移量序列生成
// 4. timekeeping_core实例化
// 5. fifo实例化
// 6. flash_spi实例化
// 7. SPI Flash模型模拟
// 8. 测试验证逻辑
//
// 测试场景：
// 1. GPS PPS生成（64个脉冲）
// 2. DPPS偏移量变化（0→2→3）
// 3. 差值数据生成和FIFO写入
// 4. FIFO数据读取和Flash写入
// 5. Flash擦除、编程和状态查询
// 6. 数据完整性验证
//
// 设计说明：
// 1. 系统时钟周期：100ns（10MHz）
// 2. GPS PPS周期：50个时钟周期
// 3. Flash模型容量：4096字节
// 4. 支持自动化测试验证
// 5. 包含详细的测试日志输出
`timescale 1ns / 1ps

module tb_flash_spi;
    localparam integer CLK_PERIOD_NS = 100;
    localparam integer GPS_PPS_PERIOD_CYCLES = 50;
    localparam integer GPS_PPS_PULSES = 64;

    reg sys_clk;
    reg sys_rst_n;

    reg collect_stop;

    wire fifo_re;
    wire [7:0] fifo_dout;
    wire fifo_empty_flag;
    wire fifo_full_flag;

    wire spi_sclk;
    wire spi_mosi;
    wire spi_miso;
    wire spi_cs_n;

    wire flash_busy;
    wire flash_done;
    wire flash_error;
    wire [7:0] hist_diff;
    wire hist_diff_valid;

    localparam [23:0] DIFF_ADDR_START = 24'h001000;

    flash_spi dut (
        .sys_clk         (sys_clk),
        .sys_rst_n       (sys_rst_n),
        .collect_stop    (collect_stop),
        .fifo_re         (fifo_re),
        .fifo_dout       (fifo_dout),
        .fifo_empty_flag (fifo_empty_flag),
        .fifo_full_flag  (fifo_full_flag),
        .spi_sclk        (spi_sclk),
        .spi_mosi        (spi_mosi),
        .spi_miso        (spi_miso),
        .spi_cs_n        (spi_cs_n),
        .flash_busy      (flash_busy),
        .flash_done      (flash_done),
        .flash_error     (flash_error),
        .hist_diff       (hist_diff),
        .hist_diff_valid (hist_diff_valid)
    );

    reg gps_pps;
    reg [23:0] dpps_offset;

    wire dpps;
    wire diff_sign;
    wire [6:0] diff_data;
    wire diff_valid_flag;

    timekeeping_core #(
        .CLK_CYCLES_PER_SEC(GPS_PPS_PERIOD_CYCLES)
    ) u_timekeeping_core (
        .sys_clk_10mhz    (sys_clk),
        .sys_rst_n        (sys_rst_n),
        .gps_pps          (gps_pps),
        .dpps_offset      (dpps_offset),
        .init_diff        (hist_diff),
        .init_diff_valid  (hist_diff_valid),
        .dpps             (dpps),
        .diff_sign        (diff_sign),
        .diff_data        (diff_data),
        .diff_valid_flag  (diff_valid_flag)
    );

    fifo_write u_fifo (
        .sys_clk_10mhz    (sys_clk),
        .sys_rst_n        (sys_rst_n),
        .diff_sign        (diff_sign),
        .diff_data        (diff_data),
        .diff_valid_flag  (diff_valid_flag),
        .fifo_re          (fifo_re),
        .fifo_dout        (fifo_dout),
        .fifo_empty_flag  (fifo_empty_flag),
        .fifo_full_flag   (fifo_full_flag)
    );

    spi_flash_model #(
        .MEM_BYTES(65536)
    ) u_flash (
        .sys_clk (sys_clk),
        .cs_n    (spi_cs_n),
        .sclk    (spi_sclk),
        .mosi    (spi_mosi),
        .miso    (spi_miso)
    );

    initial begin
        sys_clk = 1'b0;
        forever #(CLK_PERIOD_NS/2) sys_clk = ~sys_clk;
    end

    initial begin
        sys_rst_n = 1'b0;
        collect_stop = 1'b0;
        gps_pps = 1'b0;
        dpps_offset = 24'd0;
        repeat (10) @(posedge sys_clk);
        sys_rst_n = 1'b1;
    end

    integer gps_pps_cnt;
    integer gps_pps_pulses_sent;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            gps_pps <= 1'b0;
            gps_pps_cnt <= 0;
            gps_pps_pulses_sent <= 0;
        end else begin
            gps_pps <= 1'b0;
            if (gps_pps_pulses_sent < GPS_PPS_PULSES) begin
                if (gps_pps_cnt == (GPS_PPS_PERIOD_CYCLES - 1)) begin
                    gps_pps_cnt <= 0;
                    gps_pps <= 1'b1;
                    gps_pps_pulses_sent <= gps_pps_pulses_sent + 1;
                end else begin
                    gps_pps_cnt <= gps_pps_cnt + 1;
                end
            end
        end
    end

    initial begin
        dpps_offset = 24'd0;
        wait (sys_rst_n == 1'b1);
        repeat (300) @(posedge sys_clk);
        dpps_offset = 24'd2;
        repeat (300) @(posedge sys_clk);
        dpps_offset = 24'd3;
    end

    localparam integer MAX_CAPTURE = 256;
    reg [7:0] cap_mem [0:MAX_CAPTURE-1];
    integer cap_count;
    reg fifo_read_pend_d;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            cap_count <= 0;
            fifo_read_pend_d <= 1'b0;
        end else begin
            fifo_read_pend_d <= dut.fifo_read_pend;
            if (fifo_read_pend_d && !dut.fifo_read_pend) begin
                if (cap_count < MAX_CAPTURE) begin
                    cap_mem[cap_count] <= dut.fifo_data_latched;
                    cap_count <= cap_count + 1;
                end
            end
        end
    end

    reg test_pass;
    reg [5:0] last_state;
    integer idx;

    initial begin
        test_pass = 1'b0;
        last_state = 6'h3F;
        cap_count = 0;
        fifo_read_pend_d = 1'b0;
        wait (sys_rst_n == 1'b1);
        wait (fifo_empty_flag == 1'b0);
        repeat (50) @(posedge sys_clk);
        collect_stop <= 1'b1;
        repeat (2) @(posedge sys_clk);
        collect_stop <= 1'b0;

        fork
            begin
                #(CLK_PERIOD_NS * 600000);
                $display("TIMEOUT");
                $display("T=%0t state=%0d txn_busy=%0d txn_done=%0d txn_ctl_state=%0d tx_idx=%0d rx_idx=%0d wait_rx=%0d trdy=%0d rrdy=%0d cs_n=%0d sclk=%0d",
                         $time, dut.state, dut.txn_busy, dut.txn_done, dut.txn_ctl_state,
                         dut.txn_tx_idx, dut.txn_rx_idx, dut.txn_wait_rx, dut.spi_trdy, dut.spi_rrdy,
                         spi_cs_n, spi_sclk);
                $finish;
            end
            begin
                wait (flash_done == 1'b1);
            end
        join_any
        disable fork;

        if (flash_error) begin
            $display("FAILED: flash_error asserted");
            test_pass = 1'b0;
            $finish;
        end

        if (cap_count == 0) begin
            $display("FAILED: no fifo data captured");
            test_pass = 1'b0;
            $finish;
        end

        for (idx = 0; idx < cap_count; idx = idx + 1) begin
            if (u_flash.mem[DIFF_ADDR_START + idx] !== cap_mem[idx]) begin
                $display("FAILED: mem[%0d]=%02h expected=%02h", DIFF_ADDR_START + idx, u_flash.mem[DIFF_ADDR_START + idx], cap_mem[idx]);
                test_pass = 1'b0;
                $finish;
            end
        end

        test_pass = 1'b1;
        $display("PASSED");
        repeat (50) @(posedge sys_clk);
        $finish;
    end

    always @(posedge sys_clk) begin
        if (sys_rst_n) begin
            if (dut.state !== last_state) begin
                $display("T=%0t state=%0d fifo_empty=%0d fifo_full=%0d busy=%0d done=%0d err=%0d", $time, dut.state, fifo_empty_flag, fifo_full_flag, flash_busy, flash_done, flash_error);
                last_state <= dut.state;
            end
            if (dut.txn_done) begin
                $display("T=%0t txn_done state=%0d b0=%02h rx0=%02h rx1=%02h flash_wip=%0d flash_wel=%0d busy_cnt=%0d", $time, dut.state, dut.txn_b0, dut.txn_rx0, dut.txn_rx1, u_flash.wip, u_flash.wel, u_flash.busy_cnt);
            end
        end
    end

    integer sclk_posedges_in_txn;
    integer cs_rises_in_txn;
    reg cs_n_d;

    always @(posedge spi_sclk) begin
        if (!spi_cs_n) begin
            sclk_posedges_in_txn <= sclk_posedges_in_txn + 1;
        end
    end

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            sclk_posedges_in_txn <= 0;
            cs_rises_in_txn <= 0;
            cs_n_d <= 1'b1;
        end else begin
            cs_n_d <= spi_cs_n;
            if (dut.txn_start) begin
                sclk_posedges_in_txn <= 0;
                cs_rises_in_txn <= 0;
            end
            if (dut.txn_busy && (cs_n_d == 1'b0) && (spi_cs_n == 1'b1)) begin
                cs_rises_in_txn <= cs_rises_in_txn + 1;
            end
            if (dut.txn_done) begin
                $display("T=%0t txn_clocks len=%0d sclk_posedges=%0d cs_rises=%0d cs_n=%0d", $time, dut.txn_len, sclk_posedges_in_txn, cs_rises_in_txn, spi_cs_n);
            end
        end
    end
endmodule

module spi_flash_model #(
    parameter integer MEM_BYTES = 4096
) (
    input  wire sys_clk,
    input  wire cs_n,
    input  wire sclk,
    input  wire mosi,
    output reg  miso
);
    reg [7:0] mem [0:MEM_BYTES-1];

    reg [7:0] in_shift;
    reg [7:0] out_shift;
    reg [2:0] bit_cnt;

    reg [7:0] cmd;
    reg [1:0] cmd_state;
    reg [1:0] addr_cnt;
    reg [23:0] addr;
    reg [7:0] data_byte;
    wire [7:0] in_byte = {in_shift[6:0], mosi};
    wire [23:0] addr_next = {addr[15:0], in_byte};

    reg wel;
    reg [15:0] busy_cnt;

    wire wip = (busy_cnt != 0);
    wire [7:0] status = {6'b0, wel, wip};

    integer i;

    initial begin
        for (i = 0; i < MEM_BYTES; i = i + 1) begin
            mem[i] = 8'hFF;
        end
        miso = 1'b1;
        in_shift = 8'h00;
        out_shift = 8'hFF;
        bit_cnt = 3'd0;
        cmd = 8'h00;
        cmd_state = 2'd0;
        addr_cnt = 2'd0;
        addr = 24'h0;
        data_byte = 8'h00;
        wel = 1'b0;
        busy_cnt = 16'd0;
    end

    always @(posedge sys_clk) begin
        if (busy_cnt != 0) begin
            busy_cnt <= busy_cnt - 16'd1;
        end
    end

    always @(posedge cs_n) begin
        cmd_state <= 2'd0;
        addr_cnt <= 2'd0;
        bit_cnt <= 3'd0;
        in_shift <= 8'h00;
        out_shift <= 8'hFF;
    end

    always @(negedge sclk) begin
        if (!cs_n) begin
            miso <= out_shift[7];
            out_shift <= {out_shift[6:0], 1'b1};
        end else begin
            miso <= 1'b1;
        end
    end

    task automatic do_erase_sector;
        input [23:0] sector_base;
        integer k;
        begin
            for (k = 0; k < 4096; k = k + 1) begin
                if ((sector_base + k) < MEM_BYTES) begin
                    mem[sector_base + k] = 8'hFF;
                end
            end
        end
    endtask

    always @(posedge sclk) begin
        if (!cs_n) begin
            in_shift <= {in_shift[6:0], mosi};
            bit_cnt <= bit_cnt + 3'd1;
            if (bit_cnt == 3'd7) begin
                if (cmd_state == 2'd0) begin
                    cmd <= in_byte;
                    if (in_byte == 8'h06) begin
                        wel <= 1'b1;
                        cmd_state <= 2'd0;
                    end else if (in_byte == 8'h05) begin
                        out_shift <= status;
                        cmd_state <= 2'd3;
                    end else if (in_byte == 8'h03) begin
                        cmd_state <= 2'd1;
                        addr_cnt <= 2'd0;
                        addr <= 24'h0;
                    end else if (in_byte == 8'h20) begin
                        cmd_state <= 2'd1;
                        addr_cnt <= 2'd0;
                        addr <= 24'h0;
                    end else if (in_byte == 8'h02) begin
                        cmd_state <= 2'd1;
                        addr_cnt <= 2'd0;
                        addr <= 24'h0;
                    end else begin
                        cmd_state <= 2'd0;
                    end
                end else if (cmd_state == 2'd1) begin
                    addr <= addr_next;
                    if (addr_cnt == 2'd2) begin
                        if (cmd == 8'h20) begin
                            if (wel) begin
                                do_erase_sector({addr_next[23:12], 12'h000});
                                wel <= 1'b0;
                                busy_cnt <= 16'd800;
                            end
                            cmd_state <= 2'd0;
                            addr_cnt <= 2'd0;
                        end else if (cmd == 8'h03) begin
                            if (addr_next < MEM_BYTES) begin
                                out_shift <= mem[addr_next];
                            end else begin
                                out_shift <= 8'hFF;
                            end
                            addr_cnt <= 2'd0;
                            cmd_state <= 2'd3;
                        end else if (cmd == 8'h02) begin
                            cmd_state <= 2'd2;
                            addr_cnt <= 2'd0;
                        end else begin
                            cmd_state <= 2'd0;
                            addr_cnt <= 2'd0;
                        end
                    end else begin
                        addr_cnt <= addr_cnt + 2'd1;
                    end
                end else if (cmd_state == 2'd2) begin
                    data_byte <= in_byte;
                    if (wel) begin
                        if (addr < MEM_BYTES) begin
                            mem[addr] <= in_byte;
                        end
                        wel <= 1'b0;
                        busy_cnt <= 16'd200;
                    end
                    cmd_state <= 2'd0;
                end else if (cmd_state == 2'd3) begin
                    if (cmd == 8'h05) begin
                        out_shift <= status;
                    end else if (cmd == 8'h03) begin
                        if (addr < MEM_BYTES) begin
                            out_shift <= mem[addr];
                        end else begin
                            out_shift <= 8'hFF;
                        end
                        addr <= addr + 24'd1;
                    end else begin
                        out_shift <= 8'hFF;
                    end
                end else begin
                    cmd_state <= 2'd0;
                end
                bit_cnt <= 3'd0;
            end
        end else begin
            bit_cnt <= 3'd0;
        end
    end
endmodule

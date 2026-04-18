`timescale 1ns / 1ps
// SPI驱动模块 v8.0
// 完全参考官方例程 flash_spi.v 的时序策略：
// 采用 50MHz sys_clk 作为驱动时钟，生成 25MHz 的 SPI SCLK
// Mode 0: CPOL=0, CPHA=0，MSB first
//   - SCLK 空闲为低电平
//   - MOSI 在 SCLK 下降沿更新（在上升沿前稳定）
//   - MISO 在 SCLK 上升沿采样

module spi_driver (
    input           sys_clk,
    input           sys_rst_n,

    input           txn_start,
    input [2:0]     txn_len,
    input [7:0]     txn_b0,
    input [7:0]     txn_b1,
    input [7:0]     txn_b2,
    input [7:0]     txn_b3,
    input [7:0]     txn_b4,

    output reg      txn_busy,
    output reg      txn_done,
    output reg [7:0] txn_rx0,
    output reg [7:0] txn_rx1,
    output reg [7:0] txn_rx2,
    output reg [7:0] txn_rx3,
    output reg [7:0] txn_rx4,

    output reg      spi_sclk,
    output reg      spi_mosi,
    input           spi_miso,
    output reg      spi_cs_n
);

    localparam [2:0]
        ST_IDLE      = 3'd0,
        ST_CS_LOW    = 3'd1,
        ST_CLK_HI    = 3'd2,
        ST_CLK_LO    = 3'd3,
        ST_CS_HIGH   = 3'd4,
        ST_DONE      = 3'd5,
        ST_CS_DELAY  = 3'd6;

    reg [2:0] state;
    reg [5:0] bit_cnt;
    reg [5:0] total_bits;
    reg [39:0] tx_shift;
    reg [39:0] rx_shift;
    reg [2:0]  len_lat;
    reg [3:0]  cs_delay_cnt;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            state      <= ST_IDLE;
            spi_sclk   <= 1'b0;
            spi_mosi   <= 1'b0;
            spi_cs_n   <= 1'b1;
            txn_busy   <= 1'b0;
            txn_done   <= 1'b0;
            bit_cnt    <= 6'd0;
            total_bits <= 6'd0;
            tx_shift   <= 40'd0;
            rx_shift   <= 40'd0;
            len_lat    <= 3'd0;
            cs_delay_cnt <= 4'd0;
            {txn_rx0, txn_rx1, txn_rx2, txn_rx3, txn_rx4} <= 40'd0;
        end else begin
            txn_done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    spi_cs_n <= 1'b1;
                    spi_sclk <= 1'b0;
                    if (txn_start && !txn_busy) begin
                        txn_busy <= 1'b1;
                        tx_shift <= {txn_b0, txn_b1, txn_b2, txn_b3, txn_b4};
                        len_lat  <= txn_len;
                        case (txn_len)
                            3'd1: total_bits <= 6'd8;
                            3'd2: total_bits <= 6'd16;
                            3'd3: total_bits <= 6'd24;
                            3'd4: total_bits <= 6'd32;
                            default: total_bits <= 6'd40;
                        endcase
                        bit_cnt <= 6'd0;
                        state   <= ST_CS_LOW;
                    end
                end

                ST_CS_LOW: begin
                    spi_cs_n <= 1'b0;
                    spi_sclk <= 1'b0;
                    // 在第一个SCLK上升沿之前提前准备好MOSI数据
                    spi_mosi <= tx_shift[39];
                    tx_shift <= {tx_shift[38:0], 1'b0};
                    state    <= ST_CLK_HI;
                end

                ST_CLK_HI: begin
                    spi_sclk <= 1'b1;
                    // 在SCLK上升沿采样MISO
                    rx_shift <= {rx_shift[38:0], spi_miso};
                    state    <= ST_CLK_LO;
                end

                ST_CLK_LO: begin
                    spi_sclk <= 1'b0;
                    bit_cnt  <= bit_cnt + 6'd1;
                    
                    if (bit_cnt + 6'd1 >= total_bits) begin
                        state <= ST_CS_HIGH;
                    end else begin
                        // 在SCLK下降沿更新MOSI，为下一次采样做准备
                        spi_mosi <= tx_shift[39];
                        tx_shift <= {tx_shift[38:0], 1'b0};
                        state    <= ST_CLK_HI;
                    end
                end

                ST_CS_HIGH: begin
                    spi_cs_n <= 1'b1;
                    spi_sclk <= 1'b0;
                    
                    // 将接收到的数据对齐到对应的寄存器
                    case (len_lat)
                        3'd1: begin
                            txn_rx0 <= rx_shift[7:0];
                        end
                        3'd2: begin
                            txn_rx0 <= rx_shift[15:8];
                            txn_rx1 <= rx_shift[7:0];
                        end
                        3'd3: begin
                            txn_rx0 <= rx_shift[23:16];
                            txn_rx1 <= rx_shift[15:8];
                            txn_rx2 <= rx_shift[7:0];
                        end
                        3'd4: begin
                            txn_rx0 <= rx_shift[31:24];
                            txn_rx1 <= rx_shift[23:16];
                            txn_rx2 <= rx_shift[15:8];
                            txn_rx3 <= rx_shift[7:0];
                        end
                        default: begin
                            txn_rx0 <= rx_shift[39:32];
                            txn_rx1 <= rx_shift[31:24];
                            txn_rx2 <= rx_shift[23:16];
                            txn_rx3 <= rx_shift[15:8];
                            txn_rx4 <= rx_shift[7:0];
                        end
                    endcase
                    cs_delay_cnt <= 4'd0;
                    state <= ST_CS_DELAY;
                end

                ST_CS_DELAY: begin
                    // 保证 CS 拉高时间至少为 100ns (5个 50MHz 时钟周期)
                    if (cs_delay_cnt < 4'd5) begin
                        cs_delay_cnt <= cs_delay_cnt + 4'd1;
                    end else begin
                        state <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    txn_busy <= 1'b0;
                    txn_done <= 1'b1;
                    state    <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

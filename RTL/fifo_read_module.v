`timescale 1ns / 1ps

// Flash读取模块 v5.0
// 使用 flash_controller_enhanced 的单字节读取接口
// 不含任何 SPI_0 实例，彻底解决双实例冲突问题

module fifo_read_module (
    input           sys_clk,
    input           sys_rst_n,
    input           start_read,
    input   [23:0]  read_start_addr,
    input           use_word_limit,
    input   [15:0]  word_limit,
    input   [7:0]   ff_stop_run,

    // flash_controller_enhanced 单字节读取接口
    output reg        rd_req,        // 读取请求脉冲
    output reg [23:0] rd_addr,       // 读取地址
    input      [7:0]  rd_data,       // 读出数据
    input             rd_data_valid, // 读出数据有效脉冲
    input             flash_busy,    // Flash忙（擦除/写入时不发读请求）

    // Output to dump
    output reg             dout_valid,
    input                  dout_ready,
    output signed [31:0]   dout_value,
    output                 dout_empty,
    output                 read_done,
    output reg [7:0]       dbg_first_byte,
    output reg             dbg_first_byte_valid
);

    // ========================================================================
    // 内部 FIFO
    // ========================================================================
    wire        fifo_rst;
    reg  [31:0] fifo_din;
    reg         fifo_we;
    wire        fifo_re;
    wire [31:0] fifo_do_32;
    wire        fifo_empty;
    wire        fifo_full;
    wire        fifo_afull;

    // 内部FIFO复位极性，直接绑死为0，排除复位导致的卡死
    assign fifo_rst = 1'b0;

    FIFO_read u_fifo_read (
        .rst        (fifo_rst),
        .di         (fifo_din),
        .clk        (sys_clk),
        .we         (fifo_we),
        .do         (fifo_do_32),
        .re         (fifo_re),
        .empty_flag (fifo_empty),
        .full_flag  (fifo_full),
        .afull_flag (fifo_afull)
    );

    reg fifo_re_d1;
    assign fifo_re = dout_ready && !fifo_empty;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            fifo_re_d1 <= 1'b0;
            dout_valid <= 1'b0;
        end else begin
            fifo_re_d1 <= fifo_re;
            // dout_valid 在 fifo_re 后一个周期有效（fifo输出有1周期延迟）
            dout_valid <= fifo_re_d1;
        end
    end

    assign dout_value = $signed(fifo_do_32);
    assign dout_empty = fifo_empty;

    // ========================================================================
    // Flash 读取状态机
    // 每次读1字节：通过 rd_req/rd_addr → rd_data/rd_data_valid
    // 4字节拼成1个32位字后写入内部FIFO
    // ========================================================================
    localparam [2:0]
        ST_IDLE      = 3'd0,
        ST_WAIT_IDLE = 3'd1,  // 等 flash_busy=0
        ST_RD_REQ    = 3'd2,  // 发读请求
        ST_RD_WAIT   = 3'd3,  // 等读完成
        ST_PUSH_FIFO = 3'd4,
        ST_DONE      = 3'd5;

    localparam [23:0] ADDR_END = 24'hFFFFFF; // 修改为全地址空间允许读取

    reg [2:0]  state;
    reg [23:0] cur_addr;
    reg [31:0] rd_word;
    reg [1:0]  rd_byte_idx;
    reg [15:0] words_read;
    reg [15:0] words_target;
    reg [7:0]  ff_run_count;

    assign read_done = (state == ST_DONE);

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            state                <= ST_IDLE;
            rd_req               <= 1'b0;
            rd_addr              <= 24'h100000;
            fifo_we              <= 1'b0;
            fifo_din             <= 32'd0;
            cur_addr             <= 24'h100000;
            rd_word              <= 32'd0;
            rd_byte_idx          <= 2'd0;
            words_read           <= 16'd0;
            words_target         <= 16'd0;
            dbg_first_byte       <= 8'h00;
            dbg_first_byte_valid <= 1'b0;
            ff_run_count         <= 8'd0;
        end else begin
            rd_req  <= 1'b0;
            fifo_we <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (start_read) begin
                        cur_addr             <= read_start_addr;
                        rd_word              <= 32'd0;
                        rd_byte_idx          <= 2'd0;
                        words_read           <= 16'd0;
                        words_target         <= use_word_limit ? word_limit : 16'd0;
                        ff_run_count         <= 8'd0;
                        dbg_first_byte_valid <= 1'b0;
                        state                <= ST_WAIT_IDLE;
                    end
                end

                ST_WAIT_IDLE: begin
                    // 等 Flash 控制器空闲（不在擦除/写入）
                    if (!flash_busy) begin
                        state <= ST_RD_REQ;
                    end
                end

                ST_RD_REQ: begin
                    // 发单字节读请求
                    if (!flash_busy) begin
                        rd_addr <= cur_addr;
                        rd_req  <= 1'b1;
                        state   <= ST_RD_WAIT;
                    end
                end

                ST_RD_WAIT: begin
                    if (rd_data_valid) begin
                        // 锁存第一个字节用于调试
                        if (rd_byte_idx == 2'd0 && !dbg_first_byte_valid) begin
                            dbg_first_byte       <= rd_data;
                            dbg_first_byte_valid <= 1'b1;
                        end
                        // 按大端序拼装
                        case (rd_byte_idx)
                            2'd0: rd_word[31:24] <= rd_data;
                            2'd1: rd_word[23:16] <= rd_data;
                            2'd2: rd_word[15:8]  <= rd_data;
                            2'd3: rd_word[7:0]   <= rd_data;
                        endcase

                        if (rd_byte_idx == 2'd3) begin
                            state <= ST_PUSH_FIFO;
                        end else begin
                            rd_byte_idx <= rd_byte_idx + 2'd1;
                            cur_addr    <= cur_addr + 24'd1;
                            state       <= ST_RD_REQ;
                        end
                    end
                end

                ST_PUSH_FIFO: begin
                    // 关键修复：只有在内部FIFO有空间时才写入并前进，
                    // 避免导出大量数据时因FIFO溢出导致样本丢失。
                    if (!fifo_full) begin
                        fifo_din   <= rd_word;
                        fifo_we    <= 1'b1;
                        words_read <= words_read + 16'd1;

                        if (use_word_limit && (words_read + 16'd1 >= words_target)) begin
                            state <= ST_DONE;
                        end else if (!use_word_limit) begin
                            // 扫描模式：遇到连续 ff_stop_run 个 0xFFFFFFFF 才结束
                            if (rd_word == 32'hFFFF_FFFF)
                                ff_run_count <= ff_run_count + 8'd1;
                            else
                                ff_run_count <= 8'd0;

                            if (rd_word == 32'hFFFF_FFFF && ff_run_count + 8'd1 >= ff_stop_run)
                                state <= ST_DONE;
                            else if (cur_addr + 24'd1 <= ADDR_END) begin
                                cur_addr    <= cur_addr + 24'd1;
                                rd_byte_idx <= 2'd0;
                                state       <= ST_RD_REQ;
                            end else begin
                                state <= ST_DONE;
                            end
                        end else if (cur_addr + 24'd1 <= ADDR_END) begin
                            cur_addr    <= cur_addr + 24'd1;
                            rd_byte_idx <= 2'd0;
                            state       <= ST_RD_REQ;
                        end else begin
                            state <= ST_DONE;
                        end
                    end
                    // fifo_full时停在本状态等待，不丢样本
                end

                ST_DONE: begin
                    if (!start_read)
                        state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

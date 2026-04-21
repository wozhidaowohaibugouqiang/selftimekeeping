// 增强版Flash控制模块
// 模块功能：支持参数数据写入、地址映射表管理和数据校验
// 设计作者：TraeAI
// 设计日期：2026-03-10
// 版本号：v2.0
//
// 输入端口：
// - sys_clk：系统时钟
// - sys_rst_n：系统复位
// - collect_stop：采集停止信号（传统模式）
// - fifo_dout：FIFO数据输出（传统模式）
// - fifo_empty_flag：FIFO空标志
// - fifo_full_flag：FIFO满标志
// - spi_miso：SPI MISO输入
// - param_data：参数数据（增强模式）
// - param_addr：参数地址（增强模式）
// - param_valid：参数有效标志（增强模式）
// - param_type：参数类型（增强模式）
// - table_entry：地址映射表条目索引
// - table_write：地址映射表写入使能
//
// 输出端口：
// - fifo_re：FIFO读使能
// - spi_sclk：SPI时钟
// - spi_mosi：SPI MOSI
// - spi_cs_n：SPI CS_n
// - flash_busy：Flash忙标志
// - flash_done：Flash操作完成标志
// - flash_error：Flash操作错误标志
// - hist_diff：历史差值数据（Boot读取）
// - hist_diff_valid：历史差值数据有效标志
// - table_data：地址映射表数据读取
// - table_valid：地址映射表数据有效
//
// 设计说明：
// 1. 支持两种工作模式：传统差值存储模式和参数管理模式
// 2. 地址映射表结构：每个条目64字节
//    - 0x00: 魔数(0x55)
//    - 0x01: 参数类型
//    - 0x02-0x03: 版本号
//    - 0x04-0x07: 时间戳
//    - 0x08-0x0A: 数据起始地址
//    - 0x0B-0x0D: 数据结束地址
//    - 0x0E-0x0F: CRC16
//    - 0x10-0x3F: 保留
// 3. 支持数据CRC校验
//
`timescale 1ns / 1ps

module flash_controller_enhanced (
    input           sys_clk,
    input           sys_rst_n,
    input           collect_stop,
    input           erase_req,      // 外部全擦除请求（按键触发）

    // 传统FIFO接口
    output reg      fifo_re,
    input   [31:0]  fifo_dout,
    input           fifo_empty_flag,
    input           fifo_full_flag,

    // 增强参数接口
    input   [7:0]   param_data,
    input   [23:0]  param_addr,
    input           param_valid,
    input   [1:0]   param_type,

    // 地址映射表接口
    input   [5:0]   table_entry,
    input           table_write,
    output reg [7:0] table_data,
    output reg      table_valid,

    // SPI物理接口
    output          spi_sclk,
    output          spi_mosi,
    input           spi_miso,
    output          spi_cs_n,

    // 状态输出
    output reg      flash_busy,
    output reg      flash_done,
    output reg      write_done,   // 仅写入完成（不含擦除完成）
    output reg      erase_done,    // 新增：擦除完成脉冲
    output reg      flash_error,
    output reg [7:0] hist_diff,
    output reg      hist_diff_valid,
    output reg [31:0] last_prog_word_mirror,
    output reg [23:0] cur_addr_dbg,  // 调试：当前写入/读取地址
    output reg [15:0] poll_attempts_dbg, // 调试：最后一次轮询次数
    output reg [7:0]  last_sr_dbg,       // 调试：最后一次读到的状态寄存器
    output reg [2:0]  sr_stage_dbg,      // 调试：最近一次RDSR1所处子阶段(1=PP_WREN检查,2=PP轮询,3=ERASE_WREN检查,4=ERASE轮询)
    output reg [7:0]  last_cmd_dbg,      // 调试：最近一次SPI事务命令字节
    output reg [3:0]  last_len_dbg,      // 调试：最近一次SPI事务长度
    output reg [7:0]  last_rx0_dbg,      // 调试：最近一次SPI事务接收字节0
    output reg [7:0]  last_rx1_dbg,      // 调试：最近一次SPI事务接收字节1
    output reg [3:0]  err_code_dbg,      // 调试：错误码(1=ERASE轮询超时,2=PROG轮询超时)
    output reg [23:0] jedec_id_dbg,      // 调试：上电读取到的JEDEC ID
    output reg        jedec_valid_dbg,   // 调试：JEDEC ID是否有效
    output wire [6:0] state_dbg,         // 调试：当前状态机状态
    output reg  [7:0] txn_done_cnt_dbg,  // 调试：SPI事务完成计数（低8位）
    output wire       flush_active,      // FIFO正在写入Flash标志
    // 新增：调试计数器 - 追踪flush写入链路(8位，节省资源)
    output reg [7:0] check_fifo_cnt,    // 进入ST_CHECK_FIFO次数
    output reg [7:0] fifo_re_done_cnt,  // 完成FIFO读取次数
    output reg [7:0] fifo_latch_done_cnt, // 完成FIFO数据锁存次数
    output reg [15:0] fifo_prog_cnt,       // FIFO实际写入Flash字数
    output reg        fifo_was_empty_at_dump, // dump开始时FIFO是否为空
    input  [1:0]    fifo_state_at_dump_start, // Dump开始时刻FIFO状态
    input  [15:0]   fifo_total_cnt_at_dump_start, // Dump开始时刻累计写入字数

    // 单字节读取接口（与写入共用 spi_driver）
    input            rd_req,
    input   [23:0]   rd_addr,
    output reg [7:0] rd_data,
    output reg       rd_data_valid,
    // 读取模式标志：当为高时，忽略擦除/写入请求，优先处理读取
    input            read_mode,
    // Dump强制flush信号：绕过fifo_full_flag判断，直接触发flush写入
    input            dump_flush_force
);

    // ========================================================================
    // SPI Driver 接口信号
    // ========================================================================
    reg         txn_start;
    reg [2:0]   txn_len;
    reg [7:0]   txn_b0;
    reg [7:0]   txn_b1;
    reg [7:0]   txn_b2;
    reg [7:0]   txn_b3;
    reg [7:0]   txn_b4;
    wire        txn_busy;
    wire        txn_done;
    wire [7:0]  txn_rx0;
    wire [7:0]  txn_rx1;
    wire [7:0]  txn_rx2;
    wire [7:0]  txn_rx3;
    wire [7:0]  txn_rx4;

    spi_driver u_spi_driver (
        .sys_clk    (sys_clk),
        .sys_rst_n  (sys_rst_n),
        .txn_start  (txn_start),
        .txn_len    (txn_len),
        .txn_b0     (txn_b0),
        .txn_b1     (txn_b1),
        .txn_b2     (txn_b2),
        .txn_b3     (txn_b3),
        .txn_b4     (txn_b4),
        .txn_busy   (txn_busy),
        .txn_done   (txn_done),
        .txn_rx0    (txn_rx0),
        .txn_rx1    (txn_rx1),
        .txn_rx2    (txn_rx2),
        .txn_rx3    (txn_rx3),
        .txn_rx4    (txn_rx4),
        .spi_sclk   (spi_sclk),
        .spi_mosi   (spi_mosi),
        .spi_miso   (spi_miso),
        .spi_cs_n   (spi_cs_n)
    );

    // ========================================================================
    // 常量定义
    // ========================================================================
    localparam [7:0] FLASH_CMD_WREN      = 8'h06;
    localparam [7:0] FLASH_CMD_RDSR1     = 8'h05;
    localparam [7:0] FLASH_CMD_SE        = 8'h20;
    localparam [7:0] FLASH_CMD_PP        = 8'h02;
    localparam [7:0] FLASH_CMD_READ      = 8'h03;
    localparam [7:0] FLASH_CMD_RSTEN     = 8'h66;  // Reset Enable
    localparam [7:0] FLASH_CMD_RST       = 8'h99;  // Reset
    localparam [7:0] FLASH_CMD_JEDEC_ID  = 8'h9F;

    localparam [15:0] POLL_PROG_MAX      = 16'd1000;
    localparam [15:0] POLL_ERASE_MAX     = 16'd10000;
    localparam [23:0] POLL_PROG_WAIT_CYC = 24'd500;
    localparam [23:0] POLL_ERASE_WAIT_CYC= 24'd50000;

    localparam [23:0] ADDR_TABLE_BASE    = 24'h000000;
    localparam [23:0] ADDR_TABLE_SIZE    = 24'h001000;
    localparam [23:0] DIFF_DATA_BASE     = 24'h100000;  // 训练/回放差值区（扩大地址空间）
    localparam [23:0] DYN_PARAM_BASE     = 24'h3E0000;  // 动态参数区
    localparam [23:0] META_DATA_BASE     = 24'h3F0000;  // 元数据区
    localparam [23:0] DIFF_DATA_END      = 24'h3DFFFF;  // 预留 0x3E0000/0x3F0000 给动态参数/元数据
    localparam [23:0] ERASE_ALL_BASE     = 24'h000000;
    localparam [23:0] ERASE_ALL_END      = META_DATA_BASE; // 必须覆盖 DYN_PARAM_BASE 所在整扇区
    localparam [23:0] PARAM_DATA_BASE    = 24'h010000;  // 老化参数区
    localparam [7:0]  TABLE_MAGIC        = 8'h55;
    localparam [6:0]  TABLE_ENTRY_SIZE   = 7'd64;  // 64字节/条目

    // ========================================================================
    // 状态机定义
    // ========================================================================
    localparam [6:0]
        ST_IDLE             = 7'd0,
        ST_CHECK_TRIGGER    = 7'd1,
        ST_CHECK_NEED_ERASE = 7'd2,
        ST_WREN_ERASE       = 7'd3,
        ST_SEND_ERASE       = 7'd4,
        ST_POLL_ERASE       = 7'd5,
        ST_CHECK_FIFO       = 7'd6,
        ST_FIFO_RE          = 7'd7,
        ST_FIFO_LATCH       = 7'd8,
        ST_WREN_PROG        = 7'd9,
        ST_SEND_PROG        = 7'd10,
        ST_POLL_PROG        = 7'd11,
        ST_INC_ADDR         = 7'd12,
        ST_DONE             = 7'd13,
        ST_BOOT_READ        = 7'd14,
        ST_BOOT_WAIT        = 7'd15,
        ST_PARAM_WRITE      = 7'd16,
        ST_TABLE_READ       = 7'd17,
        ST_TABLE_WRITE      = 7'd18,
        ST_TABLE_WAIT       = 7'd19,
        ST_ERASE_ALL_WREN   = 7'd20,
        ST_ERASE_ALL_SE     = 7'd21,
        ST_ERASE_ALL_POLL   = 7'd22,
        ST_ERASE_ALL_NEXT   = 7'd23,
        ST_ERASE_ALL_DONE   = 7'd24,
        ST_RD_REQ           = 7'd25,  // 单字节读：发送命令+地址
        ST_RD_WAIT          = 7'd26,  // 单字节读：等待完成
        ST_SW_RESET_EN      = 7'd27,  // 软件复位：发 0x66 Reset Enable
        ST_SW_RESET         = 7'd28,  // 软件复位：发 0x99 Reset
        ST_SW_RESET_WAIT    = 7'd29,  // 软件复位：等待 tRST=30us
        ST_JEDEC_REQ        = 7'd30,  // 上电自检：读取JEDEC ID
        ST_JEDEC_WAIT       = 7'd31,  // 上电自检：等待读取完成
        ST_RD_DONE          = 7'd32;  // 等待 rd_req 撤销

    reg [6:0] state;
    assign state_dbg = state;
    assign flush_active = flush_pending;  // FIFO正在写入Flash标志

    // SPI事务完成计数器（用于调试确认spi_driver是否正常工作）
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            txn_done_cnt_dbg <= 8'd0;
        else if (txn_done)
            txn_done_cnt_dbg <= txn_done_cnt_dbg + 8'd1;
    end

    reg [6:0] state_max;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            state_max <= 7'd0;
        else if (state > state_max)
            state_max <= state;
    end

    // 新增：调试计数器 - 追踪flush写入链路的关键状态转移
    // 使用寄存器实现边沿检测(避免$past的SystemVerilog兼容性问题)
    reg [6:0] state_prev;
    reg [1:0] fifo_read_pend_prev;
    reg       dump_flush_force_prev;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            check_fifo_cnt      <= 8'd0;
            fifo_re_done_cnt   <= 8'd0;
            fifo_latch_done_cnt<= 8'd0;
            state_prev         <= 7'd0;
            fifo_read_pend_prev <= 2'd0;
            dump_flush_force_prev <= 1'b0;
        end else begin
            state_prev          <= state[6:0];
            fifo_read_pend_prev <= fifo_read_pend;
            dump_flush_force_prev <= dump_flush_force;
            // 进入ST_CHECK_FIFO时计数(检测状态从其他状态变为ST_CHECK_FIFO)
            if (state == ST_CHECK_FIFO && state_prev != ST_CHECK_FIFO)
                check_fifo_cnt <= check_fifo_cnt + 8'd1;
            // 进入ST_FIFO_RE时计数(检测状态从其他状态变为ST_FIFO_RE)
            if (state == ST_FIFO_RE && state_prev != ST_FIFO_RE)
                fifo_re_done_cnt <= fifo_re_done_cnt + 8'd1;
            // FIFO数据锁存完成时计数(当fifo_read_pend从1变为0时触发)
            if (fifo_read_pend == 0 && fifo_read_pend_prev == 1)
                fifo_latch_done_cnt <= fifo_latch_done_cnt + 8'd1;
        end
    end
    // ========================================================================
    reg         flush_pending;
    // FIFO写入字数统计（供调试诊断用）
    reg [15:0]  fifo_prog_cnt;
    reg         fifo_was_empty_at_dump;  // dump开始时FIFO是否为空
    reg [23:0]  cur_addr;
    reg [23:0]  erased_sector_base;
    reg [31:0]  fifo_data_latched;
    reg [1:0]   fifo_read_pend;
    reg [31:0]  prog_word;
    reg [1:0]   prog_byte_idx;
    reg [31:0]  boot_word;
    reg [1:0]   boot_byte_idx;
    reg [15:0]  poll_attempts;
    reg [23:0]  poll_wait;
    reg         prog_poll_issued;
    reg         prog_wren_checked;
    reg         erase_poll_issued;
    reg         erase_wren_checked;
    reg [7:0]   last_status;
    reg [7:0]   calc_checksum;
    reg [7:0]   read_checksum;
    reg [23:0]  verify_addr;

    // 参数模式信号
    reg         erase_pending;
    reg [23:0]  erase_sector_cur;
    // 参数模式信号
    reg [7:0]   param_data_latched;
    reg [23:0]  param_addr_latched;
    reg [1:0]   param_type_latched;
    reg         param_write_pending;

    // 地址映射表信号
    reg [23:0]  table_addr;
    reg [5:0]   table_byte_cnt;
    reg [7:0]   table_buffer[63:0];

    wire [23:0] sector_base = {cur_addr[23:12], 12'h000};
    // 动态补偿参数不需要在这里自动擦除（由外部按键决定擦除），所以不将其纳入自动擦除判断
    wire [23:0] active_sector_base = sector_base;
    wire        need_erase = (erased_sector_base != active_sector_base);

    // ========================================================================
    // 主状态机
    // ========================================================================
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            state               <= ST_SW_RESET_EN; // 上电先软件复位Flash，再进入IDLE
            fifo_re             <= 1'b0;
            flush_pending       <= 1'b0;
            fifo_prog_cnt       <= 16'd0;
            fifo_was_empty_at_dump <= 1'b0;
            cur_addr            <= DIFF_DATA_BASE;
            erased_sector_base  <= 24'hFFFFFF;
            fifo_data_latched   <= 32'h00000000;
            fifo_read_pend      <= 2'd0;
            prog_word           <= 32'h00000000;
            prog_byte_idx       <= 2'd0;
            boot_word           <= 32'h00000000;
            boot_byte_idx       <= 2'd0;
            poll_attempts       <= 16'd0;
            poll_wait           <= 24'd0;
            prog_poll_issued    <= 1'b0;
            prog_wren_checked   <= 1'b0;
            erase_poll_issued   <= 1'b0;
            erase_wren_checked  <= 1'b0;
            last_status         <= 8'h00;
            calc_checksum       <= 8'h00;
            read_checksum       <= 8'h00;
            verify_addr         <= DIFF_DATA_BASE;
            txn_start           <= 1'b0;
            txn_len             <= 3'd0;
            txn_b0              <= 8'h00;
            txn_b1              <= 8'h00;
            txn_b2              <= 8'h00;
            txn_b3              <= 8'h00;
            txn_b4              <= 8'h00;
            flash_busy          <= 1'b0;
            flash_done          <= 1'b0;
            write_done          <= 1'b0;
            erase_done          <= 1'b0;
            flash_error         <= 1'b0;
            hist_diff           <= 8'h00;
            hist_diff_valid     <= 1'b0;
            last_prog_word_mirror <= 32'h00000000;
            rd_data             <= 8'h00;
            rd_data_valid       <= 1'b0;
            cur_addr_dbg        <= DIFF_DATA_BASE;
            poll_attempts_dbg   <= 16'd0;
            last_sr_dbg         <= 8'h00;
            sr_stage_dbg        <= 3'd0;
            last_cmd_dbg        <= 8'h00;
            last_len_dbg        <= 4'h0;
            last_rx0_dbg        <= 8'h00;
            last_rx1_dbg        <= 8'h00;
            err_code_dbg        <= 4'h0;
            jedec_id_dbg        <= 24'h000000;
            jedec_valid_dbg     <= 1'b0;
            erase_pending       <= 1'b0;
            erase_sector_cur    <= ERASE_ALL_BASE;
            param_data_latched  <= 8'h00;
            param_addr_latched  <= 24'h000000;
            param_type_latched  <= 2'b00;
            param_write_pending <= 1'b0;
            table_addr          <= 24'h000000;
            table_byte_cnt      <= 6'd0;
            table_data          <= 8'h00;
            table_valid         <= 1'b0;
        end else begin
            fifo_re     <= 1'b0;
            txn_start   <= 1'b0;
            flash_done  <= 1'b0;
            write_done  <= 1'b0;
            erase_done  <= 1'b0;
            hist_diff_valid <= 1'b0;
            table_valid <= 1'b0;
            rd_data_valid <= 1'b0;

            if (txn_done) begin
                // 避免 txn_start 参与额外组合选择，降低局部布线冲突风险
                last_cmd_dbg <= txn_b0;
                last_len_dbg <= {1'b0, txn_len};
                last_rx0_dbg <= txn_rx0;
                last_rx1_dbg <= txn_rx1;
            end

            if (fifo_read_pend != 2'd0) begin
                fifo_read_pend <= fifo_read_pend - 2'd1;
                if (fifo_read_pend == 2'd1) begin
                    fifo_data_latched <= fifo_dout;
                end
            end

            // FIFO写入字数计数（每次ST_FIFO_LATCH完成时+1）
            if (state == ST_FIFO_LATCH && fifo_read_pend == 0)
                fifo_prog_cnt <= fifo_prog_cnt + 16'd1;

            // 检测dump开始时刻：dump_flush_force从0变1瞬间锁存FIFO状态
            if (dump_flush_force && !dump_flush_force_prev)
                fifo_was_empty_at_dump <= fifo_empty_flag;

            if (!flush_pending && (fifo_full_flag || collect_stop)) begin
                flush_pending <= 1'b1;
                calc_checksum <= 8'h00;
                read_checksum <= 8'h00;
            end

            if (erase_req && !erase_pending && state == ST_IDLE) begin
                erase_pending    <= 1'b1;
                erase_sector_cur <= ERASE_ALL_BASE;
            end

            if (param_valid && !param_write_pending) begin
                param_write_pending <= 1'b1;
                param_data_latched <= param_data;
                param_addr_latched <= param_addr;
                param_type_latched <= param_type;
            end

            case (state)
                // ====================================================================
                // 传统模式状态（保持兼容）
                // ====================================================================
                ST_BOOT_READ: begin
                    flash_busy <= 1'b0;
                    if (cur_addr >= DIFF_DATA_END) begin
                         state <= ST_IDLE;
                    end else if (!txn_busy) begin
                        txn_len   <= 3'd5;
                        txn_b0    <= FLASH_CMD_READ;
                        txn_b1    <= cur_addr[23:16];
                        txn_b2    <= cur_addr[15:8];
                        txn_b3    <= cur_addr[7:0];
                        txn_b4    <= 8'hFF;
                        txn_start <= 1'b1;
                        state     <= ST_BOOT_WAIT;
                    end
                end

                ST_BOOT_WAIT: begin
                    flash_busy <= 1'b0;
                    if (txn_done) begin
                        case (boot_byte_idx)
                            2'd0: boot_word[31:24] <= txn_rx4;
                            2'd1: boot_word[23:16] <= txn_rx4;
                            2'd2: boot_word[15:8]  <= txn_rx4;
                            default: boot_word[7:0] <= txn_rx4;
                        endcase

                        cur_addr <= cur_addr + 24'd1;

                        if (boot_byte_idx == 2'd3) begin
                            if ({boot_word[31:8], txn_rx4} != 32'hFFFF_FFFF && {boot_word[31:8], txn_rx4} != 32'h0000_0000) begin
                                hist_diff       <= txn_rx4;
                                hist_diff_valid <= 1'b1;
                                boot_byte_idx   <= 2'd0;
                                state           <= ST_BOOT_READ;
                            end else begin
                                state           <= ST_IDLE;
                                cur_addr        <= DIFF_DATA_BASE;
                                boot_byte_idx   <= 2'd0;
                            end
                        end else begin
                            boot_byte_idx <= boot_byte_idx + 2'd1;
                            state         <= ST_BOOT_READ;
                        end
                    end
                end

                // ====================================================================
                // 上电软件复位：把Flash从QPI/任何异常模式恢复到标准SPI
                // ====================================================================
                ST_SW_RESET_EN: begin
                    // 发 Reset Enable (0x66)
                    if (!txn_busy) begin
                        txn_len   <= 3'd1;
                        txn_b0    <= FLASH_CMD_RSTEN;
                        txn_start <= 1'b1;
                        state     <= ST_SW_RESET;
                    end
                end

                ST_SW_RESET: begin
                    // 等 0x66 完成，再发 Reset (0x99)
                    if (txn_done) begin
                        txn_len   <= 3'd1;
                        txn_b0    <= FLASH_CMD_RST;
                        txn_start <= 1'b1;
                        poll_wait <= 24'd0;  // 复用poll_wait计时tRST
                        state     <= ST_SW_RESET_WAIT;
                    end
                end

                ST_SW_RESET_WAIT: begin
                    // W25Q32 tRST=30us，@50MHz需等1500拍
                    // 用poll_wait计数，复位完成后先做JEDEC自检
                    if (poll_wait < 24'd1500) begin
                        poll_wait <= poll_wait + 24'd1;
                    end else begin
                        state <= ST_JEDEC_REQ;
                    end
                end

                ST_JEDEC_REQ: begin
                    if (!txn_busy) begin
                        txn_len   <= 3'd4;
                        txn_b0    <= FLASH_CMD_JEDEC_ID;
                        txn_b1    <= 8'h00;
                        txn_b2    <= 8'h00;
                        txn_b3    <= 8'h00;
                        txn_start <= 1'b1;
                        state     <= ST_JEDEC_WAIT;
                    end
                end

                ST_JEDEC_WAIT: begin
                    if (txn_done) begin
                        jedec_id_dbg    <= {txn_rx1, txn_rx2, txn_rx3};
                        jedec_valid_dbg <= 1'b1;
                        state           <= ST_IDLE;
                    end
                end

                ST_IDLE: begin
                    flash_busy <= 1'b0;

                    // 读取模式(read_mode=1)下，忽略所有写入请求，只处理读取
                    if (read_mode) begin
                        if (rd_req) begin
                            // 读取模式下最高优先级处理读取
                            flash_busy <= 1'b0; // 不阻塞外部
                            state <= ST_RD_REQ;
                        end
                        // read_mode=1 时忽略 erase_pending, param_write_pending, flush_pending 等
                    end else if (erase_pending) begin
                        flash_error <= 1'b0;
                        err_code_dbg <= 4'h0;
                        flash_busy <= 1'b1;
                        state <= ST_ERASE_ALL_WREN;
                    end else if (param_write_pending) begin
                        flash_error <= 1'b0;
                        err_code_dbg <= 4'h0;
                        flash_busy <= 1'b1;
                        state <= ST_PARAM_WRITE;
                    end else if (table_write) begin
                        flash_error <= 1'b0;
                        err_code_dbg <= 4'h0;
                        flash_busy <= 1'b1;
                        table_addr <= ADDR_TABLE_BASE + (table_entry * TABLE_ENTRY_SIZE);
                        table_byte_cnt <= 6'd0;
                        state <= ST_TABLE_WRITE;
                    end else if (rd_req) begin
                        // 单字节读取请求（最高优先级之一，低于擦除/参数写入）
                        flash_busy <= 1'b0; // 不阻塞外部
                        state <= ST_RD_REQ;
                    end else if (flush_pending && !fifo_empty_flag) begin
                        // 仅在收到flush触发后再搬运FIFO，避免每来1个样本就占用Flash总线
                        flash_error <= 1'b0;
                        flash_busy <= 1'b1;
                        state <= ST_CHECK_NEED_ERASE;
                    end else if (flush_pending && fifo_empty_flag) begin
                        // flush触发时若FIFO已空，直接完成本次流程
                        flash_error <= 1'b0;
                        flash_busy <= 1'b1;
                        state <= ST_DONE;
                    end else begin
                        state <= ST_IDLE;
                    end
                end

                ST_RD_REQ: begin
                    // 发 READ(0x03) + addr + dummy，共5字节
                    if (!txn_busy) begin
                        txn_len   <= 3'd5;
                        txn_b0    <= FLASH_CMD_READ;
                        txn_b1    <= rd_addr[23:16];
                        txn_b2    <= rd_addr[15:8];
                        txn_b3    <= rd_addr[7:0];
                        txn_b4    <= 8'hFF; // 提供dummy以移出数据
                        txn_start <= 1'b1;
                        state     <= ST_RD_WAIT;
                    end
                end

                ST_RD_WAIT: begin
                    if (txn_done) begin
                        rd_data       <= txn_rx4;
                        rd_data_valid <= 1'b1;
                        cur_addr_dbg  <= rd_addr; // 锁存实际读取地址
                        // 必须在此处等待上层撤销请求，否则下一拍又会进入 ST_RD_REQ
                        state         <= ST_RD_DONE;
                    end
                end

                ST_RD_DONE: begin
                    if (!rd_req) begin
                        state <= ST_IDLE;
                    end
                end

                // ====================================================================
                // 增强模式：参数写入
                // ====================================================================
                ST_PARAM_WRITE: begin
                    if (!txn_busy) begin
                        cur_addr <= param_addr_latched;
                        state <= ST_CHECK_NEED_ERASE;
                    end
                end

                // ====================================================================
                // 地址映射表操作
                // ====================================================================
                ST_TABLE_WRITE: begin
                    if (!txn_busy) begin
                        txn_len   <= 3'd1;
                        txn_b0    <= FLASH_CMD_WREN;
                        txn_start <= 1'b1;
                        state     <= ST_TABLE_WAIT;
                    end
                end

                ST_TABLE_READ: begin
                    if (!txn_busy) begin
                        txn_len   <= 3'd5;
                        txn_b0    <= FLASH_CMD_READ;
                        txn_b1    <= table_addr[23:16];
                        txn_b2    <= table_addr[15:8];
                        txn_b3    <= table_addr[7:0];
                        txn_b4    <= 8'hFF;
                        txn_start <= 1'b1;
                        state     <= ST_TABLE_WAIT;
                    end
                end

                ST_TABLE_WAIT: begin
                    if (txn_done) begin
                        table_data <= txn_rx4;
                        table_valid <= 1'b1;
                        state <= ST_DONE;
                    end
                end

                // ====================================================================
                // 共享Flash操作状态
                // ====================================================================
                ST_CHECK_NEED_ERASE: begin
                    if (param_write_pending) begin
                        if (need_erase) begin
                            state <= ST_WREN_ERASE;
                        end else begin
                            state <= ST_WREN_PROG;
                        end
                    end else if (need_erase) begin
                        state <= ST_WREN_ERASE;
                    end else begin
                        state <= ST_CHECK_FIFO;
                    end
                end

                ST_WREN_ERASE: begin
                    if (!txn_busy) begin
                        txn_len            <= 3'd1;
                        txn_b0             <= FLASH_CMD_WREN;
                        txn_start          <= 1'b1;
                        erase_wren_checked <= 1'b0;
                        erase_poll_issued  <= 1'b0;
                        if (!erase_wren_checked) poll_attempts <= 16'd0;
                        state              <= ST_SEND_ERASE;
                    end
                end

                ST_SEND_ERASE: begin
                    if (txn_done) begin
                        if (!erase_wren_checked) begin
                            // 先读SR1确认擦除前WREN已生效（WEL=1）
                            txn_len            <= 3'd2;
                            txn_b0             <= FLASH_CMD_RDSR1;
                            txn_b1             <= 8'h00;
                            txn_start          <= 1'b1;
                            erase_wren_checked <= 1'b1;
                        end else if ((txn_rx1 & 8'h02) == 8'h00) begin
                            last_sr_dbg       <= txn_rx1;
                            sr_stage_dbg      <= 3'd3;
                            poll_attempts_dbg <= poll_attempts;
                            // WEL未置位，重做WREN，带超时保护
                            if (poll_attempts >= POLL_ERASE_MAX) begin
                                flash_error  <= 1'b1;
                                err_code_dbg <= 4'h1;
                                state <= ST_DONE;
                            end else begin
                                poll_attempts <= poll_attempts + 16'd1;
                                state <= ST_WREN_ERASE;
                            end
                        end else begin
                            last_sr_dbg       <= txn_rx1;
                            sr_stage_dbg      <= 3'd3;
                            poll_attempts_dbg <= poll_attempts;
                            txn_len   <= 3'd4;
                            txn_b0    <= FLASH_CMD_SE;
                            txn_b1    <= active_sector_base[23:16];
                            txn_b2    <= active_sector_base[15:8];
                            txn_b3    <= active_sector_base[7:0];
                            txn_start <= 1'b1;
                            erased_sector_base <= active_sector_base;
                            poll_attempts      <= 16'd0;
                            poll_wait          <= 24'd0;
                            erase_poll_issued  <= 1'b0;
                            state              <= ST_POLL_ERASE;
                        end
                    end
                end

                ST_POLL_ERASE: begin
                    if (txn_done) begin
                        if (erase_poll_issued) begin
                            last_sr_dbg       <= txn_rx1;
                            sr_stage_dbg      <= 3'd4;
                            poll_attempts_dbg <= poll_attempts;
                            if ((txn_rx1 & 8'h01) == 8'h00) begin
                                // WIP=0，扇区擦除完成
                                if (param_write_pending) begin
                                    state <= ST_WREN_PROG;
                                end else begin
                                    state <= ST_CHECK_FIFO;
                                end
                            end else begin
                                poll_wait <= 24'd0;
                            end
                        end else begin
                            // 这是SE事务完成，不是RDSR返回
                            poll_wait <= 24'd0;
                        end
                    end else if (!txn_busy) begin
                        if (poll_attempts >= POLL_ERASE_MAX) begin
                            flash_error   <= 1'b1;
                            err_code_dbg  <= 4'h1;
                            state <= ST_DONE;
                        end else if (poll_wait < POLL_ERASE_WAIT_CYC) begin
                            poll_wait <= poll_wait + 24'd1;
                        end else begin
                            poll_attempts    <= poll_attempts + 16'd1;
                            erase_poll_issued<= 1'b1;
                            txn_len          <= 3'd2;
                            txn_b0           <= FLASH_CMD_RDSR1;
                            txn_b1           <= 8'h00;
                            txn_start        <= 1'b1;
                            poll_wait        <= 24'd0;
                        end
                    end
                end

                ST_CHECK_FIFO: begin
                    if (cur_addr >= DIFF_DATA_END) begin
                        state <= ST_DONE;
                    end else if (!fifo_empty_flag) begin
                        // FIFO有数据，继续读取
                        state <= ST_FIFO_RE;
                    end else if (collect_stop || dump_flush_force) begin
                        // 有外部停止/flush信号时，保持本状态等待数据或超时
                        state <= ST_CHECK_FIFO;
                    end else begin
                        // 无数据且无外部信号时，可以结束
                        state <= ST_DONE;
                    end
                end

                ST_FIFO_RE: begin
                    fifo_re        <= 1'b1;
                    fifo_read_pend <= 2'd3; // fifo_re 为时序输出，需要额外1拍让数据稳定
                    state          <= ST_FIFO_LATCH;
                end

                ST_FIFO_LATCH: begin
                    if (fifo_read_pend == 0) begin
                        calc_checksum <= calc_checksum + fifo_data_latched[7:0];
                        prog_word <= fifo_data_latched;
                        last_prog_word_mirror <= fifo_data_latched; // 最近一次写入Flash的32位字镜像
                        cur_addr_dbg          <= cur_addr; // 锁存写入地址
                        prog_byte_idx <= 2'd0;
                        state <= ST_WREN_PROG;
                    end
                end

                ST_WREN_PROG: begin
                    if (!txn_busy) begin
                        txn_len            <= 3'd1;
                        txn_b0             <= FLASH_CMD_WREN;
                        txn_start          <= 1'b1;
                        prog_wren_checked  <= 1'b0;
                        if (!prog_wren_checked) poll_attempts <= 16'd0;
                        state              <= ST_SEND_PROG;
                    end
                end

                ST_SEND_PROG: begin
                    if (txn_done) begin
                        if (!prog_wren_checked) begin
                            // 先读SR1确认WREN已生效（WEL=1）
                            txn_len           <= 3'd2;
                            txn_b0            <= FLASH_CMD_RDSR1;
                            txn_b1            <= 8'h00;
                            txn_start         <= 1'b1;
                            prog_wren_checked <= 1'b1;
                        end else if ((txn_rx1 & 8'h02) == 8'h00) begin
                            last_sr_dbg       <= txn_rx1;
                            sr_stage_dbg      <= 2'd1;
                            poll_attempts_dbg <= poll_attempts;
                            // WEL未置位，重做WREN，带超时保护
                            if (poll_attempts >= POLL_PROG_MAX) begin
                                flash_error  <= 1'b1;
                                err_code_dbg <= 4'h2;
                                state <= ST_DONE;
                            end else begin
                                poll_attempts <= poll_attempts + 16'd1;
                                state <= ST_WREN_PROG;
                            end
                        end else begin
                            last_sr_dbg       <= txn_rx1;
                            sr_stage_dbg      <= 2'd1;
                            poll_attempts_dbg <= poll_attempts;
                            // PP命令：一次写1字节，需要4次PP完成一个32位字
                            txn_len   <= 3'd5;
                            txn_b0    <= FLASH_CMD_PP;
                            txn_b1    <= cur_addr[23:16];
                            txn_b2    <= cur_addr[15:8];
                            txn_b3    <= cur_addr[7:0];

                            // 修复：将复杂的嵌套条件选择拆分，以降低MUX综合复杂度，解决布线冲突
                            if (param_write_pending) begin
                                txn_b4 <= param_data_latched;
                                cur_addr_dbg <= cur_addr;
                                last_prog_word_mirror <= {24'd0, param_data_latched};
                            end else if (prog_byte_idx == 2'd0) begin
                                txn_b4 <= prog_word[31:24];
                            end else if (prog_byte_idx == 2'd1) begin
                                txn_b4 <= prog_word[23:16];
                            end else if (prog_byte_idx == 2'd2) begin
                                txn_b4 <= prog_word[15:8];
                            end else begin
                                txn_b4 <= prog_word[7:0];
                            end

                            txn_start <= 1'b1;
                            poll_attempts   <= 16'd0;
                            poll_wait       <= 24'd0;
                            prog_poll_issued<= 1'b0;
                            state           <= ST_POLL_PROG;
                        end
                    end
                end

                ST_POLL_PROG: begin
                    // PP命令完成后，必须持续轮询RDSR1直到WIP=0
                    // 去掉 poll_attempts>0 的退出限制，改为“至少发起一次轮询”+超时保护
                    if (txn_done) begin
                        if (prog_poll_issued) begin
                            last_sr_dbg       <= txn_rx1;
                            sr_stage_dbg      <= 3'd2;
                            poll_attempts_dbg <= poll_attempts;
                            if ((txn_rx1 & 8'h01) == 8'h00) begin
                                // WIP=0，本字节编程完成
                                state <= ST_INC_ADDR;
                            end else begin
                                // WIP=1，继续轮询
                                poll_wait <= 24'd0;
                            end
                        end else begin
                            // 这是PP事务完成，不是RDSR返回
                            poll_wait <= 24'd0;
                        end
                    end else if (!txn_busy) begin
                        if (poll_attempts >= POLL_PROG_MAX) begin
                            flash_error  <= 1'b1;
                            err_code_dbg <= 4'h2;
                            state <= ST_DONE;
                        end else if (poll_wait < POLL_PROG_WAIT_CYC) begin
                            poll_wait <= poll_wait + 24'd1;
                        end else begin
                            poll_attempts    <= poll_attempts + 16'd1;
                            prog_poll_issued <= 1'b1;
                            txn_len          <= 3'd2;
                            txn_b0           <= FLASH_CMD_RDSR1;
                            txn_b1           <= 8'h00;
                            txn_start        <= 1'b1;
                            poll_wait        <= 24'd0;
                        end
                    end
                end

                ST_INC_ADDR: begin
                    if (param_write_pending) begin
                        param_write_pending <= 1'b0;
                        state <= ST_DONE;
                    end else begin
                        // 逐字节写入，每次PP后地址递增1
                        if (prog_byte_idx < 2'd3) begin
                            prog_byte_idx <= prog_byte_idx + 2'd1;
                            cur_addr <= cur_addr + 24'd1;
                            state <= ST_WREN_PROG;
                        end else begin
                            // 4字节全部写完，重置prog_byte_idx，准备下一个32位字
                            prog_byte_idx <= 2'd0;
                            cur_addr <= cur_addr + 24'd1;
                            state <= ST_CHECK_NEED_ERASE;
                        end
                    end
                end

                ST_DONE: begin
                    flush_pending       <= 1'b0;
                    param_write_pending <= 1'b0;
                    prog_poll_issued    <= 1'b0;
                    prog_wren_checked   <= 1'b0;
                    erase_poll_issued   <= 1'b0;
                    erase_wren_checked  <= 1'b0;
                    erase_pending       <= 1'b0;  // 新增：超时或完成后清除擦除挂起
                    sr_stage_dbg        <= 3'd0;
                    flash_busy          <= 1'b0;
                    flash_done          <= 1'b1;
                    write_done          <= !flash_error;  // 出错时不标记写入完成
                    state               <= ST_IDLE;
                end

                ST_ERASE_ALL_WREN: begin
                    if (!txn_busy) begin
                        txn_len   <= 3'd1;
                        txn_b0    <= FLASH_CMD_WREN;
                        txn_start <= 1'b1;
                        state     <= ST_ERASE_ALL_SE;
                    end
                end

                ST_ERASE_ALL_SE: begin
                    // SE命令：WREN完成后发送扇区擦除
                    if (txn_done) begin
                        txn_len   <= 3'd4;
                        txn_b0    <= FLASH_CMD_SE;
                        txn_b1    <= erase_sector_cur[23:16];
                        txn_b2    <= erase_sector_cur[15:8];
                        txn_b3    <= erase_sector_cur[7:0];
                        txn_start <= 1'b1;
                        poll_wait <= 24'd0;
                        poll_attempts <= 16'd0;
                        state     <= ST_ERASE_ALL_POLL;
                    end
                end

                ST_ERASE_ALL_POLL: begin
                    if (txn_done) begin
                        last_sr_dbg       <= txn_rx1;  // 每次RDSR1完成都更新SR调试
                        sr_stage_dbg      <= 3'd4;
                        poll_attempts_dbg <= poll_attempts;
                        if (poll_attempts > 0 && (txn_rx1 & 8'h01) == 8'h00) begin
                            // WIP=0，擦除完成
                            state <= ST_ERASE_ALL_NEXT;
                        end else if (poll_attempts >= POLL_ERASE_MAX) begin
                            // 超时强制退出，避免卡死
                            flash_error   <= 1'b1;
                            err_code_dbg  <= 4'h1;  // 擦除超时错误
                            state <= ST_ERASE_ALL_DONE;
                        end else begin
                            // SE刚完成(poll_attempts=0)或WIP仍为1，继续等
                            poll_wait <= 24'd0;
                        end
                    end else if (!txn_busy) begin
                        if (poll_wait < 24'd50000) begin  // 等1ms再首次轮询（W25Q32最快10ms）
                            poll_wait <= poll_wait + 24'd1;
                        end else if (poll_attempts >= POLL_ERASE_MAX) begin
                            // 超时强制退出
                            flash_error   <= 1'b1;
                            err_code_dbg  <= 4'h1;
                            state <= ST_ERASE_ALL_DONE;
                        end else begin
                            poll_attempts <= poll_attempts + 16'd1;
                            txn_len   <= 3'd2;
                            txn_b0    <= FLASH_CMD_RDSR1;
                            txn_b1    <= 8'h00;
                            txn_start <= 1'b1;
                            poll_wait <= 24'd0;
                        end
                    end
                end

                ST_ERASE_ALL_NEXT: begin
                    // 每个扇区 4KB = 0x1000，全擦除流程需要覆盖：
                    //   0x000000 地址表区
                    //   0x100000-0x3DFFFF 差值区
                    //   0x3E0000 动态参数区
                    //   0x3F0000 元数据区
                    if (erase_sector_cur + 24'h1000 <= ERASE_ALL_END) begin
                        erase_sector_cur <= erase_sector_cur + 24'h1000;
                        state <= ST_ERASE_ALL_WREN;
                    end else begin
                        state <= ST_ERASE_ALL_DONE;
                    end
                end

                ST_ERASE_ALL_DONE: begin
                    erase_pending      <= 1'b0;
                    cur_addr           <= DIFF_DATA_BASE; // 擦除后地址归零
                    erased_sector_base <= 24'hFFFFFF;     // 重置擦除标记
                    flash_busy         <= 1'b0;
                    flash_done         <= 1'b1;
                    erase_done         <= 1'b1;           // 标记擦除完成
                    state              <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

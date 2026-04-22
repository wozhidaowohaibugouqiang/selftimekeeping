// Flash控制模块
// 模块功能：负责Flash擦除、编程、校验及Boot读取，调用spi_driver实现底层通信
// 设计作者：TraeAI
// 设计日期：2026-01-21
// 版本号：v1.0
//
// 输入端口：
// - sys_clk：系统时钟
// - sys_rst_n：系统复位
// - collect_stop：采集停止信号
// - fifo_dout：FIFO数据输出
// - fifo_empty_flag：FIFO空标志
// - fifo_full_flag：FIFO满标志
// - spi_miso：SPI MISO输入
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
//
// 设计说明：
// 1. 实例化spi_driver.v处理SPI底层时序
// 2. 实现Flash操作主状态机（20个状态）
// 3. 严格保留原flash_spi.v的逻辑

`timescale 1ns / 1ps

module flash_controller (
    input           sys_clk,
    input           sys_rst_n,
    input           collect_stop,

    output reg      fifo_re,
    input   [7:0]   fifo_dout,
    input           fifo_empty_flag,
    input           fifo_full_flag,

    // SPI物理接口（输出给顶层）
    output          spi_sclk,
    output          spi_mosi,
    input           spi_miso,
    output          spi_cs_n,

    output reg      flash_busy,
    output reg      flash_done,
    output reg      flash_error,

    output reg [7:0] hist_diff,
    output reg       hist_diff_valid
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

    // 实例化 SPI Driver
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
    // Flash 控制逻辑 (原 flash_spi.v 逻辑)
    // ========================================================================

    localparam [7:0] FLASH_CMD_WREN      = 8'h06;
    localparam [7:0] FLASH_CMD_RDSR1     = 8'h05;
    localparam [7:0] FLASH_CMD_SE        = 8'h20;
    localparam [7:0] FLASH_CMD_PP        = 8'h02;
    localparam [7:0] FLASH_CMD_READ      = 8'h03;

    reg         flush_pending;
    reg [23:0]  cur_addr;
    reg [23:0]  erased_sector_base;
    reg [7:0]   fifo_data_latched;
    reg [1:0]   fifo_read_pend;
    reg [15:0]  poll_attempts;
    reg [23:0]  poll_wait;
    reg [7:0]   last_status;

    wire [23:0] sector_base = {cur_addr[23:12], 12'h000};
    wire        need_erase = (erased_sector_base != sector_base);

    localparam [23:0] DIFF_ADDR_START = 24'h001000;
    localparam [23:0] DIFF_ADDR_END   = 24'h00FFFF;

    localparam [5:0]
        ST_IDLE             = 6'd0,
        ST_CHECK_TRIGGER    = 6'd1,
        ST_CHECK_NEED_ERASE = 6'd2,
        ST_WREN_ERASE       = 6'd3,
        ST_SEND_ERASE       = 6'd4,
        ST_POLL_ERASE       = 6'd5,
        ST_CHECK_FIFO       = 6'd6,
        ST_FIFO_RE          = 6'd7,
        ST_FIFO_LATCH       = 6'd8,
        ST_WREN_PROG        = 6'd9,
        ST_SEND_PROG        = 6'd10,
        ST_POLL_PROG        = 6'd11,
        ST_INC_ADDR         = 6'd12,
        ST_DONE             = 6'd13,
        ST_BOOT_READ        = 6'd14,
        ST_BOOT_WAIT        = 6'd15,
        ST_VERIFY_START     = 6'd16,
        ST_VERIFY_READ      = 6'd17,
        ST_VERIFY_WAIT      = 6'd18,
        ST_VERIFY_CHECK     = 6'd19;

    reg [5:0] state;

    reg [7:0]   calc_checksum;
    reg [7:0]   read_checksum;
    reg [23:0]  verify_addr;

    // Flash操作主控制状态机
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            // 系统复位时，初始化主控制状态机
            state               <= ST_BOOT_READ;          // 初始状态：上电读取差值区起始地址
            fifo_re             <= 1'b0;                  // FIFO读使能初始化为低
            flush_pending       <= 1'b0;                  // 刷新挂起标志初始化为低
            cur_addr            <= DIFF_ADDR_START;       // 当前Flash地址初始化为差值区起始地址
            erased_sector_base  <= 24'hFFFFFF;            // 已擦除扇区基地址初始化为全1
            fifo_data_latched   <= 8'h00;                 // FIFO数据锁存初始化为0
            fifo_read_pend      <= 2'd0;                  // FIFO读取挂起初始化为低
            poll_attempts       <= 16'd0;                 // 轮询尝试次数初始化为0
            poll_wait           <= 24'd0;                 // 轮询等待计数器初始化为0
            last_status         <= 8'h00;                 // 最后读取的Flash状态初始化为0
            
            calc_checksum       <= 8'h00;
            read_checksum       <= 8'h00;
            verify_addr         <= DIFF_ADDR_START;

            txn_start           <= 1'b0;                  // 事务启动信号初始化为低
            txn_len             <= 3'd0;                  // 事务长度初始化为0
            txn_b0              <= 8'h00;                 // 事务字节0初始化为0
            txn_b1              <= 8'h00;                 // 事务字节1初始化为0
            txn_b2              <= 8'h00;                 // 事务字节2初始化为0
            txn_b3              <= 8'h00;                 // 事务字节3初始化为0
            txn_b4              <= 8'h00;                 // 事务字节4初始化为0

            flash_busy          <= 1'b0;                  // Flash忙标志初始化为低
            flash_done          <= 1'b0;                  // Flash操作完成标志初始化为低
            flash_error         <= 1'b0;                  // Flash操作错误标志初始化为低

            hist_diff           <= 8'h00;
            hist_diff_valid     <= 1'b0;
        end else begin
            // 清除边沿触发信号
            fifo_re     <= 1'b0;                          // 清除FIFO读使能（脉冲信号）
            txn_start   <= 1'b0;                          // 清除事务启动信号（脉冲信号）
            flash_done  <= 1'b0;                          // 清除Flash完成标志（脉冲信号）
            hist_diff_valid <= 1'b0;

            if (fifo_read_pend != 2'd0) begin
                fifo_read_pend <= fifo_read_pend - 2'd1;
                if (fifo_read_pend == 2'd1) begin
                    fifo_data_latched <= fifo_dout;
                end
            end

            if (!flush_pending && (fifo_full_flag || collect_stop)) begin
                flush_pending <= 1'b1;                    // 设置刷新挂起标志
                calc_checksum <= 8'h00;                   // Reset checksum
                read_checksum <= 8'h00;
            end

            // 主控制状态机
            case (state)
                ST_BOOT_READ: begin
                    flash_busy <= 1'b0;
                    if (cur_addr >= DIFF_ADDR_END) begin
                         $display("[%0t] FLASH_SPI: BOOT READ END (Addr Limit)", $time);
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
                        if (txn_rx4 != 8'hFF) begin
                            $display("[%0t] FLASH_SPI: READ VALID addr=%h data=%h", $time, cur_addr, txn_rx4);
                            hist_diff       <= txn_rx4;
                            hist_diff_valid <= 1'b1;
                            cur_addr        <= cur_addr + 24'd1;
                            state           <= ST_BOOT_READ;
                        end else begin
                            $display("[%0t] FLASH_SPI: READ INVALID (FF) addr=%h - BOOT COMPLETE", $time, cur_addr);
                            state           <= ST_IDLE;
                            cur_addr        <= DIFF_ADDR_START; // Reset for writing
                        end
                    end
                end

                ST_IDLE: begin                             // 空闲状态
                    flash_busy <= 1'b0;                    // 清除Flash忙标志
                    // 检查是否需要开始Flash操作
                    if (flush_pending && !fifo_empty_flag) begin
                        flash_busy <= 1'b1;                // 设置Flash忙标志
                        state      <= ST_CHECK_NEED_ERASE; // 进入检查擦除需求状态
                    end else if (flush_pending && fifo_empty_flag) begin
                        state <= ST_VERIFY_START;          // FIFO为空，Go to Verify
                    end else begin
                        state <= ST_CHECK_TRIGGER;         // 进入检查触发状态
                    end
                end

                ST_CHECK_TRIGGER: begin                    // 检查触发状态
                    // 等待触发条件满足
                    if (flush_pending && !fifo_empty_flag) begin
                        flash_busy <= 1'b1;                // 设置Flash忙标志
                        state      <= ST_CHECK_NEED_ERASE; // 进入检查擦除需求状态
                    end else if (flush_pending && fifo_empty_flag) begin
                        state <= ST_VERIFY_START;          // FIFO为空，Go to Verify
                    end else begin
                        state <= ST_CHECK_TRIGGER;         // 保持当前状态，继续等待
                    end
                end

                ST_CHECK_NEED_ERASE: begin                 // 检查擦除需求状态
                    // 根据当前地址和已擦除扇区基地址判断是否需要擦除
                    if (need_erase) begin
                        state <= ST_WREN_ERASE;            // 需要擦除，进入擦除写使能状态
                    end else begin
                        state <= ST_CHECK_FIFO;            // 不需要擦除，进入检查FIFO状态
                    end
                end

                ST_WREN_ERASE: begin                       // 擦除写使能状态
                    if (!txn_busy) begin
                        txn_len   <= 3'd1;
                        txn_b0    <= FLASH_CMD_WREN;       // 发送写使能命令
                        txn_start <= 1'b1;
                        state     <= ST_SEND_ERASE;        // 进入发送擦除命令状态
                    end
                end

                ST_SEND_ERASE: begin                       // 发送擦除命令状态
                    if (txn_done) begin
                        if (!txn_busy) begin
                            txn_len   <= 3'd4;
                            txn_b0    <= FLASH_CMD_SE;     // 发送扇区擦除命令
                            txn_b1    <= sector_base[23:16];
                            txn_b2    <= sector_base[15:8];
                            txn_b3    <= sector_base[7:0];
                            txn_start <= 1'b1;
                            
                            erased_sector_base <= sector_base; // 更新已擦除扇区基地址
                            poll_attempts      <= 16'd0;       // 重置轮询尝试次数
                            state              <= ST_POLL_ERASE; // 进入轮询擦除状态
                        end
                    end
                end

                ST_POLL_ERASE: begin                       // 轮询擦除状态
                    if (txn_done) begin
                         if ((txn_rx1 & 8'h01) == 8'h00) begin // WIP = 0
                             state <= ST_CHECK_FIFO;
                         end else begin
                             // 仍在忙，继续轮询
                             poll_attempts <= poll_attempts + 16'd1;
                             poll_wait     <= 24'd0;
                         end
                    end else if (!txn_busy) begin
                        if (poll_wait < 24'd1000) begin    // 等待一小段时间
                            poll_wait <= poll_wait + 24'd1;
                        end else begin
                            txn_len   <= 3'd2;
                            txn_b0    <= FLASH_CMD_RDSR1; // 发送读状态寄存器1命令
                            txn_b1    <= 8'h00;
                            txn_start <= 1'b1;
                        end
                    end
                end

                ST_CHECK_FIFO: begin                       // 检查FIFO状态
                    if (!fifo_empty_flag) begin
                        state <= ST_FIFO_RE;               // FIFO非空，进入FIFO读使能状态
                    end else begin
                        state <= ST_VERIFY_START;          // FIFO为空，进入校验开始状态
                    end
                end

                ST_FIFO_RE: begin                          // FIFO读使能状态
                    fifo_re        <= 1'b1;                // 产生FIFO读脉冲
                    fifo_read_pend <= 2'd2;                // 设置读取挂起，等待数据有效
                    state          <= ST_FIFO_LATCH;       // 进入FIFO锁存状态
                end

                ST_FIFO_LATCH: begin                       // FIFO锁存状态
                    if (fifo_read_pend == 0) begin
                        calc_checksum <= calc_checksum + fifo_data_latched; // 累加校验和
                        state         <= ST_WREN_PROG;     // 进入编程写使能状态
                    end
                end

                ST_WREN_PROG: begin                        // 编程写使能状态
                    if (!txn_busy) begin
                        txn_len   <= 3'd1;
                        txn_b0    <= FLASH_CMD_WREN;       // 发送写使能命令
                        txn_start <= 1'b1;
                        state     <= ST_SEND_PROG;         // 进入发送编程命令状态
                    end
                end

                ST_SEND_PROG: begin                        // 发送编程命令状态
                    if (txn_done) begin
                        if (!txn_busy) begin
                            txn_len   <= 3'd5;
                            txn_b0    <= FLASH_CMD_PP;     // 发送页编程命令
                            txn_b1    <= cur_addr[23:16];
                            txn_b2    <= cur_addr[15:8];
                            txn_b3    <= cur_addr[7:0];
                            txn_b4    <= fifo_data_latched;
                            txn_start <= 1'b1;
                            
                            poll_attempts <= 16'd0;        // 重置轮询尝试次数
                            poll_wait     <= 24'd0;        // 重置等待计数
                            state         <= ST_POLL_PROG; // 进入轮询编程状态
                        end
                    end
                end

                ST_POLL_PROG: begin                        // 轮询编程状态
                    if (txn_done) begin
                        if ((txn_rx1 & 8'h01) == 8'h00) begin // WIP = 0
                            state <= ST_INC_ADDR;
                        end else begin
                            poll_wait <= 24'd0;
                        end
                    end else if (!txn_busy) begin
                        if (poll_wait < 24'd500) begin     // 编程等待时间较短 (~10us @ 50MHz)
                            poll_wait <= poll_wait + 24'd1;
                        end else begin
                            txn_len   <= 3'd2;
                            txn_b0    <= FLASH_CMD_RDSR1; // 发送读状态寄存器1命令
                            txn_b1    <= 8'h00;
                            txn_start <= 1'b1;
                        end
                    end
                end

                ST_INC_ADDR: begin                         // 地址递增状态
                    cur_addr <= cur_addr + 24'd1;          // 地址加1
                    state    <= ST_CHECK_NEED_ERASE;       // 回到检查擦除需求状态（也即检查下一个数据）
                end

                ST_VERIFY_START: begin
                    verify_addr <= DIFF_ADDR_START;
                    read_checksum <= 8'h00;
                    state <= ST_VERIFY_READ;
                end

                ST_VERIFY_READ: begin
                    if (verify_addr < cur_addr) begin
                         if (!txn_busy) begin
                            txn_len   <= 3'd5;
                            txn_b0    <= FLASH_CMD_READ;
                            txn_b1    <= verify_addr[23:16];
                            txn_b2    <= verify_addr[15:8];
                            txn_b3    <= verify_addr[7:0];
                            txn_b4    <= 8'hFF; // Dummy
                            txn_start <= 1'b1;
                            state     <= ST_VERIFY_WAIT;
                        end
                    end else begin
                        state <= ST_VERIFY_CHECK;
                    end
                end

                ST_VERIFY_WAIT: begin
                    if (txn_done) begin
                        read_checksum <= read_checksum + txn_rx4;
                        verify_addr   <= verify_addr + 24'd1;
                        state         <= ST_VERIFY_READ;
                    end
                end

                ST_VERIFY_CHECK: begin
                    if (read_checksum == calc_checksum) begin
                        $display("[%0t] FLASH: Checksum Match! %h", $time, calc_checksum);
                        flash_error <= 1'b0;
                    end else begin
                        $display("[%0t] FLASH: Checksum MISMATCH! Calc=%h Read=%h", $time, calc_checksum, read_checksum);
                        flash_error <= 1'b1;
                    end
                    state <= ST_DONE;
                end

                ST_DONE: begin                             // 完成状态
                    flush_pending <= 1'b0;                 // 清除刷新挂起标志
                    flash_busy    <= 1'b0;                 // 清除Flash忙标志
                    flash_done    <= 1'b1;                 // 设置Flash完成标志
                    state         <= ST_IDLE;              // 回到空闲状态
                end

                default: begin
                    state <= ST_IDLE;                      // 异常状态下回到空闲状态
                end
            endcase
        end
    end

endmodule

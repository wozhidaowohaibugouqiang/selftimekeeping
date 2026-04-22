`timescale 1ns / 1ps

// 增强版自主守时系统顶层模块
// 模块功能：整合所有新增模块，实现论文要求的完整功能
// 设计作者：TraeAI
// 设计日期：2026-03-10
// 版本号：v2.0
//
// 新增功能：
// 1. 晶振上电延时稳定（100秒）
// 2. 长周期计数（256秒）
// 3. 动态补偿系数生成
// 4. 反向补偿系数管理（老化补偿）
// 5. 参数管理与Flash存储
//
module top_enhanced (
    input  wire         clk,            // 25MHz 系统时钟
    input  wire         rst_n,          // 系统复位
    input  wire         gps_pps,        // GPS PPS 信号
    input  wire         spi_miso,       // Flash SPI MISO
    input  wire         uart_rx,        // UART 接收
    
    output wire         uart_tx,        // UART 发送
    output wire         dpps_out,       // 补偿后的 DPPS 输出
    output wire         spi_sclk,       // Flash SPI Clock
    output wire         spi_mosi,       // Flash SPI MOSI
    output wire         spi_cs_n,       // Flash SPI CS_n
    output wire         uart_led,       // 串口传输状态指示灯
    output wire         gps_led         // GPS PPS信号接收指示灯
);

    // ========================================================================
    // 参数定义
    // ========================================================================
    parameter integer CLK_CYCLES_PER_SEC = 25_000_000;
    parameter integer GPS_LOST_THRESHOLD = 50_000_000;

    // ========================================================================
    // 内部信号定义
    // ========================================================================

    // 状态机定义
    localparam [4:0] 
        ST_STANDBY         = 5'b00000,
        ST_WAIT_OSC        = 5'b00001,
        ST_CALC            = 5'b00010,
        ST_TRAIN           = 5'b00011,
        ST_TRAIN_DONE      = 5'b00100,
        ST_PARAM_UPDATE    = 5'b00101,
        ST_ERR             = 5'b00110;

    reg [4:0] sys_state;
    wire [3:0] uart_cmd;
    wire       uart_cmd_valid;
    wire       read_done;
    
    // 晶振稳定相关信号
    wire       osc_ready;
    wire       stabilized_rst_n;
    
    // 长周期计数相关信号
    wire [15:0]  period_number;
    wire signed [63:0] diff_total;
    wire signed [63:0] quotient;
    wire signed [63:0] remainder;
    wire        diff_valid;
    wire        period_done;
    
    // 动态补偿系数相关信号
    wire        coeff_data_out;
    wire        coeff_data_valid;
    wire        coeff_ready;
    reg [7:0]   coeff_read_addr;
    reg         coeff_read_en;
    
    // 反向补偿系数相关信号
    wire signed [31:0] aging_comp_value;
    wire        aging_coeff_valid;
    reg         coeff_update_en;
    reg signed [31:0] new_coeff_value;
    
    // 参数管理相关信号
    wire [1:0]  param_type;
    wire [15:0] param_version;
    wire [31:0] param_timestamp;
    wire [7:0]  param_data;
    wire        param_valid;
    wire [23:0] param_addr;
    wire        addr_table_valid;
    
    // GPS 信号同步与状态检测
    reg  [2:0] gps_pps_sync;
    wire       gps_pps_clean;
    reg  [31:0] gps_lost_cnt;
    reg        gps_lost;
    
    // Timekeeping Core 信号
    wire       core_dpps;
    wire       core_diff_sign;
    wire [6:0] core_diff_data;
    wire       core_diff_valid;
    
    // FIFO Write 信号
    wire [7:0] fifo_dout;
    wire       fifo_empty;
    wire       fifo_full;
    wire       fifo_re;
    wire       flush_req;
    
    // Flash SPI 信号
    wire       flash_spi_sclk;
    wire       flash_spi_mosi;
    wire       flash_spi_cs_n;
    wire       flash_busy;
    wire       flash_done;
    wire       flash_error;
    
    // 1Hz Tick 信号
    wire       sec_tick;
    
    // 补偿信号
    reg signed [31:0] total_compensation;
    reg [23:0] dpps_offset;
    
    // 指示灯控制信号
    reg         uart_led_reg;        // 串口指示灯状态
    reg         gps_led_reg;         // GPS指示灯状态
    reg [31:0]  uart_led_counter;    // 串口指示灯计数器（2秒周期）
    reg [31:0]  gps_led_counter;     // GPS指示灯计数器（1秒周期）
    reg         uart_tx_active;      // 串口传输活动标志
    reg         gps_pps_detected;    // GPS PPS信号检测标志

    // ========================================================================
    // 1. 晶振状态模块
    // ========================================================================
    osc_stabilizer u_osc_stabilizer (
        .clk              (clk),
        .sys_rst_n        (rst_n),
        .osc_ready        (osc_ready),
        .stabilized_rst_n (stabilized_rst_n)
    );

    // ========================================================================
    // 2. 长周期计数与差值运算模块
    // ========================================================================
    long_period_counter u_long_counter (
        .clk              (clk),
        .sys_rst_n        (stabilized_rst_n),
        .gps_pps          (gps_pps_clean),
        .osc_ready        (osc_ready),
        .period_number    (period_number),
        .diff_total       (diff_total),
        .quotient         (quotient),
        .remainder        (remainder),
        .diff_valid       (diff_valid),
        .period_done      (period_done)
    );

    // ========================================================================
    // 3. 动态补偿系数生成与存储模块
    // ========================================================================
    dynamic_comp_coeff u_dynamic_coeff (
        .clk              (clk),
        .sys_rst_n        (stabilized_rst_n),
        .remainder        (remainder),
        .remainder_valid  (diff_valid),
        .coeff_read_addr  (coeff_read_addr),
        .coeff_read_en    (coeff_read_en),
        .coeff_data_out   (coeff_data_out),
        .coeff_data_valid (coeff_data_valid),
        .coeff_ready      (coeff_ready)
    );

    // ========================================================================
    // 4. 反向补偿系数管理模块（老化补偿）
    // ========================================================================
    aging_comp_coeff u_aging_coeff (
        .clk              (clk),
        .sys_rst_n        (stabilized_rst_n),
        .period_number    (period_number),
        .period_done      (period_done),
        .coeff_update_en  (coeff_update_en),
        .new_coeff_value  (new_coeff_value),
        .aging_comp_value (aging_comp_value),
        .coeff_valid      (aging_coeff_valid)
    );

    // ========================================================================
    // 5. 参数管理模块
    // ========================================================================
    param_manager u_param_manager (
        .sys_clk          (clk),
        .sys_rst_n        (stabilized_rst_n),
        .diff_sign        (core_diff_sign),
        .diff_data        (core_diff_data),
        .diff_valid_flag  (core_diff_valid),
        .gps_valid        (!gps_lost),
        .param_type       (param_type),
        .param_version    (param_version),
        .param_timestamp  (param_timestamp),
        .param_data       (param_data),
        .param_valid      (param_valid),
        .param_addr       (param_addr),
        .addr_table_valid (addr_table_valid),
        .param_state      ()
    );

    // ========================================================================
    // GPS PPS 同步与防抖
    // ========================================================================
    always @(posedge clk or negedge stabilized_rst_n) begin
        if (!stabilized_rst_n) begin
            gps_pps_sync <= 3'b000;
        end else begin
            gps_pps_sync <= {gps_pps_sync[1:0], gps_pps};
        end
    end
    assign gps_pps_clean = gps_pps_sync[2];

    // ========================================================================
    // 系统主状态机
    // ========================================================================
    always @(posedge clk or negedge stabilized_rst_n) begin
        if (!stabilized_rst_n) begin
            sys_state <= ST_STANDBY;
        end else begin
            if (uart_cmd_valid && uart_cmd != 4'd0) begin
                sys_state <= uart_cmd;
            end else begin
                case (sys_state)
                    ST_STANDBY: begin
                        if (osc_ready) sys_state <= ST_WAIT_OSC;
                    end
                    
                    ST_WAIT_OSC: begin
                        if (!gps_lost) sys_state <= ST_CALC;
                    end
                    
                    ST_CALC: begin
                        if (gps_lost) sys_state <= ST_TRAIN;
                        if (period_done) sys_state <= ST_PARAM_UPDATE;
                    end
                    
                    ST_PARAM_UPDATE: begin
                        sys_state <= ST_CALC;
                    end
                    
                    ST_TRAIN: begin
                        if (read_done) sys_state <= ST_TRAIN_DONE;
                    end
                    
                    default: begin
                        sys_state <= ST_STANDBY;
                    end
                endcase
            end
        end
    end

    // ========================================================================
    // GPS 丢失检测
    // ========================================================================
    always @(posedge clk or negedge stabilized_rst_n) begin
        if (!stabilized_rst_n) begin
            gps_lost_cnt <= 32'd0;
            gps_lost     <= 1'b0;
        end else begin
            if (gps_pps_clean) begin
                gps_lost_cnt <= 32'd0;
                gps_lost     <= 1'b0;
            end else begin
                if (gps_lost_cnt < GPS_LOST_THRESHOLD) begin
                    gps_lost_cnt <= gps_lost_cnt + 32'd1;
                end else begin
                    gps_lost     <= 1'b1;
                end
            end
        end
    end
    
    assign flush_req = (gps_lost || (sys_state != ST_CALC)) && (sys_state != ST_TRAIN);

    // ========================================================================
    // 补偿计算逻辑
    // ========================================================================
    always @(posedge clk or negedge stabilized_rst_n) begin
        if (!stabilized_rst_n) begin
            total_compensation <= 32'sd0;
            dpps_offset <= 24'd0;
            coeff_read_addr <= 8'd0;
            coeff_read_en <= 1'b0;
            coeff_update_en <= 1'b0;
            new_coeff_value <= 32'sd0;
        end else begin
            // 动态补偿系数读取
            if (sec_tick) begin
                coeff_read_addr <= coeff_read_addr + 8'd1;
                coeff_read_en <= 1'b1;
                
                // 计算总补偿：商 + 老化补偿 + 动态系数
                if (coeff_data_valid) begin
                    total_compensation <= quotient[31:0] + aging_comp_value + (coeff_data_out ? 32'sd1 : 32'sd0);
                    dpps_offset <= total_compensation[23:0];
                end
            end else begin
                coeff_read_en <= 1'b0;
            end
            
            // 在线更新老化系数
            if (period_done && diff_valid) begin
                coeff_update_en <= 1'b1;
                // 根据差值趋势调整老化系数
                new_coeff_value <= aging_comp_value + (diff_total[31:0] >>> 10);
            end else begin
                coeff_update_en <= 1'b0;
            end
        end
    end

    // ========================================================================
    // 现有模块实例化（保持兼容性）
    // ========================================================================
    
    // UART 模块
    wire        tx_en; // UART发送使能信号
    
    uart u_uart (
        .sys_clk    (clk),
        .sys_rst_n  (stabilized_rst_n),
        .uart_rx    (uart_rx),
        .uart_tx    (uart_tx),
        .sys_state  (sys_state),
        .pps_valid  (gps_pps_clean),
        .fifo_empty (fifo_empty),
        .fifo_full  (fifo_full),
        .train_done (sys_state == ST_TRAIN_DONE),
        .state_cmd  (uart_cmd),
        .cmd_valid  (uart_cmd_valid),
        .tx_en      (tx_en) // 输出UART发送使能信号
    );

    // 守时核心模块
    timekeeping_core u_core (
        .clk             (clk),
        .sys_rst_n       (stabilized_rst_n),
        .gps_pps         (gps_pps_clean),
        .dpps_offset     (dpps_offset),
        .init_diff       (8'd0),
        .init_diff_valid (1'b0),
        .dpps            (core_dpps),
        .diff_sign       (core_diff_sign),
        .diff_data       (core_diff_data),
        .diff_valid_flag (core_diff_valid)
    );
    assign dpps_out = core_dpps;

    // FIFO 写模块
    fifo_write_module u_fifo_write (
        .sys_clk_25mhz   (clk),
        .sys_rst_n       (stabilized_rst_n),
        .flush_req       (flush_req),
        .diff_sign       (core_diff_sign),
        .diff_data       (core_diff_data),
        .diff_valid_flag (core_diff_valid),
        .fifo_re         (fifo_re),
        .fifo_dout       (fifo_dout),
        .fifo_empty_flag (fifo_empty),
        .fifo_full_flag  (fifo_full)
    );

    // 增强版Flash控制器
    flash_controller_enhanced u_flash_enhanced (
        .sys_clk         (clk),
        .sys_rst_n       (stabilized_rst_n),
        .collect_stop    (flush_req),
        .fifo_re         (fifo_re),
        .fifo_dout       (fifo_dout),
        .fifo_empty_flag (fifo_empty),
        .fifo_full_flag  (fifo_full),
        .param_data      (param_data),
        .param_addr      (param_addr),
        .param_valid     (param_valid),
        .param_type      (param_type),
        .table_entry     (6'd0),
        .table_write     (1'b0),
        .table_data      (),
        .table_valid     (),
        .spi_sclk        (flash_spi_sclk),
        .spi_mosi        (flash_spi_mosi),
        .spi_miso        (spi_miso),
        .spi_cs_n        (flash_spi_cs_n),
        .flash_busy      (flash_busy),
        .flash_done      (flash_done),
        .flash_error     (flash_error),
        .hist_diff       (),
        .hist_diff_valid ()
    );

    // 1Hz 脉冲生成
    clk_1hz_gen u_sec_tick (
        .clk             (clk),
        .rst_n           (stabilized_rst_n),
        .sec_tick        (sec_tick)
    );

    // 均匀补偿模块
    avg_remain_comp u_avg_remain_comp (
        .clk               (clk),
        .rst_n             (stabilized_rst_n),
        .diff_update_valid (core_diff_valid),
        .diff_sign         (core_diff_sign),
        .diff_mag          (core_diff_data),
        .sec_tick          (sec_tick),
        .comp_valid        (),
        .comp_step         ()
    );

    // SPI 输出
    assign spi_sclk = flash_spi_sclk;
    assign spi_mosi = flash_spi_mosi;
    assign spi_cs_n = flash_spi_cs_n;
    
    // 指示灯输出
    assign uart_led = uart_led_reg;
    assign gps_led = gps_led_reg;
    
    // ========================================================================
    // 串口传输状态检测
    // ========================================================================
    // 检测UART发送状态
    always @(posedge clk or negedge stabilized_rst_n) begin
        if (!stabilized_rst_n) begin
            uart_tx_active <= 1'b0;
        end else begin
            // 当UART模块处于发送状态时，标记为活动
            if (tx_en) begin
                uart_tx_active <= 1'b1;
            end else if (sec_tick) begin
                // 1秒无活动则标记为非活动
                uart_tx_active <= 1'b0;
            end
        end
    end
    
    // ========================================================================
    // GPS PPS信号检测
    // ========================================================================
    always @(posedge clk or negedge stabilized_rst_n) begin
        if (!stabilized_rst_n) begin
            gps_pps_detected <= 1'b0;
        end else begin
            if (gps_pps_clean) begin
                gps_pps_detected <= 1'b1;
            end else if (gps_lost) begin
                gps_pps_detected <= 1'b0;
            end
        end
    end
    
    // ========================================================================
    // 串口指示灯控制逻辑（2秒周期，亮1秒灭1秒）
    // ========================================================================
    always @(posedge clk or negedge stabilized_rst_n) begin
        if (!stabilized_rst_n) begin
            uart_led_reg <= 1'b0;
            uart_led_counter <= 32'd0;
        end else begin
            if (uart_tx_active) begin
                if (uart_led_counter < CLK_CYCLES_PER_SEC) begin
                    // 亮1秒
                    uart_led_reg <= 1'b1;
                    uart_led_counter <= uart_led_counter + 32'd1;
                end else if (uart_led_counter < CLK_CYCLES_PER_SEC * 2) begin
                    // 灭1秒
                    uart_led_reg <= 1'b0;
                    uart_led_counter <= uart_led_counter + 32'd1;
                end else begin
                    // 重置计数器
                    uart_led_counter <= 32'd0;
                end
            end else begin
                // 无传输时指示灯灭
                uart_led_reg <= 1'b0;
                uart_led_counter <= 32'd0;
            end
        end
    end
    
    // ========================================================================
    // GPS指示灯控制逻辑（1秒周期，亮0.5秒灭0.5秒）
    // ========================================================================
    always @(posedge clk or negedge stabilized_rst_n) begin
        if (!stabilized_rst_n) begin
            gps_led_reg <= 1'b0;
            gps_led_counter <= 32'd0;
        end else begin
            if (gps_pps_detected) begin
                if (gps_led_counter < CLK_CYCLES_PER_SEC / 2) begin
                    // 亮0.5秒
                    gps_led_reg <= 1'b1;
                    gps_led_counter <= gps_led_counter + 32'd1;
                end else if (gps_led_counter < CLK_CYCLES_PER_SEC) begin
                    // 灭0.5秒
                    gps_led_reg <= 1'b0;
                    gps_led_counter <= gps_led_counter + 32'd1;
                end else begin
                    // 重置计数器
                    gps_led_counter <= 32'd0;
                end
            end else begin
                // 无GPS信号时指示灯灭
                gps_led_reg <= 1'b0;
                gps_led_counter <= 32'd0;
            end
        end
    end

endmodule

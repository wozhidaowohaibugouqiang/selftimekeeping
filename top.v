`timescale 1ns / 1ps

// 模块名称：top
// 文件路径：e:\fjlwork\selftimekeep\top.v
// 功能说明：守时系统顶层模�?
//          整合守时核心、FIFO缓存、Flash存储、SPI通信及补偿逻辑
//          实现GPS信号检测、数据自动存储与回放补偿
//          包含长周期计数、动态补偿系数生成、老化补偿等增强功�?
// 修改记录�?
//          - 2026-01-22: 系统时钟统一�?0MHz，移除PLL相关逻辑
//          - 2026-01-22: 增加UART模块，支持串口指令和状态回�?
//          - 2026-01-22: 修正comp_step位宽匹配 (9-bit)
//          - 2026-03-13: 移植top_enhanced.v的全部功能，移除100秒稳定等待时�?
//          - 2026-03-16: 改进GPS信号处理：增加去抖动滤波，采用上升沿超时检测丢失，修正LED误闪问题
//
// 输入输出定义�?
//      - clk: 50MHz 系统时钟
//      - rst_n: 系统复位 (低有�?
//      - gps_pps: GPS 1PPS 输入
//      - spi_miso: Flash SPI MISO
//      - uart_rx/tx: 串口通信接口
//      - dpps_out: 补偿后的本地 1PPS 输出
//      - spi_sclk, spi_mosi, spi_cs_n: Flash SPI 接口信号

module top (
    input  wire         clk,            // 50MHz 系统时钟
    input  wire         rst_n,          // 系统复位
    input  wire         gps_pps,        // GPS PPS 信号
    input  wire         spi_miso,       // Flash SPI MISO
    input  wire         uart_rx,        // UART 接收
    input  wire         dump_key,       // 导出按键(P52, 低有效)
    input  wire         erase_key,      // Flash擦除按键(P54, 低有效)
    input  wire         dyn_key,        // 动态参数读取按键(P61, 低有效)
    
    // 评估模式已禁用（节省资源）
    
    output wire         uart_tx,        // UART 发�?
    output wire         dpps_out,       // 补偿后的 DPPS 输出
    output wire         spi_sclk,       // Flash SPI Clock
    output wire         spi_mosi,       // Flash SPI MOSI
    output wire         spi_cs_n,       // Flash SPI CS_n
    output wire         uart_led,        // 串口传输状态指示灯
    output wire         gps_led         // GPS PPS信号接收指示灯
);

    reg  [25:0] dump_key_cooldown;  // dump按键冷却计数器
    // erase_key 同步去抖
    reg  [2:0]  erase_key_sync;
    reg         erase_key_prev;
    wire        erase_key_pressed;
    wire        read_dout_ready;
    localparam [25:0] DUMP_KEY_COOLDOWN_CYC = 26'd50_000_000; // 1s @50MHz
    // 参数定义
    // ========================================================================
    parameter integer CLK_CYCLES_PER_SEC = 50_000_000;
    parameter integer GPS_LOST_THRESHOLD = 100_000_000; // 2秒超�?
    parameter integer POWERUP_HOLD_CYCLES = 500_000_000;
    parameter [23:0] DYN_PARAM_BASE_ADDR = 24'h3E0000; // 动态参数存储起始地址

    // ========================================================================
    // 内部信号定义
    // ========================================================================

    // 状态机定义
    localparam [4:0] 
        ST_STANDBY         = 5'b00000,
        ST_CALC            = 5'b00010,
        ST_TRAIN           = 5'b00011,
        ST_TRAIN_DONE      = 5'b00100,
        ST_PARAM_UPDATE    = 5'b00101,
        ST_ERR             = 5'b00110;

    reg [4:0] sys_state;
    wire [3:0] uart_cmd;
    wire       uart_cmd_valid;
    wire       read_done;
    reg [28:0] powerup_hold_cnt;
    reg        powerup_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            powerup_hold_cnt <= 29'd0;
            powerup_ready <= 1'b0;
        end else if (!powerup_ready) begin
            if (powerup_hold_cnt >= POWERUP_HOLD_CYCLES - 1) begin
                powerup_ready <= 1'b1;
            end else begin
                powerup_hold_cnt <= powerup_hold_cnt + 29'd1;
            end
        end
    end
    
    // 长周期计数相关信�?
    wire [15:0]  period_number;
    wire signed [63:0] diff_total;
    wire signed [63:0] quotient;
    wire signed [63:0] remainder;
    wire        diff_valid;
    wire        period_done;
    
    // 新增：period_done 寄存一�?
    reg period_done_d1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) period_done_d1 <= 1'b0;
        else period_done_d1 <= period_done;
    end
    
    // 新增：ST_PARAM_UPDATE 状态持续计数器
    reg [3:0] param_update_cnt;
    
    // 调试信号
    wire [2:0]   dbg_state;
    wire [7:0]   dbg_pps_cnt;
    wire         dbg_gps_pps_posedge;
    wire         dbg_counting_active;
    
    // 仅允许“有效且非零差值”进入FIFO写入链路（导出模式下停止）
    // 门限：|diff| <= 200us @50MHz = 10,000 cycles，超过则视为异常样本丢弃
    localparam signed [31:0] DIFF_ABS_MAX = 32'sd10000;
    wire [31:0] aging_diff_abs = aging_diff_value[31] ? (-aging_diff_value) : aging_diff_value;
    wire aging_diff_in_range = (aging_diff_abs <= DIFF_ABS_MAX[31:0]);
    wire aging_diff_write_valid = aging_diff_valid && dynamic_comp_ready && (aging_diff_value != 32'sd0) && aging_diff_in_range && !dump_active;

    // 新增：FIFO写入事件计数器（用于观测“差值有效→写入”链路）
    // 计数条件：写入条件满足且FIFO未满（与fifo_write_module写入条件一致）
        // erase done pulse: reset counters after erase
    wire flash_erase_done;
    wire erase_done_pulse = flash_erase_done;

    reg [15:0] fwe_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fwe_cnt <= 16'd0;
        end else if (erase_done_pulse) begin
            fwe_cnt <= 16'd0;
        end else if (aging_diff_write_valid && !fifo_full) begin
            fwe_cnt <= fwe_cnt + 16'd1;
        end
    end

    // 新增：FIFO读使能事件计数器（用于观测Flash是否在持续读走FIFO）
    reg [15:0] fre_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fre_cnt <= 16'd0;
        end else if (erase_done_pulse) begin
            fre_cnt <= 16'd0;
        end else if (fifo_re) begin
            fre_cnt <= fre_cnt + 16'd1;
        end
    end
    
    // 动态补偿系数相关信�?
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
    
    // 补偿计算相关信号
    reg signed [31:0] base_offset;
    reg signed [31:0] total_compensation;
    reg signed [23:0] dynamic_offset;
    reg signed [23:0] dynamic_offset_apply;
    reg [4:0]          dynamic_rem_acc;
    reg                comp_param_locked_d1;
    reg                dyn_apply_wait_one_window;
    reg signed [23:0]  dpps_offset;
    reg signed [31:0]  dpps_offset_sum_dbg;
    reg signed [31:0]  dpps_offset_sum_last_dbg;
    wire signed [31:0] dynamic_dpps_offset_ext = {{8{dpps_offset[23]}}, dpps_offset};
    wire signed [31:0] dpps_period_target_top_dbg = 32'sd50_000_000 + dynamic_dpps_offset_ext;
    wire signed [31:0] dpps_offset_dbg_ext = dynamic_dpps_offset_ext;
    wire signed [31:0] long_diff_dbg = diff_total[31:0];
    wire signed [31:0] long_residual_dbg = long_diff_dbg - dpps_offset_sum_last_dbg;
    wire signed [31:0] dynamic_quotient_16 = (comp_param_fixed >= 32'sd0) ? (comp_param_fixed >>> 4) : -(((-comp_param_fixed) >>> 4));
    wire signed [31:0] dynamic_remainder_16 = comp_param_fixed - (dynamic_quotient_16 <<< 4);
    wire [4:0]         dynamic_remainder_abs = dynamic_remainder_16[31] ? (-dynamic_remainder_16[4:0]) : dynamic_remainder_16[4:0];
    wire signed [23:0] dynamic_quotient_24 = dynamic_quotient_16[23:0];
    wire        sec_tick;
    wire        core_dpps;                  // 来自 timekeeping_core 的DPPS 输出
    wire signed [31:0] core_diff_result;   // 来自 timekeeping_core 的32位完整差值
    wire        core_diff_valid;            // 来自 timekeeping_core 的差值有效标志
    wire [3:0]  core_pps_cnt_8;             // 来自 timekeeping_core �?PPS 计数器（0-15�?
    wire        core_gps_pps_posedge;       // 来自 timekeeping_core �?GPS PPS 上升�?
    wire        core_dbg_diff_valid;        // 来自 timekeeping_core 的调试用差值有效标�?
    wire [31:0] core_gps_phase_sum;         // 来自 timekeeping_core 的GPS累积计数
    wire [31:0] core_dpps_phase_sum;        // 来自 timekeeping_core 的DPPS累积计数
    wire        core_dbg_dpps;              // 来自 timekeeping_core 的DPPS脉冲输出
    wire [31:0] core_dbg_dpps_phase_cnt;    // 来自 timekeeping_core 的DPPS相位计数�?
    wire [31:0] core_dbg_dpps_pulse_cnt;    // 来自 timekeeping_core 的DPPS脉冲宽度计数�?
    wire        core_dbg_dpps_pulse_active; // 来自 timekeeping_core 的DPPS脉冲活跃标志
    wire [3:0]  core_dbg_dpps_cnt;          // 来自 timekeeping_core 的DPPS上升沿计�?
    wire        core_dbg_dpps_posedge;
    wire [23:0] core_dbg_dpps_offset_dbg;
    wire [31:0] core_dbg_dpps_period_target_dbg;      // 来自 timekeeping_core 的DPPS上升沿检�?
    
    // FIFO相关信号
    wire [31:0] fifo_dout;
    wire        fifo_empty;
    wire        fifo_full;
    wire        fifo_re;
    wire        flush_req;
    wire        fifo_we_debug;
    wire [1:0]  fifo_state_debug;
    wire [15:0] fifo_total_write_cnt;
    
    // ==========================================
    // 新增：关键信号锁存器 (用于调试和定位问题)
    // ==========================================
    reg latch_fifo_we;
    reg latch_flush_req;
    reg latch_fifo_re;
    reg latch_flash_prog;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latch_fifo_we    <= 1'b0;
            latch_flush_req  <= 1'b0;
            latch_fifo_re    <= 1'b0;
            latch_flash_prog <= 1'b0;
        end else begin
            // 收到清零指令(比如通过erase或者特定的读取后)可以清零，但为了抓取状态，我们保持锁存直到下一次复位或Erase
            if (erase_done_pulse) begin
                latch_fifo_we    <= 1'b0;
                latch_flush_req  <= 1'b0;
                latch_fifo_re    <= 1'b0;
                latch_flash_prog <= 1'b0;
            end else begin
                if (fifo_we_debug) 
                    latch_fifo_we <= 1'b1;
                if (flush_req)
                    latch_flush_req <= 1'b1;
                if (fifo_re)
                    latch_fifo_re <= 1'b1;
                // 9 = ST_WREN_PROG, 10 = ST_SEND_PROG, 11 = ST_POLL_PROG
                if (flash_state_dbg == 7'd9 || flash_state_dbg == 7'd10 || flash_state_dbg == 7'd11)
                    latch_flash_prog <= 1'b1;
            end
        end
    end
    wire [3:0] dbg_latches = {latch_flash_prog, latch_fifo_re, latch_flush_req, latch_fifo_we};
    // ==========================================

    // Flash控制相关信号
    wire        flash_spi_sclk;
    wire        flash_spi_mosi;
    wire        flash_spi_cs_n;
    wire        flash_busy;
    wire        flash_done;
    wire        write_done;   // only write done, not erase done
    wire        flash_error;
    wire [7:0]  hist_diff;
    wire        hist_diff_valid;
    wire [31:0] last_prog_word_mirror;
    wire        flash_read_mode;  // 读取模式标志，dump时拉高
    wire        flush_active;    // FIFO正在写入Flash标志

    // FIFO读相关信�?
    wire        read_spi_sclk;
    wire        read_spi_mosi;
    wire        read_spi_cs_n;
    wire        read_dout_valid;
    wire        read_dout_empty;
    wire signed [31:0] read_diff_result;
    wire [7:0]  dbg_rd_first_byte;
    wire        dbg_rd_first_valid;
    wire [23:0] flash_cur_addr_dbg;
    wire [15:0] flash_poll_attempts_dbg;
    wire [7:0]  flash_last_sr_dbg;
    wire [2:0]  flash_sr_stage_dbg;
    wire [7:0]  flash_last_cmd_dbg;
    wire [3:0]  flash_last_len_dbg;
    wire [7:0]  flash_last_rx0_dbg;
    wire [7:0]  flash_last_rx1_dbg;
    wire [3:0]  flash_err_code_dbg;
    wire [23:0] flash_jedec_id_dbg;
    wire        flash_jedec_valid_dbg;
    wire [6:0]  flash_state_dbg;
    wire [7:0]  flash_txn_done_cnt;
    wire [7:0]  flash_check_fifo_cnt;
    wire [7:0]  flash_fifo_re_done_cnt;
    wire [7:0]  flash_fifo_latch_done_cnt;
    wire [15:0] flash_fifo_prog_cnt;
    wire        flash_fifo_was_empty_at_dump;
    wire        dump_force_flush_wire;
    wire        fl_rd_req;
    wire [23:0] fl_rd_addr;
    wire [7:0]  fl_rd_data;
    wire        fl_rd_data_valid;

    // 临时方案：固定导出上限，用于读取“历史老数据”（不依赖运行时计数器）
    localparam [15:0] DUMP_WORD_LIMIT_FIXED = 16'd3600;
    localparam [23:0] AGING_DATA_BASE_ADDR = 24'h100000;

    // Dump导出控制信号
    reg         dump_active;
    reg         dump_arm_wait_window;
    reg         dump_force_flush;
    reg         dump_read_active;
    reg         dump_pending;
    reg         dump_done_flag;
    reg [15:0]  dump_word_limit;
    reg [23:0]  dump_read_start_addr;
    reg [31:0] dump_start_lw;  // Dump开始时刻锁存的last_prog_word
    reg [1:0]  dump_start_fifo_state;  // Dump开始时刻FIFO状态(00=有数据,01=空,10=满)
    reg [15:0] dump_start_fifo_cnt;    // Dump开始时刻累计写入字数
    reg [23:0]  dump_addr;
    reg [31:0]  dump_data;
    reg         dump_meta_pending;
    reg [31:0]  dump_flush_timeout;
    reg [31:0]  dump_read_timeout;
    reg [15:0]  dump_exported_words;
    reg         dump_flush_done_seen;
    // 元数据(持久化导出limit)管理：
    // 记录格式(8B): A5 5A CNT_H CNT_L ~CNT_H ~CNT_L FF FF
    // 存储区: 0x3F0000 扇区，最多256条
    localparam [23:0] META_BASE_ADDR   = 24'h3F0000;
    localparam [7:0]  META_MAX_RECORDS = 8'd255;
    localparam [23:0] META_REC_STRIDE  = 24'd8;

    reg         md_boot_active;
    reg [1:0]   md_boot_state;
    reg [7:0]   md_boot_idx;
    reg [2:0]   md_boot_byte;
    reg [7:0]   md_b0, md_b1, md_b2, md_b3, md_b4, md_b5;
    reg         md_limit_valid;
    reg [15:0]  md_limit_restored;
    reg         md_rd_req;
    reg [23:0]  md_rd_addr;

    reg         md_wr_pending;
    reg [2:0]   md_wr_byte;
    reg [7:0]   md_wr_idx;
    reg [15:0]  md_last_committed;
    reg [15:0]  md_cnt_snapshot;
    reg         md_param_valid;
    reg [7:0]   md_param_data;
    reg [23:0]  md_param_addr;

    wire        fc_param_valid;
    wire [7:0]  fc_param_data;
    wire [23:0] fc_param_addr;
    wire [1:0]  fc_param_type;

    wire        fl_rd_req_dump;
    wire [23:0] fl_rd_addr_dump;
    reg         dyn_fl_rd_req;
    reg  [23:0] dyn_fl_rd_addr;
    // 评估模式Flash读取已禁用
    wire        fl_rd_req_mux;
    wire [23:0] fl_rd_addr_mux;

    assign fc_param_valid = md_param_valid;
    assign fc_param_data  = md_param_data;
    assign fc_param_addr  = md_param_addr;
    assign fc_param_type  = 2'b00;

    assign fl_rd_req_dump  = fl_rd_req;
    assign fl_rd_addr_dump = fl_rd_addr;
    assign fl_rd_req_mux  = dump_read_active ? fl_rd_req_dump : dyn_fl_rd_req;
    assign fl_rd_addr_mux = dump_read_active ? fl_rd_addr_dump : dyn_fl_rd_addr;
    // 读取模式：dump读取时拉高，阻止其他写入操作抢占SPI总线
    assign flash_read_mode = dump_read_active;
    // 强制flush信号：dump时绕过fifo_full判断，强制从FIFO读取数据写入Flash
    assign dump_force_flush_wire = dump_force_flush;

    
    wire        dump_meta_ready;
    wire        dump_word_ready;
    wire        dump_word_valid = dump_pending;
    wire [23:0] dump_word_addr = dump_addr;
    wire [31:0] dump_word_data = dump_data;
    reg  [2:0]  dump_key_sync;
    reg         dump_key_prev;
    wire        dump_key_pressed;
    reg  [2:0]  dyn_key_sync;
    reg         dyn_key_prev;
    wire        dyn_key_pressed;
    // ========================================================================
    // UART相关信号
    wire        tx_en;
    wire        uart_read_diff_valid;
    wire signed [31:0] uart_read_diff_value;
    wire        uart_flash_dyn_empty;
    
    // 均匀补偿模块输出
    wire        comp_valid;
    wire signed [15:0] comp_step;

    // 两层差值链路：老化差值(长期) + 动态残差(短期)
    wire signed [31:0] aging_diff_value;
    wire               aging_diff_valid;
    wire signed [31:0] dynamic_diff_value;
    wire               dynamic_diff_valid;

    // 动态补偿输入：以16s周期差值为基准
    // 规则：GPS稳定后先等待10个16s窗口（仅稳定，不训练），再用后10个窗口求平均并锁定参数
    wire        comp_diff_valid;
    wire signed [31:0] comp_diff_value;
    // 心跳/周期脉冲不再依赖 flash_busy，避免Flash写入期间串口“假死”
    wire        period_diff_pulse = powerup_ready && period_done && diff_valid && gps_stable && !dump_active;
    wire        uart_hb_pulse = powerup_ready && period_done && gps_stable;

    reg [3:0]         comp_settle_cnt;
    // 稳定后额外等待的16s窗口数。设为2表示跳过前2个窗口，用后3个窗口求平均。
    localparam [3:0]  COMP_SETTLE_WINDOWS = 4'd2;
    reg signed [39:0] comp_init_sum;
    reg [3:0]         comp_init_cnt;
    wire signed [39:0] comp_init_total = comp_init_sum + $signed({{8{diff_total[31]}}, diff_total[31:0]});
    wire signed [31:0] comp_param_next = $signed((comp_init_total * 40'sd10923) >>> 15);
    reg               comp_param_locked;
    reg signed [31:0] comp_param_fixed;
    
    // 动态参数管理相关信号
    reg               dyn_param_ok_pulse;
    reg signed [31:0] dyn_param_from_flash_value;
    reg               dyn_param_empty_pulse;
    reg               dyn_param_save_pulse;
    reg signed [31:0] dyn_param_save_value;
    reg               dyn_param_store_pending;
    reg               dyn_auto_check_req_pulse;
    reg signed [31:0] dyn_auto_check_req_value;
    reg               dyn_reinit_pulse;
    reg               dyn_param_from_flash_valid;
    reg               dyn_rd_active;
    reg               dyn_rd_wait;
    reg [1:0]         dyn_rd_byte_idx;
    reg signed [31:0] dyn_rd_value;
    reg               dyn_rd_delay;
    reg               dyn_store_active;
    reg               dyn_store_wait_done;
    reg [1:0]         dyn_store_byte_idx;
    reg               dyn_store_start_pulse;
    reg               dyn_store_done_pulse;
    reg               dyn_store_fail_pulse;
    reg               dyn_store_req_from_key;
    reg signed [31:0] dyn_store_req_value;
    reg               dyn_key_locked_snapshot;
    reg signed [31:0] dyn_key_value_snapshot;
    reg               dyn_auto_check_pending;
    reg signed [31:0] dyn_auto_check_value;
    reg               dyn_boot_check_pending;
    reg               dyn_rd_from_auto;

    // 补偿后的差值输出（软件补偿核心）
    // aging_diff_value 和 dynamic_diff_value 在后面定义为补偿后的值
    assign uart_read_diff_valid = dyn_param_ok_pulse;
    assign uart_read_diff_value = dyn_param_from_flash_value;
    assign uart_flash_dyn_empty = dyn_param_empty_pulse;

    // GPS丢失时沿用回读差值；GPS正常时仅在“参数已锁定”后按16s周期喂入固定参数
    assign comp_diff_valid = (gps_lost && !dump_active) ? read_dout_valid : (period_diff_pulse && comp_param_locked);
    assign comp_diff_value = (gps_lost && !dump_active) ? read_diff_result : comp_param_fixed;
    
    // ==================== GPS 信号处理 (修改�? ====================
    reg  [2:0] gps_pps_sync;                // 三级同步�?
    reg [1:0] pps_meta;                     // 用于去抖动的两级采样
    reg       pps_debounced;                 // 去抖动后的GPS信号
    reg       pps_debounced_prev;            // 上一拍值，用于边沿检�?
    wire      pps_rising;                     // 去抖动后的上升沿
    wire      gps_pps_clean;                  // 最终使用的GPS信号（赋值给原有名称，保持兼容）
    
    reg [25:0] gps_timeout_cnt;               // 超时计数器
    reg        gps_lost;                       // GPS丢失标志（1=丢失）
    wire       gps_pps_detected;               // GPS有效标志（持续高电平表示有PPS）

    // 晶振稳定判定：GPS连续有效一段时间后才开始采集老化基准
    reg [7:0]  gps_stable_sec_cnt;
    reg        gps_stable;
    reg        dynamic_comp_ready;
    localparam [7:0] GPS_STABLE_SEC = 8'd30;
    // 三级同步
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gps_pps_sync <= 3'b000;
        end else begin
            gps_pps_sync <= {gps_pps_sync[1:0], gps_pps};
        end
    end

    // 去抖动：连续两次采样相同才更新输�?
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pps_meta <= 2'b00;
            pps_debounced <= 1'b0;
        end else begin
            pps_meta <= {pps_meta[0], gps_pps_sync[2]};
            if (pps_meta[1] == pps_meta[0]) // 连续两次相同
                pps_debounced <= pps_meta[1];
            // else 保持原�?
        end
    end

    // 边沿检�?
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pps_debounced_prev <= 1'b0;
        end else begin
            pps_debounced_prev <= pps_debounced;
        end
    end
    assign pps_rising = pps_debounced & ~pps_debounced_prev;

    // 输出给其他模块的 GPS 信号（兼容原有命名）
    assign gps_pps_clean = pps_debounced;

    // GPS 丢失检测（基于上升沿超时）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gps_timeout_cnt <= 26'd0;
            gps_lost <= 1'b1; // 复位时认为丢�?
        end else begin
            if (pps_rising) begin
                gps_timeout_cnt <= 26'd0;
                gps_lost <= 1'b0;
            end else if (gps_timeout_cnt < GPS_LOST_THRESHOLD) begin
                gps_timeout_cnt <= gps_timeout_cnt + 26'd1;
            end else begin
                gps_lost <= 1'b1;
            end
        end
    end

    // GPS 有效标志（持续高电平，用于LED控制）
    assign gps_pps_detected = !gps_lost;

    // 晶振稳定判定：GPS连续有效30秒后拉高
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gps_stable_sec_cnt <= 8'd0;
            gps_stable <= 1'b0;
        end else if (gps_lost) begin
            gps_stable_sec_cnt <= 8'd0;
            gps_stable <= 1'b0;
        end else if (sec_tick) begin
            if (!gps_stable) begin
                if (gps_stable_sec_cnt >= GPS_STABLE_SEC - 1) begin
                    gps_stable <= 1'b1;
                end else begin
                    gps_stable_sec_cnt <= gps_stable_sec_cnt + 8'd1;
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dynamic_comp_ready <= 1'b0;
            comp_settle_cnt <= 4'd0;
            comp_init_sum <= 40'sd0;
            comp_init_cnt <= 4'd0;
            comp_param_locked <= 1'b0;
            comp_param_fixed <= 32'sd0;
            dyn_param_save_pulse <= 1'b0;
            dyn_param_save_value <= 32'sd0;
            dyn_param_store_pending <= 1'b0;
            dyn_auto_check_req_pulse <= 1'b0;
            dyn_auto_check_req_value <= 32'sd0;
        end else if (dyn_reinit_pulse) begin
            dynamic_comp_ready <= 1'b0;
            comp_settle_cnt <= 4'd0;
            comp_init_sum <= 40'sd0;
            comp_init_cnt <= 4'd0;
            comp_param_locked <= 1'b0;
            dyn_param_save_pulse <= 1'b0;
            dyn_auto_check_req_pulse <= 1'b0;
        end else if (erase_key_pressed || flash_erase_done) begin
            dynamic_comp_ready <= 1'b0;
            comp_settle_cnt <= 4'd0;
            comp_init_sum <= 40'sd0;
            comp_init_cnt <= 4'd0;
            comp_param_locked <= 1'b0;
            comp_param_fixed <= 32'sd0;
            dyn_param_save_pulse <= 1'b0;
            dyn_param_store_pending <= 1'b0;
            dyn_auto_check_req_pulse <= 1'b0;
            dyn_auto_check_req_value <= 32'sd0;
        end else if (dyn_store_done_pulse || dyn_store_fail_pulse) begin
            dyn_param_store_pending <= 1'b0;
            dyn_auto_check_req_pulse <= 1'b0;
        end else if (dyn_store_req_from_key) begin
            dyn_param_save_value <= dyn_store_req_value;
            dyn_param_save_pulse <= 1'b1;
            dyn_param_store_pending <= 1'b1;
            dyn_auto_check_req_pulse <= 1'b0;
        end else if (gps_lost || !gps_stable) begin
            dynamic_comp_ready <= 1'b0;
            comp_settle_cnt <= 4'd0;
            comp_init_sum <= 40'sd0;
            comp_init_cnt <= 4'd0;
            comp_param_locked <= 1'b0;
            if (!dyn_param_from_flash_valid)
                comp_param_fixed <= 32'sd0;
            dyn_param_save_pulse <= 1'b0;
            dyn_param_store_pending <= 1'b0;
            dyn_auto_check_req_pulse <= 1'b0;
        end else if (period_diff_pulse && !dyn_rd_active && !dyn_store_active) begin
            // 新增保护：当 Flash 正在进行擦除/读写操作时，暂停核心的统计和状态推进
            dyn_param_save_pulse <= 1'b0;
            dyn_auto_check_req_pulse <= 1'b0;
            if (!comp_param_locked) begin
                if (dyn_param_from_flash_valid) begin
                    comp_param_fixed <= dyn_param_from_flash_value;
                    comp_param_locked <= 1'b1;
                    dynamic_comp_ready <= 1'b1;
                end else if (comp_settle_cnt < COMP_SETTLE_WINDOWS) begin
                    comp_settle_cnt <= comp_settle_cnt + 4'd1;
                    comp_init_sum <= 40'sd0;
                    comp_init_cnt <= 4'd0;
                end else begin
                    // diff_total 异常过滤：每次累加前检查当前窗口差值是否在合理范围内
                    // |diff_total| > 10000 说明当前窗口测量异常（如GPS信号跳变），重置累加过程
                    if ($signed(diff_total[31:0]) > 32'sd10000 || $signed(diff_total[31:0]) < -32'sd10000) begin
                        comp_init_sum <= 40'sd0;
                        comp_init_cnt <= 4'd0;
                    end else if (comp_init_cnt < 4'd2) begin
                        // cnt=0,1: 累加前2个窗口
                        comp_init_sum <= comp_init_sum + $signed(diff_total[31:0]);
                        comp_init_cnt <= comp_init_cnt + 4'd1;
                    end else if (comp_init_cnt == 4'd2) begin
                        // cnt=2: 第3个窗口到达，sum=W1+W2，此时加入W3得到W1+W2+W3
                        // 用这3个窗口的平均值作为 comp_param_fixed（不再加第4个值）
                        // 注意：需要先把W3加进去再算平均，所以这里用临时计算
                        // 使用乘以 10923 然后右移 14 位来近似除以 3 (10923/16384 = 0.6667)
                        // 重要：对平均值取反，实现反向补偿
                        //   D < 0 (DPPS太快) → comp_param_fixed > 0 → dpps_offset > 0
                        //   dpps_period_target = 50M + 正值 → 周期增大 → DPPS 变慢 ✓
                        // Corrected behavior: 10923 / 32768 ~= 1/3; keep the DIFF sign for dpps_offset.
                        // Older encoded comments above describe the previous wrong scale/sign.
                        comp_param_fixed <= comp_param_next;
                        dyn_param_save_value <= comp_param_next;
                        // 累加第3个窗口（为后续可能的下一次锁定准备）
                        comp_init_sum <= comp_init_sum + $signed(diff_total[31:0]);
                        comp_init_cnt <= comp_init_cnt + 4'd1;
                        // 仅锁定动态补偿参数；禁用自动落Flash，避免首次锁定后占满Flash状态机
                        dyn_param_save_pulse <= 1'b0;
                        dyn_param_store_pending <= 1'b0;
                        dyn_auto_check_req_pulse <= 1'b0;
                        dyn_auto_check_req_value <= 32'sd0;
                        comp_param_locked <= 1'b1;
                        dynamic_comp_ready <= 1'b1;
                    end
                end
            end
        end else begin
            dyn_param_save_pulse <= 1'b0;
            dyn_auto_check_req_pulse <= 1'b0;
        end
    end

    assign aging_diff_valid = core_diff_valid & gps_stable & powerup_ready;
    assign dynamic_diff_valid = core_diff_valid & gps_stable & powerup_ready;

    // The hardware DPPS period correction is already effective (LD - DS ~= 0).
    // timekeeping_core still reports a one-second boundary residue near the current
    // per-second offset, so remove that residue from the user-facing/recorded D.
    wire dynamic_diff_sw_comp_en = !dump_active && !gps_lost &&
                                   (sys_state == ST_CALC) &&
                                   dynamic_comp_ready && comp_param_locked;
    // Use the same 16-second window source as dynamic compensation itself.
    // LD is diff_total and DS is the accumulated dpps_offset over that window,
    // so RS=LD-DS is the real post-compensation residual.
    wire signed [31:0] dynamic_diff_corrected =
        dynamic_diff_sw_comp_en ? long_residual_dbg : core_diff_result;
    assign dynamic_diff_value = dynamic_diff_corrected;
    assign aging_diff_value = dynamic_diff_corrected;
    // 1) 按下P52(低有效)后先等待当前16s窗口结束
    // 2) 触发一次强制flush，待flash_done后进入只读导出
    // 3) 读一条发一条（ready/valid握手），保证115200下不丢包
    // dump_key_pressed: 需要去抖通过 + 冷却时间已过 + powerup完成
    assign dump_key_pressed = powerup_ready && dump_key_prev && !dump_key_sync[2] && (dump_key_cooldown == 26'd0);
    assign dyn_key_pressed = powerup_ready && dyn_key_prev && !dyn_key_sync[2];
    // eval_key_pressed 已禁用
    assign erase_key_pressed = powerup_ready && erase_key_prev && !erase_key_sync[2];
    assign read_dout_ready = dump_active ? (!dump_pending || (dump_pending && dump_word_ready)) : 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dump_key_sync <= 3'b111;
            dump_key_prev <= 1'b1;
            dump_active <= 1'b0;
            dump_arm_wait_window <= 1'b0;
            dump_force_flush <= 1'b0;
            dump_read_active <= 1'b0;
            dump_pending <= 1'b0;
            dump_done_flag <= 1'b0;
            dump_word_limit <= 16'd0;
            dump_read_start_addr <= AGING_DATA_BASE_ADDR;
            dump_addr <= AGING_DATA_BASE_ADDR;
            dump_data <= 32'd0;
            dump_meta_pending <= 1'b0;
            dump_flush_timeout <= 32'd0;
            dump_read_timeout <= 32'd0;
            dump_exported_words <= 16'd0;
            dump_flush_done_seen <= 1'b0;
            dyn_key_sync <= 3'b111;
            dyn_key_prev <= 1'b1;
            erase_key_sync <= 3'b111;
            erase_key_prev <= 1'b1;
            dump_key_cooldown <= 26'd0;
        end else begin
            // 关键修复：当擦除完成时，清除 dump_force_flush
            // 否则会进入无限擦除循环，从不写入FIFO
            if (erase_done_pulse)
                dump_force_flush <= 1'b0;

            dump_key_sync <= {dump_key_sync[1:0], dump_key};
            dump_key_prev <= dump_key_sync[2];
            dyn_key_sync <= {dyn_key_sync[1:0], dyn_key};
            dyn_key_prev <= dyn_key_sync[2];
            erase_key_sync <= {erase_key_sync[1:0], erase_key};
            erase_key_prev <= erase_key_sync[2];

            // dump按键冷却计数器递减
            if (dump_key_cooldown != 26'd0)
                dump_key_cooldown <= dump_key_cooldown - 26'd1;

            // 只有在未激活时才接受按键
            if (dump_key_pressed && !dump_active && !dump_arm_wait_window) begin
                // 改为直接激活，不再等待 GPS 同步窗口
                dump_active <= 1'b1;
                dump_force_flush <= 1'b1;
                dump_read_active <= 1'b0;
                dump_pending <= 1'b0;
                dump_done_flag <= 1'b0;
                dump_exported_words <= 16'd0;
                dump_word_limit <= 16'd0;
                dump_read_start_addr <= AGING_DATA_BASE_ADDR;
                dump_start_lw <= last_prog_word_mirror;
                dump_start_fifo_state <= fifo_state_debug;
                dump_start_fifo_cnt <= fifo_total_write_cnt;
                dump_addr <= AGING_DATA_BASE_ADDR;
                dump_flush_timeout <= 32'd0;
                dump_read_timeout <= 32'd0;
                dump_flush_done_seen <= 1'b0;
                dump_key_cooldown <= DUMP_KEY_COOLDOWN_CYC;  // 启动1秒冷却
            end

            if (dump_arm_wait_window && core_diff_valid && (core_pps_cnt_8 == 4'h0)) begin
                dump_active <= 1'b1;
                dump_arm_wait_window <= 1'b0;
                dump_force_flush <= 1'b1;
                dump_read_active <= 1'b0;
                dump_pending <= 1'b0;
                dump_done_flag <= 1'b0;
                dump_exported_words <= 16'd0;
                dump_word_limit <= fwe_cnt;
                dump_read_start_addr <= AGING_DATA_BASE_ADDR;
                dump_start_lw <= last_prog_word_mirror;
                dump_start_fifo_state <= fifo_state_debug;
                dump_start_fifo_cnt <= fifo_total_write_cnt;
                dump_addr <= AGING_DATA_BASE_ADDR;
                dump_flush_done_seen <= 1'b0;
            end

            // Increment timeout counter while waiting for write_done
            if (dump_active && dump_force_flush && !write_done)
                dump_flush_timeout <= dump_flush_timeout + 32'd1;
            else if (!dump_active)
                dump_flush_timeout <= 32'd0;

            // 锁存 flush 完成，避免 write_done 单拍脉冲被错过导致卡在 RAD START
            if (dump_active && dump_force_flush && write_done)
                dump_flush_done_seen <= 1'b1;
            if (!dump_active)
                dump_flush_done_seen <= 1'b0;

            // Proceed after latched write_done OR after 5s timeout
            if (dump_active && dump_force_flush && (dump_flush_done_seen || (dump_flush_timeout >= 32'd250000000))) begin
                dump_force_flush <= 1'b0;
                dump_read_active <= 1'b1;
                dump_meta_pending <= 1'b1;
                // 改为扫描模式：不设固定条数上限，交给 fifo_read_module
                // 通过连续 0xFFFFFFFF 结束判定自动收敛到真实“结束地址”
                dump_word_limit <= 16'd0;
                dump_read_start_addr <= AGING_DATA_BASE_ADDR;
                dump_start_lw <= last_prog_word_mirror;
                dump_start_fifo_state <= fifo_state_debug;
                dump_start_fifo_cnt <= fifo_total_write_cnt;
                dump_addr <= AGING_DATA_BASE_ADDR;
                
                // 复位底层的导出计数器，防止上一次的遗留值导致马上终止
                dump_exported_words <= 16'd0;
                dump_read_timeout <= 32'd0;
            end

            if (dump_meta_pending && dump_meta_ready) begin
                dump_meta_pending <= 1'b0;
            end

            if (dump_active && dump_read_active) begin
                if (read_dout_valid && !dump_pending) begin
                    dump_data <= read_diff_result;
                    dump_pending <= 1'b1;
                    dump_read_timeout <= 32'd0;
                end

                if (dump_pending && dump_word_ready) begin
                    dump_pending <= 1'b0;
                    dump_addr <= dump_addr + 24'd4;
                    dump_exported_words <= dump_exported_words + 16'd1;
                    dump_read_timeout <= 32'd0;
                end else begin
                    dump_read_timeout <= dump_read_timeout + 32'd1;
                end

                // 结束条件：
                // 1) 按固定条数导出完成
                // 2) 底层读状态机显式完成
                // 3) 长时间无新数据（防卡死保底，约2秒）
                if (((dump_word_limit != 16'd0) && (dump_exported_words >= dump_word_limit) && !dump_pending) ||
                    (read_done && read_dout_empty && !dump_pending) ||
                    (dump_read_timeout >= 32'd100000000 && !dump_pending)) begin
                    dump_done_flag <= 1'b1;
                    dump_read_active <= 1'b0;
                    // Dump流程结束后退出dump模式，恢复正常心跳与状态机
                    dump_active <= 1'b0;
                    dump_read_timeout <= 32'd0;
                end
            end
        end
    end

    // 评估模式控制与评估数据写入
    // 记录规则：
    //  - PRE 区(0x020000): 补偿前误差估计 err_pre = err_post + dpps_offset
    //  - POST区(0x028000): 补偿后误差 err_post = core_diff_result
    // 每16秒记录一组 PRE/POST（各4字节）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dyn_rd_active    <= 1'b0;
            dyn_rd_wait      <= 1'b0;
            dyn_rd_byte_idx  <= 2'd0;
            dyn_rd_value     <= 32'sd0;
            dyn_rd_delay     <= 1'b0;
            dyn_param_from_flash_valid <= 1'b0;
            dyn_param_from_flash_value <= 32'sd0;
            dyn_param_ok_pulse <= 1'b0;
            dyn_param_empty_pulse <= 1'b0;
            dyn_reinit_pulse <= 1'b0;
            dyn_store_active <= 1'b0;
            dyn_store_wait_done <= 1'b0;
            dyn_store_byte_idx <= 2'd0;
            dyn_store_start_pulse <= 1'b0;
            dyn_store_done_pulse <= 1'b0;
            dyn_store_fail_pulse <= 1'b0;
            dyn_store_req_from_key <= 1'b0;
            dyn_store_req_value <= 32'sd0;
            dyn_key_locked_snapshot <= 1'b0;
            dyn_key_value_snapshot <= 32'sd0;
            dyn_auto_check_pending <= 1'b0;
            dyn_auto_check_value <= 32'sd0;
            dyn_boot_check_pending <= 1'b1;
            dyn_rd_from_auto <= 1'b0;
            dyn_fl_rd_req    <= 1'b0;
            dyn_fl_rd_addr   <= 24'd0;
            md_param_valid   <= 1'b0;
            md_param_data    <= 8'h00;
            md_param_addr    <= 24'h000000;
        end else begin
            md_param_valid <= 1'b0;
            dyn_fl_rd_req <= 1'b0;
            dyn_param_ok_pulse <= 1'b0;
            dyn_param_empty_pulse <= 1'b0;
            dyn_reinit_pulse <= 1'b0;
            dyn_store_start_pulse <= 1'b0;
            dyn_store_done_pulse <= 1'b0;
            dyn_store_fail_pulse <= 1'b0;
            dyn_store_req_from_key <= 1'b0;
            if (erase_key_pressed || flash_erase_done) begin
                dyn_param_from_flash_valid <= 1'b0;
                dyn_param_from_flash_value <= 32'sd0;
                dyn_rd_active <= 1'b0;
                dyn_rd_wait <= 1'b0;
                dyn_rd_byte_idx <= 2'd0;
                dyn_rd_delay <= 1'b0;
                dyn_rd_value <= 32'sd0;
                dyn_store_active <= 1'b0;
                dyn_store_wait_done <= 1'b0;
                dyn_store_byte_idx <= 2'd0;
                dyn_auto_check_pending <= 1'b0;
                dyn_boot_check_pending <= 1'b0;
                dyn_rd_from_auto <= 1'b0;
            end
            if (dyn_auto_check_req_pulse) begin
                dyn_auto_check_pending <= 1'b1;
                dyn_auto_check_value <= dyn_auto_check_req_value;
            end

            if (dyn_param_store_pending && !dyn_store_active && !dyn_rd_active) begin
                dyn_store_active <= 1'b1;
                dyn_store_wait_done <= 1'b0;
                dyn_store_byte_idx <= 2'd0;
                dyn_store_start_pulse <= 1'b1;
            end
            if (powerup_ready && (dyn_auto_check_pending || dyn_boot_check_pending) && !dyn_rd_active && !dyn_store_active && !dyn_param_store_pending) begin
                dyn_rd_active <= 1'b1;
                dyn_rd_wait <= 1'b0;
                dyn_rd_byte_idx <= 2'd0;
                dyn_rd_value <= 32'sd0;
                dyn_rd_from_auto <= dyn_auto_check_pending;
            end

            if (dyn_key_pressed && !dump_active && !dyn_rd_active && !dyn_store_active && !dyn_param_store_pending) begin
                dyn_rd_active <= 1'b1;
                dyn_rd_wait <= 1'b0;
                dyn_rd_byte_idx <= 2'd0;
                dyn_rd_value <= 32'sd0;
                dyn_key_locked_snapshot <= comp_param_locked;
                dyn_key_value_snapshot <= comp_param_fixed;
                dyn_rd_from_auto <= 1'b0;
                // 移除 dyn_reinit_pulse，防止用户按下按钮时重置正在进行的 7 分钟后台训练
            end

            if (dyn_rd_active) begin
                if (!dyn_rd_wait && !flash_busy) begin
                    dyn_fl_rd_req  <= 1'b1;
                    dyn_fl_rd_addr <= DYN_PARAM_BASE_ADDR + {22'd0, dyn_rd_byte_idx};
                    dyn_rd_wait    <= 1'b1;
                end else if (dyn_rd_wait && fl_rd_data_valid) begin
                    case (dyn_rd_byte_idx)
                        2'd0: dyn_rd_value[31:24] <= fl_rd_data;
                        2'd1: dyn_rd_value[23:16] <= fl_rd_data;
                        2'd2: dyn_rd_value[15:8]  <= fl_rd_data;
                        2'd3: dyn_rd_value[7:0]   <= fl_rd_data;
                    endcase
                    dyn_rd_wait <= 1'b0;
                    dyn_fl_rd_req <= 1'b0; // 读完单字节撤销请求，等待底层回 IDLE
                    // 引入一个时钟周期的延迟，确保底层单字节读状态机已经完全退出 ST_RD_DONE
                    dyn_rd_delay <= 1'b1;
                end else if (dyn_rd_delay) begin
                    dyn_rd_delay <= 1'b0;
                    if (dyn_rd_byte_idx == 2'd3) begin
                        dyn_rd_active <= 1'b0;
                        dyn_rd_byte_idx <= 2'd0;
                        // 注意：这里需要检查是否读出了默认的全FF或者是上电初态的全0
                        if ({dyn_rd_value[31:8], fl_rd_data} != 32'hFFFF_FFFF && {dyn_rd_value[31:8], fl_rd_data} != 32'h0000_0000) begin
                            dyn_param_from_flash_valid <= 1'b1;
                            dyn_param_from_flash_value <= {dyn_rd_value[31:8], fl_rd_data};
                            dyn_param_ok_pulse <= 1'b1;
                        end else begin
                            dyn_param_from_flash_valid <= 1'b0;
                            dyn_param_from_flash_value <= 32'sd0;
                            dyn_param_empty_pulse <= 1'b1;
                            if (dyn_rd_from_auto) begin
                                dyn_store_req_from_key <= 1'b1;
                                dyn_store_req_value <= dyn_auto_check_value;
                            end else if (dyn_key_locked_snapshot) begin
                                dyn_store_req_from_key <= 1'b1;
                                dyn_store_req_value <= dyn_key_value_snapshot;
                            end else if (dynamic_comp_ready) begin
                                dyn_store_req_from_key <= 1'b1;
                                dyn_store_req_value <= comp_param_fixed;
                            end
                        end
                        if (dyn_rd_from_auto) begin
                            dyn_auto_check_pending <= 1'b0;
                            dyn_rd_from_auto <= 1'b0;
                        end else if (dyn_boot_check_pending) begin
                            dyn_boot_check_pending <= 1'b0;
                        end
                    end else begin
                        dyn_rd_byte_idx <= dyn_rd_byte_idx + 2'd1;
                    end
                end
            end

            if (dyn_store_active) begin
                if (!dyn_store_wait_done && !flash_busy) begin
                    md_param_valid <= 1'b1;
                    case (dyn_store_byte_idx)
                        2'd0: begin md_param_addr <= DYN_PARAM_BASE_ADDR + 24'd0; md_param_data <= dyn_param_save_value[31:24]; end
                        2'd1: begin md_param_addr <= DYN_PARAM_BASE_ADDR + 24'd1; md_param_data <= dyn_param_save_value[23:16]; end
                        2'd2: begin md_param_addr <= DYN_PARAM_BASE_ADDR + 24'd2; md_param_data <= dyn_param_save_value[15:8]; end
                        default: begin md_param_addr <= DYN_PARAM_BASE_ADDR + 24'd3; md_param_data <= dyn_param_save_value[7:0]; end
                    endcase
                    dyn_store_wait_done <= 1'b1;
                end else if (dyn_store_wait_done && flash_done) begin
                    dyn_store_wait_done <= 1'b0;
                    md_param_valid <= 1'b0; // 确保写完当前字节后撤销有效标志
                    // 更新下一字节的地址和数据（即使当前拍 flash_done 也会在下拍生效）
                    if (dyn_store_byte_idx < 2'd3) begin
                        md_param_addr <= DYN_PARAM_BASE_ADDR + {22'd0, dyn_store_byte_idx + 2'd1};
                        case (dyn_store_byte_idx)
                            2'd0: md_param_data <= dyn_param_save_value[23:16];
                            2'd1: md_param_data <= dyn_param_save_value[15:8];
                            default: md_param_data <= dyn_param_save_value[7:0];
                        endcase
                    end
                    if (dyn_store_byte_idx == 2'd3) begin
                        dyn_store_active <= 1'b0;
                        dyn_store_byte_idx <= 2'd0;
                        dyn_param_from_flash_valid <= 1'b1;
                        dyn_param_from_flash_value <= dyn_param_save_value;
                        dyn_store_done_pulse <= 1'b1;
                    end else begin
                        dyn_store_byte_idx <= dyn_store_byte_idx + 2'd1;
                    end
                end else if (dyn_store_wait_done && flash_error) begin
                    dyn_store_wait_done <= 1'b0;
                    md_param_valid <= 1'b0;
                    dyn_store_active <= 1'b0;
                    dyn_store_byte_idx <= 2'd0;
                    dyn_store_fail_pulse <= 1'b1;
                end
            end

            // eval mode 相关代码已禁用
            // if (eval_key_pressed && (eval_key_cooldown == 26'd0)) begin
            //     eval_mode <= ~eval_mode;
            //     eval_key_cooldown <= EVAL_KEY_COOLDOWN_CYC;
            //     if (!eval_mode) begin
            //         eval_replay_idx  <= 10'd0;
            //         eval_rd_active   <= 1'b0;
            //         eval_rd_wait     <= 1'b0;
            //         eval_rd_byte_idx <= 2'd0;
            //     end
            // end

            // eval mode 相关代码已禁用
            // if (!dyn_rd_active && eval_mode && period_diff_pulse && !eval_wr_active) begin
            //     eval_post_latched <= aging_diff_value;
            //     eval_pre_latched  <= aging_diff_value + {{8{dpps_offset[23]}}, dpps_offset};
            //     eval_wr_active    <= 1'b1;
            //     eval_wr_wait_done <= 1'b0;
            //     eval_wr_idx       <= 4'd0;
            //     if (!eval_rd_active) begin
            //         eval_rd_active   <= 1'b1;
            //         eval_rd_wait     <= 1'b0;
            //         eval_rd_byte_idx <= 2'd0;
            //         eval_fl_rd_addr  <= EVAL_REPLAY_BASE_ADDR + {eval_replay_idx, 2'b00};
            //     end
            // end

            // eval mode 相关代码已禁用
            // 评估模式下：每16s从老化差值区顺序读取1个32位样本作为“Flash老化补偿项”
            // if (!dyn_rd_active && eval_rd_active) begin
            //     if (!eval_rd_wait && !flash_busy) begin
            //         eval_fl_rd_req  <= 1'b1;
            //         eval_fl_rd_addr <= eval_fl_rd_addr;
            //         eval_rd_wait    <= 1'b1;
            //     end else if (eval_rd_wait && fl_rd_data_valid) begin
            //         case (eval_rd_byte_idx)
            //             2'd0: eval_replay_value[31:24] <= fl_rd_data;
            //             2'd1: eval_replay_value[23:16] <= fl_rd_data;
            //             2'd2: eval_replay_value[15:8]  <= fl_rd_data;
            //             2'd3: eval_replay_value[7:0]   <= fl_rd_data;
            //         endcase

            //         eval_rd_wait <= 1'b0;
            //         if (eval_rd_byte_idx == 2'd3) begin
            //             eval_rd_active <= 1'b0;
            //             eval_rd_byte_idx <= 2'd0;
            //             if (eval_replay_idx >= 10'd1023) begin
            //                 eval_replay_idx <= 10'd0;
            //                 eval_fl_rd_addr <= EVAL_REPLAY_BASE_ADDR;
            //             end else begin
            //                 eval_replay_idx <= eval_replay_idx + 10'd1;
            //                 eval_fl_rd_addr <= EVAL_REPLAY_BASE_ADDR + {eval_replay_idx + 10'd1, 2'b00};
            //             end
            //         end else begin
            //             eval_rd_byte_idx <= eval_rd_byte_idx + 2'd1;
            //             eval_fl_rd_addr  <= eval_fl_rd_addr + 24'd1;
            //         end
            //     end
            // end

            // eval mode 相关代码已禁用
            // if (eval_wr_active && !dyn_store_active) begin
            //     if (!eval_wr_wait_done && !flash_busy) begin
            //         md_param_valid <= 1'b1;
            //         case (eval_wr_idx)
            //             4'd0: begin md_param_addr <= eval_pre_wr_addr + 24'd0;  md_param_data <= eval_pre_latched[31:24]; end
            //             4'd1: begin md_param_addr <= eval_pre_wr_addr + 24'd1;  md_param_data <= eval_pre_latched[23:16]; end
            //             4'd2: begin md_param_addr <= eval_pre_wr_addr + 24'd2;  md_param_data <= eval_pre_latched[15:8];  end
            //             4'd3: begin md_param_addr <= eval_pre_wr_addr + 24'd3;  md_param_data <= eval_pre_latched[7:0];   end
            //             4'd4: begin md_param_addr <= eval_post_wr_addr + 24'd0; md_param_data <= eval_post_latched[31:24];end
            //             4'd5: begin md_param_addr <= eval_post_wr_addr + 24'd1; md_param_data <= eval_post_latched[23:16];end
            //             4'd6: begin md_param_addr <= eval_post_wr_addr + 24'd2; md_param_data <= eval_post_latched[15:8]; end
            //             default: begin md_param_addr <= eval_post_wr_addr + 24'd3; md_param_data <= eval_post_latched[7:0]; end
            //         endcase
            //         eval_wr_wait_done <= 1'b1;
            //     end else if (eval_wr_wait_done && flash_done) begin
            //         eval_wr_wait_done <= 1'b0;
            //         if (eval_wr_idx == 4'd7) begin
            //             eval_wr_active    <= 1'b0;
            //             eval_pre_wr_addr  <= eval_pre_wr_addr + 24'd4;
            //             eval_post_wr_addr <= eval_post_wr_addr + 24'd4;
            //             eval_sample_cnt   <= eval_sample_cnt + 16'd1;
            //         end else begin
            //             eval_wr_idx <= eval_wr_idx + 4'd1;
            //         end
            //     end
            // end
        end
    end

    // ========================================================================
    // 3. 长周期计数与差值运算模�?
    // ========================================================================
    long_period_counter u_long_counter (
        .clk              (clk),
        .sys_rst_n        (rst_n),
        .gps_pps          (gps_pps_clean),   // 使用去抖动后的信�?
        .osc_ready        (1'b1), // 直接置为1，移�?00秒等�?
        .period_number    (period_number),
        .diff_total       (diff_total),
        .quotient         (quotient),
        .remainder        (remainder),
        .diff_valid       (diff_valid),
        .period_done      (period_done),
        // 调试输出信号
        .dbg_state        (dbg_state),
        .dbg_pps_cnt      (dbg_pps_cnt),
        .dbg_gps_pps_posedge (dbg_gps_pps_posedge),
        .dbg_counting_active (dbg_counting_active)
    );
    
    // ========================================================================
    // 4. 动态补偿系数生成与存储模块
    // ========================================================================
    dynamic_comp_coeff u_dynamic_coeff (
        .clk              (clk),
        .sys_rst_n        (rst_n),
        .remainder        (remainder),
        .remainder_valid  (diff_valid),
        .coeff_read_addr  (coeff_read_addr),
        .coeff_read_en    (coeff_read_en),
        .coeff_data_out   (coeff_data_out),
        .coeff_data_valid (coeff_data_valid),
        .coeff_ready      (coeff_ready)
    );
    
    // ========================================================================
    // 5. 反向补偿系数管理模块（老化补偿�?
    // ========================================================================
    aging_comp_coeff u_aging_coeff (
        .clk              (clk),
        .sys_rst_n        (rst_n),
        .period_number    (period_number),
        .period_done      (period_done),
        .coeff_update_en  (coeff_update_en),
        .new_coeff_value  (new_coeff_value),
        .aging_comp_value (aging_comp_value),
        .coeff_valid      (aging_coeff_valid)
    );
    
    // ========================================================================
    // 6. 参数管理模块
    // ========================================================================
    param_manager u_param_manager (
        .sys_clk          (clk),
        .sys_rst_n        (rst_n),
        .diff_value       (aging_diff_value),
        .diff_valid_flag  (aging_diff_write_valid), // 仅在动态补偿完成后才更新参数
        .gps_valid        (!gps_lost),      // 使用丢失标志的反�?
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
    // 系统主状态机 (System FSM)
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sys_state <= ST_STANDBY;
            param_update_cnt <= 4'd0;
        end else if (!powerup_ready) begin
            sys_state <= ST_STANDBY;
            param_update_cnt <= 4'd0;
        end else begin
            if (dump_active) begin
                sys_state <= ST_TRAIN;
                param_update_cnt <= 4'd0;
            end
            // 1. 串口指令优先控制
            else if (uart_cmd_valid && uart_cmd != 4'd0) begin
                sys_state <= uart_cmd;
            end 
            // 2. 自动状态跳�?
            else begin
                case (sys_state)
                    ST_STANDBY: begin
                        // 上电默认逻辑：有GPS则计算，无GPS等待指令
                        if (!gps_lost) sys_state <= ST_CALC;
                    end
                    ST_CALC: begin
                        if (gps_lost) begin
                            sys_state <= ST_TRAIN;
                        end else if (period_done_d1) begin
                            sys_state <= ST_PARAM_UPDATE;
                        end
                    end
                    ST_PARAM_UPDATE: begin
                        if (param_update_cnt < 4'd15) begin
                            param_update_cnt <= param_update_cnt + 4'd1;
                        end else begin
                            param_update_cnt <= 4'd0;
                            sys_state <= ST_CALC;
                        end
                    end
                    ST_TRAIN: begin
                        if (read_done) sys_state <= ST_TRAIN_DONE;
                    end
                    // 其他状态保�?
                endcase
            end
        end
    end

    // 仅在两种场景触发写Flash：
    // 1) FIFO接近满（fifo_full来自afull门限）
    // 2) 用户按下dump按键触发强制flush
    wire flush_batch_ready = fifo_full;
    assign flush_req = powerup_ready && ((flush_batch_ready || dump_force_flush) && !dump_read_active);
    
    // ========================================================================
    // 补偿计算逻辑
    // ========================================================================
    localparam signed [23:0] DYNAMIC_OFFSET_MAX = 24'sh7FFFFF;
    localparam signed [23:0] DYNAMIC_OFFSET_MIN = -24'sh800000;
    reg signed [24:0] dyn_sum;
    reg signed [31:0] base_sum;
    reg core_dpps_d1;
    wire dpps_offset_sum_en = core_dpps && !core_dpps_d1 &&
                              !dump_active && !gps_lost &&
                              (sys_state == ST_CALC) &&
                              dynamic_comp_ready && comp_param_locked;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            base_offset <= 32'sd0;
            total_compensation <= 32'sd0;
            dynamic_offset <= 24'sd0;
            dynamic_offset_apply <= 24'sd0;
            dynamic_rem_acc <= 5'd0;
            comp_param_locked_d1 <= 1'b0;
            dyn_apply_wait_one_window <= 1'b0;
            core_dpps_d1 <= 1'b0;
            dpps_offset <= 24'sd0;
            dpps_offset_sum_dbg <= 32'sd0;
            dpps_offset_sum_last_dbg <= 32'sd0;
            coeff_read_addr <= 8'd0;
            coeff_read_en <= 1'b0;
            coeff_update_en <= 1'b0;
            new_coeff_value <= 32'sd0;
        end else begin
            comp_param_locked_d1 <= comp_param_locked;

            if (sec_tick && !flash_busy) begin
                coeff_read_addr <= coeff_read_addr + 8'd1;
                coeff_read_en <= 1'b1;
            end else begin
                coeff_read_en <= 1'b0;
            end

            core_dpps_d1 <= core_dpps;
            if (period_diff_pulse) begin
                dpps_offset_sum_last_dbg <= dpps_offset_sum_en ? (dpps_offset_sum_dbg + dpps_offset_dbg_ext) : dpps_offset_sum_dbg;
                dpps_offset_sum_dbg <= 32'sd0;
            end else if (dpps_offset_sum_en) begin
                dpps_offset_sum_dbg <= dpps_offset_sum_dbg + dpps_offset_dbg_ext;
            end
            // 正确的单位换算：
            // comp_param_fixed 是“16秒窗口总差值”，不能整值直接给每秒DPPS。
            // 必须按 /16 分成商和余数，再把余数通过累加器分摊到16个1秒周期。
            if (!dump_active && !gps_lost && (sys_state == ST_CALC) && dynamic_comp_ready && comp_param_locked) begin
                if (!comp_param_locked_d1 && comp_param_locked) begin
                    dynamic_rem_acc <= 5'd0;
                    dynamic_offset_apply <= dynamic_quotient_24;
                end else if (core_dpps && !core_dpps_d1) begin
                    if (dynamic_remainder_abs == 5'd0) begin
                        dynamic_rem_acc <= 5'd0;
                        dynamic_offset_apply <= dynamic_quotient_24;
                    end else if ((dynamic_rem_acc + dynamic_remainder_abs) >= 5'd16) begin
                        dynamic_rem_acc <= dynamic_rem_acc + dynamic_remainder_abs - 5'd16;
                        dynamic_offset_apply <= dynamic_remainder_16[31] ? (dynamic_quotient_24 - 24'sd1) : (dynamic_quotient_24 + 24'sd1);
                    end else begin
                        dynamic_rem_acc <= dynamic_rem_acc + dynamic_remainder_abs;
                        dynamic_offset_apply <= dynamic_quotient_24;
                    end
                end
                dpps_offset <= dynamic_offset_apply;
            end else begin
                dynamic_offset_apply <= 24'sd0;
                dynamic_rem_acc <= 5'd0;
                dpps_offset <= 24'sd0;
            end

            if (!dump_active && coeff_data_valid) begin
                base_sum = 32'sd0;
                base_offset <= base_sum;
                total_compensation <= base_sum;
            end else begin
                base_offset <= 32'sd0;
                total_compensation <= 32'sd0;
            end

            if (period_done && diff_valid && dynamic_comp_ready) begin
                coeff_update_en <= 1'b1;
                new_coeff_value <= aging_comp_value + (diff_total[31:0] >>> 10);
            end else begin
                coeff_update_en <= 1'b0;
            end
        end
    end 

    // ------------------------------------------------------------------------
    // 0. UART 模块
    //    负责上位机指令交互与状态回�?
    // ------------------------------------------------------------------------
    wire uart_tx_done;
    wire uart_erase_cmd;
    wire combined_erase_req = powerup_ready && (erase_key_pressed || uart_erase_cmd);

    uart u_uart (
        .sys_clk    (clk),
        .sys_rst_n  (rst_n),
        .uart_rx    (uart_rx),
        .uart_tx    (uart_tx),
        .erase_cmd_pulse(uart_erase_cmd),
        .erase_start_pulse  (combined_erase_req),
        .erase_done_pulse   (flash_erase_done),
        .read_start_pulse   (dump_key_pressed),
        .sys_state  (sys_state),
        .pps_valid  (gps_pps_detected),    // 使用改进后的有效标志
        .fifo_empty (fifo_empty),
        .fifo_full  (fifo_full),
        .fifo_dout  (8'd0),
        .train_done (sys_state == ST_TRAIN_DONE),
        .dbg_state  (3'd0),
        .dbg_pps_cnt(dbg_pps_cnt),
        .period_diff_pulse   (period_diff_pulse),  // 16s一个大周期发一次心跳包
        .dbg_gps_pps_posedge (dbg_gps_pps_posedge),
        .dbg_counting_active (dbg_counting_active),
        .period_cnt (16'd0),
        .fwe_cnt    (fwe_cnt),
        .fre_cnt    (fre_cnt),
        .dv50_cnt   (16'd0),
        .dv25_cnt   (16'd0),
        .state_changed (1'b0),
        .dbg_diff_valid(dynamic_diff_valid),
        .diff_result (dynamic_diff_value),
        .flash_busy  (flash_busy),
        .flash_done  (flash_done),
        .write_done  (write_done),
        .flash_error (flash_error),
        .dbg_rd_first_byte (8'd0),
        .dbg_rd_first_valid(1'b0),
        .dbg_flash_cur_addr(24'd0),
        .dbg_poll_attempts (16'd0),
        .dbg_last_sr       (8'd0),
        .dbg_sr_stage      (3'd0),
        .dbg_last_cmd      (8'd0),
        .dbg_last_len      (4'd0),
        .dbg_last_rx0      (8'd0),
        .dbg_last_rx1      (8'd0),
        .dbg_err_code    (flash_err_code_dbg),
        .dbg_jedec_id    (flash_jedec_id_dbg),
        .dbg_jedec_valid (flash_jedec_valid_dbg),
        .dbg_txn_done_cnt(flash_txn_done_cnt),
        .hist_diff       (hist_diff),
        .hist_diff_valid (hist_diff_valid),
        .read_diff       (uart_read_diff_value),
        .read_diff_valid (uart_read_diff_valid),
        .flash_dyn_empty (uart_flash_dyn_empty),
        .dpps_offset_dbg (dpps_offset),
        .dyn_offset_dbg  (dpps_offset),
        .comp_step_dbg   (comp_step),
        .comp_valid_dbg  (comp_valid),
        .dyn_ready_dbg   (dynamic_comp_ready),
        .comp_fixed_dbg  (comp_param_fixed),
        .dyn_q_dbg       (dynamic_quotient_16),
        .dyn_r_dbg       (dynamic_remainder_16),
        .dyn_ds_dbg      (dpps_offset_sum_last_dbg),
        .dyn_ld_dbg      (long_diff_dbg),
        .dyn_rs_dbg      (long_residual_dbg),
        .dump_mode      (dump_active),
        .dump_word_valid(dump_word_valid),
        .dump_word_addr (dump_word_addr),
        .dump_word_data (dump_word_data),
        .dump_done      (dump_done_flag),
        .dump_meta_valid(1'b0),
        .dump_meta_fwe  (16'd0),
        .dump_meta_fre  (16'd0),
        .dump_meta_limit(16'd0),
        .dump_meta_exported(16'd0),
        .dump_word_ready(dump_word_ready),
        .dump_meta_ready(dump_meta_ready),
        .last_prog_word (last_prog_word_mirror),
        .dpps_offset_dbg_core(core_dbg_dpps_offset_dbg),
        .dpps_period_target_dbg(dpps_period_target_top_dbg),
        .dump_read_start_addr(dump_read_start_addr),
        .dump_start_lw       (dump_start_lw),
        .state_cmd  (uart_cmd),
        .cmd_valid  (uart_cmd_valid),
        .tx_en      (tx_en),
        .tx_done    (uart_tx_done),
        .fifo_write_cnt (fifo_total_write_cnt), // 修复：连接到正确的FIFO写入计数器
        .dbg_flash_state    (dbg_flash_state),    // Flash状态机当前状态
        .check_fifo_cnt (flash_check_fifo_cnt),  // ST_CHECK_FIFO进入次数
        .fifo_re_done_cnt (flash_fifo_re_done_cnt), // FIFO读取完成次数
        .fifo_latch_done_cnt (flash_fifo_latch_done_cnt), // FIFO锁存完成次数
        .fifo_prog_cnt (flash_fifo_prog_cnt), // FIFO实际写入Flash字数
        .fifo_was_empty_at_dump (flash_fifo_was_empty_at_dump), // dump开始时FIFO是否为空
        .fifo_state_at_dump_start (dump_start_fifo_state), // dump开始FIFO状态
        .fifo_total_cnt_at_dump_start (dump_start_fifo_cnt)  // dump开始累计写入字数
    );

    // ------------------------------------------------------------------------
    // 1. 守时核心模块 (Timekeeping Core)
    //    产生本地 DPPS，并计算�?GPS PPS 的差�?
    // ------------------------------------------------------------------------
    timekeeping_core u_core (
        .clk             (clk),
        .sys_rst_n       (rst_n),
        .gps_pps         (gps_pps_clean),   // 使用去抖动后的信�?
        .dpps_offset     (dpps_offset),
        .init_diff       (8'd0),
        .init_diff_valid (1'b0),
        .dpps            (core_dpps),
        .diff_result     (core_diff_result),
        .diff_valid_flag (core_diff_valid),
        .pps_cnt_8       (core_pps_cnt_8),
        .gps_pps_posedge (core_gps_pps_posedge),
        .dbg_diff_valid  (core_dbg_diff_valid),
        .dbg_gps_phase_sum   (core_gps_phase_sum),
        .dbg_dpps_phase_sum  (core_dpps_phase_sum),
        .dbg_dpps            (core_dbg_dpps),
        .dbg_dpps_phase_cnt  (core_dbg_dpps_phase_cnt),
        .dbg_dpps_pulse_cnt  (core_dbg_dpps_pulse_cnt),
        .dbg_dpps_pulse_active (core_dbg_dpps_pulse_active),
        .dbg_dpps_cnt          (core_dbg_dpps_cnt),
        .dbg_dpps_posedge      (core_dbg_dpps_posedge),
        .dbg_dpps_offset       (core_dbg_dpps_offset_dbg),
        .dbg_dpps_period_target(core_dbg_dpps_period_target_dbg)
    );
    assign dpps_out = core_dpps;

    // ------------------------------------------------------------------------
    // 2. FIFO 写模�?
    // ------------------------------------------------------------------------
    // 直接将带老化补偿的差值送入 FIFO
    fifo_write_module u_fifo_write (
        .sys_clk_50mhz   (clk),
        .sys_rst_n       (rst_n),
        .flush_req       (flush_req),
        .diff_value      (aging_diff_value),
        .diff_valid_flag (aging_diff_write_valid),  // 修复：使用 aging_diff_write_valid (包含 dynamic_comp_ready 门限)
        .fifo_re         (fifo_re),
        .fifo_dout       (fifo_dout),
        .fifo_empty_flag (fifo_empty),
        .fifo_full_flag  (fifo_full),
        .fifo_we_debug   (fifo_we_debug),
        .fifo_state_debug(fifo_state_debug),
        .fifo_total_write_cnt(fifo_total_write_cnt)
    );

    // ------------------------------------------------------------------------
    // 3. Flash 控制模块 (增强?
    // ------------------------------------------------------------------------

    flash_controller_enhanced u_flash_enhanced (
        .sys_clk         (clk),
        .sys_rst_n       (rst_n),
        .collect_stop    (flush_req),
        .erase_req       (combined_erase_req),
        .fifo_re         (fifo_re),
        .fifo_dout       (fifo_dout),
        .fifo_empty_flag (fifo_empty),
        .fifo_full_flag  (fifo_full),
        .param_data      (fc_param_data),
        .param_addr      (fc_param_addr),
        .param_valid     (fc_param_valid),
        .param_type      (fc_param_type),
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
        .write_done      (write_done),
        .erase_done      (flash_erase_done),
        .flash_error     (flash_error),
        .hist_diff       (hist_diff),
        .hist_diff_valid (hist_diff_valid),
        .last_prog_word_mirror(last_prog_word_mirror),
        .rd_req          (fl_rd_req_mux),
        .rd_addr         (fl_rd_addr_mux),
        .rd_data         (fl_rd_data),
        .rd_data_valid       (fl_rd_data_valid),
        .cur_addr_dbg        (flash_cur_addr_dbg),
        .poll_attempts_dbg   (flash_poll_attempts_dbg),
        .last_sr_dbg         (flash_last_sr_dbg),
        .sr_stage_dbg        (flash_sr_stage_dbg),
        .last_cmd_dbg        (flash_last_cmd_dbg),
        .last_len_dbg        (flash_last_len_dbg),
        .last_rx0_dbg        (flash_last_rx0_dbg),
        .last_rx1_dbg        (flash_last_rx1_dbg),
        .err_code_dbg        (flash_err_code_dbg),
        .jedec_id_dbg        (flash_jedec_id_dbg),
        .jedec_valid_dbg     (flash_jedec_valid_dbg),
        .state_dbg           (flash_state_dbg),
        .txn_done_cnt_dbg    (flash_txn_done_cnt),
        .check_fifo_cnt      (flash_check_fifo_cnt),
        .fifo_re_done_cnt    (flash_fifo_re_done_cnt),
        .fifo_latch_done_cnt (flash_fifo_latch_done_cnt),
        .fifo_prog_cnt       (flash_fifo_prog_cnt),
        .fifo_was_empty_at_dump (flash_fifo_was_empty_at_dump),
        .fifo_state_at_dump_start (dump_start_fifo_state),
        .fifo_total_cnt_at_dump_start (dump_start_fifo_cnt),
        .read_mode           (flash_read_mode),  // 读取模式：dump时拉高，优先处理读取
        .flush_active        (flush_active),     // FIFO正在写入Flash标志
        .dump_flush_force    (dump_force_flush_wire)  // 强制flush：绕过fifo_full判断
    );

    // ------------------------------------------------------------------------
    // 4. FIFO 读模�?
    // ------------------------------------------------------------------------
    fifo_read_module u_fifo_read_flash (
        .sys_clk         (clk),
        .sys_rst_n       (rst_n),
        .start_read      (dump_read_active),
        .read_start_addr (dump_read_start_addr),
        .use_word_limit  (dump_active && (dump_word_limit != 16'd0)),
        .word_limit      (dump_word_limit),
        .ff_stop_run     (8'd16),
        .rd_req          (fl_rd_req),
        .rd_addr         (fl_rd_addr),
        .rd_data         (fl_rd_data),
        .rd_data_valid   (fl_rd_data_valid),
        .flash_busy      (flash_busy),
        .dout_valid      (read_dout_valid),
        .dout_ready      (read_dout_ready),
        .dout_value      (read_diff_result),
        .dout_empty      (read_dout_empty),
        .read_done       (read_done),
        .dbg_first_byte  (dbg_rd_first_byte),
        .dbg_first_byte_valid (dbg_rd_first_valid)
    );

    // ------------------------------------------------------------------------
    // 5. 1Hz 脉冲生成
    // ------------------------------------------------------------------------
    clk_1hz_gen u_sec_tick (
        .clk             (clk),
        .rst_n           (rst_n),
        .sec_tick        (sec_tick)
    );

    // ------------------------------------------------------------------------
    // 6. 均匀补偿模块
    // ------------------------------------------------------------------------
    avg_remain_comp u_avg_remain_comp (
        .clk               (clk),
        .rst_n             (rst_n),
        .diff_update_valid (comp_diff_valid),
        .diff_value        (comp_diff_value),
        .sec_tick          (sec_tick),
        .comp_valid        (comp_valid),
        .comp_step         (comp_step)
    );

    // ========================================================================
    // SPI总线在“写入控制器”和“训练回放读取器”之间复用
    // dump流程下必须先完成flush写入，再切到读取SPI，避免把写通道抢占掉
    // Single SPI driver in flash_controller_enhanced
    assign spi_sclk = flash_spi_sclk;
    assign spi_mosi = flash_spi_mosi;
    assign spi_cs_n = flash_spi_cs_n;

    // LED输出
    assign uart_led = uart_tx_active;
    assign gps_led = gps_pps_detected;
    
    // ========================================================================
    // 串口传输状态检�?
    // ========================================================================
    reg         uart_tx_active;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_tx_active <= 1'b0;
        end else begin
            if (tx_en) begin
                uart_tx_active <= 1'b1;
            end else if (sec_tick) begin
                uart_tx_active <= 1'b0;
            end
        end
    end
    
    // ========================================================================
    // 指示灯控制逻辑 (保持不变)

    // LED输出（简化版：无计数器）
    assign uart_led = uart_tx_active;
    assign gps_led = gps_pps_detected;

endmodule

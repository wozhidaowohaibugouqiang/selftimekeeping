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
    
    output wire         uart_tx,        // UART 发�?
    output wire         dpps_out,       // 补偿后的 DPPS 输出
    output wire         spi_sclk,       // Flash SPI Clock
    output wire         spi_mosi,       // Flash SPI MOSI
    output wire         spi_cs_n,       // Flash SPI CS_n
    output wire         uart_led,       // 串口传输状态指示灯
    output wire         gps_led         // GPS PPS信号接收指示�?
);

    // ========================================================================
    // 参数定义
    // ========================================================================
    parameter integer CLK_CYCLES_PER_SEC = 50_000_000;
    parameter integer GPS_LOST_THRESHOLD = 100_000_000; // 2秒超�?

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
    
    // 新增：状态变化标�?
    reg [3:0] last_sys_state;
    reg state_changed;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_sys_state <= 4'd0;
            state_changed <= 1'b0;
        end else begin
            if (sys_state != last_sys_state) begin
                state_changed <= 1'b1;
                last_sys_state <= sys_state;
            end else if (uart_tx_done) begin
                state_changed <= 1'b0;
            end
        end
    end
    
    // 新增：ST_PARAM_UPDATE 状态持续计数器
    reg [3:0] param_update_cnt;
    
    // 调试信号
    wire [2:0]   dbg_state;
    wire [7:0]   dbg_pps_cnt;
    wire         dbg_gps_pps_posedge;
    wire         dbg_counting_active;
    
    // 新增：period_done 计数�?
    reg [31:0] period_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) period_cnt <= 32'd0;
        else if (period_done) period_cnt <= period_cnt + 32'd1;
    end

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
        end else if (aging_diff_write_valid && !fifo_full) begin
            fwe_cnt <= fwe_cnt + 16'd1;
        end
    end

    // 新增：差值有效脉冲计数（用于定位是否在同步链路丢失）
    reg [15:0] dv50_cnt; // core_diff_valid 计数（源�?
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dv50_cnt <= 16'd0;
        end else if (core_diff_valid) begin
            dv50_cnt <= dv50_cnt + 16'd1;
        end
    end

    // 新增：FIFO读使能事件计数器（用于观测Flash是否在持续读走FIFO�?
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
    
    // 新增：进入FIFO写模块的diff_valid计数（用于判断同步链路是否丢脉冲�?
    reg [15:0] dv25_cnt; // sync_diff_valid[1] 计数（在时钟域同步块中更新）
    
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
    reg signed [31:0]  base_offset;
    reg signed [31:0]  total_compensation;
    reg signed [23:0]  dynamic_offset;
    reg signed [23:0]  dpps_offset;
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
    wire        core_dbg_dpps_posedge;      // 来自 timekeeping_core 的DPPS上升沿检�?
    
    // FIFO相关信号
    wire [31:0] fifo_dout;
    wire        fifo_empty;
    wire        fifo_full;
    wire        fifo_re;
    wire        flush_req;
    wire        fifo_we_debug;
    wire [1:0]  fifo_state_debug;
    
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
    wire        fl_rd_req;
    wire [23:0] fl_rd_addr;
    wire [7:0]  fl_rd_data;
    wire        fl_rd_data_valid;
    wire        fl_rd_req;
    wire [23:0] fl_rd_addr;
    wire [7:0]  fl_rd_data;
    wire        fl_rd_data_valid;
    wire [7:0]  dbg_rd_first_byte;
    wire        dbg_rd_first_valid;

    // 临时方案：固定导出上限，用于读取“历史老数据”（不依赖运行时计数器）
    localparam [15:0] DUMP_WORD_LIMIT_FIXED = 16'd3600;

    // Dump导出控制信号
    reg         dump_active;
    reg         dump_arm_wait_window;
    reg         dump_force_flush;
    reg         dump_read_active;
    reg         dump_pending;
    reg         dump_done_flag;
    reg [15:0]  dump_word_limit;
    reg [23:0]  dump_read_start_addr;
    reg [23:0]  dump_addr;
    reg [31:0]  dump_data;
    reg         dump_meta_pending;
    reg [31:0]  dump_flush_timeout;
    reg [15:0]  dump_meta_fwe;
    reg [15:0]  dump_meta_fre;
    reg [15:0]  dump_meta_limit;
    reg [15:0]  dump_exported_words;
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
    wire        fl_rd_req_mux;
    wire [23:0] fl_rd_addr_mux;

    assign fc_param_valid = md_param_valid;
    assign fc_param_data  = md_param_data;
    assign fc_param_addr  = md_param_addr;
    assign fc_param_type  = 2'b00;

    assign fl_rd_req_mux  = md_boot_active ? md_rd_req : fl_rd_req_dump;
    assign fl_rd_addr_mux = md_boot_active ? md_rd_addr : fl_rd_addr_dump;

    
    wire        dump_meta_ready;
    wire        dump_word_ready;
    wire        dump_word_valid = dump_pending;
    wire [23:0] dump_word_addr = dump_addr;
    wire [31:0] dump_word_data = dump_data;
    reg  [2:0]  dump_key_sync;
    reg         dump_key_prev;
    wire        dump_key_pressed;
    // erase_key 同步去抖
    reg  [2:0]  erase_key_sync;
    reg         erase_key_prev;
    wire        erase_key_pressed;
    wire        read_dout_ready;

    // UART相关信号
    wire        tx_en;
    
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
    wire        period_diff_pulse = period_done && diff_valid && gps_stable && !dump_active;

    reg [3:0]         comp_settle_cnt;
    reg signed [39:0] comp_init_sum;
    reg [3:0]         comp_init_cnt;
    reg               comp_param_locked;
    reg signed [31:0] comp_param_fixed;

    assign aging_diff_value = core_diff_result;
    assign dynamic_diff_value = aging_diff_value;

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
    
    reg [31:0] gps_timeout_cnt;               // 超时计数器
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
            gps_timeout_cnt <= 32'd0;
            gps_lost <= 1'b1; // 复位时认为丢�?
        end else begin
            if (pps_rising) begin
                gps_timeout_cnt <= 32'd0;
                gps_lost <= 1'b0;
            end else if (gps_timeout_cnt < GPS_LOST_THRESHOLD) begin
                gps_timeout_cnt <= gps_timeout_cnt + 32'd1;
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

    // 动态补偿就绪判定：GPS稳定后先等待10个16秒窗口，再用后10个16秒窗口求平均并锁定参数
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dynamic_comp_ready <= 1'b0;
            comp_settle_cnt <= 4'd0;
            comp_init_sum <= 40'sd0;
            comp_init_cnt <= 4'd0;
            comp_param_locked <= 1'b0;
            comp_param_fixed <= 32'sd0;
        end else if (gps_lost || !gps_stable) begin
            dynamic_comp_ready <= 1'b0;
            comp_settle_cnt <= 4'd0;
            comp_init_sum <= 40'sd0;
            comp_init_cnt <= 4'd0;
            comp_param_locked <= 1'b0;
            comp_param_fixed <= 32'sd0;
        end else if (period_diff_pulse) begin
            if (!comp_param_locked) begin
                // 先稳定等待10个窗口，不参与训练
                if (comp_settle_cnt < 4'd10) begin
                    comp_settle_cnt <= comp_settle_cnt + 4'd1;
                    comp_init_sum <= 40'sd0;
                    comp_init_cnt <= 4'd0;
                end else begin
                    // 再采10个窗口求平均并锁定
                    comp_init_sum <= comp_init_sum + $signed(diff_total[31:0]);
                    if (comp_init_cnt == 4'd9) begin
                        comp_param_fixed <= $signed((comp_init_sum + $signed(diff_total[31:0])) / 10);
                        comp_param_locked <= 1'b1;
                        dynamic_comp_ready <= 1'b1;
                    end else begin
                        comp_init_cnt <= comp_init_cnt + 4'd1;
                    end
                end
            end
        end
    end

    assign aging_diff_valid = core_diff_valid & gps_stable;
    assign dynamic_diff_valid = core_diff_valid & gps_stable;

    // Dump按键同步与控制流程：
    // 1) 按下P52(低有效)后先等待当前16s窗口结束
    // 2) 触发一次强制flush，待flash_done后进入只读导出
    // 3) 读一条发一条（ready/valid握手），保证115200下不丢包
    assign dump_key_pressed = dump_key_prev && !dump_key_sync[2];
    assign erase_key_pressed = erase_key_prev && !erase_key_sync[2];
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
            dump_read_start_addr <= 24'h001000;
            dump_addr <= 24'h001000;
            dump_data <= 32'd0;
            dump_meta_pending <= 1'b0;
            dump_flush_timeout <= 32'd0;
            dump_meta_fwe <= 16'd0;
            dump_meta_fre <= 16'd0;
            dump_meta_limit <= 16'd0;
            dump_exported_words <= 16'd0;
            erase_key_sync <= 3'b111;
            erase_key_prev <= 1'b1;
        end else begin
            dump_key_sync <= {dump_key_sync[1:0], dump_key};
            dump_key_prev <= dump_key_sync[2];
            erase_key_sync <= {erase_key_sync[1:0], erase_key};
            erase_key_prev <= erase_key_sync[2];

            if (dump_key_pressed && !dump_active) begin
                // 按键后立即进入Dump流程：
                // 1) 直接跳过当前16s统计窗口（用户接受）
                // 2) 立即触发一次强制flush，将FIFO现有数据写入Flash
                // 3) flush完成后开始读取并串口导出
                dump_active <= 1'b1;
                dump_arm_wait_window <= 1'b0;
                dump_force_flush <= 1'b1;
                dump_read_active <= 1'b0;
                dump_pending <= 1'b0;
                dump_done_flag <= 1'b0;
                dump_exported_words <= 16'd0;
                dump_word_limit <= fwe_cnt;
                dump_read_start_addr <= 24'h001000;
                dump_addr <= 24'h001000;
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
                dump_read_start_addr <= 24'h001000;
                dump_addr <= 24'h001000;
            end

            // Increment timeout counter while waiting for write_done
            if (dump_active && dump_force_flush && !write_done)
                dump_flush_timeout <= dump_flush_timeout + 32'd1;
            else
                dump_flush_timeout <= 32'd0;
            // Proceed after write_done OR after 5s timeout
            if (dump_active && dump_force_flush && (write_done || dump_flush_timeout >= 32'd250000000)) begin
                dump_force_flush <= 1'b0;
                dump_read_active <= 1'b1;
                dump_meta_pending <= 1'b1;
                dump_meta_fwe <= fwe_cnt;
                dump_meta_fre <= fre_cnt;
                // 新增：计算当前 Flash 中所有有效老化参数记录的数量 (FWE写入的总数)
                dump_word_limit <= fwe_cnt; // 改为实际写入的总条数，实现“暴力全量回传”
                dump_read_start_addr <= 24'h001000;
                dump_addr <= 24'h001000;
                dump_meta_limit <= fwe_cnt; // 更新元数据中的长度
            end

            if (dump_meta_pending && dump_meta_ready) begin
                dump_meta_pending <= 1'b0;
            end

            if (dump_active && dump_read_active) begin
                if (read_dout_valid && !dump_pending) begin
                    dump_data <= read_diff_result;
                    dump_pending <= 1'b1;
                end

                if (dump_pending && dump_word_ready) begin
                    dump_pending <= 1'b0;
                    dump_addr <= dump_addr + 24'd4;
                    dump_exported_words <= dump_exported_words + 16'd1;
                end

                if (read_done && read_dout_empty && !dump_pending) begin
                    dump_done_flag <= 1'b1;
                    dump_read_active <= 1'b0;
                    // Dump流程结束后退出dump模式，恢复正常心跳与状态机
                    dump_active <= 1'b0;
                end
            end
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
        .diff_valid_flag  (aging_diff_valid),
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

    // GPS丢失 OR 非计算状态 -> 停止采集并触发刷写
    // 互斥保护：当处于 ST_TRAIN (读取模式) 时，禁止触发 Flush (写入模式)
    // 节流：仅当FIFO累计写入比读出至少多10次时，才允许触发Flash读FIFO
    wire flush_batch_ready = (fwe_cnt >= (fre_cnt + 16'd4));
    assign flush_req = (((gps_lost || (sys_state != ST_CALC)) && (sys_state != ST_TRAIN) && flush_batch_ready) || dump_force_flush) && !dump_read_active;
    
    // ========================================================================
    // 补偿计算逻辑
    // ========================================================================
    localparam signed [23:0] DYNAMIC_OFFSET_MAX = 24'sh7FFFFF;
    localparam signed [23:0] DYNAMIC_OFFSET_MIN = -24'sh800000;
    reg signed [24:0] dyn_sum;
    reg signed [31:0] base_sum;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            base_offset <= 32'sd0;
            total_compensation <= 32'sd0;
            dynamic_offset <= 24'sd0;
            dpps_offset <= 24'sd0;
            coeff_read_addr <= 8'd0;
            coeff_read_en <= 1'b0;
            coeff_update_en <= 1'b0;
            new_coeff_value <= 32'sd0;
        end else begin
            if (sec_tick) begin
                coeff_read_addr <= coeff_read_addr + 8'd1;
                coeff_read_en <= 1'b1;

                if (!dump_active) begin
                    if (!gps_lost && (sys_state == ST_CALC) && dynamic_comp_ready) begin
                        // 每秒直接应用当前步进，不做跨秒累加，避免补偿量线性漂移
                        dynamic_offset <= $signed({{8{comp_step[15]}}, comp_step});
                    end

                    if (!gps_lost && coeff_data_valid) begin
                        base_sum = $signed(quotient[31:0]) + $signed(aging_comp_value) + (coeff_data_out ? 32'sd1 : 32'sd0);
                        base_offset <= base_sum;
                        total_compensation <= base_sum;
                    end
                end
            end else begin
                coeff_read_en <= 1'b0;
            end

            dpps_offset <= $signed(base_offset[23:0]) + $signed(dynamic_offset);

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

    // ------------------------------------------------------------------------
    // 0. UART 模块
    //    负责上位机指令交互与状态回�?
    // ------------------------------------------------------------------------
    wire uart_tx_done;
    wire uart_erase_cmd;
    wire combined_erase_req = erase_key_pressed || uart_erase_cmd;

    wire        period_diff_pulse;
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
        .fifo_dout  (fifo_dout[7:0]),
        .train_done (sys_state == ST_TRAIN_DONE),
        // 调试输入信号
        .dbg_state  (dbg_state),
        .dbg_pps_cnt ({4'd0, core_pps_cnt_8}),
        .period_diff_pulse   (period_diff_pulse),
        .dbg_gps_pps_posedge (core_gps_pps_posedge),
        .dbg_counting_active (dbg_counting_active),
        // 新增：period_done 计数�?
        .period_cnt (period_cnt),
        // 新增：FIFO写入事件计数
        .fwe_cnt    (fwe_cnt),
        // 新增：FIFO读出事件计数
        .fre_cnt    (fre_cnt),
        // 新增：diff_valid脉冲计数（源/写入端）
        .dv50_cnt   (dv50_cnt),
        .dv25_cnt   (dv25_cnt),
        // 新增：状态变化标�?
        .state_changed (state_changed),
        // 新增：diff_valid调试信号
        .dbg_diff_valid (core_dbg_diff_valid),
        // 新增：差值数据
        .diff_result (dynamic_diff_value),
        .aging_diff_result (aging_diff_value),
        // 新增：高精度计数调试信号
        .dbg_gps_phase_sum   (core_gps_phase_sum),
        .dbg_dpps_phase_sum  (core_dpps_phase_sum),
        .dbg_dpps_phase_cnt  (core_dbg_dpps_phase_cnt),
        .dbg_dpps_pulse_cnt  (core_dbg_dpps_pulse_cnt),
        .dbg_dpps_pulse_active (core_dbg_dpps_pulse_active),
        .dbg_dpps_cnt  (core_dbg_dpps_cnt),
        .dbg_dpps_posedge  (core_dbg_dpps_posedge),
        // 新增：FIFO流程检测信号
        .fifo_flush_req  (flush_req),
        .fifo_we_debug  (fifo_we_debug),
        .fifo_state_debug  (fifo_state_debug),
        // 新增：Flash状态检测信号
        .flash_busy  (flash_busy),
        .flash_done  (flash_done),
        .write_done  (write_done),
        .flash_error  (flash_error),
        .dbg_rd_first_byte  (dbg_rd_first_byte),
        .dbg_rd_first_valid (dbg_rd_first_valid),
        .dbg_flash_cur_addr (flash_cur_addr_dbg),
        .dbg_poll_attempts  (flash_poll_attempts_dbg),
        .dbg_last_sr        (flash_last_sr_dbg),
        .dbg_sr_stage       (flash_sr_stage_dbg),
        .dbg_last_cmd       (flash_last_cmd_dbg),
        .dbg_last_len       (flash_last_len_dbg),
        .dbg_last_rx0       (flash_last_rx0_dbg),
        .dbg_last_rx1       (flash_last_rx1_dbg),
        .dbg_err_code       (flash_err_code_dbg),
        .dbg_jedec_id       (flash_jedec_id_dbg),
        .dbg_jedec_valid    (flash_jedec_valid_dbg),
        .dbg_flash_state    (flash_state_dbg),
        .dbg_txn_done_cnt   (flash_txn_done_cnt),
        .hist_diff  (hist_diff),
        .hist_diff_valid  (hist_diff_valid),
        .read_diff       (read_diff_result),
        .read_diff_valid (read_dout_valid),
        .dpps_offset_dbg (dpps_offset),
        .dyn_offset_dbg  (dynamic_offset),
        .comp_step_dbg   (comp_step),
        .comp_valid_dbg  (comp_valid),
        .dyn_ready_dbg   (dynamic_comp_ready),
        .comp_fixed_dbg  (comp_param_fixed),
        .last_prog_word  (last_prog_word_mirror),
        .dump_mode      (dump_active),
        .dump_word_valid(dump_word_valid),
        .dump_word_addr (dump_word_addr),
        .dump_word_data (dump_word_data),
        .dump_done      (dump_done_flag),
        .dump_meta_valid(dump_meta_pending),
        .dump_meta_fwe  (dump_meta_fwe),
        .dump_meta_fre  (dump_meta_fre),
        .dump_meta_limit(dump_meta_limit),
        .dump_meta_exported(dump_exported_words),
        .dump_word_ready(dump_word_ready),
        .dump_meta_ready(dump_meta_ready),
        .state_cmd  (uart_cmd),
        .cmd_valid  (uart_cmd_valid),
        .tx_en      (tx_en),
        .tx_done    (uart_tx_done)
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
        .dbg_dpps_pulse_active (core_dbg_dpps_pulse_active)
    );
    assign dpps_out = core_dpps;

    // ------------------------------------------------------------------------
    // 2. FIFO 写模�?
    // ------------------------------------------------------------------------
    // 时钟域同步：将“老化采样差值”同步到FIFO写入链路
    reg [1:0] sync_diff_valid; // 用于同步aging_diff_valid的两级触发器
    reg signed [31:0] sync_diff_value; // 同步后的完整32位老化差值
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_diff_valid <= 2'b00;
            sync_diff_value <= 32'sd0;
        end else begin
            sync_diff_valid <= 2'b00; // 同一时钟域，不需要延迟
            sync_diff_value <= 32'sd0;
        end
    end

    // 统计进入 FIFO 写模块的 diff_valid 脉冲次数
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dv25_cnt <= 16'd0;
        end else if (aging_diff_write_valid) begin
            dv25_cnt <= dv25_cnt + 16'd1;
        end
    end
    
    fifo_write_module u_fifo_write (
        .sys_clk_25mhz   (clk),
        .sys_rst_n       (rst_n),
        .flush_req       (flush_req),
        .diff_value      (aging_diff_value),
        .diff_valid_flag (aging_diff_write_valid),  
        .fifo_re         (fifo_re),
        .fifo_dout       (fifo_dout),
        .fifo_empty_flag (fifo_empty),
        .fifo_full_flag  (fifo_full),
        .fifo_we_debug   (fifo_we_debug),
        .fifo_state_debug(fifo_state_debug)
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
        .txn_done_cnt_dbg    (flash_txn_done_cnt)
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

    // 可选 ChipWatcher（工具在部分版本下可能导致 place 崩溃）
`ifdef ENABLE_CHIPWATCHER
    wire cw_st_wren_erase = (flash_state_dbg == 7'd3);
    wire cw_st_send_erase = (flash_state_dbg == 7'd4);
    wire cw_st_poll_erase = (flash_state_dbg == 7'd5);

    ChipWatcher_8e2139500150 u_chipwatcher (
        .probe0 (spi_cs_n),
        .probe1 (spi_sclk),
        .probe2 (spi_mosi),
        .probe3 (spi_miso),
        .probe4 (flash_busy),
        .probe5 (flash_done),
        .probe6 (flash_error),
        .probe7 (cw_st_wren_erase),
        .probe8 (cw_st_send_erase),
        .probe9 (cw_st_poll_erase),
        .clk    (clk)
    );
`endif
    
    // 指示灯输�?
    assign uart_led = uart_led_reg;
    assign gps_led = gps_led_reg;
    
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
    // ========================================================================
    reg         uart_led_reg;
    reg         gps_led_reg;
    reg [31:0]  uart_led_counter;
    reg [31:0]  gps_led_counter;

    // 串口指示灯（2秒周期，�?秒灭1秒）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_led_reg <= 1'b0;
            uart_led_counter <= 32'd0;
        end else begin
            if (uart_tx_active) begin
                if (uart_led_counter < CLK_CYCLES_PER_SEC) begin
                    uart_led_reg <= 1'b1;
                    uart_led_counter <= uart_led_counter + 32'd1;
                end else if (uart_led_counter < CLK_CYCLES_PER_SEC * 2) begin
                    uart_led_reg <= 1'b0;
                    uart_led_counter <= uart_led_counter + 32'd1;
                end else begin
                    uart_led_counter <= 32'd0;
                end
            end else begin
                uart_led_reg <= 1'b0;
                uart_led_counter <= 32'd0;
            end
        end
    end
    
    // GPS指示灯（1秒周期，�?.5秒灭0.5秒）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gps_led_reg <= 1'b0;
            gps_led_counter <= 32'd0;
        end else begin
            if (gps_pps_detected) begin
                if (gps_led_counter < CLK_CYCLES_PER_SEC / 2) begin
                    gps_led_reg <= 1'b1;
                    gps_led_counter <= gps_led_counter + 32'd1;
                end else if (gps_led_counter < CLK_CYCLES_PER_SEC) begin
                    gps_led_reg <= 1'b0;
                    gps_led_counter <= gps_led_counter + 32'd1;
                end else begin
                    gps_led_counter <= 32'd0;
                end
            end else begin
                gps_led_reg <= 1'b0;
                gps_led_counter <= 32'd0;
            end
        end
    end
    




endmodule
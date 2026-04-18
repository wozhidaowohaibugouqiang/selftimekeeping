// 自主守时核心模块 (改进版 - 高精度100MHz计数)
// 模块功能：用50MHz时钟计数器计算GPS PPS与DPPS的周期差
// 设计作者：TraeAI
// 设计日期：2026-01-19
// 版本号：v3.0 (高精度计数版)
//
// 改进说明：
// - 在16秒内（16个PPS周期）
// - 用dpps_phase_cnt（50MHz计数器）计数GPS这16个周期内有多少个时钟脉冲
// - 用dpps_phase_cnt（50MHz计数器）计数DPPS这16个周期内有多少个时钟脉冲
// - 对比两个计数：diff = (GPS_count - DPPS_count) / 16
// - 得到真实的周期差（精度：20ns）

module timekeeping_core
(
    input           clk,            // 50MHz 系统时钟
    input           sys_rst_n,      // 系统复位，低有效
    input           gps_pps,        // GPS标准PPS信号
    input [23:0]    dpps_offset,    // DPPS动态偏移量输入
    input [7:0]     init_diff,
    input           init_diff_valid,
    output reg      dpps,           // 带偏差的本地DPPS
    output reg signed [31:0] diff_result, // 差值（32位有符号，不除以16）
    output reg      diff_valid_flag, // 有效标志位
    output reg [3:0] pps_cnt_8,     // PPS计数器，用于调试
    output reg      gps_pps_posedge, // GPS PPS上升沿，用于调试
    output reg      dbg_diff_valid,  // 调试用的diff_valid_flag
    output wire [31:0] dbg_gps_phase_sum,   // 调试：GPS累积计数
    output wire [31:0] dbg_dpps_phase_sum,  // 调试：DPPS累积计数
    output wire        dbg_dpps,            // 调试：DPPS脉冲输出
    output wire [31:0] dbg_dpps_phase_cnt,  // 调试：DPPS相位计数器
    output wire [31:0] dbg_dpps_pulse_cnt,  // 调试：DPPS脉冲宽度计数器
    output wire        dbg_dpps_pulse_active, // 调试：DPPS脉冲活跃标志
    output wire [3:0]  dbg_dpps_cnt,        // 调试：DPPS上升沿计数
    output wire        dbg_dpps_posedge,     // 调试：DPPS上升沿检测
    // 新增：调试输出 dpps_offset 和 dpps_period_target
    output wire [23:0] dbg_dpps_offset,      // 调试：接收到的 dpps_offset
    output wire [31:0] dbg_dpps_period_target // 调试：计算后的 dpps_period_target
);

parameter integer CLK_CYCLES_PER_SEC = 50_000_000; // 50MHz -> 1s count
parameter integer WIN_SECS = 16;  // 16个PPS周期为一个窗口

// ========================================================================
// GPS PPS 同步与边沿检测
// ========================================================================
reg [1:0] gps_pps_sync;
reg gps_pps_posedge_int;

always @(posedge clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        gps_pps_sync <= 2'b00;
        gps_pps_posedge_int <= 1'b0;
    end else begin
        gps_pps_sync <= {gps_pps_sync[0], gps_pps};
        gps_pps_posedge_int <= (gps_pps_sync == 2'b01);
    end
end

// ========================================================================
// DPPS 生成逻辑（改进版 v3 - 清晰的状态机）
// ========================================================================
// 生成精准的1Hz DPPS信号，脉冲宽度与GPS PPS一致
reg signed [31:0] comp_offset_cycles;  // 补偿偏移量（用于后续的差值补偿）
wire signed [31:0] dpps_offset_signed;
wire signed [31:0] dpps_period_target;
reg [31:0] dpps_phase_cnt;             // 相位计数器（0 ~ 50M-1）
reg [31:0] dpps_pulse_cnt;             // 脉冲宽度计数器

// 独立的自由运行计数器，用于测量GPS和DPPS的实际周期
reg [31:0] free_cnt;  // 50MHz自由运行计数器（不复位）
always @(posedge clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        free_cnt <= 32'd0;
    else
        free_cnt <= free_cnt + 32'd1;
end

// 直接使用硬编码的50M周期，不依赖计算
parameter signed [31:0] DPPS_PERIOD = 32'sd50_000_000;  // 50MHz时钟的1秒周期（有符号数）
parameter integer DPPS_PULSE_WIDTH = 5_000_000;  // 100ms脉冲宽度

// 注意：dpps_offset 的符号约定
// diff_result = gps_cycles - dpps_cycles
//   D < 0: DPPS太快，需要增大周期
//   D > 0: DPPS太慢，需要减小周期
//
// comp_param_fixed 已经取反，与 diff_result 符号相反
//   D < 0 → comp_param_fixed > 0 → dpps_offset > 0
//   要增大周期：dpps_period_target = DPPS_PERIOD + (正offset) = DPPS_PERIOD + |offset|
assign dpps_offset_signed = {{8{dpps_offset[23]}}, dpps_offset};
assign dpps_period_target = DPPS_PERIOD + dpps_offset_signed;

// 新增：调试输出
assign dbg_dpps_offset = dpps_offset;
assign dbg_dpps_period_target = dpps_period_target;

always @(posedge clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        dpps_phase_cnt <= 32'd0;
        dpps_pulse_cnt <= 32'd0;
        dpps <= 1'b0;
    end else begin
        // 相位计数器逻辑：从0递增到DPPS_PERIOD-1，然后复位
        if (dpps_phase_cnt >= (dpps_period_target - 1)) begin
            // 到达周期末尾，复位计数器并启动脉冲
            dpps_phase_cnt <= 32'd0;
            dpps_pulse_cnt <= 32'd0;
            dpps <= 1'b1;  // 脉冲开始
        end else begin
            // 继续计数
            dpps_phase_cnt <= dpps_phase_cnt + 32'd1;
            
            // 脉冲宽度控制
            if (dpps_pulse_cnt < DPPS_PULSE_WIDTH) begin
                // 脉冲还在进行中
                dpps_pulse_cnt <= dpps_pulse_cnt + 32'd1;
                dpps <= 1'b1;
            end else begin
                // 脉冲已结束
                dpps <= 1'b0;
            end
        end
    end
end

// ========================================================================
// DPPS 边沿检测
// ========================================================================
reg dpps_prev;
wire dpps_posedge;

always @(posedge clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        dpps_prev <= 1'b0;
    else
        dpps_prev <= dpps;
end

assign dpps_posedge = (dpps == 1'b1) && (dpps_prev == 1'b0);

// ========================================================================
// 高精度计数逻辑：用50MHz时钟计算GPS和DPPS的周期差
// ========================================================================

// 累积计数 - 改为累积实际测量的周期数
reg [31:0] gps_total_cycles;   // 16个GPS周期内的实际时钟周期累积
reg [31:0] dpps_total_cycles;  // 16个DPPS周期内的实际时钟周期累积
reg [31:0] gps_window_cycles_dbg;   // 调试：上一个完整16s窗口的GPS总周期
reg [31:0] dpps_window_cycles_dbg;  // 调试：上一个完整16s窗口的DPPS总周期
reg [3:0] gps_cnt;             // GPS周期计数（0~15，表示已累积的1s区间数）
reg [3:0] dpps_cnt;            // DPPS上升沿计数（调试）
reg [31:0] gps_phase_cnt_prev;  // GPS采样时的自由计数器前值
reg [31:0] dpps_phase_cnt_prev; // DPPS采样时的自由计数器前值
reg        gps_window_active;   // 16s窗口已起始标志
reg        dpps_window_active;  // DPPS窗口已起始标志
reg [2:0]  gps_warmup_win_cnt;  // GPS稳定预热窗口计数（0~7）
reg        gps_edge_seen;       // 是否已经看到过GPS上升沿
reg [1:0]  gps_stable_edge_cnt; // 连续边沿计数（达到3后才允许进入16s窗口）
reg        gps_edge_since_dpps; // 最近1秒内是否看到GPS边沿
reg [1:0]  gps_miss_sec_cnt;    // 连续丢GPS秒计数（用于中途插拔废窗）

// 计算中间变量
reg signed [31:0] period_diff;
reg signed [31:0] avg_period_diff;
reg signed [31:0] mag;
reg signed [31:0] init_total;
reg signed [31:0] init_avg;

always @(posedge clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        gps_total_cycles <= 32'd0;
        dpps_total_cycles <= 32'd0;
        gps_window_cycles_dbg <= 32'd0;
        dpps_window_cycles_dbg <= 32'd0;
        gps_cnt <= 4'd0;
        dpps_cnt <= 4'd0;
        gps_phase_cnt_prev <= 32'd0;
        dpps_phase_cnt_prev <= 32'd0;
        gps_window_active <= 1'b0;
        dpps_window_active <= 1'b0;
        gps_warmup_win_cnt <= 3'd0;
        gps_edge_seen <= 1'b0;
        gps_stable_edge_cnt <= 2'd0;
        gps_edge_since_dpps <= 1'b0;
        gps_miss_sec_cnt <= 2'd0;
        pps_cnt_8 <= 4'd0;
        gps_pps_posedge <= 1'b0;
        diff_valid_flag <= 1'b0;
        dbg_diff_valid <= 1'b0;
        comp_offset_cycles <= 32'sd0;
    end else begin
        diff_valid_flag <= 1'b0;
        dbg_diff_valid <= 1'b0;
        gps_pps_posedge <= gps_pps_posedge_int;

        // 处理初始差值
        if (init_diff_valid) begin
            init_total <= init_diff[7] ? -$signed({25'd0, init_diff[6:0]}) : $signed({25'd0, init_diff[6:0]});
            init_avg <= init_total >>> 4;
            comp_offset_cycles <= comp_offset_cycles + init_avg;
        end

        // 每个DPPS上升沿：记录本地实际周期（用于动态残差）
        // 同时做轻量GPS稳定检测：若连续2秒未见GPS边沿，则判为波动并废弃当前窗口
        if (dpps_posedge) begin
            dpps_cnt <= dpps_cnt + 4'd1;
            if (!dpps_window_active) begin
                dpps_window_active <= 1'b1;
                dpps_phase_cnt_prev <= free_cnt;
            end else if (gps_window_active) begin
                dpps_total_cycles <= dpps_total_cycles + (free_cnt - dpps_phase_cnt_prev);
                dpps_phase_cnt_prev <= free_cnt;
            end

            if (gps_edge_seen) begin
                if (gps_edge_since_dpps) begin
                    gps_edge_since_dpps <= 1'b0;
                    gps_miss_sec_cnt <= 2'd0;
                end else begin
                    if (gps_miss_sec_cnt < 2'd3)
                        gps_miss_sec_cnt <= gps_miss_sec_cnt + 2'd1;
                    if (gps_miss_sec_cnt >= 2'd1) begin
                        // 连续2秒未见GPS：视为中途插拔/波动，清空窗口并重新等待稳定
                        gps_window_active <= 1'b0;
                        dpps_window_active <= 1'b0;
                        gps_warmup_win_cnt <= 3'd0;
                        gps_stable_edge_cnt <= 2'd0;
                        gps_cnt <= 4'd0;
                        pps_cnt_8 <= 4'd0;
                        gps_total_cycles <= 32'd0;
                        dpps_total_cycles <= 32'd0;
                        dpps_cnt <= 4'd0;
                        dpps_phase_cnt_prev <= 32'd0;
                    end
                end
            end
        end

        // 每个GPS PPS上升沿
        if (gps_pps_posedge_int) begin
            gps_edge_since_dpps <= 1'b1;

            if (!gps_edge_seen) begin
                // 首次看到GPS边沿：仅建立参考，不做窗口计算
                gps_edge_seen <= 1'b1;
                gps_stable_edge_cnt <= 2'd1;
                gps_window_active <= 1'b0;
                dpps_window_active <= 1'b0;
                gps_warmup_win_cnt <= 3'd0;
                gps_cnt <= 4'd0;
                pps_cnt_8 <= 4'd0;
                gps_total_cycles <= 32'd0;
                dpps_total_cycles <= 32'd0;
                dpps_cnt <= 4'd0;
                dpps_phase_cnt_prev <= 32'd0;
            end else begin
                if (gps_stable_edge_cnt < 2'd3)
                    gps_stable_edge_cnt <= gps_stable_edge_cnt + 2'd1;

                // 需连续看到3个GPS边沿后，才允许进入16s窗口计算
                if (gps_stable_edge_cnt >= 2'd2) begin
                    if (!gps_window_active) begin
                        // 0s起点：仅锁存起点，不累计本秒
                        gps_window_active <= 1'b1;
                        gps_phase_cnt_prev <= free_cnt;
                        gps_cnt <= 4'd0;
                        pps_cnt_8 <= 4'd0;
                        gps_total_cycles <= 32'd0;
                        dpps_total_cycles <= 32'd0;
                        dpps_cnt <= 4'd0;
                        dpps_window_active <= 1'b0;
                        dpps_phase_cnt_prev <= 32'd0;
                        gps_warmup_win_cnt <= 3'd0;
                    end else begin
                        // 窗口内每个1s区间累加GPS周期
                        gps_total_cycles <= gps_total_cycles + (free_cnt - gps_phase_cnt_prev);
                        gps_phase_cnt_prev <= free_cnt;

                        if (gps_cnt == 4'd15) begin
                            // 满16个1s区间（0s->16s）完成，输出整窗结果
                            gps_window_cycles_dbg <= gps_total_cycles + (free_cnt - gps_phase_cnt_prev);
                            dpps_window_cycles_dbg <= dpps_total_cycles;

                            period_diff <= $signed(gps_total_cycles + (free_cnt - gps_phase_cnt_prev)) - $signed(dpps_total_cycles);
                            diff_result <= $signed(gps_total_cycles + (free_cnt - gps_phase_cnt_prev)) - $signed(dpps_total_cycles);
                            avg_period_diff <= ($signed(gps_total_cycles + (free_cnt - gps_phase_cnt_prev)) - $signed(dpps_total_cycles)) >>> 4;

                            // 仅在GPS连续稳定2个完整16s周期之后，才对外发布有效差值
                            if (gps_warmup_win_cnt >= 3'd1) begin
                                diff_valid_flag <= 1'b1;
                                dbg_diff_valid <= 1'b1;
                            end else begin
                                gps_warmup_win_cnt <= gps_warmup_win_cnt + 3'd1;
                            end

                            // 下一窗口从当前GPS边沿重新作为0s起点
                            gps_cnt <= 4'd0;
                            pps_cnt_8 <= 4'd0;
                            gps_total_cycles <= 32'd0;
                            dpps_total_cycles <= 32'd0;
                            dpps_cnt <= 4'd0;
                        end else begin
                            gps_cnt <= gps_cnt + 4'd1;
                            pps_cnt_8 <= pps_cnt_8 + 4'd1;
                        end
                    end
                end else begin
                    // 尚未达到“连续3个GPS边沿”：不做窗口计算
                    gps_window_active <= 1'b0;
                    dpps_window_active <= 1'b0;
                    gps_cnt <= 4'd0;
                    pps_cnt_8 <= 4'd0;
                    gps_total_cycles <= 32'd0;
                    dpps_total_cycles <= 32'd0;
                    dpps_cnt <= 4'd0;
                    dpps_phase_cnt_prev <= 32'd0;
                end
            end
        end
    end
end

// 调试输出
assign dbg_gps_phase_sum = gps_window_cycles_dbg;
assign dbg_dpps_phase_sum = dpps_window_cycles_dbg;
assign dbg_dpps = dpps;
assign dbg_dpps_phase_cnt = dpps_phase_cnt;
assign dbg_dpps_pulse_cnt = dpps_pulse_cnt;
assign dbg_dpps_pulse_active = (dpps_pulse_cnt < DPPS_PULSE_WIDTH) ? 1'b1 : 1'b0;
assign dbg_dpps_cnt = dpps_cnt;
assign dbg_dpps_posedge = dpps_posedge;

endmodule

// 长周期计数与差值运算模块
// 模块功能：实现256秒累计周期的计数、差值计算和除法运算
// 设计作者：TraeAI
// 设计日期：2026-03-10
// 版本号：v1.0
//
// 输入端口：
// - clk：系统时钟（50MHz）
// - sys_rst_n：系统复位（低有效）
// - gps_pps：GPS标准PPS信号
// - osc_ready：晶振稳定标志
//
// 输出端口：
// - period_number：周期编号
// - diff_total：总差值（N_actual - N_nominal）
// - quotient：商（驯服后DPPS计数基准）
// - remainder：余数（需均匀补偿的偏差量）
// - diff_valid：差值有效标志
// - period_done：周期完成标志
//
// 设计说明：
// 1. 累计周期：256秒
// 2. 理想计数值：N_nominal = 50MHz * 256s = 12,800,000,000
// 3. 实际计数值：N_actual：256秒内实际计数
// 4. 差值：diff_total = N_actual - N_nominal
// 5. 除法：将差值拆分为商和余数
//
`timescale 1ns / 1ps

module long_period_counter (
    input           clk,
    input           sys_rst_n,
    input           gps_pps,
    input           osc_ready,
    
    output reg [15:0]  period_number,
    output reg signed [63:0] diff_total,
    output reg signed [63:0] quotient,
    output reg signed [63:0] remainder,
    output reg          diff_valid,
    output reg          period_done,
    
    // 调试输出信号
    output reg [2:0]   dbg_state,
    output reg [7:0]   dbg_pps_cnt,
    output reg         dbg_gps_pps_posedge,
    output reg         dbg_counting_active
);

    // 参数定义
    parameter [31:0] CLK_CYCLES_PER_SEC = 32'd50_000_000;
    // 原始参数：256秒计数
    // parameter [7:0]  ACCUM_PERIOD_SECS = 8'd256;
    // parameter signed [63:0] N_NOMINAL = 64'sd12_800_000_000; // 50MHz * 256s
    // 调试参数：16秒计数
    parameter [7:0]  ACCUM_PERIOD_SECS = 8'd16;
    parameter signed [63:0] N_NOMINAL = 64'sd800_000_000; // 50MHz * 16s
    
    // 内部信号
    reg [31:0]  sec_counter;
    reg [63:0]  cycle_counter;
    reg [7:0]   pps_count;
    reg         counting_active;
    
    // GPS PPS同步
    reg [2:0]   gps_pps_sync;
    reg         gps_pps_posedge;
    
    // 状态机定义
    localparam [2:0]
        ST_IDLE = 3'd0,
        ST_WAIT_OSC = 3'd1,
        ST_COUNTING = 3'd2,
        ST_CALC_DIFF = 3'd3,
        ST_DIVIDE = 3'd4,
        ST_DONE = 3'd5;
    
    reg [2:0] state;
    
    // 除法运算内部信号
    reg signed [63:0] div_numerator;
    reg signed [63:0] div_denominator;
    reg [7:0] div_step;
    reg signed [63:0] div_quotient;
    reg signed [63:0] div_remainder;

    // GPS PPS同步与边沿检测
    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            gps_pps_sync <= 3'b000;
            gps_pps_posedge <= 1'b0;
        end else begin
            gps_pps_sync <= {gps_pps_sync[1:0], gps_pps};
            gps_pps_posedge <= (gps_pps_sync[2:1] == 2'b01);
        end
    end

    // 主状态机
    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            state <= ST_IDLE;
            period_number <= 16'd0;
            diff_total <= 64'sd0;
            quotient <= 64'sd0;
            remainder <= 64'sd0;
            diff_valid <= 1'b0;
            period_done <= 1'b0;
            sec_counter <= 32'd0;
            cycle_counter <= 64'd0;
            pps_count <= 8'd0;
            counting_active <= 1'b0;
            div_numerator <= 64'sd0;
            div_denominator <= 64'sd0;
            div_step <= 8'd0;
            div_quotient <= 64'sd0;
            div_remainder <= 64'sd0;
            // 初始化调试信号
            dbg_state <= ST_IDLE;
            dbg_pps_cnt <= 8'd0;
            dbg_gps_pps_posedge <= 1'b0;
            dbg_counting_active <= 1'b0;
        end else begin
            diff_valid <= 1'b0;
            period_done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (osc_ready) begin
                        state <= ST_WAIT_OSC;
                    end
                end

                ST_WAIT_OSC: begin
                    // 等待第一个GPS PPS开始计数
                    if (gps_pps_posedge) begin
                        state <= ST_COUNTING;
                        cycle_counter <= 64'd0;
                        pps_count <= 8'd0;
                        counting_active <= 1'b1;
                    end
                end

                ST_COUNTING: begin
                    // 计数256个PPS周期
                    cycle_counter <= cycle_counter + 64'd1;
                    
                    if (gps_pps_posedge) begin
                        // 检查是否达到计数上限
                        // 原始逻辑：if (pps_count >= 256 - 1) 即 255
                        // 调试逻辑：if (pps_count >= 16 - 1) 即 15
                        if (pps_count >= ACCUM_PERIOD_SECS - 8'd1) begin
                            state <= ST_CALC_DIFF;
                            counting_active <= 1'b0;
                        end
                        
                        // 更新计数
                        pps_count <= pps_count + 8'd1;
                    end
                end

                ST_CALC_DIFF: begin
                    // 计算总差值
                    diff_total <= $signed(cycle_counter) - N_NOMINAL;
                    period_number <= period_number + 16'd1;
                    state <= ST_DIVIDE;
                    
                    // 初始化除法
                    div_numerator <= $signed(cycle_counter) - N_NOMINAL;
                    // 原始除数：256（秒数）
                    // div_denominator <= 64'sd256;
                    // 调试除数：16（秒数）
                    div_denominator <= 64'sd16; // 除数为16（秒数）
                    div_step <= 8'd0;
                    div_quotient <= 64'sd0;
                    div_remainder <= 64'sd0;
                end

                ST_DIVIDE: begin
                    // 简单的除法运算（恢复余数法）
                    // 原始逻辑：除以256（右移8位）
                    // if (div_step == 8'd0) begin
                    //     div_quotient <= div_numerator >>> 8;
                    //     div_remainder <= div_numerator & 64'hFF;
                    // 调试逻辑：除以16（右移4位）
                    if (div_step == 8'd0) begin
                        div_quotient <= div_numerator >>> 4;
                        div_remainder <= div_numerator & 64'h0F;
                        div_step <= 8'd1;
                    end else begin
                        quotient <= div_quotient;
                        remainder <= div_remainder;
                        diff_valid <= 1'b1;
                        period_done <= 1'b1;
                        state <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    // 等待一个周期后重新开始
                    state <= ST_WAIT_OSC;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
            
            // 更新调试信号
            dbg_state <= state;
            dbg_pps_cnt <= pps_count;
            dbg_gps_pps_posedge <= gps_pps_posedge;
            dbg_counting_active <= counting_active;
        end
    end

endmodule

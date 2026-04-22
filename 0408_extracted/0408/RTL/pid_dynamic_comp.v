`timescale 1ns / 1ps

// PID动态补偿模块（独立实验模块）
// 说明：
// 1) 本模块不依赖现有顶层调用，可单独接入做A/B实验。
// 2) 输入误差建议使用“窗口差值”（如16s窗口后的diff），err_valid拉高1拍更新一次PID。
// 3) 输出comp_step按sec_tick每秒输出一次，内部带“余数均匀分布”机制，减少量化抖动。

module pid_dynamic_comp #(
    // 固定点小数位数（Q格式）
    parameter integer GAIN_Q = 10,

    // PID增益（Q10默认值，可后续调参）
    parameter signed [31:0] KP = 32'sd64,   // 64 / 1024 = 0.0625
    parameter signed [31:0] KI = 32'sd8,    // 8  / 1024 = 0.0078125
    parameter signed [31:0] KD = 32'sd16,   // 16 / 1024 = 0.015625

    // 积分项限幅（Q10域）
    parameter signed [47:0] I_MAX_Q = 48'sd4_194_304,
    parameter signed [47:0] I_MIN_Q = -48'sd4_194_304,

    // 输出步进限幅（整数步进域）
    parameter signed [15:0] STEP_MAX = 16'sd300,
    parameter signed [15:0] STEP_MIN = -16'sd300
) (
    input  wire              clk,
    input  wire              rst_n,

    // 误差输入（建议来自窗口统计后的差值）
    input  wire              err_valid,
    input  wire signed [31:0] err_value,

    // 每秒执行补偿
    input  wire              sec_tick,

    // 输出：每秒补偿步进（已包含余数均匀分布）
    output reg               comp_valid,
    output reg signed [15:0] comp_step,

    // 调试输出
    output reg signed [31:0] u_int_q,      // 当前控制量（Q格式整数部分）
    output reg signed [31:0] p_term_dbg,
    output reg signed [31:0] i_term_dbg,
    output reg signed [31:0] d_term_dbg,
    output reg signed [31:0] rem_acc_dbg
);

    localparam signed [31:0] SCALE = (32'sd1 <<< GAIN_Q);

    // 误差历史
    reg signed [31:0] e_k;
    reg signed [31:0] e_k_1;
    reg signed [31:0] e_k_2;

    // PID内部（Q格式）
    reg signed [47:0] i_acc_q;
    reg signed [47:0] u_q;

    // 余数均匀分布累加器（Q格式）
    reg signed [31:0] rem_acc;

    // 组合中间量
    reg signed [47:0] p_term_q;
    reg signed [47:0] d_term_q;
    reg signed [47:0] i_next_q;
    reg signed [47:0] u_next_q;

    reg signed [31:0] base_step;
    reg signed [31:0] frac_part;
    reg signed [31:0] step_next;
    reg signed [31:0] rem_next;

    // =====================================================================
    // PID更新：仅在err_valid时更新（例如每16秒一次）
    // =====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e_k       <= 32'sd0;
            e_k_1     <= 32'sd0;
            e_k_2     <= 32'sd0;
            i_acc_q   <= 48'sd0;
            u_q       <= 48'sd0;
            p_term_q  <= 48'sd0;
            d_term_q  <= 48'sd0;
            i_next_q  <= 48'sd0;
            u_next_q  <= 48'sd0;
        end else if (err_valid) begin
            e_k   <= err_value;
            e_k_2 <= e_k_1;
            e_k_1 <= e_k;

            // P项：Kp * (e(k)-e(k-1))
            p_term_q <= KP * (err_value - e_k);

            // D项：Kd * (e(k)-2e(k-1)+e(k-2))
            d_term_q <= KD * (err_value - (e_k <<< 1) + e_k_1);

            // I项：积分累加 + 限幅
            i_next_q <= i_acc_q + KI * err_value;
            if (i_next_q > I_MAX_Q)
                i_acc_q <= I_MAX_Q;
            else if (i_next_q < I_MIN_Q)
                i_acc_q <= I_MIN_Q;
            else
                i_acc_q <= i_next_q;

            // 总控制量（Q格式）
            u_next_q <= i_acc_q + p_term_q + d_term_q;
            u_q <= u_next_q;
        end
    end

    // =====================================================================
    // 每秒输出步进：整数部分 + 余数均匀分布
    // =====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            comp_valid   <= 1'b0;
            comp_step    <= 16'sd0;
            rem_acc      <= 32'sd0;
            base_step    <= 32'sd0;
            frac_part    <= 32'sd0;
            step_next    <= 32'sd0;
            rem_next     <= 32'sd0;
            u_int_q      <= 32'sd0;
            p_term_dbg   <= 32'sd0;
            i_term_dbg   <= 32'sd0;
            d_term_dbg   <= 32'sd0;
            rem_acc_dbg  <= 32'sd0;
        end else begin
            comp_valid <= 1'b0;

            // 调试镜像
            p_term_dbg  <= p_term_q[31:0];
            i_term_dbg  <= i_acc_q[31:0];
            d_term_dbg  <= d_term_q[31:0];
            u_int_q     <= u_q[31:0];
            rem_acc_dbg <= rem_acc;

            if (sec_tick) begin
                comp_valid <= 1'b1;

                // base_step = floor(u_q / 2^Q)
                base_step <= u_q >>> GAIN_Q;

                // frac_part = u_q - base_step*2^Q （带符号）
                frac_part <= u_q[31:0] - ((u_q >>> GAIN_Q) <<< GAIN_Q);

                // 余数累加并均匀分布
                rem_next  <= rem_acc + frac_part;
                step_next <= base_step;

                if (rem_next >= SCALE) begin
                    rem_next  <= rem_next - SCALE;
                    step_next <= base_step + 32'sd1;
                end else if (rem_next <= -SCALE) begin
                    rem_next  <= rem_next + SCALE;
                    step_next <= base_step - 32'sd1;
                end

                rem_acc <= rem_next;

                // 输出限幅
                if (step_next > STEP_MAX)
                    comp_step <= STEP_MAX;
                else if (step_next < STEP_MIN)
                    comp_step <= STEP_MIN;
                else
                    comp_step <= step_next[15:0];
            end
        end
    end

endmodule

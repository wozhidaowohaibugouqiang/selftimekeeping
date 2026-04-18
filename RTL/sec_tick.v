`timescale 1ns / 1ps

// 模块名称：clk_1hz_gen
// 文件路径：e:\fjlwork\selftimekeep\RTL\sec_tick.v
// 功能说明：1Hz秒脉冲生成模块
//          输入10MHz温漂晶振时钟，输出1Hz的单周期脉冲信号(sec_tick)
//          专用于驱动 avg_remain_comp 模块的 sec_tick 输入接口
//
// 设计要求：
//          - 输入时钟：10MHz (温漂晶振)
//          - 输出脉冲：1Hz，高电平持续1个时钟周期 (100ns)
//          - 计数器位宽：32位 (防止溢出)
//          - 复位逻辑：低电平异步复位同步释放 (此处采用标准异步复位写法)

module clk_1hz_gen (
    input  wire clk,      // 10MHz系统时钟输入
    input  wire rst_n,    // 系统复位，低有效
    output reg  sec_tick  // 1Hz秒脉冲输出（单周期高电平）
);

    // ========================================================================
    // 参数定义
    // ========================================================================
    // 系统时钟频率：50MHz (修改原因：系统时钟升级为50MHz)
    // parameter integer CLK_FREQ = 10_000_000; // 原10MHz配置
    // parameter integer CLK_FREQ = 25_000_000; // 原25MHz配置
    parameter integer CLK_FREQ = 50_000_000;
    
    // 分频计数器最大值计算过程：
    // 周期 T = 1s
    // 计数值 = (T * Fclk) - 1 = (1 * 25,000,000) - 1 = 24,999,999
    // 计数范围：0 ~ 24,999,999
    localparam integer CNT_MAX = CLK_FREQ - 1;

    // ========================================================================
    // 内部信号
    // ========================================================================
    // 32位计数器，足以覆盖 10,000,000 (需要24位，32位有充足余量)
    reg [31:0] cnt;

    // ========================================================================
    // 逻辑实现
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 32'd0;
            sec_tick <= 1'b0;
        end else begin
            if (cnt >= CNT_MAX) begin
                cnt <= 32'd0;       // 计数器归零
                sec_tick <= 1'b1;   // 生成1个时钟周期的脉冲
            end else begin
                cnt <= cnt + 1'b1;  // 计数器累加
                sec_tick <= 1'b0;   // 保持低电平
            end
        end
    end

endmodule

/*
// ============================================================================
// 模块例化示例 (对接 avg_remain_comp)
// ============================================================================
// 该注释块展示了如何将本模块与 avg_remain_comp 模块进行连接

// 1. 定义连接信号
wire sec_tick_signal; // 用于连接 clk_1hz_gen 输出和 avg_remain_comp 输入

// 2. 实例化 1Hz 生成模块
clk_1hz_gen u_clk_1hz_gen (
    .clk      (sys_clk_10mhz),    // 连接 10MHz 温漂晶振时钟
    .rst_n    (sys_rst_n),        // 连接系统复位
    .sec_tick (sec_tick_signal)   // 输出 1Hz 秒脉冲
);

// 3. 实例化 均匀补偿模块 (avg_remain_comp)
avg_remain_comp u_avg_remain_comp (
    .clk               (sys_clk_10mhz),
    .rst_n             (sys_rst_n),
    .diff_update_valid (diff_valid),     // 差值有效信号
    .diff_sign         (diff_sign),      // 差值符号
    .diff_mag          (diff_mag),       // 差值大小
    .sec_tick          (sec_tick_signal),// *** 连接 1Hz 秒脉冲 ***
    .comp_valid        (comp_valid),     // 补偿有效输出
    .comp_step         (comp_step)       // 补偿步进输出
);
// ============================================================================
*/

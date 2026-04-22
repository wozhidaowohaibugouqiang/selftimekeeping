// 自主守时核心模块Testbench（多场景测试）
// 测试功能：验证timekeeping_core在三种不同场景下的功能
// 设计作者：TraeAI
// 设计日期：2026-01-19
// 版本号：v1.0
//
// 测试场景：
// 1. 场景1（0~2秒）：DPPS与GPS PPS完全对齐（offset=0）
// 2. 场景2（2~4秒）：DPPS超前GPS PPS 200ns（offset=2，单位100ns）
// 3. 场景3（4~6秒）：DPPS滞后GPS PPS 300ns（offset=3，单位100ns）
//
// 设计说明：
// 1. 系统时钟频率：10MHz
// 2. GPS PPS周期：1秒
// 3. 自动切换偏移量，验证三种场景
// 4. 包含详细的信号打印信息
`timescale 1ns/1ns
module tb_timekeeping_core_all_scene();

// 信号定义
reg             sys_clk_10mhz;
reg             sys_rst_n;
reg             gps_pps;
reg [23:0]      dpps_offset;  // 动态偏移量，TB控制
wire            dpps;
wire            diff_sign;
wire [6:0]      diff_data;
wire            diff_valid_flag;
wire [7:0]      init_diff;
wire            init_diff_valid;

// 打印参数，便于观察仿真过程
initial begin
    $monitor("Time: %t, Offset: %d, DPPS: %b, Diff_Sign: %b, Diff_Data: %d, Valid: %b", 
             $time, dpps_offset, dpps, diff_sign, diff_data, diff_valid_flag);
end

// 1. 生成10MHz时钟（周期100ns）
initial begin
    sys_clk_10mhz = 1'b0;
    forever #50 sys_clk_10mhz = ~sys_clk_10mhz;
end

// 2. 生成复位信号
initial begin
    sys_rst_n = 1'b0;
    #200;
    sys_rst_n = 1'b1;
end

// 3. 生成标准GPS PPS（1秒1个脉冲）
initial begin
    gps_pps = 1'b0;
    #200;
    forever begin
        #1000000000;  // 1秒间隔
        gps_pps = 1'b1;
        #100;         // 脉冲宽度100ns
        gps_pps = 1'b0;
    end
end

// 4. 核心：按时间分段自动切换偏移量（三场景）
initial begin
    dpps_offset = 24'd0;  // 初始值：对齐
    // 场景1：0~2秒 → DPPS与GPS PPS 完全对齐（offset=0）
    #2000000000; 
    // 场景2：2~4秒 → DPPS超前GPS PPS 200ns（offset=2，单位100ns）
    dpps_offset = 24'd2;  
    #2000000000;
    // 场景3：4~6秒 → DPPS滞后GPS PPS 300ns（offset=3，单位100ns）
    dpps_offset = 24'd3;  
    #2000000000;
    // 仿真结束
    $stop;
end

// 例化设计模块
assign init_diff = 8'h00;
assign init_diff_valid = 1'b0;

timekeeping_core u_timekeeping_core
(
    .sys_clk_10mhz    (sys_clk_10mhz),
    .sys_rst_n        (sys_rst_n),
    .gps_pps          (gps_pps),
    .dpps_offset      (dpps_offset),
    .init_diff        (init_diff),
    .init_diff_valid  (init_diff_valid),
    .dpps             (dpps),
    .diff_sign        (diff_sign),
    .diff_data        (diff_data),
    .diff_valid_flag  (diff_valid_flag)
);

endmodule

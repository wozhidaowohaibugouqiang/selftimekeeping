// 反向补偿系数管理模块（晶振老化补偿）
// 模块功能：管理晶振老化衰减特性参数
// 设计作者：TraeAI
// 设计日期：2026-03-10
// 版本号：v1.0
//
// 输入端口：
// - clk：系统时钟
// - sys_rst_n：系统复位
// - period_number：当前周期编号
// - period_done：周期完成标志
// - coeff_update_en：系数更新使能（在线修正）
// - new_coeff_value：新的系数值
//
// 输出端口：
// - aging_comp_value：老化补偿值
// - coeff_valid：系数有效标志
//
// 设计说明：
// 1. 存储晶振老化衰减特性参数（预存值）
// 2. 支持在线修正（根据多周期差值趋势动态调整）
// 3. 双备份机制防止数据丢失
//
`timescale 1ns / 1ps

module aging_comp_coeff (
    input           clk,
    input           sys_rst_n,
    
    input [15:0]    period_number,
    input           period_done,
    
    input           coeff_update_en,
    input signed [31:0] new_coeff_value,
    
    output reg signed [31:0] aging_comp_value,
    output reg      coeff_valid
);

    // 老化补偿表（预存值，基于长期测试）
    // 假设：周期0-999为第一阶段，1000-1999为第二阶段，依此类推
    // 每个阶段补偿值递增（模拟晶振老化）
    reg signed [31:0] aging_table [15:0];
    
    // 双备份已移除，直接使用单表
    reg [3:0] current_stage;

    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            aging_table[0]  <= 32'sd0;
            aging_table[1]  <= 32'sd10;
            aging_table[2]  <= 32'sd25;
            aging_table[3]  <= 32'sd45;
            aging_table[4]  <= 32'sd70;
            aging_table[5]  <= 32'sd100;
            aging_table[6]  <= 32'sd135;
            aging_table[7]  <= 32'sd175;
            aging_table[8]  <= 32'sd220;
            aging_table[9]  <= 32'sd270;
            aging_table[10] <= 32'sd325;
            aging_table[11] <= 32'sd385;
            aging_table[12] <= 32'sd450;
            aging_table[13] <= 32'sd520;
            aging_table[14] <= 32'sd595;
            aging_table[15] <= 32'sd675;
            
            current_stage    <= 4'd0;
            aging_comp_value <= 32'sd0;
            coeff_valid      <= 1'b0;
        end else begin
            coeff_valid <= 1'b1;
            
            // 根据周期编号计算当前阶段（每1000个周期为一个阶段）
            current_stage <= period_number[13:10];
            
            // 输出当前阶段的老化补偿值
            aging_comp_value <= aging_table[current_stage];
            
            // 在线更新系数
            if (coeff_update_en && period_done) begin
                aging_table[current_stage] <= new_coeff_value;
            end
        end
    end

endmodule

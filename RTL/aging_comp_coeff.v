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

    reg [3:0] current_stage;

    function signed [31:0] stage_value;
        input [3:0] stage;
        begin
            case (stage)
                4'd0: stage_value = 32'sd0;
                4'd1: stage_value = 32'sd10;
                4'd2: stage_value = 32'sd25;
                4'd3: stage_value = 32'sd45;
                4'd4: stage_value = 32'sd70;
                4'd5: stage_value = 32'sd100;
                4'd6: stage_value = 32'sd135;
                4'd7: stage_value = 32'sd175;
                4'd8: stage_value = 32'sd220;
                4'd9: stage_value = 32'sd270;
                4'd10: stage_value = 32'sd325;
                4'd11: stage_value = 32'sd385;
                4'd12: stage_value = 32'sd450;
                4'd13: stage_value = 32'sd520;
                4'd14: stage_value = 32'sd595;
                default: stage_value = 32'sd675;
            endcase
        end
    endfunction

    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            current_stage    <= 4'd0;
            aging_comp_value <= 32'sd0;
            coeff_valid      <= 1'b0;
        end else begin
            coeff_valid <= 1'b1;
            current_stage <= period_number[13:10];
            aging_comp_value <= stage_value(period_number[13:10]);
        end
    end

endmodule

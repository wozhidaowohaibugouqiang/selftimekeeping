// 晶振状态模块
// 模块功能：直接输出晶振稳定状态，不再需要延迟
// 设计作者：TraeAI
// 设计日期：2026-03-13
// 版本号：v1.2
//
// 输入端口：
// - clk：系统时钟（50MHz）
// - sys_rst_n：系统复位（低有效）
//
// 输出端口：
// - osc_ready：晶振稳定标志
// - stabilized_rst_n：稳定后的复位信号
//
// 设计说明：
// 由于使用普通晶振，不再需要等待晶振稳定的时间延迟
// 系统上电后直接进入稳定状态
//
`timescale 1ns / 1ps

module osc_stabilizer (
    input           clk,
    input           sys_rst_n,
    output reg      osc_ready,
    output reg      stabilized_rst_n
);

    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            osc_ready <= 1'b0;
            stabilized_rst_n <= 1'b0;
        end else begin
            // 直接进入稳定状态
            osc_ready <= 1'b1;
            stabilized_rst_n <= 1'b1;
        end
    end

endmodule

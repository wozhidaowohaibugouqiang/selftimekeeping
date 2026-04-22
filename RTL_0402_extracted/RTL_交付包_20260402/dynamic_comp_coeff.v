// 动态补偿系数生成与存储模块
// 模块功能：生成基于余数的均匀补偿序列并存储
// 设计作者：TraeAI
// 设计日期：2026-03-10
// 版本号：v1.0
//
// 输入端口：
// - clk：系统时钟
// - sys_rst_n：系统复位
// - remainder：余数（来自长周期计数模块）
// - remainder_valid：余数有效标志
// - coeff_read_addr：系数读取地址
// - coeff_read_en：系数读取使能
//
// 输出端口：
// - coeff_data_out：补偿系数数据输出
// - coeff_data_valid：补偿系数数据有效
// - coeff_ready：系数生成完成标志
//
// 设计说明：
// 1. 基于余数生成均匀分布的补偿序列
// 2. 例如：余数3对应256个位置中3个1，253个0
// 3. 序列存储在内部RAM中供快速读取
//
`timescale 1ns / 1ps

module dynamic_comp_coeff (
    input           clk,
    input           sys_rst_n,
    
    input signed [63:0] remainder,
    input           remainder_valid,
    
    input [7:0]     coeff_read_addr,
    input           coeff_read_en,
    
    output reg      coeff_data_out,
    output reg      coeff_data_valid,
    output reg      coeff_ready
);

    // 参数定义
    localparam [7:0] SEQ_LENGTH = 8'd256;
    
    // 内部信号
    reg [7:0] seq_mem [255:0];  // 256字节的序列存储器
    reg [7:0] write_addr;
    reg [7:0] ones_count;
    reg [7:0] ones_placed;
    reg signed [63:0] remainder_reg;
    reg [8:0] step;
    
    // 状态机定义
    localparam [2:0]
        ST_IDLE = 3'd0,
        ST_INIT = 3'd1,
        ST_PLACE_ONES = 3'd2,
        ST_DONE = 3'd3;
    
    reg [2:0] state;

    always @(posedge clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            state <= ST_IDLE;
            coeff_data_out <= 1'b0;
            coeff_data_valid <= 1'b0;
            coeff_ready <= 1'b0;
            write_addr <= 8'd0;
            ones_count <= 8'd0;
            ones_placed <= 8'd0;
            remainder_reg <= 64'sd0;
            step <= 9'd0;
            
            // 初始化存储器为0
            for (integer i = 0; i < 256; i = i + 1) begin
                seq_mem[i] <= 8'd0;
            end
        end else begin
            coeff_data_valid <= 1'b0;
            
            // 读取逻辑
            if (coeff_read_en) begin
                coeff_data_out <= seq_mem[coeff_read_addr][0];
                coeff_data_valid <= 1'b1;
            end

            case (state)
                ST_IDLE: begin
                    coeff_ready <= 1'b0;
                    if (remainder_valid) begin
                        remainder_reg <= remainder;
                        state <= ST_INIT;
                    end
                end

                ST_INIT: begin
                    // 初始化
                    ones_count <= (remainder_reg >= 0) ? remainder_reg[7:0] : -remainder_reg[7:0];
                    ones_placed <= 8'd0;
                    write_addr <= 8'd0;
                    step <= 9'd0;
                    
                    // 清空存储器
                    for (integer i = 0; i < 256; i = i + 1) begin
                        seq_mem[i] <= 8'd0;
                    end
                    
                    state <= ST_PLACE_ONES;
                end

                ST_PLACE_ONES: begin
                    // 使用 Bresenham 算法均匀放置1
                    if (ones_placed < ones_count && write_addr < SEQ_LENGTH) begin
                        step <= step + ones_count;
                        
                        if (step >= SEQ_LENGTH) begin
                            step <= step - SEQ_LENGTH;
                            seq_mem[write_addr] <= 8'd1;
                            ones_placed <= ones_placed + 8'd1;
                        end
                        
                        write_addr <= write_addr + 8'd1;
                    end else begin
                        state <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    coeff_ready <= 1'b1;
                    // 等待新的余数
                    if (!remainder_valid) begin
                        state <= ST_IDLE;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

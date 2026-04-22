// 参数管理模块
// 模块功能：实现动态补偿参数的分类、版本管理和地址映射
// 设计作者：TraeAI
// 设计日期：2026-03-10
// 版本号：v1.0
//
// 输入端口：
// - sys_clk：系统时钟（50MHz）
// - sys_rst_n：系统复位（低有效）
// - diff_sign：差值符号位
// - diff_data：差值数值（7位）
// - diff_valid_flag：差值有效标志
// - gps_valid：GPS信号有效标志
//
// 输出端口：
// - param_type：参数类型（0=单次采样，1=温度段，2=电压段，3=综合）
// - param_version：参数版本号（16位）
// - param_timestamp：参数时间戳（32位秒数）
// - param_data：参数数据（8位）
// - param_valid：参数数据有效标志
// - param_addr：参数存储地址（24位）
// - addr_table_valid：地址映射表有效标志
//
// 设计说明：
// 1. 参数数据结构设计：
//    - 元数据区（前16字节）：
//      - 0x00: 魔数（0xAA）
//      - 0x01: 参数类型
//      - 0x02-0x03: 版本号（大端）
//      - 0x04-0x07: 时间戳（大端，秒数）
//      - 0x08-0x09: 数据点数
//      - 0x0A-0x0B: CRC16校验
//      - 0x0C-0x0F: 保留
//    - 数据区：实际参数数据
//
// 2. 参数分类：
//    - TYPE_SINGLE: 单次采样数据
//    - TYPE_TEMP: 按温度段分类
//    - TYPE_VOLT: 按电压段分类
//    - TYPE_COMBINED: 综合参数
//
// 3. 地址映射表：
//    - 固定在Flash起始区域（0x000000-0x000FFF）
//    - 每个条目64字节，支持最多64个参数集
//
`timescale 1ns / 1ps

module param_manager (
    input           sys_clk,
    input           sys_rst_n,
    
    // 差值输入（来自timekeeping_core）
    input  signed [31:0] diff_value,
    input                diff_valid_flag,
    input                gps_valid,
    
    // 参数输出（给Flash控制器）
    output reg [1:0]    param_type,
    output reg [15:0]   param_version,
    output reg [31:0]   param_timestamp,
    output reg [7:0]    param_data,
    output reg           param_valid,
    output reg [23:0]   param_addr,
    output reg           addr_table_valid,
    
    // 状态输出
    output reg [2:0]     param_state
);

    // ========================================================================
    // 参数类型定义
    // ========================================================================
    localparam [1:0]
        TYPE_SINGLE   = 2'b00,
        TYPE_TEMP     = 2'b01,
        TYPE_VOLT     = 2'b10,
        TYPE_COMBINED = 2'b11;

    // ========================================================================
    // Flash地址映射定义
    // ========================================================================
    localparam [23:0]
        ADDR_TABLE_BASE    = 24'h000000,    // 地址映射表基地址
        ADDR_TABLE_SIZE    = 24'h001000,    // 地址映射表大小（4KB）
        PARAM_DATA_BASE    = 24'h010000,    // 参数数据区基地址（与回放区隔离）
        MAGIC_NUMBER       = 8'hAA;           // 魔数

    // ========================================================================
    // 状态机定义
    // ========================================================================
    localparam [2:0]
        ST_IDLE         = 3'd0,
        ST_COLLECT      = 3'd1,
        ST_BUILD_META   = 3'd2,
        ST_WRITE_PARAM  = 3'd3,
        ST_UPDATE_TABLE = 3'd4,
        ST_DONE         = 3'd5;

    reg [2:0] state;

    // ========================================================================
    // 内部信号定义
    // ========================================================================
    reg [15:0]  current_version;      // 当前参数版本
    reg [31:0]  current_timestamp;    // 当前时间戳
    reg [15:0]  data_count;           // 数据点数计数
    reg [31:0]  second_counter;       // 秒计数器
    reg         sec_tick;             // 秒脉冲
    reg [23:0]  next_param_addr;      // 下一个参数存储地址
    reg [7:0]   meta_buffer[15:0];    // 元数据缓冲区
    reg [3:0]   meta_index;           // 元数据索引
    reg signed [31:0] latest_diff_value; // 本轮采样的老化差值
    reg [15:0]  crc16_value;          // CRC16校验值

    // ========================================================================
    // 秒计数器（生成时间戳）
    // ========================================================================
    localparam [24:0] SEC_COUNT_MAX = 25'd49999999; // 50MHz时钟，1秒
    reg [24:0] sec_counter_reg;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            sec_counter_reg <= 25'd0;
            sec_tick <= 1'b0;
            second_counter <= 32'd0;
        end else begin
            if (sec_counter_reg >= SEC_COUNT_MAX) begin
                sec_counter_reg <= 25'd0;
                sec_tick <= 1'b1;
                second_counter <= second_counter + 32'd1;
            end else begin
                sec_counter_reg <= sec_counter_reg + 25'd1;
                sec_tick <= 1'b0;
            end
        end
    end

    // ========================================================================
    // CRC16计算函数（简化版）
    // ========================================================================
    function [15:0] crc16_update;
        input [15:0] crc;
        input [7:0] data;
        integer i;
        reg [15:0] c;
        begin
            c = crc ^ {8'h00, data};
            for (i = 0; i < 8; i = i + 1) begin
                if (c[0])
                    c = (c >> 1) ^ 16'hA001;
                else
                    c = c >> 1;
            end
            crc16_update = c;
        end
    endfunction

    // ========================================================================
    // 主状态机
    // ========================================================================
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            state <= ST_IDLE;
            param_type <= TYPE_SINGLE;
            param_version <= 16'd0;
            param_timestamp <= 32'd0;
            param_data <= 8'd0;
            param_valid <= 1'b0;
            param_addr <= PARAM_DATA_BASE;
            addr_table_valid <= 1'b0;
            param_state <= ST_IDLE;
            current_version <= 16'd0;
            current_timestamp <= 32'd0;
            data_count <= 16'd0;
            next_param_addr <= PARAM_DATA_BASE;
            meta_index <= 4'd0;
            latest_diff_value <= 32'sd0;
            crc16_value <= 16'd0;
        end else begin
            param_valid <= 1'b0;
            param_state <= state;

            case (state)
                ST_IDLE: begin
                    // 仅在GPS有效时采集“稳定后的老化差值样本”
                    if (gps_valid && diff_valid_flag) begin
                        latest_diff_value <= diff_value;
                        data_count <= 16'd1;
                        crc16_value <= crc16_update(crc16_update(crc16_update(crc16_update(16'd0, diff_value[31:24]), diff_value[23:16]), diff_value[15:8]), diff_value[7:0]);
                        state <= ST_BUILD_META;
                        current_version <= current_version + 16'd1;
                        current_timestamp <= second_counter;
                        meta_index <= 4'd0;
                    end
                end

                ST_COLLECT: begin
                    // 保留状态定义，当前版本不使用批量采集
                    state <= ST_IDLE;
                end

                ST_BUILD_META: begin
                    // 构建元数据
                    case (meta_index)
                        4'd0:  meta_buffer[0] <= MAGIC_NUMBER;
                        4'd1:  meta_buffer[1] <= {6'd0, param_type};
                        4'd2:  meta_buffer[2] <= current_version[15:8];
                        4'd3:  meta_buffer[3] <= current_version[7:0];
                        4'd4:  meta_buffer[4] <= current_timestamp[31:24];
                        4'd5:  meta_buffer[5] <= current_timestamp[23:16];
                        4'd6:  meta_buffer[6] <= current_timestamp[15:8];
                        4'd7:  meta_buffer[7] <= current_timestamp[7:0];
                        4'd8:  meta_buffer[8] <= data_count[15:8];
                        4'd9:  meta_buffer[9] <= data_count[7:0];
                        4'd10: meta_buffer[10] <= crc16_value[15:8];
                        4'd11: meta_buffer[11] <= crc16_value[7:0];
                        4'd12: meta_buffer[12] <= latest_diff_value[31:24];
                        4'd13: meta_buffer[13] <= latest_diff_value[23:16];
                        4'd14: meta_buffer[14] <= latest_diff_value[15:8];
                        4'd15: meta_buffer[15] <= latest_diff_value[7:0];
                    endcase

                    if (meta_index < 4'd15) begin
                        meta_index <= meta_index + 4'd1;
                    end else begin
                        state <= ST_WRITE_PARAM;
                        param_addr <= next_param_addr;
                        meta_index <= 4'd0;
                    end
                end

                ST_WRITE_PARAM: begin
                    // 输出参数数据给Flash控制器
                    if (meta_index < 4'd16) begin
                        param_data <= meta_buffer[meta_index];
                        param_type <= TYPE_SINGLE;
                        param_version <= current_version;
                        param_timestamp <= current_timestamp;
                        param_valid <= 1'b1;
                        param_addr <= next_param_addr + meta_index;
                        meta_index <= meta_index + 4'd1;
                    end else begin
                        state <= ST_UPDATE_TABLE;
                    end
                end

                ST_UPDATE_TABLE: begin
                    // 更新地址映射表
                    addr_table_valid <= 1'b1;
                    state <= ST_DONE;
                end

                ST_DONE: begin
                    // 完成后立即回到空闲，等待下一次稳定差值采样
                    addr_table_valid <= 1'b0;
                    state <= ST_IDLE;
                    next_param_addr <= next_param_addr + 24'd16;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

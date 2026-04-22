// ================================================================================
// Module: uart
// File: RTL/uart.v
// Description: UART communication module with command parsing and telemetry output
// Author: fjl
// Created: 2026-03-10
// Version: v2.0
// ================================================================================
// VERILOG STANDARD: This file uses SystemVerilog (IEEE 1800-2017) features.
// IMPORTANT: When editing this file, ensure:
//   - All port declarations inside module(...) end with semicolons (,)
//   - input/output declarations are inside the module port list
//   - Use 'input wire', 'output wire', 'output reg', 'input signed [N:0]' syntax
//   - function declarations use 'function [N:0] funcname;' not 'function funcname;'
// ================================================================================

`timescale 1ns / 1ps

module uart (
    input  wire        sys_clk,      // 50MHz系统时钟
    input  wire        sys_rst_n,    // 系统复位

    // UART Physical Interface
    input  wire        uart_rx,
    output wire        uart_tx,

    // System Status Inputs (for Reporting)
    input  wire        erase_start_pulse, // 擦除开始脉冲
    input  wire        erase_done_pulse,  // 擦除完成脉冲
    input  wire        read_start_pulse,  // 读取开始脉冲

    input  wire [3:0]  sys_state,
    input  wire        pps_valid,
    input  wire        fifo_empty,
    input  wire        fifo_full,
    input  wire [7:0]  fifo_dout,
    input  wire        train_done,
    // 调试输入信号
    input  wire [2:0]  dbg_state,
    input  wire [7:0]  dbg_pps_cnt,
    input  wire        period_diff_pulse,
    input  wire        dbg_gps_pps_posedge,
    input  wire        dbg_counting_active,
    input  wire [15:0] period_cnt,
    input  wire [15:0] fwe_cnt,
    input  wire [15:0] fre_cnt,
    input  wire [15:0] dv50_cnt,
    input  wire [15:0] dv25_cnt,
    input  wire        state_changed,
    input  wire        dbg_diff_valid,
    input  wire signed [31:0] diff_result,
    output wire        erase_cmd_pulse,
    input  wire signed [31:0] aging_diff_result,
    input  wire [31:0] dbg_gps_phase_sum,
    input  wire [31:0] dbg_dpps_phase_sum,
    input  wire [31:0] dbg_dpps_phase_cnt,
    input  wire [31:0] dbg_dpps_pulse_cnt,
    input  wire        dbg_dpps_pulse_active,
    input  wire [3:0]  dbg_dpps_cnt,
    input  wire        dbg_dpps_posedge,
    input  wire        fifo_flush_req,
    input  wire        fifo_we_debug,
    input  wire [1:0]  fifo_state_debug,
    input  wire        flash_busy,
    input  wire        flash_done,
    input  wire        write_done,
    input  wire        flash_error,
    input  wire [7:0]  dbg_rd_first_byte,
    input  wire        dbg_rd_first_valid,
    input  wire [23:0] dbg_flash_cur_addr,
    input  wire [15:0] dbg_poll_attempts,
    input  wire [7:0]  dbg_last_sr,
    input  wire [2:0]  dbg_sr_stage,
    input  wire [7:0]  dbg_last_cmd,
    input  wire [3:0]  dbg_last_len,
    input  wire [7:0]  dbg_last_rx0,
    input  wire [7:0]  dbg_last_rx1,
    input  wire [3:0]  dbg_err_code,
    input  wire [23:0] dbg_jedec_id,
    input  wire        dbg_jedec_valid,
    input  wire [6:0]  dbg_flash_state,
    input  wire [7:0]  dbg_txn_done_cnt,
    input  wire [7:0]  hist_diff,
    input  wire        hist_diff_valid,
    input  wire signed [31:0] read_diff,
    input  wire        read_diff_valid,
    input  wire        flash_dyn_empty,
    input  wire [23:0] dpps_offset_dbg,
    input  wire signed [23:0] dyn_offset_dbg,
    input  wire signed [15:0] comp_step_dbg,
    input  wire        comp_valid_dbg,
    input  wire        dyn_ready_dbg,
    input  wire signed [31:0] comp_fixed_dbg,
    input  wire signed [31:0] dyn_q_dbg,
    input  wire signed [31:0] dyn_r_dbg,
    input  wire signed [31:0] dyn_ds_dbg,
    input  wire signed [31:0] dyn_ld_dbg,
    input  wire signed [31:0] dyn_rs_dbg,
    input  wire [23:0] dpps_offset_dbg_core,
    input  wire [31:0] dpps_period_target_dbg,
    input  wire [23:0] dump_read_start_addr,
    input  wire [31:0] dump_start_lw,
    input  wire [15:0] fifo_write_cnt,
    input  wire [7:0]  check_fifo_cnt,
    input  wire [7:0]  fifo_re_done_cnt,
    input  wire [7:0]  fifo_latch_done_cnt,
    input  wire [15:0] fifo_prog_cnt,
    input  wire        fifo_was_empty_at_dump,
    input  wire [1:0]  fifo_state_at_dump_start,
    input  wire [15:0] fifo_total_cnt_at_dump_start,
    input  wire [31:0] last_prog_word,
    // 调试信息：参数锁定状态
    input  wire        dyn_ready_in,        // dynamic_comp_ready 信号
    input  wire        comp_locked_in,     // comp_param_locked 信号
    input  wire [3:0]  comp_init_cnt_in,   // comp_init_cnt 信号
    input  wire        gps_stable_in,      // gps_stable 信号
    input  wire        gps_lost_in,        // gps_lost 信号
    input  wire        dump_active_in,      // dump_active 信号
    input  wire        dyn_rd_active_in,   // dyn_rd_active 信号
    input  wire        dyn_store_active_in, // dyn_store_active 信号
    input  wire        dyn_boot_check_pending_in, // boot阶段自动读请求挂起
    input  wire        dyn_rd_wait_in       // 动态参数读取等待Flash返回

    // Dump stream interface (from top)
    ,input  wire        dump_mode
    ,input  wire        dump_word_valid
    ,input  wire [23:0] dump_word_addr
    ,input  wire [31:0] dump_word_data
    ,input  wire        dump_done
    ,input  wire        dump_meta_valid
    ,input  wire [15:0] dump_meta_fwe
    ,input  wire [15:0] dump_meta_fre
    ,input  wire [15:0] dump_meta_limit
    ,input  wire [15:0] dump_meta_exported
    ,output wire        dump_word_ready
    ,output wire        dump_meta_ready
    ,output reg  [3:0]  state_cmd
    ,output reg         cmd_valid
    ,output wire        tx_en
    ,output wire        tx_done
    ,output reg         dyn_read_cmd_pulse   // 'D' 命令：读取动态补偿参数
    ,output reg         dyn_write_cmd_pulse   // 'W' 命令：写入当前动态补偿参数到Flash
);

    // ========================================================================
    // 1. Instantiate EF2M45 UART IP
    // ========================================================================
    wire       rx_vld;
    wire [7:0] rx_data;
    wire       tx_rdy;
    wire [7:0] tx_data;
    reg        uart_tx_en;
    wire       rx_err_unused;
    localparam UART_BIT_CLKS = 16'd432; // 50 MHz / 115200 baud, matching old UART_0 CLK_DIV_NUM=27 x16

    uart_rx_simple #(
        .BIT_CLKS(UART_BIT_CLKS)
    ) u_uart_rx_simple (
        .clk    (sys_clk),
        .rst_n  (sys_rst_n),
        .rxd    (uart_rx),
        .rx_vld (rx_vld),
        .rx_data(rx_data),
        .rx_err (rx_err_unused)
    );

    uart_tx_simple #(
        .BIT_CLKS(UART_BIT_CLKS)
    ) u_uart_tx_simple (
        .clk    (sys_clk),
        .rst_n  (sys_rst_n),
        .tx_en  (uart_tx_en),
        .tx_data(tx_data),
        .tx_rdy (tx_rdy),
        .txd    (uart_tx)
    );

    // ========================================================================
    // 2. RX Logic: Command Parsing & Echo FIFO
    // ========================================================================
    localparam MAX_CMD_LEN = 16;
    reg [7:0] cmd_buf [0:MAX_CMD_LEN-1];
    reg [3:0] buf_idx;

    reg rx_received_flag;
    reg df_latch;
    reg df_latch_next;

    wire signed [31:0] diff_disp = pps_valid ? diff_result : 32'sd0;
    wire [31:0] diff_abs = diff_disp[31] ? (-diff_disp) : diff_disp;
    wire [23:0] dyn_abs  = dyn_offset_dbg[23] ? (-dyn_offset_dbg) : dyn_offset_dbg;
    wire [15:0] comp_abs = comp_step_dbg[15] ? (-comp_step_dbg) : comp_step_dbg;
    wire signed [31:0] comp_fixed_disp = comp_fixed_dbg;
    wire [31:0] comp_fixed_abs = comp_fixed_disp[31] ? (-comp_fixed_disp) : comp_fixed_disp;
    wire [31:0] dyn_read_abs = dyn_read_value_latched[31] ? (-dyn_read_value_latched) : dyn_read_value_latched;
    wire [31:0] dyn_q_abs = dyn_q_dbg[31] ? (-dyn_q_dbg) : dyn_q_dbg;
    wire [31:0] dyn_r_abs = dyn_r_dbg[31] ? (-dyn_r_dbg) : dyn_r_dbg;
    wire [31:0] dyn_ds_abs = dyn_ds_dbg[31] ? (-dyn_ds_dbg) : dyn_ds_dbg;
    wire [31:0] dyn_ld_abs = dyn_ld_dbg[31] ? (-dyn_ld_dbg) : dyn_ld_dbg;
    wire [31:0] dyn_rs_abs = dyn_rs_dbg[31] ? (-dyn_rs_dbg) : dyn_rs_dbg;
    wire [23:0] doffs_abs = dpps_offset_dbg[23] ? (~dpps_offset_dbg + 24'd1) : dpps_offset_dbg;

    reg erase_cmd_reg;
    assign erase_cmd_pulse = erase_cmd_reg;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            buf_idx <= 4'd0;
            state_cmd <= 4'd0;
            cmd_valid <= 1'b0;
            rx_received_flag <= 1'b0;
            erase_cmd_reg <= 1'b0;
            dyn_read_cmd_pulse <= 1'b0;
            dyn_write_cmd_pulse <= 1'b0;
        end else begin
            cmd_valid <= 1'b0;
            erase_cmd_reg <= 1'b0;
            dyn_read_cmd_pulse <= 1'b0;
            dyn_write_cmd_pulse <= 1'b0;

            if (rx_vld) begin
                rx_received_flag <= 1'b1;
                if (buf_idx < MAX_CMD_LEN) begin
                    cmd_buf[buf_idx] <= rx_data;
                    buf_idx <= buf_idx + 4'd1;
                end
                if (rx_data == 8'h0A) begin
                    if (cmd_buf[0] == "C" && buf_idx >= 9) begin
                        state_cmd <= 4'b0001;
                        cmd_valid <= 1'b1;
                    end else if (cmd_buf[0] == "T" && buf_idx >= 10) begin
                        state_cmd <= 4'b0010;
                        cmd_valid <= 1'b1;
                    end else if (cmd_buf[0] == "E") begin
                        erase_cmd_reg <= 1'b1;
                        state_cmd <= 4'b0000;
                        cmd_valid <= 1'b1;
                    end else if (cmd_buf[0] == "S") begin
                        state_cmd <= 4'b0000;
                        cmd_valid <= 1'b1;
                    end else if (cmd_buf[0] == "D") begin
                        dyn_read_cmd_pulse <= 1'b1;
                        state_cmd <= 4'b0000;
                        cmd_valid <= 1'b1;
                    end else if (cmd_buf[0] == "W") begin
                        dyn_write_cmd_pulse <= 1'b1;
                        state_cmd <= 4'b0000;
                        cmd_valid <= 1'b1;
                    end
                    buf_idx <= 4'd0;
                end
            end
        end
    end

    // ========================================================================
    // 3. Heartbeat Timer
    // ========================================================================
    reg [2:0] tx_state;
    localparam ST_TX_IDLE = 3'd0, ST_TX_SEND = 3'd2;

    reg [27:0] hb_cnt;
    reg        hb_req;
    reg        reply_req;
    reg        rdv_latched;
    reg [15:0] rdv_cnt;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            hb_cnt <= 28'd0;
            hb_req <= 1'b0;
            reply_req <= 1'b0;
            df_latch <= 1'b0;
            df_latch_next <= 1'b0;
            rdv_latched <= 1'b0;
            rdv_cnt <= 16'd0;
        end else begin
            if (period_diff_pulse && !dump_mode)
                hb_req <= 1'b1;
            if (cmd_valid)
                reply_req <= 1'b1;
            if (read_diff_valid) begin
                rdv_latched <= 1'b1;
                rdv_cnt <= rdv_cnt + 16'd1;
            end
            df_latch_next = df_latch;
            if (dbg_diff_valid)
                df_latch_next = 1'b1;
            else if (tx_state == ST_TX_SEND && tx_msg_type == 4'd1 && uart_tx_en && tx_rdy && (tx_ptr == (tx_len - 8'd1)))
                df_latch_next = 1'b0;
            df_latch <= df_latch_next;
            if (tx_state == ST_TX_IDLE) begin
                if (reply_req)
                    reply_req <= 1'b0;
                else if (hb_req)
                    hb_req <= 1'b0;
            end
        end
    end

    // ========================================================================
    // 4. TX Logic: 直接寄存器发送（简单可靠）
    // ========================================================================

    reg [7:0] tx_ptr;
    reg [7:0] tx_len;

    // ASCII Helpers
    function [7:0] hex2ascii;
        input [3:0] hex;
        begin
            hex2ascii = (hex < 10) ? (hex + "0") : (hex - 10 + "A");
        end
    endfunction

    reg [1:0] fifo_status;
    always @(*) begin
        if (fifo_full) fifo_status = 2'b10;
        else if (fifo_empty) fifo_status = 2'b01;
        else fifo_status = 2'b00;
    end

    // flash_done 脉冲锁存
    reg flash_done_latch;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            flash_done_latch <= 1'b0;
        else begin
            if (flash_done)
                flash_done_latch <= 1'b1;
            else if (tx_state == ST_TX_SEND && tx_msg_type == 4'd1 && uart_tx_en && tx_rdy && (tx_ptr == (tx_len - 8'd1)))
                flash_done_latch <= 1'b0;
        end
    end

    // write_done 脉冲锁存
    reg write_done_latch;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            write_done_latch <= 1'b0;
        else begin
            if (write_done)
                write_done_latch <= 1'b1;
            else if (tx_state == ST_TX_SEND && tx_msg_type == 4'd1 && uart_tx_en && tx_rdy && (tx_ptr == (tx_len - 8'd1)))
                write_done_latch <= 1'b0;
        end
    end

    // 消息类型识别
    localparam [3:0] MSG_REPLY       = 4'd0;
    localparam [3:0] MSG_HEARTBEAT   = 4'd1;
    localparam [3:0] MSG_DUMP_WORD   = 4'd2;
    localparam [3:0] MSG_DUMP_DONE   = 4'd3;
    localparam [3:0] MSG_ERASE_START = 4'd5;
    localparam [3:0] MSG_ERASE_DONE  = 4'd6;
    localparam [3:0] MSG_READ_START  = 4'd7;
    localparam [3:0] MSG_DYN_OK      = 4'd8;
    localparam [3:0] MSG_DYN_EMPTY   = 4'd9;

    reg [3:0] tx_msg_type;
    reg [7:0] tx_data_mux;
    reg [23:0] dump_addr_latched;
    reg [31:0] dump_data_latched;
    reg        dump_done_sent;
    reg        dyn_ok_req;
    reg        dyn_empty_req;
    reg signed [31:0] dyn_read_value_latched;

    function [7:0] hex8_at;
        input [7:0] value;
        input [0:0] idx;
        begin
            hex8_at = (idx == 1'd0) ? hex2ascii(value[7:4]) : hex2ascii(value[3:0]);
        end
    endfunction

    function [7:0] hex16_at;
        input [15:0] value;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: hex16_at = hex2ascii(value[15:12]);
                2'd1: hex16_at = hex2ascii(value[11:8]);
                2'd2: hex16_at = hex2ascii(value[7:4]);
                default: hex16_at = hex2ascii(value[3:0]);
            endcase
        end
    endfunction

    function [7:0] hex24_at;
        input [23:0] value;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: hex24_at = hex2ascii(value[23:20]);
                3'd1: hex24_at = hex2ascii(value[19:16]);
                3'd2: hex24_at = hex2ascii(value[15:12]);
                3'd3: hex24_at = hex2ascii(value[11:8]);
                3'd4: hex24_at = hex2ascii(value[7:4]);
                default: hex24_at = hex2ascii(value[3:0]);
            endcase
        end
    endfunction

    function [7:0] hex32_at;
        input [31:0] value;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: hex32_at = hex2ascii(value[31:28]);
                3'd1: hex32_at = hex2ascii(value[27:24]);
                3'd2: hex32_at = hex2ascii(value[23:20]);
                3'd3: hex32_at = hex2ascii(value[19:16]);
                3'd4: hex32_at = hex2ascii(value[15:12]);
                3'd5: hex32_at = hex2ascii(value[11:8]);
                3'd6: hex32_at = hex2ascii(value[7:4]);
                default: hex32_at = hex2ascii(value[3:0]);
            endcase
        end
    endfunction

    function [7:0] tx_msg_len;
        input [3:0] msg;
        begin
            case (msg)
                MSG_REPLY:       tx_msg_len = 8'd9;
                MSG_HEARTBEAT:   tx_msg_len = 8'd53;
                MSG_DUMP_WORD:   tx_msg_len = 8'd27;
                MSG_DUMP_DONE:   tx_msg_len = 8'd3;
                MSG_ERASE_START: tx_msg_len = 8'd13;
                MSG_ERASE_DONE:  tx_msg_len = 8'd12;
                MSG_READ_START:  tx_msg_len = 8'd12;
                MSG_DYN_OK:      tx_msg_len = 8'd34;
                MSG_DYN_EMPTY:   tx_msg_len = 8'd11;
                default:         tx_msg_len = 8'd0;
            endcase
        end
    endfunction

    function [7:0] tx_msg_byte;
        input [3:0] msg;
        input [7:0] idx;
        reg [7:0] rdy_byte;
        begin
            tx_msg_byte = 8'h0A;
            // 计算 RDY 字节
            // bit[7]=dyn_boot_check_pending, bit[6]=dyn_rd_wait
            rdy_byte = {dyn_boot_check_pending_in,
                        dyn_rd_wait_in,
                        dyn_store_active_in,
                        dyn_rd_active_in,
                        dump_active_in,
                        gps_lost_in,
                        comp_locked_in,
                        gps_stable_in};
            case (msg)
                MSG_REPLY: begin
                    case (idx)
                        8'd0: tx_msg_byte = "S";
                        8'd1: tx_msg_byte = ":";
                        8'd2: tx_msg_byte = hex2ascii(sys_state);
                        8'd3: tx_msg_byte = ",";
                        8'd4: tx_msg_byte = "T";
                        8'd5: tx_msg_byte = ":";
                        8'd6: tx_msg_byte = train_done ? "1" : "0";
                        8'd7: tx_msg_byte = 8'h0D;
                        default: tx_msg_byte = 8'h0A;
                    endcase
                end

                MSG_HEARTBEAT: begin
                    // 格式: DIFF=XXXXXX,CA=XXXXXX,PT=XXXXXXXX,DO=XXXX,RDY=XX
                    // RDY 字段：bit[7]=dyn_boot_check_pending, bit[6]=dyn_rd_wait
                    // bit[5]=dyn_store_active, bit[4]=dyn_rd_active, bit[3]=dump_active
                    // bit[2]=gps_lost, bit[1]=comp_locked, bit[0]=gps_stable
                    // 新增: FP=fifo_prog_cnt(实际写入Flash字数), FB=flash_busy, FS=flash_state
                    case (idx)
                        // DIFF=+XXXXXX (12 bytes: D,I,F,F,=,+,-,6 hex)
                        8'd0:  tx_msg_byte = "D";    8'd1: tx_msg_byte = "I";    8'd2: tx_msg_byte = "F";    8'd3: tx_msg_byte = "F";    8'd4: tx_msg_byte = "=";
                        8'd5: tx_msg_byte = diff_disp[31] ? "-" : "+";
                        8'd12: tx_msg_byte = ",";
                        // CA=+XXXXXXXX (12 bytes: C,A,=,+,-,8 hex)
                        8'd13: tx_msg_byte = "C";    8'd14: tx_msg_byte = "A";    8'd15: tx_msg_byte = "=";
                        8'd16: tx_msg_byte = comp_fixed_disp[31] ? "-" : "+";
                        8'd25: tx_msg_byte = ",";
                        // PT=XXXXXXXX (10 bytes: P,T,=,+,-,6 hex)
                        8'd26: tx_msg_byte = "P";    8'd27: tx_msg_byte = "T";    8'd28: tx_msg_byte = "=";
                        8'd29: tx_msg_byte = dpps_period_target_dbg[31] ? "-" : "+";
                        8'd37: tx_msg_byte = ",";
                        // DO=XXXX (6 bytes: D,O,=,+,-,2 hex)
                        8'd38: tx_msg_byte = "D";    8'd39: tx_msg_byte = "O";    8'd40: tx_msg_byte = "=";
                        8'd41: tx_msg_byte = dpps_offset_dbg[23] ? "-" : "+";
                        8'd45: tx_msg_byte = ",";
                        // RDY=XX (6 bytes: R,D,Y,=,2 hex)
                        8'd46: tx_msg_byte = "R";    8'd47: tx_msg_byte = "D";    8'd48: tx_msg_byte = "Y";    8'd49: tx_msg_byte = "=";
                        8'd50: tx_msg_byte = hex2ascii(rdy_byte[7:4]);
                        8'd51: tx_msg_byte = hex2ascii(rdy_byte[3:0]);
                        8'd52: tx_msg_byte = 8'h0D;
                        default: tx_msg_byte = 8'h0A;
                    endcase
                    // DIFF abs: idx 6-11 (6 hex digits for 24-bit value)
                    if (idx >= 8'd6 && idx <= 8'd11)  tx_msg_byte = hex24_at(diff_abs[23:0], idx - 8'd6);
                    // CA abs: idx 17-24 (8 hex digits for 32-bit value)
                    if (idx >= 8'd17 && idx <= 8'd24)  tx_msg_byte = hex32_at(comp_fixed_abs, idx - 8'd17);
                    // PT: idx 30-36 (6 hex digits for 24-bit value)
                    if (idx >= 8'd30 && idx <= 8'd36)  tx_msg_byte = hex24_at(dpps_period_target_dbg[23:0], idx - 8'd30);
                    // DO abs: idx 42-44 (4 hex digits for 16-bit value)
                    if (idx >= 8'd42 && idx <= 8'd44) tx_msg_byte = hex16_at(doffs_abs[15:0], idx - 8'd42);
                end
                MSG_DUMP_WORD: begin
                    case (idx)
                        8'd0: tx_msg_byte = "A"; 8'd1: tx_msg_byte = "D"; 8'd2: tx_msg_byte = "D"; 8'd3: tx_msg_byte = "R"; 8'd4: tx_msg_byte = "=";
                        8'd11: tx_msg_byte = ",";
                        8'd12: tx_msg_byte = "D"; 8'd13: tx_msg_byte = "A"; 8'd14: tx_msg_byte = "T"; 8'd15: tx_msg_byte = "A"; 8'd16: tx_msg_byte = "=";
                        8'd25: tx_msg_byte = 8'h0D;
                        default: tx_msg_byte = 8'h0A;
                    endcase
                    if (idx >= 8'd5  && idx <= 8'd10) tx_msg_byte = hex24_at(dump_addr_latched, idx - 8'd5);
                    if (idx >= 8'd17 && idx <= 8'd24) tx_msg_byte = hex32_at(dump_data_latched, idx - 8'd17);
                end

                MSG_DUMP_DONE: begin
                    case (idx)
                        8'd0: tx_msg_byte = "E";
                        8'd1: tx_msg_byte = 8'h0D;
                        default: tx_msg_byte = 8'h0A;
                    endcase
                end

                MSG_ERASE_START: begin
                    case (idx)
                        8'd0: tx_msg_byte = "E"; 8'd1: tx_msg_byte = "R"; 8'd2: tx_msg_byte = "A"; 8'd3: tx_msg_byte = "S"; 8'd4: tx_msg_byte = "E"; 8'd5: tx_msg_byte = " ";
                        8'd6: tx_msg_byte = "S"; 8'd7: tx_msg_byte = "T"; 8'd8: tx_msg_byte = "A"; 8'd9: tx_msg_byte = "R"; 8'd10: tx_msg_byte = "T";
                        8'd11: tx_msg_byte = 8'h0D;
                        default: tx_msg_byte = 8'h0A;
                    endcase
                end

                MSG_ERASE_DONE: begin
                    case (idx)
                        8'd0: tx_msg_byte = "E"; 8'd1: tx_msg_byte = "R"; 8'd2: tx_msg_byte = "A"; 8'd3: tx_msg_byte = "S"; 8'd4: tx_msg_byte = "E"; 8'd5: tx_msg_byte = " ";
                        8'd6: tx_msg_byte = "D"; 8'd7: tx_msg_byte = "O"; 8'd8: tx_msg_byte = "N"; 8'd9: tx_msg_byte = "E";
                        8'd10: tx_msg_byte = 8'h0D;
                        default: tx_msg_byte = 8'h0A;
                    endcase
                end

                MSG_READ_START: begin
                    case (idx)
                        8'd0: tx_msg_byte = "R"; 8'd1: tx_msg_byte = "E"; 8'd2: tx_msg_byte = "A"; 8'd3: tx_msg_byte = "D"; 8'd4: tx_msg_byte = " ";
                        8'd5: tx_msg_byte = "S"; 8'd6: tx_msg_byte = "T"; 8'd7: tx_msg_byte = "A"; 8'd8: tx_msg_byte = "R"; 8'd9: tx_msg_byte = "T";
                        8'd10: tx_msg_byte = 8'h0D;
                        default: tx_msg_byte = 8'h0A;
                    endcase
                end

                MSG_DYN_OK: begin
                    case (idx)
                        8'd0: tx_msg_byte = "D"; 8'd1: tx_msg_byte = "Y"; 8'd2: tx_msg_byte = "N"; 8'd3: tx_msg_byte = ":";
                        8'd4: tx_msg_byte = "O"; 8'd5: tx_msg_byte = "K"; 8'd6: tx_msg_byte = ",";
                        8'd7: tx_msg_byte = "C"; 8'd8: tx_msg_byte = "A"; 8'd9: tx_msg_byte = ":";
                        8'd10: tx_msg_byte = dyn_read_value_latched[31] ? "-" : "+";
                        8'd19: tx_msg_byte = ",";
                        8'd20: tx_msg_byte = "L"; 8'd21: tx_msg_byte = "C"; 8'd22: tx_msg_byte = ":";
                        8'd23: tx_msg_byte = comp_fixed_disp[31] ? "-" : "+";
                        8'd32: tx_msg_byte = 8'h0D;
                        default: tx_msg_byte = 8'h0A;
                    endcase
                    if (idx >= 8'd11 && idx <= 8'd18) tx_msg_byte = hex32_at(dyn_read_abs, idx - 8'd11);
                    if (idx >= 8'd24 && idx <= 8'd31) tx_msg_byte = hex32_at(comp_fixed_abs, idx - 8'd24);
                end

                MSG_DYN_EMPTY: begin
                    case (idx)
                        8'd0: tx_msg_byte = "D"; 8'd1: tx_msg_byte = "Y"; 8'd2: tx_msg_byte = "N"; 8'd3: tx_msg_byte = ":";
                        8'd4: tx_msg_byte = "E"; 8'd5: tx_msg_byte = "M"; 8'd6: tx_msg_byte = "P"; 8'd7: tx_msg_byte = "T"; 8'd8: tx_msg_byte = "Y";
                        8'd9: tx_msg_byte = 8'h0D;
                        default: tx_msg_byte = 8'h0A;
                    endcase
                end

                default: tx_msg_byte = 8'h0A;
            endcase
        end
    endfunction

    always @(*) begin
        tx_data_mux = tx_msg_byte(tx_msg_type, tx_ptr);
    end

    // 按键/Flash事件锁存
    reg erase_start_req, erase_done_req, read_start_req;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            erase_start_req <= 1'b0;
            erase_done_req  <= 1'b0;
            read_start_req  <= 1'b0;
            dyn_ok_req      <= 1'b0;
            dyn_empty_req   <= 1'b0;
            dyn_read_value_latched <= 32'sd0;
        end else begin
            if (erase_start_pulse) erase_start_req <= 1'b1;
            if (erase_done_pulse)  erase_done_req  <= 1'b1;
            if (read_start_pulse)  read_start_req  <= 1'b1;
            if (read_diff_valid) begin
                dyn_ok_req <= 1'b1;
                dyn_read_value_latched <= read_diff;
            end
            if (flash_dyn_empty)
                dyn_empty_req <= 1'b1;
            if (tx_state == ST_TX_IDLE) begin
                if (erase_start_req)      erase_start_req <= 1'b0;
                else if (erase_done_req)  erase_done_req  <= 1'b0;
                else if (read_start_req)  read_start_req  <= 1'b0;
                else if (dyn_ok_req)      dyn_ok_req      <= 1'b0;
                else if (dyn_empty_req)   dyn_empty_req   <= 1'b0;
            end
        end
    end

    assign dump_word_ready = (tx_state == ST_TX_IDLE) && !reply_req && !hb_req && !cmd_valid
                             && !erase_start_req && !erase_done_req && !read_start_req && !dyn_ok_req && !dyn_empty_req;
    assign dump_meta_ready = (tx_state == ST_TX_IDLE) && !dump_word_valid && !reply_req && !hb_req && !cmd_valid
                             && !erase_start_req && !erase_done_req && !read_start_req && !dyn_ok_req && !dyn_empty_req;

    // TX状态机：直接流式发送，不使用BRAM/tx_buf缓存
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            tx_state          <= ST_TX_IDLE;
            tx_ptr            <= 8'd0;
            tx_len            <= 8'd0;
            tx_msg_type       <= MSG_REPLY;
            uart_tx_en        <= 1'b0;
            dump_addr_latched <= 24'd0;
            dump_data_latched <= 32'd0;
            dump_done_sent    <= 1'b0;
        end else begin
            case (tx_state)
                ST_TX_IDLE: begin
                    uart_tx_en <= 1'b0;
                    tx_ptr <= 8'd0;
                    if (!dump_done)
                        dump_done_sent <= 1'b0;

                    // D命令响应优先级最高（立即回复，不被心跳覆盖）
                    if (dyn_ok_req) begin
                        tx_msg_type <= MSG_DYN_OK;
                        tx_len <= tx_msg_len(MSG_DYN_OK);
                        tx_state <= ST_TX_SEND;
                    end else if (dyn_empty_req) begin
                        tx_msg_type <= MSG_DYN_EMPTY;
                        tx_len <= tx_msg_len(MSG_DYN_EMPTY);
                        tx_state <= ST_TX_SEND;
                    end else if (erase_start_req) begin
                        tx_msg_type <= MSG_ERASE_START;
                        tx_len <= tx_msg_len(MSG_ERASE_START);
                        tx_state <= ST_TX_SEND;
                    end else if (erase_done_req) begin
                        tx_msg_type <= MSG_ERASE_DONE;
                        tx_len <= tx_msg_len(MSG_ERASE_DONE);
                        tx_state <= ST_TX_SEND;
                    end else if (read_start_req) begin
                        tx_msg_type <= MSG_READ_START;
                        tx_len <= tx_msg_len(MSG_READ_START);
                        tx_state <= ST_TX_SEND;
                    end else if (dump_mode && dump_word_valid) begin
                        dump_addr_latched <= dump_word_addr;
                        dump_data_latched <= dump_word_data;
                        tx_msg_type <= MSG_DUMP_WORD;
                        tx_len <= tx_msg_len(MSG_DUMP_WORD);
                        tx_state <= ST_TX_SEND;
                    end else if (dump_mode && dump_done && !dump_done_sent) begin
                        tx_msg_type <= MSG_DUMP_DONE;
                        tx_len <= tx_msg_len(MSG_DUMP_DONE);
                        dump_done_sent <= 1'b1;
                        tx_state <= ST_TX_SEND;
                    end else if (reply_req) begin
                        tx_msg_type <= MSG_REPLY;
                        tx_len <= tx_msg_len(MSG_REPLY);
                        tx_state <= ST_TX_SEND;
                    end else if (hb_req && !dump_mode) begin
                        tx_msg_type <= MSG_HEARTBEAT;
                        tx_len <= tx_msg_len(MSG_HEARTBEAT);
                        tx_state <= ST_TX_SEND;
                    end
                end

                ST_TX_SEND: begin
                    if (tx_ptr < tx_len) begin
                        uart_tx_en <= 1'b1;
                        if (uart_tx_en && tx_rdy) begin
                            if (tx_ptr == (tx_len - 8'd1)) begin
                                tx_ptr <= 8'd0;
                                tx_state <= ST_TX_IDLE;
                                uart_tx_en <= 1'b0;
                            end else begin
                                tx_ptr <= tx_ptr + 8'd1;
                            end
                        end
                    end else begin
                        uart_tx_en <= 1'b0;
                        tx_ptr <= 8'd0;
                        tx_state <= ST_TX_IDLE;
                    end
                end

                default: begin
                    uart_tx_en <= 1'b0;
                    tx_ptr <= 8'd0;
                    tx_state <= ST_TX_IDLE;
                end
            endcase
        end
    end
    // 发送完成脉冲
    // TX done pulse
    reg tx_done_reg;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n)
            tx_done_reg <= 1'b0;
        else begin
            if (tx_state == ST_TX_SEND && uart_tx_en && tx_rdy && (tx_ptr == (tx_len - 8'd1)))
                tx_done_reg <= 1'b1;
            else
                tx_done_reg <= 1'b0;
        end
    end

    assign tx_en   = uart_tx_en;
    assign tx_data = tx_data_mux;
    assign tx_done = tx_done_reg;

endmodule

module uart_tx_simple #(
    parameter BIT_CLKS = 432
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tx_en,
    input  wire [7:0] tx_data,
    output wire       tx_rdy,
    output reg        txd
);
    reg        busy;
    reg [15:0] baud_cnt;
    reg [3:0]  bit_idx;
    reg [9:0]  shifter;
    localparam [15:0] BIT_CLKS_U = BIT_CLKS;

    assign tx_rdy = !busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy     <= 1'b0;
            baud_cnt <= 16'd0;
            bit_idx  <= 4'd0;
            shifter  <= 10'h3FF;
            txd      <= 1'b1;
        end else if (!busy) begin
            txd      <= 1'b1;
            baud_cnt <= 16'd0;
            bit_idx  <= 4'd0;
            if (tx_en) begin
                busy     <= 1'b1;
                shifter  <= {1'b1, tx_data, 1'b0};
                txd      <= 1'b0;
                baud_cnt <= BIT_CLKS_U - 16'd1;
            end
        end else if (baud_cnt == 16'd0) begin
            baud_cnt <= BIT_CLKS_U - 16'd1;
            if (bit_idx == 4'd9) begin
                busy    <= 1'b0;
                bit_idx <= 4'd0;
                shifter <= 10'h3FF;
                txd     <= 1'b1;
            end else begin
                bit_idx <= bit_idx + 4'd1;
                txd     <= shifter[1];
                shifter <= {1'b1, shifter[9:1]};
            end
        end else begin
            baud_cnt <= baud_cnt - 16'd1;
        end
    end
endmodule

module uart_rx_simple #(
    parameter BIT_CLKS = 432
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rxd,
    output reg        rx_vld,
    output reg  [7:0] rx_data,
    output reg        rx_err
);
    localparam RX_IDLE  = 2'd0;
    localparam RX_START = 2'd1;
    localparam RX_DATA  = 2'd2;
    localparam RX_STOP  = 2'd3;

    reg [1:0]  state;
    reg [1:0]  rxd_sync;
    reg [15:0] baud_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  data_shift;
    wire       rxd_s = rxd_sync[1];
    localparam [15:0] BIT_CLKS_U = BIT_CLKS;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rxd_sync <= 2'b11;
        else
            rxd_sync <= {rxd_sync[0], rxd};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= RX_IDLE;
            baud_cnt   <= 16'd0;
            bit_idx    <= 3'd0;
            data_shift <= 8'd0;
            rx_data    <= 8'd0;
            rx_vld     <= 1'b0;
            rx_err     <= 1'b0;
        end else begin
            rx_vld <= 1'b0;

            case (state)
                RX_IDLE: begin
                    rx_err <= 1'b0;
                    bit_idx <= 3'd0;
                    if (!rxd_s) begin
                        state <= RX_START;
                        baud_cnt <= (BIT_CLKS_U >> 1);
                    end
                end

                RX_START: begin
                    if (baud_cnt == 16'd0) begin
                        if (!rxd_s) begin
                            state <= RX_DATA;
                            baud_cnt <= BIT_CLKS_U - 16'd1;
                        end else begin
                            state <= RX_IDLE;
                        end
                    end else begin
                        baud_cnt <= baud_cnt - 16'd1;
                    end
                end

                RX_DATA: begin
                    if (baud_cnt == 16'd0) begin
                        data_shift[bit_idx] <= rxd_s;
                        baud_cnt <= BIT_CLKS_U - 16'd1;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state <= RX_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        baud_cnt <= baud_cnt - 16'd1;
                    end
                end

                RX_STOP: begin
                    if (baud_cnt == 16'd0) begin
                        rx_data <= data_shift;
                        rx_vld <= rxd_s;
                        rx_err <= !rxd_s;
                        state <= RX_IDLE;
                    end else begin
                        baud_cnt <= baud_cnt - 16'd1;
                    end
                end

                default: begin
                    state <= RX_IDLE;
                end
            endcase
        end
    end
endmodule

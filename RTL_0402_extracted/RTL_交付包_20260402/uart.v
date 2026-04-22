`timescale 1ns / 1ps

module uart (
    input  wire        sys_clk,      // 50MHz系统时钟 (已确认IP核配置为50MHz输入, 波特率9600)
    input  wire        sys_rst_n,    // 系统复位
    
    // UART Physical Interface
    input  wire        uart_rx,
    output wire        uart_tx,
    
    // System Status Inputs (for Reporting)
    input  wire [3:0]  sys_state,    // 状态机状态
    input  wire        pps_valid,    // GPS PPS有效标志
    input  wire        fifo_empty,   // FIFO空标志
    input  wire        fifo_full,    // FIFO满标志
    input  wire [7:0]  fifo_dout,    // FIFO数据输出
    input  wire        train_done,   // 训练完成标志
    // 调试输入信号
    input  wire [2:0]  dbg_state,    // 长周期计数器状态
    input  wire [7:0]  dbg_pps_cnt,  // 已计数的PPS数
    input  wire        dbg_gps_pps_posedge, // GPS PPS上升沿
    input  wire        dbg_counting_active, // 计数激活标志
    // 新增：period_done 计数器
    input  wire [31:0] period_cnt,   // period_done 计数
    // 新增：FIFO写入事件计数器（用于观测写入发生）
    input  wire [15:0] fwe_cnt,
    // 新增：FIFO读出事件计数器（用于观测读走发生）
    input  wire [15:0] fre_cnt,
    // 新增：diff_valid脉冲计数（源/写入端）
    input  wire [15:0] dv50_cnt,
    input  wire [15:0] dv25_cnt,
    // 新增：状态变化标志
    input  wire state_changed,       // 状态变化标志
    // 新增：diff_valid调试信号
    input  wire        dbg_diff_valid, // diff_valid标志
    // 新增：差值数据
    input  wire signed [31:0] diff_result, // 动态残差（用于在线补偿）
    input  wire signed [31:0] aging_diff_result, // 老化采样差值（用于长期建模）
    // 新增：高精度计数调试信号
    input  wire [31:0] dbg_gps_phase_sum,   // GPS累积计数
    input  wire [31:0] dbg_dpps_phase_sum,  // DPPS累积计数
    input  wire [31:0] dbg_dpps_phase_cnt,  // DPPS相位计数器
    input  wire [31:0] dbg_dpps_pulse_cnt,  // DPPS脉冲宽度计数器
    input  wire        dbg_dpps_pulse_active, // DPPS脉冲活跃标志
    input  wire [3:0]  dbg_dpps_cnt,        // DPPS上升沿计数
    input  wire        dbg_dpps_posedge,    // DPPS上升沿检测
    // 新增：FIFO流程检测信号
    input  wire        fifo_flush_req,       // FIFO刷新请求
    input  wire        fifo_we_debug,        // FIFO写使能调试
    input  wire [1:0]  fifo_state_debug,    // FIFO状态调试(00=有数据,01=空,10=满)
    // 新增：Flash状态检测信号
    input  wire        flash_busy,           // Flash忙标志
    input  wire        flash_done,           // Flash操作完成
    input  wire        write_done,           // 仅写入完成（不含擦除）
    input  wire        flash_error,          // Flash操作错误
    input  wire [7:0]  dbg_rd_first_byte,    // 调试：读Flash第一个字节
    input  wire        dbg_rd_first_valid,   // 调试：第一个字节有效
    input  wire [23:0] dbg_flash_cur_addr,   // 调试：Flash当前地址
    input  wire [15:0] dbg_poll_attempts,    // 调试：最后轮询次数
    input  wire [7:0]  dbg_last_sr,          // 调试：最后状态寄存器值
    input  wire [2:0]  dbg_sr_stage,         // 调试：最近SR对应子阶段(1=PP_WREN检查,2=PP轮询,3=ERASE_WREN检查,4=ERASE轮询)
    input  wire [7:0]  dbg_last_cmd,         // 调试：最近一次SPI事务命令字节
    input  wire [3:0]  dbg_last_len,         // 调试：最近一次SPI事务长度
    input  wire [7:0]  dbg_last_rx0,         // 调试：最近一次SPI事务接收字节0
    input  wire [7:0]  dbg_last_rx1,         // 调试：最近一次SPI事务接收字节1
    input  wire [3:0]  dbg_err_code,         // 调试：错误码(1=ERASE轮询超时,2=PROG轮询超时)
    input  wire [23:0] dbg_jedec_id,         // 调试：JEDEC ID
    input  wire        dbg_jedec_valid,      // 调试：JEDEC ID有效
    input  wire [6:0]  dbg_flash_state,      // 调试：Flash状态机当前状态
    input  wire [7:0]  dbg_txn_done_cnt,     // 调试：SPI事务完成计数
    input  wire [7:0]  hist_diff,            // 历史差值数据
    input  wire        hist_diff_valid,      // 历史差值有效
    input  wire signed [31:0] read_diff,     // Flash读出的差值（符号扩展）
    input  wire        read_diff_valid,      // Flash读出差值有效
    input  wire [23:0] dpps_offset_dbg,      // 当前补偿偏移量
    input  wire signed [23:0] dyn_offset_dbg, // 动态补偿偏移
    input  wire signed [15:0] comp_step_dbg,  // 每秒动态补偿步进
    input  wire        comp_valid_dbg,         // 动态补偿步进有效
    input  wire        dyn_ready_dbg,          // 动态补偿就绪
    input  wire signed [31:0] comp_fixed_dbg,  // 稳定后3窗口平均得到的固定残差参数
    input  wire [31:0] last_prog_word,       // 最后写入Flash的32位数据镜像

    // Dump stream interface (from top)
    input  wire        dump_mode,
    input  wire        dump_word_valid,
    input  wire [23:0] dump_word_addr,
    input  wire [31:0] dump_word_data,
    input  wire        dump_done,
    input  wire        dump_meta_valid,
    input  wire [15:0] dump_meta_fwe,
    input  wire [15:0] dump_meta_fre,
    input  wire [15:0] dump_meta_limit,
    output wire        dump_word_ready,
    output wire        dump_meta_ready,
        output reg  [3:0]  state_cmd,    // 状态切换指令 (0=No Cmd)
    output reg         cmd_valid,    // 指令有效脉冲
    output wire        tx_en,        // UART发送使能信号
    output wire        tx_done       // UART发送完成信号
);

    // ========================================================================
    // 1. Instantiate EF2M45 UART IP
    // ========================================================================
    wire       rx_vld;
    wire [7:0] rx_data;
    wire       tx_rdy;
    reg  [7:0] tx_data_reg;
    wire [7:0] tx_data;
    reg        uart_tx_en;
    
    // EF2M45 UART IP Instance
    UART_0 u_uart_ip (
        .clk        (sys_clk),
        .rst_n      (sys_rst_n),
        .rx_vld     (rx_vld),
        .rx_data    (rx_data),
        .rx_err     (),             // Ignore errors for now
        .rxd        (uart_rx),
        .tx_rdy     (tx_rdy),
        .tx_en      (uart_tx_en),
        .tx_data    (tx_data),
        .txd        (uart_tx)
    );

    // ========================================================================
    // 2. RX Logic: Command Parsing & Echo FIFO
    // ========================================================================
    // Commands:
    // "CALC_DIFF\r\n"
    // "TRAIN_DPPS\r\n"
    // "QUERY_STATE\r\n"
    
    localparam MAX_CMD_LEN = 16;
    reg [7:0] cmd_buf [0:MAX_CMD_LEN-1];
    reg [3:0] buf_idx;
    
    // 新增：心跳包RX状态记录
    reg rx_received_flag;

    // 新增：DF 锁存
    reg df_latch;
    reg df_latch_next;

    // diff_result 绝对值（用于串口显示）
    // 说明：在 Holdover(M:H, pps_valid=0) 下，将GPS/DIFF/ALN相关显示字段清零，避免误解为仍在实时计算
    wire signed [31:0] diff_disp = pps_valid ? diff_result : 32'sd0;
    wire [31:0] diff_abs = diff_disp[31] ? (-diff_disp) : diff_disp;
    wire [23:0] dyn_abs  = dyn_offset_dbg[23] ? (-dyn_offset_dbg) : dyn_offset_dbg;
    wire [15:0] comp_abs = comp_step_dbg[15] ? (-comp_step_dbg) : comp_step_dbg;
    wire signed [31:0] comp_fixed_disp = comp_fixed_dbg;
    wire [31:0] comp_fixed_abs = comp_fixed_disp[31] ? (-comp_fixed_disp) : comp_fixed_disp;


    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            buf_idx <= 4'd0;
            state_cmd <= 4'd0;
            cmd_valid <= 1'b0;
            rx_received_flag <= 1'b0; // 复位清零
        end else begin
            cmd_valid <= 1'b0; // Pulse default low
            
            if (rx_vld) begin
                // 新增：记录收到过数据
                rx_received_flag <= 1'b1;

                // Store char
                if (buf_idx < MAX_CMD_LEN) begin
                    cmd_buf[buf_idx] <= rx_data;
                    buf_idx <= buf_idx + 4'd1;
                end
                
                // Check for end of line (\n = 0x0A)
                if (rx_data == 8'h0A) begin
                    // Simple parsing based on first few chars and length to save logic
                    // CALC_DIFF (Length 11 incl \r\n) -> Starts with 'C' (0x43)
                    // TRAIN_DPPS (Length 12 incl \r\n) -> Starts with 'T' (0x54)
                    // STATE (Length 6 incl \r\n) -> Starts with 'S' (0x53)
                    
                    if (cmd_buf[0] == "C" && buf_idx >= 9) begin
                        state_cmd <= 4'b0001; // CALC_DIFF -> Target State 1
                        cmd_valid <= 1'b1;
                    end else if (cmd_buf[0] == "T" && buf_idx >= 10) begin
                        state_cmd <= 4'b0010; // TRAIN_DPPS -> Target State 2
                        cmd_valid <= 1'b1;
                    end else if (cmd_buf[0] == "S") begin
                        // State query triggers report
                        state_cmd <= 4'b0000; // No state change
                        cmd_valid <= 1'b1; // Pulse valid to trigger report
                    end
                    
                    // Reset buffer
                    buf_idx <= 4'd0;
                end
            end
        end
    end

    // ========================================================================
    // 3. Heartbeat Timer (New Feature)
    // ========================================================================
    
    // Move Declarations up for visibility
    reg [2:0] tx_state;
    localparam ST_TX_IDLE = 3'd0, ST_TX_PREP = 3'd1, ST_TX_SEND = 3'd2;

    // 新增：心跳包定时逻辑
    // 50MHz clock, 1 second = 50,000,000 cycles
    localparam integer HB_PERIOD = 50_000_000;
    reg [27:0] hb_cnt;
    reg        hb_req; // 心跳发送请求
    reg        reply_req; // 新增：指令回复请求Latch
    reg        rdv_latched; // 读回有效锁存
    reg [15:0] rdv_cnt;     // 读回有效计数
    


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
            // 计数器逻辑
            if (hb_cnt >= HB_PERIOD - 1) begin
                hb_cnt <= 28'd0;
                hb_req <= 1'b1; // 触发请求
            end else begin
                hb_cnt <= hb_cnt + 28'd1;
            end
            
            // Latch cmd_valid as reply_req
            if (cmd_valid) begin
                reply_req <= 1'b1;
            end

            // RDV锁存与计数
            if (read_diff_valid) begin
                rdv_latched <= 1'b1;
                rdv_cnt <= rdv_cnt + 16'd1;
            end

            // DF 锁存逻辑 - 使用临时变量避免多驱动冲突
            df_latch_next = df_latch;
            
            if (dbg_diff_valid) begin
                df_latch_next = 1'b1;
            end else if (tx_state == ST_TX_PREP && tx_msg_type == 2'b01) begin
                // 在“准备心跳包”的这一拍清零：
                // - 本拍 TX_PREP 会用旧的 df_latch 填入 tx_buf（非阻塞读到旧值）
                // - 清零发生在本拍时钟沿更新后，确保下一次事件还能被锁存
                df_latch_next = 1'b0;
                rdv_latched <= 1'b0;
            end
            
            // 最终赋值
            df_latch <= df_latch_next;
            
            // 请求清除逻辑
            if (tx_state == ST_TX_IDLE) begin
                if (reply_req) begin
                    reply_req <= 1'b0; // Clear reply req when taken
                end else if (hb_req) begin
                    hb_req <= 1'b0; // Clear hb req when taken
                end
            end
        end
    end

    // ========================================================================
    // 4. TX Logic: Status Reporting & Heartbeat & Echo
    // ========================================================================
    // Format 1 (Cmd Reply): "STATE:xx,PPS:x,FIFO:xx,TRAIN:x\r\n" (~32 chars)
    // Format 2 (Heartbeat): "HEARTBEAT:CLK:25M,RST:OK,PPS:x,FIFO:xx,STATE:xx,RX:x\r\n" (~54 chars)
    // Format 3 (Echo): Single Byte
    
    // 修改：缩减缓冲区大小以节省资源（当前最长心跳131字节）
    reg [7:0] tx_buf [0:160]; 
    reg [7:0] tx_ptr; // 8位宽以支持最多255字节
    reg [7:0] tx_len; // 8位宽以支持最多255字节
    
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
    
    // flash_done 脉冲锁存：单拍脉冲展宽为心跳可见的持续标志
    reg flash_done_latch;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            flash_done_latch <= 1'b0;
        end else begin
            if (flash_done)
                flash_done_latch <= 1'b1;
            else if (tx_state == ST_TX_PREP && tx_msg_type == 3'b001)
                flash_done_latch <= 1'b0; // 心跳发出后清零
        end
    end

    // write_done 脉冲锁存
    reg write_done_latch;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            write_done_latch <= 1'b0;
        end else begin
            if (write_done)
                write_done_latch <= 1'b1;
            else if (tx_state == ST_TX_PREP && tx_msg_type == 3'b001)
                write_done_latch <= 1'b0;
        end
    end
    
    // 消息类型标识 (000=Cmd Reply,001=Heartbeat,010=DumpWord,011=DumpDone,100=DumpMeta)
    reg [2:0] tx_msg_type;
    reg [23:0] dump_addr_latched;
    reg [31:0] dump_data_latched;
    reg        dump_done_sent;

    assign dump_word_ready = (tx_state == ST_TX_IDLE) && !reply_req && !hb_req && !cmd_valid;
    assign dump_meta_ready = (tx_state == ST_TX_IDLE) && !dump_word_valid && !reply_req && !hb_req && !cmd_valid;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            tx_state <= ST_TX_IDLE;
            tx_ptr <= 7'd0;
            tx_msg_type <= 3'b000;
            tx_data_reg <= 8'd0;
            uart_tx_en <= 1'b0;
            dump_addr_latched <= 24'd0;
            dump_data_latched <= 32'd0;
            dump_done_sent <= 1'b0;
        end else begin
            case (tx_state)
                ST_TX_IDLE: begin
                    uart_tx_en <= 1'b0;
                    // 优先级：按键元信息 > Dump数据 > Dump结束 > 指令回复 > 心跳包
                    if (dump_meta_valid && dump_meta_ready) begin
                        tx_state <= ST_TX_PREP;
                        tx_msg_type <= 3'b100; // DumpMeta
                    end else if (dump_mode && dump_word_valid) begin
                        dump_addr_latched <= dump_word_addr;
                        dump_data_latched <= dump_word_data;
                        tx_state <= ST_TX_PREP;
                        tx_msg_type <= 3'b010; // DumpWord
                    end else if (dump_mode && dump_done && !dump_done_sent) begin
                        tx_state <= ST_TX_PREP;
                        tx_msg_type <= 3'b011; // DumpDone
                        dump_done_sent <= 1'b1;
                    end else if (reply_req) begin // Use latched request
                        tx_state <= ST_TX_PREP;
                        tx_msg_type <= 3'b000; // Cmd Reply
                    end else if (hb_req && !dump_mode) begin
                        tx_state <= ST_TX_PREP;
                        tx_msg_type <= 3'b001; // Heartbeat
                    end
                end
                
                ST_TX_PREP: begin
                    uart_tx_en <= 1'b0;
                    // Prepare Buffer based on msg type
                    if (tx_msg_type == 3'b000) begin
                        // Compact cmd reply: "S:x,T:x\r\n"
                        tx_buf[0]  <= "S"; tx_buf[1]  <= ":";
                        tx_buf[2]  <= hex2ascii(sys_state);
                        tx_buf[3]  <= ",";
                        tx_buf[4]  <= "T"; tx_buf[5]  <= ":";
                        tx_buf[6]  <= train_done ? "1" : "0";
                        tx_buf[7]  <= 8'h0D; // \r
                        tx_buf[8]  <= 8'h0A; // \n
                        tx_len <= 8'd9;
                        tx_ptr <= 8'd0;
                        tx_state <= ST_TX_SEND;
                    end else if (tx_msg_type == 3'b010) begin
                        // Dump word: ADDR=xxxxxx,DATA=xxxxxxxx\r\n（去掉DUMP前缀）
                        tx_buf[0]  <= "A"; tx_buf[1]  <= "D"; tx_buf[2]  <= "D"; tx_buf[3]  <= "R"; tx_buf[4]  <= "=";
                        tx_buf[5] <= hex2ascii(dump_addr_latched[23:20]);
                        tx_buf[6] <= hex2ascii(dump_addr_latched[19:16]);
                        tx_buf[7] <= hex2ascii(dump_addr_latched[15:12]);
                        tx_buf[8] <= hex2ascii(dump_addr_latched[11:8]);
                        tx_buf[9] <= hex2ascii(dump_addr_latched[7:4]);
                        tx_buf[10] <= hex2ascii(dump_addr_latched[3:0]);
                        tx_buf[11] <= ",";
                        tx_buf[12] <= "D"; tx_buf[13] <= "A"; tx_buf[14] <= "T"; tx_buf[15] <= "A"; tx_buf[16] <= "=";
                        tx_buf[17] <= hex2ascii(dump_data_latched[31:28]);
                        tx_buf[18] <= hex2ascii(dump_data_latched[27:24]);
                        tx_buf[19] <= hex2ascii(dump_data_latched[23:20]);
                        tx_buf[20] <= hex2ascii(dump_data_latched[19:16]);
                        tx_buf[21] <= hex2ascii(dump_data_latched[15:12]);
                        tx_buf[22] <= hex2ascii(dump_data_latched[11:8]);
                        tx_buf[23] <= hex2ascii(dump_data_latched[7:4]);
                        tx_buf[24] <= hex2ascii(dump_data_latched[3:0]);
                        tx_buf[25] <= 8'h0D;
                        tx_buf[26] <= 8'h0A;
                        tx_len <= 8'd27;
                        tx_ptr <= 8'd0;
                        tx_state <= ST_TX_SEND;
                    end else if (tx_msg_type == 3'b011) begin
                        // Dump done marker: "E\r\n"
                        tx_buf[0] <= "E";
                        tx_buf[1] <= 8'h0D;
                        tx_buf[2] <= 8'h0A;
                        tx_len <= 8'd3;
                        tx_ptr <= 8'd0;
                        tx_state <= ST_TX_SEND;
                    end else if (tx_msg_type == 3'b100) begin
                        // One-shot dump metadata
                        // DUMP_META:FWE=xxxx,FRE=xxxx,LIMIT=xxxx\r\n
                        tx_buf[0]  <= "D"; tx_buf[1]  <= "U"; tx_buf[2]  <= "M"; tx_buf[3]  <= "P"; tx_buf[4]  <= "_";
                        tx_buf[5]  <= "M"; tx_buf[6]  <= "E"; tx_buf[7]  <= "T"; tx_buf[8]  <= "A"; tx_buf[9]  <= ":";
                        tx_buf[10] <= "F"; tx_buf[11] <= "W"; tx_buf[12] <= "E"; tx_buf[13] <= "=";
                        tx_buf[14] <= hex2ascii(dump_meta_fwe[15:12]); tx_buf[15] <= hex2ascii(dump_meta_fwe[11:8]);
                        tx_buf[16] <= hex2ascii(dump_meta_fwe[7:4]);   tx_buf[17] <= hex2ascii(dump_meta_fwe[3:0]);
                        tx_buf[18] <= ",";
                        tx_buf[19] <= "F"; tx_buf[20] <= "R"; tx_buf[21] <= "E"; tx_buf[22] <= "=";
                        tx_buf[23] <= hex2ascii(dump_meta_fre[15:12]); tx_buf[24] <= hex2ascii(dump_meta_fre[11:8]);
                        tx_buf[25] <= hex2ascii(dump_meta_fre[7:4]);   tx_buf[26] <= hex2ascii(dump_meta_fre[3:0]);
                        tx_buf[27] <= ",";
                        tx_buf[28] <= "L"; tx_buf[29] <= "I"; tx_buf[30] <= "M"; tx_buf[31] <= "I"; tx_buf[32] <= "T"; tx_buf[33] <= "=";
                        tx_buf[34] <= hex2ascii(dump_meta_limit[15:12]); tx_buf[35] <= hex2ascii(dump_meta_limit[11:8]);
                        tx_buf[36] <= hex2ascii(dump_meta_limit[7:4]);   tx_buf[37] <= hex2ascii(dump_meta_limit[3:0]);
                        tx_buf[38] <= 8'h0D;
                        tx_buf[39] <= 8'h0A;
                        tx_len <= 8'd40;
                        tx_ptr <= 8'd0;
                        tx_state <= ST_TX_SEND;
                    end else if (tx_msg_type == 3'b001) begin
                        // 心跳按场景分流：
                        // 1) pps_valid=1 -> GPS锁定训练视图
                        // 2) pps_valid=0 -> Holdover守时视图
                        begin
                        // New Format: "HEARTBEAT:CLK:25M,RST:OK,PPS:x,FIFO:xx,STATE:xx,CHG:x,DBG:xx,CNT:xx,PD:xxxx,DF:x,DVRAW:x\r\n"
                        // 字段说明：
                        // CLK: 时钟频率
                        // RST: 复位状态
                        // PPS: GPS PPS信号有效标志 (1=有效, 0=无效)
                        // FIFO: FIFO状态 (00=有数据, 01=空, 10=满)
                        // STATE: 系统状态 (00=ST_STANDBY, 02=ST_CALC, 03=ST_TRAIN, 04=ST_TRAIN_DONE, 05=ST_PARAM_UPDATE)
                        // CHG: 状态变化标志 (1=状态变化, 0=无变化)
                        // DBG: 长周期计数器状态
                        // CNT: PPS计数器 (来自timekeeping_core的pps_cnt_8，范围00-0F)
                        // PD: period_done事件计数
                        // DF: 差值有效标志 (1=差值计算有效, 0=无效，来自timekeeping_core的diff_valid_flag)
                        // 简化格式：删除STATE，添加DPPS调试信息
                        // GPS有效：仅保留核心训练/统计字段
                        // CNT,DIFF,FWE,FRE,GPS,DPS,FL_DONE,HDIFF,HDIFV,ALN
                        // 精简心跳：CT,DIFF,FWE,FRE,FL_DONE,M
                        tx_buf[0]  <= "C"; tx_buf[1]  <= "T"; tx_buf[2]  <= ":";
                        tx_buf[3]  <= hex2ascii(dbg_pps_cnt[7:4]); tx_buf[4] <= hex2ascii(dbg_pps_cnt[3:0]); tx_buf[5] <= ",";

                        tx_buf[6]  <= "D"; tx_buf[7]  <= "I"; tx_buf[8]  <= "F"; tx_buf[9] <= "F"; tx_buf[10] <= ":";
                        tx_buf[11] <= diff_disp[31] ? "-" : "+";
                        tx_buf[12] <= "0"; tx_buf[13] <= "0";
                        tx_buf[14] <= hex2ascii(diff_abs[23:20]); tx_buf[15] <= hex2ascii(diff_abs[19:16]);
                        tx_buf[16] <= hex2ascii(diff_abs[15:12]); tx_buf[17] <= hex2ascii(diff_abs[11:8]);
                        tx_buf[18] <= hex2ascii(diff_abs[7:4]);   tx_buf[19] <= hex2ascii(diff_abs[3:0]);
                        tx_buf[20] <= ",";

                        tx_buf[21] <= "F"; tx_buf[22] <= "W"; tx_buf[23] <= "E"; tx_buf[24] <= ":";
                        tx_buf[25] <= hex2ascii(fwe_cnt[15:12]); tx_buf[26] <= hex2ascii(fwe_cnt[11:8]);
                        tx_buf[27] <= hex2ascii(fwe_cnt[7:4]);   tx_buf[28] <= hex2ascii(fwe_cnt[3:0]);
                        tx_buf[29] <= ",";

                        tx_buf[30] <= "F"; tx_buf[31] <= "R"; tx_buf[32] <= "E"; tx_buf[33] <= ":";
                        tx_buf[34] <= hex2ascii(fre_cnt[15:12]); tx_buf[35] <= hex2ascii(fre_cnt[11:8]);
                        tx_buf[36] <= hex2ascii(fre_cnt[7:4]);   tx_buf[37] <= hex2ascii(fre_cnt[3:0]);
                        tx_buf[38] <= ",";

                        tx_buf[39] <= "F"; tx_buf[40] <= "L"; tx_buf[41] <= "_"; tx_buf[42] <= "D"; tx_buf[43] <= "O"; tx_buf[44] <= "N"; tx_buf[45] <= "E"; tx_buf[46] <= ":";
                        tx_buf[47] <= flash_done_latch ? "1" : "0";
                        tx_buf[48] <= ",";

                        tx_buf[49] <= "W"; tx_buf[50] <= "D"; tx_buf[51] <= ":";
                        tx_buf[52] <= write_done_latch ? "1" : "0";
                        tx_buf[53] <= ",";

                        tx_buf[54] <= "L"; tx_buf[55] <= "W"; tx_buf[56] <= ":";
                        tx_buf[57] <= "0"; tx_buf[58] <= "0";
                        tx_buf[59] <= hex2ascii(last_prog_word[23:20]); tx_buf[60] <= hex2ascii(last_prog_word[19:16]);
                        tx_buf[61] <= hex2ascii(last_prog_word[15:12]); tx_buf[62] <= hex2ascii(last_prog_word[11:8]);
                        tx_buf[63] <= hex2ascii(last_prog_word[7:4]);   tx_buf[64] <= hex2ascii(last_prog_word[3:0]);
                        tx_buf[65] <= ",";

                        tx_buf[66] <= "L"; tx_buf[67] <= "C"; tx_buf[68] <= "M"; tx_buf[69] <= "D"; tx_buf[70] <= ":";
                        tx_buf[71] <= hex2ascii(dbg_last_cmd[7:4]);
                        tx_buf[72] <= hex2ascii(dbg_last_cmd[3:0]);
                        tx_buf[73] <= ",";

                        tx_buf[74] <= "R"; tx_buf[75] <= "X"; tx_buf[76] <= "1"; tx_buf[77] <= ":";
                        tx_buf[78] <= hex2ascii(dbg_last_rx1[7:4]);
                        tx_buf[79] <= hex2ascii(dbg_last_rx1[3:0]);
                        tx_buf[80] <= ",";

                        tx_buf[81] <= "J"; tx_buf[82] <= "V"; tx_buf[83] <= ":";
                        tx_buf[84] <= dbg_jedec_valid ? "1" : "0";
                        tx_buf[85] <= ",";

                        tx_buf[86] <= "J"; tx_buf[87] <= "I"; tx_buf[88] <= ":";
                        tx_buf[89] <= hex2ascii(dbg_jedec_id[23:20]);
                        tx_buf[90] <= hex2ascii(dbg_jedec_id[19:16]);
                        tx_buf[91] <= hex2ascii(dbg_jedec_id[15:12]);
                        tx_buf[92] <= hex2ascii(dbg_jedec_id[11:8]);
                        tx_buf[93] <= hex2ascii(dbg_jedec_id[7:4]);
                        tx_buf[94] <= hex2ascii(dbg_jedec_id[3:0]);
                        tx_buf[95] <= ",";

                        tx_buf[96] <= "D"; tx_buf[97] <= "Y"; tx_buf[98] <= ":";
                        tx_buf[99] <= dyn_offset_dbg[23] ? "-" : "+";
                        tx_buf[100] <= hex2ascii(dyn_abs[23:20]);
                        tx_buf[101] <= hex2ascii(dyn_abs[19:16]);
                        tx_buf[102] <= hex2ascii(dyn_abs[15:12]);
                        tx_buf[103] <= hex2ascii(dyn_abs[11:8]);
                        tx_buf[104] <= hex2ascii(dyn_abs[7:4]);
                        tx_buf[105] <= hex2ascii(dyn_abs[3:0]);
                        tx_buf[106] <= ",";

                        tx_buf[107] <= "C"; tx_buf[108] <= "S"; tx_buf[109] <= ":";
                        tx_buf[110] <= comp_step_dbg[15] ? "-" : "+";
                        tx_buf[111] <= hex2ascii(comp_abs[15:12]);
                        tx_buf[112] <= hex2ascii(comp_abs[11:8]);
                        tx_buf[113] <= hex2ascii(comp_abs[7:4]);
                        tx_buf[114] <= hex2ascii(comp_abs[3:0]);
                        tx_buf[115] <= ",";

                        tx_buf[116] <= "C"; tx_buf[117] <= "V"; tx_buf[118] <= ":";
                        tx_buf[119] <= comp_valid_dbg ? "1" : "0";
                        tx_buf[120] <= ",";

                        tx_buf[121] <= "D"; tx_buf[122] <= "R"; tx_buf[123] <= ":";
                        tx_buf[124] <= dyn_ready_dbg ? "1" : "0";
                        tx_buf[125] <= ",";

                        tx_buf[126] <= "C"; tx_buf[127] <= "A"; tx_buf[128] <= ":";
                        tx_buf[129] <= comp_fixed_disp[31] ? "-" : "+";
                        tx_buf[130] <= "0";
                        tx_buf[131] <= "0";
                        tx_buf[132] <= hex2ascii(comp_fixed_abs[23:20]);
                        tx_buf[133] <= hex2ascii(comp_fixed_abs[19:16]);
                        tx_buf[134] <= hex2ascii(comp_fixed_abs[15:12]);
                        tx_buf[135] <= hex2ascii(comp_fixed_abs[11:8]);
                        tx_buf[136] <= hex2ascii(comp_fixed_abs[7:4]);
                        tx_buf[137] <= hex2ascii(comp_fixed_abs[3:0]);
                        tx_buf[138] <= ",";

                        tx_buf[139] <= "M"; tx_buf[140] <= ":";
                        tx_buf[141] <= pps_valid ? "L" : "H";
                        tx_buf[142] <= 8'h0D;
                        tx_buf[143] <= 8'h0A;

                        tx_len <= 8'd144;
                        end

                        tx_ptr <= 8'd0;
                        tx_state <= ST_TX_SEND;
                    end
                end
                
                ST_TX_SEND: begin
                    if (tx_ptr < tx_len) begin
                        // 锁存当前数据
                        tx_data_reg <= tx_buf[tx_ptr];
                        // 使能发送
                        uart_tx_en <= 1'b1;
                        // 等待发送完成
                        if (tx_rdy) begin
                            // 发送完成，移动指针
                            tx_ptr <= tx_ptr + 8'd1;
                        end
                    end else begin
                        uart_tx_en <= 1'b0;
                        tx_state <= ST_TX_IDLE;
                    end
                end
            endcase
        end
    end
    
    // 新增：发送完成信号
    reg tx_done_reg;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            tx_done_reg <= 1'b0;
        end else begin
            if (tx_state == ST_TX_SEND && tx_ptr == tx_len - 1 && tx_rdy) begin
                tx_done_reg <= 1'b1;
            end else begin
                tx_done_reg <= 1'b0;
            end
        end
    end
    
    // Connect to IP
    assign tx_en   = uart_tx_en;
    assign tx_data = tx_data_reg;
    assign tx_done = tx_done_reg;

endmodule

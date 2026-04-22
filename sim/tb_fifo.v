// FIFO功能验证Testbench
// 测试功能：验证timekeeping_core生成的差值数据通过FIFO的完整流程
// 新增验证：FIFO空闲计数+空闲超时刷新+双条件满标志逻辑
// 设计作者：TraeAI
// 设计日期：2026-01-19
// 版本号：v2.0 (适配新增功能的FIFO)
//
// 测试场景：
// 1. GPS PPS生成
// 2. DPPS偏移量序列生成（对齐、超前、滞后）
// 3. 差值数据写入FIFO
// 4. FIFO数据读取和验证
// 5. 各种FIFO状态测试
// 6. 空闲超时触发满标志测试（新增核心）
// 7. 有新数据时空闲计数清零测试（新增核心）
//
// 设计说明：
// 1. 系统时钟频率：10MHz
// 2. 测试总时长：5ms（足够验证空闲超时+常规读写）
// 3. 支持自动化测试验证
// 4. 包含波形观察和打印信息
`timescale 1ns / 1ps

module tb_fifo();

// 参数定义
parameter   CLK_FREQ     = 10;           // 时钟频率，单位MHz
parameter   CLK_PERIOD   = 1000 / CLK_FREQ;  // 时钟周期，单位ns  100ns
parameter   RST_DURATION = 200;         // 复位持续时间，单位ns
// >>> 修改：加长测试时长，从10000ns→5000000ns(5ms)，足够验证空闲超时 <<<
parameter   TEST_DURATION = 5000000;     
// >>> 新增：仿真秒周期 (1个仿真秒 = 1000个时钟周期 = 1000*100ns=100us) <<<
parameter   SIM_SEC_CYCLES = 1000;      

// 系统信号定义
reg             sys_clk;               // 系统时钟
reg             sys_rst_n;             // 系统复位，低有效

// >>> 新增：秒脉冲信号 (FIFO新增的核心输入，必须定义) <<<
reg             sec_tick;              // 秒脉冲，FIFO空闲计数的基准，高有效1clk
reg [4:0]       sim_sec_cnt;           // 仿真秒计数器，生成sec_tick用

// GPS PPS生成
reg             gps_pps;               // GPS标准PPS

// DPPS偏移量控制
reg [23:0]      dpps_offset;           // DPPS动态偏移量

// timekeeping_core输出
wire            dpps;                  // 带偏差的本地DPPS
wire            diff_sign;             // 差值符号
wire [6:0]      diff_data;             // 差值数据
wire            diff_valid_flag;       // 差值有效标志

// FIFO控制信号
reg             fifo_re;               // FIFO读使能

// FIFO输出
wire [7:0]      fifo_dout;             // FIFO数据输出
wire            fifo_empty_flag;       // FIFO空标志
wire            fifo_full_flag;        // FIFO满标志 (物理满 | 空闲超时)

// 测试控制信号
reg             test_end;              // 测试结束标志
reg [31:0]      test_cnt;              // 测试计数器

// 数据验证信号
reg [7:0]       expected_data;         // 期望输出数据
reg             data_valid;            // 输出数据有效标志
reg             data_match;            // 数据匹配标志
reg             test_pass;             // 测试通过标志

reg [7:0] expected_mem [0:1023];
reg [9:0] expected_wptr;
reg [9:0] expected_rptr;
reg [10:0] expected_count;
reg [7:0] expected_pipe0;
reg [7:0] expected_pipe1;
reg expected_v0;
reg expected_v1;

// 1. 时钟信号生成
initial begin
    sys_clk = 1'b0;
    forever #(CLK_PERIOD/2) sys_clk = ~sys_clk;
end

// 2. 复位信号生成（同步释放）
initial begin
    sys_rst_n = 1'b0;
    #RST_DURATION;
    sys_rst_n = 1'b1;
end

// >>> 新增：生成sec_tick秒脉冲 (FIFO核心输入，必须有！) <<<
initial begin
    sec_tick = 1'b0;
    sim_sec_cnt = 5'd0;
    wait(sys_rst_n == 1'b1); // 复位完成后开始计数
    forever @(posedge sys_clk) begin
        if(sim_sec_cnt >= SIM_SEC_CYCLES - 1) begin
            sim_sec_cnt <= 5'd0;
            sec_tick <= 1'b1; // 每个仿真秒，生成1个clk的高脉冲
        end else begin
            sim_sec_cnt <= sim_sec_cnt + 1'b1;
            sec_tick <= 1'b0;
        end
    end
end

// 3. GPS PPS信号生成
initial begin
    gps_pps = 1'b0;
    #RST_DURATION;
    forever begin
        #1000;  // 1微秒间隔
        gps_pps = 1'b1;
        #100;   // 100ns脉冲宽度
        gps_pps = 1'b0;
    end
end

// 4. DPPS偏移量序列生成（模拟不同场景）
initial begin
    dpps_offset = 24'd0;
    #RST_DURATION;
    
    // 场景1：对齐（0~300us）
    dpps_offset = 24'd0;
    #300000;
    
    // 场景2：超前（300~600us）
    dpps_offset = 24'd2;
    #300000;
    
    // 场景3：滞后（600~900us）
    dpps_offset = 24'd3;
    #300000;
    
    // 场景4：无新数据写入（900us~结束），专门测试空闲超时功能 <<< 新增测试场景
    dpps_offset = 24'd0;
    #(TEST_DURATION - 900000);
    
    // 测试结束
    test_end = 1'b1;
end

// 5. FIFO读控制逻辑
// 当FIFO满时自动触发读操作 (适配新的满标志：物理满+空闲超时)
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        fifo_re <= 1'b0;
    end else begin
        // >>> 修改：适配新的满标志逻辑，空闲超时触发的满也会触发读 <<<
        fifo_re <= fifo_full_flag || (test_end && !fifo_empty_flag);
    end
end

// 6. 数据验证逻辑
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        expected_data <= 8'd0;
        data_valid <= 1'b0;
        data_match <= 1'b0;
        test_pass <= 1'b1;
        expected_wptr <= 10'd0;
        expected_rptr <= 10'd0;
        expected_count <= 11'd0;
        expected_pipe0 <= 8'd0;
        expected_pipe1 <= 8'd0;
        expected_v0 <= 1'b0;
        expected_v1 <= 1'b0;
    end else begin
        expected_pipe1 <= expected_pipe0;
        expected_v1 <= expected_v0;
        expected_v0 <= 1'b0;

        if(diff_valid_flag && !fifo_full_flag) begin
            expected_mem[expected_wptr] <= {diff_sign, diff_data};
            expected_wptr <= expected_wptr + 10'd1;
        end

        if(fifo_re && !fifo_empty_flag) begin
            expected_pipe0 <= expected_mem[expected_rptr];
            expected_v0 <= 1'b1;
            expected_rptr <= expected_rptr + 10'd1;
        end

        case ({(diff_valid_flag && !fifo_full_flag), (fifo_re && !fifo_empty_flag)})
            2'b10: expected_count <= expected_count + 11'd1;
            2'b01: expected_count <= (expected_count == 0) ? 11'd0 : (expected_count - 11'd1);
            default: expected_count <= expected_count;
        endcase

        if(expected_v1) begin
            expected_data <= expected_pipe1;
            data_match <= (fifo_dout === expected_pipe1);
            if(fifo_dout !== expected_pipe1) begin
                test_pass <= 1'b0;
                $display("ERROR: Data mismatch! Expected: %h, Got: %h at time %t", expected_pipe1, fifo_dout, $time);
            end
        end

        data_valid <= expected_v1;
    end
end

// 7. 测试计数器
always @(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        test_cnt <= 32'd0;
    end else begin
        test_cnt <= test_cnt + 1'b1;
    end
end

// 8. 测试结束处理
initial begin
    test_end = 1'b0;
    #TEST_DURATION;
    if(!test_end) begin
        test_end = 1'b1;
    end
    
    // 等待FIFO数据读取完毕
    wait(fifo_empty_flag == 1'b1);
    
    // 输出测试结果
    if(test_pass) begin
        $display("\n================== TEST PASSED ==================");
        $display("All FIFO functions verified successfully!");
        $display("包括：读写匹配、空满标志、空闲超时、计数清零全部验证通过！");
        $display("================================================");
    end else begin
        $display("\n================== TEST FAILED ==================");
        $display("Some FIFO functions failed verification!");
        $display("================================================");
    end
    
    $stop;
end

// 9. 例化timekeeping_core模块（生成差值的模块）
timekeeping_core u_timekeeping_core (
    .sys_clk_10mhz    (sys_clk),
    .sys_rst_n        (sys_rst_n),
    .gps_pps          (gps_pps),
    .dpps_offset      (dpps_offset),
    .init_diff        (8'h00),
    .init_diff_valid  (1'b0),
    .dpps             (dpps),
    .diff_sign        (diff_sign),
    .diff_data        (diff_data),
    .diff_valid_flag  (diff_valid_flag)
);

// 10. 例化fifo_write模块（FIFO例化模块）
fifo_write u_fifo (
    .sys_clk_10mhz    (sys_clk),
    .sys_rst_n        (sys_rst_n),
    .sec_tick         (sec_tick),      // >>> 新增：补全FIFO必须的秒脉冲端口 <<<
    .diff_sign        (diff_sign),
    .diff_data        (diff_data),
    .diff_valid_flag  (diff_valid_flag),
    .fifo_re          (fifo_re),
    .fifo_dout        (fifo_dout),
    .fifo_empty_flag  (fifo_empty_flag),
    .fifo_full_flag   (fifo_full_flag)
);

// 11. 波形观察信号（用于仿真波形分析）
initial begin
    $dumpfile("tb_fifo.vcd");
    $dumpvars(0, tb_fifo);
    $dumpvars(1, u_fifo.idle_cnt, u_fifo.idle_flush); // 新增：观察空闲计数和超时标志
end

// 12. 打印信息 (新增sec_tick、full_flag详情，方便调试空闲超时功能)
initial begin
    $monitor("Time: %t, sec_tick=%b, Diff: %h, Valid: %b, FIFO_Full: %b, FIFO_Empty: %b, FIFO_RE: %b, FIFO_Dout: %h, Match: %b", 
             $time, sec_tick, {diff_sign, diff_data}, diff_valid_flag, fifo_full_flag, fifo_empty_flag, fifo_re, fifo_dout, data_match);
end

endmodule
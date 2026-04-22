// FIFO实例化模块
// 模块功能：用于存储timekeeping_core产生的差值数据，实现8位差值数据的FIFO缓存
// 设计作者：TraeAI
// 设计日期：2026-01-19
// 版本号：v1.0
//
// 输入端口：
// - sys_clk_10mhz：系统时钟，10MHz
// - sys_rst_n：系统复位，低有效
// - diff_sign：差值符号位，0=超前，1=滞后
// - diff_data：差值数值，7位，单位100ns
// - diff_valid_flag：差值有效标志
// - fifo_re：FIFO读使能，高有效
//
// 输出端口：
// - fifo_dout：FIFO数据输出，8位
// - fifo_empty_flag：FIFO空标志，高有效
// - fifo_full_flag：FIFO满标志，高有效
//
// 设计说明：
// 1. 将1位符号位和7位数值位组合为8位数据
// 2. 差值有效且FIFO未满时写入数据
// 3. 实例化Anlogic FIFO_0 IP核
`timescale 1ns / 1ps

module fifo_write_module (
    // 系统时钟和复位
    input           sys_clk_25mhz,
    input           sys_rst_n,
    
    // 强制刷新请求（GPS PPS丢失触发）
    input           flush_req,

    // 差值输入（来自timekeeping_core）
    input  signed [31:0] diff_value,
    input                diff_valid_flag,
    
    // FIFO控制信号
    input           fifo_re,
    
    // FIFO输出
    output [31:0]   fifo_dout,
    output          fifo_empty_flag,
    output          fifo_full_flag,
    // 新增：FIFO写使能调试输出
    output          fifo_we_debug,
    // 新增：FIFO状态调试输出(00=有数据,01=空,10=满)
    output [1:0]    fifo_state_debug
);
    
    // 信号定义
    wire [31:0]     fifo_di;           // FIFO输入完整32位差值


    wire            fifo_we;           // FIFO写使能
    wire            fifo_rst;          // FIFO复位
    wire            fifo_ip_full;      // FIFO IP满标志
    wire            fifo_ip_empty;     // FIFO IP空标志
    wire            fifo_ip_afull;     // FIFO IP几乎满标志
    wire [31:0]     fifo_do_32;        // FIFO IP 32位输出
    
    // CDC Synchronization Removed - Same Clock Domain (25MHz)
    
    // 将差值数据组合为8位FIFO输入
    // 第7位：diff_sign（0=超前，1=滞后）
    // 第6-0位：diff_data（差值数值）
    assign fifo_di = diff_value;


    
    // FIFO写使能：当差值有效且FIFO未满时写入
    assign fifo_we = diff_valid_flag & ~fifo_ip_full;
    assign fifo_we_debug = fifo_we;
    
    // FIFO满标志输出：快满(afull) OR (强制刷新且非空)
    // 适配EF2M45 IP核 修改：使用IP核的afull_flag作为触发写入Flash的标志，
    // 避免在完全物理满(full_flag)时等待期间丢失新来的数据
    assign fifo_full_flag = fifo_ip_afull | (flush_req & ~fifo_empty_flag);

    // FIFO空标志输出
    assign fifo_empty_flag = fifo_ip_empty;
    assign fifo_state_debug = fifo_ip_full ? 2'b10 : (fifo_ip_empty ? 2'b01 : 2'b00);
    
    // FIFO数据输出：截取低8位 (IP核输出为128位)
    assign fifo_dout = fifo_do_32;

    // FIFO复位信号：与系统复位同步 (IP核仅需一个复位)
    assign fifo_rst = ~sys_rst_n;

    // 适配EF2M45 IP核 修改：实例化新版 FIFO_write
    FIFO_write u_FIFO_write (
        .rst            (fifo_rst),
        .di             (fifo_di),
        .clk            (sys_clk_25mhz),
        .we             (fifo_we),
        .do             (fifo_do_32),
        .re             (fifo_re),
        .empty_flag     (fifo_ip_empty),
        .full_flag      (fifo_ip_full),
        .afull_flag     (fifo_ip_afull)
    );
    
endmodule

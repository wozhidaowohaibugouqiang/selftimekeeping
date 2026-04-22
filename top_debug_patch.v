// 请在 top.v 的第202行之后添加以下代码：

    // 新增：调试信号
    wire [23:0] core_dbg_dpps_offset;       // 来自 timekeeping_core 的 dpps_offset
    wire [31:0] core_dbg_dpps_period_target; // 来自 timekeeping_core 的 dpps_period_target

// 然后在 timekeeping_core 实例化部分（约第1391行之后）添加以下连接：
        .dbg_dpps_offset      (core_dbg_dpps_offset),
        .dbg_dpps_period_target(core_dbg_dpps_period_target)

// 最后在 uart 实例化部分添加以下连接（将调试信号传递给 UART）：
        .dpps_offset_dbg      (core_dbg_dpps_offset),
        .dpps_period_target_dbg(core_dbg_dpps_period_target)

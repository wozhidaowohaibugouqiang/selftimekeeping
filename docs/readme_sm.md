# 守时系统仿真设计文档

## 1. 概述
本仿真环境旨在验证自主守时系统的全流程功能，包括 GPS 锁定时的差值采集、GPS 丢失时的数据存储以及守时模式下的频率补偿。仿真采用闭环反馈机制，数学模拟晶振漂移与补偿效果，能够精确评估系统的守时精度。

## 2. 仿真模块详解

### 2.1 系统顶层测试基准 (`sim/tb_top.v`)
这是仿真的核心控制模块，负责协调整个测试流程。
- **闭环反馈模拟**：仿真环境内部维护一个虚拟的“本地时钟周期” (`comp_period`)。
  - 初始状态：`comp_period` = 标准周期 (10MHz) + `osc_delta` (模拟晶振漂移)。
  - 补偿状态：`comp_period` = 标准周期 + `osc_delta` - `comp_step` (DUT输出的补偿值)。
- **漂移注入**：通过 `set_osc_delta` 任务动态改变 `osc_delta`，模拟环境温度变化引起的晶振频率漂移。
- **误差统计**：实时计算未补偿时钟 (`dpps_osc`) 和补偿后时钟 (`dpps_comp`) 相对于理想 GPS PPS (`ideal_pps`) 的相位误差，并统计最大绝对误差。

### 2.2 SPI Flash 仿真模型 (`sim/spi_flash_model.v`)
模拟标准 SPI Flash 存储器的行为，支持以下指令：
- **06h (WREN)**: 写使能。
- **20h (SE)**: 扇区擦除 (4KB)。
- **02h (PP)**: 页编程 (256B)。
- **03h (READ)**: 读数据。
- **05h (RDSR)**: 读状态寄存器 (模拟 WIP 忙碌位)。
模型包含后门接口 (`dbg_mem_xxxx`)，允许测试平台直接校验 Flash 内部存储的数据是否正确。

### 2.3 守时核心仿真模型 (`sim/timekeeping_core.v`)
虽然 RTL 目录中有同名文件，但在仿真中使用了 `sim/` 目录下的行为级模型，以简化相位误差的测量逻辑，专注于验证补偿算法的正确性。

## 3. 关键算法与逻辑

### 3.1 晶振漂移模拟
仿真通过 `osc_delta` 变量引入每秒的周期偏差。例如，若 `osc_delta = 1`，则表示 10MHz 时钟每秒多走 1 个周期（即 +100ns/s 漂移）。

### 3.2 闭环补偿验证
DUT (Device Under Test) 输出的 `comp_step` (补偿步进) 被回环连接到仿真环境的 `comp_period` 计算中：
```verilog
next_period = SIM_SEC_CYCLES + osc_delta - comp_step; // 简化示意
```
如果 DUT 的补偿算法正确，`comp_step` 应逐渐逼近 `osc_delta`，使得 `next_period` 回归到标准值，从而消除累积误差。

### 3.3 数据存储与回放
- **存储**：当 `gps_enable` 拉低模拟 GPS 丢失时，TB 产生 `collect_stop` 信号，触发 RTL 中的 `flush_req`，将 FIFO 中的差值数据写入 Flash 模型。
- **回放**：随后复位或进入回放模式，RTL 从 Flash 模型读取数据，`avg_remain_comp` 模块计算平均值并输出 `comp_step`。

## 4. 仿真测试用例设计

测试流程分为三个主要阶段：

### 阶段 1：学习/采集 (Learning Phase)
- **条件**：`gps_enable = 1`。
- **行为**：仿真注入固定的 `osc_delta` (如 +1 周期/秒)。DUT 测量 GPS PPS 与本地 PPS 的差值，并将数据存入 FIFO。
- **验证**：检查 FIFO 是否非空，差值数据是否与注入的漂移量一致。

### 阶段 2：GPS 丢失与存储 (Flush Phase)
- **条件**：`gps_enable` 拉低。
- **行为**：触发 `collect_stop`。
- **验证**：
  - 监测 SPI 总线活动，确认 Flash 写操作 (WREN -> SE -> PP) 发生。
  - 等待 `flash_done` 信号。
  - **数据完整性校验**：比对 Flash 模型特定地址 (`0x1000` 起始) 的数据与仿真预期的 `expected_diff` 是否一致。

### 阶段 3：守时/补偿 (Holdover Phase)
- **条件**：GPS 保持关闭，系统复位或进入守时模式。
- **行为**：DUT 从 Flash 读取数据，启动开环补偿。
- **验证**：
  - 记录 `err_comp_ns` (补偿后的相位误差)。
  - 统计 `max_abs_comp_ns`。
  - 若 `max_abs_comp_ns` < 300ns (或其他设定阈值)，则测试通过。

## 5. 仿真结果预期输出

### 5.1 控制台日志 (Log)
成功运行的仿真应输出类似以下信息：
```text
DBG: gps off, fifo_empty=0 ...
[Time] PHASE: Flash Write Done
[Time] COMP_UPDATE: osc_delta=1 comp_step=1 ...
PASSED: Max Abs Comp Error = 150 ns < Threshold
```

### 5.2 性能报告 (`sim/csv/tb_top_report.csv`)
CSV 文件包含秒级的误差数据，可用于绘图分析：
| sec | phase_err_uncomp_ns | phase_err_comp_ns | osc_delta | comp_step |
|-----|---------------------|-------------------|-----------|-----------|
| 0   | 100                 | 5                 | 1         | 1         |
| 1   | 200                 | 10                | 1         | 1         |
...
- **uncomp_ns**: 未补偿误差，应随时间线性增加。
- **comp_ns**: 补偿后误差，应保持在 0 附近波动，不随时间发散。

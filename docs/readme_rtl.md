# 自主守时系统 RTL 设计文档

## 1. 概述
本设计实现了一个基于 EF2M45 FPGA 的高精度自主守时系统。系统在 GPS 信号正常时，实时测量并学习本地晶振相对于 GPS 的频率漂移特性；在 GPS 信号丢失时，自动切换至守时模式，利用历史学习数据进行频率补偿，维持本地秒脉冲 (DPPS) 的高精度输出。

## 2. 系统整体架构

### 2.1 核心子系统
系统主要由以下核心子系统组成：
1.  **信号采集与预处理模块**
2.  **差值计算与存储控制模块**
3.  **动态补偿与守时输出模块**
4.  **核心辅助功能模块**

### 2.2 模块层次结构
```
top.v (顶层模块)
├── timekeeping_core.v (守时核心)
├── fifo_write_module.v (FIFO写模块)
├── fifo_read_module.v (FIFO读模块)
├── flash_controller_enhanced.v (增强版Flash控制器)
├── long_period_counter.v (长周期计数与差值运算模块)
├── avg_remain_comp.v (均匀补偿模块)
├── uart.v (UART通信模块)
└── sec_tick.v (秒脉冲生成)
```

## 3. 模块间接口定义

### 3.1 顶层模块 (`top.v`)

**输入接口**:
| 信号名称 | 方向 | 位宽 | 功能描述 |
|---------|------|------|----------|
| `clk` | 输入 | 1 | 50MHz系统时钟 |
| `rst_n` | 输入 | 1 | 系统复位 (低有效) |
| `gps_pps` | 输入 | 1 | GPS秒脉冲输入 |
| `spi_miso` | 输入 | 1 | Flash SPI MISO |
| `uart_rx` | 输入 | 1 | UART接收 |
| `dump_key` | 输入 | 1 | 数据导出按键(P52, 低有效) |
| `erase_key` | 输入 | 1 | Flash擦除按键(P54, 低有效) |
| `dyn_key` | 输入 | 1 | 动态参数读取按键(P61, 低有效) |

**输出接口**:
| 信号名称 | 方向 | 位宽 | 功能描述 |
|---------|------|------|----------|
| `uart_tx` | 输出 | 1 | UART发送 |
| `dpps_out` | 输出 | 1 | 补偿后的DPPS输出 |
| `spi_sclk` | 输出 | 1 | Flash SPI时钟 |
| `spi_mosi` | 输出 | 1 | Flash SPI MOSI |
| `spi_cs_n` | 输出 | 1 | Flash SPI片选 (低有效) |
| `uart_led` | 输出 | 1 | 串口传输状态指示灯 |
| `gps_led` | 输出 | 1 | GPS PPS信号接收指示灯 |

**内部关键信号**:
| 信号名称 | 位宽 | 功能描述 |
|---------|------|----------|
| `gps_lost` | 1 | GPS丢失标志（1=丢失） |
| `gps_stable` | 1 | GPS稳定标志 |
| `dynamic_comp_ready` | 1 | 动态补偿就绪标志 |
| `comp_param_locked` | 1 | 参数锁定标志 |
| `comp_param_fixed` | 32 | 锁定的补偿参数 |
| `dump_active` | 1 | Dump模式激活标志 |
| `powerup_ready` | 1 | 上电就绪标志 |

### 3.2 守时核心 (`timekeeping_core.v`)

**输入接口**:
| 信号名称 | 方向 | 位宽 | 功能描述 |
|---------|------|------|----------|
| `clk` | 输入 | 1 | 50MHz系统时钟 |
| `sys_rst_n` | 输入 | 1 | 系统复位 (低有效) |
| `gps_pps` | 输入 | 1 | GPS秒脉冲输入 |
| `dpps_offset` | 输入 | 24 | 补偿偏移值 |
| `init_diff` | 输入 | 8 | 初始差值 |
| `init_diff_valid` | 输入 | 1 | 初始差值有效标志 |

**输出接口**:
| 信号名称 | 方向 | 位宽 | 功能描述 |
|---------|------|------|----------|
| `dpps` | 输出 | 1 | 本地DPPS输出 |
| `diff_result` | 输出 | 32 | 差值结果（32位有符号） |
| `diff_valid_flag` | 输出 | 1 | 差值有效标志 |
| `pps_cnt_8` | 输出 | 4 | PPS计数器（0-15） |
| `gps_pps_posedge` | 输出 | 1 | GPS PPS上升沿 |
| `dbg_diff_valid` | 输出 | 1 | 调试用差值有效标志 |
| `gps_phase_sum` | 输出 | 32 | GPS累积计数 |
| `dpps_phase_sum` | 输出 | 32 | DPPS累积计数 |

### 3.3 长周期计数与差值运算模块 (`long_period_counter.v`)

**输入接口**:
| 信号名称 | 方向 | 位宽 | 功能描述 |
|---------|------|------|----------|
| `clk` | 输入 | 1 | 50MHz系统时钟 |
| `sys_rst_n` | 输入 | 1 | 系统复位 (低有效) |
| `gps_pps` | 输入 | 1 | GPS秒脉冲输入 |
| `osc_ready` | 输入 | 1 | 晶振就绪标志 |

**输出接口**:
| 信号名称 | 方向 | 位宽 | 功能描述 |
|---------|------|------|----------|
| `period_number` | 输出 | 16 | 周期编号 |
| `diff_total` | 输出 | 64 | 总差值 |
| `quotient` | 输出 | 64 | 商 |
| `remainder` | 输出 | 64 | 余数 |
| `diff_valid` | 输出 | 1 | 差值有效标志 |
| `period_done` | 输出 | 1 | 周期完成标志 |

### 3.4 增强版Flash控制器 (`flash_controller_enhanced.v`)

**输入接口**:
| 信号名称 | 方向 | 位宽 | 功能描述 |
|---------|------|------|----------|
| `sys_clk` | 输入 | 1 | 系统时钟 |
| `sys_rst_n` | 输入 | 1 | 系统复位 (低有效) |
| `collect_stop` | 输入 | 1 | 停止采集/触发写入 |
| `erase_req` | 输入 | 1 | 擦除请求 |
| `fifo_re` | 输入 | 1 | FIFO读使能 |
| `fifo_dout` | 输入 | 32 | FIFO数据输出 |
| `fifo_empty_flag` | 输入 | 1 | FIFO空标志 |
| `fifo_full_flag` | 输入 | 1 | FIFO满标志 |
| `spi_miso` | 输入 | 1 | SPI MISO |
| `param_data` | 输入 | 8 | 参数数据 |
| `param_addr` | 输入 | 24 | 参数地址 |
| `param_valid` | 输入 | 1 | 参数有效标志 |

**输出接口**:
| 信号名称 | 方向 | 位宽 | 功能描述 |
|---------|------|------|----------|
| `spi_sclk` | 输出 | 1 | SPI时钟 |
| `spi_mosi` | 输出 | 1 | SPI MOSI |
| `spi_cs_n` | 输出 | 1 | SPI片选 (低有效) |
| `flash_busy` | 输出 | 1 | Flash忙标志 |
| `flash_done` | 输出 | 1 | Flash操作完成标志 |
| `write_done` | 输出 | 1 | 仅写入完成（不含擦除） |
| `flash_error` | 输出 | 1 | Flash操作错误标志 |
| `hist_diff` | 输出 | 8 | 历史差值 |
| `hist_diff_valid` | 输出 | 1 | 历史差值有效标志 |
| `last_prog_word_mirror` | 输出 | 32 | 最后编程字镜像 |
| `rd_req` | 输出 | 1 | 读请求 |
| `rd_addr` | 输出 | 24 | 读地址 |
| `rd_data` | 输入 | 8 | 读数据 |
| `rd_data_valid` | 输入 | 1 | 读数据有效标志 |
| `cur_addr_dbg` | 输出 | 23 | 当前地址调试 |
| `state_dbg` | 输出 | 7 | 状态机调试 |

### 3.5 UART通信模块 (`uart.v`)

**输入接口**:
| 信号名称 | 方向 | 位宽 | 功能描述 |
|---------|------|------|----------|
| `sys_clk` | 输入 | 1 | 50MHz系统时钟 |
| `sys_rst_n` | 输入 | 1 | 系统复位 |
| `uart_rx` | 输入 | 1 | UART接收 |
| `sys_state` | 输入 | 4 | 状态机状态 |
| `pps_valid` | 输入 | 1 | GPS PPS有效标志 |
| `train_done` | 输入 | 1 | 训练完成标志 |
| `period_diff_pulse` | 输入 | 1 | 16s周期脉冲 |
| `diff_result` | 输入 | 32 | 差值结果 |
| `erase_start_pulse` | 输入 | 1 | 擦除开始脉冲 |
| `erase_done_pulse` | 输入 | 1 | 擦除完成脉冲 |
| `read_start_pulse` | 输入 | 1 | 读取开始脉冲 |
| `flash_busy` | 输入 | 1 | Flash忙标志 |
| `flush_active` | 输入 | 1 | FIFO正在写入Flash标志 |
| `read_diff` | 输入 | 32 | Flash读出的差值 |
| `read_diff_valid` | 输入 | 1 | Flash读出差值有效 |
| `dpps_offset_dbg` | 输入 | 23 | 当前补偿偏移量 |
| `flash_state` | 输入 | 7 | Flash状态机当前状态 |
| `check_fifo_cnt` | 输入 | 8 | 检查FIFO次数 |
| `fifo_re_done_cnt` | 输入 | 8 | FIFO读取完成次数 |
| `fifo_latch_done_cnt` | 输入 | 8 | FIFO锁存完成次数 |
| `fifo_prog_cnt` | 输入 | 16 | FIFO实际写入Flash字数 |
| `last_prog_word` | 输入 | 32 | 最后写入Flash的字 |
| `dbg_latches` | 输入 | 4 | 调试锁存器 |
| `dump_read_start_addr` | 输入 | 23 | Dump读取起始地址 |
| `dump_start_lw` | 输入 | 32 | Dump开始时的last_prog_word |
| `dump_mode` | 输入 | 1 | Dump模式 |
| `dump_word_valid` | 输入 | 1 | Dump字有效 |
| `dump_word_addr` | 输入 | 23 | Dump字地址 |
| `dump_word_data` | 输入 | 32 | Dump字数据 |
| `dump_done` | 输入 | 1 | Dump完成 |

**输出接口**:
| 信号名称 | 方向 | 位宽 | 功能描述 |
|---------|------|------|----------|
| `uart_tx` | 输出 | 1 | UART发送 |
| `state_cmd` | 输出 | 4 | 状态切换指令 |
| `cmd_valid` | 输出 | 1 | 命令有效脉冲 |
| `tx_en` | 输出 | 1 | UART发送使能 |
| `tx_done` | 输出 | 1 | UART发送完成 |
| `erase_cmd_pulse` | 输出 | 1 | 擦除命令脉冲 |
| `dump_word_ready` | 输出 | 1 | Dump字就绪 |
| `dump_meta_ready` | 输出 | 1 | Dump元数据就绪 |

## 4. 系统状态机

### 4.1 顶层状态机 (`top.v`)

| 状态码 | 状态名 | 描述 | 跳转条件 |
| :--- | :--- | :--- | :--- |
| `00000` | `ST_STANDBY` | 待机 | 上电默认，等待powerup_ready |
| `00010` | `ST_CALC` | 计算差值 | GPS有效 |
| `00011` | `ST_TRAIN` | 训练模式 | GPS丢失或串口指令或dump模式 |
| `00100` | `ST_TRAIN_DONE` | 训练完成 | read_done=1 |
| `00101` | `ST_PARAM_UPDATE` | 参数更新 | period_done=1 |
| `00110` | `ST_ERR` | 错误状态 | 异常情况 |

**状态转换流程**:
1. 上电 → ST_STANDBY
2. ST_STANDBY → ST_CALC (powerup_ready=1且GPS有效)
3. ST_CALC → ST_PARAM_UPDATE (period_done=1)
4. ST_PARAM_UPDATE → ST_CALC (参数更新完成)
5. ST_CALC → ST_TRAIN (GPS丢失)
6. ST_TRAIN → ST_TRAIN_DONE (read_done=1)
7. ST_TRAIN_DONE → ST_CALC (GPS恢复)

### 4.2 动态参数学习流程

1. **稳定阶段** (前10个16秒窗口):
   - GPS稳定30秒后开始
   - 仅计算差值，不训练
   - comp_settle_cnt从0计数到9

2. **训练阶段** (后10个16秒窗口):
   - 累加10个差值
   - 使用乘以3277右移15位的方法近似除以10
   - 计算平均值并锁定参数

3. **存储阶段**:
   - 参数锁定后自动存储到Flash
   - 存储地址: 0x3E0000
   - 存储4字节有符号参数

## 5. 关键参数与配置

| 参数名 | 值 | 说明 | 所在文件 |
|--------|-----|------|----------|
| `CLK_CYCLES_PER_SEC` | 50,000,000 | 系统时钟频率（50MHz） | top.v |
| `GPS_LOST_THRESHOLD` | 100,000,000 | GPS丢失检测阈值（2秒） | top.v |
| `POWERUP_HOLD_CYCLES` | 500,000,000 | 上电保持时间（10秒） | top.v |
| `GPS_STABLE_SEC` | 30 | GPS稳定等待时间（秒） | top.v |
| `DYN_PARAM_BASE_ADDR` | 0x3E0000 | 动态参数存储起始地址 | top.v |
| `Flash_DATA_BASE_ADDR` | 0x001000 | Flash数据存储起始地址 | top.v |
| `DUMP_WORD_LIMIT` | 5000 | Dump导出数据条数上限 | top.v |
| `DUMP_KEY_COOLDOWN_CYC` | 50,000,000 | Dump按键冷却时间（1秒） | top.v |

## 6. 技术实现要点

### 6.1 时钟域管理
- 系统统一使用50MHz时钟
- 所有模块均在同一时钟域下工作，简化时序设计
- GPS PPS信号采用三级寄存器同步，消除亚稳态

### 6.2 信号完整性
- GPS PPS信号采用三级寄存器同步+去抖动
- SPI通信采用标准时序，确保数据可靠传输
- UART接收采用状态机解析，支持不同终止符格式

### 6.3 数据可靠性
- Flash操作采用扇区擦除+页编程策略
- 动态参数使用4字节存储，支持从Flash恢复
- 关键操作有完成标志和错误标志

### 6.4 动态参数学习算法
- 使用10个16秒窗口的差值求平均
- 避免使用硬件除法器，使用近似算法：乘以3277右移15位（3277/32768 ≈ 0.1000）
- 参数锁定后自动存储到Flash

### 6.5 数据导出机制
- 按键触发后先flush FIFO到Flash
- 从地址0x001000开始逐条读取
- 每条数据包含地址、数据、Flash状态等信息
- 导出完成后发送结束标志

### 6.6 可测试性
- 三个按键控制：导出、擦除、动态参数读取
- 丰富的UART心跳包，包含大量调试信息
- 模块化设计便于单独测试
- 支持从Flash读取历史参数

## 7. 接口时序要求

### 7.1 SPI Flash接口
- 时钟频率：由系统时钟分频
- 数据采样：上升沿采样
- 片选信号：低有效，操作期间保持低电平

### 7.2 UART接口
- 波特率：由IP核配置
- 数据格式：8N1（8位数据，无校验，1位停止位）
- 指令格式：ASCII字符串，以\r\n或\n结尾

### 7.3 GPS PPS接口
- 信号类型：LVCMOS25
- 脉冲宽度：≥100ms
- 上升沿时间：≤10ns

## 8. 设计约束与限制

### 8.1 硬件约束
- 系统时钟：50MHz
- 工作电压：2.5V ± 0.1V
- 温度范围：0°C ~ 70°C

### 8.2 软件约束
- Flash擦写次数：≤100,000次
- 数据存储容量：取决于Flash芯片
- 补偿精度：取决于晶振特性

### 8.3 性能限制
- 差值计算：16秒一次
- Flash读写速度：取决于SPI时钟
- UART通信：波特率由IP核决定

## 9. 版本历史

| 版本 | 日期 | 变更内容 | 作者 |
|------|------|----------|------|
| v1.0 | 2026-01-22 | 初始版本，基础功能实现 | FPGA开发团队 |
| v2.0 | 2026-04-15 | 系统时钟改为50MHz，增加动态参数学习、数据导出等功能 | FPGA开发团队 |

## 10. 总结

本RTL设计文档详细描述了自主守时系统的模块间接口、交互流程和实现细节。系统采用模块化设计，通过守时核心、动态参数学习、Flash存储、数据导出等核心模块，实现了高精度的自主守时功能。

关键技术亮点包括：
- 50MHz系统时钟
- 16秒周期差值计算
- 动态参数学习与锁定（20个窗口）
- 完善的Flash存储机制（单字节读写支持）
- 丰富的UART调试信息（心跳包）
- 数据导出功能（Dump）
- 按键控制（导出、擦除、动态参数读取）
- GPS信号处理（去抖动、丢失检测）

系统设计考虑了可测试性、可靠性和性能优化，为实际应用提供了坚实的基础。

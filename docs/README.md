# 自主守时系统

## 1. 项目概述

本项目实现了一个基于 EF2M45 FPGA 的高精度自主守时系统。系统在 GPS 信号正常时，实时测量并学习本地晶振相对于 GPS 的频率漂移特性；在 GPS 信号丢失时，自动切换至守时模式，利用历史学习数据进行频率补偿，维持本地秒脉冲 (DPPS) 的高精度输出。

**主要特点**:
- 50MHz 系统时钟
- 16秒周期差值计算
- 动态参数学习与存储
- Flash 数据持久化
- 丰富的串口调试信息
- 支持数据导出功能

## 2. 项目结构

```
selftimekeep/
├── RTL/                    # RTL 源代码
│   ├── top.v              # 顶层模块（唯一顶层）
│   ├── timekeeping_core.v # 守时核心
│   ├── fifo_write_module.v # FIFO 写模块
│   ├── fifo_read_module.v # FIFO 读模块
│   ├── flash_controller_enhanced.v # 增强版Flash控制器
│   ├── long_period_counter.v # 长周期计数与差值运算模块
│   ├── avg_remain_comp.v # 均匀补偿模块
│   ├── uart.v             # UART 通信模块
│   └── sec_tick.v         # 秒脉冲生成
├── al_ip/                 # IP 核文件
│   ├── SPI_0/
│   ├── UART_0/
│   ├── FIFO_0/
│   └── FIFO_write/
├── docs/                  # 文档
│   ├── README.md          # 本文档
│   ├── readme_rtl.md      # RTL 设计详细文档
│   └── readme_sm.md      # 仿真设计详细文档
└── logs/                 # 日志文件
```

## 3. 已实现功能模块说明

### 3.1 顶层模块 (`top.v`)

**功能说明**:
- 系统主控制器，整合所有子模块
- 实现系统主状态机，管理工作模式切换
- GPS 信号丢失检测机制
- 动态参数学习与存储
- 数据导出功能支持

**涉及进程**:
1. **GPS PPS 同步与防抖进程**
   - 使用三级寄存器同步 GPS 输入信号
   - 消除亚稳态，提高系统可靠性
   - 上升沿超时检测GPS丢失

2. **系统主状态机进程**
   - 管理 6 种工作状态：STANDBY、CALC、TRAIN、TRAIN_DONE、PARAM_UPDATE、ERR
   - 串口指令优先级高于自动状态跳转

3. **动态参数学习进程**
   - GPS稳定后先等待10个16秒窗口（仅稳定，不训练）
   - 再用后10个窗口求平均并锁定参数
   - 支持参数存储到Flash和从Flash读取

4. **数据导出控制进程**
   - 支持按键触发数据导出
   - 自动flush FIFO数据到Flash
   - 逐条读取并串口发送

**实现的具体功能**:
- ✅ 6状态主状态机
- ✅ GPS信号有效性检测（2秒阈值）
- ✅ UART指令控制接口
- ✅ 模块互联与信号路由
- ✅ 动态参数学习与锁定
- ✅ 参数Flash存储与读取
- ✅ 按键控制（导出、擦除、动态参数读取）

### 3.2 守时核心 (`timekeeping_core.v`)

**功能说明**:
- 生成本地秒脉冲 (DPPS)
- 测量 GPS PPS 与本地 DPPS 的相位差
- 接收补偿值并调整本地时钟

**实现的具体功能**:
- ✅ 本地 DPPS 生成
- ✅ 相位差测量（32位有符号）
- ✅ 16秒统计周期
- ✅ 补偿值接收与应用

### 3.3 FIFO 写模块 (`fifo_write_module.v`)

**功能说明**:
- 实时缓存相位差数据
- 支持强制刷写功能，在 GPS 丢失时确保数据不丢失

**实现的具体功能**:
- ✅ 差值数据缓存
- ✅ FIFO 状态输出（空/满/正常）
- ✅ 强制刷写机制
- ✅ 数据完整性保护

### 3.4 增强版Flash控制器 (`flash_controller_enhanced.v`)

**功能说明**:
- 管理 SPI Flash 的擦写操作
- 将 FIFO 数据写入 Flash
- 支持单字节读写操作
- 支持动态参数存储

**实现的具体功能**:
- ✅ SPI Flash 读写操作
- ✅ 扇区擦除（4KB）
- ✅ 页编程（256B）
- ✅ 状态寄存器读取
- ✅ 单字节读写支持
- ✅ 动态参数存储区域

### 3.5 FIFO 读模块 (`fifo_read_module.v`)

**功能说明**:
- 从 Flash 读取历史数据
- 将 Flash 串行数据流转换为并行数据
- 支持 Dump 模式数据导出

**实现的具体功能**:
- ✅ SPI Flash 读操作
- ✅ 数据串并转换
- ✅ 读取启动控制
- ✅ 读取完成标志
- ✅ Dump模式支持

### 3.6 均匀补偿模块 (`avg_remain_comp.v`)

**功能说明**:
- 计算历史数据的平均频率漂移
- 采用积分控制算法
- 输出动态调节的补偿步进

**实现的具体功能**:
- ✅ 历史数据平均计算
- ✅ 积分控制算法
- ✅ 补偿步进输出
- ✅ 秒脉冲同步更新

### 3.7 UART 模块 (`uart.v`)

**功能说明**:
- 实例化 UART IP 核
- 接收上位机指令
- 回传系统状态（每秒心跳包）
- 支持 Dump 数据导出

**支持的指令**:
- `CALC_DIFF\r\n` - 切换到计算状态
- `TRAIN_DPPS\r\n` - 切换到训练状态
- `QUERY_STATE\r\n` - 查询系统状态
- `ERASE\r\n` - 触发Flash擦除

**心跳包格式 (每16秒发送一次)**:
```
DF:±xxxxxxxx,M:L/H,DP:±xxxxxx,FS:xx,CF:xx,FR:xx,FL:xx,LW:xxxxxxxx,DL:x,SL:xxxxxxxx,FI:x,FC:xxxx,FT:xxxx,FP:x
```

**字段说明**:
- `DF`: 差值数据（32位有符号）
- `M`: 模式（L=锁定GPS，H=守时模式）
- `DP`: DPPS补偿偏移量
- `FS`: Flash状态机状态
- `CF`: 检查FIFO次数
- `FR`: FIFO读取完成次数
- `FL`: FIFO锁存完成次数
- `LW`: 最后写入Flash的字
- `DL`: 调试锁存器状态
- `SL`: Dump起始时的last_prog_word
- `FI`: Dump起始时FIFO状态
- `FC`: Dump起始时FIFO总计数
- `FT`: FIFO实际写入Flash字数
- `FP`: Dump开始时FIFO是否为空

**Dump数据格式**:
- `READ START,DL:x,RSA:xxxxxx,RSL:xxxxxxxx` - 读取开始
- `ADDR=xxxxxx,DATA=xxxxxxxx,FL:x,FW:x,LW:xxxxxxxx` - 数据条目
- `E` - 导出结束

**实现的具体功能**:
- ✅ UART 通信
- ✅ 多条指令解析
- ✅ 系统状态回传（心跳包）
- ✅ 指令终止符检测（\r\n 或 \n）
- ✅ Dump数据导出
- ✅ 按键事件通知

## 4. 系统工作流程

### 4.1 系统启动流程
1. 上电复位
2. 等待10秒上电保持时间
3. 进入ST_STANDBY状态
4. 检测GPS信号

### 4.2 GPS正常时的工作流程
1. 检测到GPS信号，进入ST_CALC状态
2. GPS稳定30秒后开始差值计算
3. 前10个16秒窗口：仅稳定，不训练
4. 后10个16秒窗口：求平均并锁定参数
5. 参数锁定后自动存储到Flash
6. 每16秒计算一次差值并写入FIFO
7. FIFO积累到一定数量后批量写入Flash

### 4.3 GPS丢失时的守时流程
1. 2秒无GPS信号判定为丢失
2. 进入ST_TRAIN状态
3. 从Flash读取历史参数
4. 使用历史参数进行补偿
5. 继续输出DPPS信号

### 4.4 数据导出流程（按下P52按键）
1. 触发强制flush，将FIFO数据写入Flash
2. 等待Flash写入完成
3. 从Flash地址0x001000开始读取数据
4. 逐条通过串口发送
5. 发送完成后退出导出模式

### 4.5 动态参数读取流程（按下P61按键）
1. 从Flash动态参数区读取参数
2. 如果有有效参数，加载并锁定
3. 如果无有效参数，继续等待学习

### 4.6 Flash擦除流程（按下P54按键）
1. 触发Flash擦除操作
2. 擦除完成后重置所有计数器
3. 重新开始参数学习

## 5. 系统关键参数

| 参数名 | 值 | 说明 |
|--------|-----|------|
| CLK_CYCLES_PER_SEC | 50,000,000 | 系统时钟频率（50MHz） |
| GPS_LOST_THRESHOLD | 100,000,000 | GPS 丢失检测阈值（2秒） |
| POWERUP_HOLD_CYCLES | 500,000,000 | 上电保持时间（10秒） |
| GPS_STABLE_SEC | 30 | GPS稳定等待时间（秒） |
| DYN_PARAM_BASE_ADDR | 0x3E0000 | 动态参数存储起始地址 |
| Flash_SECTOR_SIZE | 4096 | Flash 扇区大小（4KB） |
| Flash_PAGE_SIZE | 256 | Flash 页大小（256B） |
| Flash_DATA_BASE_ADDR | 0x001000 | Flash 数据存储起始地址 |
| DUMP_WORD_LIMIT | 5000 | Dump导出数据条数上限 |

## 6. 硬件连接说明

### 6.1 引脚连接映射表

| 信号名称 | 物理引脚 | 方向 | 功能描述 |
|---------|---------|------|----------|
| **clk** | P21 | 输入 | 系统时钟（50MHz） |
| **dpps_out** | P1 | 输出 | 补偿后的DPPS输出 |
| **gps_pps** | P2 | 输入 | GPS标准PPS信号 |
| **rst_n** | P3 | 输入 | 系统复位（低有效） |
| **dump_key** | P52 | 输入 | 数据导出按键（低有效） |
| **erase_key** | P54 | 输入 | Flash擦除按键（低有效） |
| **dyn_key** | P61 | 输入 | 动态参数读取按键（低有效） |
| **spi_cs_n** | P5 | 输出 | Flash SPI片选（低有效） |
| **spi_miso** | P6 | 输入 | Flash SPI MISO |
| **spi_mosi** | P11 | 输出 | Flash SPI MOSI |
| **spi_sclk** | P12 | 输出 | Flash SPI时钟 |
| **uart_rx** | P13 | 输入 | UART接收 |
| **uart_tx** | P15 | 输出 | UART发送 |
| **uart_led** | - | 输出 | 串口传输状态指示灯 |
| **gps_led** | - | 输出 | GPS PPS信号接收指示灯 |

### 6.2 按键说明

- **P52 (dump_key)**: 数据导出按键，按下后触发Flash数据导出
- **P54 (erase_key)**: Flash擦除按键，按下后擦除Flash并重置系统
- **P61 (dyn_key)**: 动态参数读取按键，按下后从Flash读取已保存的参数

## 7. 串口回传信息详解

### 7.1 心跳包（每16秒一次）

格式：
```
DF:±xxxxxxxx,M:L/H,DP:±xxxxxx,FS:xx,CF:xx,FR:xx,FL:xx,LW:xxxxxxxx,DL:x,SL:xxxxxxxx,FI:x,FC:xxxx,FT:xxxx,FP:x
```

示例：
```
DF:-00000A20,M:L,DP:+000000,FS:1D,CF:00,FR:00,FL:00,LW:FFFFF5E0,DL:0,SL:00000000,FI:00,FC:0000,FT:0000,FP:0
```

### 7.2 Dump数据导出

1. 读取开始信息：
```
READ START,DL:x,RSA:xxxxxx,RSL:xxxxxxxx
```

2. 数据条目（多条）：
```
ADDR=xxxxxx,DATA=xxxxxxxx,FL:x,FW:x,LW:xxxxxxxx
```

3. 导出结束：
```
E
```

### 7.3 事件通知

- `ERASE START` - Flash擦除开始
- `ERASE DONE` - Flash擦除完成
- `DYN_STORE_START` - 动态参数存储开始
- `DYN_STORE_DONE` - 动态参数存储完成
- `DYN_STORE_FAIL` - 动态参数存储失败

## 8. 文档索引

- **[RTL 设计详细文档 (readme_rtl.md)](./readme_rtl.md)**
  - 系统架构详细说明
  - 各模块接口定义
  - 状态机逻辑详解
  - UART 交互协议
  - 技术规格说明

## 9. 开发进度

### 已完成（✅）
- ✅ 系统整体框架搭建（50MHz时钟）
- ✅ 顶层模块与状态机
- ✅ 守时核心模块（16秒周期）
- ✅ FIFO 读写模块
- ✅ 增强版Flash控制器（单字节读写）
- ✅ 均匀补偿模块
- ✅ UART 通信模块（丰富的心跳包）
- ✅ 动态参数学习与锁定
- ✅ 参数Flash存储与读取
- ✅ 数据导出功能（Dump）
- ✅ 按键控制（导出、擦除、动态参数读取）
- ✅ GPS信号处理（去抖动、丢失检测）

### 待完成（⏳）
- ⏳ 温度传感器接口集成
- ⏳ 电压传感器接口集成
- ⏳ 完整的性能测试
- ⏳ 长期稳定性测试

## 10. 维护建议

1. **代码风格**: 遵循现有代码风格，保持注释格式一致
2. **参数修改**: 修改关键参数时需同步更新文档和相关模块
3. **版本管理**: 建议使用 Git 进行版本控制
4. **文档更新**: 功能变更时及时更新相关文档

---

**最后更新时间**: 2026-04-15
**项目维护者**: FPGA 开发团队

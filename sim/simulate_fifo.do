# FIFO仿真专用脚本 (修复版)
# 用于验证timekeeping_core生成的差值数据通过FIFO的完整流程
# 修复内容：1.路径拼接错误(核心报错) 2.加长仿真运行时间 3.优化路径判断逻辑 4.适配新TB的空闲超时功能
# 优化内容：1.精准指定工程根目录 2.优化编译顺序 3.增强日志打印 4.适配Anlogic FIFO IP仿真

# >>> 核心修复1：手动指定【绝对正确的工程根目录】，彻底杜绝路径拼接错误 <<<
# 你的工程根目录，直接写死，最稳妥！！！
set PROJ_DIR "E:/fjlwork/selftimekeep"
set PROJ_DIR [file normalize $PROJ_DIR]

# FIFO IP文件路径 (绝对精准，无拼接，不会错)
set FIFO_IP_SIM [file join $PROJ_DIR al_ip FIFO_0_sim.v]
set FIFO_IP_RTL [file join $PROJ_DIR al_ip FIFO_0.v]

# 是否使用原厂IP仿真 (1=用Anlogic的FIFO_0.v，0=用自建模型)
if {![info exists ::USE_VENDOR_FIFO]} {
    set ::USE_VENDOR_FIFO 1
}
if {$::USE_VENDOR_FIFO} {
    if {[file exists $FIFO_IP_SIM]} {
        set FIFO_IP_FILE $FIFO_IP_SIM
    } else {
        set FIFO_IP_FILE $FIFO_IP_RTL
    }
} else {
    set FIFO_IP_FILE [file join $PROJ_DIR sim FIFO_0_model.v]
}

echo "=================================================="
echo "INFO: 工程根目录 PROJ_DIR = $PROJ_DIR"
echo "INFO: FIFO IP文件 FIFO_IP_FILE = $FIFO_IP_FILE"
echo "=================================================="
onerror {resume}

# Anlogic仿真库路径 (你的路径正确，保留)
if {[info exists ::ANLOGIC_SIM_LIB_ROOT] && $::ANLOGIC_SIM_LIB_ROOT ne ""} {
    set ANLOGIC_SIM_LIB_ROOT [file normalize $::ANLOGIC_SIM_LIB_ROOT]
} else {
    set ANLOGIC_SIM_LIB_ROOT E:/TD_6.2.1/sim_release
}

if {[string tolower [file tail $ANLOGIC_SIM_LIB_ROOT]] eq "ph1"} {
    set ANLOGIC_PH1_DIR $ANLOGIC_SIM_LIB_ROOT
    set ANLOGIC_SIM_LIB_ROOT [file dirname $ANLOGIC_SIM_LIB_ROOT]
} else {
    set ANLOGIC_PH1_DIR [file join $ANLOGIC_SIM_LIB_ROOT ph1]
}
echo "INFO: ANLOGIC_SIM_LIB_ROOT = $ANLOGIC_SIM_LIB_ROOT"
echo "INFO: ANLOGIC_PH1_DIR = $ANLOGIC_PH1_DIR"

# Anlogic库文件列表 (你的配置正确，保留)
if {$FIFO_IP_FILE eq $FIFO_IP_SIM} {
    set ANLOGIC_LIB_FILES [list \
        $ANLOGIC_PH1_DIR/ph1_phy_config_v2.sv \
        $ANLOGIC_PH1_DIR/ph1_phy_fifoctrl.v \
        $ANLOGIC_PH1_DIR/ph1_phy_eram.v \
        $ANLOGIC_PH1_DIR/ph1_phy_gsr.v \
    ]
} else {
    set ANLOGIC_LIB_FILES [list \
        $ANLOGIC_PH1_DIR/ph1_logic_fifo.v \
        $ANLOGIC_PH1_DIR/ph1_logic_eram.v \
        $ANLOGIC_PH1_DIR/ph1_logic_config_v2.v \
    ]
}

# 文件存在性检查
if {$::USE_VENDOR_FIFO} {
    foreach f $ANLOGIC_LIB_FILES {
        if {![file exists $f]} {
            echo "ERROR: 缺少 Anlogic 仿真库文件: $f"
            echo "ERROR: 当前 ANLOGIC_SIM_LIB_ROOT = $ANLOGIC_SIM_LIB_ROOT"
            return
        }
    }
}
if {![file exists $FIFO_IP_FILE]} {
    echo "ERROR: 缺少 FIFO IP 文件: $FIFO_IP_FILE"
    echo "提示: 请检查IP文件是否在 $PROJ_DIR/al_ip/ 目录下"
    return
}

# 1. 清理工作区 (保留)
cd $PROJ_DIR
set TRANSCRIPT_FILE [file join $PROJ_DIR sim modelsim_simulate_fifo.log]
catch {transcript file $TRANSCRIPT_FILE}
catch {transcript on}
catch {vdel -all}

# 2. 创建并映射工作区 (保留)
set WORK_LIB_DIR [file join $PROJ_DIR work]
catch {vmap -c}
vlib $WORK_LIB_DIR
vmap work $WORK_LIB_DIR
if {[catch {vmap work} WORK_MAP_INFO]} {
    echo "ERROR: work 库映射失败（vmap work 失败）"
    return
} else {
    echo "INFO: $WORK_MAP_INFO"
}
catch {vdel -lib work -all}

# 3. 编译Anlogic库源文件 (保留)
if {$::USE_VENDOR_FIFO} {
    foreach f $ANLOGIC_LIB_FILES { vlog -sv -work work $f }
}

# >>> 优化1：精准指定所有设计文件的路径，杜绝编译找不到文件 <<<
# 4. 编译设计文件 (顺序正确：IP→RTL→仿真文件→TB)
vlog -sv -work work [file join $PROJ_DIR sim glbl.v]
vlog -sv -work work $FIFO_IP_FILE
vlog -sv -work work [file join $PROJ_DIR RTL fifo.v]
vlog -sv -work work [file join $PROJ_DIR sim timekeeping_core.v]
vlog -sv -work work [file join $PROJ_DIR sim tb_fifo.v]

# 设计单元检查 (保留)
set REQUIRED_UNITS [list tb_fifo fifo timekeeping_core FIFO_0]
foreach u $REQUIRED_UNITS {
    if {[catch {vdir work.$u} VDIR_ERR]} {
        echo "ERROR: 设计单元未生成/不可见: work.$u"
        echo "ERROR: $VDIR_ERR"
        return
    }
}
if {$::USE_VENDOR_FIFO} {
    if {$FIFO_IP_FILE eq $FIFO_IP_SIM} {
        set REQUIRED_ANLOGIC_UNITS [list PH1_PHY_CONFIG_V2 PH1_PHY_FIFOCTRL PH1_PHY_ERAM PH1_PHY_GSR]
    } else {
        set REQUIRED_ANLOGIC_UNITS [list PH1_LOGIC_FIFO]
    }
    foreach u $REQUIRED_ANLOGIC_UNITS {
        if {[catch {vdir work.$u} VDIR_ERR]} {
            echo "ERROR: Anlogic 依赖模块缺失: work.$u"
            echo "ERROR: $VDIR_ERR"
            return
        }
    }
}

# 5. 加载测试模块到仿真器（使用非优化模式，确保信号可见）(保留)
catch {unalias vsim}
catch {unalias vopt}
catch {unalias vdel}
set USE_FALLBACK_NOVOPT 0
if {$::USE_VENDOR_FIFO && ($FIFO_IP_FILE eq $FIFO_IP_SIM)} {
    set VOPT_TOPS [list work.tb_fifo work.PH1_PHY_GSR]
} else {
    set VOPT_TOPS [list work.tb_fifo]
}
if {[catch {eval vopt +acc $VOPT_TOPS -o tb_fifo_opt} VOPT_ERR]} {
    echo "WARN: vopt 展开失败，准备回退到 vsim -novopt"
    echo "WARN: $VOPT_ERR"
    set USE_FALLBACK_NOVOPT 1
}
if {!$USE_FALLBACK_NOVOPT} {
    set VSIM_OPT_CMD [concat {vsim} tb_fifo_opt {-l} $TRANSCRIPT_FILE]
    if {[catch {eval $VSIM_OPT_CMD} VSIM_ERR]} {
        echo "WARN: vsim tb_fifo_opt 装载失败，准备回退到未优化装载"
        echo "WARN: $VSIM_ERR"
        set USE_FALLBACK_NOVOPT 1
    }
}
if {$USE_FALLBACK_NOVOPT} {
    if {$::USE_VENDOR_FIFO && ($FIFO_IP_FILE eq $FIFO_IP_SIM)} {
        set VSIM_TOPS [list work.tb_fifo work.PH1_PHY_GSR]
    } else {
        set VSIM_TOPS [list work.tb_fifo]
    }
    set VSIM_FALLBACK_CMD [concat {vsim} -novopt $VSIM_TOPS {-l} $TRANSCRIPT_FILE]
    if {[catch {eval $VSIM_FALLBACK_CMD} VSIM_ERR]} {
        echo "ERROR: vsim -novopt 装载失败"
        echo "ERROR: $VSIM_ERR"
        return
    }
}

# 6. 确保信号可见 (保留)
view structure
view signals

# >>> 优化2：新增观察FIFO内部的空闲计数/超时标志，重点验证新增功能 <<<
# 7. 添加波形观察信号（深度递归，包含所有内部信号）
add wave -noupdate -divider {===== TB顶层信号 =====}
add wave -r /tb_fifo/*
add wave -noupdate -divider {===== FIFO核心内部信号 (新增功能重点观察) =====}
add wave -r /tb_fifo/u_fifo/*

# 8. 设置波形显示格式 (保留+优化)
configure wave -namecolwidth 180
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -gridperiod 1000ns
configure wave -griddisplay on
configure wave -timelineunits ns

# >>> 核心修复2：加长仿真运行时间，适配tb_fifo的5ms测试时长 <<<
# 9. 运行仿真 (关键修改：10000ns → 5000000ns，刚好跑完TB的所有测试场景)
run 5000000ns

# >>> 优化3：增强验证结果打印，明确新增功能是否验证 <<<
# 10. 自动检查验证结果
if {[examine -radix decimal /tb_fifo/test_pass] == 1} {
    echo "\n=================================================="
    echo "🎉 FIFO功能验证 全部 PASSED! 🎉"
    echo "=================================================="
    echo "✅ 验证内容："
    echo "✅ 1. 有效差值写入功能"
    echo "✅ 2. 写满触发读功能"
    echo "✅ 3. 数据完整性验证"
    echo "✅ 4. 剩余数据读取验证"
    echo "✅ 5. 空闲计数递增功能 (新增)"
    echo "✅ 6. 空闲超时触发满标志 (新增)"
    echo "✅ 7. 计数清零逻辑验证 (新增)"
    echo "=================================================="
} else {
    echo "\n=================================================="
    echo "❌ FIFO功能验证 FAILED! ❌"
    echo "=================================================="
    echo "请检查仿真波形和 sim/modelsim_simulate_fifo.log 错误信息"
    echo "=================================================="
}

# 11. 保持仿真窗口打开，不自动退出
# 注意：没有添加quit -f命令，仿真结束后可以查看结果/波形
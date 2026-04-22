if {[info exists ::SELF_TIMEKEEP_PROJ_DIR] && $::SELF_TIMEKEEP_PROJ_DIR ne ""} {
    set PROJ_DIR [file normalize $::SELF_TIMEKEEP_PROJ_DIR]
} else {
    set BASE_DIR [pwd]
    if {[file exists [file join $BASE_DIR al_ip]] && [file exists [file join $BASE_DIR sim]]} {
        set PROJ_DIR [file normalize $BASE_DIR]
    } else {
        set PROJ_DIR [file normalize [file join $BASE_DIR selftimekeep]]
    }
}

echo "INFO: PROJ_DIR = $PROJ_DIR"
onerror {resume}

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

if {![info exists ::USE_VENDOR_FIFO]} {
    set ::USE_VENDOR_FIFO 1
}

set FIFO_IP_SIM [file join $PROJ_DIR al_ip FIFO_0_sim.v]
set FIFO_IP_RTL [file join $PROJ_DIR al_ip FIFO_0.v]
if {$::USE_VENDOR_FIFO} {
    if {[file exists $FIFO_IP_SIM]} {
        set FIFO_IP_FILE $FIFO_IP_SIM
    } else {
        set FIFO_IP_FILE $FIFO_IP_RTL
    }
} else {
    set FIFO_IP_FILE [file join $PROJ_DIR sim FIFO_0_model.v]
}

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
    return
}

cd $PROJ_DIR
set TS [clock format [clock seconds] -format "%Y%m%d_%H%M%S"]
set TRANSCRIPT_FILE [file join $PROJ_DIR sim "modelsim_run_fifo_${TS}.log"]
set WLF_FILE [file join $PROJ_DIR sim "modelsim_run_fifo_${TS}.wlf"]
catch {transcript file $TRANSCRIPT_FILE}
catch {transcript on}
echo "INFO: TRANSCRIPT_FILE = $TRANSCRIPT_FILE"
echo "INFO: WLF_FILE = $WLF_FILE"
catch {vdel -all}

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

if {$::USE_VENDOR_FIFO} {
    foreach f $ANLOGIC_LIB_FILES {
        if {[catch {vlog -sv -work work $f} MSG]} { echo "ERROR: $MSG"; return }
    }
}

if {[catch {vlog -sv -work work [file join $PROJ_DIR sim glbl.v]} MSG]} { echo "ERROR: $MSG"; return }
if {[catch {vlog -sv -work work $FIFO_IP_FILE} MSG]} { echo "ERROR: $MSG"; return }
if {[catch {vlog -sv -work work [file join $PROJ_DIR RTL fifo.v]} MSG]} { echo "ERROR: $MSG"; return }
if {[catch {vlog -sv -work work [file join $PROJ_DIR sim timekeeping_core.v]} MSG]} { echo "ERROR: $MSG"; return }
if {[catch {vlog -sv -work work [file join $PROJ_DIR sim tb_fifo.v]} MSG]} { echo "ERROR: $MSG"; return }

set REQUIRED_UNITS [list tb_fifo fifo timekeeping_core FIFO_0]
set WORK_LISTING ""
if {[catch {set WORK_LISTING [vdir work]} VDIR_WORK_ERR]} {
    if {[catch {set WORK_LISTING [vdir -lib work]} VDIR_WORK_ERR2]} {
        echo "ERROR: 无法访问 work 库内容"
        echo "ERROR: $VDIR_WORK_ERR"
        echo "ERROR: $VDIR_WORK_ERR2"
        return
    }
}
echo "INFO: work 库内容如下："
echo "$WORK_LISTING"
foreach u $REQUIRED_UNITS {
    if {![regexp -nocase "\\y${u}\\y" $WORK_LISTING]} {
        echo "ERROR: 设计单元未生成/不可见: work.$u"
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
        if {![regexp -nocase "\\y${u}\\y" $WORK_LISTING]} {
            echo "ERROR: Anlogic 依赖模块缺失: work.$u"
            return
        }
    }
}

catch {unalias vsim}
catch {unalias vopt}
catch {unalias vdel}

set VSIM_ALIAS_INFO ""
if {![catch {alias vsim} VSIM_ALIAS_INFO]} {
    echo "INFO: alias vsim = $VSIM_ALIAS_INFO"
}
set VOPT_ALIAS_INFO ""
if {![catch {alias vopt} VOPT_ALIAS_INFO]} {
    echo "INFO: alias vopt = $VOPT_ALIAS_INFO"
}
set USE_FALLBACK_NOVOPT 0
if {$::USE_VENDOR_FIFO && ($FIFO_IP_FILE eq $FIFO_IP_SIM)} {
    set VOPT_TOPS [list work.tb_fifo work.PH1_PHY_GSR]
} else {
    set VOPT_TOPS [list work.tb_fifo]
}
if {[catch {eval vopt +acc $VOPT_TOPS -o tb_fifo_opt} VOPT_ERR]} {
    echo "WARN: vopt 展开失败，准备回退到 vsim -novopt"
    echo "WARN: $VOPT_ERR"
    echo "WARNINFO: $errorInfo"
    echo "WARNLOG: $TRANSCRIPT_FILE"
    set USE_FALLBACK_NOVOPT 1
}
if {!$USE_FALLBACK_NOVOPT} {
    set VSIM_OPT_CMD [concat {vsim} -suppress 12110 tb_fifo_opt {-l} $TRANSCRIPT_FILE {-wlf} $WLF_FILE]
    if {[catch {eval $VSIM_OPT_CMD} VSIM_ERR]} {
        echo "WARN: vsim tb_fifo_opt 装载失败，准备回退到未优化装载"
        echo "WARN: $VSIM_ERR"
        echo "WARNINFO: $errorInfo"
        echo "WARNLOG: $TRANSCRIPT_FILE"
        set USE_FALLBACK_NOVOPT 1
    }
}
if {$USE_FALLBACK_NOVOPT} {
    if {$::USE_VENDOR_FIFO && ($FIFO_IP_FILE eq $FIFO_IP_SIM)} {
        set VSIM_TOPS [list work.tb_fifo work.PH1_PHY_GSR]
    } else {
        set VSIM_TOPS [list work.tb_fifo]
    }
    set VSIM_FALLBACK_CMD [concat {vsim} -novopt -suppress 12110 $VSIM_TOPS {-l} $TRANSCRIPT_FILE {-wlf} $WLF_FILE]
    if {[catch {eval $VSIM_FALLBACK_CMD} VSIM_ERR]} {
        echo "ERROR: vsim -novopt 装载失败"
        echo "ERROR: $VSIM_ERR"
        echo "ERRORINFO: $errorInfo"
        echo "ERRORLOG: $TRANSCRIPT_FILE"
        return
    }
}
catch {log -r /*}
catch {view wave}
catch {view structure}
catch {view signals}
catch {add wave -r /tb_fifo/*}
catch {add wave -r /tb_fifo/u_fifo/*}
catch {add wave -r /tb_fifo/u_timekeeping_core/*}
run -all

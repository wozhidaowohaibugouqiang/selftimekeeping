if {[info exists ::SELF_TIMEKEEP_PROJ_DIR] && $::SELF_TIMEKEEP_PROJ_DIR ne ""} {
    set PROJ_DIR [file normalize $::SELF_TIMEKEEP_PROJ_DIR]
} else {
    set BASE_DIR [pwd]
    if {[file exists [file join $BASE_DIR RTL]] && [file exists [file join $BASE_DIR sim]]} {
        set PROJ_DIR [file normalize $BASE_DIR]
    } else {
        if {[string tolower [file tail $BASE_DIR]] eq "sim" && [file exists [file join $BASE_DIR .. RTL]]} {
            set PROJ_DIR [file normalize [file join $BASE_DIR ..]]
        } else {
            set PROJ_DIR [file normalize [file join $BASE_DIR selftimekeep]]
        }
    }
}

set FIFO_IP_SIM [file join $PROJ_DIR al_ip FIFO_0_sim.v]
set FIFO_IP_RTL [file join $PROJ_DIR al_ip FIFO_0.v]
if {![info exists ::USE_VENDOR_FIFO]} {
    set ::USE_VENDOR_FIFO 1
}
if {![info exists ::PREFER_FIFO_IP_SIM]} {
    set ::PREFER_FIFO_IP_SIM 0
}
if {$::USE_VENDOR_FIFO} {
    if {$::PREFER_FIFO_IP_SIM && [file exists $FIFO_IP_SIM]} {
        set FIFO_IP_FILE $FIFO_IP_SIM
    } else {
        set FIFO_IP_FILE $FIFO_IP_RTL
    }
} else {
    set FIFO_IP_FILE [file join $PROJ_DIR sim FIFO_0_model.v]
}

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
            echo "ERROR: missing anlogic sim lib: $f"
            echo "ERROR: ANLOGIC_SIM_LIB_ROOT = $ANLOGIC_SIM_LIB_ROOT"
            return
        }
    }
}
if {![file exists $FIFO_IP_FILE]} {
    echo "ERROR: missing FIFO IP file: $FIFO_IP_FILE"
    return
}

set LOG_DIR [file join $PROJ_DIR sim log]
if {![file exists $LOG_DIR]} {
    file mkdir $LOG_DIR
}
set CSV_DIR [file join $PROJ_DIR sim csv]
if {![file exists $CSV_DIR]} {
    file mkdir $CSV_DIR
}

cd $PROJ_DIR
set TRANSCRIPT_FILE [file join $PROJ_DIR sim_debug_auto.log]
if {[file exists $TRANSCRIPT_FILE]} {
    catch {file delete -force $TRANSCRIPT_FILE}
}
transcript file $TRANSCRIPT_FILE
transcript on

catch {quit -sim}
catch {vdel -all}

set WORK_LIB_DIR [file join $PROJ_DIR work]
if {[file exists $WORK_LIB_DIR]} {
    catch {file delete -force $WORK_LIB_DIR}
}

catch {vmap -c}
vlib $WORK_LIB_DIR
vmap work $WORK_LIB_DIR


if {$::USE_VENDOR_FIFO} {
    foreach f $ANLOGIC_LIB_FILES { vlog -sv -work work $f }
}

vlog -sv -work work [file join $PROJ_DIR sim glbl.v]
vlog -sv -work work [file join $PROJ_DIR sim spi_flash_model.v]
vlog -sv -work work [file join $PROJ_DIR sim PLL_0_sim.v]
vlog -sv -work work $FIFO_IP_FILE
vlog -sv -work work [file join $PROJ_DIR al_ip FIFO_read.v]
vlog -sv -work work [file join $PROJ_DIR al_ip SPI_0 SPI_0.v]
vlog -sv -work work [file join $PROJ_DIR al_ip SPI_0 RTL SPI_Master_c06a3a2e2c87.v]
vlog -sv -work work [file join $PROJ_DIR al_ip UART_0 UART_0.v]
vlog -sv -work work [file join $PROJ_DIR al_ip UART_0 RTL uart_2bd8831e6282.v]
vlog -sv -work work [file join $PROJ_DIR RTL sec_tick.v]
vlog -sv -work work [file join $PROJ_DIR RTL uart.v]
vlog -sv -work work [file join $PROJ_DIR RTL spi_driver.v]
vlog -sv -work work [file join $PROJ_DIR RTL fifo_write_module.v]
vlog -sv -work work [file join $PROJ_DIR RTL fifo_read_module.v]
vlog -sv -work work [file join $PROJ_DIR RTL flash_controller.v]
vlog -sv -work work [file join $PROJ_DIR RTL flash_controller_enhanced.v]
vlog -sv -work work [file join $PROJ_DIR RTL avg_remain_comp.v]
vlog -sv -work work [file join $PROJ_DIR RTL timekeeping_core.v]
vlog -sv -work work [file join $PROJ_DIR RTL long_period_counter.v]
vlog -sv -work work [file join $PROJ_DIR RTL dynamic_comp_coeff.v]
vlog -sv -work work [file join $PROJ_DIR RTL aging_comp_coeff.v]
vlog -sv -work work [file join $PROJ_DIR RTL param_manager.v]
vlog -sv -work work [file join $PROJ_DIR top.v]
vlog -sv -work work [file join $PROJ_DIR sim tb_top.v]

catch {unalias vsim}
catch {unalias vopt}
catch {unalias vdel}

if {$::USE_VENDOR_FIFO && ($FIFO_IP_FILE eq $FIFO_IP_SIM)} {
    set SIM_TOPS [list work.tb_top work.PH1_PHY_GSR]
} else {
    set SIM_TOPS [list work.tb_top]
}

# Explicitly run optimization with full visibility (+acc)
# This is the standard ModelSim flow and avoids -novopt deprecation issues
if {[catch {eval vopt +acc $SIM_TOPS -o tb_top_opt} VOPT_ERR]} {
    echo "ERROR: vopt failed"
    echo "$VOPT_ERR"
    return
}

# Run simulation on the optimized design
# Use -suppress 12110 to ignore the error caused by implicit -novopt in user environment
if {[catch {vsim -suppress 12110 tb_top_opt} VSIM_ERR]} {
    echo "ERROR: vsim failed"
    echo "$VSIM_ERR"
    return
}

onfinish stop
run -all

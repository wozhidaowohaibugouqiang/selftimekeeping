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
if {$::USE_VENDOR_FIFO} {
    if {[file exists $FIFO_IP_SIM]} {
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

cd $PROJ_DIR
set TRANSCRIPT_FILE [file join $PROJ_DIR sim modelsim_simulate_flash_spi.log]
catch {transcript file $TRANSCRIPT_FILE}
catch {transcript on}
catch {vdel -all}

set WORK_LIB_DIR [file join $PROJ_DIR work]
catch {vmap -c}
vlib $WORK_LIB_DIR
vmap work $WORK_LIB_DIR
catch {vdel -lib work -all}

if {$::USE_VENDOR_FIFO} {
    foreach f $ANLOGIC_LIB_FILES { vlog -sv -work work $f }
}

vlog -sv -work work [file join $PROJ_DIR sim glbl.v]
vlog -sv -work work $FIFO_IP_FILE
vlog -sv -work work [file join $PROJ_DIR al_ip SPI_0 RTL SPI_Master_d4275a1ace18.v]
vlog -sv -work work [file join $PROJ_DIR al_ip SPI_0 SPI_0.v]
vlog -sv -work work [file join $PROJ_DIR RTL fifo.v]
vlog -sv -work work [file join $PROJ_DIR RTL flash_spi.v]
vlog -sv -work work [file join $PROJ_DIR sim timekeeping_core.v]
vlog -sv -work work [file join $PROJ_DIR sim tb_flash_spi.v]

catch {unalias vsim}
catch {unalias vopt}
catch {unalias vdel}

set USE_FALLBACK_NOVOPT 0
if {$::USE_VENDOR_FIFO && ($FIFO_IP_FILE eq $FIFO_IP_SIM)} {
    set VOPT_TOPS [list work.tb_flash_spi work.PH1_PHY_GSR]
} else {
    set VOPT_TOPS [list work.tb_flash_spi]
}
if {[catch {eval vopt +acc $VOPT_TOPS -o tb_flash_spi_opt} VOPT_ERR]} {
    set USE_FALLBACK_NOVOPT 1
}
if {!$USE_FALLBACK_NOVOPT} {
    if {[catch {vsim tb_flash_spi_opt -l $TRANSCRIPT_FILE} VSIM_ERR]} {
        set USE_FALLBACK_NOVOPT 1
    }
}
if {$USE_FALLBACK_NOVOPT} {
    if {$::USE_VENDOR_FIFO && ($FIFO_IP_FILE eq $FIFO_IP_SIM)} {
        set VSIM_TOPS [list work.tb_flash_spi work.PH1_PHY_GSR]
    } else {
        set VSIM_TOPS [list work.tb_flash_spi]
    }
    if {[catch {eval vsim -novopt $VSIM_TOPS -l $TRANSCRIPT_FILE} VSIM_ERR]} {
        echo "ERROR: vsim failed"
        echo "$VSIM_ERR"
        return
    }
}

view structure
view signals

add wave -r /tb_flash_spi/*
add wave -r /tb_flash_spi/dut/*

onfinish stop
run 80ms

set TP [string trim [examine -radix binary /tb_flash_spi/test_pass]]
if {$TP eq "1"} {
    echo "FLASH_SPI TB PASSED"
} else {
    echo "FLASH_SPI TB FAILED (test_pass=$TP)"
}

quit -f

transcript file sim_debug_new.log

set PROJ_DIR E:/fjlwork/selftimekeep
# Vendor libs are already compiled in 'work', so we skip them to save time/avoid issues
# If needed, we can add them back.

vlog -sv -work work $PROJ_DIR/al_ip/FIFO_0.v
vlog -sv -work work +incdir+$PROJ_DIR/RTL $PROJ_DIR/RTL/fifo.v
vlog -sv -work work +incdir+$PROJ_DIR/RTL $PROJ_DIR/RTL/flash_spi.v
vlog -sv -work work +incdir+$PROJ_DIR/RTL $PROJ_DIR/sim/spi_flash_model.v
vlog -sv -work work +incdir+$PROJ_DIR/RTL $PROJ_DIR/sim/timekeeping_core.v
vlog -sv -work work +incdir+$PROJ_DIR/RTL $PROJ_DIR/RTL/avg_remain_comp.v
vlog -sv -work work +incdir+$PROJ_DIR/RTL $PROJ_DIR/sim/tb_top.v

vopt +acc -suppress 12110 tb_top -o tb_top_opt
vsim -suppress 12110 tb_top_opt
run -all
quit -f

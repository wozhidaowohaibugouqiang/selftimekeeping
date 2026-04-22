# 仿真脚本文件
# 用于 ModelSim/QuestaSim 仿真

# 完全清理工作区（使用vdel命令避免权限问题）
vdel -all

# 创建新的工作区
vlib work

# 映射工作区到逻辑库
vmap work work

# 编译设计文件
vlog -sv -work work timekeeping_core.v
vlog -sv -work work tb_timekeeping_core.v

# 使用 vopt 优化设计，同时保留信号可见性
vopt tb_timekeeping_core_all_scene -o optimized_design +acc

# 加载优化后的设计
vsim optimized_design

# 添加波形到仿真窗口（使用通配符添加所有信号）
add wave *

# 设置波形显示格式
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 1

# 运行仿真
run -all

# 注意：没有添加 quit -f 命令，仿真结束后可以查看结果

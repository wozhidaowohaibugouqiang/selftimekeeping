@echo off
rem FIFO仿真命令脚本
rem 使用Anlogic库和原始FIFO_0 IP核

rem 设置ModelSim路径
set MODELSIM_PATH=D:\modeltech64_107\win64

rem 设置Anlogic库路径
set ANLOGIC_LIB_PATH=E:\TD_6.2.1\sim_release\ph1\ph1

rem 设置工作目录
cd /d E:\fjlwork\selftimekeep\sim

rem 运行仿真脚本
%MODELSIM_PATH%\vsim.exe -do "simulate_fifo.do"

pause

quit -sim

.main clear

vlib work
vlog ./tb_PFIFORM.v
vlog ./../design/*.v

vsim -voptargs=+acc work.tb_PFIFORM

add wave tb_PFIFORM/PFIFORM_instance/*
add wave -color red tb_PFIFORM/PFIFORM_instance/i_core_clk
#add wave -color green tb_PFIFORM/PFIFORM_instance/i_rx_rstn
add wave -color blue tb_PFIFORM/PFIFORM_instance/JoinEnable
add wave -unsigned -color red tb_PFIFORM/PFIFORM_instance/JoinAmout
add wave -unsigned -color green tb_PFIFORM/PFIFORM_instance/PopAmout
add wave -octal -color blue tb_PFIFORM/PFIFORM_instance/JoinData
add wave -octal -color red tb_PFIFORM/PFIFORM_instance/PopData
add wave -binary -color green tb_PFIFORM/PFIFORM_instance/RegisterCounterBranchCode
#add wave -color blue tb_PFIFORM/PFIFORM_instance/PopEnable
#add wave -unsigned -color red tb_PFIFORM/PFIFORM_instance/RegisterCounter
add wave -octal -color green tb_PFIFORM/PFIFORM_instance/CacheRegisterFIFO
#add wave -octal -color blue tb_PFIFORM/PFIFORM_instance/JoinDataPro
#add wave -octal -color green tb_PFIFORM/PFIFORM_instance/PopDataCache

run 80us
quit -sim

.main clear

vlib work
vlog ./tb_PFIFORM.v
vlog ./../design/*.v

vsim -voptargs=+acc work.tb_PFIFORM

add wave -color blue tb_PFIFORM/WrEnable
add wave -color red tb_PFIFORM/PFIFORM_instance/JoinPermit
add wave -color red tb_PFIFORM/PFIFORM_instance/i_core_clk
add wave -color green tb_PFIFORM/PFIFORM_instance/i_rx_rstn
add wave -color blue tb_PFIFORM/PFIFORM_instance/JoinEnable
add wave -unsigned -color red tb_PFIFORM/PFIFORM_instance/JoinAmout
add wave -unsigned -color green tb_PFIFORM/PFIFORM_instance/PopAmout
add wave -hex -color blue tb_PFIFORM/PFIFORM_instance/JoinData
add wave -hex -color red tb_PFIFORM/PFIFORM_instance/PopData
add wave -binary -color green tb_PFIFORM/PFIFORM_instance/RegisterCounterBranchCode
add wave -color blue tb_PFIFORM/PFIFORM_instance/PopPermit
add wave -color blue tb_PFIFORM/PFIFORM_instance/PopEnable
add wave -unsigned -color red tb_PFIFORM/PFIFORM_instance/RegisterCounter
add wave -hex -color blue tb_PFIFORM/PFIFORM_instance/CacheRegisterFIFO
add wave -hex -color blue tb_PFIFORM/PFIFORM_instance/JoinDataPro
add wave -hex -color blue tb_PFIFORM/PFIFORM_instance/PopDataCache

run 80us
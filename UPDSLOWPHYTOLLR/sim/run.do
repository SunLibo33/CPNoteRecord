quit -sim

.main clear

vlib work
vlog ./tb_UPDSLOWPHYTOLLR.v
vlog ./../design/*.v

vsim -voptargs=+acc work.tb_UPDSLOWPHYTOLLR

#add wave -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/*
add wave -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/i_core_clk
add wave -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/i_rx_fsm_rstn
add wave -color green tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/IQ_FIFO_Empty
add wave -color green tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/Noise_FIFO_Empty
add wave -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/IQ_FIFO_Read_Enable
add wave -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/Noise_FIFO_Read_Enable
add wave -color green tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/i_core_clk
add wave -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/o_data_strobe
add wave -hex -color green tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/o_re0_data_i
add wave -hex -color green tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/o_re0_data_q
add wave -hex -color green tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/o_re1_data_i
add wave -hex -color green tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/o_re1_data_q
add wave -hex -color blue tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/o_noise_data

add wave -decimal -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/LoopCycleCounter
add wave -decimal -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/SendIQCycleCounter
add wave -decimal -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/LoopNoiseInnerCycleCounter
add wave -decimal -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/SendNoiseCycleCounterPre
add wave -decimal -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/SendNoiseCycleCounter
add wave -decimal -color red tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/SendIQCycleCounter
add wave -decimal -color green tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/RE_Counter
 
add wave -hex -color green tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/Current_State
add wave -hex -color green tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/Next_State


run 80us
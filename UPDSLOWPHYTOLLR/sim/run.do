quit -sim

.main clear

vlib work
vlog ./tb_FSM_Combine.v
vlog ./../design/*.v

vsim -voptargs=+acc work.tb_FSM_Combine

add wave tb_FSM_Combine/FSM_Combine_instance/*
add wave tb_FSM_Combine/FSM_Combine_instance/DualPort_SRAM_COMB_Ping_Buffer/*
add wave tb_FSM_Combine/FSM_Combine_instance/DualPort_SRAM_COMB_Pong_Buffer/*

run 10us
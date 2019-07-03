quit -sim

.main clear

vlib work
vlog ./tb_UPDSLOWPHYTOLLR.v
vlog ./../design/*.v

vsim -voptargs=+acc work.tb_UPDSLOWPHYTOLLR

add wave tb_UPDSLOWPHYTOLLR/UPDSLOWPHYTOLLR_instance/*
 

run 40us
quit -sim

.main clear

vlib work
vlog ./tb_rdm_dpsram.v
vlog ./../design/*.v

vsim -voptargs=+acc work.tb_rdm_dpsram

add wave tb_rdm_dpsram/rdm_dpsram_instance/*

run 80us
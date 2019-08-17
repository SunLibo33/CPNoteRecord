quit -sim
.main clear

vlib work
vmap work work

vlog ./tb_DataArray.v
vlog ./../design/*.v

vsim -voptargs=+acc work.tb_DataArray

add wave /tb_DataArray/RAMS
add wave /tb_DataArray/DataArray_instance/*
add wave /tb_DataArray/DataArray_instance/RAM

run 8us

 

 
 


 
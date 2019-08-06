quit -sim

.main clear

vlib work
vlog ./tb_sync_fifo.v
vlog ./../design/*.v 

vsim -voptargs=+acc work.tb_sync_fifo

add wave -color red tb_sync_fifo/sync_fifo_instance/clk
add wave tb_sync_fifo/sync_fifo_instance/rst_n
add wave tb_sync_fifo/sync_fifo_instance/wr
add wave -unsigned -color red tb_sync_fifo/sync_fifo_instance/datain
add wave tb_sync_fifo/sync_fifo_instance/rd
add wave -unsigned -color red tb_sync_fifo/sync_fifo_instance/dataout
add wave tb_sync_fifo/sync_fifo_instance/full
add wave tb_sync_fifo/sync_fifo_instance/empty
add wave -unsigned tb_sync_fifo/sync_fifo_instance/rp
add wave -unsigned tb_sync_fifo/sync_fifo_instance/wp


run 1200ns
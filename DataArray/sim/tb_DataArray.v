`timescale 1ns/100ps
module tb_DataArray();


reg tb_sclk;
reg tb_rst_n;
reg [3:0]RAMS;

reg i_demux_user_end;
reg [5:0]i_demux_user_idx;
reg i_demux_user_start;


wire [19:0]RAMSA;


initial begin
	tb_sclk=1'b0;
	tb_rst_n=1'b0;
	#100;
	tb_rst_n=1'b1;
end

initial begin
 	gen_data();	
end

initial begin
 	i_demux_user_start=1'b0;
	i_demux_user_idx=6'd10;
	i_demux_user_end=1'b0;
	#100
	i_demux_user_idx=6'd10;	
	#80
    i_demux_user_end=1'b1;
	#20
    i_demux_user_end=1'b0;
	

	#80
	i_demux_user_idx=6'd18;	
	#80
    i_demux_user_end=1'b1;
	#20
    i_demux_user_end=1'b0;	
	
	#80
	i_demux_user_idx=6'd12;	
	#80
    i_demux_user_end=1'b1;
	#20
    i_demux_user_end=1'b0;	
	
	#80
	i_demux_user_idx=6'd39;	
	#80
    i_demux_user_end=1'b1;
	#20
    i_demux_user_end=1'b0;		

	#80
	i_demux_user_idx=6'd1;	
	#80
    i_demux_user_start=1'b1;
	#20
    i_demux_user_start=1'b0;		
	
	#80
	i_demux_user_idx=6'd18;	
	#80
    i_demux_user_start=1'b1;
	#20
    i_demux_user_start=1'b0;

	#80
	i_demux_user_idx=6'd39;	
	#80
    i_demux_user_start=1'b1;
	#20
    i_demux_user_start=1'b0;	
	
	#80
	i_demux_user_idx=6'd10;	
	#80
    i_demux_user_start=1'b1;
	#20
    i_demux_user_start=1'b0;	

	#80
	i_demux_user_idx=6'd12;	
	#80
    i_demux_user_start=1'b1;
	#20
    i_demux_user_start=1'b0;		
	
 
end

always #10 tb_sclk=~tb_sclk;

DataArray DataArray_instance(
      .i_core_clk         (tb_sclk ),
      .i_rx_rstn          (tb_rst_n),
      .RAMS               ( ({5{RAMS}}) ),

      .i_demux_user_end   (i_demux_user_end ),
      .i_demux_user_idx   (i_demux_user_idx ),
	  .i_demux_user_start (i_demux_user_start),
	  .RAMSA              (RAMSA)
);
 

task gen_data();
     integer i;
	 begin
	   for(i=0;i<256;i=i+1)
	   begin
	     @(posedge tb_sclk);
		 RAMS=i;
	   end
	 end

endtask

/* 
task wr_dara();
  begin
    #80
	i_demux_user_end=1'b0;
	#80
 	i_demux_user_end=1'b1;	
    #20
	i_demux_user_end=1'b0;
    #80
 end
endtask

task rd_data();
  begin
    #80
	i_demux_user_start=1'b0;
	#80
 	i_demux_user_start=1'b1;	
    #20
	i_demux_user_start=1'b0;
    #80 
 end
endtask 
*/

 


endmodule
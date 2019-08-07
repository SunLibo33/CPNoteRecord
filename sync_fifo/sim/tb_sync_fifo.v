`timescale 1ns/100ps
module tb_sync_fifo();

reg tb_sclk;
reg tb_rst_n;

reg[7:0]tb_in;

reg wr;
reg rd;

wire [7:0]dataout;
wire full;
wire empty;

 
initial 
  begin
    tb_rst_n = 1'b0;
	tb_sclk  = 1'b0;
    #100
	tb_rst_n = 1'b1;
	gen_data();
 
  end
  
initial 
  begin
    wr  = 1'b0;
	rd  = 1'b0;
    #180
    wr  = 1'b1;
    #190
    wr  = 1'b0;
    #290
    rd  = 1'b1;
     
  end  
  
   
  
always #10  tb_sclk=~tb_sclk; 
  
sync_fifo sync_fifo_instance(

  .rst_n(tb_rst_n),  
  .clk(tb_sclk),
  .wr(wr),
  .datain(tb_in),
  .rd(rd),
  .dataout(dataout),
  .full(full),
  .empty(empty)
  
);

 task gen_data();
     integer m,n;
	 begin
	   for(n=0;n<1024;n=n+1) 
	     begin
		   for(m=0;m<256;m=m+1)
		   begin
			 @(posedge tb_sclk);
			     begin
				   tb_in=m;
				 end
		   end
		end
	 end
endtask 


endmodule


`timescale 1ns/100ps
module tb_rdm_dpsram();

reg tb_sclk;
reg tb_rst_n;

initial 
  begin
    tb_rst_n = 1'b0;
	tb_sclk  = 1'b0;
    #100
	tb_rst_n = 1'b1;
	gen_data();
  end
  
wire [1151:0]doutb;
reg [15:0] tb_in_a,tb_in_b;
reg [143:0]wea;


initial 
  begin
    wea = { {4{1'b0}},{140{1'b1}} };
    #1000
	wea = { {4{1'b1}},{140{1'b0}} };
  end
    
always #10  tb_sclk=~tb_sclk; 
  

rdm_dpsram rdm_dpsram_instance(
  .dina({144{8'haa}}),  
  .addra(11'd100),
  .addrb(11'd100),
  .wea(wea),
  .clka(tb_sclk),
  .clkb(tb_sclk),  
  .doutb(doutb)    
);


task gen_data();
     integer i,data_tmp;
	 begin
	   for(i=0;i<256;i=i+1)
	   begin
	     @(posedge tb_sclk);
		 data_tmp={$random}%32768;
		 tb_in_a=data_tmp;
		 tb_in_b=data_tmp;
	   end
	 end
endtask  

endmodule

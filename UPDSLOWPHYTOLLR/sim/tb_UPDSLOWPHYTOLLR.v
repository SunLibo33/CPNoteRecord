`timescale 1ns/100ps
module tb_UPDSLOWPHYTOLLR();

reg tb_sclk;
reg tb_rst_n;

wire IQ_FIFO_Read_Enable;
wire Noise_FIFO_Read_Enable;
wire o_data_strobe;
wire Strobe_Enable;

wire [15:0] o_re0_data_i;
wire [15:0] o_re0_data_q;
wire [15:0] o_re1_data_i;
wire [15:0] o_re1_data_q;
wire [15:0] o_noise_data;

reg IQ_FIFO_Empty;
reg Noise_FIFO_Empty;

reg [15:0] tb_in_a;


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
   IQ_FIFO_Empty=1'b0;
   Noise_FIFO_Empty=1'b0;
   
   #2090
   
   Noise_FIFO_Empty=1'b1;
   #120
   
   IQ_FIFO_Empty=1'b1;
   
   #780
   
   Noise_FIFO_Empty=1'b0;
   #800
   IQ_FIFO_Empty=1'b0;
   

  end  
  
always #10  tb_sclk=~tb_sclk; 
  
UPDSLOWPHYTOLLR UPDSLOWPHYTOLLR_instance(
  .i_rx_rstn(tb_rst_n), 
  .i_rx_fsm_rstn(tb_rst_n), 			 
  .i_core_clk(tb_sclk), 
  .i_user_iq_noise_rate(16'd6),
  .i_cur_user_re_amounts(16'd1797),
  .Noise_Data_SUM({16'h77,16'h66,16'h55,16'h44,16'h33,16'h22,16'h11,16'h0C}),
  .IQ_Data_SUM({16'h77,16'h66,16'h55,16'h44,16'h33,16'h22,16'h11,16'h0C}),
  //.Noise_Data_SUM(({8{tb_in_a}})),
  //.IQ_Data_SUM(({8{tb_in_a}})),
  .IQ_FIFO_Empty(IQ_FIFO_Empty),
  .Noise_FIFO_Empty(Noise_FIFO_Empty),
  .IQ_FIFO_Read_Enable(IQ_FIFO_Read_Enable),
  .Noise_FIFO_Read_Enable(Noise_FIFO_Read_Enable),
  .Strobe_Enable(Strobe_Enable),
  .o_data_strobe(o_data_strobe),
  .o_re0_data_i(o_re0_data_i),
  .o_re0_data_q(o_re0_data_q),
  .o_re1_data_i(o_re1_data_i),
  .o_re1_data_q(o_re1_data_q),
  .o_noise_data(o_noise_data)
  
);
/* 
  input  wire [15:0]  i_user_iq_noise_rate,
  input  wire [15:0]  i_cur_user_re_amounts,
  
  output wire         IQ_FIFO_Read_Enable,
  output wire         Noise_FIFO_Read_Enable,
 
  output reg          o_data_strobe,
  output reg  [15:0]  o_re0_data_i, 
  output reg  [15:0]  o_re0_data_q, 
  output reg  [15:0]  o_re1_data_i, 
  output reg  [15:0]  o_re1_data_q, 
  output reg  [15:0]  o_noise_data */

 task gen_data();
     integer m,n;
	 begin
	   for(n=0;n<1024;n=n+1) 
	     begin
		   for(m=0;m<128;m=m+1)
		   begin
			 @(posedge tb_sclk);
			   tb_in_a=m;
		   end
		end
	 end
endtask 

/*  task gen_data();
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
endtask  */

endmodule


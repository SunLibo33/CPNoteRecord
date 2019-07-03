`timescale 1ns/100ps
module tb_FSM_Combine();

reg tb_sclk;
reg tb_rst_n;
reg i_RDM_Data_Comp;
reg i_RDM_Data_Valid;
reg i_Combine_process_request;
 
reg signed[5:0]tb_in_a;

wire [159:0]DualPort_SRAM_COMB_Ping_Buffer_Read_Data;
wire [159:0]DualPort_SRAM_COMB_Pong_Buffer_Read_Data; 

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
    i_Combine_process_request=1'b1;
    i_RDM_Data_Comp = 1'b0;
    #1000
	i_RDM_Data_Comp = 1'b1;
	#20
	i_Combine_process_request=1'b0;
	#140
	i_Combine_process_request=1'b1;
	i_RDM_Data_Comp = 1'b0;
    #1000
	i_RDM_Data_Comp = 1'b1;
	#20
	i_Combine_process_request=1'b0;
  end  
  
  initial 
  begin
    i_RDM_Data_Valid = 1'b1;
    #400
	i_RDM_Data_Valid = 1'b0;
    #40
	i_RDM_Data_Valid = 1'b1;
    #140
    i_RDM_Data_Valid = 1'b1;
    #400
	i_RDM_Data_Valid = 1'b0;
    #40
	i_RDM_Data_Valid = 1'b1;	
  end  

always #10  tb_sclk=~tb_sclk; 
  
FSM_Combine FSM_Combine_instance(
  .i_rx_rstn(tb_rst_n), 
  .i_rx_fsm_rstn(tb_rst_n), 			 
  .i_core_clk(tb_sclk), 
  .i_Combine_process_request(i_Combine_process_request),
  .i_Combine_user_index(4'b0001),

  .i_RDM_Data_Valid(i_RDM_Data_Valid),  
  .i_RDM_Data_Content({16{tb_in_a}}),
  .i_RDM_Data_Comp(i_RDM_Data_Comp), 
  .i_users_ncb({8{16'd160}}), 
  .i_SENDHARQ_Data_Comp(1'b1),
  .DualPort_SRAM_COMB_Ping_Buffer_Read_Data(DualPort_SRAM_COMB_Ping_Buffer_Read_Data),
  .DualPort_SRAM_COMB_Pong_Buffer_Read_Data(DualPort_SRAM_COMB_Pong_Buffer_Read_Data) 
   
);

task gen_data();
     integer m,n;
	 begin
	   for(n=0;n<256;n=n+1) 
	     begin
		   for(m=0;m<32;m=m+1)
		   begin
			 @(posedge tb_sclk);
			   tb_in_a=1;
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


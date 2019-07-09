`timescale 1ns/100ps
module tb_PFIFORM();

reg tb_sclk;
reg tb_rst_n;

reg [5:0]  tb_in_a1;
reg [5:0]  tb_in_a2;
reg [5:0]  tb_in_a3;
reg [5:0]  tb_in_a4;
reg [5:0]  tb_in_a5;
reg [5:0]  tb_in_a6;
reg [5:0]  tb_in_a7;
reg [5:0]  tb_in_a8;
reg [5:0]  tb_in_a9;
reg [5:0]  tb_in_a10;
reg [5:0]  tb_in_a11;
reg [5:0]  tb_in_a12;
reg [5:0]  tb_in_a13;
reg [5:0]  tb_in_a14;
reg [5:0]  tb_in_a15;
reg [5:0]  tb_in_a16;
reg [95:0]  tb_in_a_1d;


reg        JoinEnable;
wire       PopEnable;
wire       JoinPermit;

wire [95:0]PopData;

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
    JoinEnable=1'b0;
    #200
    JoinEnable=1'b1;
  end
  
  
always #10  tb_sclk=~tb_sclk; 
  
PFIFORM PFIFORM_instance(
  .i_rx_rstn(tb_rst_n),  
  .i_core_clk(tb_sclk),
  .JoinEnable(JoinEnable),
  .JoinPermit(JoinPermit),
  .PopEnable(PopEnable),
  .PopPermit(1'b1),  
  .JoinAmout(4'd15),
  .PopAmout(4'd11),
  //.JoinData({12{8'hAA}}),
  .JoinData(tb_in_a_1d),
  .PopData(PopData)

  
  
);
/*     input  wire         i_rx_rstn, 		 
  input  wire         i_core_clk,
  input  wire         JoinEnable,
  output wire         JoinPermit,
  
  input  wire         PopPermit,

  input  wire  [3:0]  JoinAmout,
  input  wire  [3:0]  PopAmout,
  
  input  wire  [95:0] JoinData,
  
  output wire  [95:0] PopData,
  output wire         PopEnable  
   */

 task gen_data();
     integer m,n;
	 begin
	   for(n=0;n<1024;n=n+1) 
	     begin
		   for(m=0;m<256;m=m+16)
		   begin
			 @(posedge tb_sclk);
			   tb_in_a1=m;
               tb_in_a2=m+1;
               tb_in_a3=m+2;
               tb_in_a4=m+3;
               tb_in_a5=m+4;
               tb_in_a6=m+5;
               tb_in_a7=m+6;
               tb_in_a8=m+7;
               tb_in_a9=m+8;
               tb_in_a10=m+9;
               tb_in_a11=m+10;
               tb_in_a12=m+11;
               tb_in_a13=m+12;
               tb_in_a14=m+13;
               tb_in_a15=m+14;
               tb_in_a16=m+15;
		   end
		end
	 end
endtask 

always @(posedge tb_sclk)
begin
  tb_in_a_1d<={
  tb_in_a16,
  tb_in_a15,
  tb_in_a14,
  tb_in_a13, 
  tb_in_a12,
  tb_in_a11,
  tb_in_a10,
  tb_in_a9,
  tb_in_a8,
  tb_in_a7,
  tb_in_a6,
  tb_in_a5,
  tb_in_a4,
  tb_in_a3,
  tb_in_a2,
  tb_in_a1
  
  };
end

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


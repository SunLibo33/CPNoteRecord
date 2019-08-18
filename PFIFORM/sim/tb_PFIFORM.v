`timescale 1ns/100ps
module tb_PFIFORM();

reg tb_sclk;
reg tb_rst_n;

reg [7:0]  tb_in_a1;
reg [7:0]  tb_in_a2;
reg [7:0]  tb_in_a3;
reg [7:0]  tb_in_a4;
reg [7:0]  tb_in_a5;
reg [7:0]  tb_in_a6;
reg [7:0]  tb_in_a7;
reg [7:0]  tb_in_a8;
reg [7:0]  tb_in_a9;
reg [7:0]  tb_in_a10;
reg [7:0]  tb_in_a11;
reg [7:0]  tb_in_a12;
reg [7:0]  tb_in_a13;
reg [7:0]  tb_in_a14;
reg [7:0]  tb_in_a15;
reg [7:0]  tb_in_a16;
reg [7:0]  tb_in_a17;
reg [7:0]  tb_in_a18;
reg [7:0]  tb_in_a19;
reg [7:0]  tb_in_a20;
reg [7:0]  tb_in_a21;
reg [7:0]  tb_in_a22;
reg [7:0]  tb_in_a23;
reg [7:0]  tb_in_a24;
reg [7:0]  tb_in_a25;
reg [7:0]  tb_in_a26;
reg [7:0]  tb_in_a27;
reg [7:0]  tb_in_a28;
reg [7:0]  tb_in_a29;
reg [7:0]  tb_in_a30;
reg [7:0]  tb_in_a31;
reg [7:0]  tb_in_a32;

reg [127:0]  tb_in_a_1d;
reg [127:0]  tb_in_a_2d;
reg [127:0]  tb_in_a_3d;

reg[4:0]JoinAmount;
reg[4:0]PopAmount;


reg        JoinEnable;
reg        JoinEnableCache;
wire       PopEnable;
wire       JoinPermit;
reg        PopPermit;

wire [255:0]PopData;

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
	PopPermit=1'b0;
    #200
    JoinEnable=1'b1;
    #1930.7
    JoinEnable=1'b0;
	#1870
	PopPermit=1'b1;
    #2000
    JoinEnable=1'b1;
    #18000
    JoinEnable=1'b0;
  end
  
  
initial 
  begin
   JoinAmount=5'd10;
   PopAmount=5'd19;
   #2290.1
   JoinAmount=5'd31;
   PopAmount=5'd19;
 
  end
  
  
  
always #10  tb_sclk=~tb_sclk; 
  
//Libo Sun , Unit Testing JoinAmount ,  PopAmount 
//                              7        19
//                              19       7
//                              7        7
//                              19       19
//                              31       31
//                              15       15
//                              23       23
//                              17       17
//                              17       15
//                              23       15

PFIFORM PFIFORM_instance(
  .i_rx_rstn(tb_rst_n),  
  .i_core_clk(tb_sclk),
  .JoinEnable(JoinEnable),
  .JoinPermit(JoinPermit),
  .PopEnable(PopEnable),
  .PopPermit(PopPermit),  
  .JoinAmount(JoinAmount),
  .PopAmount(PopAmount),
  //.JoinData({12{8'hAA}}),
  .JoinData({tb_in_a_2d,tb_in_a_1d}),
  .PopData(PopData)
 
  
);
/*     input  wire         i_rx_rstn, 		 
  input  wire         i_core_clk,
  input  wire         JoinEnable,
  output wire         JoinPermit,
  
  input  wire         PopPermit,

  input  wire  [3:0]  JoinAmount,
  input  wire  [3:0]  PopAmount,
  
  input  wire  [95:0] JoinData,
  
  output wire  [95:0] PopData,
  output wire         PopEnable  
   */
   
 

 task gen_data();
     integer m,n;
	 begin
	   for(n=0;n<1024;n=n+1) 
	     begin
		   for(m=0;m<256;m=m+32)
		   begin
			 @(posedge tb_sclk);
			   if(JoinPermit==1'b1)
			     begin
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
				   tb_in_a17=m+16;
				   tb_in_a18=m+17;
				   tb_in_a19=m+18;
				   tb_in_a20=m+19;
				   tb_in_a21=m+20;
				   tb_in_a22=m+21;
				   tb_in_a23=m+22;
				   tb_in_a24=m+23;
				   tb_in_a25=m+24;
				   tb_in_a26=m+25;
				   tb_in_a27=m+26;
				   tb_in_a28=m+27;
				   tb_in_a29=m+28;
				   tb_in_a30=m+29;
				   tb_in_a31=m+30;
				   tb_in_a32=m+31;
				   
				   
				 end
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

always @(posedge tb_sclk)
begin
  tb_in_a_2d<={
  tb_in_a32,
  tb_in_a31,
  tb_in_a30,
  tb_in_a29, 
  tb_in_a28,
  tb_in_a27,
  tb_in_a26,
  tb_in_a25,
  tb_in_a24,
  tb_in_a23,
  tb_in_a22,
  tb_in_a21,
  tb_in_a20,
  tb_in_a19,
  tb_in_a18,
  tb_in_a17
  
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


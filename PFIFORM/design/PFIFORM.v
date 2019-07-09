////////////////////////////////////////////////////////////////////////////////
// Company: <...>
// Engineer: <Libo Sun>
//
// Create Date: <2019 June 10>
// Design Name: <name_of_top-level_design>
// Module Name: <DeRateMatching>
// Target Device: <target device>
// Tool versions: <tool_versions>
// Description:
//    <5G NR TS38.212 for the DeRateMatching function of decode Phase>
// Dependencies:
//    <Dependencies here>
// Revision:
//    <V0.1>
// Additional Comments:
//    <Additional_comments>
////////////////////////////////////////////////////////////////////////////////

module PFIFORM 
(
  input  wire         i_rx_rstn, 		 
  input  wire         i_core_clk,
  
  input  wire         JoinEnable, //Write Enable signal from  FIFO Write module
  output wire         JoinPermit, //Write Ready  signal to    FIFO Write module 
                                  
                                  //JoinPermit==1'b0, means write operation will be ignored
                                  
  input  wire         PopPermit,  //Read Ready signal from  FIFO Read module

  input  wire  [3:0]  JoinAmout,  //Write data amount from  FIFO Write module
  input  wire  [3:0]  PopAmout,   //Read  data amount from  FIFO Read  module
  
  input  wire  [95:0] JoinData,   //Write data   LSB alignment
  
  output wire  [95:0] PopData,    //Read  data   LSB alignment
  output wire         PopEnable   //Read  enable signal to  FIFO Read module 
);

wire           JoinEnableInner;
wire [1:0]     RegisterCounterBranchCode;
reg  [7:0]     RegisterCounter=8'd0;
reg  [287:0]   CacheRegisterFIFO=288'd0;

assign         JoinEnableInner=((JoinEnable==1'b1)&&(JoinPermit==1'b1));
assign         JoinPermit=(JoinAmout+RegisterCounter+1'b1)<=8'd48;
assign         RegisterCounterBranchCode={PopEnable,JoinEnableInner};
assign         PopEnable=( ((PopAmout+1'b1)<=RegisterCounter) && (PopPermit==1'b1) );

wire [95:0]  JoinDataPro;
assign       JoinDataPro=JoinData & ( ({96{1'b1}})>>((15-JoinAmout)*6) );

wire [287:0] PopDataCache;
assign       PopData= PopDataCache[95:0] & ( ({96{1'b1}})>>((15-PopAmout)*6) );

always @(posedge i_core_clk or negedge i_rx_rstn)
begin
  if(i_rx_rstn==1'b0)
    CacheRegisterFIFO<=288'd0;
  else if(JoinEnableInner==1'b1)
    CacheRegisterFIFO<=( ( CacheRegisterFIFO<<((JoinAmout+1)*6) ) | JoinDataPro );
end

always @(posedge i_core_clk or negedge i_rx_rstn)
begin
  if(i_rx_rstn==1'b0)
    RegisterCounter<=8'd0;
  else
    begin
      case(RegisterCounterBranchCode)
        2'b00: RegisterCounter<=RegisterCounter;
        2'b01: RegisterCounter<=RegisterCounter+JoinAmout+1'b1;
        2'b10: RegisterCounter<=RegisterCounter-PopAmout-1'b1;
        2'b11: RegisterCounter<=RegisterCounter+JoinAmout-PopAmout;
        default: RegisterCounter<=RegisterCounter;
      endcase
    end
end
 
assign PopDataCache=CacheRegisterFIFO>>((RegisterCounter-PopAmout-1'b1)*6); 

endmodule

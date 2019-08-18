////////////////////////////////////////////////////////////////////////////////
// Company: <...>
// Engineer: <Libo Sun>
//
// Create Date: <2019 July 09 23:21>
// Design Name: <name_of_top-level_design>
// Module Name: <RateDeMatching->parallel_fifo>
// Target Device: <target device>
// Tool versions: <tool_versions>
// Description:
//    <5G NR TS38.212 for the RateDeMatching function of decode Phase>
// Dependencies:
//    <Dependencies here>
// Revision:
//    <V0.1>
// Additional Comments:
//    <Additional_comments>
////////////////////////////////////////////////////////////////////////////////

module PFIFORM 
(
  input  wire         i_rx_rstn,  //Reset signal with asynchronous and active low		 
  input  wire         i_core_clk, //System Clock
  
  input  wire         JoinEnable, //Write Enable signal come from   the FIFO Write module
  output wire         JoinPermit, //Write Ready  signal to   inform the FIFO Write module                                   
                                  //if (JoinPermit==1'b1) means that current write operation is Effective
                                  
  input  wire         PopPermit,  //Read Ready signal from FIFO Read module  
                                  //if (PopPermit==1'b1) means that FIFO Read module is ready to receive data

  input  wire  [4:0]  JoinAmount,  //Write data amount from  FIFO Write module , Actual numbers minus one (hard bits)
  input  wire  [4:0]  PopAmount,   //Read  data amount from  FIFO Read  module , Actual numbers minus one (hard bits)
  
  input  wire  [255:0] JoinData,   //Write data   LSB alignment ,  Valid data bits is [(JoinAmount*8+7):0]
  
  output wire  [255:0] PopData,    //Read  data   LSB alignment ,  Valid data bits is [(PopAmount*8+7):0]
  output wire          PopEnable   //Read  enable signal to  FIFO Read module (strobe signal)
);

wire           JoinEnableInner;
wire [1:0]     RegisterCounterBranchCode;
reg  [6:0]     RegisterCounter=7'd0;    //may be the data width should be reduce
reg  [767:0]   CacheRegisterFIFO=768'd0;

assign         JoinEnableInner=((JoinEnable==1'b1)&&(JoinPermit==1'b1));
assign         JoinPermit=(JoinAmount+RegisterCounter)<7'd96;

assign         RegisterCounterBranchCode={PopEnable,JoinEnableInner};
assign         PopEnable=( (PopAmount<RegisterCounter) && (PopPermit==1'b1) );

wire [255:0]  JoinDataPro;
assign        JoinDataPro= JoinData <<( {(31-JoinAmount),3'b000} ) ;

wire [767:0] PopDataCache;
//assign      PopData= PopDataCache[255:0];
assign        PopData= PopDataCache[255:0] & ( ({256{1'b1}})>>( {(31-PopAmount),3'b000} ) );

always @(posedge i_core_clk or negedge i_rx_rstn)
begin
  if(i_rx_rstn==1'b0)
    CacheRegisterFIFO<=768'd0;
  else if(JoinEnableInner==1'b1)
	CacheRegisterFIFO<=( ( CacheRegisterFIFO>>( {(JoinAmount+1),3'b000} ) ) | ({JoinDataPro,512'd0}) );
end

always @(posedge i_core_clk or negedge i_rx_rstn)
begin
  if(i_rx_rstn==1'b0)
    RegisterCounter<=7'd0;
  else
    begin
      case(RegisterCounterBranchCode)
        2'b00: RegisterCounter<=RegisterCounter;
        2'b01: RegisterCounter<=RegisterCounter+JoinAmount+1'b1;
        2'b10: RegisterCounter<=RegisterCounter-PopAmount-1'b1;
        2'b11: RegisterCounter<=RegisterCounter+JoinAmount-PopAmount;
        default: RegisterCounter<=RegisterCounter;
      endcase
    end
end

assign PopDataCache=CacheRegisterFIFO>>( {(7'd96-RegisterCounter),3'b000} ); 

endmodule



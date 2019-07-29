////////////////////////////////////////////////////////////////////////////////
// Company: <...>
// Engineer: <Libo Sun>
//
// Create Date: <2019 July 09 23:21>
// Design Name: <name_of_top-level_design>
// Module Name: <RateDeMatching>
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

  input  wire  [4:0]  JoinAmout,  //Write data amount from  FIFO Write module , Actual numbers minus one (hard bits)
  input  wire  [4:0]  PopAmout,   //Read  data amount from  FIFO Read  module , Actual numbers minus one (hard bits)
  
  input  wire  [191:0] JoinData,   //Write data   LSB alignment ,  Valid data bits is [(JoinAmout*6+5):0]
  
  output wire  [191:0] PopData,    //Read  data   LSB alignment ,  Valid data bits is [(PopAmout*6+5):0]
  output wire          PopEnable   //Read  enable signal to  FIFO Read module (strobe signal)
);

wire           JoinEnableInner;
wire [1:0]     RegisterCounterBranchCode;
reg  [7:0]     RegisterCounter=8'd0;    //may be the data width should be reduce
reg  [383:0]   CacheRegisterFIFO=384'd0;

assign         JoinEnableInner=((JoinEnable==1'b1)&&(JoinPermit==1'b1));
//assign       JoinPermit=(JoinAmout+RegisterCounter+1'b1)<=8'd48;
assign         JoinPermit=(JoinAmout+RegisterCounter)<8'd64;
assign         RegisterCounterBranchCode={PopEnable,JoinEnableInner};
//assign       PopEnable=( ((PopAmout+1'b1)<=RegisterCounter) && (PopPermit==1'b1) );
assign         PopEnable=( (PopAmout<RegisterCounter) && (PopPermit==1'b1) );

wire [191:0]  JoinDataPro;
assign        JoinDataPro= JoinData <<((31-JoinAmout)*6) ;

wire [383:0] PopDataCache;
assign       PopData= PopDataCache[191:0] & ( ({192{1'b1}})>>((31-PopAmout)*6) );

always @(posedge i_core_clk or negedge i_rx_rstn)
begin
  if(i_rx_rstn==1'b0)
    CacheRegisterFIFO<=384'd0;
  else if(JoinEnableInner==1'b1)
	CacheRegisterFIFO<=( ( CacheRegisterFIFO>>((JoinAmout+1)*6) ) | ({JoinDataPro,192'd0}) );
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
 
//assign PopDataCache=CacheRegisterFIFO>>((8'd48-RegisterCounter)*6); 
assign PopDataCache=CacheRegisterFIFO>>((8'd64-RegisterCounter)*6); 



endmodule


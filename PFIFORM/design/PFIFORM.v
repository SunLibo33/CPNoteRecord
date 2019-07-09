module PFIFORM //
//#(parameter DATA_WIDTH=48, parameter ADDR_WIDTH=11)
(
  input  wire         i_rx_rstn, 		 
  input  wire         i_core_clk,
  input  wire         JoinEnable,
  output wire         JoinPermit,
  
  input  wire         PopPermit,

  input  wire  [3:0]  JoinAmout,
  input  wire  [3:0]  PopAmout,
  
  input  wire  [95:0] JoinData,
  
  output wire  [95:0] PopData,
  output wire         PopEnable  
);

wire           JoinEnableInner;
wire   [1:0]   RegisterCounterBranchCode;
reg  [5:0]RegisterCounter=6'd0;
reg  [287:0]CacheRegisterFIFO=288'd0;

assign         JoinEnableInner=((JoinEnable==1'b1)&&(JoinPermit==1'b1));
assign         JoinPermit=(JoinAmout+RegisterCounter+1'b1)<=6'd48;
assign         RegisterCounterBranchCode={PopEnable,JoinEnableInner};
assign         PopEnable=( ((PopAmout+1'b1)<=RegisterCounter) && (PopPermit==1'b1) );

wire [95:0]JoinDataPro;
assign     JoinDataPro=JoinData & ( ({96{1'b1}})>>((15-JoinAmout)*6) );

wire [287:0]PopDataCache;
assign     PopData= PopDataCache[95:0] & ( ({96{1'b1}})>>((15-PopAmout)*6) );

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
    RegisterCounter<=6'd0;
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

`timescale 1ns/100ps
module DataArray(
       input wire i_core_clk,
       input wire i_rx_rstn,
	   input wire [19:0]RAMS,

	   input wire i_demux_user_end,
	   input wire [5:0]i_demux_user_idx,
	   input wire i_demux_user_start,
	   output reg [19:0]RAMSA 

);

reg [19:0] RAM [39:0];

reg [39:0]i_user_idx_one_hot;

generate 
  genvar i;
    for(i=0;i<40;i=i+1)
      begin:KeyRegistersValueCache_I
	      always @(posedge i_core_clk or negedge i_rx_rstn)
	        begin
              if(i_rx_rstn==1'b0)
	              RAM [i] <= 20'd0;
                else if((i_demux_user_end==1'b1)&&(i_user_idx_one_hot[i]==1'b1))
		      //else if((i_demux_user_end==1'b1)&&(i_user_idx_one_hot[i]==1'b0))//Error Condition	
                  RAM [i] <= RAMS; 				  
				//RAM [i][3:0] <= RAMS;//Partly assign
				//RAM [i][7:0] <= RAMS;//Partly assign
	        end
      end
endgenerate

always @(posedge i_core_clk or negedge i_rx_rstn)
begin
  if(i_rx_rstn==1'b0)
    RAMSA<=20'd0;
  else if(i_demux_user_start==1'b1)
    RAMSA<=RAM[i_demux_user_idx];  
end

always @(*)
begin
  case(i_demux_user_idx)
    6'd0: i_user_idx_one_hot=40'h00_0000_0001;
    6'd1: i_user_idx_one_hot=40'h00_0000_0002;
    6'd2: i_user_idx_one_hot=40'h00_0000_0004;
    6'd3: i_user_idx_one_hot=40'h00_0000_0008;
    6'd4: i_user_idx_one_hot=40'h00_0000_0010;
    6'd5: i_user_idx_one_hot=40'h00_0000_0020;
    6'd6: i_user_idx_one_hot=40'h00_0000_0040;
    6'd7: i_user_idx_one_hot=40'h00_0000_0080;
    6'd8: i_user_idx_one_hot=40'h00_0000_0100;
    6'd9: i_user_idx_one_hot=40'h00_0000_0200;
    6'd10:i_user_idx_one_hot=40'h00_0000_0400;
    6'd11:i_user_idx_one_hot=40'h00_0000_0800;
    6'd12:i_user_idx_one_hot=40'h00_0000_1000;
    6'd13:i_user_idx_one_hot=40'h00_0000_2000;
    6'd14:i_user_idx_one_hot=40'h00_0000_4000;
    6'd15:i_user_idx_one_hot=40'h00_0000_8000;
    6'd16:i_user_idx_one_hot=40'h00_0001_0000;
    6'd17:i_user_idx_one_hot=40'h00_0002_0000;
    6'd18:i_user_idx_one_hot=40'h00_0004_0000;
    6'd19:i_user_idx_one_hot=40'h00_0008_0000;
    6'd20:i_user_idx_one_hot=40'h00_0010_0000;
    6'd21:i_user_idx_one_hot=40'h00_0020_0000;
    6'd22:i_user_idx_one_hot=40'h00_0040_0000;
    6'd23:i_user_idx_one_hot=40'h00_0080_0000;
    6'd24:i_user_idx_one_hot=40'h00_0100_0000;
    6'd25:i_user_idx_one_hot=40'h00_0200_0000;
    6'd26:i_user_idx_one_hot=40'h00_0400_0000;
    6'd27:i_user_idx_one_hot=40'h00_0800_0000;
    6'd28:i_user_idx_one_hot=40'h00_1000_0000;
    6'd29:i_user_idx_one_hot=40'h00_2000_0000;
    6'd30:i_user_idx_one_hot=40'h00_4000_0000;
    6'd31:i_user_idx_one_hot=40'h00_8000_0000;
    6'd32:i_user_idx_one_hot=40'h01_0000_0000;
    6'd33:i_user_idx_one_hot=40'h02_0000_0000;
    6'd34:i_user_idx_one_hot=40'h04_0000_0000;
    6'd35:i_user_idx_one_hot=40'h08_0000_0000;
    6'd36:i_user_idx_one_hot=40'h10_0000_0000;
    6'd37:i_user_idx_one_hot=40'h20_0000_0000;
    6'd38:i_user_idx_one_hot=40'h40_0000_0000;
    6'd39:i_user_idx_one_hot=40'h80_0000_0000;
    default: i_user_idx_one_hot=40'h00_0000_0000;
  endcase
end

endmodule
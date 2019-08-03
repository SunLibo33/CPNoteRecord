////////////////////////////////////////////////////////////////////////////////
// Company: <Casa Systems>
// Engineer: <Libo Sun>
//
// Create Date: <2019 Aug 01>
// Design Name: <Dual Port SRAM With Byte Enable>
// Description: Data with = 1152 , Address space from 0 to 2047
// Version    : V0.1
////////////////////////////////////////////////////////////////////////////////
				
module rdm_dpsram
#(parameter DATA_WIDTH=1152, parameter ADDR_WIDTH=11)
(
	input [(DATA_WIDTH-1):0] dina,
	input [(ADDR_WIDTH-1):0] addrb, addra,
    input [143:0]            wea,
	input clkb, clka,
	output reg [(DATA_WIDTH-1):0] doutb
);
	
    reg [(DATA_WIDTH-1):0] doutb_internal_reg;    
	// Declare the RAM variable
	reg [DATA_WIDTH-1:0] ram[2**ADDR_WIDTH-1:0];    

    generate // Write
    genvar i;
      for(i=0;i<144;i=i+1)
        begin:ByteEnableWriteCtrl
	      always @ (posedge clka)
	        begin		      
		      if (wea[i]==1'b1)
			    ram[addra][(i*8+7):(i*8)] <= dina[(i*8+7):(i*8)];
	        end
        end
    endgenerate
	
	always @ (posedge clkb) // Read 
	begin
		doutb_internal_reg <= ram[addrb];
        doutb              <= doutb_internal_reg;
	end
	
endmodule
// Description
// Gcrc24A and Gcrc24B of CRC decoder
// Version: v1.0
// version 1.1
// Changed the computation so that it accepts input bits in pair
module	pdsch_decoder_crc_decoder_crc24a_pair
			(
   		input			i_sys_200_clk,
   		input      		i_rstn,

		input			i_crc_decode_start,
   		input [1:0]    		i_din, 			// LSB is 1st bit and MSB is the 2nd bit
   		input      		i_parity_vld,
   		input      		i_din_vld,

   		output 	reg 		o_crc_status		//1: fail;0: correct 
			);

//-----------------------------------------
//variable
//-----------------------------------------
reg	[23:0]	c;              // crc registers
wire    [ 1:0]  d;              // crc decoder input in pair
wire	[ 1:0]	golden_parity;	// computed parity bits in pair

//-----------------------------------------
//logic
//-----------------------------------------
assign d = i_din;
   
always @ (posedge i_sys_200_clk or negedge i_rstn)
	if (!i_rstn) 
		c <= 24'd0;
	else if (i_crc_decode_start)
		c <= 24'd0;
	else if (i_din_vld & ~i_parity_vld) begin
		// G(x) = X^24 + X^23 + X^18 + X^17 + X^14 + X^11 + X^10 + X^7 + X^6 + X^5 + X^4 + X^3 + X + 1
		c[23] <= d[1] ^ d[0] ^ c[23] ^ c[22] ^ c[21];
		c[22] <= c[20];
		c[21] <= c[19];
		c[20] <= c[18];
		c[19] <= d[0] ^ c[23] ^ c[17];
		c[18] <= d[1] ^ c[22] ^ c[16];
		c[17] <= d[1] ^ d[0] ^ c[23] ^ c[22] ^ c[15];
		c[16] <= c[14];
		c[15] <= d[0] ^ c[23] ^c[13];
		c[14] <= d[1] ^ d[0] ^ c[23] ^ c[22] ^ c[12];
		c[13] <= c[11];
		c[12] <= d[0] ^ c[23] ^ c[10];
		c[11] <= d[1] ^ c[22] ^ c[9];
		c[10] <= d[1] ^ d[0] ^ c[23] ^ c[22] ^ c[8];
		c[9]  <= c[7];
		c[8]  <= d[0] ^ c[23] ^ c[6];
		c[7]  <= d[1] ^ c[22] ^ c[5];
		c[6]  <= d[1] ^ c[22] ^ c[4];
		c[5]  <= d[1] ^ c[22] ^ c[3];
		c[4]  <= d[1] ^ c[22] ^ c[2];
		c[3]  <= d[1] ^ d[0] ^ c[23] ^ c[22] ^ c[1];
		c[2]  <= d[0] ^ c[23] ^ c[0];
		c[1]  <= d[1] ^ c[23] ^ c[22] ^ c[23];
		c[0]  <= d[1] ^ d[0] ^ c[23] ^ c[22];
	end
	else if (i_din_vld & i_parity_vld) begin // flushing the shift registers
		c[23] <= c[21];
		c[22] <= c[20];
		c[21] <= c[19];
		c[20] <= c[18];
		c[19] <= c[17];
		c[18] <= c[16];
		c[17] <= c[15];
		c[16] <= c[14];
		c[15] <= c[13];
		c[14] <= c[12];
	       	c[13] <= c[11];
		c[12] <= c[10];
		c[11] <= c[9];
		c[10] <= c[8];
		c[9]  <= c[7];
		c[8]  <= c[6];
		c[7]  <= c[5];
		c[6]  <= c[4];
		c[5]  <= c[3];
		c[4]  <= c[2];
		c[3]  <= c[1];
		c[2]  <= c[0];
		c[1]  <= 1'b0;
		c[0]  <= 1'b0; 
	end		

assign	golden_parity = i_parity_vld ? {c[22], c[23]} : 2'd0;

assign	parity_bit_error = i_parity_vld & (i_din != golden_parity);  

always @ (posedge i_sys_200_clk or negedge i_rstn)
	if (!i_rstn)
		o_crc_status <= 1'd0;
	else if (i_crc_decode_start)
		o_crc_status <= 1'd0;
	else if (parity_bit_error)
		o_crc_status <= 1'd1;

endmodule


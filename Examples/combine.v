// Description
// This module is for combine HARQ read data and current descramble data
//    


module combine (

  // global signal
  input                         i_rst_n,
  input                         i_harq_clk,

  // input configuration
  input [2-1:0]                 i_dibit_mode,               // 00 : normal, 01 : master ,10 slave

  // interface with descr_buf
  input                         i_descr_buf_data_strb,      // descramble data strobe
  input [8-1:0]                 i_descr_buf_data,           // descramble data
  
  // interface with slave to rcombine
  input [8-1:0]                 i_rcombine_data,            // read combine  data  
  input [8-1:0]                 i_slave_rcombine_data,       // slave read combine data for channel 1 only

  // interface with wcombine
  output reg                    o_combine_data_strb,        // combine data strobe
  output reg [8-1:0]            o_combine_data              // combine data
 
 
);

  //---------------------------------------------------------------------------
  // Parameter declaration
  //---------------------------------------------------------------------------

  
  //---------------------------------------------------------------------------
  // Internal wire declaration
  //---------------------------------------------------------------------------
  reg [8-1:0]                   sync_descr_buf_data;
  reg                           sync_descr_buf_data_strb;
  wire [8-1:0]                  int_rcombine_data; // selected read combine data
  wire [9-1:0]                  int_combine_data;  // combined data before clipping

  //---------------------------------------------------------------------------
  // Sequential reg declaration
  //---------------------------------------------------------------------------


  //---------------------------------------------------------------------------
  // Instantiation of submodules 
  //---------------------------------------------------------------------------


  //---------------------------------------------------------------------------
  // Combinational logic
  //---------------------------------------------------------------------------
  assign int_rcombine_data = (i_dibit_mode == 2'b10) ? i_slave_rcombine_data : i_rcombine_data;
  assign int_combine_data = {sync_descr_buf_data[7], sync_descr_buf_data}  + {int_rcombine_data[7], int_rcombine_data};

  //---------------------------------------------------------------------------
  // Flip-flops
  //---------------------------------------------------------------------------
  // the rcombine data is triggered by descr_buf_data_strb, so the descr_buf_data/ strobe need to delay 1 cycle from combining
  always @ (posedge i_harq_clk or negedge i_rst_n)
  if (!i_rst_n) begin
	  sync_descr_buf_data <= 8'h00;
	  sync_descr_buf_data_strb <= 1'b0;
  end
  else if(i_descr_buf_data_strb)begin
	  sync_descr_buf_data <= i_descr_buf_data;
	  sync_descr_buf_data_strb <= 1'b1;
  end
  else begin
	  sync_descr_buf_data <= sync_descr_buf_data;
	  sync_descr_buf_data_strb <= 1'b0;
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    o_combine_data <= 8'b0; // keep normal
  end else begin
    case(int_combine_data[8:7]) // clipping
      2'b00 : o_combine_data <= int_combine_data[7:0]; // 0 to 127 = keep normal
      2'b01 : o_combine_data <= 8'b0111_1111; // > 127  = 127
      2'b10 : o_combine_data <= 8'b1000_0001; // < -128  = -127
      2'b11 : begin
        if (int_combine_data[6:0] == 7'b000_0000) begin // = -128
          o_combine_data <= 8'b1000_0001; // = -127
        end else begin // -127 to -1
          o_combine_data <= int_combine_data[7:0]; // keep normal
        end
      end
    endcase
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    o_combine_data_strb <= 1'b0;
  end else begin
    o_combine_data_strb <= sync_descr_buf_data_strb;    
  end

endmodule

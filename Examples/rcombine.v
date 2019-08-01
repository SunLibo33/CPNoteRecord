// Description : 
// This module is for reading data from the local Ncb ping-pong buffer to combine module 
// which would perform HARQ combing
//    
// Creation Date: 2011/07/18
// Version: v1.0
// Remark: for RWAIT_BUF_R0/1 and RBUF_R0/1 need to merge for the behavior in dibit_mode=0 and dibit_mode=1
//
// Revision Data : 2013/02/23
// Version v1.2
// add the o_harq0_combine_start and o_harq0_combine_start output port 
//
// Revision Data : 2014/01/08
// Version v1.3
// correct the ndi_cnt counter in the boundary case for Ncb is divisibled by 8.

module rcombine (

  // global signal
  input                         i_rst_n,
  input                         i_harq_clk,

  // rx ctrl input control
  input                         i_harq_start,
  input                         i_harq_end,

  // input configuration
  input                         i_dibit_en,         // 0 : normal , 1 : enable dibit
  input [5-1:0]                 i_cb_num,           // number of CB
  input [26-1:0]                i_e_bmp_fmt,        // E bitmap format
  input [17-1:0]                i_e0_size,          // E0 size
  input [17-1:0]                i_e1_size,          // E1 size
  input [15-1:0]                i_k0_pos,           // K0 position
  input                         i_ndi,              // New data indicator
  input [15-1:0]                i_ncb_size,         // Ncb size

  // interface with descr_buf
  input                         i_descr_buf_data_strb,      // descramble data strobe
  output                        o_rcombine_hold_req,        // request to hold few cycle at the boundary of Ncb or E
  output                        o_rcombine_rdy,             // read combine data ready
  
  // interface with fetch Ncb
  input                         i_fetch0_done,
  input                         i_fetch1_done,

  // interface with combine
  output reg                    o_rcombine_data_strb,       // rcombine data strobe
  output reg [8-1:0]            o_rcombine_data,            // rcombine data  
  output reg [8-1:0]            o_slave_rcombine_data,      // slave rcombine data

  // control signal to wcombine
  output reg                    o_combine_data_end,         // end of the combine data for wcombine indicate end of combine data in current CB

  // interface with Sto Ncb
  output                        o_harq0_combine_start,
  output                        o_harq1_combine_start,
  
  // interface with combine arbiter
  output reg                    o_rcombine_ren,
  output reg [12-1:0]           o_rcombine_addr,
  input  [64-1:0]               i_rcombine_rdata,
  
  
  // error control
  output reg                    o_rcombine_err
  
);

  //---------------------------------------------------------------------------
  // Parameter declaration
  //---------------------------------------------------------------------------
  parameter RCOMBINE_SWIDTH =4;

  parameter RCOMBINE_IDLE               = 4'b0000;
  parameter RCOMBINE_START              = 4'b0001;
  parameter RCOMBINE_WAIT_FETCH         = 4'b0010;
  parameter RCOMBINE_RBUF_R0_START      = 4'b0011;

  parameter RCOMBINE_RBUF_R1_START_DLY  = 4'b0100;
  parameter RCOMBINE_RBUF_R1_START      = 4'b0101;
  parameter RCOMBINE_RWAIT_COMBINE      = 4'b0110;

  parameter RCOMBINE_RWAIT_BUF_R0       = 4'b0111;
  parameter RCOMBINE_RBUF_R0            = 4'b1000;
  parameter RCOMBINE_RWAIT_BUF_R1       = 4'b1001;
  parameter RCOMBINE_RBUF_R1            = 4'b1010;
  
  parameter RCOMBINE_RD_E_END           = 4'b1011;
  parameter RCOMBINE_RD_E_END_HOLD      = 4'b1100;
  parameter RCOMBINE_RD_TB_END          = 4'b1101;
  parameter RCOMBINE_ERR                = 4'b1110;
  
  parameter MAX_HOLD = 4'b1111;
  parameter DATA_WIDTH = 8;
  
  //---------------------------------------------------------------------------
  // Internal wire declaration
  //---------------------------------------------------------------------------
  wire [4-1:0]                  slave_k0_pos;                   // slave k0 position = k0 + 1; for dibit mode only

  //---------------------------------------------------------------------------
  // Sequential reg declaration
  //---------------------------------------------------------------------------
  reg [RCOMBINE_SWIDTH-1:0]     rcombine_cs, rcombine_ns;       // rcombine ncb control state machine
  reg [5-1:0]                   cb_num_cnt;                     // cb number counter
  
  reg                           fetch0_rdy;                     // ready for fetch Ncb buffer 0
  reg                           fetch1_rdy;                     // ready for fetch Ncb buffer 1
  reg                           fetch0_clr;                     // clear for fetch Ncb buffer 0 ready signal
  reg                           fetch1_clr;                     // clear for fetch Ncb buffer 1 ready signal
  reg                           ncb_ptr;                        // indicate the current ping-pong Ncb buffer

  reg [12-1:0]                  ncb_8;                          // ceiling Ncb / 8 for 8 byte alignment
  reg [12-1:0]                  k0_8;                           // floor k0 / 8 for 8 byte alignment
  reg                           buf_r0_en;                      // local Read Buffer 0 enable
  reg                           buf_r1_en;                      // local Read Buffer 1 enable
  reg                           buf_r0_en_dly;
  reg                           buf_r1_en_dly;
  reg [64-1:0]                  buf_r0;                         // local 64 bit buffer 0 prefetech for harq combine
  reg [64-1:0]                  buf_r1;                         // local 64 bit buffer 1 prefetech for harq combine
  reg                           empty_rbuf_r0;                  // Empty read buffer 0
  reg                           empty_rbuf_r1;                  // Empty read buffer 1

  reg [17-1:0]                  e_cnt;                          // Counter for counting in descr data strb
  reg [15-1:0]                  ncb_cnt;                        // Counter for counting in Ncb boundary position of the input data
  reg [15-1:0]                  slave_ncb_cnt;                  // Slave Counter for counting in Ncb boundary position of the input data
    
  reg                           rcombine_ini;                   // Read combine counter initial signal to reset the e_cnt counter
  
//  reg                           ncb_cnt_eq_end;                 // end of the ncb signal before the last descr_data coming in
  reg                           e_cnt_eq_end;                   // end of the E signal before the last descr_data coming in
  reg [4-1:0]                   rptr;                           // normal read pointer of the local buffer
  reg [4-1:0]                   slave_rptr;                     // slave read pointer of the local buffer
  reg [4-1:0]                   end_hold_cnt;                   // hold counter is the end of the E of current Ncb
  reg [26-1:0]                  int_e_bmp_fmt;                  // internal E bitmap format for choosing E1 or E2
  reg [17-1:0]                  e_size;                         // E size in current Ncb
  reg [17-1:0]                  ndi_cnt;                        // NDI data count which store to NCB buffer;

  reg                           int_rcombine_rdy;
  reg                           ncb_last_qword_en;
  //---------------------------------------------------------------------------
  // Instantiation of submodules 
  //---------------------------------------------------------------------------


  //---------------------------------------------------------------------------
  // Combinational logic
  //---------------------------------------------------------------------------
  assign slave_k0_pos = {1'b0, i_k0_pos[2:0]} + 4'b1;
  assign o_rcombine_hold_req = i_descr_buf_data_strb & (e_cnt_eq_end | ncb_last_qword_en);
  //2012-02-22, 10:33:17 AM
  assign o_rcombine_rdy = int_rcombine_rdy;
  assign o_harq0_combine_start = fetch0_clr;
  assign o_harq1_combine_start = fetch1_clr;
  
  always @ (*) begin
    if (i_harq_end & (rcombine_cs != RCOMBINE_IDLE)) begin
      rcombine_ns = RCOMBINE_ERR;
    end else begin
      rcombine_ns = rcombine_cs;
      case(rcombine_cs)
        RCOMBINE_IDLE : begin
          if (i_harq_start & (i_cb_num != 5'd0)) begin
            rcombine_ns = RCOMBINE_START;
          end
        end
        RCOMBINE_START : begin
          rcombine_ns = RCOMBINE_WAIT_FETCH;
        end
        RCOMBINE_WAIT_FETCH : begin
          if ((fetch0_rdy & !ncb_ptr) | (fetch1_rdy & ncb_ptr)) begin
            rcombine_ns = RCOMBINE_RBUF_R0_START;
          end
        end
        RCOMBINE_RBUF_R0_START : begin
          rcombine_ns = RCOMBINE_RBUF_R1_START_DLY;
        end
        RCOMBINE_RBUF_R1_START_DLY : begin
          rcombine_ns = RCOMBINE_RBUF_R1_START;
        end
        RCOMBINE_RBUF_R1_START : begin
          rcombine_ns = RCOMBINE_RWAIT_COMBINE;
        end
        RCOMBINE_RWAIT_COMBINE : begin
          rcombine_ns = RCOMBINE_RWAIT_BUF_R0;
        end
        RCOMBINE_RWAIT_BUF_R0 : begin
	  if(i_descr_buf_data_strb & ((e_cnt == e_size - 17'b1) | ((e_cnt == e_size - 17'd2) & i_dibit_en)) ) begin
	     rcombine_ns = RCOMBINE_RD_E_END;
	  end else if (empty_rbuf_r0) begin
	     rcombine_ns = RCOMBINE_RBUF_R0;
	  end
        end
        RCOMBINE_RBUF_R0 : begin
	  if(i_descr_buf_data_strb & ((e_cnt == e_size - 17'b1) | ((e_cnt == e_size - 17'd2) & i_dibit_en)) ) begin
             rcombine_ns = RCOMBINE_RD_E_END;
          end else begin
             rcombine_ns = RCOMBINE_RWAIT_BUF_R1;
          end
        end
        RCOMBINE_RWAIT_BUF_R1 : begin
	  if(i_descr_buf_data_strb & ((e_cnt == e_size - 17'b1) | ((e_cnt == e_size - 17'd2) & i_dibit_en)) ) begin
            rcombine_ns = RCOMBINE_RD_E_END;
	  end else if(empty_rbuf_r1) begin
	    rcombine_ns = RCOMBINE_RBUF_R1;
          end           
        end
        RCOMBINE_RBUF_R1 : begin
	  if(i_descr_buf_data_strb & ((e_cnt == e_size - 17'b1) | ((e_cnt == e_size - 17'd2) & i_dibit_en)) ) begin
            rcombine_ns = RCOMBINE_RD_E_END;
	  end else begin
            rcombine_ns = RCOMBINE_RWAIT_BUF_R0;
	  end
        end
        RCOMBINE_RD_E_END : begin
          rcombine_ns = RCOMBINE_RD_E_END_HOLD;
        end
        RCOMBINE_RD_E_END_HOLD : begin
          if (end_hold_cnt==4'b0000) begin
            if (cb_num_cnt != i_cb_num ) begin
              rcombine_ns = RCOMBINE_WAIT_FETCH;
            end else begin
              rcombine_ns = RCOMBINE_RD_TB_END;
            end
          end
        end
        RCOMBINE_RD_TB_END : begin
          rcombine_ns = RCOMBINE_IDLE;
        end
        RCOMBINE_ERR : begin
          rcombine_ns = RCOMBINE_IDLE;
        end
        default : begin
          rcombine_ns = RCOMBINE_IDLE;
        end
      endcase
    end
  end


  //---------------------------------------------------------------------------
  // Flip-flops
  //---------------------------------------------------------------------------
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    rcombine_cs <= RCOMBINE_IDLE;
  end else begin
    rcombine_cs <= rcombine_ns;
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    fetch0_rdy <= 1'b0;
  end else begin
    if (i_harq_start) begin
      fetch0_rdy <= 1'b0;
    end else if (fetch0_clr) begin
      fetch0_rdy <= 1'b0;
    end else if (i_fetch0_done) begin
      fetch0_rdy <= 1'b1;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    fetch1_rdy <= 1'b0;
  end else begin
    if (i_harq_start) begin
      fetch1_rdy <= 1'b0;
    end else if (fetch1_clr) begin
      fetch1_rdy <= 1'b0;
    end else if (i_fetch1_done) begin
      fetch1_rdy <= 1'b1;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_r0_en_dly <= 1'b0;
    buf_r1_en_dly <= 1'b0;
  end else begin
    buf_r0_en_dly <= buf_r0_en;
    buf_r1_en_dly <= buf_r1_en;
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_r0 <= 64'b0;
  end else begin
    if (i_ndi & ((ndi_cnt + 17'b1000) < i_ncb_size)) begin
      buf_r0 <= 64'b0;
    end else if (buf_r0_en_dly) begin
      buf_r0 <= i_rcombine_rdata;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_r1 <= 64'b0;
  end else begin
    if (i_ndi & ((ndi_cnt + 17'b1000) < i_ncb_size)) begin
      buf_r1 <= 64'b0;
    end else if (buf_r1_en_dly) begin
      buf_r1 <= i_rcombine_rdata;
    end
  end
  
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    o_rcombine_data_strb <= 1'b0;
  end else begin
    o_rcombine_data_strb <= i_descr_buf_data_strb;
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    o_rcombine_data <= 8'b0;
  end else begin
    if (i_descr_buf_data_strb) begin
      case(rptr)
        4'd0 : o_rcombine_data <= buf_r0[1*DATA_WIDTH-1: 0];
        4'd1 : o_rcombine_data <= buf_r0[2*DATA_WIDTH-1: 1*DATA_WIDTH];
        4'd2 : o_rcombine_data <= buf_r0[3*DATA_WIDTH-1: 2*DATA_WIDTH];
        4'd3 : o_rcombine_data <= buf_r0[4*DATA_WIDTH-1: 3*DATA_WIDTH];
        4'd4 : o_rcombine_data <= buf_r0[5*DATA_WIDTH-1: 4*DATA_WIDTH];
        4'd5 : o_rcombine_data <= buf_r0[6*DATA_WIDTH-1: 5*DATA_WIDTH];
        4'd6 : o_rcombine_data <= buf_r0[7*DATA_WIDTH-1: 6*DATA_WIDTH];
        4'd7 : o_rcombine_data <= buf_r0[8*DATA_WIDTH-1: 7*DATA_WIDTH];
        4'd8 : o_rcombine_data <=  buf_r1[1*DATA_WIDTH-1: 0];
        4'd9 : o_rcombine_data <=  buf_r1[2*DATA_WIDTH-1: 1*DATA_WIDTH];
        4'd10 : o_rcombine_data <= buf_r1[3*DATA_WIDTH-1: 2*DATA_WIDTH];
        4'd11 : o_rcombine_data <= buf_r1[4*DATA_WIDTH-1: 3*DATA_WIDTH];
        4'd12 : o_rcombine_data <= buf_r1[5*DATA_WIDTH-1: 4*DATA_WIDTH];
        4'd13 : o_rcombine_data <= buf_r1[6*DATA_WIDTH-1: 5*DATA_WIDTH];
        4'd14 : o_rcombine_data <= buf_r1[7*DATA_WIDTH-1: 6*DATA_WIDTH];
        4'd15 : o_rcombine_data <= buf_r1[8*DATA_WIDTH-1: 7*DATA_WIDTH];
        default :  o_rcombine_data <= buf_r0[1*DATA_WIDTH-1: 0];
      endcase
    end
  end
  
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    o_slave_rcombine_data <= 8'b0;
  end else begin
    if (i_descr_buf_data_strb) begin
      case(slave_rptr)
        4'd0 : o_slave_rcombine_data <= buf_r0[1*DATA_WIDTH-1:0];
        4'd1 : o_slave_rcombine_data <= buf_r0[2*DATA_WIDTH-1:1*DATA_WIDTH];
        4'd2 : o_slave_rcombine_data <= buf_r0[3*DATA_WIDTH-1:2*DATA_WIDTH];
        4'd3 : o_slave_rcombine_data <= buf_r0[4*DATA_WIDTH-1:3*DATA_WIDTH];
        4'd4 : o_slave_rcombine_data <= buf_r0[5*DATA_WIDTH-1:4*DATA_WIDTH];
        4'd5 : o_slave_rcombine_data <= buf_r0[6*DATA_WIDTH-1:5*DATA_WIDTH];
        4'd6 : o_slave_rcombine_data <= buf_r0[7*DATA_WIDTH-1:6*DATA_WIDTH];
        4'd7 : o_slave_rcombine_data <= buf_r0[8*DATA_WIDTH-1:7*DATA_WIDTH];
        4'd8 : o_slave_rcombine_data <=  buf_r1[1*DATA_WIDTH-1:0];
        4'd9 : o_slave_rcombine_data <=  buf_r1[2*DATA_WIDTH-1:1*DATA_WIDTH];
        4'd10 : o_slave_rcombine_data <= buf_r1[3*DATA_WIDTH-1:2*DATA_WIDTH];
        4'd11 : o_slave_rcombine_data <= buf_r1[4*DATA_WIDTH-1:3*DATA_WIDTH];
        4'd12 : o_slave_rcombine_data <= buf_r1[5*DATA_WIDTH-1:4*DATA_WIDTH];
        4'd13 : o_slave_rcombine_data <= buf_r1[6*DATA_WIDTH-1:5*DATA_WIDTH];
        4'd14 : o_slave_rcombine_data <= buf_r1[7*DATA_WIDTH-1:6*DATA_WIDTH];
        4'd15 : o_slave_rcombine_data <= buf_r1[8*DATA_WIDTH-1:7*DATA_WIDTH];
        default :  o_slave_rcombine_data <= buf_r0[1*DATA_WIDTH-1:0];
      endcase
    end
  end

  // monitor counter for e size checking
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    e_cnt <= 17'd0;
  end else begin
    if (rcombine_ini) begin
      e_cnt <= 17'd0;
    end else if (i_descr_buf_data_strb) begin
      if(i_dibit_en) begin
      	e_cnt <= e_cnt + 17'd2;
      end else begin
      	e_cnt <= e_cnt + 17'd1;
      end
    end
  end
  
  // send to the wcombine to end the Ncb at the last combine data
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    o_combine_data_end <= 1'b0;
  end else begin
    if (i_harq_start) begin
      o_combine_data_end <= 1'b0;
    end else if ((e_cnt == e_size - 17'b1) & i_descr_buf_data_strb ) begin
      o_combine_data_end <= 1'b1;
    end else begin
      o_combine_data_end <= 1'b0;
    end
  end

  // generate the end of E signal for holding descr_buf input
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    e_cnt_eq_end <= 1'b0;   
  end else begin
    if (!i_dibit_en) begin // normal mode
      if ((e_cnt == e_size - 17'd2) & i_descr_buf_data_strb) begin
        e_cnt_eq_end <= 1'b1;   
      end else if (e_cnt == e_size - 17'd1) begin
        e_cnt_eq_end <= 1'b1;   
      end else begin
        e_cnt_eq_end <= 1'b0;   
      end
    end else begin // dibit mode enable
      if ((e_cnt == e_size - 17'd4) & i_descr_buf_data_strb) begin
        e_cnt_eq_end <= 1'b1;   
      end else if (e_cnt == e_size - 17'd2) begin
        e_cnt_eq_end <= 1'b1;   
      end else begin
        e_cnt_eq_end <= 1'b0;   
      end
    end
  end

/*  // generate the end of Ncb signal for holding descr_buf input
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    ncb_cnt_eq_end <= 1'b0;   
  end else begin
    if (!i_dibit_en) begin // normal mode
      if ((ncb_cnt == i_ncb_size - 15'd2) & i_descr_buf_data_strb) begin
        ncb_cnt_eq_end <= 1'b1;   
      end else if (ncb_cnt == i_ncb_size - 15'd1) begin
        ncb_cnt_eq_end <= 1'b1;   
      end else begin
        ncb_cnt_eq_end <= 1'b0;   
      end
    end else begin // dibit mode enable
      if ((ncb_cnt == i_ncb_size - 15'd4) & i_descr_buf_data_strb) begin
        ncb_cnt_eq_end <= 1'b1;   
      end else if ((ncb_cnt == i_ncb_size - 15'd3) & i_descr_buf_data_strb) begin
	ncb_cnt_eq_end <= 1'b1;
      end else if (ncb_cnt == i_ncb_size - 15'd2) begin
        ncb_cnt_eq_end <= 1'b1;
      end else if (ncb_cnt == i_ncb_size - 15'd1) begin
        ncb_cnt_eq_end <= 1'b1;
      end else begin
        ncb_cnt_eq_end <= 1'b0;   
      end
    end
  end */

//2012-03-13, 02:13:59 PM corner case, hold 8 cycles when last NCB quarter word. most time last NCB quarter word is less than 8bytes valid data, 
//then it will cause state machine has no enough time to keep data buffer always ready for fetch.
  always @ (posedge i_harq_clk or negedge i_rst_n)
  if (!i_rst_n) begin
    ncb_last_qword_en <= 1'b0;
  end else begin
    if (!i_dibit_en) begin // normal mode
      if ((ncb_cnt == {i_ncb_size[14:3], 3'b000} - 15'd2) & i_descr_buf_data_strb) begin
	  ncb_last_qword_en <= 1'b1;
      end else if(ncb_cnt == {i_ncb_size[14:3], 3'b000} - 15'd1) begin
          ncb_last_qword_en <= 1'b1; //2012-03-16, 02:39:01 PM, Make sure it is clear in the last one, aviod false last one occurred if descramble buffer empty now.
      end else begin
          ncb_last_qword_en <= 1'b0;
      end
    end else begin //dibit mode enable
      if ((ncb_cnt == {i_ncb_size[14:3], 3'b000} - 15'd4) & i_descr_buf_data_strb) begin
          ncb_last_qword_en <= 1'b1;
      end else if ((ncb_cnt == {i_ncb_size[14:3], 3'b000} - 15'd3) & i_descr_buf_data_strb) begin
          ncb_last_qword_en <= 1'b1;
      end else if (ncb_cnt == {i_ncb_size[14:3], 3'b000} - 15'd2) begin
          ncb_last_qword_en <= 1'b1;
      end else if (ncb_cnt == {i_ncb_size[14:3], 3'b000} - 15'd1) begin
	  ncb_last_qword_en <= 1'b1;
      end else begin
          ncb_last_qword_en <= 1'b0;
      end
    end
  end

  // monitor counter for ncb boundary checking
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    ncb_cnt <= 15'b0;
  end else begin
    if (rcombine_ini) begin
      ncb_cnt <= i_k0_pos;
    end else if (i_descr_buf_data_strb) begin
      if (!i_dibit_en) begin // normal mode
        if (ncb_cnt == i_ncb_size - 15'b1) begin
          ncb_cnt <= 15'b0;
        end else begin
          ncb_cnt <= ncb_cnt + 15'd1;
        end
      end else begin // dibit mode enable
        if (ncb_cnt == i_ncb_size - 15'b1) begin
          ncb_cnt <= 15'b1;
        end else if (ncb_cnt == i_ncb_size - 15'd2) begin
          ncb_cnt <= 15'b0;
        end else begin
          ncb_cnt <= ncb_cnt + 15'd2;
        end
      end
    end
  end


  // slave monitor counter for ncb boundary checking for dibit mode only
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    slave_ncb_cnt <= 15'b0;
  end else if (i_dibit_en) begin 
    if (rcombine_ini) begin
      slave_ncb_cnt <= i_k0_pos + 15'b1;
    end else if (i_descr_buf_data_strb) begin
      if (!i_dibit_en) begin // normal mode
        if (slave_ncb_cnt == i_ncb_size - 15'b1) begin
          slave_ncb_cnt <= 15'b0;
        end else begin
          slave_ncb_cnt <= slave_ncb_cnt + 15'd1;
        end
      end else begin // dibit mode enable
        if (slave_ncb_cnt == i_ncb_size - 15'b1) begin
          slave_ncb_cnt <= 15'b1;
        end else if (slave_ncb_cnt == i_ncb_size - 15'd2) begin
          slave_ncb_cnt <= 15'b0;
        end else begin
          slave_ncb_cnt <= slave_ncb_cnt + 15'd2;
        end
      end
    end
  end

  // read pointer to select the local buffer 0 and 1 data as rcombine data
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    rptr <= 4'd0;
  end else begin
    if (rcombine_ini) begin
      rptr <= {1'b0,i_k0_pos[2:0]}; // initial the pointer start from local buffer 0 with alignment shift
    end else if (i_descr_buf_data_strb) begin
      if (!i_dibit_en) begin // normal mode
        if (ncb_cnt == i_ncb_size-15'd1) begin // switch to another local buffer at the 
          if (rptr[3]) begin
            rptr <= 4'd0;
          end else begin
            rptr <= 4'd8;
          end
        end else begin
          rptr <= rptr + 4'd1;
        end
      end else begin // dibit mode
        if (ncb_cnt == i_ncb_size-15'd1) begin // switch to another local buffer at the boundry of Ncb
          if (rptr[3]) begin
            rptr <= 4'd1;
          end else begin
            rptr <= 4'd9;
          end
        end else if (ncb_cnt == i_ncb_size - 15'd2) begin
          if (rptr[3]) begin
            rptr <= 4'd0;
          end else begin
            rptr <= 4'd8;
          end
        end else begin
          rptr <= rptr + 4'd2;
        end
      end
    end
  end

  // slave read pointer to select the local buffer 0 and 1 data as slave rcombine data
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    slave_rptr <= 4'd0;
  end else if (i_dibit_en) begin
    if (rcombine_ini) begin // initial the pointer start from local buffer 0 with alignment shift
      slave_rptr <= slave_k0_pos;
    end else if (i_descr_buf_data_strb) begin // only for dibit mode
      if (slave_ncb_cnt == i_ncb_size-15'd1) begin // switch to another local buffer at the boundry of Ncb
        if (slave_rptr[3]) begin
          slave_rptr <= 4'd1;
        end else begin
          slave_rptr <= 4'd9;
        end
      end else if (slave_ncb_cnt == i_ncb_size - 15'd2) begin
        if (slave_rptr[3]) begin
          slave_rptr <= 4'd0;
        end else begin
          slave_rptr <= 4'd8;
        end
      end else begin
        slave_rptr <= slave_rptr + 4'd2;
      end
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    empty_rbuf_r0 <= 1'b0;
  end else begin
    if (i_harq_start) begin
      empty_rbuf_r0 <= 1'b0;
    end else if (i_descr_buf_data_strb) begin
      if (!i_dibit_en) begin // normal case
        if (rptr == 4'd7) begin
          empty_rbuf_r0 <= 1'b1;
        end else if ((ncb_cnt == i_ncb_size - 15'd1) & (rptr < 4'd7)) begin
          empty_rbuf_r0 <= 1'b1;
        end else begin
          empty_rbuf_r0 <= 1'b0;
        end
      end else begin // dibit_en = 1 is the master of dibit mode
        if ((rptr == 4'd6)| (rptr == 4'd7)) begin
          empty_rbuf_r0 <= 1'b1;
        end else if (((ncb_cnt == i_ncb_size - 15'd1)| (ncb_cnt == i_ncb_size - 15'd2)) & (rptr < 4'd7)) begin
          empty_rbuf_r0 <= 1'b1;
        end else begin
          empty_rbuf_r0 <= 1'b0;
        end
      end
    end else begin
      empty_rbuf_r0 <= 1'b0;
    end
  end
  
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    empty_rbuf_r1 <= 1'b0;
  end else begin
    if (i_harq_start) begin
      empty_rbuf_r1 <= 1'b0;
    end else if (i_descr_buf_data_strb) begin
      if (!i_dibit_en) begin // normal case
        if (rptr == 4'd15) begin
          empty_rbuf_r1 <= 1'b1;
        end else if ((ncb_cnt == i_ncb_size - 15'd1) & (rptr > 4'd7)) begin
          empty_rbuf_r1 <= 1'b1;
        end else begin
          empty_rbuf_r1 <= 1'b0;
        end
      end else begin // dibit_en = 1 is the master of dibit mode
        if ((rptr == 4'd14)| (rptr == 4'd15)) begin
          empty_rbuf_r1 <= 1'b1;
        end else if (((ncb_cnt == i_ncb_size - 15'd1)| (ncb_cnt == i_ncb_size - 15'd2)) & (rptr > 4'd7)) begin
          empty_rbuf_r1 <= 1'b1;
        end else begin
          empty_rbuf_r1 <= 1'b0;
        end
      end
    end else begin
      empty_rbuf_r1 <= 1'b0;
    end
  end


  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    ncb_ptr <= 1'b0;
    cb_num_cnt <= 5'b0;
    fetch0_clr <= 1'b0;
    fetch1_clr <= 1'b0;
    end_hold_cnt <= 4'b0;
    rcombine_ini <= 1'b0;
    
    o_rcombine_ren <= 1'b0;
    o_rcombine_addr <= 12'b0;
    ndi_cnt <= 17'd0;
    o_rcombine_err <= 1'b0;
    int_rcombine_rdy <= 1'b0;

    buf_r0_en <= 1'b0;
    buf_r1_en <= 1'b0;
    e_size <= 17'b0;
    k0_8 <= 12'b0;
    ncb_8 <= 12'b0;
    
    int_e_bmp_fmt <= 26'b0;
  end else begin
    case(rcombine_cs)
      RCOMBINE_IDLE : begin
        ncb_ptr <= 1'b0;
        cb_num_cnt <= 5'b0;
        fetch0_clr <= 1'b0;
        fetch1_clr <= 1'b0;
        end_hold_cnt <= 4'b0;
        rcombine_ini <= 1'b0;
 
        o_rcombine_ren <= 1'b0;
        o_rcombine_addr <= 12'b0;
	ndi_cnt <= 17'd0;
        o_rcombine_err <= 1'b0;
        int_rcombine_rdy <= 1'b0;

        buf_r0_en <= 1'b0;
        buf_r1_en <= 1'b0;
        e_size <= 17'b0;
        k0_8 <= 12'b0;
        ncb_8 <= 12'b0;
 
        int_e_bmp_fmt <= 26'b0;
      end
      RCOMBINE_START : begin
        int_e_bmp_fmt <= i_e_bmp_fmt;
        k0_8 <= i_k0_pos[14:3];
        ncb_8 <= i_ncb_size[14:3] + (|i_ncb_size[2:0]);
	ndi_cnt <= 17'd0;
      end
      RCOMBINE_WAIT_FETCH : begin
        e_size <= int_e_bmp_fmt[0] ? i_e1_size : i_e0_size;
        o_rcombine_ren <= 1'b0;
        end_hold_cnt <= 4'b0;
      end
      RCOMBINE_RBUF_R0_START : begin
        o_rcombine_ren <= !i_ndi;
        o_rcombine_addr <= k0_8;
	ndi_cnt <= 17'd8  - {14'b0, i_k0_pos[2:0]};
	fetch0_clr <= !ncb_ptr;
        fetch1_clr <= ncb_ptr;
        buf_r0_en <= 1'b1;        
      end
      RCOMBINE_RBUF_R1_START_DLY : begin
        o_rcombine_ren <= 1'b0;
        fetch0_clr <= 1'b0;
        fetch1_clr <= 1'b0;
        buf_r0_en <= 1'b0;        
        rcombine_ini <= 1'b1;
      end
      RCOMBINE_RBUF_R1_START : begin
        o_rcombine_ren <= !i_ndi | ((ndi_cnt + 17'b1000) > i_ncb_size);
        if (o_rcombine_addr != ncb_8 - 12'b1) begin
          o_rcombine_addr <= o_rcombine_addr + 12'b1;
        end else begin
          o_rcombine_addr <= 12'b0;
        end
	if (o_rcombine_addr != ncb_8 - 12'd2) begin
	  ndi_cnt <= ndi_cnt + 17'b1000;
	end else begin
	  ndi_cnt <= ndi_cnt + {14'b0, i_ncb_size[2:0]};
	end
        buf_r1_en <= 1'b1;
        rcombine_ini <= 1'b0;
      end
      RCOMBINE_RWAIT_COMBINE : begin
        int_rcombine_rdy <= 1'b1;
        buf_r1_en <= 1'b0;
      end
      RCOMBINE_RWAIT_BUF_R0 : begin
        //o_rcombine_ren <= 1'b0;
        buf_r1_en <= 1'b0;
	if(i_dibit_en & empty_rbuf_r0) begin
	  o_rcombine_ren <= !i_ndi | ((ndi_cnt + 17'b1000) > i_ncb_size);
	  buf_r0_en <= !i_ndi | ((ndi_cnt + 17'b1000) > i_ncb_size);
	  if(o_rcombine_addr != ncb_8 - 12'b1) begin
	    o_rcombine_addr <= o_rcombine_addr + 12'b1;
	  end else begin
	    o_rcombine_addr <= 12'b0;
	  end 
          if ((ndi_cnt + 17'b1000) > i_ncb_size) begin 
	    ndi_cnt <= ndi_cnt;
          end else begin
            if(o_rcombine_addr != ncb_8 - 12'd2) begin
	      ndi_cnt <= ndi_cnt + 17'b1000;
            end else begin
	      ndi_cnt <= ndi_cnt + {13'b0, ~|i_ncb_size[2:0], i_ncb_size[2:0]};
	    end
          end
	end else begin 
	  o_rcombine_ren <= 1'b0;
	end
      end
      RCOMBINE_RBUF_R0 : begin
        if(!i_dibit_en) begin
	  o_rcombine_ren <= !i_ndi | ((ndi_cnt + 17'b1000) > i_ncb_size);
          buf_r0_en <= !i_ndi | ((ndi_cnt + 17'b1000) > i_ncb_size);
          if (o_rcombine_addr != ncb_8 - 12'b1) begin
            o_rcombine_addr <= o_rcombine_addr + 12'b1;
          end else begin
            o_rcombine_addr <= 12'b0;
          end
          if ((ndi_cnt + 17'b1000) > i_ncb_size) begin 
	    ndi_cnt <= ndi_cnt;
          end else begin
	    if (o_rcombine_addr != ncb_8 - 12'd2) begin
	      ndi_cnt <= ndi_cnt + 17'b1000;
	    end else begin
	      ndi_cnt <= ndi_cnt + {13'b0, ~|i_ncb_size[2:0], i_ncb_size[2:0]};
	    end
          end
          
	end else begin
	  o_rcombine_ren <= 1'b0;  //2012-03-12, 03:06:23 PM
	  buf_r0_en <= 1'b0;
	end
      end
      RCOMBINE_RWAIT_BUF_R1: begin
        //o_rcombine_ren <= 1'b0;
        buf_r0_en <= 1'b0;
	if(i_dibit_en & empty_rbuf_r1) begin
	  o_rcombine_ren <= !i_ndi | ((ndi_cnt + 17'b1000) > i_ncb_size);
	  buf_r1_en <= !i_ndi | ((ndi_cnt + 17'b1000) > i_ncb_size);
	  if (o_rcombine_addr != ncb_8 - 12'b1) begin
	       o_rcombine_addr <= o_rcombine_addr + 12'b1;
	  end else begin
	       o_rcombine_addr <= 12'b0;
	  end
          if ((ndi_cnt + 17'b1000) > i_ncb_size) begin 
	    ndi_cnt <= ndi_cnt;
          end else begin
            if (o_rcombine_addr != ncb_8 - 12'd2) begin
              ndi_cnt <= ndi_cnt + 17'b1000;
            end else begin
              ndi_cnt <= ndi_cnt + {13'b0, ~|i_ncb_size[2:0], i_ncb_size[2:0]};
            end
          end
	end else begin
	   o_rcombine_ren <= 1'b0;
	end //dibit mode, read buffer is bottleneck and need to read data 1T ahead.
      end
      RCOMBINE_RBUF_R1 : begin
        if(!i_dibit_en) begin
          o_rcombine_ren <= !i_ndi | ((ndi_cnt + 17'b1000) > i_ncb_size);
          buf_r1_en <= !i_ndi | ((ndi_cnt + 17'b1000) > i_ncb_size);
          if (o_rcombine_addr != ncb_8 - 12'b1) begin
            o_rcombine_addr <= o_rcombine_addr + 12'b1;
          end else begin
            o_rcombine_addr <= 12'b0;
          end
          if ((ndi_cnt + 17'b1000) > i_ncb_size) begin 
	    ndi_cnt <= ndi_cnt;
          end else begin
            if (o_rcombine_addr != ncb_8 - 12'd2) begin
              ndi_cnt <= ndi_cnt + 17'b1000;
              end else begin
	      ndi_cnt <= ndi_cnt + {13'b0, ~|i_ncb_size[2:0], i_ncb_size[2:0]};
            end
          end
	end else begin
	  o_rcombine_ren <= 1'b0;   //2012-03-12, 03:06:37 PM
	  buf_r1_en <= 1'b0;
	end
      end
      RCOMBINE_RD_E_END : begin
        int_e_bmp_fmt <= {1'b0, int_e_bmp_fmt[25:1]};
        buf_r0_en <= 1'b0;
        buf_r1_en <= 1'b0;
        ncb_ptr <= !ncb_ptr;
        cb_num_cnt <= cb_num_cnt + 5'b1;
        end_hold_cnt <= MAX_HOLD;
        int_rcombine_rdy <= 1'b0;
      end
      RCOMBINE_RD_E_END_HOLD : begin
        end_hold_cnt <= end_hold_cnt - 4'b1;
      end
      RCOMBINE_RD_TB_END : begin
        end_hold_cnt <= 4'b0;
      end
      RCOMBINE_ERR : begin
        o_rcombine_err <= 1'b1;
      end
      default : begin
        ncb_ptr <= 1'b0;
        cb_num_cnt <= 5'b0;
        fetch0_clr <= 1'b0;
        fetch1_clr <= 1'b0;
        end_hold_cnt <= 4'b0;
        rcombine_ini <= 1'b0;
 
        o_rcombine_ren <= 1'b0;
        o_rcombine_addr <= 12'b0;
	ndi_cnt <= 17'd0;
        o_rcombine_err <= 1'b0;
        int_rcombine_rdy <= 1'b0;

        buf_r0_en <= 1'b0;
        buf_r1_en <= 1'b0;
        e_size <= 17'b0;
        k0_8 <= 12'b0;
        ncb_8 <= 12'b0;
 
        int_e_bmp_fmt <= 26'b0;
      end
    endcase
  end




endmodule

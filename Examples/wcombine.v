// Description
// This module is for writing the combined data into the local Ncb ping-pong buffer
//    
// Creation Date: 2011/07/18
// Version: v1.0
//
// Version v1.1
// Revision Data : 2013/02/22
// modify for fine clock gating
//
// Revision Data : 2013/02/23
// Version v1.2
// delete the o_harq0_combine_start and o_harq0_combine_start output port 
//
// Revision Data : 2013/02/28
// Version v1.2
// 1) reset burst counter when i_harq_start
// 2) add the i_harq_lb_burst_len control function.
//
// Revision Data : 2014/01/10
// Version v1.3
// 1) Fixed the k0e0_mod_ncb and k0e0_mod_cnt and k0e1_mod_ncb and k0e1_mod_cnt evaluation 
// when k0e0_mod_ncb = i_ncb_size
//
// Revision Data : 2014/01/15
// Version v1.4
// 1) When k0e_mod_ncb_8 == ncb_8 in mode 2, it implies e1 region is zero. It don't need to assert e1_burst_incr signal
//
//
// Revision Data : 2014/01/21
// Version v1.5
// 1) add restrict e0 incr at the end of the write data position after the start position.
//


module wcombine (

  // global signal
  input                         i_rst_n,
  input                         i_harq_clk,

  // rx ctrl input control
  input                         i_harq_start,
  input                         i_harq_end,

  // input configuration
  input                         i_dibit_en,          // 0 : normal , 1 : enable dibit
  input [5-1:0]                 i_cb_num,            // number of CB
  input [26-1:0]                i_e_bmp_fmt,         // E bitmap format
  input [17-1:0]                i_e0_size,           // E1 size
  input [17-1:0]                i_e1_size,           // E2 size
  input [15-1:0]                i_k0_pos,            // K0 position
  input [15-1:0]                i_ncb_size,          // Ncb size
  input [2-1:0]                 i_harq_lb_burst_len, // HARQ local bus interface burst length 0-16, 1-32, 2-64

  // interface with combine
  input                         i_combine_data_strb,    // combine data strobe
  input [8-1:0]                 i_combine_data,         // combine data

  // interface with TB 2 combine output
  input [8-1:0]                 i_slave_combine_data,   // slave combine data  
  
  // control signal with  rcombine
//  input                         i_combine_data_end,  // end of the combine data for wcombine indicate end of combine data in current CB

  // interface with Sto Ncb and combine arbiter
  output reg                    o_harq0_done,
  output reg                    o_harq1_done,
  output reg                    o_harq0_ncb_done,
  output reg                    o_harq1_ncb_done,
  output reg                    o_harq0_e0_burst_en,
  output reg                    o_harq0_e1_burst_en,
  output reg                    o_harq1_e0_burst_en,
  output reg                    o_harq1_e1_burst_en,

  // interface with combine arbiter
  output reg                    o_wcombine_en,
  output reg [8-1:0]            o_wcombine_wen,
  output reg [12-1:0]           o_wcombine_addr,
  output reg [64-1:0]           o_wcombine_wdata,
  
  // for fine clock gating
  output reg                    o_combine_ncb_done, // assert when all ncb finished

  // error control
  output reg                    o_wcombine_err
  
);

  //---------------------------------------------------------------------------
  // Parameter declaration
  //---------------------------------------------------------------------------
  parameter WCOMBINE_SWIDTH = 5;

  parameter WCOMBINE_IDLE             = 5'b00000;
  parameter WCOMBINE_CFG0             = 5'b00001;
  parameter WCOMBINE_CFG1             = 5'b00010;
  parameter WCOMBINE_CFG2             = 5'b00011;
  parameter WCOMBINE_START_WCB        = 5'b00100;
  parameter WCOMBINE_WAIT_START_WBUF0 = 5'b00101;
  parameter WCOMBINE_START_WBUF0      = 5'b00110;
  parameter WCOMBINE_WAIT_WBUF0       = 5'b00111;
  parameter WCOMBINE_WBUF0            = 5'b01000;
  parameter WCOMBINE_WAIT_WBUF1       = 5'b01001;
  parameter WCOMBINE_WBUF1            = 5'b01010;
  parameter WCOMBINE_WBUF_R0_HOLD     = 5'b01011;
  parameter WCOMBINE_END_E_WBUF_R0    = 5'b01100;
  parameter WCOMBINE_WBUF_R1_HOLD     = 5'b01101;
  parameter WCOMBINE_END_E_WBUF_R1    = 5'b01110;
  parameter WCOMBINE_END_WCB_DLY0     = 5'b01111;
  parameter WCOMBINE_END_WCB_DLY1     = 5'b10000;
  parameter WCOMBINE_END_WCB_DLY2     = 5'b10001;
  parameter WCOMBINE_END_WCB          = 5'b10010;
  parameter WCOMBINE_END_WCB_ALL      = 5'b10011;
  parameter WCOMBINE_ERR              = 5'b10100;
  
  parameter DATA_WIDTH = 8;
  
//  parameter max_burst_cnt = 5'd31;
  
  //---------------------------------------------------------------------------
  // Internal wire declaration
  //---------------------------------------------------------------------------
  wire [15-1:0]                 slave_k0_pos;
  wire [4-1:0]                  slave_k0_start;
  wire                          int_combine_data_end;
  wire [12-1:0]                 e0_start_8;         // E0 start position in 8 byte alignment.
  wire                          last_ncb_region_en; // Last ncb region for mode 2 only

  wire [18-1:0]                 k0e0;                      // k0 + E0 size;
  wire [18-1:0]                 k0e1;                      // k0 + E1 size;
  wire [12-1:0]                 k0e_mod_ncb_8;

  //---------------------------------------------------------------------------
  // Sequential reg declaration
  //---------------------------------------------------------------------------
  reg [WCOMBINE_SWIDTH-1:0]     wcombine_cs, wcombine_ns;  // wcombine ncb control state machine
  reg [5-1:0]                   cb_num_cnt;                // cb number counter
  reg                           ncb_ptr;                   // indicate the current ping-pong Ncb buffer
  
  reg [12-1:0]                  ncb_8;                     // ceiling Ncb / 8 for 8 byte alignment
  reg [12-1:0]                  k0_8;                      // floor k0 / 8 for 8 byte alignment
  reg [18-1:0]                  k0e0_mod_ncb;              // k0 + E0 size; mod Ncb
  reg [18-1:0]                  k0e1_mod_ncb;              // k0 + E1 size; mod Ncb
  reg [3-1:0]                   end_pos;                   // end position of the last 8 byte alignment for generating write mask enable
  
  
  reg [64-1:0]                  buf_w0;                    // local 64 bit write buffer 0
  reg [64-1:0]                  buf_w1;                    // local 64 bit write buffer 1
  reg                           full_wbuf_w0;              // indicate write local buffer 0 full;
  reg                           full_wbuf_w1;              // indicate write local buffer 1 full;
  
  
  reg [15-1:0]                  ncb_cnt;                   // Counter for counting in Ncb boundary position of the input data
  reg [15-1:0]                  slave_ncb_cnt;             // Slave Counter for counting in Ncb boundary position of the input data
  reg [4-1:0]                   wptr;                      // normal write pointer of the local buffer
  reg [4-1:0]                   slave_wptr;                // slave write pointer of the local buffer
  reg [26-1:0]                  int_e_bmp_fmt;             // internal E bitmap format for choosing E1 or E2
  reg                           wcombine_ini;              // Write combine counter initial signal to reset the e_cnt counter
  
  reg [17-1:0]                  e_cnt;
  reg                           int_combine_data_end_sync1;
  reg                           int_combine_data_end_sync2;
  reg [17-1:0]                  e_size;  
  reg [2-1:0]                   mode; // mode=0 : E < Ncb, k0+E <= Ncb, mode=1: E< Ncb, K0+E > Ncb, mode=2 E >= Ncb

  reg [15-1:0]                  k0e_mod_ncb;
  
  reg                           e0_burst_incr;
  reg                           e1_burst_incr;
  reg [6-1:0]                   e0_burst_wr_cnt;
  reg [6-1:0]                   e1_burst_wr_cnt;
  
  
  reg                           e1_incr_last_ncb_region_en;
  reg                           e0_end_wdata_en;
//  reg                           last_ncb_done_region;
  
  reg                           int_harq0_ncb_done;
  reg                           int_harq1_ncb_done;
  reg [1:0]                     int_harq0_ncb_done_dly;
  reg [1:0]                     int_harq1_ncb_done_dly;
  
  reg [8-1:0]                   k0e0_mod_cnt;
  reg [8-1:0]                   k0e1_mod_cnt;
  reg [8-1:0]                   k0e_mod_cnt;
  reg [8-1:0]                   ncb_mod_cnt;
  
  reg [6-1:0]                   max_burst_cnt;
  
  //---------------------------------------------------------------------------
  // Instantiation of submodules 
  //---------------------------------------------------------------------------


  //---------------------------------------------------------------------------
  // Combinational logic
  //---------------------------------------------------------------------------  
  assign slave_k0_pos = i_k0_pos + 15'd1;
  assign slave_k0_start = {1'b0, i_k0_pos[2:0]} + 4'd1;
  assign int_combine_data_end = (i_dibit_en & (end_pos == 3'b1)) ? int_combine_data_end_sync2 : int_combine_data_end_sync1;
  assign k0e0 = {3'b0,i_k0_pos} + {1'b0,i_e0_size};
  assign k0e1 = {3'b0,i_k0_pos} + {1'b0,i_e1_size};
  
  assign k0e_mod_ncb_8 = k0e_mod_ncb[14:3] + (|k0e_mod_ncb[2:0]);

  assign e0_start_8 = (k0e_mod_ncb_8 == ncb_8) ? 12'b0 : k0e_mod_ncb_8;
  assign last_ncb_region_en = (e_cnt >= e_size - {2'b0, i_ncb_size}); // for mode 2 only
  
  always @ (*) begin
    case(i_harq_lb_burst_len)
      2'b00 : max_burst_cnt = 6'd15;
      2'b01 : max_burst_cnt = 6'd31;
      2'b10 : max_burst_cnt = 6'd63;
      default : max_burst_cnt = 6'd31;
    endcase
  end


  always @ (*) begin
    if (i_harq_end & (wcombine_cs != WCOMBINE_IDLE)) begin
      wcombine_ns = WCOMBINE_ERR;
    end else begin
      wcombine_ns = wcombine_cs;
      case(wcombine_cs)
        WCOMBINE_IDLE : begin
          if (i_harq_start & (i_cb_num != 5'b0)) begin
            wcombine_ns = WCOMBINE_CFG0;
          end
        end
        WCOMBINE_CFG0 : begin
          wcombine_ns = WCOMBINE_CFG1;
        end
        WCOMBINE_CFG1 : begin
          if (k0e0_mod_ncb <= i_ncb_size) begin
            wcombine_ns = WCOMBINE_CFG2;
          end
        end
        WCOMBINE_CFG2 : begin
          if (k0e1_mod_ncb <= i_ncb_size) begin
            wcombine_ns = WCOMBINE_START_WCB;
          end
        end
        WCOMBINE_START_WCB : begin
          wcombine_ns = WCOMBINE_WAIT_START_WBUF0;
        end
        WCOMBINE_WAIT_START_WBUF0 : begin
          if (full_wbuf_w0) begin
            wcombine_ns = WCOMBINE_START_WBUF0;
          end
        end
        WCOMBINE_START_WBUF0 : begin
          wcombine_ns = WCOMBINE_WAIT_WBUF1;
        end
        WCOMBINE_WAIT_WBUF0 : begin
          if (int_combine_data_end) begin
            wcombine_ns = WCOMBINE_WBUF_R0_HOLD;
          end else if (full_wbuf_w0) begin
            wcombine_ns = WCOMBINE_WBUF0;
          end
        end
        WCOMBINE_WBUF0 : begin
          if (int_combine_data_end) begin
            wcombine_ns = WCOMBINE_WBUF_R1_HOLD;
          end else begin
            wcombine_ns = WCOMBINE_WAIT_WBUF1;
          end
        end
        WCOMBINE_WAIT_WBUF1 : begin
          if (int_combine_data_end) begin
            wcombine_ns = WCOMBINE_WBUF_R1_HOLD;
          end else if (full_wbuf_w1) begin
            wcombine_ns = WCOMBINE_WBUF1;
          end
        end
        WCOMBINE_WBUF1 : begin
          if (int_combine_data_end) begin
            wcombine_ns = WCOMBINE_WBUF_R0_HOLD;
          end else begin
            wcombine_ns = WCOMBINE_WAIT_WBUF0;
          end
        end
        WCOMBINE_WBUF_R0_HOLD : begin
          wcombine_ns = WCOMBINE_END_E_WBUF_R0;
        end
        WCOMBINE_END_E_WBUF_R0 : begin
          wcombine_ns = WCOMBINE_END_WCB_DLY0;
        end
        WCOMBINE_WBUF_R1_HOLD : begin
          wcombine_ns = WCOMBINE_END_E_WBUF_R1;
        end
        WCOMBINE_END_E_WBUF_R1 : begin
          wcombine_ns = WCOMBINE_END_WCB_DLY0;
        end
        WCOMBINE_END_WCB_DLY0 : begin
          wcombine_ns = WCOMBINE_END_WCB_DLY1;
        end
        WCOMBINE_END_WCB_DLY1 : begin
          wcombine_ns = WCOMBINE_END_WCB_DLY2;
        end        
        WCOMBINE_END_WCB_DLY2 : begin
          wcombine_ns = WCOMBINE_END_WCB;
        end        
        WCOMBINE_END_WCB : begin
          if (cb_num_cnt != i_cb_num - 5'b1) begin
            wcombine_ns = WCOMBINE_START_WCB;            
          end else begin
            wcombine_ns = WCOMBINE_END_WCB_ALL;
          end
        end
        WCOMBINE_END_WCB_ALL : begin
          wcombine_ns = WCOMBINE_IDLE;
        end
        WCOMBINE_ERR : begin
          wcombine_ns = WCOMBINE_IDLE;
        end
        default : begin
          wcombine_ns = WCOMBINE_IDLE;
        end
      endcase
    end
  end


  //---------------------------------------------------------------------------
  // Flip-flops
  //---------------------------------------------------------------------------
  //20120223 calculate e_cnt dedicately for combine data strobe have two cycle delay;
  // monitor counter for e size checking
  always @ (posedge i_harq_clk or negedge i_rst_n)
  if (!i_rst_n) begin
    e_cnt <= 17'd0;
  end else begin
    if (wcombine_ini) begin
      e_cnt <= 17'd0;
    end else if (i_combine_data_strb) begin
      if(i_dibit_en) begin
        e_cnt <= e_cnt + 17'd2;
      end else begin 
      	e_cnt <= e_cnt + 17'd1;
      end
    end
  end

  // use for the wcombine to end the Ncb at the last combine data
  always @ (posedge i_harq_clk or negedge i_rst_n)
  if (!i_rst_n) begin
    int_combine_data_end_sync1 <= 1'b0;
  end else begin
    if (i_harq_start) begin
      int_combine_data_end_sync1 <= 1'b0;
    end else if (i_combine_data_strb) begin
      if(i_dibit_en & (e_cnt == e_size - 17'd2)) begin
      	int_combine_data_end_sync1 <= 1'b1;
      end else if(e_cnt == e_size - 17'b1) begin
      	int_combine_data_end_sync1 <= 1'b1;
      end else begin
      	int_combine_data_end_sync1 <= 1'b0;
      end
    end else begin
      int_combine_data_end_sync1 <= 1'b0;
    end
  end   

  always @ (posedge i_harq_clk or negedge i_rst_n)
  if (!i_rst_n) begin
      int_combine_data_end_sync2 <= 1'b0;
  end else begin
      int_combine_data_end_sync2 <= int_combine_data_end_sync1;
  end
  
  //from rcombine, need to check again;
  // monitor counter for ncb boundary checking
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    ncb_cnt <= 15'b0;
  end else begin
    if (wcombine_ini) begin
      ncb_cnt <= i_k0_pos;
    end else if (i_combine_data_strb) begin
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
    if (wcombine_ini) begin
      slave_ncb_cnt <= slave_k0_pos;
    end else if (i_combine_data_strb) begin
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

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    wcombine_cs <= WCOMBINE_IDLE;
  end else begin
    wcombine_cs <= wcombine_ns;
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    full_wbuf_w0 <= 1'b0;
  end else begin
    if (i_harq_start) begin
      full_wbuf_w0 <= 1'b0;
    end else if (i_combine_data_strb) begin
      if (!i_dibit_en) begin // normal case
        if (wptr == 4'd7) begin
          full_wbuf_w0 <= 1'b1;
        end else if ((ncb_cnt == i_ncb_size - 15'd1) & (wptr < 4'd7)) begin
          full_wbuf_w0 <= 1'b1;
        end else begin
          full_wbuf_w0 <= 1'b0;
        end
      end else begin // dibit_en = 1 is the master of dibit mode
        if ((wptr == 4'd6)| (wptr == 4'd7)) begin
          full_wbuf_w0 <= 1'b1;
        end else if (((ncb_cnt == i_ncb_size - 15'd1)| (ncb_cnt == i_ncb_size - 15'd2)) & (wptr < 4'd7)) begin
          full_wbuf_w0 <= 1'b1;
        end else begin
          full_wbuf_w0 <= 1'b0;
        end
      end
    end else begin
      full_wbuf_w0 <= 1'b0;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    full_wbuf_w1 <= 1'b0;
  end else begin
    if (i_harq_start) begin
      full_wbuf_w1 <= 1'b0;
    end else if (i_combine_data_strb) begin
      if (!i_dibit_en) begin // normal case
        if (wptr == 4'd15) begin
          full_wbuf_w1 <= 1'b1;
        end else if ((ncb_cnt == i_ncb_size - 15'd1) & (wptr > 4'd7)) begin
          full_wbuf_w1 <= 1'b1;
        end else begin
          full_wbuf_w1 <= 1'b0;
        end
      end else begin // dibit_en = 1 is the master of dibit mode
        if ((wptr == 4'd14)| (wptr == 4'd15)) begin
          full_wbuf_w1 <= 1'b1;
        end else if (((ncb_cnt == i_ncb_size - 15'd1)| (ncb_cnt == i_ncb_size - 15'd2)) & (wptr > 4'd7)) begin
          full_wbuf_w1 <= 1'b1;
        end else begin
          full_wbuf_w1 <= 1'b0;
        end
      end
    end else begin
      full_wbuf_w1 <= 1'b0;
    end
  end

  // write pointer to select the local buffer 0 and 1 data as rcombine data
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    wptr <= 4'd0;
  end else begin
    if (wcombine_ini) begin
      wptr <= {1'b0,i_k0_pos[2:0]}; // initial the pointer start from local buffer 0 with alignment shift
    end else if (i_combine_data_strb) begin
      if (!i_dibit_en) begin // normal mode
        if (ncb_cnt == i_ncb_size-15'd1) begin // switch to another local buffer at the 
          if (wptr[3]) begin
            wptr <= 4'd0;
          end else begin
            wptr <= 4'd8;
          end
        end else begin
          wptr <= wptr + 4'd1;
        end
      end else begin // dibit mode
        if (ncb_cnt == i_ncb_size-15'd1) begin // switch to another local buffer at the boundry of Ncb
          if (wptr[3]) begin
            wptr <= 4'd1;
          end else begin
            wptr <= 4'd9;
          end
        end else if (ncb_cnt == i_ncb_size - 15'd2) begin
          if (wptr[3]) begin
            wptr <= 4'd0;
          end else begin
            wptr <= 4'd8;
          end
        end else begin
          wptr <= wptr + 4'd2;
        end
      end
    end
  end

  // slave read pointer to select the local buffer 0 and 1 data as slave rcombine data
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    slave_wptr <= 4'd0;
  end else if (i_dibit_en) begin
    if (wcombine_ini) begin // initial the pointer start from local buffer 0 with alignment shift
      slave_wptr <= slave_k0_start;
    end else if (i_combine_data_strb) begin // only for dibit mode
      if (slave_ncb_cnt == i_ncb_size-15'd1) begin // switch to another local buffer at the boundry of Ncb
        if (slave_wptr[3]) begin
          slave_wptr <= 4'd1;
        end else begin
          slave_wptr <= 4'd9;
        end
      end else if (slave_ncb_cnt == i_ncb_size - 15'd2) begin
        if (slave_wptr[3]) begin
          slave_wptr <= 4'd0;
        end else begin
          slave_wptr <= 4'd8;
        end
      end else begin
        slave_wptr <= slave_wptr + 4'd2;
      end
    end
  end else begin
    slave_wptr <= 4'd0;
  end
  
  // ===================================================
  // Start : write local buffer 0 
  // ===================================================  
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w0[DATA_WIDTH-1:0] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd0) begin
      buf_w0[DATA_WIDTH-1:0] <= i_combine_data;
    end else if ((slave_wptr == 4'd0) & i_dibit_en) begin
      buf_w0[DATA_WIDTH-1:0] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w0[2*DATA_WIDTH-1:DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd1) begin
      buf_w0[2*DATA_WIDTH-1:DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd1) begin
      buf_w0[2*DATA_WIDTH-1:DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w0[3*DATA_WIDTH-1:2*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd2) begin
      buf_w0[3*DATA_WIDTH-1:2*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd2) begin
      buf_w0[3*DATA_WIDTH-1:2*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w0[4*DATA_WIDTH-1:3*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd3) begin
      buf_w0[4*DATA_WIDTH-1:3*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd3) begin
      buf_w0[4*DATA_WIDTH-1:3*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w0[5*DATA_WIDTH-1:4*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd4) begin
      buf_w0[5*DATA_WIDTH-1:4*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd4) begin
      buf_w0[5*DATA_WIDTH-1:4*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w0[6*DATA_WIDTH-1:5*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd5) begin
      buf_w0[6*DATA_WIDTH-1:5*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd5) begin
      buf_w0[6*DATA_WIDTH-1:5*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w0[7*DATA_WIDTH-1:6*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd6) begin
      buf_w0[7*DATA_WIDTH-1:6*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd6) begin
      buf_w0[7*DATA_WIDTH-1:6*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w0[8*DATA_WIDTH-1:7*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd7) begin
      buf_w0[8*DATA_WIDTH-1:7*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd7) begin
      buf_w0[8*DATA_WIDTH-1:7*DATA_WIDTH] <= i_slave_combine_data;
    end
  end
  // ===================================================
  // End : write local buffer 0 
  // ===================================================  

  // ===================================================
  // Start : write local buffer 1 
  // ===================================================  
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w1[DATA_WIDTH-1:0] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd8) begin
      buf_w1[DATA_WIDTH-1:0] <= i_combine_data;
    end else if ((slave_wptr == 4'd8) & i_dibit_en) begin
      buf_w1[DATA_WIDTH-1:0] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w1[2*DATA_WIDTH-1:DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd9) begin
      buf_w1[2*DATA_WIDTH-1:DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd9) begin
      buf_w1[2*DATA_WIDTH-1:DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w1[3*DATA_WIDTH-1:2*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd10) begin
      buf_w1[3*DATA_WIDTH-1:2*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd10) begin
      buf_w1[3*DATA_WIDTH-1:2*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w1[4*DATA_WIDTH-1:3*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd11) begin
      buf_w1[4*DATA_WIDTH-1:3*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd11) begin
      buf_w1[4*DATA_WIDTH-1:3*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w1[5*DATA_WIDTH-1:4*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd12) begin
      buf_w1[5*DATA_WIDTH-1:4*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd12) begin
      buf_w1[5*DATA_WIDTH-1:4*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w1[6*DATA_WIDTH-1:5*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd13) begin
      buf_w1[6*DATA_WIDTH-1:5*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd13) begin
      buf_w1[6*DATA_WIDTH-1:5*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w1[7*DATA_WIDTH-1:6*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd14) begin
      buf_w1[7*DATA_WIDTH-1:6*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd14) begin
      buf_w1[7*DATA_WIDTH-1:6*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    buf_w1[8*DATA_WIDTH-1:7*DATA_WIDTH] <= 8'b0;
  end else if (i_combine_data_strb) begin
    if (wptr == 4'd15) begin
      buf_w1[8*DATA_WIDTH-1:7*DATA_WIDTH] <= i_combine_data;
    end else if (slave_wptr == 4'd15) begin
      buf_w1[8*DATA_WIDTH-1:7*DATA_WIDTH] <= i_slave_combine_data;
    end
  end

  // ===================================================
  // End : write local buffer 1 
  // ===================================================  

  // ===================================================
  // Start : burst enable generation
  // ===================================================  
  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    e0_burst_incr <= 1'b0;
  end else begin
    if (o_wcombine_en) begin
      case(mode)
        2'b00: begin
          if ((o_wcombine_addr >= k0_8) && (o_wcombine_addr < k0e_mod_ncb_8)) begin
            e0_burst_incr <= 1'b1;
          end else begin
            e0_burst_incr <= 1'b0;
          end
        end
        2'b01: begin
          if ((o_wcombine_addr >= k0_8) && (o_wcombine_addr < ncb_8)) begin
            e0_burst_incr <= 1'b1;
          end else begin
            e0_burst_incr <= 1'b0;
          end
        end
        2'b10: begin
          if ((o_wcombine_addr >= e0_start_8) && (o_wcombine_addr < ncb_8 - 12'b1) && last_ncb_region_en) begin
            e0_burst_incr <= 1'b1;
          end else if ((o_wcombine_addr == ncb_8 - 12'b1 ) && e0_end_wdata_en) begin 
            e0_burst_incr <= 1'b1;
          end else begin
            e0_burst_incr <= 1'b0;
          end
        end
        default : begin
          e0_burst_incr <= 1'b0;
        end
      endcase
    end else begin
      e0_burst_incr <= 1'b0;
    end
  end

  // ensure the end write data is enabled f
  always @ (posedge i_harq_clk or negedge i_rst_n)
  if (!i_rst_n) begin
    e0_end_wdata_en <= 1'd0;
  end else begin
    if (wcombine_ini) begin
      e0_end_wdata_en <= 1'b0;
    end else if ((o_wcombine_addr == e0_start_8) && o_wcombine_en && last_ncb_region_en)  begin 
      e0_end_wdata_en <= 1'b1;
    end
  end

  // need to delay for e1 increment to avoid the first e1_incr before the e0_burst_incr
  always @ (posedge i_harq_clk or negedge i_rst_n)
  if (!i_rst_n) begin
    e1_incr_last_ncb_region_en <= 1'd0;
  end else begin
    if (wcombine_ini) begin
      e1_incr_last_ncb_region_en <= 1'b0;
    end else if (e0_burst_incr) begin // e1 incr ncb region is always after the first e0_burst_incr;
      e1_incr_last_ncb_region_en <= last_ncb_region_en;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    e1_burst_incr <= 1'b0;
  end else begin
    if (o_wcombine_en) begin
      case(mode)
        2'b01: begin
          if ((o_wcombine_addr >= 12'b0) && (o_wcombine_addr < k0e_mod_ncb_8)) begin
            e1_burst_incr <= 1'b1;
          end else begin
            e1_burst_incr <= 1'b0;
          end
        end
        2'b10: begin
          if ((o_wcombine_addr >= 12'b0) && (o_wcombine_addr < k0e_mod_ncb_8) && e1_incr_last_ncb_region_en 
            && (k0e_mod_ncb_8 != ncb_8) ) begin
            e1_burst_incr <= 1'b1;
          end else begin
            e1_burst_incr <= 1'b0;
          end
        end
        default : begin
          e1_burst_incr <= 1'b0;
        end
      endcase
    end else begin
      e1_burst_incr <= 1'b0;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    e0_burst_wr_cnt <= 6'b0;
    o_harq0_e0_burst_en <= 1'b0;
    o_harq1_e0_burst_en <= 1'b0;
  end else begin
    if (wcombine_ini | i_harq_start) begin
      e0_burst_wr_cnt <= 6'b0;
      o_harq0_e0_burst_en <= 1'b0;
      o_harq1_e0_burst_en <= 1'b0;
    end else if (e0_burst_incr) begin
      if (e0_burst_wr_cnt == max_burst_cnt)  begin
        e0_burst_wr_cnt <= 6'b0;
        o_harq0_e0_burst_en <= !ncb_ptr;
        o_harq1_e0_burst_en <= ncb_ptr;
      end else begin
        e0_burst_wr_cnt <= e0_burst_wr_cnt + 6'b1;
        o_harq0_e0_burst_en <= 1'b0;
        o_harq1_e0_burst_en <= 1'b0;
      end
    end else begin
      e0_burst_wr_cnt <= e0_burst_wr_cnt;
      o_harq0_e0_burst_en <= 1'b0;
      o_harq1_e0_burst_en <= 1'b0;
    end
  end

  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    int_harq0_ncb_done_dly <= 2'b0;
    o_harq0_ncb_done <= 1'b0;
    int_harq1_ncb_done_dly <= 2'b0;
    o_harq1_ncb_done <= 1'b0;    
  end else begin
    {o_harq0_ncb_done, int_harq0_ncb_done_dly} <= {int_harq0_ncb_done_dly, int_harq0_ncb_done};
    {o_harq1_ncb_done, int_harq1_ncb_done_dly} <= {int_harq1_ncb_done_dly, int_harq1_ncb_done};
  end


  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    e1_burst_wr_cnt <= 6'b0;
    o_harq0_e1_burst_en <= 1'b0;
    o_harq1_e1_burst_en <= 1'b0;
  end else begin
    if (wcombine_ini | i_harq_start) begin
      e1_burst_wr_cnt <= 6'b0;
      o_harq0_e1_burst_en <= 1'b0;
      o_harq1_e1_burst_en <= 1'b0;
    end else if (e1_burst_incr) begin
      if (e1_burst_wr_cnt == max_burst_cnt)  begin
        e1_burst_wr_cnt <= 6'b0;
        o_harq0_e1_burst_en <= !ncb_ptr;
        o_harq1_e1_burst_en <= ncb_ptr;
      end else begin
        e1_burst_wr_cnt <= e1_burst_wr_cnt + 6'b1;
        o_harq0_e1_burst_en <= 1'b0;
        o_harq1_e1_burst_en <= 1'b0;
      end
    end else begin
      e1_burst_wr_cnt <= e1_burst_wr_cnt;
      o_harq0_e1_burst_en <= 1'b0;
      o_harq1_e1_burst_en <= 1'b0;
    end
  end

  // ===================================================
  // End : burst enable generation
  // ===================================================  


  always @ (posedge i_harq_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    cb_num_cnt <= 5'b0;
    ncb_8 <= 12'b0;

    wcombine_ini <= 1'b0;
    ncb_ptr <= 1'b0;
    o_harq0_done <= 1'b0;
    o_harq1_done <= 1'b0;
    k0e0_mod_ncb <= 18'b0;
    k0e1_mod_ncb <= 18'b0;
    e_size <= 17'b0;
    k0_8 <= 12'b0;
    ncb_8 <= 12'b0;
    end_pos <= 3'b0;

    o_wcombine_err <= 1'b0;

    o_wcombine_en <= 1'b0;
    o_wcombine_wen <= 8'b0;
    o_wcombine_addr <= 12'b0;
    o_wcombine_wdata <= 64'b0;

    int_e_bmp_fmt <= 26'b0;
    mode <= 2'b0;
    k0e_mod_ncb <= 15'b0;
    int_harq0_ncb_done <= 1'b0;
    int_harq1_ncb_done <= 1'b0;
//    o_harq0_combine_start <= 1'b0;
//    o_harq1_combine_start <= 1'b0;

    k0e0_mod_cnt <= 8'b0;
    k0e1_mod_cnt <= 8'b0;
    k0e_mod_cnt <= 8'b0;
    ncb_mod_cnt <= 8'b0;
    o_combine_ncb_done <= 1'b0;

  end else begin
    case(wcombine_cs)
      WCOMBINE_IDLE : begin
        cb_num_cnt <= 5'b0;
        ncb_8 <= 12'b0;

        wcombine_ini <= 1'b0;
        ncb_ptr <= 1'b0;
        o_harq0_done <= 1'b0;
        o_harq1_done <= 1'b0;
        k0e0_mod_ncb <= 18'b0;
        k0e1_mod_ncb <= 18'b0;
        e_size <= 17'b0;
        k0_8 <= 12'b0;
        ncb_8 <= 12'b0;
        end_pos <= 3'b0;

        o_wcombine_err <= 1'b0;

        o_wcombine_en <= 1'b0;
        o_wcombine_wen <= 8'b0;
        o_wcombine_addr <= 12'b0;
        o_wcombine_wdata <= 64'b0;

        int_e_bmp_fmt <= 26'b0;
        mode <= 2'b0;
        k0e_mod_ncb <= 15'b0;
        int_harq0_ncb_done <= 1'b0;
        int_harq1_ncb_done <= 1'b0;
//        o_harq0_combine_start <= 1'b0;
//        o_harq1_combine_start <= 1'b0;

        k0e0_mod_cnt <= 8'b0;
        k0e1_mod_cnt <= 8'b0;
        k0e_mod_cnt <= 8'b0;
        ncb_mod_cnt <= 8'b0;
        o_combine_ncb_done <= 1'b0;

      end
      WCOMBINE_CFG0 : begin
        int_e_bmp_fmt <= i_e_bmp_fmt;
        ncb_8 <= i_ncb_size[14:3] + (|i_ncb_size[2:0]);
        k0_8 <= i_k0_pos[14:3];
        k0e0_mod_ncb <= k0e0; 
        k0e1_mod_ncb <= k0e1; 
        k0e0_mod_cnt <= 8'b0;
        k0e1_mod_cnt <= 8'b0;
      end
      // When k0e0_mod_ncb == i_ncb_size the end point is at the Ncb size boundary it don't need to 
      // count up the mod counter and end_pos is equal to ncb_size poistion.
      WCOMBINE_CFG1 : begin 
        if (k0e0_mod_ncb > i_ncb_size) begin // k0e0_mod_ncb mod Ncb
          k0e0_mod_ncb <= k0e0_mod_ncb - {3'b0, i_ncb_size};
          k0e0_mod_cnt <= k0e0_mod_cnt + 8'b1;
        end
      end
      WCOMBINE_CFG2 : begin
        if (k0e1_mod_ncb > i_ncb_size) begin // k0e0_mod_ncb mod Ncb
          k0e1_mod_ncb <= k0e1_mod_ncb - {3'b0, i_ncb_size};
          k0e1_mod_cnt <= k0e1_mod_cnt + 8'b1;
        end
      end
      WCOMBINE_START_WCB : begin
        wcombine_ini <= 1'b1;
//        o_harq0_combine_start <= !ncb_ptr;
//        o_harq1_combine_start <= ncb_ptr;
        ncb_8 <= i_ncb_size[14:3] + (|i_ncb_size[2:0]);
        end_pos <= int_e_bmp_fmt[0] ? k0e1_mod_ncb[2:0] : k0e0_mod_ncb[2:0];
        e_size <= int_e_bmp_fmt[0] ? i_e1_size : i_e0_size;
        k0e_mod_ncb <= int_e_bmp_fmt[0] ? k0e1_mod_ncb[14:0] : k0e0_mod_ncb[14:0];
        k0e_mod_cnt <= int_e_bmp_fmt[0] ? k0e1_mod_cnt  : k0e0_mod_cnt;
        ncb_mod_cnt <= 8'b0;

        o_harq0_done <= 1'b0;
        o_harq1_done <= 1'b0;
        if (!int_e_bmp_fmt[0]) begin
          if (i_e0_size < i_ncb_size) begin
            if (k0e0 <= i_ncb_size) begin
              mode <= 2'b00;
            end else begin
              mode <= 2'b01;
            end
          end else begin
            mode <= 2'b10;
          end
        end else begin
          if (i_e1_size < i_ncb_size) begin
            if (k0e1 <= i_ncb_size) begin
              mode <= 2'b00;
            end else begin
              mode <= 2'b01;
            end
          end else begin
            mode <= 2'b10;
          end
        end
        
      end
      WCOMBINE_WAIT_START_WBUF0 : begin
        o_wcombine_en <= 1'b0;
	wcombine_ini <= 1'b0;
//        o_harq0_combine_start <= 1'b0;
//        o_harq1_combine_start <= 1'b0;
      end
      WCOMBINE_START_WBUF0 : begin
        o_wcombine_en <= 1'b1;
        case(i_k0_pos[2:0])
          3'd0 : o_wcombine_wen <= 8'b1111_1111;
          3'd1 : o_wcombine_wen <= 8'b1111_1110;
          3'd2 : o_wcombine_wen <= 8'b1111_1100;
          3'd3 : o_wcombine_wen <= 8'b1111_1000;
          3'd4 : o_wcombine_wen <= 8'b1111_0000;
          3'd5 : o_wcombine_wen <= 8'b1110_0000;
          3'd6 : o_wcombine_wen <= 8'b1100_0000;
          3'd7 : o_wcombine_wen <= 8'b1000_0000;
          default : o_wcombine_wen <= 8'b1111_1111;
        endcase
        o_wcombine_addr <= k0_8;
        o_wcombine_wdata <= buf_w0;
      end
      WCOMBINE_WAIT_WBUF0 : begin
        wcombine_ini <= 1'b0;
        o_wcombine_en <= 1'b0;
        o_wcombine_wen <= 8'b0;
        int_harq0_ncb_done <= 1'b0;
        int_harq1_ncb_done <= 1'b0;
      end
      WCOMBINE_WBUF0 : begin
        o_wcombine_en <= 1'b1;
	if(o_wcombine_addr == ncb_8 - 12'd2) begin
	  case (i_ncb_size[2:0])
            3'd0 : o_wcombine_wen <= 8'b1111_1111; //NCB end need to pad zero to 64bit align;
            3'd1 : o_wcombine_wen <= 8'b0000_0001;
            3'd2 : o_wcombine_wen <= 8'b0000_0011;
            3'd3 : o_wcombine_wen <= 8'b0000_0111;
            3'd4 : o_wcombine_wen <= 8'b0000_1111;
            3'd5 : o_wcombine_wen <= 8'b0001_1111;
            3'd6 : o_wcombine_wen <= 8'b0011_1111;
            3'd7 : o_wcombine_wen <= 8'b0111_1111;
            default : o_wcombine_wen <= 8'b1111_1111;
          endcase
	end else begin
          o_wcombine_wen <= 8'b1111_1111;
	end
        if(o_wcombine_addr == ncb_8 - 12'd2) begin
          ncb_mod_cnt <= ncb_mod_cnt + 8'b1;
          if (ncb_mod_cnt == k0e_mod_cnt - 8'b1 + {7'b0, (k0e_mod_ncb_8 == ncb_8)}) begin
            int_harq0_ncb_done <= !ncb_ptr;
            int_harq1_ncb_done <= ncb_ptr;
          end else begin
            int_harq0_ncb_done <= 1'b0;
            int_harq1_ncb_done <= 1'b0;
          end
        end else begin
          ncb_mod_cnt <= ncb_mod_cnt;
          int_harq0_ncb_done <= 1'b0;
          int_harq1_ncb_done <= 1'b0;
        end

        if (o_wcombine_addr != ncb_8 - 12'b1) begin
          o_wcombine_addr <= o_wcombine_addr + 12'b1;
        end else begin
          o_wcombine_addr <= 12'b0;
        end
        o_wcombine_wdata <= buf_w0;
      end
      WCOMBINE_WAIT_WBUF1 : begin
        o_wcombine_en <= 1'b0;
        o_wcombine_wen <= 8'b0;
        int_harq0_ncb_done <= 1'b0;
        int_harq1_ncb_done <= 1'b0;
      end
      WCOMBINE_WBUF1 : begin
        o_wcombine_en <= 1'b1;
        if(o_wcombine_addr == ncb_8 - 12'd2) begin
          case (i_ncb_size[2:0])
            3'd0 : o_wcombine_wen <= 8'b1111_1111;
            3'd1 : o_wcombine_wen <= 8'b0000_0001;
            3'd2 : o_wcombine_wen <= 8'b0000_0011;
            3'd3 : o_wcombine_wen <= 8'b0000_0111;
            3'd4 : o_wcombine_wen <= 8'b0000_1111;
            3'd5 : o_wcombine_wen <= 8'b0001_1111;
            3'd6 : o_wcombine_wen <= 8'b0011_1111;
            3'd7 : o_wcombine_wen <= 8'b0111_1111;
            default : o_wcombine_wen <= 8'b1111_1111;
          endcase
	end else begin
          o_wcombine_wen <= 8'b1111_1111;
        end
        if(o_wcombine_addr == ncb_8 - 12'd2) begin
          ncb_mod_cnt <= ncb_mod_cnt + 8'b1;
          if (ncb_mod_cnt == k0e_mod_cnt - 8'b1 + {7'b0, (k0e_mod_ncb_8 == ncb_8)}) begin
            int_harq0_ncb_done <= !ncb_ptr;
            int_harq1_ncb_done <= ncb_ptr;
          end else begin
            int_harq0_ncb_done <= 1'b0;
            int_harq1_ncb_done <= 1'b0;
          end
        end else begin
          ncb_mod_cnt <= ncb_mod_cnt;
          int_harq0_ncb_done <= 1'b0;
          int_harq1_ncb_done <= 1'b0;
        end

        if (o_wcombine_addr != ncb_8 - 12'b1) begin
          o_wcombine_addr <= o_wcombine_addr + 12'b1;
        end else begin
          o_wcombine_addr <= 12'b0;
        end
        o_wcombine_wdata <= buf_w1;
      end
      WCOMBINE_WBUF_R0_HOLD : begin
        o_wcombine_en <= 1'b0;
        int_harq0_ncb_done <= 1'b0;
        int_harq1_ncb_done <= 1'b0;
      end
      WCOMBINE_END_E_WBUF_R0 : begin
        o_wcombine_en <= 1'b1;
        case(end_pos)
          3'd0 : o_wcombine_wen <= 8'b1111_1111;
          3'd1 : o_wcombine_wen <= 8'b0000_0001;
          3'd2 : o_wcombine_wen <= 8'b0000_0011;
          3'd3 : o_wcombine_wen <= 8'b0000_0111;
          3'd4 : o_wcombine_wen <= 8'b0000_1111;
          3'd5 : o_wcombine_wen <= 8'b0001_1111;
          3'd6 : o_wcombine_wen <= 8'b0011_1111;
          3'd7 : o_wcombine_wen <= 8'b0111_1111;
          default : o_wcombine_wen <= 8'b1111_1111;
        endcase
        if(o_wcombine_addr == ncb_8 - 12'd2) begin
          ncb_mod_cnt <= ncb_mod_cnt + 8'b1;
          if (ncb_mod_cnt == k0e_mod_cnt - 8'b1 + {7'b0, (k0e_mod_ncb_8 == ncb_8)}) begin
            int_harq0_ncb_done <= !ncb_ptr;
            int_harq1_ncb_done <= ncb_ptr;
          end else begin
            int_harq0_ncb_done <= 1'b0;
            int_harq1_ncb_done <= 1'b0;
          end
        end else begin
          ncb_mod_cnt <= ncb_mod_cnt;
          int_harq0_ncb_done <= 1'b0;
          int_harq1_ncb_done <= 1'b0;
        end
        
        if (o_wcombine_addr != ncb_8 - 12'b1) begin
          o_wcombine_addr <= o_wcombine_addr + 12'b1;
        end else begin
          o_wcombine_addr <= 12'b0;
        end
        o_wcombine_wdata <= buf_w0;
      end
      WCOMBINE_WBUF_R1_HOLD : begin
        o_wcombine_en <= 1'b0;
        int_harq0_ncb_done <= 1'b0;
        int_harq1_ncb_done <= 1'b0;
      end
      WCOMBINE_END_E_WBUF_R1 : begin
        o_wcombine_en <= 1'b1;
        case(end_pos)
          3'd0 : o_wcombine_wen <= 8'b1111_1111;
          3'd1 : o_wcombine_wen <= 8'b0000_0001;
          3'd2 : o_wcombine_wen <= 8'b0000_0011;
          3'd3 : o_wcombine_wen <= 8'b0000_0111;
          3'd4 : o_wcombine_wen <= 8'b0000_1111;
          3'd5 : o_wcombine_wen <= 8'b0001_1111;
          3'd6 : o_wcombine_wen <= 8'b0011_1111;
          3'd7 : o_wcombine_wen <= 8'b0111_1111;
          default : o_wcombine_wen <= 8'b1111_1111;
        endcase

        if(o_wcombine_addr == ncb_8 - 12'd2) begin
          ncb_mod_cnt <= ncb_mod_cnt + 8'b1;
          if (ncb_mod_cnt == k0e_mod_cnt - 8'b1 + {7'b0, (k0e_mod_ncb_8 == ncb_8)}) begin
            int_harq0_ncb_done <= !ncb_ptr;
            int_harq1_ncb_done <= ncb_ptr;
          end else begin
            int_harq0_ncb_done <= 1'b0;
            int_harq1_ncb_done <= 1'b0;
          end
        end else begin
          ncb_mod_cnt <= ncb_mod_cnt;
          int_harq0_ncb_done <= 1'b0;
          int_harq1_ncb_done <= 1'b0;
        end

        if (o_wcombine_addr != ncb_8 - 12'b1) begin
          o_wcombine_addr <= o_wcombine_addr + 12'b1;
        end else begin
          o_wcombine_addr <= 12'b0;
        end
        o_wcombine_wdata <= buf_w1;
      end
      WCOMBINE_END_WCB_DLY0 : begin
        o_wcombine_en <= 1'b0;
        o_wcombine_wen <= 8'h0;
        o_wcombine_addr <= 12'b0;
        int_harq0_ncb_done <= 1'b0;
        int_harq1_ncb_done <= 1'b0;
      end
      WCOMBINE_END_WCB_DLY1 : begin
        o_wcombine_en <= 1'b0;
        o_wcombine_wen <= 8'h0;
        o_wcombine_addr <= 12'b0;
        int_harq0_ncb_done <= 1'b0;
        int_harq1_ncb_done <= 1'b0;
      end
      WCOMBINE_END_WCB_DLY2 : begin
        o_wcombine_en <= 1'b0;
        o_wcombine_wen <= 8'h0;
        o_wcombine_addr <= 12'b0;
        int_harq0_ncb_done <= 1'b0;
        int_harq1_ncb_done <= 1'b0;
      end
      WCOMBINE_END_WCB : begin
        o_wcombine_en <= 1'b0;
        o_wcombine_wen <= 8'h0;
        o_wcombine_addr <= 12'b0;
        ncb_mod_cnt <= 8'b0;

        o_harq0_done <= !ncb_ptr;
        o_harq1_done <= ncb_ptr;
        ncb_ptr <= !ncb_ptr;
        cb_num_cnt <= cb_num_cnt + 5'b1;
        int_e_bmp_fmt <= {1'b0, int_e_bmp_fmt[25:1]};
      end
      WCOMBINE_END_WCB_ALL : begin
        o_wcombine_en <= 1'b0;
        o_wcombine_wen <= 8'h0;
        o_wcombine_addr <= 12'b0;
        o_harq0_done <= 1'b0;
        o_harq1_done <= 1'b0;
        o_combine_ncb_done <= 1'b1;
      end
      WCOMBINE_ERR : begin
        o_wcombine_err <= 1'b1;
        o_wcombine_en <= 1'b0;
        o_wcombine_wen <= 8'h0;
        o_combine_ncb_done <= 1'b1;
      end
      default : begin
        cb_num_cnt <= 5'b0;
        ncb_8 <= 12'b0;

        wcombine_ini <= 1'b0;
        ncb_ptr <= 1'b0;
        o_harq0_done <= 1'b0;
        o_harq1_done <= 1'b0;
        k0e0_mod_ncb <= 18'b0;
        k0e1_mod_ncb <= 18'b0;
        k0_8 <= 12'b0;
        ncb_8 <= 12'b0;
        end_pos <= 3'b0;

        o_wcombine_err <= 1'b0;

        o_wcombine_en <= 1'b0;
        o_wcombine_wen <= 8'b0;
        o_wcombine_addr <= 12'b0;
        o_wcombine_wdata <= 64'b0;

        int_e_bmp_fmt <= 26'b0;
        mode <= 2'b0;
        k0e_mod_ncb <= 15'b0;
        int_harq0_ncb_done <= 1'b0;
        int_harq1_ncb_done <= 1'b0;
//        o_harq0_combine_start <= 1'b0;
//        o_harq1_combine_start <= 1'b0;

        k0e0_mod_cnt <= 8'b0;
        k0e1_mod_cnt <= 8'b0;
        k0e_mod_cnt <= 8'b0;
        ncb_mod_cnt <= 8'b0;
        o_combine_ncb_done <= 1'b0;

      end
    endcase
  end

endmodule

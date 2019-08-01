// Description
// This module is for Store local Ncb ping-pong buffer to memory
//
// Creation Date: 2011/07/12
// Version: v1.0
//
// Revision Data : 2013/02/19
// Version v1.1
// Fix the write zero problem for pdsch_decoder interface
//
// Revision Data : 2013/02/21
// Version v1.1
// 1) Fix the data zero problem under ndi=0
// 2) Fix generation of the harq_clr / harq_ncb_clr signal problem
//
// Revision Data : 2013/02/28
// Version v1.2
// 1) reset burst counter when i_harq_start and i_harq0/1_combine_start is asserted
// 2) Fix the harq_rdy clear problem when the last burst = burst number size.
// 3) add the i_harq_lb_burst_len control function.
//
// Revision Data : 2013/03/13
// Version v1.3
// Fix lint
//
// Revision Data : 2014/01/21 Bob Wong
// Version v1.4
// Add restriction for z2_8 == 0 when (e1_8_end == e0_8_start) | e0_8_start == 12'b0
// Add restriction for z1_8, z0_8 = 0.
//

module sto_ncb (

  // global signal
  input                         i_rst_n,
  input                         i_mem_clk,

  // rx ctrl input control
  input                         i_harq_start,
  input                         i_harq_end,
  input                         i_pdsch_dec_wr_start, // PDSCH decoder write buffer start -- sto ncb should write the data after this signal

  // input configuration
  input                         i_combine_disable_flag, // 0 : normal , 1 : disable the DDR transfer for HARQ combining process
  input [5-1:0]                 i_cb_num,           // number of CB
  input [26-1:0]                i_e_bmp_fmt,        // E bitmap format
  input [17-1:0]                i_e0_size,          // E0 size
  input [17-1:0]                i_e1_size,          // E1 size
  input [15-1:0]                i_k0_pos,           // K0 position
  input                         i_ndi,              // New data indicator
  input [15-1:0]                i_ncb_size,         // Ncb size
  input [32-1:0]                i_tb_harq_baddr,    // Harq Base address
  input [2-1:0]                 i_harq_lb_burst_len, // HARQ local bus interface burst length 0-16, 1-32, 2-64

  // control signal to fetch_ncb
  output reg                    o_sto0_done,
  output reg                    o_sto1_done,
  
  // control signal to sync_sto_cfg
  output reg                    o_sto_ch_done,      // indicate all Store Ncb operation done signal
  
  // control signal from combine_top
  input                         i_harq0_done,
  input                         i_harq1_done,
  input                         i_harq0_ncb_done,
  input                         i_harq1_ncb_done,
  input                         i_harq0_e0_burst_en,
  input                         i_harq0_e1_burst_en,
  input                         i_harq1_e0_burst_en,
  input                         i_harq1_e1_burst_en,
  input                         i_harq0_combine_start,
  input                         i_harq1_combine_start,
  
  // output signal for local ncb buffer
  output                        o_sto_ptr,
  output                        o_sto_ren,
  output [12-1:0]               o_sto_addr,
  input  [64-1:0]               i_sto_rdata,
  
  // output data local read axi bus
  output reg                    o_wr_cmd_strb,
  input                         i_wr_cmd_done,
  output reg [16-1:0]           o_wr_data_number,
  output reg [32-1:0]           o_wr_baddr,
  output                        o_wr,
  output [64-1:0]               o_wdata,
  input                         i_wfull,
  output                        o_wr_termi,
  
  // output data for PDSCH turbo decoder interface
  output [12-1:0]               o_harq_ncb_addr, // CB internal addresss for turbo decoder NCB buffer only
  output                        o_harq_ncb_wr,
  input                         i_pdsch_dec_ncb_rdy, // indicate the PDSCH decoder write NCB buffer ready
  
  // erro control
  output reg                    o_sto_err
  
);

  //---------------------------------------------------------------------------
  // Parameter declaration
  //---------------------------------------------------------------------------
//  parameter max_burst_cnt = 16'd32;

  parameter STO_SWIDTH = 6;

  parameter STO_IDLE                  = 6'b000000;
  parameter STO_INI                   = 6'b000001;
  parameter STO_CFG0                  = 6'b000010;
  parameter STO_CFG1                  = 6'b000011;

  parameter STO_WAIT_DEC_WR_START     = 6'b000100;
  parameter STO_E_CFG0                = 6'b000101;
  parameter STO_E_CFG1                = 6'b000110;
  parameter WAIT_STO_START            = 6'b000111;

  parameter STO_START                 = 6'b001000;
  parameter WAIT_CMD_STO_Z0           = 6'b001001;
  parameter CMD_STO_Z0                = 6'b001010;
  parameter WAIT_START_DATA_STO_Z0    = 6'b001011;

  parameter DATA_STO_Z0               = 6'b001100;
  parameter WAIT_DATA_STO_Z0_END      = 6'b001101;
  parameter WAIT_CMD_STO_Z1           = 6'b001110;
  parameter CMD_STO_Z1                = 6'b001111;

  parameter WAIT_START_DATA_STO_Z1    = 6'b010000;
  parameter DATA_STO_Z1               = 6'b010001;
  parameter WAIT_DATA_STO_Z1_END      = 6'b010010;
  parameter WAIT_CMD_STO_E0_A         = 6'b010011;

  parameter CMD_STO_E0_A              = 6'b010100; // 14
  parameter WAIT_START_DATA_STO_E0_A  = 6'b010101; // 15
  parameter DATA_STO_E0_A             = 6'b010110; // 16
  parameter WAIT_DATA_STO_E0_A_END    = 6'b010111; // 17

  parameter STO_E0_A_END              = 6'b011000; // 18
  parameter WAIT_NXT_CMD_E0_A         = 6'b011001; // 19
  parameter WAIT_CMD_STO_Z2           = 6'b011010; // 1a
  parameter CMD_STO_Z2                = 6'b011011;

  parameter WAIT_START_DATA_STO_Z2    = 6'b011100; // 1c
  parameter DATA_STO_Z2               = 6'b011101; // 1d
  parameter WAIT_DATA_STO_Z2_END      = 6'b011110; // 1e
  parameter WAIT_CMD_STO_E0_B         = 6'b011111; // 1f

  parameter CMD_STO_E0_B              = 6'b100000;
  parameter WAIT_START_DATA_STO_E0_B  = 6'b100001;
  parameter DATA_STO_E0_B             = 6'b100010;
  parameter WAIT_DATA_STO_E0_B_END    = 6'b100011;

  parameter STO_E0_B_END              = 6'b100100;
  parameter STO_WO_E1_END             = 6'b100101;
  parameter WAIT_NXT_CMD_E0_B         = 6'b100110;
  parameter WAIT_CMD_STO_E1_B         = 6'b100111;

  parameter CMD_STO_E1_B              = 6'b101000;
  parameter WAIT_START_DATA_STO_E1_B  = 6'b101001;
  parameter DATA_STO_E1_B             = 6'b101010;
  parameter WAIT_DATA_STO_E1_B_END    = 6'b101011;

  parameter STO_E1_B_END              = 6'b101100;
  parameter WAIT_NXT_CMD_E1_B         = 6'b101101;
  parameter STO_END                   = 6'b101110;
  parameter STO_END_ALL               = 6'b101111;

  parameter STO_ERR                   = 6'b110000;


  //---------------------------------------------------------------------------
  // Internal wire declaration
  //---------------------------------------------------------------------------
//  wire [18-1:0]                 k0e;                    // k0 + E size 
//  wire [15-1:0]                 e_8_tmp;                // E size for 8 byte alignment
//  wire [18-1:0]                 k0e_mod_tmp;            // ( k0 + E ) mod Ncb only use for E < Ncb and size is small than Ncb
//  wire [12-1:0]                 k0e_mod_8;              // ceiling (k0 + E ) mod Ncb for 8 byte alignment
//  wire                          zero_pad_valid;         // valid position of padding zero;

  wire [18-1:0]                 k0e0;
  wire [18-1:0]                 k0e1;
  
  wire [12-1:0]                 k0_e_mod_ncb_8;
  wire [12-1:0]                 z0_8_start;
  wire [12-1:0]                 z0_8_end;
  wire [12-1:0]                 z1_8_start;
  wire [12-1:0]                 z1_8_end;
  wire [12-1:0]                 z2_8_start;
  wire [12-1:0]                 z2_8_end;
  wire [12-1:0]                 e1_8_start;
  wire [12-1:0]                 e1_8_end;

  wire [11-1:0]                 e0_burst_cnt;
  wire [11-1:0]                 e1_burst_cnt;
  
  wire                          harq_ncb_rdy;
  wire                          harq_rdy;
  wire                          harq0_e0_burst_decr;
  wire                          harq0_e1_burst_decr;
  wire                          harq1_e0_burst_decr;
  wire                          harq1_e1_burst_decr;

  //---------------------------------------------------------------------------
  // Sequential reg declaration
  //---------------------------------------------------------------------------
  reg [STO_SWIDTH-1:0]          sto_cs, sto_ns;         // sto ncb control state machine
  reg                           ncb_ptr;                // ncb pointer to point the Ncb ping-pong buffer
  reg                           harq0_clr;              // clear the store 0 ready signal
  reg                           harq1_clr;              // clear the store 1 ready signal
  reg                           harq0_rdy;              // Harq 0 ready signal
  reg                           harq1_rdy;              // Harq 1 ready signal
  reg [26-1:0]                  int_e_bmp_fmt;          // internal E bitmap format for choosing E1 or E2
  reg [17-1:0]                  e_size;                 // E size in current Ncb
  reg [32-1:0]                  baddr;                  // current base address1;
  reg [32-1:0]                  ncb_baddr;              // current Ncb base address1;

  reg [12-1:0]                  ncb_8;                  // ceiling Ncb / 8 for 8 byte alignment

//  reg [12-1:0]                  k0_8;                   // floor k0 / 8 for 8 byte alignment
//  reg [14-1:0]                  e_8;                    // ceiling e/8 size for 8 byte alignment
//  reg [12-1:0]                  e1_8;                   // ceiling e1/8 size for 8 byte alignment
//  reg [12-1:0]                  e2_8;                   // ceiling e2/8 size for 8 byte alignment
//  reg [15-1:0]                  k0e_8;                  // ceiling k0 + E /8 size for 8 byte alignment
//  reg [15-1:0]                  k0e_mod;                // ( k0 + E ) mod Ncb only use for E < Ncb and size is small than Ncb


  reg [12-1:0]                  rd_ncb_buf_cnt;         // write store counter 
  reg [12-1:0]                  int_raddr;              // internal read address
  reg                           int_ren;                // internal read enable
  reg [5-1:0]                   cb_num_cnt;             // cb number counter
  reg                           data_zero_en;           // set the data output as zero enable for zero filling of the NDI=1
  reg                           int_wr;                 // Store write enable delay for axi local bus
  reg                           int_ncb_wr;              // Store write enable delay for turbo decoder Ncb buffer
  reg [12-1:0]                  prefetch_raddr;         // keep the prefetch address of the local Ncb memory when i_wfull is 1.
  
  reg                           harq0_ncb_rdy;
  reg                           harq1_ncb_rdy;
  reg                           combine0_rdy;
  reg                           combine1_rdy;  
  
  reg                           combine0_clr;
  reg                           combine1_clr;
  reg                           harq0_ncb_clr;
  reg                           harq1_ncb_clr;  

  reg [18-1:0]                  k0e0_mod_ncb;
  reg [18-1:0]                  k0e1_mod_ncb;
  reg [15-1:0]                  k0e_mod_ncb;
  reg [18-1:0]                  k0e;
  
  reg [11-1:0]                  harq0_e0_burst_cnt;
  reg [11-1:0]                  harq0_e1_burst_cnt;
  reg [11-1:0]                  harq1_e0_burst_cnt;
  reg [11-1:0]                  harq1_e1_burst_cnt;
  reg                           e0_burst_decr;
  reg                           e1_burst_decr;
  reg [12-1:0]                  e0_8_start;
  reg [12-1:0]                  e0_8_end;

  reg                           last_burst;
  reg [12-1:0]                  burst_num;
  
//  reg [12-1:0]                  k0_e_mod_ncb_8;
  
  reg [12-1:0]                  e0_8;
  reg [12-1:0]                  e1_8;
  reg [12-1:0]                  z0_8;
  reg [12-1:0]                  z1_8;
  reg [12-1:0]                  z2_8;
  
  reg [16-1:0]                  max_burst_cnt;
  //---------------------------------------------------------------------------
  // Instantiation of submodules 
  //---------------------------------------------------------------------------


  //---------------------------------------------------------------------------
  // Combinational logic
  //---------------------------------------------------------------------------
  always @ (*) begin
    case(i_harq_lb_burst_len)
      2'b00 : max_burst_cnt = 16'd16;
      2'b01 : max_burst_cnt = 16'd32;
      2'b10 : max_burst_cnt = 16'd64;
      default : max_burst_cnt = 16'd32;
    endcase
  end

  assign o_wr_termi = o_sto_err;
  
  assign o_sto_ptr = ncb_ptr;
  assign o_sto_ren = (int_ren | i_wfull);  //add i_wfull to keep the last SRAM output data who have not stored but i_wfull; 
                                           //at the end NCB store, it will have a redundant i_wfull pulse, but it will not impact rigth NCB store
					   //for it dont transfer to local axi write cycle. 
  assign o_sto_addr = i_wfull ? prefetch_raddr : int_raddr;
  
  assign o_wr = (!i_wfull & int_wr);
  assign o_wdata = data_zero_en ? 64'h0 : i_sto_rdata;

  assign o_harq_ncb_addr = prefetch_raddr;
  assign o_harq_ncb_wr = (!i_wfull & int_ncb_wr);
  
  
//  assign k0e = i_k0_pos + e_size;
//  assign k0e_mod_tmp = k0e - i_ncb_size; // only 15 bit valid 
//  assign zero_pad_valid = k0e_mod_tmp < i_k0_pos;  
//  assign k0e_mod_8 = k0e_mod[14:3] + (|k0e_mod[2:0]);  
//  assign e_8_tmp = k0e_8 - k0_8; // 15 bit - 12 bit = 15 bit but possible e_8 size is only 14 bit; 

  assign k0e0 = {3'b0,i_k0_pos} + {1'b0,i_e0_size};
  assign k0e1 = {3'b0,i_k0_pos} + {1'b0,i_e1_size};
  assign k0_e_mod_ncb_8 = k0e_mod_ncb[14:3] + |k0e_mod_ncb[2:0];
    
  assign z0_8_start = 12'b0;
  assign z0_8_end = i_k0_pos[14:3] - 12'b1;
  
  assign z1_8_start = k0_e_mod_ncb_8;
  assign z1_8_end = ncb_8 - 12'b1;
  
  assign z2_8_start = k0_e_mod_ncb_8;
  assign z2_8_end = i_k0_pos[14:3] - 12'b1; // z0_8_end;

  assign harq_ncb_rdy = ncb_ptr ? harq1_ncb_rdy : harq0_ncb_rdy;
  assign harq_rdy = ncb_ptr ? harq1_rdy : harq0_rdy;

  assign e0_burst_cnt = ncb_ptr ? harq1_e0_burst_cnt : harq0_e0_burst_cnt;
  assign e1_burst_cnt = ncb_ptr ? harq1_e1_burst_cnt : harq0_e1_burst_cnt;

  assign harq0_e0_burst_decr = !ncb_ptr & e0_burst_decr;
  assign harq0_e1_burst_decr = !ncb_ptr & e1_burst_decr;
  assign harq1_e0_burst_decr = ncb_ptr & e0_burst_decr;
  assign harq1_e1_burst_decr = ncb_ptr & e1_burst_decr;


  
  always @ (*) begin
    if (e_size < {2'b0,i_ncb_size} - 17'b1) begin
      e0_8_start = i_k0_pos[14:3];
    end else begin
      e0_8_start = (k0_e_mod_ncb_8 == ncb_8) ? 12'b0 : k0_e_mod_ncb_8;          
    end
  end

//  assign e0_8_end = k0_e_mod_ncb_8 - 12'b1;

  always @ (*) begin // for mode = 1 or mode = 2
    if (k0e > {3'b0,i_ncb_size}) begin
      e0_8_end = ncb_8 - 12'b1;
    end else begin
      e0_8_end = k0_e_mod_ncb_8 - 12'b1;
    end
  end

  assign e1_8_start = 12'b0;
  assign e1_8_end = k0_e_mod_ncb_8 - 12'b1;  
  
  always @ (*) begin
    if (i_harq_end & (sto_cs != STO_IDLE)) begin
      sto_ns = STO_ERR;
    end else begin
      sto_ns = sto_cs;
      case(sto_cs)
       STO_IDLE : begin
         if (i_harq_start & (i_cb_num != 5'd0)) begin
           sto_ns = STO_INI;
         end
       end
       STO_INI : begin
         sto_ns = STO_CFG0;
       end
       STO_CFG0 : begin
         if (k0e0_mod_ncb <= i_ncb_size) begin
           sto_ns = STO_CFG1;
         end
       end
       STO_CFG1 : begin
         if (k0e1_mod_ncb <= i_ncb_size) begin
           sto_ns = STO_WAIT_DEC_WR_START;
         end
       end
       STO_WAIT_DEC_WR_START : begin
         if (i_pdsch_dec_wr_start) begin
           sto_ns = STO_E_CFG0;
         end
       end
       STO_E_CFG0 : begin
         sto_ns = STO_E_CFG1;
       end
       STO_E_CFG1 : begin
         sto_ns = WAIT_STO_START;
       end
       WAIT_STO_START : begin
         if (((combine0_rdy & !ncb_ptr) | (combine1_rdy & ncb_ptr)) & i_pdsch_dec_ncb_rdy) begin
           sto_ns = STO_START;
         end
       end
       STO_START : begin
         if (e_size >= {2'b0,i_ncb_size}) begin
           sto_ns = WAIT_CMD_STO_E0_B;
         end else begin
           if (k0e <= i_ncb_size) begin
             sto_ns = WAIT_CMD_STO_Z0;
           end else begin
             sto_ns = WAIT_CMD_STO_Z2;
           end           
         end
       end
       WAIT_CMD_STO_Z0 : begin
         if (z0_8 == 12'b0) begin
           sto_ns = WAIT_CMD_STO_Z1;
         end else if (i_wr_cmd_done) begin
           sto_ns = CMD_STO_Z0;
         end 
       end
       CMD_STO_Z0 : begin
         sto_ns = WAIT_START_DATA_STO_Z0;
       end
       WAIT_START_DATA_STO_Z0 : begin
         if (!i_wfull) begin
           sto_ns = DATA_STO_Z0;
         end
       end
       DATA_STO_Z0 : begin
         if (!i_wfull && (rd_ncb_buf_cnt == z0_8 - 12'b1)) begin
           sto_ns = WAIT_DATA_STO_Z0_END;
         end
       end
       WAIT_DATA_STO_Z0_END : begin
         if (!i_wfull) begin
           if (e0_8_end == ncb_8 - 12'b1) begin
             sto_ns = WAIT_CMD_STO_E0_A;
           end else begin
             sto_ns = WAIT_CMD_STO_Z1;
           end
         end
       end
       WAIT_CMD_STO_Z1 : begin
         if (z1_8 == 12'b0) begin
           sto_ns = WAIT_CMD_STO_E0_A;
         end else if (i_wr_cmd_done) begin
           sto_ns = CMD_STO_Z1;
         end 
       end
       CMD_STO_Z1 : begin
         sto_ns = WAIT_START_DATA_STO_Z1;
       end
       WAIT_START_DATA_STO_Z1 : begin
         if (!i_wfull) begin
           sto_ns = DATA_STO_Z1;
         end
       end
       DATA_STO_Z1 : begin
         if (!i_wfull && (rd_ncb_buf_cnt == z1_8 - 12'b1)) begin
           sto_ns = WAIT_DATA_STO_Z1_END;
         end
       end
       WAIT_DATA_STO_Z1_END : begin
         if (!i_wfull) begin
           sto_ns = WAIT_CMD_STO_E0_A;         
         end
       end
       WAIT_CMD_STO_E0_A : begin
         if (i_wr_cmd_done && ((e0_burst_cnt != 11'b0) | harq_rdy) ) begin
           sto_ns = CMD_STO_E0_A;
         end 
       end
       CMD_STO_E0_A : begin
         sto_ns = WAIT_START_DATA_STO_E0_A;
       end
       WAIT_START_DATA_STO_E0_A : begin
         if (!i_wfull) begin
           sto_ns = DATA_STO_E0_A;
         end
       end
       DATA_STO_E0_A : begin
         if (!i_wfull && (rd_ncb_buf_cnt == o_wr_data_number[11:0] - 12'b1)) begin
           sto_ns = WAIT_DATA_STO_E0_A_END;
         end
       end
       WAIT_DATA_STO_E0_A_END : begin
         if (!i_wfull) begin
           sto_ns = STO_E0_A_END;         
         end
       end
       STO_E0_A_END : begin
         if (!last_burst && (burst_num != max_burst_cnt[11:0])) begin
           sto_ns = WAIT_NXT_CMD_E0_A;
         end else begin
           sto_ns = STO_END;
         end
       end
       WAIT_NXT_CMD_E0_A : begin
         if (i_wr_cmd_done && ((e0_burst_cnt != 11'b0) | harq_rdy) ) begin
           sto_ns = CMD_STO_E0_A;
         end
       end
       WAIT_CMD_STO_Z2 : begin
         if (z2_8 == 12'b0) begin
           sto_ns = WAIT_CMD_STO_E0_B;
         end else if (i_wr_cmd_done) begin
           sto_ns = CMD_STO_Z2;
         end 
       end
       CMD_STO_Z2 : begin
         sto_ns = WAIT_START_DATA_STO_Z2;
       end
       WAIT_START_DATA_STO_Z2 : begin
         if (!i_wfull) begin
           sto_ns = DATA_STO_Z2;
         end
       end
       DATA_STO_Z2 : begin
         if (!i_wfull && (rd_ncb_buf_cnt == z2_8 - 12'b1)) begin
           sto_ns = WAIT_DATA_STO_Z2_END;
         end
       end
       WAIT_DATA_STO_Z2_END : begin
         if (!i_wfull) begin
           sto_ns = WAIT_CMD_STO_E0_B;
         end
       end
       WAIT_CMD_STO_E0_B : begin
         if (i_wr_cmd_done && ((e0_burst_cnt != 11'b0) | harq_ncb_rdy) ) begin
           sto_ns = CMD_STO_E0_B;
         end 
       end
       CMD_STO_E0_B : begin
         sto_ns = WAIT_START_DATA_STO_E0_B;
       end
       WAIT_START_DATA_STO_E0_B : begin
         if (!i_wfull) begin
           sto_ns = DATA_STO_E0_B;
         end
       end
       DATA_STO_E0_B : begin
         if (!i_wfull && (rd_ncb_buf_cnt == o_wr_data_number[11:0] - 12'b1)) begin
           sto_ns = WAIT_DATA_STO_E0_B_END;
         end
       end
       WAIT_DATA_STO_E0_B_END : begin
         if (!i_wfull) begin
           sto_ns = STO_E0_B_END;         
         end
       end
       STO_E0_B_END : begin
         if (!last_burst && (burst_num != max_burst_cnt[11:0])) begin
           sto_ns = WAIT_NXT_CMD_E0_B;
         end else begin
           if (e0_8_start != 12'b0) begin
             sto_ns = WAIT_CMD_STO_E1_B;
           end else begin
             sto_ns = STO_WO_E1_END;
           end
         end
       end
       STO_WO_E1_END : begin
         sto_ns = STO_END;
       end
       WAIT_NXT_CMD_E0_B : begin
         if (i_wr_cmd_done && ((e0_burst_cnt != 11'b0) | harq_ncb_rdy) ) begin
           sto_ns = CMD_STO_E0_B;
         end
       end
       WAIT_CMD_STO_E1_B : begin
         if (i_wr_cmd_done && ((e1_burst_cnt != 11'b0) | harq_rdy) ) begin
           sto_ns = CMD_STO_E1_B;
         end 
       end
       CMD_STO_E1_B : begin
         sto_ns = WAIT_START_DATA_STO_E1_B;
       end
       WAIT_START_DATA_STO_E1_B : begin
         if (!i_wfull) begin
           sto_ns = DATA_STO_E1_B;
         end
       end
       DATA_STO_E1_B : begin
         if (!i_wfull && (rd_ncb_buf_cnt == o_wr_data_number[11:0] - 12'b1)) begin
           sto_ns = WAIT_DATA_STO_E1_B_END;
         end
       end
       WAIT_DATA_STO_E1_B_END : begin
         if (!i_wfull) begin
           sto_ns = STO_E1_B_END;         
         end
       end
       STO_E1_B_END : begin
         if (!last_burst && (burst_num != max_burst_cnt[11:0])) begin
           sto_ns = WAIT_NXT_CMD_E1_B;
         end else begin
           sto_ns = STO_END;
         end
       end
       WAIT_NXT_CMD_E1_B : begin
         if (i_wr_cmd_done && ((e1_burst_cnt != 11'b0) | harq_rdy) ) begin
           sto_ns = CMD_STO_E1_B;
         end
       end
       STO_END : begin
         if (cb_num_cnt != i_cb_num - 5'b1) begin
           sto_ns = STO_E_CFG0;
         end else begin
           sto_ns = STO_END_ALL;
         end
       end 
       STO_END_ALL : begin
         sto_ns = STO_IDLE;
       end
       STO_ERR : begin
         sto_ns = STO_IDLE;
       end
       default : begin
         sto_ns = STO_IDLE;
       end
      endcase
    end
  end


  //---------------------------------------------------------------------------
  // Flip-flops
  //---------------------------------------------------------------------------
  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    sto_cs <= STO_IDLE;
  end else begin
    sto_cs <= sto_ns;
  end

  // keep the addr when wfull is assert
  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    prefetch_raddr <= 12'b0;
  end else if (o_sto_ren & !i_wfull) begin
    prefetch_raddr <= int_raddr;
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    harq0_rdy <= 1'b0;
  end else begin
    if (i_harq_start) begin
      harq0_rdy <= 1'b0;
    end else if (harq0_clr) begin
      harq0_rdy <= 1'b0;
    end else if (i_harq0_done) begin
      harq0_rdy <= 1'b1;
    end
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    harq1_rdy <= 1'b0;
  end else begin
    if (i_harq_start) begin
      harq1_rdy <= 1'b0;
    end else if (harq1_clr) begin
      harq1_rdy <= 1'b0;
    end else if (i_harq1_done) begin
      harq1_rdy <= 1'b1;
    end
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    combine0_rdy <= 1'b0;
  end else begin
    if (i_harq_start) begin
      combine0_rdy <= 1'b0;
    end else if (combine0_clr) begin
      combine0_rdy <= 1'b0;
    end else if (i_harq0_combine_start) begin
      combine0_rdy <= 1'b1;
    end
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    combine1_rdy <= 1'b0;
  end else begin
    if (i_harq_start) begin
      combine1_rdy <= 1'b0;
    end else if (combine1_clr) begin
      combine1_rdy <= 1'b0;
    end else if (i_harq1_combine_start) begin
      combine1_rdy <= 1'b1;
    end
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    harq0_ncb_rdy <= 1'b0;
  end else begin
    if (i_harq_start) begin
      harq0_ncb_rdy <= 1'b0;
    end else if (harq0_ncb_clr) begin
      harq0_ncb_rdy <= 1'b0;
    end else if (i_harq0_ncb_done) begin
      harq0_ncb_rdy <= 1'b1;
    end
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    harq1_ncb_rdy <= 1'b0;
  end else begin
    if (i_harq_start) begin
      harq1_ncb_rdy <= 1'b0;
    end else if (harq1_ncb_clr) begin
      harq1_ncb_rdy <= 1'b0;
    end else if (i_harq1_ncb_done) begin
      harq1_ncb_rdy <= 1'b1;
    end
  end
  
  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    harq0_e0_burst_cnt <= 12'b0;
  end else if (i_harq0_done| i_harq0_ncb_done | i_harq_start | i_harq0_combine_start) begin 
    harq0_e0_burst_cnt <= 12'b0;
  end else begin
    case({i_harq0_e0_burst_en , harq0_e0_burst_decr})
      2'b00, 2'b11 : harq0_e0_burst_cnt <= harq0_e0_burst_cnt;
      2'b01 : harq0_e0_burst_cnt <= harq0_e0_burst_cnt - 12'b1;
      2'b10 : harq0_e0_burst_cnt <= harq0_e0_burst_cnt + 12'b1;
      default : harq0_e0_burst_cnt <= harq0_e0_burst_cnt;
    endcase
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    harq1_e0_burst_cnt <= 12'b0;
  end else if (i_harq1_done| i_harq1_ncb_done | i_harq_start | i_harq1_combine_start) begin
    harq1_e0_burst_cnt <= 12'b0;
  end else begin
    case({i_harq1_e0_burst_en , harq1_e0_burst_decr})
      2'b00, 2'b11 : harq1_e0_burst_cnt <= harq1_e0_burst_cnt;
      2'b01 : harq1_e0_burst_cnt <= harq1_e0_burst_cnt - 12'b1;
      2'b10 : harq1_e0_burst_cnt <= harq1_e0_burst_cnt + 12'b1;
      default : harq1_e0_burst_cnt <= harq1_e0_burst_cnt;
    endcase
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    harq0_e1_burst_cnt <= 12'b0;
  end else if (i_harq0_done | i_harq0_ncb_done | i_harq_start | i_harq0_combine_start) begin
    harq0_e1_burst_cnt <= 12'b0;
  end else begin
    case({i_harq0_e1_burst_en , harq0_e1_burst_decr})
      2'b00, 2'b11 : harq0_e1_burst_cnt <= harq0_e1_burst_cnt;
      2'b01 : harq0_e1_burst_cnt <= harq0_e1_burst_cnt - 12'b1;
      2'b10 : harq0_e1_burst_cnt <= harq0_e1_burst_cnt + 12'b1;
      default : harq0_e1_burst_cnt <= harq0_e1_burst_cnt;
    endcase
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    harq1_e1_burst_cnt <= 12'b0;
  end else if (i_harq1_done | i_harq1_ncb_done | i_harq_start | i_harq1_combine_start) begin
    harq1_e1_burst_cnt <= 12'b0;
  end else begin
    case({i_harq1_e1_burst_en , harq1_e1_burst_decr})
      2'b00, 2'b11 : harq1_e1_burst_cnt <= harq1_e1_burst_cnt;
      2'b01 : harq1_e1_burst_cnt <= harq1_e1_burst_cnt - 12'b1;
      2'b10 : harq1_e1_burst_cnt <= harq1_e1_burst_cnt + 12'b1;
      default : harq1_e1_burst_cnt <= harq1_e1_burst_cnt;
    endcase
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    ncb_ptr <= 1'b0;
    o_sto0_done <= 1'b0;
    o_sto1_done <= 1'b0;
    harq0_clr <= 1'b0;
    harq1_clr <= 1'b0;
    combine0_clr <= 1'b0;
    combine1_clr <= 1'b0;
    harq0_ncb_clr <= 1'b0;
    harq1_ncb_clr <= 1'b0;
 
    k0e0_mod_ncb <= 18'b0;
    k0e1_mod_ncb <= 18'b0;

    e_size <= 17'b0;
    k0e <= 18'b0;
    ncb_8 <= 12'b0;
    k0e_mod_ncb <= 15'b0;
    
    e0_8 <= 12'b0;
    e1_8 <= 12'b0;        
    z0_8 <= 12'b0;
    z1_8 <= 12'b0;        
    z2_8 <= 12'b0;                
 
    o_wr_cmd_strb <= 1'b0;
    o_wr_data_number <= 16'b0;
    o_wr_baddr <= 32'b0;
    baddr <= 32'b0;
    ncb_baddr <= 32'b0;        
 
    int_raddr <= 12'b0;
    int_ren <= 1'b0;
    int_wr <= 1'b0;
    int_ncb_wr <= 1'b0;
    data_zero_en <= 1'b0;
 
    o_sto_err <= 1'b0;
    o_sto_ch_done <= 1'b0;
    rd_ncb_buf_cnt <= 12'b0;
    cb_num_cnt <= 5'b0;

    int_e_bmp_fmt <= 26'b0;        

    last_burst <= 1'b0;
    burst_num <= 12'b0;
    e0_burst_decr <= 1'b0;
    e1_burst_decr <= 1'b0;
    
  end else begin
    case(sto_cs)
      STO_IDLE : begin

        ncb_ptr <= 1'b0;
        o_sto0_done <= 1'b0;
        o_sto1_done <= 1'b0;
        harq0_clr <= 1'b0;
        harq1_clr <= 1'b0;
        combine0_clr <= 1'b0;
        combine1_clr <= 1'b0;
        harq0_ncb_clr <= 1'b0;
        harq1_ncb_clr <= 1'b0;
 
        k0e0_mod_ncb <= 18'b0;
        k0e1_mod_ncb <= 18'b0;

        e_size <= 17'b0;
        k0e <= 18'b0;
        ncb_8 <= 12'b0;
        k0e_mod_ncb <= 15'b0;
        
        e0_8 <= 12'b0;
        e1_8 <= 12'b0;        
        z0_8 <= 12'b0;
        z1_8 <= 12'b0;        
        z2_8 <= 12'b0;                
 
        o_wr_cmd_strb <= 1'b0;
        o_wr_data_number <= 16'b0;
        o_wr_baddr <= 32'b0;
        baddr <= 32'b0;
        ncb_baddr <= 32'b0;        
 
        int_raddr <= 12'b0;
        int_ren <= 1'b0;
	int_wr <= 1'b0;
        int_ncb_wr <= 1'b0;

        data_zero_en <= 1'b0;
 
        o_sto_err <= 1'b0;
        o_sto_ch_done <= 1'b0;
        rd_ncb_buf_cnt <= 12'b0;
        cb_num_cnt <= 5'b0;

        int_e_bmp_fmt <= 26'b0;        

        last_burst <= 1'b0;
        burst_num <= 12'b0;
        e0_burst_decr <= 1'b0;
        e1_burst_decr <= 1'b0;

      end
      STO_INI : begin
        int_e_bmp_fmt <= i_e_bmp_fmt;
        ncb_baddr <= i_tb_harq_baddr;
        k0e0_mod_ncb <= k0e0;
        k0e1_mod_ncb <= k0e1;        
      end
      STO_CFG0 : begin
        if (k0e0_mod_ncb > i_ncb_size) begin // k0e0_mod_ncb mod Ncb
          k0e0_mod_ncb <= k0e0_mod_ncb - {3'b0, i_ncb_size};
        end
      end
      STO_CFG1 : begin
        if (k0e1_mod_ncb > i_ncb_size) begin // k0e1_mod_ncb mod Ncb
          k0e1_mod_ncb <= k0e1_mod_ncb - {3'b0, i_ncb_size};
        end
      end
      STO_WAIT_DEC_WR_START : begin
        k0e0_mod_ncb <= k0e0_mod_ncb;
        k0e1_mod_ncb <= k0e1_mod_ncb;
      end      
      STO_E_CFG0 : begin
        e_size <= int_e_bmp_fmt[0] ? i_e1_size : i_e0_size;
        k0e <= int_e_bmp_fmt[0] ? k0e1 : k0e0;
        ncb_8 <= i_ncb_size[14:3] + (|i_ncb_size[2:0]);
        k0e_mod_ncb <= int_e_bmp_fmt[0] ? k0e1_mod_ncb[14:0] : k0e0_mod_ncb[14:0];

	o_sto0_done <= 1'b0;
	o_sto1_done <= 1'b0;
      end
      STO_E_CFG1 : begin
        e0_8 <= e0_8_end - e0_8_start + 12'b1;
        e1_8 <= e1_8_end - e1_8_start + 12'b1;
        
        if (i_k0_pos[14:3] == 12'b0) begin
          z0_8 <= 12'b0;
        end else begin
          z0_8 <= z0_8_end - z0_8_start + 12'b1;
        end        
        if (k0_e_mod_ncb_8 == ncb_8) begin
          z1_8 <= 12'b0;
        end else begin
          z1_8 <= z1_8_end - z1_8_start + 12'b1;
        end
        if ((e1_8_end == e0_8_start) | e0_8_start == 12'b0) begin
          z2_8 <= 12'b0;
        end else begin
          z2_8 <= z2_8_end - z2_8_start + 12'b1;
        end
        
        int_e_bmp_fmt <= {1'b0, int_e_bmp_fmt[25:1]};
      end
      WAIT_STO_START : begin
        ncb_ptr <= ncb_ptr;
      end
      STO_START : begin
        combine0_clr <= !ncb_ptr;
        combine1_clr <= ncb_ptr;
      end
      WAIT_CMD_STO_Z0 : begin
        combine0_clr <= 1'b0;
        combine1_clr <= 1'b0;
        baddr <= ncb_baddr;
        int_raddr <= 12'b0;
      end
      CMD_STO_Z0 : begin
        o_wr_cmd_strb <= i_ndi & !i_combine_disable_flag;
        o_wr_baddr <= baddr;
	o_wr_data_number <= {4'b0, z0_8};
      end
      WAIT_START_DATA_STO_Z0 : begin
        int_ren <= 1'b1;
	int_wr <= int_ren & i_ndi & !i_combine_disable_flag;
        int_ncb_wr <= int_ren;

        rd_ncb_buf_cnt <= 12'b0;
        o_wr_cmd_strb <= 1'b0;
      end
      DATA_STO_Z0 : begin
        if (!i_wfull) begin
          int_raddr <= int_raddr + 12'b1;
          rd_ncb_buf_cnt <= rd_ncb_buf_cnt + 12'b1;
          if (rd_ncb_buf_cnt == z0_8 - 12'b1) begin
            int_ren <= 1'b0;
          end else begin
            int_ren <= 1'b1;
	  end
        end
	int_wr <= int_ren & i_ndi & !i_combine_disable_flag;
	int_ncb_wr <= int_ren;

        data_zero_en <= i_ndi;
      end
      WAIT_DATA_STO_Z0_END : begin
	int_ren <= 1'b0;
	int_wr <= (int_ren | i_wfull) & i_ndi & !i_combine_disable_flag;
	int_ncb_wr <= (int_ren | i_wfull);

      end
      WAIT_CMD_STO_Z1 : begin
        baddr <= ncb_baddr + {z1_8_start, 3'b0};
        int_raddr <= z1_8_start;
	int_wr <= 1'b0;
        int_ncb_wr <= 1'b0;
      end
      CMD_STO_Z1 : begin
        o_wr_cmd_strb <= i_ndi & !i_combine_disable_flag;
        o_wr_baddr <= baddr;
	o_wr_data_number <= {4'b0, z1_8};
      end
      WAIT_START_DATA_STO_Z1 : begin
        int_ren <= 1'b1;
	int_wr <= (int_ren | i_wfull) & i_ndi & !i_combine_disable_flag;
	int_ncb_wr <= (int_ren | i_wfull);
        rd_ncb_buf_cnt <= 12'b0;
        o_wr_cmd_strb <= 1'b0;
      end
      DATA_STO_Z1 : begin
        if (!i_wfull) begin
          int_raddr <= int_raddr + 12'b1;
          rd_ncb_buf_cnt <= rd_ncb_buf_cnt + 12'b1;
          if (rd_ncb_buf_cnt == z1_8 - 12'b1) begin
            int_ren <= 1'b0;
          end else begin
            int_ren <= 1'b1;
	  end
        end
	int_wr <= int_ren & i_ndi & !i_combine_disable_flag;
	int_ncb_wr <= int_ren;
        data_zero_en <= i_ndi;
      end
      WAIT_DATA_STO_Z1_END : begin
	int_ren <= 1'b0;
	int_wr <= (int_ren | i_wfull) & i_ndi & !i_combine_disable_flag;
	int_ncb_wr <= (int_ren | i_wfull);
      end
      WAIT_CMD_STO_E0_A : begin
        combine0_clr <= 1'b0;
        combine1_clr <= 1'b0;
        data_zero_en <= 1'b0;
        baddr <= ncb_baddr + {e0_8_start, 3'b0};
        burst_num <= e0_8;
        int_raddr <= e0_8_start;        
	int_wr <= 1'b0;
	int_ncb_wr <= 1'b0;

      end
      CMD_STO_E0_A : begin
        o_wr_cmd_strb <= !i_combine_disable_flag;
        o_wr_baddr <= baddr;
        if (harq_rdy) begin
          o_wr_data_number <= {4'b0, burst_num};
          last_burst <= 1'b1;
          e0_burst_decr <= 1'b0;
        end else begin
          o_wr_data_number <= max_burst_cnt;
          last_burst <= 1'b0;
          e0_burst_decr <= 1'b1;
        end
        harq0_clr <= harq_rdy & !ncb_ptr;
        harq1_clr <= harq_rdy & ncb_ptr;
      end
      WAIT_START_DATA_STO_E0_A : begin
        harq0_clr <= 1'b0;
        harq1_clr <= 1'b0;
        int_ren <= 1'b1;
	int_wr <= int_ren & !i_combine_disable_flag;
	int_ncb_wr <= int_ren;
        rd_ncb_buf_cnt <= 12'b0;
        o_wr_cmd_strb <= 1'b0;
        e0_burst_decr <= 1'b0;
      end
      DATA_STO_E0_A : begin
        if (!i_wfull) begin
          int_raddr <= int_raddr + 12'b1;
          rd_ncb_buf_cnt <= rd_ncb_buf_cnt + 12'b1;
          if (rd_ncb_buf_cnt == o_wr_data_number[11:0] - 12'b1) begin
            int_ren <= 1'b0;
          end else begin
            int_ren <= 1'b1;
	  end
        end
	int_wr <= int_ren & !i_combine_disable_flag;
	int_ncb_wr <= int_ren;
      end
      WAIT_DATA_STO_E0_A_END : begin
	int_ren <= 1'b0;
	int_wr <= (int_ren | i_wfull) & !i_combine_disable_flag;
	int_ncb_wr <= int_ren | i_wfull;
      end
      STO_E0_A_END : begin
        rd_ncb_buf_cnt <= 12'b0;
        last_burst <= 1'b0;
        int_wr <= 1'b0;
	int_ncb_wr <= 1'b0;
        baddr <= baddr + {13'b0, max_burst_cnt, 3'b0};
        if (burst_num > max_burst_cnt) begin // keep the last packet for the harq_rdy signal
          burst_num <= burst_num - max_burst_cnt; 
        end else begin
          burst_num <= burst_num; 
        end
        if (burst_num == max_burst_cnt[11:0]) begin
          harq0_clr <= harq_rdy & !ncb_ptr;
          harq1_clr <= harq_rdy & ncb_ptr;
        end
      end
      WAIT_NXT_CMD_E0_A : begin
        baddr <= baddr;
        burst_num <= burst_num; 
      end
      WAIT_CMD_STO_Z2 : begin
        combine0_clr <= 1'b0;
        combine1_clr <= 1'b0;
        baddr <= ncb_baddr + {z2_8_start, 3'b0};
        int_raddr <= z2_8_start;
      end
      CMD_STO_Z2 : begin
        o_wr_cmd_strb <= i_ndi & !i_combine_disable_flag;
        o_wr_baddr <= baddr;
	o_wr_data_number <= {4'b0, z2_8};
      end
      WAIT_START_DATA_STO_Z2 : begin
        int_ren <= 1'b1;
	int_wr <= (int_ren & i_ndi) & !i_combine_disable_flag;
	int_ncb_wr <= int_ren;
        rd_ncb_buf_cnt <= 12'b0;
        o_wr_cmd_strb <= 1'b0;
      end
      DATA_STO_Z2 : begin
        if (!i_wfull) begin
          int_raddr <= int_raddr + 12'b1;
          rd_ncb_buf_cnt <= rd_ncb_buf_cnt + 12'b1;
          if (rd_ncb_buf_cnt == z2_8 - 12'b1) begin
            int_ren <= 1'b0;
          end else begin
            int_ren <= 1'b1;
	  end
        end
	int_wr <= (int_ren & i_ndi) & !i_combine_disable_flag;
	int_ncb_wr <= int_ren;
        data_zero_en <= i_ndi;
      end
      WAIT_DATA_STO_Z2_END : begin
	int_ren <= 1'b0;
	int_wr <= (int_ren | i_wfull) & i_ndi & !i_combine_disable_flag;
	int_ncb_wr <= (int_ren | i_wfull);
      end
      WAIT_CMD_STO_E0_B : begin
        combine0_clr <= 1'b0;
        combine1_clr <= 1'b0;
        data_zero_en <= 1'b0;
        baddr <= ncb_baddr + {e0_8_start, 3'b0};
        burst_num <= e0_8;
        int_raddr <= e0_8_start;                
	int_wr <= 1'b0;
	int_ncb_wr <= 1'b0;
      end
      CMD_STO_E0_B : begin
        o_wr_cmd_strb <= !i_combine_disable_flag;
        o_wr_baddr <= baddr;
        if (harq_ncb_rdy) begin
          o_wr_data_number <= {4'b0, burst_num};
          last_burst <= 1'b1;
          e0_burst_decr <= 1'b0;
        end else begin
          o_wr_data_number <= max_burst_cnt;
          last_burst <= 1'b0;
          e0_burst_decr <= 1'b1;
        end
        harq0_ncb_clr <= harq_ncb_rdy & !ncb_ptr;
        harq1_ncb_clr <= harq_ncb_rdy & ncb_ptr;
      end
      WAIT_START_DATA_STO_E0_B : begin
        int_ren <= 1'b1;
	int_wr <= int_ren & !i_combine_disable_flag;
	int_ncb_wr <= int_ren;
        rd_ncb_buf_cnt <= 12'b0;
        o_wr_cmd_strb <= 1'b0;
        e0_burst_decr <= 1'b0;
        harq0_ncb_clr <= 1'b0;
        harq1_ncb_clr <= 1'b0;
      end
      DATA_STO_E0_B : begin
        if (!i_wfull) begin
          int_raddr <= int_raddr + 12'b1;
          rd_ncb_buf_cnt <= rd_ncb_buf_cnt + 12'b1;
          if (rd_ncb_buf_cnt == o_wr_data_number[11:0] - 12'b1) begin
            int_ren <= 1'b0;
          end else begin
            int_ren <= 1'b1;
	  end
        end
	int_wr <= int_ren & !i_combine_disable_flag;
	int_ncb_wr <= int_ren;
      end
      WAIT_DATA_STO_E0_B_END : begin
	int_ren <= 1'b0;
	int_wr <= (int_ren | i_wfull) & !i_combine_disable_flag;
	int_ncb_wr <= int_ren | i_wfull;
      end
      STO_E0_B_END : begin
        rd_ncb_buf_cnt <= 12'b0;
        last_burst <= 1'b0;
	int_ren <= 1'b0;
        baddr <= baddr + {13'b0, max_burst_cnt, 3'b0};
        if (burst_num > max_burst_cnt) begin // keep the last packet for the harq_ncb_rdy signal
          burst_num <= burst_num - max_burst_cnt; 
        end else begin
          burst_num <= burst_num; 
        end
        if (burst_num == max_burst_cnt[11:0]) begin
          harq0_ncb_clr <= harq_ncb_rdy & !ncb_ptr;
          harq1_ncb_clr <= harq_ncb_rdy & ncb_ptr;
        end
      end
      WAIT_NXT_CMD_E0_B : begin
        baddr <= baddr;
        burst_num <= burst_num;
      end
      STO_WO_E1_END : begin
        harq0_clr <= harq_rdy & !ncb_ptr;
        harq1_clr <= harq_rdy & ncb_ptr;
      end
      WAIT_CMD_STO_E1_B : begin
        baddr <= ncb_baddr;
        burst_num <= e1_8;
        int_raddr <= 12'b0;        
	int_wr <= 1'b0;
	int_ncb_wr <= 1'b0;
      end
      CMD_STO_E1_B : begin
        o_wr_cmd_strb <= !i_combine_disable_flag;
        o_wr_baddr <= baddr;
        if (harq_rdy) begin
          o_wr_data_number <= {4'b0, burst_num};
          last_burst <= 1'b1;
          e1_burst_decr <= 1'b0;
        end else begin
          o_wr_data_number <= max_burst_cnt;
          last_burst <= 1'b0;
          e1_burst_decr <= 1'b1;
        end
        harq0_clr <= harq_rdy & !ncb_ptr;
        harq1_clr <= harq_rdy & ncb_ptr;
      end
      WAIT_START_DATA_STO_E1_B : begin
        harq0_clr <= 1'b0;
        harq1_clr <= 1'b0;
        int_ren <= 1'b1;
	int_wr <= int_ren & !i_combine_disable_flag;
	int_ncb_wr <= int_ren;
        rd_ncb_buf_cnt <= 12'b0;
        o_wr_cmd_strb <= 1'b0;
        e1_burst_decr <= 1'b0;
      end
      DATA_STO_E1_B : begin
        if (!i_wfull) begin
          int_raddr <= int_raddr + 12'b1;
          rd_ncb_buf_cnt <= rd_ncb_buf_cnt + 12'b1;
          if (rd_ncb_buf_cnt == o_wr_data_number[11:0] - 12'b1) begin
            int_ren <= 1'b0;
          end else begin
            int_ren <= 1'b1;
	  end
        end
	int_wr <= int_ren & !i_combine_disable_flag;
	int_ncb_wr <= int_ren;
      end
      WAIT_DATA_STO_E1_B_END : begin
	int_ren <= 1'b0;
	int_wr <= (int_ren | i_wfull) & !i_combine_disable_flag;
	int_ncb_wr <= int_ren | i_wfull;
      end
      STO_E1_B_END : begin
        rd_ncb_buf_cnt <= 12'b0;
        last_burst <= 1'b0;
        int_wr <= 1'b0;
        int_ncb_wr <= 1'b0;
        baddr <= baddr + {13'b0, max_burst_cnt, 3'b0};
        if (burst_num > max_burst_cnt) begin // keep the last packet for the harq_rdy signal
          burst_num <= burst_num - max_burst_cnt; 
        end else begin
          burst_num <= burst_num; 
        end
        if (burst_num == max_burst_cnt[11:0]) begin
          harq0_clr <= harq_rdy & !ncb_ptr;
          harq1_clr <= harq_rdy & ncb_ptr;      
        end
      end
      WAIT_NXT_CMD_E1_B : begin
        baddr <= baddr;
        burst_num <= burst_num;
      end
      STO_END : begin
        int_raddr <= 12'b0;
        rd_ncb_buf_cnt <= 12'b0;
        int_ren <= 1'b0;
        data_zero_en <= 1'b0;
        ncb_ptr <= !ncb_ptr;
        o_sto0_done <= !ncb_ptr;
        o_sto1_done <= ncb_ptr;
        ncb_baddr <= ncb_baddr + {17'b0, ncb_8, 3'b0}; 
	cb_num_cnt <= cb_num_cnt + 5'b1; 
	int_wr <= 1'b0;
	int_ncb_wr <= 1'b0;
      end
      STO_END_ALL : begin
        o_sto_ch_done <= 1'b1;
	o_sto0_done <= 1'b0;
	o_sto1_done <= 1'b0;
      end
      STO_ERR : begin
        o_sto_ch_done <= 1'b1;
        o_sto_err <= 1'b1;
      end
      default : begin
        ncb_ptr <= 1'b0;
        o_sto0_done <= 1'b0;
        o_sto1_done <= 1'b0;
        harq0_clr <= 1'b0;
        harq1_clr <= 1'b0;
        combine0_clr <= 1'b0;
        combine1_clr <= 1'b0;
        harq0_ncb_clr <= 1'b0;
        harq1_ncb_clr <= 1'b0;
 
        k0e0_mod_ncb <= 18'b0;
        k0e1_mod_ncb <= 18'b0;

        e_size <= 17'b0;
        k0e <= 18'b0;
        ncb_8 <= 12'b0;
        k0e_mod_ncb <= 15'b0;
        
        e0_8 <= 12'b0;
        e1_8 <= 12'b0;        
        z0_8 <= 12'b0;
        z1_8 <= 12'b0;        
        z2_8 <= 12'b0;                
 
        o_wr_cmd_strb <= 1'b0;
        o_wr_data_number <= 16'b0;
        o_wr_baddr <= 32'b0;
        baddr <= 32'b0;
        ncb_baddr <= 32'b0;        
 
        int_raddr <= 12'b0;
        int_ren <= 1'b0;
	int_wr <= 1'b0;
	int_ncb_wr <= 1'b0;
        data_zero_en <= 1'b0;
 
        o_sto_err <= 1'b0;
        o_sto_ch_done <= 1'b0;
        rd_ncb_buf_cnt <= 12'b0;
        cb_num_cnt <= 5'b0;

        int_e_bmp_fmt <= 26'b0;        

        last_burst <= 1'b0;
        burst_num <= 12'b0;
        e0_burst_decr <= 1'b0;
        e1_burst_decr <= 1'b0;
      end
    endcase
  end




endmodule

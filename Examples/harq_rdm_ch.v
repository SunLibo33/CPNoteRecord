// Description
// This module would opertaion harq combine and Rate dematching in one channel. 
//    
// correct the o_harq_ncb_done output in eacn NCB instead of all NCB
// modify for fine clock control

module harq_rdm_ch (
  // global signal
  input                 i_harq_rst_n,
  input                 i_mem_rst_n,
  input                 i_harq_free_clk,
  input                 i_mem_free_clk,
  input [1:0]           i_harq_clk,
  input [1:0]           i_mem_clk,
  input [2:0]           i_twm,

  // rx ctrl input control
  input                 i_pdsch_dec_wr_start, // PDSCH decoder write buffer start -- sto ncb should write the data after this signal
  input                 i_hclk_harq_start,
  input                 i_hclk_harq_end,

  input                 i_mclk_harq_start,
  input                 i_mclk_harq_end,
  
  // input TB configuration
  input [2-1:0]         i_dibit_mode,   // 00 : normal , 01 : master , 10 : slave
  input                 i_combine_disable_flag, // 0 : normal , 1 : disable the DDR transfer for HARQ combining process
  input [5-1:0]         i_cb_num,       // number of CB 
  input [26-1:0]        i_e_bmp_fmt,    // E bitmap format
  input [17-1:0]        i_e0_size,      // E0 size
  input [17-1:0]        i_e1_size,      // E1 size
  input [15-1:0]        i_k0_pos,       // K0 position
//  input [13-1:0]        i_d_size,       // D size
//  input [5-1:0]         i_nd_size,      // Nd size
  input                 i_ndi,          // New data indicator
  input [15-1:0]        i_ncb_size,     // Ncb size
//  input [18-1:0]        i_tb_size,      // TB size
  input [32-1:0]        i_tb_harq_baddr,// Harq Base address
//  input                 i_cw_swap_flag,
  input [2-1:0]         i_harq_lb_burst_len, // HARQ local bus interface burst length 0-16, 1-32, 2-64
    
  // interface for dibit mode for 2 layer in 1 codeword spatial multiplexing mode
  input [8-1:0]         i_slave_rcombine_data,   // for slave only read data from Ncb to combine
  output [8-1:0]        o_slave_rcombine_data,   // for master only Ncb data to slave combine module

  input  [8-1:0]        i_slave_combine_data,    //  master only for combine data input
  output [8-1:0]        o_slave_combine_data,    //  slave only for combine data output

  output                o_slave_rcombine_hold_req,    // master only
  output                o_slave_rcombine_rdy,         // master only

  input                 i_slave_rcombine_hold_req,    // slave only
  input                 i_slave_rcombine_rdy,         // slave only

  // input data stream
  input                 i_descr_data_strb,      // descramble data strobe
  input [8-1:0]         i_descr_data,           // descramble data

  // output data local write axi bus
  output                o_wr_cmd_strb,
  input                 i_wr_cmd_done,
  output                o_wr_width,  // 1 : 64 bit width 0: 32 bit width
  output [16-1:0]       o_wr_data_number,
  output [32-1:0]       o_wr_baddr,
  output                o_wr,
  output [64-1:0]       o_wdata,
  input                 i_wfull,
  output                o_wr_termi,

  // output data for PDSCH turbo decoder interface
  output [12-1:0]       o_harq_ncb_addr, // CB internal addresss for turbo decoder NCB buffer only
  output                o_harq_ncb_wr,
  output                o_harq_ncb_done,
  input                 i_pdsch_dec_ncb_rdy, // indicate the PDSCH decoder write NCB buffer ready

  // output data local read axi bus
  output                o_rd_cmd_strb,
  input                 i_rd_cmd_done,
  output                o_rd_width, // 1 : 64 bit width 0: 32 bit width
  output [16-1:0]       o_rd_data_number,
  output [32-1:0]       o_rd_baddr,
  output                o_rd,
  input  [64-1:0]       i_rdata,
  input                 i_rempty,  
  output                o_rd_termi,
  
  // for fine clock gating
  output                o_fetch_ncb_done,   // assert when all ncb finished
  output                o_combine_ncb_done, // assert when all ncb finished
  output                o_sto_ncb_done,     // assert when all ncb finished

  
  // error signal
  output                o_descr_buf_overflow,
  output                o_rcombine_err,
  output                o_wcombine_err,
  output                o_fetch_err,
  output                o_sto_err
  
);


  //---------------------------------------------------------------------------
  // Internal wire declaration
  //---------------------------------------------------------------------------
  wire                          fetch0_done;
  wire                          fetch1_done;
  wire                          harq0_done;
  wire                          harq1_done;
  wire                          harq0_ncb_done;
  wire                          harq1_ncb_done;
  wire                          harq0_e0_burst_en;
  wire                          harq0_e1_burst_en;
  wire                          harq1_e0_burst_en;
  wire                          harq1_e1_burst_en;
  wire                          harq0_combine_start;
  wire                          harq1_combine_start;

  wire                          sto0_done;
  wire                          sto1_done;

  wire                          sync_fetch0_done;
  wire                          sync_fetch1_done;

  wire                          sync_harq0_done;
  wire                          sync_harq1_done;
  wire                          sync_harq0_ncb_done;
  wire                          sync_harq1_ncb_done;
  wire                          sync_harq0_e0_burst_en;
  wire                          sync_harq0_e1_burst_en;
  wire                          sync_harq1_e0_burst_en;
  wire                          sync_harq1_e1_burst_en;
  wire                          sync_harq0_combine_start;
  wire                          sync_harq1_combine_start;

  wire                          rcombine_hold_req;    // request to hold wire at the boundary of the Ncb or E
  wire                          rcombine_rdy;         // read combine ready signal

  wire                          int_rcombine_hold_req;  // request to hold wire at the boundary of the Ncb or E after the dibit mode selection
  wire                          int_rcombine_rdy;       // read combine ready signal after the dibit mode selection

  wire                          descr_buf_data_strb;
  wire [8-1:0]                  descr_buf_data;
  
//  wire                          sto_ch_done;

  // memory aribter interface
  wire                          harq_en;
  wire [8-1:0]                  harq_wen;
  wire [12-1:0]                 harq_addr;
  wire [64-1:0]                 harq_wdata;
  wire  [64-1:0]                harq_rdata;
 
  wire                          fetch_ptr;
  wire                          fetch_wen;
  wire [12-1:0]                 fetch_addr;
  wire [64-1:0]                 fetch_wdata;

  wire                          sto_ptr;
  wire                          sto_ren;
  wire [12-1:0]                 sto_addr;
  wire [64-1:0]                 sto_rdata;

  // interface with ncb ping-pong memory 
  wire [8-1:0]                  ncb_arb0_harq_wen;
  wire                          ncb_arb0_harq_en;
  wire [12-1:0]                 ncb_arb0_harq_addr;
  wire [64-1:0]                 ncb_arb0_harq_wdata;
  wire  [64-1:0]                ncb_arb0_harq_rdata;
 
  wire [8-1:0]                  ncb_arb1_harq_wen;
  wire                          ncb_arb1_harq_en;
  wire [12-1:0]                 ncb_arb1_harq_addr;
  wire [64-1:0]                 ncb_arb1_harq_wdata;
  wire  [64-1:0]                ncb_arb1_harq_rdata;

  wire [8-1:0]                  ncb_arb0_mem_wen;
  wire                          ncb_arb0_mem_en;
  wire [12-1:0]                 ncb_arb0_mem_addr;
  wire [64-1:0]                 ncb_arb0_mem_wdata;
  wire [64-1:0]                 ncb_arb0_mem_rdata;
 
  wire [8-1:0]                  ncb_arb1_mem_wen;
  wire                          ncb_arb1_mem_en;
  wire [12-1:0]                 ncb_arb1_mem_addr;
  wire [64-1:0]                 ncb_arb1_mem_wdata;
  wire [64-1:0]                 ncb_arb1_mem_rdata;

//  wire [3-1:0]                  harq_data_type_float;
//  wire                          harq_cw_swap_flag_float;
  //---------------------------------------------------------------------------
  // Combinational logic
  //---------------------------------------------------------------------------
  assign o_slave_rcombine_hold_req = rcombine_hold_req;
  assign o_slave_rcombine_rdy = rcombine_rdy;
  
  assign int_rcombine_hold_req = (i_dibit_mode==2'b10) ? i_slave_rcombine_hold_req : rcombine_hold_req;
  assign int_rcombine_rdy = (i_dibit_mode==2'b10) ? i_slave_rcombine_rdy : rcombine_rdy;

  assign o_rd_width = 1'b1;
  assign o_wr_width = 1'b1;
  assign o_harq_ncb_done = sto0_done | sto1_done;

  //---------------------------------------------------------------------------
  // Instantiation of submodules 
  //---------------------------------------------------------------------------


  // -----------------------------------------------------
  // HARQ clock domain
  // -----------------------------------------------------
  domain_switch_reg U_SYNC_FETCH0_DONE (

    .rst_a_n(i_mem_rst_n),
    .rst_b_n(i_harq_rst_n),
    .clk_a(i_mem_free_clk),
    .clk_b(i_harq_free_clk),
    .a(fetch0_done),
    .b(sync_fetch0_done)

  );

  domain_switch_reg U_SYNC_FETCH1_DONE (

    .rst_a_n(i_mem_rst_n),
    .rst_b_n(i_harq_rst_n),
    .clk_a(i_mem_free_clk),
    .clk_b(i_harq_free_clk),
    .a(fetch1_done),
    .b(sync_fetch1_done)

  );

  descr_buf_regular DESCR_BUF_REGULAR (
    // global signal
    .i_rst_n(i_harq_rst_n),
    .i_harq_clk(i_harq_clk[0]),
    .i_twm(i_twm),

    // rx ctrl input control
    .i_harq_start(i_hclk_harq_start),
    .i_harq_end(i_hclk_harq_end),

    // input data stream
    .i_descr_data_strb(i_descr_data_strb),      // descramble data strobe
    .i_descr_data(i_descr_data),                // descramble data
  
    // output data control
    .i_rcombine_hold_req(int_rcombine_hold_req),    // request to hold output at the boundary of the Ncb or E
    .i_rcombine_rdy(int_rcombine_rdy),              // read combine ready signal

    // output data stream
    .o_descr_buf_data_strb(descr_buf_data_strb),
    .o_descr_buf_data(descr_buf_data),
 
    // error control
    .o_descr_buf_overflow(o_descr_buf_overflow)
  
  );


  combine_top  COMBINE_TOP (
    // global signal
    .i_rst_n(i_harq_rst_n),
    .i_harq_clk(i_harq_clk[1]),

    // rx ctrl input control
    .i_harq_start(i_hclk_harq_start),
    .i_harq_end(i_hclk_harq_end),
 
    // input TB configuration
    .i_dibit_mode(i_dibit_mode),       // 00 : normal , 01 : master , 10 : slave
    .i_cb_num(i_cb_num),               // number of CB
    .i_e_bmp_fmt(i_e_bmp_fmt),         // E bitmap format
    .i_e0_size(i_e0_size),             // E0 size
    .i_e1_size(i_e1_size),             // E1 size
    .i_k0_pos(i_k0_pos),               // K0 position
    .i_ndi(i_ndi),                     // New data indicator
    .i_ncb_size(i_ncb_size),           // Ncb size
//    .i_d_size(i_d_size),               // D size
//    .i_nd_size(i_nd_size),             // Nd size
//    .i_tb_size(i_tb_size),             // TB size
//    .i_tb_harq_baddr(i_tb_harq_baddr), // Harq Base address
    .i_harq_lb_burst_len(i_harq_lb_burst_len), // HARQ local bus interface burst length 0-16, 1-32, 2-64
    
    // interface with descr_buf
    .i_descr_buf_data_strb(descr_buf_data_strb),   // descramble data strobe
    .i_descr_buf_data(descr_buf_data),             // descramble data
    .o_rcombine_hold_req(rcombine_hold_req),       // request to hold few cycle at the boundary of Ncb
    .o_rcombine_rdy(rcombine_rdy),                 // read combine data ready

    // interface for dibit mode for 2 layer in 1 codeword spatial multiplexing mode
    .i_slave_rcombine_data(i_slave_rcombine_data),  // for slave only read data from Ncb to combine
    .o_slave_rcombine_data(o_slave_rcombine_data),  // for master only Ncb data to slave combine module

    .i_slave_combine_data(i_slave_combine_data),  // master only for combine data input
    .o_slave_combine_data(o_slave_combine_data),  // slave only for combine data output

    // interface with fetch_ncb
    .i_fetch0_done(sync_fetch0_done),
    .i_fetch1_done(sync_fetch1_done),

    // interface with sto_ncb
    .o_harq0_done(harq0_done),
    .o_harq1_done(harq1_done),
    .o_harq0_ncb_done(harq0_ncb_done),
    .o_harq1_ncb_done(harq1_ncb_done),
    .o_harq0_e0_burst_en(harq0_e0_burst_en),
    .o_harq0_e1_burst_en(harq0_e1_burst_en),
    .o_harq1_e0_burst_en(harq1_e0_burst_en),
    .o_harq1_e1_burst_en(harq1_e1_burst_en),
    .o_harq0_combine_start(harq0_combine_start),
    .o_harq1_combine_start(harq1_combine_start),

    // interface with ncb_arb
    .o_harq_en(harq_en),
    .o_harq_wen(harq_wen),
    .o_harq_addr(harq_addr),
    .o_harq_wdata(harq_wdata),
    .i_harq_rdata(harq_rdata),
 
    // for fine clock gating
    .o_combine_ncb_done(o_combine_ncb_done),

    // error signal
    .o_rcombine_err(o_rcombine_err),
    .o_wcombine_err(o_wcombine_err)
    
  );

  // -----------------------------------------------------
  // memory clock domain
  // -----------------------------------------------------
  domain_switch_reg SYNC_STO0_DONE (

    .rst_a_n(i_harq_rst_n),
    .rst_b_n(i_mem_rst_n),
    .clk_a(i_harq_free_clk),
    .clk_b(i_mem_free_clk),
    .a(harq0_done),
    .b(sync_harq0_done)

  );

  domain_switch_reg SYNC_STO1_DONE (

    .rst_a_n(i_harq_rst_n),
    .rst_b_n(i_mem_rst_n),
    .clk_a(i_harq_free_clk),
    .clk_b(i_mem_free_clk),
    .a(harq1_done),
    .b(sync_harq1_done)

  );

  domain_switch_reg SYNC_STO0_NCB_DONE (

    .rst_a_n(i_harq_rst_n),
    .rst_b_n(i_mem_rst_n),
    .clk_a(i_harq_free_clk),
    .clk_b(i_mem_free_clk),
    .a(harq0_ncb_done),
    .b(sync_harq0_ncb_done)

  );

  domain_switch_reg SYNC_STO1_NCB_DONE (

    .rst_a_n(i_harq_rst_n),
    .rst_b_n(i_mem_rst_n),
    .clk_a(i_harq_free_clk),
    .clk_b(i_mem_free_clk),
    .a(harq1_ncb_done),
    .b(sync_harq1_ncb_done)

  );

  domain_switch_reg SYNC_STO0_E0_BURST_EN (

    .rst_a_n(i_harq_rst_n),
    .rst_b_n(i_mem_rst_n),
    .clk_a(i_harq_free_clk),
    .clk_b(i_mem_free_clk),
    .a(harq0_e0_burst_en),
    .b(sync_harq0_e0_burst_en)

  );

  domain_switch_reg SYNC_STO0_E1_BURST_EN (

    .rst_a_n(i_harq_rst_n),
    .rst_b_n(i_mem_rst_n),
    .clk_a(i_harq_free_clk),
    .clk_b(i_mem_free_clk),
    .a(harq0_e1_burst_en),
    .b(sync_harq0_e1_burst_en)

  );

  domain_switch_reg SYNC_STO1_E0_BURST_EN (

    .rst_a_n(i_harq_rst_n),
    .rst_b_n(i_mem_rst_n),
    .clk_a(i_harq_free_clk),
    .clk_b(i_mem_free_clk),
    .a(harq1_e0_burst_en),
    .b(sync_harq1_e0_burst_en)

  );

  domain_switch_reg SYNC_STO1_E1_BURST_EN (

    .rst_a_n(i_harq_rst_n),
    .rst_b_n(i_mem_rst_n),
    .clk_a(i_harq_free_clk),
    .clk_b(i_mem_free_clk),
    .a(harq1_e1_burst_en),
    .b(sync_harq1_e1_burst_en)

  );

  domain_switch_reg SYNC_STO0_COMBINE_START (

    .rst_a_n(i_harq_rst_n),
    .rst_b_n(i_mem_rst_n),
    .clk_a(i_harq_free_clk),
    .clk_b(i_mem_free_clk),
    .a(harq0_combine_start),
    .b(sync_harq0_combine_start)

  );

  domain_switch_reg SYNC_STO1_COMBINE_START (

    .rst_a_n(i_harq_rst_n),
    .rst_b_n(i_mem_rst_n),
    .clk_a(i_harq_free_clk),
    .clk_b(i_mem_free_clk),
    .a(harq1_combine_start),
    .b(sync_harq1_combine_start)

  );

  fetch_ncb FETCH_NCB (

    // global signal
    .i_rst_n(i_mem_rst_n),
    .i_mem_clk(i_mem_clk[0]),

    // rx ctrl input control
    .i_harq_start(i_mclk_harq_start),
    .i_harq_end(i_mclk_harq_end),

    // input configuration
    .i_cb_num(i_cb_num),                // number of CB
    .i_e_bmp_fmt(i_e_bmp_fmt),          // E bitmap format
    .i_e0_size(i_e0_size),              // E0 size
    .i_e1_size(i_e1_size),              // E1 size
    .i_k0_pos(i_k0_pos),                // K0 position
    .i_ndi(i_ndi),                      // New data indicator
    .i_ncb_size(i_ncb_size),            // Ncb size
    .i_tb_harq_baddr(i_tb_harq_baddr),  // Harq Base address

    // control signal to combine_top
    .o_fetch0_done(fetch0_done),
    .o_fetch1_done(fetch1_done),
    .o_fetch_ncb_done(o_fetch_ncb_done),
 
    // control signal from sto_ncb
    .i_sto0_done(sto0_done),
    .i_sto1_done(sto1_done),
 
    // output signal for local ncb buffer
    .o_fetch_ptr(fetch_ptr),
    .o_fetch_wen(fetch_wen),
    .o_fetch_addr(fetch_addr),
    .o_fetch_wdata(fetch_wdata),
 
    // output data local read axi bus
    .o_rd_cmd_strb(o_rd_cmd_strb),
    .i_rd_cmd_done(i_rd_cmd_done),
    .o_rd_data_number(o_rd_data_number),
    .o_rd_baddr(o_rd_baddr),
    .o_rd(o_rd),
    .i_rdata(i_rdata),
    .i_rempty(i_rempty),
    .o_rd_termi(o_rd_termi),
 
    // erro control
    .o_fetch_err(o_fetch_err)
  
  );

  sto_ncb STO_NCB (

    // global signal
    .i_rst_n(i_mem_rst_n),
    .i_mem_clk(i_mem_clk[1]),

    // rx ctrl input control
    .i_harq_start(i_mclk_harq_start),
    .i_harq_end(i_mclk_harq_end),
    .i_pdsch_dec_wr_start(i_pdsch_dec_wr_start), // PDSCH decoder write buffer start -- sto ncb should write the data after this signal

    // input configuration
    .i_combine_disable_flag(i_combine_disable_flag),
    .i_cb_num(i_cb_num),                // number of CB
    .i_e_bmp_fmt(i_e_bmp_fmt),          // E bitmap format
    .i_e0_size(i_e0_size),              // E0 size
    .i_e1_size(i_e1_size),              // E1 size
    .i_k0_pos(i_k0_pos),                // K0 position
    .i_ndi(i_ndi),                      // New data indicator
    .i_ncb_size(i_ncb_size),            // Ncb size
    .i_tb_harq_baddr(i_tb_harq_baddr),  // Harq Base address
    .i_harq_lb_burst_len(i_harq_lb_burst_len), // HARQ local bus interface burst length 0-16, 1-32, 2-64

    // control signal to fetch_ncb
    .o_sto0_done(sto0_done),
    .o_sto1_done(sto1_done),
 
    // control signal to sync_sto_cfg
    .o_sto_ch_done(o_sto_ncb_done),      // indicate all Store Ncb operation done signal
 
    // control signal from combine_top
    .i_harq0_done(sync_harq0_done),
    .i_harq1_done(sync_harq1_done),
    .i_harq0_ncb_done(sync_harq0_ncb_done),
    .i_harq1_ncb_done(sync_harq1_ncb_done),
    .i_harq0_e0_burst_en(sync_harq0_e0_burst_en),
    .i_harq0_e1_burst_en(sync_harq0_e1_burst_en),
    .i_harq1_e0_burst_en(sync_harq1_e0_burst_en),
    .i_harq1_e1_burst_en(sync_harq1_e1_burst_en),
    .i_harq0_combine_start(sync_harq0_combine_start),
    .i_harq1_combine_start(sync_harq1_combine_start),

    // output signal for local ncb buffer
    .o_sto_ptr(sto_ptr),
    .o_sto_ren(sto_ren),
    .o_sto_addr(sto_addr),
    .i_sto_rdata(sto_rdata),
 
    // output data local read axi bus
    .o_wr_cmd_strb(o_wr_cmd_strb),
    .i_wr_cmd_done(i_wr_cmd_done),
    .o_wr_data_number(o_wr_data_number),
    .o_wr_baddr(o_wr_baddr),
    .o_wr(o_wr),
    .o_wdata(o_wdata),
    .i_wfull(i_wfull),
    .o_wr_termi(o_wr_termi),

 
    // output data for PDSCH turbo decoder interface
    .o_harq_ncb_addr(o_harq_ncb_addr), // CB internal addresss for turbo decoder NCB buffer only
    .o_harq_ncb_wr(o_harq_ncb_wr),
    .i_pdsch_dec_ncb_rdy(i_pdsch_dec_ncb_rdy), // indicate the PDSCH decoder write NCB buffer ready

    // error control
    .o_sto_err(o_sto_err)
  
  );


  // -----------------------------------------------------
  // memory aribiter and memory instance 
  // -----------------------------------------------------
  ncb_arb NCB_ARB (
    // global signal
    .i_harq_rst_n(i_harq_rst_n),
    .i_mem_rst_n(i_mem_rst_n),
    .i_harq_clk(i_harq_free_clk),
    .i_mem_clk(i_mem_free_clk),

    // rx ctrl input control
    .i_hclk_harq_start(i_hclk_harq_start),
    .i_mclk_harq_start(i_mclk_harq_start),

    // interface with combine_top
    .i_harq_en(harq_en),
    .i_harq_wen(harq_wen),
    .i_harq_addr(harq_addr),
    .i_harq_wdata(harq_wdata),
    .o_harq_rdata(harq_rdata),
    .i_harq0_done(harq0_done),
    .i_harq1_done(harq1_done),

    // interface with fetch_ncb
    .i_fetch_ptr(fetch_ptr),
    .i_fetch_wen(fetch_wen),
    .i_fetch_addr(fetch_addr),
    .i_fetch_wdata(fetch_wdata),
    .i_fetch0_done(fetch0_done),
    .i_fetch1_done(fetch1_done),

    // interface with sto_ncb
    .i_sto_ptr(sto_ptr),
    .i_sto_ren(sto_ren),
    .i_sto_addr(sto_addr),
    .o_sto_rdata(sto_rdata),
    .i_sto0_done(sto0_done),
    .i_sto1_done(sto1_done),
 
    // interface with ncb ping-pong memory
    .o_ncb_arb0_harq_wen(ncb_arb0_harq_wen),
    .o_ncb_arb0_harq_en(ncb_arb0_harq_en),
    .o_ncb_arb0_harq_addr(ncb_arb0_harq_addr),
    .o_ncb_arb0_harq_wdata(ncb_arb0_harq_wdata),
    .i_ncb_arb0_harq_rdata(ncb_arb0_harq_rdata),
 
    .o_ncb_arb1_harq_wen(ncb_arb1_harq_wen),
    .o_ncb_arb1_harq_en(ncb_arb1_harq_en),
    .o_ncb_arb1_harq_addr(ncb_arb1_harq_addr),
    .o_ncb_arb1_harq_wdata(ncb_arb1_harq_wdata),
    .i_ncb_arb1_harq_rdata(ncb_arb1_harq_rdata),

    .o_ncb_arb0_mem_wen(ncb_arb0_mem_wen),
    .o_ncb_arb0_mem_en(ncb_arb0_mem_en),
    .o_ncb_arb0_mem_addr(ncb_arb0_mem_addr),
    .o_ncb_arb0_mem_wdata(ncb_arb0_mem_wdata),
    .i_ncb_arb0_mem_rdata(ncb_arb0_mem_rdata),
 
    .o_ncb_arb1_mem_wen(ncb_arb1_mem_wen),
    .o_ncb_arb1_mem_en(ncb_arb1_mem_en),
    .o_ncb_arb1_mem_addr(ncb_arb1_mem_addr),
    .o_ncb_arb1_mem_wdata(ncb_arb1_mem_wdata),
    .i_ncb_arb1_mem_rdata(ncb_arb1_mem_rdata)

  );

  tpram_2306x64_ByteEn_w TPRAM_2306X64_0 (

    .A_clk(i_harq_free_clk),
    .T_RWM(i_twm),
    .A_en(ncb_arb0_harq_en),
    .A_we(ncb_arb0_harq_wen),
    .A_addr(ncb_arb0_harq_addr),
    .A_wdata(ncb_arb0_harq_wdata),
    .A_rdata(ncb_arb0_harq_rdata),

    .B_clk(i_mem_free_clk),
    .B_en(ncb_arb0_mem_en),
    .B_we(ncb_arb0_mem_wen),
    .B_addr(ncb_arb0_mem_addr),
    .B_wdata(ncb_arb0_mem_wdata),
    .B_rdata(ncb_arb0_mem_rdata)

  );


  tpram_2306x64_ByteEn_w TPRAM_2306X64_1 (

    .A_clk(i_harq_free_clk),
    .T_RWM(i_twm),
    .A_en(ncb_arb1_harq_en),
    .A_we(ncb_arb1_harq_wen),
    .A_addr(ncb_arb1_harq_addr),
    .A_wdata(ncb_arb1_harq_wdata),
    .A_rdata(ncb_arb1_harq_rdata),

    .B_clk(i_mem_free_clk),
    .B_en(ncb_arb1_mem_en),
    .B_we(ncb_arb1_mem_wen),
    .B_addr(ncb_arb1_mem_addr),
    .B_wdata(ncb_arb1_mem_wdata),
    .B_rdata(ncb_arb1_mem_rdata)

  );

endmodule

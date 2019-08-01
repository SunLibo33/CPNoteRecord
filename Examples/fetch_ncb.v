// Description
// This module is for fetching memory to the local Ncb ping-pong buffer
//    
// add comment in o_fetch_ncb_done signals



module fetch_ncb (

  // global signal
  input                         i_rst_n,
  input                         i_mem_clk,

  // rx ctrl input control
  input                         i_harq_start,
  input                         i_harq_end,

  // input configuration
  input [5-1:0]                 i_cb_num,           // number of CB
  input [26-1:0]                i_e_bmp_fmt,        // E bitmap format
  input [17-1:0]                i_e0_size,          // E0 size
  input [17-1:0]                i_e1_size,          // E1 size
  input [15-1:0]                i_k0_pos,           // K0 position
  input                         i_ndi,              // New data indicator
  input [15-1:0]                i_ncb_size,         // Ncb size
  input [32-1:0]                i_tb_harq_baddr,    // Harq Base address

  // control signal to combine_top
  output reg                    o_fetch0_done,
  output reg                    o_fetch1_done,
  output reg                    o_fetch_ncb_done, // assert when all ncb finished
  
  // control signal from sto_ncb
  input                         i_sto0_done,
  input                         i_sto1_done,
  
  // output signal for local ncb buffer
  output                        o_fetch_ptr,
  output reg                    o_fetch_wen,
  output reg [12-1:0]           o_fetch_addr,
  output reg [64-1:0]           o_fetch_wdata,
  
  // output data local read axi bus
  output reg                    o_rd_cmd_strb,
  input                         i_rd_cmd_done,
  output reg [16-1:0]           o_rd_data_number,
  output reg [32-1:0]           o_rd_baddr,
  output                        o_rd,
  input [64-1:0]                i_rdata,
  input                         i_rempty,
  output                        o_rd_termi,
  
  // error control
  output reg                    o_fetch_err
  
);

  //---------------------------------------------------------------------------
  // Parameter declaration
  //---------------------------------------------------------------------------
  parameter FETCH_SWIDTH = 4;

  parameter FETCH_IDLE                  = 4'b0000;
  parameter FETCH_INI                   = 4'b0001;
  parameter FETCH_CFG0                  = 4'b0010;
  parameter FETCH_CFG1                  = 4'b0011;

  parameter FETCH_START                 = 4'b0100;
  parameter FETCH_FZ_WR_K0              = 4'b0101;
  parameter FETCH_FZ_WR_K0_PLUS_E_MOD8  = 4'b0110;
  parameter FETCH_FZ_WR_NCB_MOD8        = 4'b0111;

  parameter FETCH_WAIT_CMD_RD_NCB       = 4'b1000;
  parameter FETCH_CMD_RD_NCB            = 4'b1001;
  parameter FETCH_DATA_RD_NCB           = 4'b1010;
  parameter FETCH_WAIT_START            = 4'b1011;

  parameter FETCH_END                   = 4'b1100;
  parameter FETCH_END_ALL               = 4'b1101;
  parameter FETCH_ERR                   = 4'b1110;
  
  //---------------------------------------------------------------------------
  // Internal wire declaration
  //---------------------------------------------------------------------------
  wire [18-1:0]                 k0e;                    // k0 + E size 
  wire [15-1:0]                 e_8_tmp;                // E size for 8 byte alignment
  wire [18-1:0]                 k0e_mod_tmp;            // ( k0 + E ) mod Ncb only use for E < Ncb and size is small than Ncb
  wire [12-1:0]                 k0e_mod_8;              // ceiling (k0 + E ) mod Ncb for 8 byte alignment



  //---------------------------------------------------------------------------
  // Sequential reg declaration
  //---------------------------------------------------------------------------
  reg [FETCH_SWIDTH-1:0]        fetch_cs, fetch_ns;     // fetch ncb control state machine
  reg                           ncb_ptr;                // ncb pointer to point the Ncb ping-pong buffer
  reg                           sto0_clr;               // clear the store 0 ready signal
  reg                           sto1_clr;               // clear the store 1 ready signal

  reg [26-1:0]                  int_e_bmp_fmt;          // internal E bitmap format for choosing E1 or E2
  reg [17-1:0]                  e_size;                 // E size in current Ncb
  reg [32-1:0]                  baddr;                  // current Ncb base address1;<2n

  reg [12-1:0]                  ncb_8;                  // ceiling Ncb / 8 for 8 byte alignment
  reg [12-1:0]                  k0_8;                   // floor k0 / 8 for 8 byte alignment
  reg [14-1:0]                  e_8;                    // ceiling e/8 size for 8 byte alignment
  reg [15-1:0]                  k0e_8;                  // ceiling k0 + E /8 size for 8 byte alignment

  reg [15-1:0]                  k0e_mod;                // ( k0 + E ) mod Ncb only use for E < Ncb and size is small than Ncb

  reg [12-1:0]                  wr_ncb_buf_cnt;                 // read fetch counter 
  reg [5-1:0]                   cb_num_cnt;             // cb number counter
  
  reg                           sto0_rdy;
  reg                           sto1_rdy;
  //---------------------------------------------------------------------------
  // Instantiation of submodules 
  //---------------------------------------------------------------------------


  //---------------------------------------------------------------------------
  // Combinational logic
  //---------------------------------------------------------------------------
  assign o_rd_termi = o_fetch_err;
  assign o_rd = ~i_rempty;
  assign o_fetch_ptr = ncb_ptr;
  assign k0e = i_k0_pos + e_size;
  assign k0e_mod_tmp = k0e - i_ncb_size; // only 15 bit valid 
  assign k0e_mod_8 = (k0e > i_ncb_size) ? k0e_mod[14:3] : k0e[14:3];
  
  assign e_8_tmp = k0e_8 - k0_8; // 15 bit - 12 bit = 15 bit but possible e_8 size is only 14 bit; 
//  assign o_fetch_ncb_done = i_harq_end;

  
  always @ (*) begin
    if (i_harq_end & (fetch_cs != FETCH_IDLE)) begin
      fetch_ns = FETCH_ERR;
    end else begin
      fetch_ns = fetch_cs;
      case(fetch_cs)
        FETCH_IDLE : begin
          if (i_harq_start & (i_cb_num != 5'd0)) begin
            fetch_ns = FETCH_INI;
          end
        end
        FETCH_INI : begin
          fetch_ns = FETCH_CFG0;
        end
        FETCH_CFG0 : begin
          fetch_ns = FETCH_CFG1;
        end
        FETCH_CFG1 : begin
          fetch_ns = FETCH_START;
        end
        FETCH_START : begin
          if (i_ndi) begin
            fetch_ns = FETCH_FZ_WR_K0;          
          end else begin
            fetch_ns = FETCH_WAIT_CMD_RD_NCB;
          end
        end
        // =============================================== //
        // ndi = 1 without harq combining
        // =============================================== //
        FETCH_FZ_WR_K0 : begin
          fetch_ns = FETCH_FZ_WR_K0_PLUS_E_MOD8;
        end
        FETCH_FZ_WR_K0_PLUS_E_MOD8 : begin
          fetch_ns = FETCH_FZ_WR_NCB_MOD8;
        end
	FETCH_FZ_WR_NCB_MOD8: begin
	  fetch_ns = FETCH_END;
	end
        // =============================================== //
        // ndi = 0 with harq combining
        // =============================================== //
        FETCH_WAIT_CMD_RD_NCB : begin
          if (i_rd_cmd_done) begin
            fetch_ns = FETCH_CMD_RD_NCB;
          end
        end
        FETCH_CMD_RD_NCB : begin
          fetch_ns = FETCH_DATA_RD_NCB;            
        end
        FETCH_DATA_RD_NCB : begin
          if ((wr_ncb_buf_cnt == ncb_8 - 12'b1) & ~i_rempty) begin
            fetch_ns = FETCH_END;            
          end
        end
        FETCH_END : begin
          if (cb_num_cnt != i_cb_num - 5'b1) begin
            fetch_ns = FETCH_WAIT_START;
          end else begin
            fetch_ns = FETCH_END_ALL;            
          end
        end
        FETCH_END_ALL : begin
          fetch_ns = FETCH_IDLE;            
        end
        FETCH_WAIT_START : begin
          if ((sto0_rdy & !ncb_ptr) | (sto1_rdy & ncb_ptr)) begin
            fetch_ns = FETCH_CFG0;
          end
        end
        FETCH_ERR : begin
          fetch_ns = FETCH_IDLE;
        end
        default : begin
          fetch_ns = FETCH_IDLE;
        end
      endcase
    end
  end


  //---------------------------------------------------------------------------
  // Flip-flops
  //---------------------------------------------------------------------------
  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    fetch_cs <= FETCH_IDLE;
  end else begin
    fetch_cs <= fetch_ns;
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    sto0_rdy <= 1'b0;
  end else begin
    if (i_harq_start) begin
      sto0_rdy <= 1'b1;
    end else if (sto0_clr) begin
      sto0_rdy <= 1'b0;
    end else if (i_sto0_done) begin
      sto0_rdy <= 1'b1;
    end
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    sto1_rdy <= 1'b0;
  end else begin
    if (i_harq_start) begin
      sto1_rdy <= 1'b1;
    end else if (sto1_clr) begin
      sto1_rdy <= 1'b0;
    end else if (i_sto1_done) begin
      sto1_rdy <= 1'b1;
    end
  end

  always @ (posedge i_mem_clk or negedge i_rst_n) 
  if (!i_rst_n) begin
    ncb_ptr <= 1'b0;
    o_fetch0_done <= 1'b0;
    o_fetch1_done <= 1'b0;
    sto0_clr <= 1'b0;
    sto1_clr <= 1'b0;

    ncb_8 <= 12'b0;
    k0_8 <= 12'b0;
    e_8 <= 14'b0;
    k0e_8 <= 15'b0;
    k0e_mod <= 15'b0;
    e_size <= 17'b0;

    o_rd_cmd_strb <= 1'b0;
    o_rd_data_number <= 16'b0;
    o_rd_baddr <= 32'b0;    
    baddr <= 32'b0;    

    o_fetch_wen <= 1'b0;
    o_fetch_addr <= 12'b0;
    o_fetch_wdata <= 64'b0;
    o_fetch_err <= 1'b0;
    wr_ncb_buf_cnt <= 12'b0;
    cb_num_cnt <= 5'b0;

    int_e_bmp_fmt <= 26'b0;
    o_fetch_ncb_done <= 1'b1;

  end else begin
    case(fetch_cs)
      FETCH_IDLE : begin
        ncb_ptr <= 1'b0;
        o_fetch0_done <= 1'b0;
        o_fetch1_done <= 1'b0;
        sto0_clr <= 1'b0;
        sto1_clr <= 1'b0;

        ncb_8 <= 12'b0;
        k0_8 <= 12'b0;
        e_8 <= 14'b0;
        k0e_8 <= 15'b0;
        k0e_mod <= 15'b0;
        e_size <= 17'b0;
    
        o_rd_cmd_strb <= 1'b0;
        o_rd_data_number <= 16'b0;
        o_rd_baddr <= 32'b0;
        baddr <= 32'b0;

        o_fetch_wen <= 1'b0;
        o_fetch_addr <= 12'b0;
        o_fetch_wdata <= 64'b0;
	o_fetch_err <= 1'b0;
        wr_ncb_buf_cnt <= 12'b0;
        cb_num_cnt <= 5'b0;

        int_e_bmp_fmt <= i_e_bmp_fmt;
        o_fetch_ncb_done <= 1'b0;
      end
      FETCH_INI : begin
        e_size <= int_e_bmp_fmt[0] ? i_e1_size : i_e0_size;
        baddr <= i_tb_harq_baddr;
      end
      FETCH_CFG0 : begin
        ncb_8 <= i_ncb_size[14:3] + (|i_ncb_size[2:0]);
        k0_8 <= i_k0_pos[14:3];
        k0e_8 <= k0e[17:3] + (|k0e[2:0]);
        k0e_mod <= k0e_mod_tmp[14:0];
      end
      FETCH_CFG1 : begin
        e_8 <= e_8_tmp[13:0];
      end
      FETCH_START : begin
        sto0_clr <= !ncb_ptr;
        sto1_clr <= ncb_ptr;        
      end
      FETCH_FZ_WR_K0 : begin
        sto0_clr <= 1'b0;
        sto1_clr <= 1'b0;
        o_fetch_wen <= 1'b1;
	o_fetch_addr <= k0_8;
        o_fetch_wdata <= 64'b0;
      end
      FETCH_FZ_WR_NCB_MOD8: begin
        o_fetch_wen <= 1'b1;
	o_fetch_addr <= i_ncb_size[14:3];
	o_fetch_wdata <= 64'b0;
      end
      FETCH_FZ_WR_K0_PLUS_E_MOD8 : begin
        o_fetch_wen <= 1'b1;
        o_fetch_addr <= k0e_mod_8;
        o_fetch_wdata <= 64'b0;
      end
      FETCH_WAIT_CMD_RD_NCB : begin
        sto0_clr <= 1'b0;
        sto1_clr <= 1'b0;         
      end
      FETCH_CMD_RD_NCB : begin
        o_rd_cmd_strb <= 1'b1;
        o_rd_baddr <= baddr;
        o_rd_data_number <= {4'b0, ncb_8};
        wr_ncb_buf_cnt <= 12'b0;        
        o_fetch_addr <= 12'b0;
      end
      FETCH_DATA_RD_NCB : begin
        o_rd_cmd_strb <= 1'b0;
        o_fetch_wen <= ~i_rempty;
        if(o_fetch_wen) begin
          o_fetch_addr <= o_fetch_addr + 12'b1;
        end
	if(~i_rempty) begin
	  wr_ncb_buf_cnt <= wr_ncb_buf_cnt + 12'b1;
	end
	o_fetch_wdata <= i_rdata;
      end
      FETCH_WAIT_START : begin
        o_fetch0_done <= 1'b0;
        o_fetch1_done <= 1'b0;
        e_size <= int_e_bmp_fmt[0] ? i_e1_size : i_e0_size;
      end
      FETCH_END : begin
        o_fetch_wen <= 1'b0; 
        wr_ncb_buf_cnt <= 12'b0;
        ncb_ptr <= !ncb_ptr;
        o_fetch0_done <= !ncb_ptr;
        o_fetch1_done <= ncb_ptr;
        cb_num_cnt <= cb_num_cnt + 5'b1;
        int_e_bmp_fmt <= {1'b0, int_e_bmp_fmt[25:1]};
        baddr <= baddr + {17'b0, ncb_8, 3'b0};
      end
      FETCH_END_ALL : begin
        o_fetch0_done <= 1'b0;
        o_fetch1_done <= 1'b0;
        o_fetch_ncb_done <= 1'b1;
      end
      FETCH_ERR : begin
        o_fetch_err <= 1'b1; 
        o_fetch_ncb_done <= 1'b1;
      end
      default : begin
        ncb_ptr <= 1'b0;
        o_fetch0_done <= 1'b0;
        o_fetch1_done <= 1'b0;
        sto0_clr <= 1'b0;
        sto1_clr <= 1'b0;

        ncb_8 <= 12'b0;
        k0_8 <= 12'b0;
        e_8 <= 14'b0;
        k0e_8 <= 15'b0;
        k0e_mod <= 15'b0;
        e_size <= 17'b0;
        
        o_rd_cmd_strb <= 1'b0;
        o_rd_data_number <= 16'b0;
        o_rd_baddr <= 32'b0;
        baddr <= 32'b0;

        o_fetch_wen <= 1'b0;
        o_fetch_addr <= 12'b0;
        o_fetch_wdata <= 64'b0;
        o_fetch_err <= 1'b0;
        wr_ncb_buf_cnt <= 12'b0;
        cb_num_cnt <= 5'b0;
        int_e_bmp_fmt <= 26'b0;
        o_fetch_ncb_done <= 1'b0;
      end
    endcase
  end




endmodule

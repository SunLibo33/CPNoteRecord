module UPDSLOWPHYTOLLR
//#(parameter DATA_WIDTH=48, parameter ADDR_WIDTH=11)
(
  input  wire         i_rx_rstn, 
  input  wire         i_rx_fsm_rstn, 			 
  input  wire         i_core_clk, 
  input  wire [15:0]  i_user_iq_noise_rate,
  input  wire [15:0]  i_cur_user_re_amounts,
  
  input wire  [127:0] Noise_Data_SUM,
  input wire  [127:0] IQ_Data_SUM,
  input wire          IQ_FIFO_Empty,
  input wire          Noise_FIFO_Empty,
  
  output wire         IQ_FIFO_Read_Enable,
  output wire         Noise_FIFO_Read_Enable,
  
  
 
  output reg          o_data_strobe,
  output reg  [15:0]  o_re0_data_i, 
  output reg  [15:0]  o_re0_data_q, 
  output reg  [15:0]  o_re1_data_i, 
  output reg  [15:0]  o_re1_data_q, 
  output reg  [15:0]  o_noise_data
  
);


wire data_permit_send;

reg [15:0] LoopCycleCounter=16'd0;
reg [15:0] SendIQCycleCounter=16'd0;
reg [15:0] SendNoiseCycleCounter=16'd0;
reg [15:0] SendNoiseCycleCounterPre=16'd0;
reg [15:0] LoopNoiseInnerCycleCounter=16'd0;

reg [15:0]RE_Counter=16'd0;

parameter IDLE        = 8'b0000_0001;
parameter USERSTART   = 8'b0000_0010;
parameter WAIT        = 8'b0000_0100;
parameter USERSEND    = 8'b0000_1000;
parameter USERCOMP    = 8'b0001_0000;


reg [7:0]Current_State = IDLE;
reg [7:0]Next_State    = IDLE;

assign data_permit_send       = ( (IQ_FIFO_Empty != 1'b1) && (Noise_FIFO_Empty != 1'b1) );
assign IQ_FIFO_Read_Enable    = ( ( LoopCycleCounter[0] ==  1'b0 )&& (Current_State==USERSEND) && (IQ_FIFO_Empty!=1'b1) );
assign Noise_FIFO_Read_Enable = ( ( LoopCycleCounter    == 16'd0 )&& (Current_State==USERSEND) && (Noise_FIFO_Empty!=1'b1) );

always @(posedge i_core_clk or negedge i_rx_rstn or negedge i_rx_fsm_rstn)
begin
  if((i_rx_rstn==1'b0)||(i_rx_fsm_rstn==1'b0))
    begin
	  Current_State<=IDLE;
	end
  else
    begin
	  Current_State<=Next_State;
	end
end

 
always @(*)
begin
  if((i_rx_rstn==1'b0)||(i_rx_fsm_rstn==1'b0))
    begin
	  Next_State=IDLE;
	end
  else
    begin
	  case(Current_State)
	    IDLE: Next_State=USERSTART;
	    USERSTART: Next_State=WAIT;
	    WAIT:
		  begin
		    if(data_permit_send==1'b1)
			  Next_State=USERSEND;
			else
			  Next_State=WAIT;
		  end
	    USERSEND:
		  begin
            if(RE_Counter>=i_cur_user_re_amounts)
              Next_State=USERCOMP;
		    else if(LoopCycleCounter >= ( (i_user_iq_noise_rate<<2)-16'd1 ) )
			  begin
                if(Noise_FIFO_Empty==1'b1)
                  Next_State=WAIT;
                else if(IQ_FIFO_Empty==1'b1)
                  Next_State=WAIT;
                else 
                  Next_State=USERSEND;
              end
			else if(IQ_FIFO_Empty==1'b1)
			  begin
                Next_State=WAIT;
              end
		  end
        USERCOMP: Next_State=IDLE;

        default: Next_State=IDLE;
      endcase		
	end
end

always @(posedge i_core_clk or negedge i_rx_rstn or negedge i_rx_fsm_rstn)
begin
  if((i_rx_rstn==1'b0)||(i_rx_fsm_rstn==1'b0))
    begin
	  RE_Counter<=16'd0;
	end
  else
    begin
      if(Current_State==USERSTART)
        RE_Counter<=16'd0;
      else if(Current_State==USERSEND)
        begin
          if(IQ_FIFO_Read_Enable)
            RE_Counter<=RE_Counter+16'd4;  
        end
	end
end

always @(posedge i_core_clk or negedge i_rx_rstn or negedge i_rx_fsm_rstn)
begin
  if((i_rx_rstn==1'b0)||(i_rx_fsm_rstn==1'b0))
    begin
	  o_data_strobe<=1'b0;
	end
  else
    begin
      if(Current_State==USERSEND)
        o_data_strobe<=1'b1;//data_permit_send;
      else
        o_data_strobe<=1'b0; 
	end
end

always @(posedge i_core_clk or negedge i_rx_rstn or negedge i_rx_fsm_rstn)
begin
  if((i_rx_rstn==1'b0)||(i_rx_fsm_rstn==1'b0))
    begin
	  LoopCycleCounter<=16'd0;
	end
  else
    begin
      //if((Current_State==USERSEND)&&(data_permit_send==1'b1))
      if(Current_State==USERSTART)
        LoopCycleCounter<=16'd0;
      else if(Current_State==USERSEND)
        begin
          if(LoopCycleCounter >= ( (i_user_iq_noise_rate<<2)-16'd1 ) )
            LoopCycleCounter<=16'd0;
          else
            LoopCycleCounter<=LoopCycleCounter+16'd1;
        end
	end
end

always @(posedge i_core_clk or negedge i_rx_rstn or negedge i_rx_fsm_rstn)
begin
  if((i_rx_rstn==1'b0)||(i_rx_fsm_rstn==1'b0))
    begin
	  SendIQCycleCounter<=16'd0;
	end
  else
    begin
      SendIQCycleCounter<=LoopCycleCounter;
	end
end

always @(posedge i_core_clk or negedge i_rx_rstn or negedge i_rx_fsm_rstn)
begin
  if((i_rx_rstn==1'b0)||(i_rx_fsm_rstn==1'b0))
    begin
	  LoopNoiseInnerCycleCounter<=16'd0;
      SendNoiseCycleCounterPre<=16'd0;
	end
  else
    begin
      //if((Current_State==USERSEND)&&(data_permit_send==1'b1))
      if(Current_State==USERSTART)
        begin
	      LoopNoiseInnerCycleCounter<=16'd0;
          SendNoiseCycleCounterPre<=16'd0;
        end
      else if(Current_State==USERSEND)
        begin
          if(LoopNoiseInnerCycleCounter >= ( (i_user_iq_noise_rate>>1)-16'd1 ) )
            begin
              LoopNoiseInnerCycleCounter<=16'd0;
              if(SendNoiseCycleCounterPre>=16'd7)
                SendNoiseCycleCounterPre<=16'd0;
              else
                SendNoiseCycleCounterPre<=SendNoiseCycleCounterPre+16'd1;
            end
          else
            begin
              LoopNoiseInnerCycleCounter<=LoopNoiseInnerCycleCounter+16'd1;
            end
        end
	end
end


always @(posedge i_core_clk or negedge i_rx_rstn or negedge i_rx_fsm_rstn)
begin
  if((i_rx_rstn==1'b0)||(i_rx_fsm_rstn==1'b0))
    begin
      SendNoiseCycleCounter<=16'd0;
	end
  else
    begin
      SendNoiseCycleCounter<=SendNoiseCycleCounterPre;
	end
end


always @(*)
begin
  case(SendIQCycleCounter[0])
    1'b0: o_re0_data_i = IQ_Data_SUM[15:0];
    1'b1: o_re0_data_i = IQ_Data_SUM[79:64];
    default: o_re0_data_i=16'd0;
  endcase
end

always @(*)
begin
  case(SendIQCycleCounter[0])
    1'b0: o_re0_data_q = IQ_Data_SUM[31:16];
    1'b1: o_re0_data_q = IQ_Data_SUM[95:80];
    default: o_re0_data_i=16'd0;
  endcase
end

always @(*)
begin
  case(SendIQCycleCounter[0])
    1'b0: o_re1_data_i = IQ_Data_SUM[47:32];
    1'b1: o_re1_data_i = IQ_Data_SUM[111:96];
    default: o_re1_data_i=16'd0;
  endcase
end

always @(*)
begin
  case(SendIQCycleCounter[0])
    1'b0: o_re1_data_q = IQ_Data_SUM[63:48];
    1'b1: o_re1_data_q = IQ_Data_SUM[127:112];
    default: o_re1_data_i=16'd0;
  endcase
end

always @(*)
begin
  case(SendNoiseCycleCounter)
    16'd0: o_noise_data = Noise_Data_SUM[15:0];
    16'd1: o_noise_data = Noise_Data_SUM[31:16];
    16'd2: o_noise_data = Noise_Data_SUM[47:32];
    16'd3: o_noise_data = Noise_Data_SUM[63:48];
    16'd4: o_noise_data = Noise_Data_SUM[79:64];
    16'd5: o_noise_data = Noise_Data_SUM[95:80];
    16'd6: o_noise_data = Noise_Data_SUM[111:96];
    16'd7: o_noise_data = Noise_Data_SUM[127:112];
    default: o_noise_data=16'd0;
  endcase
end


endmodule




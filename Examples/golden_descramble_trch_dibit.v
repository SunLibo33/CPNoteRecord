//function  : golden sequence descramble for two data at a time
****************************************************************************/
// $Log: golden_descramble_trch.v,v $
// modify from golden_descramble_trch.v, accept, descramble and output 2 data per clk
//
// Rename i_clk to i_clk_2x to indicate its working frequency
module golden_descramble_trch_dibit(
        i_clk_2x,                  //user module clock
        i_rstn,                 //user module async reset, low is active

        i_start,                //input seq start, initial seq, high is active
        i_data,                 //input data, 0th,2th,4th,...
        i_data2,                //input data, 1th,3th,5th,...
        i_strb,                 //input data strb
        i_x1_init,              //input X1 sequence initial value
        i_x2_init,              //input X2 sequence initial value

        o_data,                 //output data, 0th,2th,4th,...
        o_data2,                //output data, 1th,3th,5th,...
        o_strb                  //output data strobe, high is active

        );
//-------------------------------------------------------------------
// parameter define
//-------------------------------------------------------------------
parameter IN_DATA_WIDTH = 8;
parameter OUT_DATA_WIDTH = 8;
//-------------------------------------------------------------------
// port define
//-------------------------------------------------------------------
input i_clk_2x;                    //user module clock
input i_rstn;                   //user module async reset, low is active


input i_strb;                   //input data strobe, high is active
input signed [IN_DATA_WIDTH-1:0] i_data;        //input data
input signed [IN_DATA_WIDTH-1:0] i_data2;        //input data
input i_start;                  //input symbol start, high is active
input [30:0] i_x1_init; //input x1 initial value
input [30:0] i_x2_init; //input x2 initial value

output o_strb;                  //output data strobe, high is active
output signed [OUT_DATA_WIDTH-1 : 0] o_data;                    //output data
output signed [OUT_DATA_WIDTH-1 : 0] o_data2;                    //output data
//-------------------------------------------------------------------
// wire define
//-------------------------------------------------------------------
wire [30:0] x1_nxt;
wire [30:0] x1_nxt2;
wire [30:0] x2_nxt;
wire [30:0] x2_nxt2;
wire [30:0] golden_seq;
wire [30:0] golden_seq2;
wire scramable_flag;
wire scramable_flag2;
wire signed [IN_DATA_WIDTH : 0] tmp_i_data;  // extra bit for handling overflow
wire signed [IN_DATA_WIDTH - 1 : 0] neg_i_data;  // 2's complement of i_data
wire signed [IN_DATA_WIDTH : 0] tmp_i_data2;  // extra bit for handling overflow
wire signed [IN_DATA_WIDTH - 1 : 0] neg_i_data2;  // 2's complement of i_data
//-------------------------------------------------------------------
// reg define
//-------------------------------------------------------------------
reg [30:0] x1;
reg [30:0] x2;
reg signed [OUT_DATA_WIDTH-1 : 0] o_data;
reg signed [OUT_DATA_WIDTH-1 : 0] o_data2;
reg o_strb;
//-------------------------------------------------------------------
// output logic
//-------------------------------------------------------------------
assign scramable_flag  = golden_seq[0];   //0,2,4,...
assign scramable_flag2 = golden_seq2[0];  //1,3,5,...


always @(posedge i_clk_2x or negedge i_rstn)
        begin
        if(~i_rstn)
                begin
                o_strb <= # 1 1'b0;
                end
        else
                begin
                o_strb <= # 1 i_strb;
                end
        end

// 2's complement of i_data 
// use saturation arithematic to handle overflow
assign tmp_i_data = ~{i_data[IN_DATA_WIDTH - 1], i_data} + 1;
assign neg_i_data = (tmp_i_data[IN_DATA_WIDTH] != tmp_i_data[IN_DATA_WIDTH - 1]) ?
                    8'b0111_1111 :                     // overflow => saturated at +127
                    tmp_i_data[IN_DATA_WIDTH - 1 : 0]; // no overflow

assign tmp_i_data2 = ~{i_data2[IN_DATA_WIDTH - 1], i_data2} + 1;
assign neg_i_data2 = (tmp_i_data2[IN_DATA_WIDTH] != tmp_i_data2[IN_DATA_WIDTH - 1]) ?
                    8'b0111_1111 :                     // overflow => saturated at +127
                    tmp_i_data2[IN_DATA_WIDTH - 1 : 0]; // no overflow


always @(posedge i_clk_2x or negedge i_rstn)
        begin
        if(~i_rstn)
                begin
                o_data <= # 1 8'h0;
                end
        else
                begin

                if(i_strb)
                        begin
                      //if(~scramable_flag)
                        if( scramable_flag)
                                begin
                                o_data <= # 1 neg_i_data;
                                end
                        else
                                begin
                                o_data <= # 1 i_data;
                                end
                        end
                end
        end

always @(posedge i_clk_2x or negedge i_rstn)
        begin
        if(~i_rstn)
                begin
                o_data2 <= # 1 8'h0;
                end
        else
                begin

                if(i_strb)
                        begin
                      //if(~scramable_flag2)
                        if( scramable_flag2)
                                begin
                                o_data2 <= # 1 neg_i_data2;
                                end
                        else
                                begin
                                o_data2 <= # 1 i_data2;
                                end
                        end
                end
        end
//-------------------------------------------------------------------
// golden sequence gen logic
//-------------------------------------------------------------------
assign x1_nxt = {(x1[3]^x1[0]),x1[30:1]};
assign x1_nxt2 = {(x1_nxt[3]^x1_nxt[0]),x1_nxt[30:1]};

assign x2_nxt = {(x2[3]^x2[2]^x2[1]^x2[0]),x2[30:1]};
assign x2_nxt2 = {(x2_nxt[3]^x2_nxt[2]^x2_nxt[1]^x2_nxt[0]),x2_nxt[30:1]};

assign golden_seq = x1^x2;
assign golden_seq2 = x1_nxt^x2_nxt;



always @(posedge i_clk_2x or negedge i_rstn)
        begin
        if(~i_rstn)
                begin
                x1 <= # 1 31'h00000000;
                end
        else
                begin
                if(i_start)
                        begin
                        x1 <= # 1 i_x1_init;
                        end
                else if(i_strb)
                        begin
                        x1 <= # 1 x1_nxt2;   //0,2,4,...
                        end
                end
        end

always @(posedge i_clk_2x or negedge i_rstn)
        begin
        if(~i_rstn)
                begin
                x2 <= # 1 31'h00000000;
                end
        else
                begin
                if(i_start)
                        begin
                        x2 <= # 1 i_x2_init;
                        end
                else if(i_strb)
                        begin
                        x2 <= # 1 x2_nxt2;
                        end
                end
        end

endmodule

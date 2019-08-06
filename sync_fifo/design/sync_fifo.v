////////////////////////////////////////////////////////////////////////////////
// Company: <Casa Systems>
// Engineer: <Libo Sun>
//
// Create Date: <2019 Aug 01>
// Design Name: <SYNC FIFO>
// Description: Data with and ADDR_WIDTH could be re definition
// Version    : V0.1
////////////////////////////////////////////////////////////////////////////////
				
module sync_fifo
#(parameter DATA_WIDTH=8, parameter ADDR_WIDTH=3)
(
    input                          clk,  	
    input                          rst_n,
    input                          wr,
    input      [(DATA_WIDTH-1):0]  datain,
    
    input                          rd, 
    output reg [(DATA_WIDTH-1):0]  dataout, 
    output                         full, 
    output                         empty    
);
	

reg                    full_in, empty_in;

reg [(DATA_WIDTH-1):0] mem [2**ADDR_WIDTH-1:0];  //Dual Port SRAM

reg [(ADDR_WIDTH-1):0] rp, wp;


assign full = full_in;
assign empty = empty_in;

// memory read out
//assign dataout = mem[rp];
always@(posedge clk or negedge rst_n) 
begin
    if(!rst_n) 
      dataout <= {DATA_WIDTH{1'b0}};
    else 
      dataout <= mem[rp];
end

// memory write in
always@(posedge clk) 
begin
    if(wr && ~full_in) 
      mem[wp] <= datain;
end

// memory write pointer increment
always@(posedge clk or negedge rst_n) 
begin
    if(!rst_n) 
      wp<=0;
    else 
      begin
        if(wr && ~full_in) 
          wp <= wp + 1'b1;
      end
end

// memory read pointer increment
always@(posedge clk or negedge rst_n)
begin
    if(!rst_n) 
      rp <= 0;
    else 
      begin
        if(rd && ~empty_in) 
          rp <= rp + 1'b1;
      end
end

// Full signal generate
always@(posedge clk or negedge rst_n) 
begin
    if(!rst_n) 
      full_in <= 1'b0;
    else 
      begin
        if( (~rd && wr) && ( (wp==rp-1) || ( (rp==({ADDR_WIDTH{1'b0}})) && (wp==({ADDR_WIDTH{1'b1}})) ) ) )
          full_in <= 1'b1;
        else if(full_in && rd) 
          full_in <= 1'b0;
      end
end

// Empty signal generate
always@(posedge clk or negedge rst_n) 
begin
    if(!rst_n) 
      empty_in <= 1'b1;
    else 
      begin
        if( (rd && ~wr) && ( (rp==wp-1) || ( (rp==({ADDR_WIDTH{1'b1}})) && (wp==({ADDR_WIDTH{1'b0}})) ) ) )
          empty_in<=1'b1;
        else if(empty_in && wr) 
          empty_in<=1'b0;
      end
end

endmodule

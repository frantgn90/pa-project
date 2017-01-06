`ifndef _regfile
`define _regfile

`include "../../define.v"

module regfile(
	input wire clk,
	input wire [`REG_ADDR-1:0] rreg1, rreg2,
	output reg [`REG_SIZE-1:0] rdata1 = {`REG_SIZE{1'b0}},
	output reg [`REG_SIZE-1:0] rdata2 = {`REG_SIZE{1'b0}},
	input wire regwrite,
	input wire [`REG_ADDR-1:0] wreg,
	input wire [`REG_SIZE-1:0] wdata
);

integer i;

reg [`REG_SIZE-1:0] mem [`REG_N-1:0];

// Reading first register asynch
//
always @* 
begin
	rdata1 <= mem[rreg1]; //[`REG_SIZE-1:0];
end

// Reading second register asynch
//
always @* 
begin
	rdata2 <= mem[rreg2]; //[`REG_SIZE-1:0];
end

// Writing dst register synchronously
//
always @(posedge clk) 
begin
	if (regwrite == 1) 
	begin
		mem[wreg] <= wdata;
	end
end

endmodule

`endif
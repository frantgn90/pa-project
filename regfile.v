`ifndef _regfile
`define _regfile

`include "define.v"

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

// Reading asynch
//
always @* begin

    // Source 1
    if (rreg1 == 0) begin 
        rdata1 <= {`REG_SIZE{1'b0}};
    end
    else if (rreg1 == wreg && regwrite) begin
        rdata1 <= wdata;
    end 
    else
        rdata1 <= mem[rreg1][`REG_SIZE-1:0];

    // Source 2
    if (rreg2 == 0) begin
        rdata2 <= {`REG_SIZE{1'b0}};
    end
    else if (rreg2 == wreg && regwrite) begin
        rdata2 <= wdata;
    end 
    else
        rdata2 <= mem[rreg2][`REG_SIZE-1:0];
end


// Writing dst register synchronously
//
always @(posedge clk) 
begin
	if (regwrite == 1 && wreg != 0) 
	begin
		mem[wreg] <= wdata;
	end
end

endmodule

`endif
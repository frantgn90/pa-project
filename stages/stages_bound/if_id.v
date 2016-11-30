`include "../../define.v"

module if_id(
	clk,		// Clock signal
	stall, 		// Stall signal, when it is true it means that the processor is stalled
	pc, 		// Program Counter from FETCH
	instruction,	// Instruction from FETCH 
	qpc, 		// Program Counter to DECODE
	qinstruction	// Instruction to DECODE
);
	// Input port declaration
	input clk;
	input stall;
	input [15:0] pc;
	input [15:0] instruction;

	// Output port declaration
	output [15:0] qpc;
	output [15:0] qinstruction;

	// Internal variables
	reg [15:0] qpc;
	reg [15:0] qinstruction;

	// Behavior
	always @(posedge clk) 
	begin
		if (!stall) 
		begin
			qpc <= pc;
			qinstruction = instruction;
		end
		else 
		begin
			qpc = 16'b0;
			qinstruction = 16'b0;
		end
	end
endmodule
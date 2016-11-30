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
	input wire clk;
	input wire stall;
	input wire [`ADDR_SIZE-1:0] pc;
	input wire [`INSTR_SIZE-1:0] instruction;

	// Output port declaration
	output reg [`ADDR_SIZE-1:0] qpc;
	output reg [`INSTR_SIZE-1:0] qinstruction;

	// Internal variables
	reg [`ADDR_SIZE-1:0] reg_qpc;
	reg [`INSTR_SIZE-1:0] reg_qinstruction;

	// Net declaration

	always @* 
	begin
		qpc <= reg_qpc;
		qinstruction <= reg_qinstruction;
	end


	// Behavior
	always @(posedge clk) 
	begin
		if (!stall) 
		begin
			reg_qpc <= pc;
			reg_qinstruction <= instruction;
		end
		else 
		begin
			reg_qpc <= {`ADDR_SIZE{1'b0}};
			reg_qinstruction <= {`INSTR_SIZE{1'b0}};
		end
	end
endmodule
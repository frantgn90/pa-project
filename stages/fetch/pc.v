`ifndef _pc
`define _pc

`include "define.v"

module pc(
	input wire clk,
	
	input wire is_jump,
	input wire is_branch,
	input wire is_exception,
	input wire reset,
	
	input wire [31:0] pc_jump,
	input wire [31:0] pc_branch,
	input wire [31:0] old_pc,
	output reg [31:0] new_pc,
	input wire pc_write
	);
	
wire [31:0] pc;

assign pc = is_jump?		 	pc_jump :
				is_branch?	 	pc_branch:
				is_exception? `EXCEPTION_PC: old_pc+4;

always @(posedge clk) begin
	if (reset) new_pc <= `INITIAL_PC;
	else if (pc_write) new_pc <= pc;
end

endmodule

`endif
	

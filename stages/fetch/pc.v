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
	
wire pc;

assign pc = is_jump&pc_write?		 	pc_jump :
				is_branch&pc_write?	 	pc_branch:
				is_exception&pc_write? `EXCEPTION_PC:
				pc_write? old_pc+4: old_pc;

always @(posedge clk) begin
	if (reset) new_pc <= `INITIAL_PC;
	else new_pc <= pc;
end

endmodule

`endif
	

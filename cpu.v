`ifndef _cpu
`define _cpu

`include "define.v"
`include "pc.v"
`include "alu.v"



module cpu(
	input wire clk,
	input wire reset
	// Memory ports
	/*output wire mem_enable,
	output wire mem_rw,
	input wire mem_ack,
	output wire [31:0]  mem_addr,
	input wire [BWDITH-1:0] mem_data_out,
	output wire [BWDITH-1:0] mem_data_in*/
);


/*module pc(
	input wire clk,
	
	input wire is_jump,
	input wire is_branch,
	input wire is_exception,
	input wire reset,
	
	input wire [31:0] pc_jump,
	input wire [31:0] pc_branch,
	input wire [31:0] old_pc,
	output reg [31:0] new_pc
	);
*/

pc pc(
	.clk(clk),
	.is_jump(),
	.is_branch(),
	.is_exception(),
	.reset(reset),
	.pc_jump(),
	.pc_branch(),
	.old_pc(),
	.new_pc()
);

endmodule
`endif
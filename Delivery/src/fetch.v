`ifndef _fetch
 `define _fetch

 `include "define.v"

module fetch(
	           input wire                   clk,
	           input wire                   is_jump,
	           input wire                   is_branch,
	           input wire                   is_exception,
	           input wire                   reset,
	           input wire [`ADDR_SIZE-1:0]  pc_jump,
	           input wire [`ADDR_SIZE-1:0]  pc_branch,
	           output wire [`ADDR_SIZE-1:0] pc_out,
	           input wire                   pc_write
	           );

	 pc pc(
	       .clk(clk),
	       .is_jump(is_jump),
	       .is_branch(is_branch),
	       .is_exception(is_exception),
	       .reset(reset),

	       .pc_jump(pc_jump),
	       .pc_branch(pc_branch),
	       .pc_write(pc_write),
         .out_pc(pc_out)
	       );
endmodule
`endif

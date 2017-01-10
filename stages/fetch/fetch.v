`ifndef _fetch
 `define _fetch

 `include "define.v"
 `include "cache/cache.v"

module fetch(
	           input wire                   clk,
	           input wire                   is_jump,
	           input wire                   is_branch,
	           input wire                   is_exception,
	           input wire                   reset,
	           input wire [`ADDR_SIZE-1:0]  pc_jump,
	           input wire [`ADDR_SIZE-1:0]  pc_branch,
	           input wire [`ADDR_SIZE-1:0]  old_pc,
	           output wire [`ADDR_SIZE-1:0] new_pc,
	           input wire       			pc_write,
        	   input wire 					if_id_write
	           );

	 pc pc(
	       .clk(clk),
	       .is_jump(is_jump),
	       .is_branch(is_branch),
	       .is_exception(is_exception),
	       .reset(reset),

	       .pc_jump(pc_jump),
	       .pc_branch(pc_branch),
	       .old_pc(old_pc),
	       .new_pc(new_pc),
	       .pc_write(pc_write),
	       .if_id_write(if_id_write)
	       );
endmodule
`endif

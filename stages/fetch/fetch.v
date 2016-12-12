`ifndef _fetch
`define _fetch

`include "define.v"
`include "cache/cache.v"

module fetch(
	input wire clk,
	
	input wire is_jump,
	input wire is_branch,
	input wire is_exception,
	input wire reset,
	
	input wire [`ADDR_SIZE-1:0] pc_jump,
	input wire [`ADDR_SIZE-1:0] pc_branch,
	input wire [`ADDR_SIZE-1:0] old_pc,
	output reg [`ADDR_SIZE-1:0] new_pc,
	output reg [`ADDR_SIZE-1:0] instruction
	);
	
	wire ic_read_req;
	wire [`ADDR_SIZE-1:0]ic_read_addr;
	wire [`MEMORY_WIDTH-1:0]ic_read_data;
	wire ic_read_ack;

	
	pc pc(
	.clk(clk),
	.is_jump(is_jump),
	.is_branch(is_branch),
	.is_exception(is_exception),
	.reset(reset),
	
	.pc_jump(pc_jump),
	.pc_branch(pc_branch),
	.old_pc(old_pc),
	.new_pc(new_pc)
	);
	
	cache #(
		.ALIAS("icache")
	) icache(
	.clk(clk),
	.reset(reset),
	.addr(old_pc),
	.data_out(instruction),
	.data_in(`ADDR_SIZE'h0000),//We only read
	.do_read(1'b1),
	.do_write(1'b0),
	.is_byte(1'b0),
	.hit(instc_hit),
	
	//Memory ports
	.mem_read_req(ic_read_req),
	.mem_read_addr(ic_read_addr),
	.mem_read_data(ic_read_data),
	.mem_read_ack(ic_read_ack)
);
endmodule
`endif
	
	
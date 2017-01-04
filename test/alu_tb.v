`include "../stages/exec/alu.v"

// ALU Testbench
module alu_tb;

wire zero;
wire overflow;
wire [31:0] out;
alu alu(
	.aluop(7'h00),
	.src1(32'd100),
	.src2(32'd130),
	.overflow(overflow),
	.zero(zero),
	.out(out));

initial begin
	`ifdef TRACEFILE
	$dumpfile(`TRACEFILE);
	$dumpvars(0, alu_tb);
	`endif

	$monitor("%d + %d = %d", alu.src1, alu.src2, out);
end

endmodule
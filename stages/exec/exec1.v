`ifndef _exec1
`define _exec1

`include "define.v"
`include "alu.v"

module exec1(
   input wire                  alusrc,
   input wire [4:0]            aluop,
   input wire [`REG_SIZE-1:0]  src1,
   input wire [`REG_SIZE-1:0]  reg2,
   input wire [`REG_SIZE-1:0]  immediat,
   input wire [`ADDR_SIZE-1:0] old_pc,
   output reg [`REG_SIZE-1:0]  aluresult,
   output reg                  zero = 1'd0,
   output reg                  overflow = 1'd0,
   output reg [`ADDR_SIZE-1:0] new_pc = 32'h0000,
   output reg [4:0]            dst
             );

   wire src2 [`REG_SIZE-1:0];

   assign src2 = alusrc ? reg2 : immediat;
   assign new_pc = old_pc + (immediat << 2);

alu alu(
	.aluop(aluop),
	.src1(src1),
	.src2(src2),
	.zero(zero),
	.overflow(overflow),
	.out(aluresult));
endmodule // exec1
`endif

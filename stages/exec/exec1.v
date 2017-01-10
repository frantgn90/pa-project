`ifndef _exec1
`define _exec1

`include "../../define.v"
`include "alu.v"

module exec1(
             input wire                  clk,
             input wire                  regwrite_in, //Write Permission
             input wire                  alusrc, // If 1 take reg2 otherwise immediat as second operand
             input wire [7:0]            aluop, //Alu operation code
             input wire [`REG_SIZE-1:0]  src1, //Register1
             input wire [`REG_SIZE-1:0]  reg2, //Register2
             input wire [`REG_SIZE-1:0]  immediat, //Immediat
             input wire [`ADDR_SIZE-1:0] old_pc, //Old_pc
             input wire [`REG_ADDR-1:0]  wreg_in, //Destination Register

             output reg                  regwrite_out, //Write Permission
             output reg                  zero = 1'd0, //Alu zero
             output reg [`REG_SIZE-1:0]  data_store,
             output reg                  overflow = 1'd0, //Alu oveflow
             output reg [`REG_SIZE-1:0]  alu_result, //Alu result
             output reg [`ADDR_SIZE-1:0] pc_branch = 32'h0000, //New PC when branch
             output reg [`REG_ADDR-1:0]  dst_reg //Destination Register
              );
    // Internal wires
    wire [`REG_SIZE-1:0] src2;
    wire alu_zero;
    wire alu_overflow;
    wire [`REG_SIZE-1:0] aluresult;

   //assign just used on combinational parts, on wires
   assign src2 = alusrc ? reg2 : immediat;

	 always @(posedge clk) begin
      data_store <= reg2;
		  pc_branch <= old_pc + (immediat << 2);
		  zero <= alu_zero;
		  overflow <= alu_overflow;
		  alu_result <= aluresult;
      dst_reg <= wreg_in;
      regwrite_out <= regwrite_in;
	 end

alu alu(
 	.aluop(aluop),
 	.src1(src1),
 	.src2(src2),
 	.zero(alu_zero),
 	.overflow(alu_overflow),
 	.out(aluresult)
	);
 endmodule
 `endif

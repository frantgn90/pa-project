`ifndef _exec1
`define _exec1

`include "define.v"
`include "alu.v"

module exec1(
             input wire                  clk,
             input wire                  regwrite_in, //Write Permission
             input wire                  alusrc, // If 1 take reg2 otherwise immediat as second operand
             input wire [5:0]            opcode, //operation code
             input wire [5:0]            funct_code, //functional code
             input wire [`REG_SIZE-1:0]  src1, //Register1
             input wire [`REG_SIZE-1:0]  reg2, //Register2
             input wire [`REG_SIZE-1:0]  immediat, //Immediat
             input wire [`ADDR_SIZE-1:0] old_pc, //Old_pc
             input wire [`REG_ADDR-1:0]  dst_reg_in, //Destination Register
             input wire                  do_read,//memory read permission
             input wire                  memtoreg,//take data from dcache or exec1

             output reg                  regwrite_out, //Write Permission
             output reg                  zero = 1'd0, //Alu zero
             output reg [`REG_SIZE-1:0]  data_store,
             output reg                  overflow = 1'd0, //Alu oveflow
             output reg [`REG_SIZE-1:0]  alu_result, //Alu result
             output reg [`ADDR_SIZE-1:0] pc_branch = 32'h0000, //New PC when branch,
             output reg                  do_read_out,
             output reg                  memtoreg_out,
             output reg [`REG_ADDR-1:0]  dst_reg //Destination Register
              );
    // Internal wires
   wire [`REG_SIZE-1:0]                  src2;
   wire                                  alu_zero;
   wire                                  alu_overflow;
   wire [`REG_SIZE-1:0]                  aluresult;
   wire [4:0]                            aluop;

   //assign just used on combinational parts, on wires
   assign src2 = alusrc ? reg2 : immediat;

	 always @(posedge clk) begin
      do_read_out <= do_read;
      memtoreg_out <= memtoreg;
      data_store <= reg2;
		  pc_branch <= old_pc + (immediat << 2);
		  zero <= alu_zero;
		  overflow <= alu_overflow;
		  alu_result <= aluresult;
      dst_reg <= dst_reg_in;
      regwrite_out <= regwrite_in;
	 end
   alucontrol alucontrol(
                         .funct(funct_code),
                         .opcode(opcode),
                         .aluop_out(aluop)
                         );
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

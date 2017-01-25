`ifndef _alucontrol
`define _alucontrol

`include "define.v"

module alucontrol(
	input wire [5:0] funct,
	input wire [5:0] opcode,
	output reg [4:0] aluop_out = 0
);

always @* begin
		casex (opcode)
			`OP_LDB:       aluop_out <= `ALUOP_ADD;
			`OP_LDW:       aluop_out <= `ALUOP_ADD;
			`OP_STB:       aluop_out <= `ALUOP_ADD;
			`OP_LI:        aluop_out <= `ALUOP_ADD;
			`OP_ADDI:      aluop_out <= `ALUOP_ADD;
      `OP_BEQ:       aluop_out <= `ALUOP_SUB;
			`OP_STW:       aluop_out <= `ALUOP_ADD;
      `OP_ORI:       aluop_out <= `ALUOP_ORI;
      `OP_LUI:       aluop_out <= `ALUOP_LUI;
			`OP_MOV:       aluop_out <= `ALUOP_MOV;
			`OP_JUMP:      aluop_out <= `ALUOP_JUMP;
		  `OP_TLBWRITE:  aluop_out <= aluop_out;
			`OP_IRET:      aluop_out <= aluop_out;
			`OP_RTYPE:
//				`WARN(("ALU Control: Unexpected OP_RTYPE"))
//			default:
	//			`WARN(("ALU Control: Unknown opcode signal %x", opcode))
	//	endcase
		    casex (funct)
			    `FN_ADD: aluop_out <= `ALUOP_ADD;
			    `FN_MUL: aluop_out <= `ALUOP_MUL;
			    `FN_SUB: aluop_out <= `ALUOP_SUB;
          `FN_SLL: aluop_out <= `ALUOP_SLL;
			    default:
;
				   // `WARN(("ALU Control: Unknown funct signal %x", funct))
		    endcase // casex (funct)
      default:
        ;
    endcase
end

endmodule

`endif

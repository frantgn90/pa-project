`ifndef _alucontrol
`define _alucontrol

`include "define.v"

module alucontrol(
	input wire [7:0] opcode,
	output reg [7:0] aluop_out = 0
	);
	
	always @* begin
		case (opcode)
			`OPCODE_ADD: aluop_out <= `ALUOP_ADD;
			`OPCODE_SUB: aluop_out <= `ALUOP_SUB;
			`OPCODE_MUL: aluop_out <= `ALUOP_MUL;
			`OPCODE_LDB: aluop_out <= `ALUOP_LDB;
			`OPCODE_LDW: aluop_out <= `ALUOP_LDW;
			`OPCODE_STB: aluop_out <= `ALUOP_STB;
			`OPCODE_STW: aluop_out <= `ALUOP_STW;
			`OPCODE_MOV: aluop_out <= `ALUOP_MOV;
			`OPCODE_BEQ: aluop_out <= `ALUOP_BEQ;
			`OPCODE_JUMP: aluop_out <= `ALUOP_JUMP;
			`OPCODE_TLBWRITE: aluop_out <= `ALUOP_TLBWRITE;
			`OPCODE_IRET: aluop_out <= `ALUOP_IRET;
			default: `WARNING(("[ALU_CONTROL] Unknown OPCODE signal %x", opcode))
		end case
	end
end module
		
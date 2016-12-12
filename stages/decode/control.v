`ifndef _control
`define _control

`include "../../define.v"

module control (
	opcode,		// Instruction operation code
	regwrite,	// WB Stage: Register write permission
	branch,		// M Stage: Govern the Fetch stage mux for PC
	memwrite,	// M Stage: If the memory will be written or not
	alusrc,		// Decode stage: src2 source mux govern
	aluop,		// Decode stage: ALU operation
	byteword	// M Stage: If it is a byte (0) or world (1) load/store
);
	// Input signals
	input wire [7:0] opcode;

	// Output signals
	output reg regwrite;
	output reg branch;
	output reg alusrc;
	output reg [7:0] aluop;
	output reg byteword;
	output reg memwrite;
	
	always @* 
	begin
		aluop[7:0] <= opcode[7:0];
		case (opcode)
		`OPCODE_ADD, `OPCODE_SUB, `OPCODE_MOV:
		begin
			branch <= 0;
			regwrite <= 1;
			alusrc <= 0; // From register
			byteword <= 1;
			memwrite <= 0;
		end
		`OPCODE_MUL:
		begin
			branch <= 0;
			regwrite <= 1;
			alusrc <= 0; // From register
			byteword <= 1;
			memwrite <= 0;
		end
		`OPCODE_LDB:
		begin
			branch <= 0;
			regwrite <= 1;
			alusrc <= 1; // From immediat 
			byteword <= 0;
			memwrite <= 0;
		end
		`OPCODE_LDW: 
		begin
			branch <= 0;
			regwrite <= 1;
			alusrc <= 1; // From register
			byteword <= 1;
			memwrite <= 0;
		end
		`OPCODE_STB:
		begin
			branch <= 0;
			regwrite <= 0;
			alusrc <= 0; // From register
			byteword <= 0;
			memwrite <= 1;
		end
		`OPCODE_STW:
		begin
			branch <= 0;
			regwrite <= 0;
			alusrc <= 0; // From register
			byteword <= 1;
			memwrite <= 1;
		end 
		`OPCODE_BEQ, `OPCODE_JUMP, `OPCODE_IRET:
		begin
			branch <= 1;
			regwrite <= 0;
			alusrc <= 1; // From register
			byteword <= 1;
			memwrite <= 0;
		end
		/*`OPCODE_TLBWRITE: 
		begin
			`WARNING(("[CONTROL] Unknown OPCODE signal %x", opcode))
		end
		default: 
		begin
			`WARNING(("[CONTROL] Unknown OPCODE signal %x", opcode))
		end*/
		endcase
	end
endmodule

`endif
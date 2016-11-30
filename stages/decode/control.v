`ifndef _control
`define _control

module control (
	opcode,		// Instruction operation code
	regwrite,	// WB Stage: Register write permission
	branch,		// M Stage: Govern the Fetch stage mux for PC
	alusrc,		// Decode stage: src2 source mux govern
	aluop		// Decode stage: ALU operation
);

endmodule

`endif
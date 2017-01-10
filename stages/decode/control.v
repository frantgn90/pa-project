`ifndef _control
`define _control

`include "../../define.v"

module control (
    clk,
    opcode,		// Instruction operation code
    stall,
    regwrite,	// WB Stage: Register write permission
    memtoreg,	// WB Stage: Rules the mux that says if the data to the register comes from mem (1) or from the ALU (0)
    branch,		// M Stage: Govern the Fetch stage mux for PC
    memwrite,	// M Stage: If the memory will be written or not
    memread,	// M Stage: If the memory will be readed or not
    byteword,	// M Stage: If it is a byte (0) or world (1) load/store
    alusrc,		// EX stage: src2 source mux govern
    aluop		// EX stage: ALU operation
);
	// Input signals
    input clk;
	input [7:0] opcode;
    input stall;

	// Output signals
	output reg [7:0] aluop;
	output reg regwrite;
	output reg branch;
	output reg alusrc;
	output reg byteword;
	output reg memwrite;
	output reg memread;
	output reg memtoreg;

	always @(posedge clk) begin
        if (stall) begin
            regwrite 	<= 0;
			memtoreg 	<= "X";
			branch		<= "X";
			memwrite 	<= 0;
			memread 	<= 0;
			byteword 	<= "X";
			alusrc 		<= "X";
        end
        else begin
            aluop[7:0] <= opcode[7:0];
            case (opcode)
            `OPCODE_ADD, `OPCODE_SUB: begin
                regwrite 	<= 1;
                memtoreg 	<= 0;
                branch		<= 0;
                memwrite 	<= 0;
                memread 	<= 0;
                byteword 	<= "X";
                alusrc 		<= 0;
            end
            `OPCODE_MUL: begin
                regwrite 	<= 1;
                memtoreg 	<= 0;
                branch		<= 0;
                memwrite 	<= 0;
                memread 	<= 0;
                byteword 	<= "X";
                alusrc 		<= 0;
            end
            `OPCODE_LDB: begin
                regwrite 	<= 1;
                memtoreg 	<= 1;
                branch		<= 0;
                memwrite 	<= 0;
                memread 	<= 1;
                byteword 	<= 0;
                alusrc 		<= 1;
            end
            `OPCODE_LDW: begin
                regwrite 	<= 1;
                memtoreg 	<= 1;
                branch		<= 0;
                memwrite 	<= 0;
                memread 	<= 1;
                byteword 	<= 1;
                alusrc 		<= 1;
            end
            `OPCODE_STB: begin
                regwrite 	<= 0;
                memtoreg 	<= "X";
                branch		<= 0;
                memwrite 	<= 1;
                memread 	<= 0;
                byteword 	<= 0;
                alusrc 		<= 1;
            end
            `OPCODE_STW: begin
                regwrite 	<= 0;
                memtoreg 	<= "X";
                branch		<= 0;
                memwrite 	<= 1;
                memread 	<= 0;
                byteword 	<= 1;
                alusrc 		<= 1;
            end 
            `OPCODE_BEQ, `OPCODE_JUMP, `OPCODE_IRET: begin
                regwrite 	<= 0;
                memtoreg 	<= "X";
                branch		<= 1;
                memwrite 	<= 0;
                memread 	<= 0;
                byteword 	<= "X";
                alusrc 		<= 1;
            end
        
            /*`OPCODE_TLBWRITE: 
            begin
                `WARNING(("[CONTROL] Unknown OPCODE signal %x", opcode))
            end
            default: 
            begin
                `WARNING(("[CONTROL] Unknown OPCODE signal %x", opcode))
            end*/
            default: begin
                regwrite 	<= 0;
                memtoreg 	<= "X";
                branch		<= "X";
                memwrite 	<= 0;
                memread 	<= 0;
                byteword 	<= "X";
                alusrc 		<= "X";
            end
            endcase
        end
	end
endmodule

`endif
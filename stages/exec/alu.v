`ifndef _alu
`define _alu


`include "define.v"

module alu(
	input wire [4:0] aluop,
	input wire [`REG_SIZE-1:0] src1,
	input wire [`REG_SIZE-1:0] src2,
	output reg zero = 1'd0,
	output reg overflow = 1'd0,
	output reg [`REG_SIZE-1:0] out = {`REG_SIZE{1'b0}}
	);
	
	//temporal register of 64 bits (for multiplication)
	reg [((`REG_SIZE*2)-1):0] temp = 0;
	
	always @* begin
		case (aluop)
		`ALUOP_ADD: begin
					zero <= 0;
					out <= src1 + src2;
					if ((src1[`REG_SIZE-1] == src2[`REG_SIZE-1]) && (out[`REG_SIZE-1] != src1[`REG_SIZE-1])) overflow <= 1;
					else overflow <= 0;
			end
		`ALUOP_SUB: begin
					zero <= 0;
					out <= src1 - src2;
					if ((src1[`REG_SIZE-1] != src2[`REG_SIZE-1]) && (out[`REG_SIZE-1] != src1[`REG_SIZE-1])) overflow <= 1;
					else overflow <= 0;		
			end
		`ALUOP_MUL: begin
					zero <= 0;
					temp <= src1 * src2;
					out <= temp[`REG_SIZE-1:0];
					//if({`REG_SIZE{out[`REG_SIZE-1]}} != tmp[((`REG_SIZE*2)-1):`REG_SIZE]) overflow <= 1;
					//else
					overflow <= 0;
			end
		`ALUOP_LDB: begin
					zero <= 0;
					out <= src1 + src2;
					//if({`REG_SIZE{out[`REG_SIZE-1]}} != tmp[((`REG_SIZE*2)-1):`REG_SIZE]) overflow <= 1;
					//else
					overflow <= 0;
			end
		`ALUOP_LDB: begin
					zero <= 0;
					out <= src1 + src2;
					//if({`REG_SIZE{out[`REG_SIZE-1]}} != tmp[((`REG_SIZE*2)-1):`REG_SIZE]) overflow <= 1;
					//else
					overflow <= 0;
			end
		`ALUOP_LDW: begin
					zero <= 0;
					temp[`REG_SIZE-1:0] <= src1 + src2;
					out <= (temp[`REG_SIZE-1:0] & `MASK_WORD);
					//if({`REG_SIZE{out[`REG_SIZE-1]}} != tmp[((`REG_SIZE*2)-1):`REG_SIZE]) overflow <= 1;
					//else
					overflow <= 0;
			end
		`ALUOP_STB: begin
					zero <= 0;
					out <= src1 + src2;
					//if({`REG_SIZE{out[`REG_SIZE-1]}} != tmp[((`REG_SIZE*2)-1):`REG_SIZE]) overflow <= 1;
					//else
					overflow <= 0;
			end
		`ALUOP_STW: begin
					zero <= 0;
					temp[`REG_SIZE-1:0] <= src1 + src2;
					out <= (temp[`REG_SIZE-1:0] & `MASK_WORD);
					//if({`REG_SIZE{out[`REG_SIZE-1]}} != tmp[((`REG_SIZE*2)-1):`REG_SIZE]) overflow <= 1;
					//else
					overflow <= 0;
			end
		`ALUOP_MOV: begin
					zero <= 0;
					out <= src1;
			end
		`ALUOP_BEQ: begin
					out <= 32'd0;
					overflow <= 0;
					if((src1 - src2) == 0) zero <= 0;
					else zero <= 1;
			end		
		`ALUOP_JUMP: begin
					zero <= 0;
					out <= 32'd0;
			end
		`ALUOP_TLBWRITE: begin
		
			end
		
		`ALUOP_IRET: begin
		
			end
		default:
			`WARNING(("[ALU] Unknown ALUOP signal %x", aluop))
		endcase
	end
endmodule
`endif
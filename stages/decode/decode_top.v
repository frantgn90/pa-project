`ifndef _decode_top
 `define _decode_top

 `include "../../define.v"
 `include "regfile.v"
 `include "control.v"
 `include "hazard_control.v"

/*****************************************************
 * This is the top level entity for the DECODE stage *
 *****************************************************/

module decode_top(
	                clk,		// Clock signal
	                reset,		// Reset signal
	                pc,		// PC to be bypassed directly to the next stage
	                instruction,	// Instruction to be decoded
	                rreg1,		// Readed register 1
	                rreg2,		// Readed register 2
	                dst_reg,		// Destination register
	                mimmediat,	// Memory type instructions immediat
	                out_pc,

	                regwrite,	// WB stage: Permission write
	                wreg,		// WB stage: Register to write
	                wdata		// WB stage: Data to write

	                // TODO: Control signals as output
                  );
	 // Input signals
	 input wire clk;
	 input wire reset;
	 input wire [`ADDR_SIZE-1:0] pc;
	 input wire [`INSTR_SIZE-1:0] instruction;
	 input wire                   regwrite;
	 input wire [`REG_ADDR-1:0]   wreg;
	 input wire [`REG_SIZE-1:0]   wdata;

	 // Output signals
	 output reg [`REG_SIZE-1:0]   rreg1, rreg2;
	 output reg [`REG_ADDR-1:0]   dst_reg;
	 output reg [`ADDR_SIZE-1:0]  mimmediat;
	 output reg [`ADDR_SIZE-1:0]  out_pc;

	 // Internal wires
	 wire [5:0]                   opcode;	// To be connected to control
	 wire [`REG_ADDR-1:0]         src1;	// To be connected to regfile src1
	 wire [`REG_ADDR-1:0]         src2;	// To be connected to regfile src2
	 wire [`REG_ADDR-1:0]         dst;

	 // Instruction decode
	 assign opcode = instruction[31:25];
	 assign dst 	= instruction[24:20];
	 assign src1	= instruction[19:15];
	 assign src2	= instruction[14:10];
	 

	 // Direct connections to next stage
	 always @* begin
	    case (opcode)
	      `OPCODE_BEQ: begin
		       mimmediat[`ADDR_SIZE-1:0] <= { {17{instruction[24]}},instruction[24:20], instruction[9:0] };
	      end
	      `OPCODE_JUMP: begin
		       mimmediat[`ADDR_SIZE-1:0] <= { {12{instruction[14]}},instruction[24:20], instruction[14:0] };
	      end
	      default:
		      mimmediat[`ADDR_SIZE-1:0] <= { {17{instruction[14]}},instruction[14:0] };
		endcase
	 end
	 
	 always @(posedge clk)begin
		  dst_reg <= dst;
		  out_pc <= pc;
	 end

	 // Net declaration

	 registers regfile (
		                  .clk(clk),
		                  .rreg1(src1), 
		                  .rreg2(src2),
		                  .regwrite(regwrite),
		                  .wreg(wreg),
		                  .wdata(wdata),
		                  .rdata1(rreg1),
		                  .rdata2(rreg2)
	                    ); 

	 // TODO: Generate all control signals
	 control control (
		                .opcode(opcode)
	                  );

endmodule

`endif

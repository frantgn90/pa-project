`ifndef _decode_top
 `define _decode_top

 `include "../../define.v"
 `include "regfile.v"
 `include "control.v"
 `include "hazard_control.v"

/*****************************************************
 * This is the top level entity for the DECODE stage *
 *****************************************************
 * For schematic look at pag. 69, TheProcessor.pdf   *
 *****************************************************/

module decode_top(
    // INPUT SIGNALS
    clk,		    // Clock signal
    reset,		    // Reset signal
    pc,		        // PC to be bypassed directly to the next stage
    instruction,	// Instruction to be decoded
    out_pc,

    // Need to communicate with the external register bank
    src_reg1,		// Address for register 1
    src_reg2,		// Address for register 2
    rin_reg1,	    // Readed register 1
    rin_reg2,	    // Readed register 2

    rout_reg1,
    rout_reg2,

     // Instruction decoded signals
    dest_reg,	    // Destination register
    mimmediat,	    // Memory type instructions immediat

    // Instruction control signals
    regwrite,	    // WB Stage: Permission write
    memtoreg,	    // WB Stage: Rules the mux that says if the data to the register comes from mem (1) or from the ALU (0)

    branch,	        // M Stage: Govern the Fetch stage mux for PC
    memwrite,	    // M Stage: If the memory will be written or not
    memread,	    // M Stage: If the memory will be readed or not
    byteword,	    // M Stage: If it is a byte (0) or world (1) load/store

    alusrc,	// EX stage: src2 source mux govern
    aluop,		// EX stage: ALU operation
    // regdst: This signal is not neede for our ISA
);

	// Input signals
	input wire clk;
	input wire reset;
	input wire [`ADDR_SIZE-1:0] pc;
	input wire [`INSTR_SIZE-1:0] instruction;

	// Need to communicate with the external register bank
	output wire [`REG_ADDR-1:0] src_reg1;		// Address for register 1
	output wire [`REG_ADDR-1:0] src_reg2;		// Address for register 2
	input wire [`REG_SIZE-1:0]  rin_reg1;	// Readed register 1
	input wire [`REG_SIZE-1:0]  rin_reg2;	// Readed register 2

	// Signals for the next stage. There are registers
	// Output signals
	output reg [`ADDR_SIZE-1:0]  out_pc;

	output reg [`REG_SIZE-1:0] rout_reg1; 
	output reg [`REG_SIZE-1:0] rout_reg2;

	output reg [`REG_ADDR-1:0] dest_reg;
	output reg [`ADDR_SIZE-1:0] mimmediat;

	// Output control signals
	output reg regwrite;
	output reg memtoreg;

	output reg branch;
	output reg memwrite;
	output reg memread;
	output reg byteword;

	output reg alusrc;
	output reg aluop;

	 // Internal wires
	 wire [5:0]                   opcode;	// To be connected to control
	 wire [`REG_ADDR-1:0]         dst;

	 wire [`REG_SIZE-1:0] reg1_data;
	 wire [`REG_SIZE-1:0] reg2_data;

	 // Instruction decode
	 assign opcode 		= instruction[31:25];
	 assign dst 		= instruction[24:20];
	 assign src_reg1	= instruction[19:15];
	 assign src_reg2	= instruction[14:10];

	 // Direct connections to next stage
	 assign reg1_data[`REG_SIZE-1:0] = rin_reg1[`REG_SIZE-1:0];
	 assign reg2_data[`REG_SIZE-1:0] = rin_reg2[`REG_SIZE-1:0];

	 assign dst_reg = dst;

	 always @* begin
	    rout_reg1[`REG_SIZE-1:0] <= reg1_data[`REG_SIZE-1:0];
	    rout_reg2[`REG_SIZE-1:0] <= reg2_data[`REG_SIZE-1:0];
	    out_pc <= pc;

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
	 
	 // Control signals generator

	 control control (
		.opcode(opcode),
		.memwrite(memwrite),
		.memread(memread),
		.memtoreg(memtoreg),
		.branch(branch),
		.regwrite(regwrite),
		.alusrc(alusrc),
		.aluop(aluop),
		.byteword(byteword)
         );

endmodule

`endif

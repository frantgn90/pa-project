`ifndef _decode_top
 `define _decode_top

 `include "../../define.v"
 `include "control.v"

/*****************************************************
    TODO list
    - Hazard control
    
 *****************************************************/
 
/*****************************************************
 * This is the top level entity for the DECODE stage *
 *****************************************************
 * For schematic look at pag. 69, TheProcessor.pdf   *
 *****************************************************/

module decode_top(
    // INPUT SIGNALS
    clk,		    // Clock signal
    reset,		    // Reset signal
    we,
    
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

    alusrc,         // EX stage: src2 source mux govern
    funct_code,     // EX stage: FUNCtion code for rtype
    op_code,
    // regdst: This signal is not neede for our ISA
    
    is_mult,         // EX stage: Indicates if the instruction is a multiplication.
                    // This signal will gobern which pipeline will be used.

    // Those signals are not syncrhonous with clock
    jump_addr,      // F stage: Is the target address of the jump
    is_jump,         // F stage: Is the gobernor of the source mux of PC
    
    out_addr_reg1,
    out_addr_reg2,
    stall
);

    // Input signals
    input wire clk;
    input wire reset;
    input wire we;
    input wire [`ADDR_SIZE-1:0] pc;
    input wire [`INSTR_SIZE-1:0] instruction;
    
    input wire stall;
    
    // Need to communicate with the external register bank
    output wire [`REG_ADDR-1:0] src_reg1;		// Address for register 1
    output wire [`REG_ADDR-1:0] src_reg2;		// Address for register 2
    input wire [`REG_SIZE-1:0]  rin_reg1;	// Readed register 1
    input wire [`REG_SIZE-1:0]  rin_reg2;	// Readed register 2

    // Signals for the next stage. There are registers
    // Output signals
    output reg [`ADDR_SIZE-1:0]  out_pc;

    output reg [`REG_ADDR-1:0] out_addr_reg1;
    output reg [`REG_ADDR-1:0] out_addr_reg2;
    output reg [`REG_SIZE-1:0] rout_reg1; 
    output reg [`REG_SIZE-1:0] rout_reg2;

    output reg [`REG_ADDR-1:0] dest_reg;
    output reg [`ADDR_SIZE-1:0] mimmediat;
    output [`ADDR_SIZE-1:0] jump_addr;
    output is_jump; 

    // Output control signals
    output regwrite;
    output memtoreg;

    output branch;
    output memwrite;
    output memread;
    output byteword;

    output alusrc;
    output reg[5:0] funct_code;
    output reg[5:0] op_code;


    output reg is_mult;


    // Internal wires
    wire [5:0]                   opcode;	// To be connected to control
    wire [5:0]                   functcode;// Code for rtype instructions
    
    wire [`REG_ADDR-1:0] dst;
    wire [`REG_ADDR-1:0] dst_load;
    wire [`REG_SIZE-1:0] reg1_data;
    wire [`REG_SIZE-1:0] reg2_data;

    wire stall_execution;
    wire [`ADDR_SIZE-1:0] jump_imm;

    wire [`ADDR_SIZE-1:0]  uns_mimmediat;
    wire [`ADDR_SIZE-1:0]  sig_mimmediat;

    // Instruction decode
    assign opcode = instruction[31:26];
    assign functcode = instruction[5:0];
    
    assign dst = instruction[15:11];
    assign dst_load = instruction[20:16];
    assign src_reg1 = instruction[25:21];
    assign src_reg2 = instruction[20:16];

    // From regfile to boundary registers wire
    assign reg1_data[`REG_SIZE-1:0] = rin_reg1[`REG_SIZE-1:0];
    assign reg2_data[`REG_SIZE-1:0] = rin_reg2[`REG_SIZE-1:0];
    
    // Async signals
    assign jump_imm = instruction[25:0];
    assign jump_addr = pc & 32'hf0000000 | (jump_imm << 2);
    assign is_jump = (opcode == `OP_JUMP);
   
   assign sig_mimmediat[`ADDR_SIZE-1:0] = {{11{instruction[15]}},instruction[15:0]};
   assign uns_mimmediat[`ADDR_SIZE-1:0] = {{11{1'b0}},instruction[15:0]};

    always @(posedge clk) begin            
        if (we) begin 
            out_pc <= pc;
            op_code <= opcode;
            funct_code = functcode;
            rout_reg1[`REG_SIZE-1:0] <= reg1_data[`REG_SIZE-1:0];
            rout_reg2[`REG_SIZE-1:0] <= reg2_data[`REG_SIZE-1:0];
            out_addr_reg1 <= src_reg1;
            mimmediat <= sig_mimmediat;

            if (opcode == `OP_LDW) begin
                out_addr_reg2 <= 32'hXXXXXXXX;
            end
            else begin
                out_addr_reg2 <= src_reg2;
            end
            if (opcode == `OP_RTYPE && functcode == `FN_MUL) begin
                is_mult <= 1;
            end
            else begin
                is_mult <= 0;
            end
            if (opcode == `OP_STB | opcode == `OP_STW) begin
               dest_reg <= 0;//In stores there is no destination register
            end
            else if (opcode == `OP_LDW) begin // TODO: Mirar LDI LDB
                dest_reg <= dst_load;
            end
            else begin
                dest_reg <= dst;
            end

        end
    end
	 
    // Control signals generator
    // NOTE: It acts also as the stage boundary implicitly since its outputs are
    // registers that just are updated on clock posedges
    
    control control (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .stall(stall),
        .memwrite(memwrite),
        .memread(memread),
        .memtoreg(memtoreg),
        .branch(branch),
        .regwrite(regwrite),
        .alusrc(alusrc),
        .byteword(byteword)
    );
endmodule

`endif

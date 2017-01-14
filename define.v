`ifndef _define
`define _define

// Instructions

`define INSTR_SIZE 32
`define ADDR_SIZE 32

// regfile

`define REG_N 32
`define REG_SIZE 32
`define REG_ADDR 5

//Initial PC
`define INITIAL_PC 32'h1000
`define EXCEPTION_PC 32'h2000

//Number of registers
`define REG_SIZE 32

//`define REG_BITS 5

//FUNCTION CODES for RTYPE
`define FN_ADD 6'h20
`define FN_SUB 6'h22
`define FN_MUL 6'h18


//OPERATIONS CODE
`define OP_RTYPE 6'h0
`define OP_LDB 6'h20
`define OP_LDW 6'h23
`define OP_STB 6'h28
`define OP_STW 6'h2b
`define OP_MOV 6'h10
`define OP_BEQ 6'h4
`define OP_JUMP 6'h2
`define OP_TLBWRITE 6'h13
`define OP_IRET 6'h12 

/*
`define OPCODE_ADD 7'h00
`define OPCODE_SUB 7'h01
`define OPCODE_MUL 7'h02
`define OPCODE_LDB 7'h10
`define OPCODE_LDW 7'h11
`define OPCODE_STB 7'h12
`define OPCODE_STW 7'h13
`define OPCODE_MOV 7'h14
`define OPCODE_BEQ 7'h30
`define OPCODE_JUMP 7'h31
`define OPCODE_TLBWRITE 7'h32
`define OPCODE_IRET 7'h33
*/

//ALU OPERATIONS
`define ALUOP_ADD 5'h1
`define ALUOP_SUB 5'h2
`define ALUOP_MUL 5'h3
`define ALUOP_MOV 5'h4
`define ALUOP_JUMP 5'h5
/*
`define ALUOP_ADD 7'h00
`define ALUOP_SUB 7'h01
`define ALUOP_MUL 7'h02
`define ALUOP_LDB 7'h10
`define ALUOP_LDW 7'h11
`define ALUOP_STB 7'h12
`define ALUOP_STW 7'h13
`define ALUOP_MOV 7'h14
`define ALUOP_BEQ 7'h30
`define ALUOP_JUMP 7'h31
`define ALUOP_TLBWRITE 7'h32
`define ALUOP_IRET 7'h33
*/
//MASKS

`define MASK_WORD 32'hFFFC

`define WARNING(M) begin $write("%5t [warning] ", $time); $display M; end

//MEMORY AND CACHE
`define MEMORY_WIDTH 128	  //MEMORY LINE WIDTH
`define MEMORY_DEPTH 16384	//MEMORY LINES -> 2 TO THE 16
`define WIDTH 128		//CACHE LINE WIDTH
`define MEMORY_DATA "proves/3adds_mem.txt"
`ifndef MEMORY_LATENCY
`define MEMORY_LATENCY 1000 //as we want to wait 10 cycles of clock to receive from memory and cpu clock is 100
`endif


// Debug macros
`ifdef DEBUG
	`define INFO(M) begin $write("%5t ", $time); $display M ; end
`else
	`define INFO(M) begin end
`endif

`endif

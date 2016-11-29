`ifndef _define
`define _define

`define INSTR_SIZE 32
`define ADDR_SIZE 32

//Initial PC
`define INITIAL_PC 32'h1000
`define EXCEPTION_PC 32'h2000

//Number of registers
`define REG_SIZE 32
//`define REG_BITS 5

//OPERATIONS CODE
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

//ALU OPERATIONS
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

//MASKS

`define MASK_WORD 32'hFFFC
`endif

`define WARNING(M) begin $write("%5t [warning] ", $time); $display M ; end

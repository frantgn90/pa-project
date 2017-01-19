`ifndef _exec1
`define _exec1

`include "define.v"
`include "alu.v"

module exec1(
    input wire                  clk,
    input wire                  regwrite_in, //Write Permission
    input wire                  alusrc, // If 1 take reg2 otherwise immediat as second operand
    input wire                  reset,
    input wire                  we,//write enable
    input wire [5:0]            opcode, //operation code
    input wire [5:0]            funct_code, //functional code
    input wire [`REG_SIZE-1:0]  reg1_data, //Register1 data
    input wire [`REG_SIZE-1:0]  reg2_data, //Register2 data
    input wire [`REG_SIZE-1:0]  immediat, //Immediat
    input wire [`ADDR_SIZE-1:0] old_pc, //Old_pc
    input wire [`REG_ADDR-1:0]  dst_reg_in, //Destination Register
    input wire                  do_read,//memory read permission
    input wire                  do_write,//memory write permission
    input wire                  is_byte,
    input wire                  memtoreg,//take data from dcache or exec1
    input wire [1:0]            forward_src1,  // FORWARD CONTROL: Manages the forward mux 1
    input wire [1:0]            forward_src2,  // FORWARD CONTROL: Manages the forward mux 2
    input wire [`REG_SIZE-1:0]  wb_forward,    // WB stage: The result of the memory stage
    input wire [`REG_SIZE-1:0]  mem_forward,   // MEM stage: The result of the EXE stage 

    output reg                  regwrite_out, //Write Permission
    output reg                  zero = 1'd0, //Alu zero
    output reg [`REG_SIZE-1:0]  data_store,
    output reg                  overflow = 1'd0, //Alu oveflow
    output reg [`REG_SIZE-1:0]  alu_result, //Alu result
    output reg                  do_read_out,
    output reg                  do_write_out,
    output reg                  is_byte_out,

    output reg                  memtoreg_out,
    output reg [`REG_ADDR-1:0]  dst_reg //Destination Register
);
   
   // Internal wires
   wire                                  alu_zero;
   wire                                  alu_overflow;
   wire [`REG_SIZE-1:0]                  aluresult;
   wire [4:0]                            aluop;
   
   wire [`REG_SIZE-1:0]                  operand1;
   wire [`REG_SIZE-1:0]                  operand2;
   wire [`REG_SIZE-1:0]                  reg2_fw_selection;

    /*************************************************************************
     *               -                                                       *
     *       reg1 --|\|                                          --          *
     *     mem_fw --| |--[operand1]-----------------------------|   \        *
     *      wb_fw --|/|                                         |    \       *
     *               -                                          \     \      *
     *                                                           \     \     *
     *               -                                            |     |--  *
     *       reg2 --|\|                         -                /     /     *
     *     mem_fw --| |--[reg2_fw_selection]---| |              /     /      *
     *      wb_fw --|/|                        |\|              |    /       *
     *               -                         | |--[operand2]--|   /        *
     *                                         |/|               --          *
     *   immediat -----------------------------| |                           *
     *                                          -                            *
     *************************************************************************/
     
    assign operand1 = (forward_src1 == 0) ? reg1_data
        : (forward_src1 == 1) ? mem_forward
        : (forward_src1 == 2) ? wb_forward : 32'bX;
    
    assign reg2_fw_selection = (forward_src2 == 0) ? reg2_data
        : (forward_src2 == 1) ? mem_forward
        : (forward_src2 == 2) ? wb_forward : 32'bX;
    
    assign operand2 = alusrc ? reg2_fw_selection : immediat;

    always @(posedge clk) begin
        if (reset) begin
            do_read_out <= 1'b0;
            memtoreg_out <= 1'b0;
            data_store <= {`REG_SIZE{1'b0}};
            zero <= 1'b0;
            overflow <= 1'b0;
            alu_result <= {`REG_SIZE{1'b0}};
            dst_reg <= {`REG_ADDR{1'b0}};
            regwrite_out <= 1'b0;
            do_write_out <= 1'b0;
            is_byte_out <= 1'b0;

        end else if (we) begin
            do_read_out <= do_read;
            memtoreg_out <= memtoreg;
            data_store <= reg2_data;
            zero <= alu_zero;
            overflow <= alu_overflow;
            alu_result <= aluresult;
            dst_reg <= dst_reg_in;
            regwrite_out <= regwrite_in;
            do_write_out <= do_write;
            is_byte_out <= is_byte;

        end // else: !if(reset)
    end // always @ (posedge clk)

    alucontrol alucontrol(
        .funct(funct_code),
        .opcode(opcode),
        .aluop_out(aluop)
    );
        
    alu alu(
        .aluop(aluop),
        .src1(operand1),
        .src2(operand2),
        .zero(alu_zero),
        .overflow(alu_overflow),
        .out(aluresult)
    );
    
endmodule
`endif

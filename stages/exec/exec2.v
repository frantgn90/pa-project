`ifndef _exec2
`define _exec2

`include "define.v"

module exec2(
   input wire [`REG_SIZE-1:0]  pre_aluresult,
   input wire                  pre_zero,
   input wire                  pre_overflow,
   input wire [`ADDR_SIZE-1:0] pre_new_pc,
   input wire [4:0]            pre_dst,

   output reg [`REG_SIZE-1:0]  aluresult,
   output reg                  zero = 1'd0,
   output reg                  overflow = 1'd0,
   output reg [`ADDR_SIZE-1:0] new_pc = 32'h0000,
   output reg [4:0]            dst
             );

   assign aluresult = pre_aluresult;
   assign zero = pre_zero;
   assign overflow = pre_overflow;
   assign new_pc = pre_new_pc;
   assign dst = pre_dst;



endmodule
`endif

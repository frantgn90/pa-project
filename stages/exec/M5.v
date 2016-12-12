`ifndef _M5
`define _M5

`include "define.v"

module M5(
   input wire [`REG_SIZE-1:0]  pre_m4result,
   input wire                  pre_zero,
   input wire                  pre_overflow,
   input wire [4:0]            pre_dst,

   output reg [`REG_SIZE-1:0]  m5result,
   output reg                  zero = 1'd0,
   output reg                  overflow = 1'd0,
   output reg [`ADDR_SIZE-1:0] new_pc = 32'h0000,
   output reg [4:0]            dst
             );

   assign m5result = pre_m4result;
   assign zero = pre_zero;
   assign overflow = pre_overflow;
   assign dst = pre_dst;

endmodule
`endif

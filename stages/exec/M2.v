`ifndef _M2
`define _M2

`include "define.v"

module M2(
   input wire [`REG_SIZE-1:0]  pre_m1result,
   input wire                  pre_zero,
   input wire                  pre_overflow,
   input wire [4:0]            pre_dst,

   output reg [`REG_SIZE-1:0]  m2result,
   output reg                  zero = 1'd0,
   output reg                  overflow = 1'd0,
   output reg [4:0]            dst
             );

   assign m2result = pre_m1result;
   assign zero = pre_zero;
   assign overflow = pre_overflow;
   assign dst = pre_dst;

endmodule
`endif

`ifndef _M4
`define _M4

`include "define.v"

module M4(
   input wire [`REG_SIZE-1:0]  pre_m3result,
   input wire                  pre_zero,
   input wire                  pre_overflow,
   input wire [4:0]            pre_dst,

   output reg [`REG_SIZE-1:0]  m4result,
   output reg                  zero = 1'd0,
   output reg                  overflow = 1'd0,
   output reg [4:0]            dst
             );

   assign m4result = pre_m3result;
   assign zero = pre_zero;
   assign overflow = pre_overflow;
   assign dst = pre_dst;


endmodule
`endif

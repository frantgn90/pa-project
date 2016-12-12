`ifndef _M3
`define _M3

`include "define.v"

module M3(
   input wire [`REG_SIZE-1:0]  pre_m2result,
   input wire                  pre_zero,
   input wire                  pre_overflow,
   input wire [4:0]            pre_dst,

   output reg [`REG_SIZE-1:0]  m3result,
   output reg                  zero = 1'd0,
   output reg                  overflow = 1'd0,
   output reg [4:0]            dst
             );
	always @* begin
		m3result = pre_m2result;
		zero = pre_zero;
		overflow = pre_overflow;
		dst = pre_dst;
	end

endmodule
`endif

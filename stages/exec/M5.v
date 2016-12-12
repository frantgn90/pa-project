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
   output reg [4:0]            dst
             );

	always @* begin
		m5result = pre_m4result;
		zero = pre_zero;
		overflow = pre_overflow;
		dst = pre_dst;
	end

endmodule
`endif

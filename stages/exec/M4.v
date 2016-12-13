`ifndef _M4
 `define _M4

 `include "define.v"

module M4(
          input wire                 clk,
          input wire                 regwrite_mult_in, //Write Permission
          input wire [`REG_ADDR-1:0] wreg_in, //Destination Register
          input wire [`REG_SIZE-1:0] pre_m3result,
          input wire                 pre_zero,
          input wire                 pre_overflow,

          output reg                 regwrite_out = 1'd0,
          output reg                 zero = 1'd0,
          output reg                 overflow = 1'd0,
          output reg [`REG_SIZE-1:0] m4result,
          output reg [`REG_ADDR-1:0] wreg_out
          );

   always @(posedge clk) begin
	    m4result <= pre_m3result;
	    zero <= pre_zero;
	    overflow <= pre_overflow;
	    regwrite_out <= regwrite_mult_in;
      wreg_out <= wreg_in;
   end
endmodule // M4
`endif





`ifndef _M3
 `define _M3

 `include "define.v"

module M3(
          input wire                 clk,
          input wire                 reset,
          input wire                 we,
          input wire                 regwrite_mult_in, //Write Permission
          input wire [`REG_ADDR-1:0] wreg_in, //Destination Register
          input wire [`REG_SIZE-1:0] pre_m2result,
          input wire                 pre_zero,
          input wire                 pre_overflow,

          output reg                 regwrite_out = 1'd0,
          output reg                 zero = 1'd0,
          output reg                 overflow = 1'd0,
          output reg [`REG_SIZE-1:0] m3result,
          output reg [`REG_ADDR-1:0] dst_reg
          );

   always @(posedge clk) begin
            if (reset) begin
               regwrite_out = 1'd0;
               zero = 1'd0;
               overflow = 1'd0;
               m3result = {`REG_SIZE{1'd0}};
               dst_reg = {`REG_ADDR{1'd0}};
            end
            if (we) begin
            m3result <= pre_m2result;
            zero <= pre_zero;
            overflow <= pre_overflow;
             regwrite_out <= regwrite_mult_in;
               dst_reg <= wreg_in;
            end
   end

endmodule
`endif

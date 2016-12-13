`ifndef _M1
`define _M1

`include "define.v"
`include "alu.v"

module M1(
          input wire                 clk,
          input wire                 regwrite_mult_in, //Write Permission
          input wire [`REG_ADDR-1:0] wreg_in, //Destination Register for
          input wire [4:0]           aluop,
          input wire [`REG_SIZE-1:0] src1,
          input wire [`REG_SIZE-1:0] src2,

          output reg                 regwrite_out = 1'd0,
          output reg                 m1zero = 1'd0,
          output reg                 m1overflow = 1'd0,
          output reg [`REG_ADDR-1:0] wreg_out,
          output reg [`REG_SIZE-1:0] m1result
          );

   reg [((`REG_SIZE*2)-1):0] temp; //temporary reg for multiplication

   always @(posedge clk) begin
      case (aluop)
	      `ALUOP_MUL: begin
				temp <= src1*src2;
	         m1zero <= 0;
	         m1result <= temp[`REG_SIZE-1:0];
           if({`REG_SIZE{m1result[`REG_SIZE-1]}} != temp[((`REG_SIZE*2)-1):`REG_SIZE]) m1overflow <= 1;
			     else m1overflow <= 0;
           wreg_out <= wreg_in;
           regwrite_out <= regwrite_mult_in;
	      end
	      default:
	        ;
	//   `WARNING(("[M1] Unknown M1OP signal %x", aluop))
	    endcase
	 end
endmodule
`endif

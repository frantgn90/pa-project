`ifndef _M1
`define _M1

`include "define.v"
`include "alu.v"

module M1(
   input wire [4:0]            aluop,
   input wire [`REG_SIZE-1:0]  src1,
   input wire [`REG_SIZE-1:0]  src2,
   output reg [`REG_SIZE-1:0]  m1result,
   output reg                  m1zero = 1'd0,
   output reg                  m1overflow = 1'd0,
   output reg [4:0]            dst
             );

   reg [((`REG_SIZE*2)-1):0] temp = 0;

   always @* begin
      case (aluop)
	`ALUOP_MUL: begin
	   zero <= 0;
	   temp <= src1 * src2;
	   m1result <= temp[`REG_SIZE-1:0];
	   m1overflow <= 0;
	end
	default:
	;
	//   `WARNING(("[M1] Unknown M1OP signal %x", aluop))
	endcase
	end
endmodule
`endif

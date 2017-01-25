`ifndef _pc
 `define _pc

 `include "define.v"

module pc(
	        input wire         clk,

	        input wire         is_jump,
	        input wire         is_branch,
	        input wire         is_exception,
	        input wire         reset,

	        input wire [31:0]  pc_jump,
	        input wire [31:0]  pc_branch,
	        input wire         pc_write,
          output wire [31:0] out_pc
	        );

   wire [31:0]               in_pc;
   reg [31:0]                pc;

   // Input mux
   assign in_pc = is_jump?		 	pc_jump :
				          is_branch?	 	pc_branch:
				          is_exception? `EXCEPTION_PC: pc+4;

   // Reg refresh
   always @(posedge clk) begin
	    if (reset) pc <= `INITIAL_PC;
	    else if (pc_write) pc <= in_pc;
   end

   assign out_pc = reset?`INITIAL_PC: pc;

endmodule

`endif

`ifndef _regfile
 `define _regfile

 `include "define.v"

module regfile(
	             input wire                  clk,
               input wire                  reset,
	             input wire [`REG_ADDR-1:0]  rreg1, rreg2,
	             output wire [`REG_SIZE-1:0] rdata1,
	             output wire [`REG_SIZE-1:0] rdata2,
	             input wire                  regwrite,
	             input wire [`REG_ADDR-1:0]  wreg,
	             input wire [`REG_SIZE-1:0]  wdata
               );

   integer                                 i;

   reg [`REG_SIZE-1:0]                     mem [`REG_N-1:0];

   // Reading asynch
   //
   assign rdata1 = ((rreg1 == wreg) && regwrite)? wdata : mem[rreg1][`REG_SIZE-1:0];
   assign rdata2 = ((rreg2 == wreg) && regwrite)? wdata : mem[rreg2][`REG_SIZE-1:0];

   /*   always @* begin

    // Source 1
    if (rreg1 == 0) begin 
    rdata1 <= {`REG_SIZE{1'b0}};
   end
    else if (rreg1 == wreg && regwrite) begin
    rdata1 <= wdata;
      end 
    else
    rdata1 <= mem[rreg1][`REG_SIZE-1:0];

    // Source 2
    if (rreg2 == 0) begin
    rdata2 <= {`REG_SIZE{1'b0}};
   end
    else if (rreg2 == wreg && regwrite) begin
    rdata2 <= wdata;
      end 
    else
    rdata2 <= mem[rreg2][`REG_SIZE-1:0];
   end
    */

   // Writing dst register synchronously
   //
   always @(posedge clk) 
     begin
        if (reset) begin
		       for (i = 0; i < `REG_N; i = i+1) begin
			        mem[i] <= {`REG_SIZE{1'b0}};
		       end
	      end else if (regwrite == 1 && wreg != 0)
	        begin
		         mem[wreg] <= wdata;
	        end
     end

endmodule

`endif

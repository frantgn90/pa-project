`ifndef _forwarding_control
`define _forwarding_control

 `include "define.v"
 
 /*****************************************************
  * Forwarding conditions at pag. 75, TheProcessor.pdf*
  * As for the moment we have not forwarding, those   *
  * conditions are for stall.                         *
  *****************************************************/
  
 
 module forwarding_control(
    id_ex_src1,     // EX stage: register 1
    id_ex_src2,     // EX stage: register 2
    
    ex_mem_regwrite,   // MEM stage: Register write permission 
    ex_mem_dest_reg,   // MEM stage: Register destination address
    
    mem_wb_regwrite,    // WB stage: Register write permission 
    mem_wb_dest_reg,    // WB stage: Register destination address
    
    forward_src1,    // EX stage: Signal that manage the mux for source 1
    forward_src2     // EX stage: Signal that manage the mux for source 2
                // set IF/DWrite to 0
 ); 
    input wire [`REG_ADDR-1:0] id_ex_src1;
    input wire [`REG_ADDR-1:0] id_ex_src2;
    input wire                 ex_mem_regwrite;
    input wire [`REG_ADDR-1:0] ex_mem_dest_reg;
    input wire                 mem_wb_regwrite;
    input wire [`REG_ADDR-1:0] mem_wb_dest_reg;

    output reg [1:0]          forward_src1;
    output reg [1:0]          forward_src2;



    always @* begin
        // EX Hazard, forward from MEM
        if (ex_mem_regwrite && ex_mem_dest_reg != 0 
            && (ex_mem_dest_reg == id_ex_src1))
        begin
            forward_src1 = 2'b01;
        end
        else if (mem_wb_regwrite && mem_wb_dest_reg != 0
                 && (mem_wb_dest_reg == id_ex_src1)) 
          begin
             forward_src1 = 2'b10;
          end
        else begin
           forward_src1 = 2'b00;
        end

        if (ex_mem_regwrite && ex_mem_dest_reg != 0 
            && (ex_mem_dest_reg == id_ex_src2))
        begin
            forward_src2 = 2'b01;
        end

        // MEM Hazard, forward from WB
        // When two hazards occur from both stages, we will want to bypass just
        // the most recent. Then, we only will bypass from MEM when there is no 
        // hazard from EX

        // Look at pag. 76, TheProcessor.pdf


        else if (mem_wb_regwrite && mem_wb_dest_reg != 0
            && (mem_wb_dest_reg == id_ex_src2))
        begin
            forward_src2 = 2'b10;
        end
        else begin
           forward_src2 = 2'b00;
        end
    end

endmodule

`endif
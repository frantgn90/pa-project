`ifndef _forwarding_control
`define _forwarding_control

 `include "define.v"
 
 /*****************************************************
  * Forwarding conditions at pag. 75, TheProcessor.pdf*
  * As for the moment we have not forwarding, those   *
  * conditions are for stall.                         *
  *****************************************************/
  
 
module forwarding_control(
    if_id_branch_src1, //Src1 on this stage (for branch)
    if_id_branch_src2, //Src2 on this stage (for branch)

    id_ex_src1,     // EX stage: register 1
    id_ex_src2,     // EX stage: register 2
    id_ex_dst_reg,  // EX stage: register dst
   
    ex_mem_src2,       // MEM stage: Address from register 2, when we have load RX and after store RX
    ex_mem_regwrite,   // MEM stage: Register write permission 
    ex_mem_dest_reg,   // MEM stage: Register destination address
    ex_mem_writemem,   // MEM stage: Permission of writing
    
    mem_wb_regwrite,    // WB stage: Register write permission 
    mem_wb_dest_reg,    // WB stage: Register destination address
    
    forward_src1,    // EX stage: Signal that manage the mux for source 1
    forward_src2,    // EX stage: Signal that manage the mux for source 2
    forward_mem,      // MEM stage: Signal that manage the mux for write data
                // set IF/DWrite to 0
    forward_branch_src1, //D Stage: We are doing a Branch and need result from Decode
    forward_branch_src2  //D Stage: We are doing a Branch and need restult from Decode
 );
    input wire [`REG_ADDR-1:0] if_id_branch_src1;
    input wire [`REG_ADDR-1:0] if_id_branch_src2;
    input wire [`REG_ADDR-1:0] id_ex_src1;
    input wire [`REG_ADDR-1:0] id_ex_src2;
    input wire [`REG_ADDR-1:0] id_ex_dst_reg;
    input wire                 ex_mem_regwrite;
    input wire [`REG_ADDR-1:0] ex_mem_src2;
    input wire [`REG_ADDR-1:0] ex_mem_dest_reg;
    input wire                 ex_mem_writemem;
    input wire                 mem_wb_regwrite;
    input wire [`REG_ADDR-1:0] mem_wb_dest_reg;

    output reg [1:0]          forward_src1;
    output reg [1:0]          forward_src2;
    output reg [1:0]          forward_mem;
    output wire               forward_branch_src1;
    output wire               forward_branch_src2;

   assign forward_branch_src1 = (ex_mem_regwrite && (id_ex_dst_reg == if_id_branch_src1));
   assign forward_branch_src2 = (ex_mem_regwrite && (id_ex_dst_reg == if_id_branch_src2));

    always @* begin
        // EX Hazard, forward from MEM
        // MEM Hazard, forward from WB
        // When two hazards occur from both stages, we will want to bypass just
        // the most recent. Then, we only will bypass from MEM when there is no
        // hazard from EX

        // Look at pag. 76, TheProcessor.pdf

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
        else if (mem_wb_regwrite && mem_wb_dest_reg != 0
            && (mem_wb_dest_reg == id_ex_src2))
        begin
            forward_src2 = 2'b10;
        end
        else begin
           forward_src2 = 2'b00;
        end

        // Bypass mem to mem for e.g. STORE after LOAD
        // This bypass can be active at same time with bypass mem to dest.
        // This is because the store instruction at the middle is not modifying
        // RF, just memory.

        // NOTE: In case of stores, the value of the ex_mem_dest_reg is not the 
        // destination reg but the address of the source register. This is because
        // the store is not using this bus since it does not writes to RF.
        if (mem_wb_regwrite && ex_mem_writemem && mem_wb_dest_reg != 0
            && (mem_wb_dest_reg == ex_mem_src2))
        begin
           forward_mem = 2'b01;
        end
        else begin
           forward_mem = 2'b00;
        end
    end

endmodule

`endif

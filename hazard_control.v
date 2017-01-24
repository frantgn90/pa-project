`ifndef _hazard_control
`define _hazard_control

 `include "define.v"


/*****************************************************
 * For schematic look at pag. 84, TheProcessor.pdf   *
 * Hazard case at pag. 80, TheProcessor.pdf          *
 *****************************************************/  
 
 module hazard_control(
    id_ex_reg_addr1,     // EXEC stage: register 1
    id_ex_reg_addr2,     // EXEC stage: register 2
    
    ex_mem_memread,
    id_ex_memwrite,
    ex_mem_dst_reg,
    
    stall       // If is decided to stall the execution, inject bubble and 
                // set IF/DWrite to 0
 );
    
    input wire [`REG_ADDR-1:0] id_ex_reg_addr1;
    input wire [`REG_ADDR-1:0] id_ex_reg_addr2;
    
    input wire ex_mem_memread;
    input wire id_ex_memwrite;
    input wire [`REG_ADDR-1:0] ex_mem_dst_reg;
    
    output wire stall;
    
    // Load-use hazard detection, will stall one cycle
    assign stall = (ex_mem_memread
        && ((ex_mem_dst_reg == id_ex_reg_addr1) || ((ex_mem_dst_reg == id_ex_reg_addr2 ) && ~id_ex_memwrite)));
        
endmodule
        
`endif

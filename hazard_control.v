`ifndef _hazard_control
`define _hazard_control

 `include "define.v"


/*****************************************************
 * For schematic look at pag. 84, TheProcessor.pdf   *
 * Hazard case at pag. 80, TheProcessor.pdf          *
 *****************************************************/  
 
 module hazard_control(
    if_id_reg_addr1,     // D stage: register 1
    if_id_reg_addr2,     // D stage: register 2
    
    id_ex_memread,
    id_ex_memwrite,
    id_ex_dst_reg,
    
    stall       // If is decided to stall the execution, inject bubble and 
                // set IF/DWrite to 0
 );
    
    input wire [`REG_ADDR-1:0] if_id_reg_addr1;
    input wire [`REG_ADDR-1:0] if_id_reg_addr2;
    
    input wire id_ex_memread;
    input wire  id_ex_memwrite;
    input wire [`REG_ADDR-1:0] id_ex_dst_reg;
    
    output wire stall;
    
    // Load-use hazard detection, will stall one cycle
    assign stall = id_ex_memread 
        && ((id_ex_dst_reg == if_id_reg_addr1) || ((id_ex_dst_reg == if_id_reg_addr2 ) && ~id_ex_memwrite));
        
endmodule
        
`endif

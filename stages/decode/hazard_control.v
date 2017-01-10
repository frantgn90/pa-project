`ifndef _hazard_control
`define _hazard_control

 `include "../../define.v"
 
/*****************************************************
    TODO list
    - RAW dependencies bw R-instr
    - The hazards will change with bypasses
 *****************************************************/

/*****************************************************
 * For schematic look at pag. 84, TheProcessor.pdf   *
 *****************************************************/
 
 module hazard_control(
    d_src1,     // D stage: register 1
    d_src2,     // D stage: register 2
    
    ex_regwrite, // EX stage: Will the previous instruction write on regfile?
    ex_dest_reg, // EX stage: Register destination for previous instruction
    
    m_regwrite,
    m_dest_reg,
    
    stall       // If is decided to stall the execution, inject bubble and 
                // set IF/DWrite to 0
 );
 
    // NOTE: Look for the destination register at WB stage is not needed because
    // in the worst case, the write could be done at the first half of the cycle
    // and the read at the second half, avoiding the dependecy.
    
    input wire [`REG_ADDR-1:0] d_src1;
    input wire [`REG_ADDR-1:0] d_src2;
    
    input wire ex_regwrite;
    input wire [`REG_ADDR-1:0] ex_dest_reg;
    input wire m_regwrite;
    input wire [`REG_ADDR-1:0] m_dest_reg;
    
    output wire stall;
    
    assign stall = ex_regwrite && (d_src1 == ex_dest_reg || d_src2 == ex_dest_reg)
        || m_regwrite && (d_src1 == m_dest_reg || d_src2 == m_dest_reg);
        
endmodule
        
`endif
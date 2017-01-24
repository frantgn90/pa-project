`ifndef _hazard_control_mult
`define _hazard_control_mult

 `include "define.v"


/****************************************
 *  Controls hazards with mult pipeline *
 ****************************************/
  
 
 module hazard_control_mult(
    // From decode stage (wires)
    id_dst_reg,
    id_src1,
    id_src2,
    id_regwrite,
    id_is_mult,
    
    // From M1,M2,M3,M4,M5 stages
    m1_dst_reg,
    m1_regwrite,
    m2_dst_reg,
    m2_regwrite,
    m3_dst_reg,
    m3_regwrite,
    m4_dst_reg,
    m4_regwrite,
    m5_dst_reg,
    m5_regwrite,
    
    // From E,M
    id_ex_dst_reg,
    id_ex_regwrite,
    ex_mem_dst_reg,
    ex_mem_regwrite,
    
    stall
 );
    
    input wire [`REG_ADDR-1:0]  id_dst_reg;
    input wire [`REG_ADDR-1:0]  id_src1;
    input wire [`REG_ADDR-1:0]  id_src2;
    input wire                  id_regwrite;
    input wire                  id_is_mult;
    
    input wire [`REG_ADDR-1:0]  m1_dst_reg;
    input wire                  m1_regwrite;
    input wire [`REG_ADDR-1:0]  m2_dst_reg;
    input wire                  m2_regwrite;
    input wire [`REG_ADDR-1:0]  m3_dst_reg;
    input wire                  m3_regwrite;
    input wire [`REG_ADDR-1:0]  m4_dst_reg;
    input wire                  m4_regwrite;
    input wire [`REG_ADDR-1:0]  m5_dst_reg;
    input wire                  m5_regwrite;
    
    input wire [`REG_ADDR-1:0]  id_ex_dst_reg;
    input wire                  id_ex_regwrite;
    input wire [`REG_ADDR-1:0]  ex_mem_dst_reg;
    input wire                  ex_mem_regwrite;
    
    output wire stall;
    
            // RAW mult->[rtype|mtype|btype]
    assign stall =   
              (id_src1 == m1_dst_reg && m1_regwrite && id_src1 != 0) ? 1
            : (id_src2 == m1_dst_reg && m1_regwrite && id_src2 != 0) ? 1
            : (id_src1 == m2_dst_reg && m2_regwrite && id_src2 != 0) ? 1
            : (id_src2 == m2_dst_reg && m2_regwrite && id_src2 != 0) ? 1
            : (id_src1 == m3_dst_reg && m3_regwrite && id_src2 != 0) ? 1
            : (id_src2 == m3_dst_reg && m3_regwrite && id_src2 != 0) ? 1
            : (id_src1 == m4_dst_reg && m4_regwrite && id_src2 != 0) ? 1
            : (id_src2 == m4_dst_reg && m4_regwrite && id_src2 != 0) ? 1
            : (id_src1 == m5_dst_reg && m5_regwrite && id_src2 != 0) ? 1
            : (id_src2 == m5_dst_reg && m5_regwrite && id_src2 != 0) ? 1
            : (id_dst_reg == m3_dst_reg && id_regwrite && m5_regwrite && id_dst_reg != 0) ? 1 // WAW Dependency
            
            : (id_is_mult && id_src1 == id_ex_dst_reg && id_ex_regwrite && id_src1 != 0) ? 1  // RAW [rtype|load]->mult
            : (id_is_mult && id_src2 == id_ex_dst_reg && id_ex_regwrite && id_src2 != 0) ? 1
            : (id_is_mult && id_src1 == ex_mem_dst_reg && ex_mem_regwrite && id_src1 != 0) ? 1
            : (id_is_mult && id_src2 == ex_mem_dst_reg && ex_mem_regwrite && id_src2 != 0) ? 1
            : 0;
        
endmodule
        
`endif

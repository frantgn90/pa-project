`ifndef _cpu
 `define _cpu

 `include "define.v"
 `include "stages/fetch/pc.v"
 `include "stages/fetch/fetch.v"
 `include "stages/exec/exec1.v"
 `include "stages/exec/M1.v"
 `include "stages/exec/M2.v"
 `include "stages/exec/M3.v"
 `include "stages/exec/M4.v"
 `include "stages/exec/M5.v"
 `include "memory/memory_async.v"
 `include "cache/cache.v"
 `include "stages/decode/decode_top.v"
 `include "stages/decode/regfile.v"

 `include "stages/decode/hazard_control.v"


module cpu(
    input wire                  clk,
    input wire                  reset,

    // Memory ports
    output wire                 mem_enable,
    output wire                 mem_rw,
    input wire                  mem_ack,
    output wire [`REG_SIZE-1:0] mem_addr,
    input wire [`WIDTH-1:0]     mem_data_out,
    output wire [`WIDTH-1:0]    mem_data_in
);

   /***************************************************************************
    *  MEMORY                                                                 *
    ***************************************************************************/
   memory_async memory(
                      .reset(reset),
                      .addr(mem_addr),
                      .enable(mem_enable),
                      .read_write(mem_rw),
                      .data_in(mem_data_out),
                      .data_out(mem_data_in),
                      .ack(mem_ack)
                      );

   /***************************************************************************
    *  FETCH STAGE                                                            *
    ***************************************************************************/

    // WRITE BACK STAGE
    reg [`REG_ADDR-1:0]                wb_wreg; //destination register
    reg [`REG_SIZE-1:0]                wb_wdata; //result to write
    reg                                wb_regwrite; //write permission

    // WIRE FOR HAZARD CONTROL SIGNALS
    reg pc_write;
    reg if_id_write;
    
    wire                                id_is_jump;
    wire [`ADDR_SIZE-1:0]               id_pc_jump;
    
    wire                                mem_is_branch;
    wire                                if_is_exception;
    wire [`ADDR_SIZE-1:0]               mem_branch;
    wire [`ADDR_SIZE-1:0]               if_old_pc;
    wire [`ADDR_SIZE-1:0]               if_new_pc;
    reg                                 pc_reset;
    reg                                 if_id_reset;

    fetch fetch(
        .clk(clk),
        .is_jump(id_is_jump),
        .is_branch(mem_is_branch),
        .is_exception(if_is_exception),
        .reset(pc_reset),
        .pc_jump(id_pc_jump),
        .pc_branch(mem_branch),
        .old_pc(if_old_pc),
        .new_pc(if_new_pc),
        .pc_write(pc_write)
    );

    wire                                ic_is_byte;
    wire [`REG_SIZE-1:0]                ic_data_out;
    wire [`REG_SIZE-1:0]                ic_memresult;
    wire                                ic_hit;
    wire                                ic_mem_read_req;
    wire [`REG_SIZE-1:0]                ic_mem_read_addr;
    wire [`WIDTH-1:0]                   ic_mem_read_data;
    wire                                ic_mem_read_ack;
    
    reg [`INSTR_SIZE-1:0]               if_instruction;


    cache Icache(
        .clk(clk),
        .reset(reset),
        .addr(if_new_pc),
        .do_read(1'b1),
        .is_byte(ic_is_byte),
        .do_write(1'b0),
        .data_in(0),
        .data_out(ic_memresult),
        .hit(ic_hit),
        .mem_read_req(ic_mem_read_req),
        .mem_read_addr(ic_mem_read_addr),
        .mem_read_data(ic_mem_read_data),
        .mem_read_ack(ic_mem_read_ack)
    );
    
    always @(posedge clk) begin
        if (if_id_reset) if_instruction[`INSTR_SIZE-1:0] <= 32'h0;
        else if (if_id_write) if_instruction[`INSTR_SIZE-1:0] <= ic_memresult;
    end

    /**************************************************************************
     *  DECODE STAGE                                                          *
     **************************************************************************/

    //M1 and exec1: Need to be declared here because we need to plug this wires
    // for hazard control.
    wire 		                ex_regwrite;
    wire                                ex_zero;
    wire                                ex_overflow;
    wire [`REG_SIZE-1:0]                ex_result;
    wire [`ADDR_SIZE-1:0]               ex_pc_branch;
    wire [`REG_ADDR-1:0]                ex_dst_reg;
     
     
    // Wires regfile <-> decode stage
    wire [`REG_ADDR-1:0] addr_reg1;
    wire [`REG_ADDR-1:0] addr_reg2;
    wire [`REG_SIZE-1:0] data_reg1;
    wire [`REG_SIZE-1:0] data_reg2;

    // Wires decode stage <-> exec stage
    wire [`ADDR_SIZE-1:0] id_pc;
    wire [`REG_SIZE-1:0] id_data_reg1;
    wire [`REG_SIZE-1:0] id_data_reg2;

    // NOTE: The outputs of the decode stage are defined as registers, then is not
    // necesary to explicitly manage its behaviour since they will behave as flip-flips

    // Instr decoded signals
   wire [`REG_ADDR-1:0]  id_dest_reg;
   wire [`ADDR_SIZE-1:0] id_mimmediat;

   // Control signals
   wire                   id_regwrite;
   wire                   id_memtoreg;
   wire                   id_is_branch;
   wire                   id_memwrite;
   wire                   id_memread;
   wire                   id_byteword;
   wire                   id_alusrc;
   wire [5:0]             id_opcode;
   wire [5:0]             id_funct_code;
   wire                   id_regwrite_mult_in;
   
   wire                   id_hazard_stall;
   
    regfile registers(
        .clk(clk),
        .rreg1(addr_reg1),
        .rreg2(addr_reg2),
        .wreg(wb_wreg),
        .wdata(wb_wdata),
        .regwrite(wb_regwrite),
        .rdata1(data_reg1),
        .rdata2(data_reg2)
    );

    decode_top decode(
        .clk(clk),
        .reset(if_id_reset),
        .instruction(if_instruction),
        .pc(if_new_pc),
        .out_pc(id_pc),
        .we(if_id_write),

        // Regfile wires
        .src_reg1(addr_reg1),
        .src_reg2(addr_reg2),
        .rin_reg1(data_reg1),
        .rin_reg2(data_reg2),

        .rout_reg1(id_data_reg1),
        .rout_reg2(id_data_reg2),

        // Instruction decoded signals
        .dest_reg(id_dest_reg),	    // Destination register
        .mimmediat(id_mimmediat),	    // Memory type instructions immediat

        // Instruction control signals
        .regwrite(id_regwrite),	// WB Stage: Permission write
        .memtoreg(id_memtoreg),     // WB Stage: Rules the mux that says if the data to the register comes from mem (1) or from the ALU (0)

        .branch(id_is_branch),	    // M Stage: Govern the Fetch stage mux for PC
        .memwrite(id_memwrite),	// M Stage: If the memory will be written or not
        .memread(id_memread),	    // M Stage: If the memory will be readed or not
        .byteword(id_byteword),	// M Stage: If it is a byte (0) or world (1) load/store

        .alusrc(id_alusrc),	    // EX stage: src2 source mux govern
        .funct_code(id_funct_code),		    // EX stage: FUNCTION code for rtype operation
        .op_code(id_opcode),
  

        .is_mult(id_regwrite_mult_in),
        
        // Hazard control
        .ex_regwrite(id_regwrite),
        .ex_dest_reg(id_dest_reg),
        .m_regwrite(ex_regwrite),
        .m_dest_reg(ex_dst_reg),
    
        .hazard_stall(id_hazard_stall),
        
        // JUMP signals. These signals are async.
        .is_jump(id_is_jump),
        .jump_addr(id_pc_jump)
    );



    /**************************************************************************
     *  EXEC STAGE                                                            *
     **************************************************************************/
   wire [`REG_SIZE-1:0]                ex_reg_to_mem;//data to store, directly from regfile
   wire                                ex_memtoreg;
   wire                                ex_do_read;
   wire                                ex_is_branch;
   reg                                ex_mem_reset;
   reg                                ex_mem_write;
   reg                                id_ex_write;
   reg                                id_ex_reset;


   exec1 exec1(
               .clk(clk),
               .regwrite_in(id_regwrite),
               .alusrc(id_alusrc),
               .reset(ex_mem_reset),
               .we(ex_mem_write),
               .opcode(id_opcode),
               .funct_code(id_funct_code),
               .src1(id_data_reg1),
               .reg2(id_data_reg2),
               .immediat(id_mimmediat),
               .old_pc(id_pc),
               .dst_reg_in(id_dest_reg),
               .do_read(id_memread),
               .memtoreg(id_memtoreg),
               .is_branch_in(id_is_branch),

               .regwrite_out(ex_regwrite),
               .zero(ex_zero),
               .data_store(ex_reg_to_mem),
               .overflow(ex_overflow),
               .alu_result(ex_result),
               .pc_branch(ex_pc_branch),
               .do_read_out(ex_do_read),
               .memtoreg_out(ex_memtoreg),
               .is_branch_out(ex_is_branch),
               .dst_reg(ex_dst_reg)
               );


    ///// Multiplication pipeline

    wire                                m1_regwrite_out1;
    wire                                m1_zero;
    wire                                m1_overflow;
    wire [`REG_ADDR-1:0]                m1_dst_reg;
    wire [`REG_SIZE-1:0]                m1_result;

    M1 M1(
        .clk(clk),
        .regwrite_mult_in(id_regwrite_mult_in),
        .wreg_in(id_dest_reg),
        .reset(ex_mem_reset),
        .we(ex_mem_write),
        .opcode(id_opcode),
        .funct_code(id_funct_code),
        .src1(id_data_reg1),
        .src2(id_data_reg2),

        .regwrite_out(m1_regwrite_out1),
        .m1zero(m1_zero),
        .m1overflow(m1_overflow),
        .dst_reg(m1_dst_reg),
        .m1result(m1_result)
    );

   wire                                m2_regwrite_out;
   wire                                m2_zero;
   wire                                m2_overflow;
   wire [`REG_SIZE-1:0]                m2_result;
   wire [`REG_ADDR-1:0]                m2_dst_reg;

   M2 M2(
	       .clk(clk),
	       .regwrite_mult_in(m1_regwrite_out),
         .pre_m1result(m1_result),
         .reset(ex_mem_reset),
         .we(ex_mem_write),
         .pre_zero(m1_zero),
         .pre_overflow(m1_overflow),
         .wreg_in(m1_dst_reg),

         .regwrite_out(m2_regwrite_out),
         .zero(m2_zero),
         .overflow(m2_overflow),
	       .m2result(m2_result),
         .dst_reg(m2_dst_reg)
	       );

   wire                                m3_regwrite_out;
   wire                                m3_zero;
   wire                                m3_overflow;
   wire [`REG_SIZE-1:0]                m3_result;
   wire [`REG_ADDR-1:0]                m3_dst_reg;

   M3 M3(
	       .clk(clk),
	       .regwrite_mult_in(m2_regwrite_out),
         .pre_m2result(m2_result),
         .reset(ex_mem_reset),
         .we(ex_mem_write),
         .pre_zero(m2_zero),
         .pre_overflow(m2_overflow),
         .wreg_in(m2_dst_reg),

         .regwrite_out(m3_regwrite_out),
         .zero(m3_zero),
         .overflow(m3_overflow),
	       .m3result(m3_result),
         .dst_reg(m2_dst_reg)
	       );

   wire                                m4_regwrite_out;
   wire                                m4_zero;
   wire                                m4_overflow;
   wire [`REG_SIZE-1:0]                m4_result;
   wire [`REG_ADDR-1:0]                m4_dst_reg;

   M4 M4(
	       .clk(clk),
	       .regwrite_mult_in(m3_regwrite_out),
         .pre_m3result(m3_result),
         .reset(ex_mem_reset),
         .we(ex_mem_write),
         .pre_zero(m3_zero),
         .pre_overflow(m3_overflow),
         .wreg_in(m3_dst_reg),

         .regwrite_out(m4_regwrite_out),
         .zero(m4_zero),
         .overflow(m4_overflow),
	       .m4result(m4_result),
         .dst_reg(m4_dst_reg)
	       );

   wire                                m5_regwrite_out;
   wire                                m5_zero;
   wire                                m5_overflow;
   wire [`REG_SIZE-1:0]                m5_result;
   wire [`REG_ADDR-1:0]                m5_dst_reg;

   M5 M5(
	       .clk(clk),
	       .regwrite_mult_in(m4_regwrite_out),
         .pre_m4result(m4_result),
         .reset(ex_mem_reset),
         .we(ex_mem_write),
         .pre_zero(m4_zero),
         .pre_overflow(m4_overflow),
         .wreg_in(m4_dst_reg),

         .regwrite_out(m5_regwrite_out),
         .zero(m5_zero),
         .overflow(m5_overflow),
	       .m5result(m5_result),
         .dst_reg(m5_dst_reg)
	       );

   /***************************************************************************
    *   MEMORY STAGE                                                          *
    ***************************************************************************/

   reg [`REG_ADDR-1:0]                 dc_dst_reg;
   reg [`REG_SIZE-1:0]                 dc_wdata;
   reg                                 dc_regwrite;

   wire                                dc_is_byte;
   wire                                dc_is_write;
   wire [`REG_SIZE-1:0]                dc_data_out;
   wire [`REG_SIZE-1:0]                dc_memresult;
   wire                                dc_hit;

   wire                                dc_mem_write_req;
   wire [`REG_SIZE-1:0]                dc_mem_write_addr;
   wire [`WIDTH-1:0]                   dc_mem_write_data;
   wire                                dc_mem_write_ack;

   wire                                dc_mem_read_req;
   wire [`REG_SIZE-1:0]                dc_mem_read_addr;
   wire [`WIDTH-1:0]                   dc_mem_read_data;
   wire                                dc_mem_read_ack;
   reg                                mem_wb_reset;
   reg                                mem_wb_write;
    cache Dcache(
        .clk(clk),
        .reset(reset),
        .addr(ex_result),
        .do_read(ex_do_read),
        .is_byte(dc_is_byte),
        .do_write(dc_is_write),
        .data_in(ex_reg_to_mem),
        .data_out(dc_memresult),
        .hit(dc_hit),
        .mem_write_req(dc_mem_write_req),
        .mem_write_addr(dc_mem_write_addr),
        .mem_write_data(dc_mem_write_data),
        .mem_write_ack(dc_mem_write_ack),
        .mem_read_req(dc_mem_read_req),
        .mem_read_addr(dc_mem_read_addr),
        .mem_read_data(dc_mem_read_data),
        .mem_read_ack(dc_mem_read_ack)
    );

    assign mem_branch = ex_pc_branch;
    assign mem_is_branch = ex_is_branch & ex_zero; //If we branch
   always @(posedge clk) begin
      
      if (mem_wb_reset) begin
        dc_dst_reg <= {`REG_ADDR{1'b0}};
        dc_regwrite <= 1'b0;
        dc_wdata <= {`REG_SIZE{1'b0}};
      end
      else if (mem_wb_write) begin
         dc_dst_reg <= ex_dst_reg;
         dc_regwrite <= ex_regwrite;
         dc_wdata <= ex_memtoreg? dc_data_out : ex_result;
      end
      wb_wreg <= dc_regwrite? dc_dst_reg : m5_dst_reg;
      wb_wdata <= dc_regwrite? dc_wdata : m5_result;
      wb_regwrite <= m5_regwrite_out | dc_regwrite;
   end

    //ARBITER
    arbiter Arbiter(
        .clk(clk),
        .reset(reset),
        .ic_read_req(ic_mem_read_req),
        .ic_read_ack(ic_mem_read_ack),
        .ic_read_addr(ic_mem_read_addr),
        .ic_read_data(ic_mem_read_data),

        .dc_read_req(dc_mem_write_req),
        .dc_read_ack(dc_mem_write_ack),
        .dc_read_addr(dc_mem_write_addr),
        .dc_read_data(dc_mem_write_data),

        .dc_write_req(dc_mem_read_req),
        .dc_write_ack(dc_mem_read_ack),
        .dc_write_addr(dc_mem_read_addr),
        .dc_write_data(dc_mem_read_data),

        .mem_enable(mem_enable),
        .mem_rw(mem_rw),
        .mem_ack(mem_ack),
        .mem_addr(mem_addr),
        .mem_data_in(mem_data_out),
        .mem_data_out(mem_data_in)
    );

   wire ic_stall = !ic_hit;
   wire dc_stall = !dc_hit;

   always @* begin
      if (reset) begin
         pc_reset <= 1'b1;
         if_id_reset <= 1'b1;
         if_id_write <= 1'b1;
         id_ex_reset <= 1'b1;
         id_ex_write <= 1'b1;
         ex_mem_reset <= 1'b1;
         ex_mem_write <= 1'b1;
         mem_wb_reset <= 1'b1;
         mem_wb_write <= 1'b1;
         //TODO COP RESET
      end else if (dc_stall) begin
         pc_reset <= 1'b0;
         if_id_reset <= 1'b0;
         if_id_write <= 1'b0;
         id_ex_reset <= 1'b0;
         id_ex_write <= 1'b0;
         ex_mem_reset <= 1'b1;
         ex_mem_write <= 1'b1;
         mem_wb_reset <= 1'b1;
         mem_wb_write <= 1'b1;
      end // if (dc_stall)
      //TODO ex_isjump | ex_exc_ret
      else if (id_hazard_stall) begin
         pc_reset <= 1'b0;
         if_id_reset <= 1'b0;
         if_id_write <= 1'b0;
         id_ex_reset <= 1'b1;
         id_ex_write <= 1'b1;
         ex_mem_reset <= 1'b0;
         ex_mem_write <= 1'b1;
         mem_wb_reset <= 1'b0;
         mem_wb_write <= 1'b1;
      end // if (id_hazard_stall)
      else if(ic_stall) begin
         pc_reset <= 1'b0;
         if_id_reset <= 1'b0;
         if_id_write <= 1'b1;
         id_ex_reset <= 1'b0;
         id_ex_write <= 1'b1;
         ex_mem_reset <= 1'b0;
         ex_mem_write <= 1'b1;
         mem_wb_reset <= 1'b0;
         mem_wb_write <= 1'b1;
      end // if (ic_stall)
      else begin
         pc_reset <= 1'b0;
         if_id_reset <= 1'b0;
         if_id_write <= 1'b1;
         id_ex_reset <= 1'b0;
         id_ex_write <= 1'b1;
         ex_mem_reset <= 1'b0;
         ex_mem_write <= 1'b1;
         mem_wb_reset <= 1'b0;
         mem_wb_write <= 1'b1;
      end
    end
endmodule
`endif

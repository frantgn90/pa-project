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
 
 `include "regfile.v"
 `include "forwarding_control.v"
 `include "hazard_control.v"


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
    *  FREE MODULES, NONE STAGE OWNS THEM                                     *
    ***************************************************************************/
        
    // Hazard control
    wire [`REG_ADDR-1:0] addr_reg1;
    wire [`REG_ADDR-1:0] addr_reg2;
    wire                 id_regwrite;
    wire [`REG_ADDR-1:0] id_dest_reg;
    wire 		             ex_regwrite;
    wire [`REG_ADDR-1:0] ex_dst_reg;
    wire                 id_memread;
    wire                 id_memwrite;
    wire                  ex_do_write;
    wire                  ex_do_read;
    wire                 hazard_stall;
   reg [`REG_ADDR-1:0]  id_addr_reg1;
   reg [`REG_ADDR-1:0]  id_addr_reg2;
   

    hazard_control hazards (
        .id_ex_reg_addr1(id_addr_reg1),
        .id_ex_reg_addr2(id_addr_reg2),
        .ex_mem_memread(ex_do_read),
        .id_ex_memwrite(id_memwrite),
        .ex_mem_dst_reg(ex_dst_reg),
        .stall(hazard_stall)
    );
    
    // Forwarding control
    wire [`REG_ADDR-1:0] id_src1;
    wire [`REG_ADDR-1:0] id_src2;
    reg [`REG_ADDR-1:0]  dc_dst_reg; // They are reg because the flip-flop behaviour is coded here
    reg                  dc_regwrite;
    wire [1:0]           forward_src1;
    wire [1:0]           forward_src2;
    wire [1:0]           forward_mem;
    wire                 forward_branch_src1;
    wire                 forward_branch_src2;
   
    reg [`INSTR_SIZE-1:0] if_instruction;// Fetch stage but needed for forwarding branches

    forwarding_control forwarding (
        .if_id_branch_src1(if_instruction[25:21]),
        .if_id_branch_src2(if_instruction[20:16]),
        .id_ex_src1(id_src1),
        .id_ex_src2(id_src2),
        .id_ex_dst_reg(id_dest_reg),
        .ex_mem_regwrite(ex_regwrite),
        .ex_mem_dest_reg(ex_dst_reg),
        .ex_mem_readmem(ex_do_read),
        .mem_wb_regwrite(dc_regwrite),
        .mem_wb_dest_reg(dc_dst_reg),
        .id_ex_writemem(id_memwrite),
        .forward_src1(forward_src1),
        .forward_src2(forward_src2),
        .forward_mem(forward_mem),
        .forward_branch_src1(forward_branch_src1),
        .forward_branch_src2(forward_branch_src2)
    );
    

   /***************************************************************************
    *  MEMORY                                                                 *
    ***************************************************************************/
   memory_async memory(
                      .reset(reset),
                      .addr(mem_addr),
                      .enable(mem_enable),
                      .read_write(mem_rw),
                      .data_in(mem_data_in),
                      .data_out(mem_data_out),
                      .ack(mem_ack)
                      );

   /***************************************************************************
    *  FETCH STAGE                                                            *
    ***************************************************************************/

    // WRITE BACK STAGE
    wire [`REG_ADDR-1:0]                wb_wreg; //destination register
    wire [`REG_SIZE-1:0]                wb_wdata; //result to write
    wire                                wb_regwrite; //write permission
    wire [`ADDR_SIZE-1:0]               addr_branch;

    reg                                 pc_write;
    wire                                id_is_jump;
    wire [`ADDR_SIZE-1:0]               id_pc_jump;
    wire                                is_branch;
    wire                                if_is_exception = 1'b0;
    //wire [`ADDR_SIZE-1:0]               if_old_pc;
    reg [`ADDR_SIZE-1:0]                if_pc;
   wire [`ADDR_SIZE-1:0]                pc;
   reg                                  if_id_write;
   reg                                  if_id_reset;
    reg                                 pc_reset;


    fetch fetch(
        .clk(clk),
        .is_jump(id_is_jump),
        .is_branch(is_branch),
        .is_exception(if_is_exception),
        .reset(pc_reset),
        .pc_jump(id_pc_jump),
        .pc_branch(addr_branch),
        .pc_out(pc),
        .pc_write(pc_write)
    );

    wire                                ic_is_byte;
    wire [`REG_SIZE-1:0]                ic_data_out;
    wire [`REG_SIZE-1:0]                ic_memresult_wire;
    wire                                ic_hit;
    wire                                ic_mem_read_req;
    wire [`REG_SIZE-1:0]                ic_mem_read_addr;
    wire [`WIDTH-1:0]                   ic_mem_read_data;
    wire                                ic_mem_read_ack;




    cache Icache(
        .clk(clk),
        .reset(reset),
        .addr(pc),
        .do_read(1'b1 & (~is_branch)),
        .is_byte(1'b0),
        .do_write(1'b0),
        .data_in(0),
        .data_out(ic_memresult_wire),
        .hit(ic_hit),
        .mem_read_req(ic_mem_read_req),
        .mem_read_addr(ic_mem_read_addr),
        .mem_read_data(ic_mem_read_data),
        .mem_read_ack(ic_mem_read_ack)
    );

    always @(posedge clk) begin
        if (if_id_reset) begin
           if_instruction <= 32'hffffffff; // opcode 0xff
           if_pc <= {`ADDR_SIZE{1'b0}};
        end
        else if (if_id_write) begin
          if_instruction[`INSTR_SIZE-1:0] <= ic_memresult_wire;
           if_pc <= pc;
        end
    end

    /**************************************************************************
     *  DECODE STAGE                                                          *
     **************************************************************************/

    //M1 and exec1: Need to be declared here because we need to plug this wires
    // for hazard control.
    
    wire                                ex_zero;
    wire                                ex_overflow;
    wire [`REG_SIZE-1:0]                ex_result;
     
    reg                                id_ex_write;
    reg                                id_ex_reset;
     
    // Wires regfile <-> decode stage
    wire [`REG_SIZE-1:0] data_reg1;
    wire [`REG_SIZE-1:0] data_reg2;
    wire                  id_bne;

    // Wires decode stage <-> exec stage
    wire [`ADDR_SIZE-1:0] id_pc;
    //Registers boundary decode <-> exec
    reg [`REG_SIZE-1:0] id_data_reg1;
    reg [`REG_SIZE-1:0] id_data_reg2;

    // NOTE: The outputs of the decode stage are defined as registers, then is not
    // necesary to explicitly manage its behaviour since they will behave as flip-flips

   wire [`ADDR_SIZE-1:0] id_mimmediat;

   // Control signals
   wire                   id_memtoreg;
   wire                   id_is_branch;
   wire                   id_byteword;
   wire                   id_alusrc;
   wire [5:0]             id_opcode;
   wire [5:0]             id_funct_code;
   wire                   id_regwrite_mult_in;
   wire [`REG_SIZE-1:0]   id_branch_1;
   wire [`REG_SIZE-1:0]   id_branch_2;

    regfile registers(
        .clk(clk),
        .reset(reset),
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
        .reset(id_ex_reset),
        .instruction(if_instruction),
        .pc(if_pc),
        .out_pc(id_pc),
        .we(id_ex_write),

        // Regfile wires (asynchronous)
        .src_reg1(addr_reg1),
        .src_reg2(addr_reg2),

        // Instruction decoded signals
        .dest_reg(id_dest_reg),	    // Destination register
        .mimmediat(id_mimmediat),	    // Memory type instructions immediat

        // Instruction control signals
        .regwrite(id_regwrite),	// WB Stage: Permission write
        .memtoreg(id_memtoreg),     // WB Stage: Rules the mux that says if the data to the register comes from mem (1) or from the ALU (0)

        .branch(id_is_branch),	    // M Stage: Govern the Fetch stage mux for PC
        .memwrite(id_memwrite),	// M Stage: If the memory will be written or not
        .memread(id_memread),	    // M Stage: If the memory will be readed or not
        .byteword(id_byteword),	// M Stage: If it is a byte (0) or word (1) load/store

        .alusrc(id_alusrc),	    // EX stage: src2 source mux govern
        .funct_code(id_funct_code),		    // EX stage: FUNCTION code for rtype operation
        .op_code(id_opcode),



        .is_mult(id_regwrite_mult_in),

        // JUMP signals. These signals are async.
        .is_jump(id_is_jump),
        .jump_addr(id_pc_jump),
        .stall(hazard_stall),        // If 1, inject bubble to next stage

        .out_addr_reg1(id_src1),
        .out_addr_reg2(id_src2)
    );
    
   wire [`REG_SIZE-1:0]   ex_wire_alu_result;//Wire that brings the result of alu directly to decode for calculate the branch
   
   assign addr_branch = if_pc+4 + ({{16{if_instruction[15]}},if_instruction[15:0]} << 2);
   assign is_branch = id_is_branch & id_bne;
   assign id_branch_1 = forward_branch_src1? ex_wire_alu_result: data_reg1;
   assign id_branch_2 = forward_branch_src2? ex_wire_alu_result: data_reg2;
   assign id_bne = (id_branch_1 != id_branch_2);
   
   always @(posedge clk) begin //registers boundary from decode to exec1 and M1
      if (id_ex_reset) begin
        id_data_reg1 <= 32'b0;
        id_data_reg2 <= 32'b0;
        id_addr_reg1 <= 32'b0;
        id_addr_reg2 <= 32'b0;
      end
      else if (id_ex_write) begin 
        id_data_reg1 <= data_reg1;
        id_data_reg2 <= data_reg2;
        id_addr_reg1 <= addr_reg1;
        id_addr_reg2 <= addr_reg2;
      end
   end
    /**************************************************************************
     *  EXEC STAGE                                                            *
     **************************************************************************/
   wire [`REG_SIZE-1:0]                ex_reg_to_mem;//data to store, directly from regfile
   wire                                ex_memtoreg;
   wire                                ex_is_byte;
   wire                                ex_is_branch;
   reg                                ex_mem_reset;
   reg                                ex_mem_write;
   wire [`REG_SIZE-1:0]               dc_memresult_forward;


   exec1 exec1(
               .clk(clk),
               .regwrite_in(id_regwrite),
               .alusrc(id_alusrc),
               .reset(ex_mem_reset),
               .we(ex_mem_write),
               .opcode(id_opcode),
               .funct_code(id_funct_code),
               .reg1_data(id_data_reg1),
               .reg2_data(id_data_reg2),
               .immediat(id_mimmediat),
               .old_pc(id_pc),
               .dst_reg_in(id_dest_reg),
               .do_read(id_memread),
               .do_write(id_memwrite),
               .is_byte(~id_byteword),
               .memtoreg(id_memtoreg),
               
               // Forwarding mux control signals
               .forward_src1(forward_src1),
               .forward_src2(forward_src2),
               .wb_forward(wb_wdata),
               .mem_forward(ex_result), // The registers of boundary are inside the module
               .dc_memresult_forward(dc_memresult_forward),

               .regwrite_out(ex_regwrite),
               .zero(ex_zero),
               .data_store(ex_reg_to_mem),
               .overflow(ex_overflow),
               .wire_alu_result(ex_wire_alu_result),
               .alu_result(ex_result),
               .do_write_out(ex_do_write),
               .do_read_out(ex_do_read),
               .is_byte_out(ex_is_byte),
               .memtoreg_out(ex_memtoreg),
               .dst_reg(ex_dst_reg)
               );


    ///// Multiplication pipeline

    wire                                m1_regwrite_out;
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

        .regwrite_out(m1_regwrite_out),
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
   
   reg [`REG_SIZE-1:0]                 dc_wdata;

   wire                                dc_is_byte;
   wire                                dc_is_write;
   reg [`REG_SIZE-1:0]                 dc_memresult;
   wire                                dc_hit;

   wire                                dc_mem_write_req;
   wire [`REG_SIZE-1:0]                dc_mem_write_addr;
   wire [`WIDTH-1:0]                   dc_mem_write_data;
   wire                                dc_mem_write_ack;

   wire                                dc_mem_read_req;
   wire [`REG_SIZE-1:0]                dc_mem_read_addr;
   wire [`WIDTH-1:0]                   dc_mem_read_data;
   wire                                dc_mem_read_ack;
   reg                                 mem_wb_reset;
   reg                                 mem_wb_write;


    // MUX for forwarding
   // assign data_to_write = (forward_mem == 0) ? ex_reg_to_mem
   //     : (forward_mem == 1) ? dc_wdata
   //     : 32'bX;
    
    cache Dcache(
        .clk(clk),
        .reset(reset),
        .addr(ex_result),
        .do_read(ex_do_read),
        .is_byte(ex_is_byte),
        .do_write(ex_do_write),
        .data_in(ex_reg_to_mem),
        .data_out(dc_memresult_forward),
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

    assign wb_wreg = dc_regwrite? dc_dst_reg : m5_dst_reg;
    assign wb_wdata = dc_regwrite? dc_wdata : m5_result;
    assign wb_regwrite = m5_regwrite_out | dc_regwrite;

   always @(posedge clk) begin
      if (mem_wb_reset) begin
        dc_dst_reg <= {`REG_ADDR{1'b0}};
        dc_regwrite <= 1'b0;
        dc_wdata <= {`REG_SIZE{1'b0}};
        dc_memresult <= {`REG_SIZE{1'b0}};
      end
      else if (mem_wb_write) begin
         dc_dst_reg <= ex_dst_reg;
         dc_regwrite <= ex_regwrite;
         dc_memresult <= dc_memresult_forward;
         dc_wdata <= ex_memtoreg? dc_memresult_forward : ex_result;
      end
   end

    //ARBITER
    arbiter Arbiter(
        .clk(clk),
        .reset(reset),
        .ic_read_req(ic_mem_read_req),
        .ic_read_ack(ic_mem_read_ack),
        .ic_read_addr(ic_mem_read_addr),
        .ic_read_data(ic_mem_read_data),

        .dc_read_req(dc_mem_read_req),
        .dc_read_ack(dc_mem_read_ack),
        .dc_read_addr(dc_mem_read_addr),
        .dc_read_data(dc_mem_read_data),

        .dc_write_req(dc_mem_write_req),
        .dc_write_ack(dc_mem_write_ack),
        .dc_write_addr(dc_mem_write_addr),
        .dc_write_data(dc_mem_write_data),

        .mem_enable(mem_enable),
        .mem_rw(mem_rw),
        .mem_ack(mem_ack),
        .mem_addr(mem_addr),
        .mem_data_in(mem_data_in),
        .mem_data_out(mem_data_out)
    );

   wire dc_enable = ex_do_read | ex_do_write;
   wire ic_stall = ~ic_hit & ~reset;
   wire dc_stall = ~dc_hit & dc_enable;

   always @* begin
      if (reset) begin
         pc_reset <= 1'b1;
         pc_write <= 1'b1;
         if_id_reset <= 1'b1;
         if_id_write <= 1'b1;
         id_ex_reset <= 1'b1;
         id_ex_write <= 1'b1;
         ex_mem_reset <= 1'b1;
         ex_mem_write <= 1'b1;
         mem_wb_reset <= 1'b1;
         mem_wb_write <= 1'b1;
         //TODO COP RESET
      end else if (dc_stall) begin // if (reset)
         // Stall until wb
         pc_reset <= 1'b0;
         pc_write <= 1'b0;
         if_id_reset <= 1'b0;
         if_id_write <= 1'b0;
         id_ex_reset <= 1'b0;
         id_ex_write <= 1'b0;
         ex_mem_reset <= 1'b0;
         ex_mem_write <= 1'b0;
         mem_wb_reset <= 1'b1;
         mem_wb_write <= 1'b0;
      end // if (dc_stall)
      //TODO ex_isjump | ex_exc_ret
      else if (hazard_stall) begin
         pc_reset <= 1'b0;
         pc_write <= 1'b0;
         if_id_reset <= 1'b0;
         if_id_write <= 1'b0;
         id_ex_reset <= 1'b0;
         id_ex_write <= 1'b0;
         ex_mem_reset <= 1'b1;
         ex_mem_write <= 1'b1;
         mem_wb_reset <= 1'b0;
         mem_wb_write <= 1'b1;
      end // if (id_hazard_stall)
      else if(is_branch) begin
         pc_reset <= 1'b0;
         pc_write <= 1'b1;
         if_id_reset <= 1'b1;
         if_id_write <= 1'b1;
         id_ex_reset <= 1'b0;
         id_ex_write <= 1'b1;
         ex_mem_reset <= 1'b0;
         ex_mem_write <= 1'b1;
         mem_wb_reset <= 1'b0;
         mem_wb_write <= 1'b1;
      end
      else if(ic_stall) begin
        // Stall at fetch
         pc_reset <= 1'b0;
         pc_write <= 1'b0;
         if_id_reset <= 1'b1;
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
         pc_write <= 1'b1;
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

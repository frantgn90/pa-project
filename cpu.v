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
 `include "memory/memory_sync.v"
 `include "cache/cache.v"
 `include "stages/decode/decode_top.v"
 `include "stages/decode/regfile.v"




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

   //WRITE BACK STAGE
   wire [`REG_ADDR-1:0]                wb_wreg; //destination register
   wire [`REG_SIZE-1:0]                wb_wdata; //result to write
   wire                                wb_regwrite; //write permission



   wire                                if_is_jump;
   wire                                if_is_branch;
   wire                                if_is_exception;
   wire [`ADDR_SIZE-1:0]               if_pc_jump;
   wire [`ADDR_SIZE-1:0]               if_branch;
   wire [`ADDR_SIZE-1:0]               if_old_pc;
   wire [`ADDR_SIZE-1:0]               if_new_pc;

   fetch fetch(
               .clk(clk),
               .is_jump(if_is_jump),
               .is_branch(if_is_branch),
               .is_exception(if_is_exception),
               .reset(reset),
               .pc_jump(if_pc_jump),
               .pc_branch(if_branch),
               .old_pc(if_old_pc),
               .new_pc(if_new_pc)
               );

   wire                                ic_is_byte;
   wire [`REG_SIZE-1:0]                ic_data_out;
   wire [`REG_SIZE-1:0]                ic_memresult;
   wire                                ic_hit;
   wire                                ic_mem_read_req;
   wire [`REG_SIZE-1:0]                ic_mem_read_addr;
   wire [`WIDTH-1:0]                   ic_mem_read_data;
   wire                                ic_mem_read_ack;


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

   regfile registers(
                     .clk(clk),
                     .rreg1(),//address register1
                     .rreg2(),
                     .wreg(wb_wreg),//destination register
                     .wdata(wb_wdata),//data to write
                     .regwrite(wb_regwrite),//write permission
                     .rdata1(), //Data form register1
                     .radat2()
                     );




   //WIRE TO COME FROM DECODE TO EXEC1 AND M1:
   //exec1
   wire                                ex_regwrite_in;
   wire                                ex_alusrc; //If 1 take reg2 otherwise immediat
   wire [`REG_SIZE-1:0]                ex_immediat;
   wire [`ADDR_SIZE-1:0]               ex_old_pc;

   //M1:
   wire                                regwrite_mult_in;
   wire                                m1_wreg_in;

   //M1 and exec1
   wire [`REG_SIZE-1:0]                ex_src1;
   wire [`REG_SIZE-1:0]                ex_reg2;
   wire [`REG_ADDR-1:0]                ex_wreg_in;
   wire [4:0]                          ex_aluop;
   wire [`REG_SIZE-1:0]                ex_regwrite;

   exec1 exec1(
	             .clk(clk),
	             .ex_regwrite_in(ex_regwrite_in),
	             .alusrc(ex_alusrc),
	             .aluop(ex_aluop),
               .src1(ex_src1),
               .reg2(ex_reg2),
               .immediat(ex_immediat),
               .old_pc(ex_old_pc),
	             .wreg_in(ex_wreg_in),

	             .regwrite_out(ex_regwrite),
               .zero(ex_zero),
               .overflow(ex_oveflow),
               .alu_result(ex_result),
               .pc_branch(ex_pc_branch),
               .wreg_out(ex_wreg_out)
               );


   wire                                m1_regwrite_out;
   wire                                m1_zero;
   wire                                m1_overflow;
   wire [`REG_ADDR-1:0]                m1_wreg_out;
   wire [`REG_SIZE-1:0]                m1_result;

   M1 M1(
	       .clk(clk),
	       .regwrite_mult_in(m1_regwrite_mult_in),
	       .wreg_in(m1_wreg_in),
         .aluop(ex_aluop),
         .src1(ex_src1),
         .src2(ex_src2),

	       .regwrite_out(m1_regwrite_out),
         .m1zero(m1_zero),
         .m1overflow(m1_overflow),
	       .wreg_out(m1_wreg_out),
	       .m1result(m1_result)
         );

   wire                                m2_regwrite_out;
   wire                                m2_zero;
   wire                                m2_overflow;
   wire [`REG_SIZE-1:0]                m2_result;
   wire [`REG_ADDR-1:0]                m2_wreg_out;

   M2 M2(
	       .clk(clk),
	       .regwrite_mult_in(m1_regwrite_out),
         .pre_m1result(m1_result),
         .pre_zero(m1_zero),
         .pre_overflow(m1_overflow),
         .wreg_in(m1_wreg_out),

         .regwrite_out(m2_regwrite_out),
         .zero(m2_zero),
         .overflow(m2_overflow),
	       .m2result(m2_result),
         .wreg_out(m2_wreg_out)
	       );

   wire                                m3_regwrite_out;
   wire                                m3_zero;
   wire                                m3_overflow;
   wire [`REG_SIZE-1:0]                m3_result;
   wire [`REG_ADDR-1:0]                m3_wreg_out;

   M3 M3(
	       .clk(clk),
	       .regwrite_mult_in(m2_regwrite_out),
         .pre_m2result(m2_result),
         .pre_zero(m2_zero),
         .pre_overflow(m2_overflow),
         .wreg_in(m2_wreg_out),

         .regwrite_out(m3_regwrite_out),
         .zero(m3_zero),
         .overflow(m3_overflow),
	       .m3result(m3_result),
         .wreg_out(m2_wreg_out)
	       );

   wire                                m4_regwrite_out;
   wire                                m4_zero;
   wire                                m4_overflow;
   wire [`REG_SIZE-1:0]                m4_result;
   wire [`REG_ADDR-1:0]                m4_wreg_out;

   M4 M4(
	       .clk(clk),
	       .regwrite_mult_in(m3_regwrite_out),
         .pre_m3result(m3_result),
         .pre_zero(m3_zero),
         .pre_overflow(m3_overflow),
         .wreg_in(m3_wreg_out),

         .regwrite_out(m4_regwrite_out),
         .zero(m4_zero),
         .overflow(m4_overflow),
	       .m4result(m4_result),
         .wreg_out(m4_wreg_out)
	       );

   wire                                m5_regwrite_out;
   wire                                m5_zero;
   wire                                m5_overflow;
   wire [`REG_SIZE-1:0]                m5_result;
   wire [`REG_ADDR-1:0]                m5_wreg_out;

   M5 M5(
	       .clk(clk),
	       .regwrite_mult_in(m4_regwrite_out),
         .pre_m4result(m4_result),
         .pre_zero(m4_zero),
         .pre_overflow(m4_overflow),
         .wreg_in(m4_wreg_out),

         .regwrite_out(m5_regwrite_out),
         .zero(m5_zero),
         .overflow(m5_overflow),
	       .m5result(m5_result),
         .wreg_out(m5_wreg_out)
	       );

   //MEM STAGE
   reg [`REG_ADDR-1:0]                 dc_wreg_out;
   reg [`REG_SIZE-1:0]                 dc_wdata;
   reg                                 dc_regwrite;

   wire                                dc_do_read;
   wire                                dc_is_byte;
   wire                                dc_is_write;
   wire [`REG_SIZE-1:0]                dc_data_in;
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

   cache Dcache(
                .clk(clk),
                .reset(reset),
                .addr(ex_result),
	              .do_read(dc_do_read),
	              .is_byte(dc_is_byte),
	              .do_write(dc_is_write),
	              .data_in(dc_data_in),
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
   //ARBITER
   Arbiter Arbiter(
                   .clk(clk),
		               .reset(reset),
		               .ic_read_req(ic_read_req),
		               .ic_read_ack(ic_read_ack),
		               .ic_read_addr(ic_read_addr),
		               .ic_read_data(ic_read_data),

		               .dc_read_req(dc_read_req),
		               .dc_read_ack(dc_read_ack),
		               .dc_read_addr(dc_read_addr),
		               .dc_read_data(dc_read_data),

		               .dc_write_req(dc_write_req),
		               .dc_write_ack(dc_write_ack),
		               .dc_write_addr(dc_write_addr),
		               .dc_write_data(dc_write_data),

		               .mem_enable(mem_enable),
		               .mem_rw(mem_rw),
		               .mem_ack(mem_ack),
		               .mem_addr(mem_addr),
		               .mem_data_in(mem_data_out),
		               .mem_data_out(mem_data_in)
                   );

always @(posedge clk) begin
   dc_wreg_out <= ex_wreg_out;
   dc_regwrite <= ex_regwrite;
   dc_wdata <= dc_do_read? dc_data_out : ex_result;

end

   assign wb_wreg = dc_regwrite? dc_wreg_out : m5_wreg_out;
   assign wb_wdata = dc_regwrite? dc_memresult : m5_result;
   assign regwrite = m5_regwrite_out | dc_regwrite;

endmodule
`endif

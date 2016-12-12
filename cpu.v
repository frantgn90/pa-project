`ifndef _cpu
`define _cpu

`include "define.v"
`include "stages/fetch/pc.v"
`include "stages/exec/alu.v"
`include "stages/exec/exec1.v"
`include "stages/exec/M1.v"
`include "stages/exec/M2.v"
`include "stages/exec/M3.v"
`include "stages/exec/M4.v"
`include "stages/exec/M5.v"





module cpu(
	input wire clk,
	input wire reset
	// Memory ports
	/*output wire mem_enable,
	output wire mem_rw,
	input wire mem_ack,
	output wire [31:0]  mem_addr,
	input wire [BWDITH-1:0] mem_data_out,
	output wire [BWDITH-1:0] mem_data_in*/
);


/*module pc(
	input wire clk,
	
	input wire is_jump,
	input wire is_branch,
	input wire is_exception,
	input wire reset,
	
	input wire [31:0] pc_jump,
	input wire [31:0] pc_branch,
	input wire [31:0] old_pc,
	output reg [31:0] new_pc
	);
*/
/*
pc pc(
	.clk(clk),
	.is_jump(),
	.is_branch(),
	.is_exception(),
	.reset(reset),
	.pc_jump(),
	.pc_branch(),
	.old_pc(),
	.new_pc()
);
*/
//falta ssignar
exec1 exec1(
	.alusrc(alusrc),
	.aluop(aluop),
   .src1(src1),
   .reg2(reg2),
   .immediat(immediat),
   .old_pc(old_pc),
   .aluresult(aluresult),
   .zero(zero),
   .overflow(oveflow),
   .new_pc(new_pc),
   .dst(dst)
);

wire [`REG_SIZE-1:0] m1result;
wire m1zero;
wire m1overflow;
wire [4:0] M1_dst;

M1 M1(
   .aluop(aluop),
   .src1(src1),
   .src2(src2),
   .m1result(m1result),
   .m1zero(m1zero),
   .m1overflow(m1overflow),
   .dst(M1_dst)
   );

wire [`REG_SIZE-1:0] m2result;
wire m2zero;
wire m2overflow;
wire [4:0] M2_dst;
	 
M2 M2(
   .pre_m1result(m1result),
   .pre_zero(m1zero),
   .pre_overflow(m1overflow),
   .pre_dst(M1_dst),

   .m2result(m2result),
   .zero(m1zero),
   .overflow(m1overflow),
   .dst(M2_dst)
	);

wire [`REG_SIZE-1:0] m3result;
wire m3zero;
wire m3overflow;
wire [4:0] M3_dst;
	
M3 M3(
   .pre_m2result(m2result),
   .pre_zero(m2zero),
   .pre_overflow(m2overflow),
   .pre_dst(M2_dst),

   .m3result(m3result),
   .zero(m3zero),
   .overflow(m3overflow),
   .dst(M3_dst)
	);
	
wire [`REG_SIZE-1:0] m4result;
wire m4zero;
wire m4overflow;
wire [4:0] M4_dst;

M4 M4(
   .pre_m3result(m3result),
   .pre_zero(m3zero),
   .pre_overflow(m3overflow),
   .pre_dst(M3_dst),

   .m4result(m4result),
   .zero(m4zero),
   .overflow(m4overflow),
   .dst(M4_dst)
	);

wire [`REG_SIZE-1:0] m5result;
wire m5zero;
wire m5overflow;
wire [4:0] M5_dst;

	
M5 M5(
   .pre_m4result(m4result),
   .pre_zero(m4zero),
   .pre_overflow(m4overflow),
   .pre_dst(M4_dst),

   .m5result(m5result),
   .zero(m5zero),
   .overflow(m5overflow),
   .dst(M5_dst)
	);
endmodule
`endif

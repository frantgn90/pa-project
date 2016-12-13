`ifndef _cpu
`define _cpu

`include "define.v"
`include "stages/fetch/pc.v"
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

//WIRE TO COME FROM DECODE TO EXEC1 AND M1:
//exec1
wire						 regwrite_in;
wire						 alusrc;
wire [`REG_SIZE-1:0]  immediat;
wire [`ADDR_SIZE-1:0] old_pc;

//M1:
wire 						regwrite_mult_in;

//M1 and exec1
wire [`REG_SIZE-1:0] src1;
wire [`REG_SIZE-1:0] src2;
wire [`REG_ADDR-1:0] wreg_in;
wire [4:0]				aluop;

exec1 exec1(
	.clk(clk),
	.regwrite_in(regwrite_in),
	.alusrc(alusrc),
	.aluop(aluop),
   .src1(src1),
   .reg2(reg2),
   .immediat(immediat),
   .old_pc(old_pc),
	.wreg_in(wreg_in),
	
	.regwrite_out(regwrite_out),
   .zero(zero),
   .overflow(oveflow),
   .aluresult(aluresult),
   .pc_branch(pc_branch),
   .wreg_out(wreg_out)
);


wire regwrite_out1;
wire m1zero;
wire m1overflow;
wire [`REG_ADDR-1:0] M1wreg_out;
wire [`REG_SIZE-1:0] m1result;

M1 M1(
	.clk(clk),
	.regwrite_mult_in(regwrite_mult_in),
	.wreg_in(wreg_in),
   .aluop(aluop),
   .src1(src1),
   .src2(src2),
	
	.regwrite_out(regwrite_out1),
   .m1zero(m1zero),
   .m1overflow(m1overflow),
	.wreg_out(M1wreg_out),
	.m1result(m1result)
   );

wire regwrite_out2;
wire m2zero;
wire m2overflow;
wire [`REG_SIZE-1:0] m2result;
wire [`REG_ADDR-1:0] M2wreg_out;

M2 M2(
	.clk(clk),
	.regwrite_mult_in(regwrite_out1),
   .pre_m1result(m1result),
   .pre_zero(m1zero),
   .pre_overflow(m1overflow),
   .wreg_in(M1wreg_out),

   .regwrite_out(regwrite_out2),
   .zero(m2zero),
   .overflow(m2overflow),
	.m2result(m2result),
   .wreg_out(M2wreg_out)
	);

wire regwrite_out3;
wire m3zero;
wire m3overflow;
wire [`REG_SIZE-1:0] m3result;
wire [`REG_ADDR-1:0] M3wreg_out;

M3 M3(
	.clk(clk),
	.regwrite_mult_in(regwrite_out2),
   .pre_m2result(m2result),
   .pre_zero(m2zero),
   .pre_overflow(m2overflow),
   .wreg_in(M2wreg_out),

   .regwrite_out(regwrite_out3),
   .zero(m3zero),
   .overflow(m3overflow),
	.m3result(m3result),
   .wreg_out(M3wreg_out)
	);
	
wire regwrite_out4;
wire m4zero;
wire m4overflow;
wire [`REG_SIZE-1:0] m4result;
wire [`REG_ADDR-1:0] M4wreg_out;

M4 M4(
	.clk(clk),
	.regwrite_mult_in(regwrite_out3),
   .pre_m3result(m3result),
   .pre_zero(m3zero),
   .pre_overflow(m4overflow),
   .wreg_in(M3wreg_out),

   .regwrite_out(regwrite_out4),
   .zero(m4zero),
   .overflow(m4overflow),
	.m4result(m4result),
   .wreg_out(M4wreg_out)
	);

wire regwrite_out5;
wire m5zero;
wire m5overflow;
wire [`REG_SIZE-1:0] m5result;
wire [`REG_ADDR-1:0] M5wreg_out;

	
M5 M5(
	.clk(clk),
	.regwrite_mult_in(regwrite_out4),
   .pre_m4result(m4result),
   .pre_zero(m4zero),
   .pre_overflow(m4overflow),
   .wreg_in(M4wreg_out),

   .regwrite_out(regwrite_out5),
   .zero(m5zero),
   .overflow(m5overflow),
	.m5result(m5result),
   .wreg_out(M5wreg_out)
	);
	
endmodule
`endif

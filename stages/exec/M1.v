`ifndef _M1
`define _M1

`include "define.v"
`include "alu.v"

module M1(
          input wire                 clk,
          input wire                 reset,
          input wire                 we,
          input wire                 regwrite_mult_in, //Write Permission
          input wire [`REG_ADDR-1:0] wreg_in, //Destination Register for
          input wire [5:0]           opcode,
          input wire [5:0]           funct_code,
          input wire [`REG_SIZE-1:0] src1,
          input wire [`REG_SIZE-1:0] src2,

          output reg                 regwrite_out = 1'd0,
          output reg                 m1zero = 1'd0,
          output reg                 m1overflow = 1'd0,
          output reg [`REG_ADDR-1:0] dst_reg,
          output reg [`REG_SIZE-1:0] m1result
          );

   wire [((`REG_SIZE*2)-1):0] temp; //temporary reg for multiplication

   assign temp = src1*src2;
   
   always @(posedge clk) begin
      if (reset) begin
        regwrite_out = 1'd0;
        m1zero = 1'd0;
        m1overflow = 1'd0;
        dst_reg = {`REG_ADDR{1'd0}};
        m1result = {`REG_SIZE{1'd0}};
      end
        if (we) begin
            case (opcode)
                `OP_RTYPE: begin
                    case (funct_code)
                        `FN_MUL: begin
                            
                            m1zero <= 0;
                            m1result <= temp[`REG_SIZE-1:0];
                            if({`REG_SIZE{m1result[`REG_SIZE-1]}} != temp[((`REG_SIZE*2)-1):`REG_SIZE]) m1overflow <= 1;
                            else m1overflow <= 0;
                        end
                        default:
                        ;
                    endcase
                end
            default:
            ;
            //   `WARNING(("[M1] Unknown M1OP signal %x", aluop))
            endcase
            
            regwrite_out <= regwrite_mult_in;
            dst_reg <= wreg_in;
        end
	 end
endmodule
`endif

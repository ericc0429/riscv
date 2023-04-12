module reg_if_id
import rv32i_types::*;

// ports
(
    // inputs
    input clk,
    input rst,
    input logic load,

    input rv32i_word instr,
    input rv32i_word pc_in, 

    // outputs
    output logic [2:0] funct3,
    output logic [6:0] funct7,
    output rv32i_opcode opcode,

    output rv32i_word i_imm,
    output rv32i_word s_imm,
    output rv32i_word b_imm,
    output rv32i_word u_imm,
    output rv32i_word j_imm,
    
    output rv32i_reg rs1,
    output rv32i_reg rs2,
    output rv32i_reg rd,

    output rv32i_word pc_out
);

ir IR(
    .clk (clk),
    .rst (rst),
    .load (load),
    .rs1 (rs1),
    .rs2 (rs2),
    .rd (rd),
    .i_imm (i_imm),
    .u_imm (u_imm),
    .b_imm (b_imm),
    .s_imm (s_imm),
    .j_imm (j_imm),
    .in (instr),
    .opcode (opcode),
    .funct3 (funct3),
    .funct7 (funct7)
);

register PC (
    .clk (clk),
    .rst (rst),
    .load (load),
    .in (pc_in),
    .out (pc_out)
);

endmodule : reg_if_id

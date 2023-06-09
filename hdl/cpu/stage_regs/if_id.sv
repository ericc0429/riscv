module reg_if_id
import rv32i_types::*;
(
    input clk,
    input rst,
    input logic load,

    input rv32i_word pc_if,
    input rv32i_word instr_if,
    input rv32i_opcode opcode_if,
    input logic [2:0] funct3_if,
    input logic [6:0] funct7_if,
    input rv32i_reg_word regs_if,

    output rv32i_word pc_id,
    output rv32i_word instr_id,
    output rv32i_opcode opcode_id,
    output logic [2:0] funct3_id,
    output logic [6:0] funct7_id,
    output rv32i_reg_word regs_id
    
);

always_ff @(posedge clk)
begin
    if (rst)
    begin
        pc_id <= '0;
        instr_id <= '0;
        opcode_id <= rv32i_opcode'('0);
        funct3_id <= '0;
        funct7_id <= '0;
        regs_id <= '0;
    end

    else if (load)
    begin
        pc_id <= pc_if;
        instr_id <= instr_if;
        opcode_id <= opcode_if;
        funct3_id <= funct3_if;
        funct7_id <= funct7_if;
        regs_id <= regs_if;
    end
    
    else
    begin
        pc_id <= pc_id;
        instr_id <= instr_id;
        opcode_id <= opcode_id;
        funct3_id <= funct3_id;
        funct7_id <= funct7_id;
        regs_id <= regs_id;
    end
end

endmodule : reg_if_id

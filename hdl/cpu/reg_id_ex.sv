module reg_id_ex
import rv32i_types::*;

// ports
(
    // inputs
    input clk,
    input rst,
    input logic load,

    input rv32i_control_word ctrl_word,
    input rv32i_word pc,

    input rv32i_word rs1,
    input rv32i_word rs2,
    input rv32i_reg rd,

    input rv32i_word i_imm,
    input rv32i_word s_imm,
    input rv32i_word b_imm,
    input rv32i_word u_imm,
    input rv32i_word j_imm, 

    // outputs
    output rv32i_control_word ctrl_word_out,
    output rv32i_word pc_out,

    output rv32i_word rs1_out,
    output rv32i_word rs2_out,
    output rv32i_reg rd_out,

    output rv32i_word i_imm_out,
    output rv32i_word s_imm_out,
    output rv32i_word b_imm_out,
    output rv32i_word u_imm_out,
    output rv32i_word j_imm_out
);

// assignments
always_ff @(posedge clk)
begin
    if (rst)
    begin
        j_imm_out <= '0;
        u_imm_out <= '0;
        b_imm_out <= '0;
        s_imm_out <= '0;
        i_imm_out <= '0;

        rs2_out <= '0;
        rs1_out <= '0;
        rd_out <= '0;

        pc_out <= '0;
        ctrl_word_out <= '0;
    end

    else if (load)
    begin
        j_imm_out <= j_imm;
        u_imm_out <= u_imm;
        b_imm_out <= b_imm;
        s_imm_out <= s_imm;
        i_imm_out <= i_imm;

        rs2_out <= rs2;
        rs1_out <= rs1;
        rd_out <= rd;
        
        pc_out <= pc;
        ctrl_word_out <= ctrl_word;
    end

    else
    begin
        j_imm_out <= j_imm_out;
        u_imm_out <= u_imm_out;
        b_imm_out <= b_imm_out;
        s_imm_out <= s_imm_out;
        i_imm_out <= i_imm_out;

        rs2_out <= rs2_out;
        rs1_out <= rs1_out;
        rd_out <= rd_out;
        
        pc_out <= pc_out;
        ctrl_word_out <= ctrl_word_out;
    end
end

endmodule : reg_id_ex
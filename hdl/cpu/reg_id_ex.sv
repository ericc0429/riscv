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

    input rv32i_reg rs1_addr,
    input rv32i_reg rs2_addr,

    input rv32i_reg rd,

    input rv32i_word rs1_data,
    input rv32i_word rs2_data,

    input rv32i_word i_imm,
    input rv32i_word s_imm,
    input rv32i_word b_imm,
    input rv32i_word u_imm,
    input rv32i_word j_imm,

    // outputs
    output rv32i_control_word ctrl_word_out,
    output rv32i_word pc_out,

    output rv32i_reg rs1_out,
    output rv32i_reg rs2_out,
    output rv32i_reg rd_out,

    output rv32i_word rs1_data_out,
    output rv32i_word rs2_data_out,

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
        ctrl_word_out <= '0;
        pc_out <= '0;

        rs1_out <= '0;
        rs2_out <= '0;
        rd_out <= '0;

        rs1_data_out <= '0;
        rs2_data_out <= '0;

        i_imm_out <= '0;
        s_imm_out <= '0;
        b_imm_out <= '0;
        u_imm_out <= '0;
        j_imm_out <= '0;
    end

    else if (load)
    begin
        ctrl_word_out <= ctrl_word;
        pc_out <= pc;

        rs1_out <= rs1_addr;
        rs2_out <= rs2_addr;
        rd_out <= rd;

        rs1_data_out <= rs1_data;
        rs2_data_out <= rs2_data;

        i_imm_out <= i_imm;
        s_imm_out <= s_imm;
        b_imm_out <= b_imm;
        u_imm_out <= u_imm;
        j_imm_out <= j_imm;
    end

    else
    begin
        ctrl_word_out <= ctrl_word_out;
        pc_out <= pc_out;

        rs1_out <= rs1_out;
        rs2_out <= rs2_out;
        rd_out <= rd_out;

        rs1_data_out <= rs1_data_out;
        rs2_data_out <= rs2_data_out;

        i_imm_out <= i_imm_out;
        s_imm_out <= s_imm_out;
        b_imm_out <= b_imm_out;
        u_imm_out <= u_imm_out;
        j_imm_out <= j_imm_out;
    end
end

endmodule : reg_id_ex
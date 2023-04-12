module reg_mem_wr
import rv32i_types::*;

// ports
(
    // inputs
    input clk,
    input rst,
    input logic load,

    input rv32i_control_word ctrl_word,
    input rv32i_word pc,
    input logic br_en,

    input rv32i_reg rd,

    input rv32i_word mem_rdata,
    input rv32i_word alu_res,       // represents original address or alu result
    input logic [1:0] bit_shift,

    input rv32i_word u_imm,

    // outputs
    output rv32i_control_word ctrl_word_out,
    output rv32i_word pc_out,
    output logic br_en_out,

    output rv32i_reg rd_out,

    output rv32i_word mem_rdata_out,
    output rv32i_word alu_res_out,
    output logic [1:0] bit_shift_out,

    output rv32i_word u_imm_out
);

always_ff @(posedge clk)
begin
    if (rst)
    begin
        ctrl_word_out <= '0;
        pc_out <= '0;
        br_en_out <= '0;

        rd_out <= '0;

        mem_rdata_out <= '0;
        alu_res_out <= '0;
        bit_shift_out <= '0;

        u_imm_out <= '0;
    end

    else if (load)
    begin
        ctrl_word_out <= ctrl_word;
        pc_out <= pc;
        br_en_out <= br_en;

        rd_out <= rd;

        mem_rdata_out <= mem_rdata;
        alu_res_out <= alu_res;
        bit_shift_out <= bit_shift;

        u_imm_out <= u_imm;
    end
    
    else
    begin
        ctrl_word_out <= ctrl_word_out;
        pc_out <= pc_out;
        br_en_out <= br_en_out;

        rd_out <= rd_out;

        mem_rdata_out <= mem_rdata_out;
        alu_res_out <= alu_res_out;
        bit_shift_out <= bit_shift_out;

        u_imm_out <= u_imm_out;
    end
end

endmodule : reg_mem_wr

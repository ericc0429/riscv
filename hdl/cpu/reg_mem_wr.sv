module reg_mem_wr
import rv32i_types::*;

// ports
(
    // inputs
    input clk,
    input rst,
    input logic load,

    input rv32i_control_word ctrl_word,

    input rv32i_word mem_addr,
    input rv32i_word mem_rdata,

    input rv32i_reg rd,
    input logic br_en_in,

    input rv32i_word u_imm_in,
    input rv32i_word pc_in,

    // outputs
    output rv32i_control_word ctrl_word_out,

    output rv32i_word mem_addr_out,
    output rv32i_word mem_rdata_out,

    input logic [1:0] bit_shift_in,
    output logic [1:0] bit_shift_out,

    output rv32i_reg rd_out,
    output logic br_en_out,

    output rv32i_word u_imm_out,
    output rv32i_word pc_out,

    input rv32i_word orig_addr_in,
    output rv32i_word orig_addr_out
);

always_ff @(posedge clk)
begin
    if (rst)
    begin
        rd_out <= '0;
        mem_rdata_out <= '0;
        mem_addr_out <= '0;
        orig_addr_out <= '0;
        bit_shift_out <= '0;
        ctrl_word_out <= '0;
        br_en_out <= '0;
        u_imm_out <= '0;
        pc_out <= '0;
    end

    else if (load)
    begin
        rd_out <= rd;
        mem_rdata_out <= mem_rdata;
        orig_addr_out <= orig_addr_in;
        mem_addr_out <= mem_addr;
        bit_shift_out <= bit_shift_in;
        ctrl_word_out <= ctrl_word;
        br_en_out <= br_en_in;
        u_imm_out <= u_imm_in;
        pc_out <= pc_in;
    end
    
    else
    begin
        rd_out <= rd_out;
        mem_rdata_out <= mem_rdata_out;
        orig_addr_out <= orig_addr_out;
        mem_addr_out <= mem_addr_out;
        bit_shift_out <= bit_shift_out;
        ctrl_word_out <= ctrl_word_out;
        br_en_out <= br_en_out;
        u_imm_out <= u_imm_out;
        pc_out <= pc_out;
    end
end

endmodule : reg_mem_wr

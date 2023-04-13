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

    input rv32i_reg rs1_addr,
    input rv32i_reg rs2_addr,
    input rv32i_reg rd,

    input logic cur_stall_in,

    input rv32i_word rs1_data,
    input rv32i_word rs2_data,

    input rv32i_word mem_rdata,
    input rv32i_word alu_res,       // represents original address or alu result
    input logic [1:0] bit_shift,

    input rv32i_word u_imm,

    input rv32i_word write_data,

    input logic [3:0] wmask_in,
    input logic [3:0] rmask_in,
    input logic trap_in,

    // outputs
    output rv32i_control_word ctrl_word_out,
    output rv32i_word pc_out,
    output logic br_en_out,

    output rv32i_reg rs1_addr_out,
    output rv32i_reg rs2_addr_out,
    output rv32i_reg rd_out,

    output logic cur_stall_out,

    output rv32i_word rs1_data_out,
    output rv32i_word rs2_data_out,

    output rv32i_word mem_rdata_out,
    output rv32i_word alu_res_out,
    output logic [1:0] bit_shift_out,

    output rv32i_word u_imm_out,

    output rv32i_word write_data_out,

    output logic [3:0] wmask_out,
    output logic [3:0] rmask_out,
    output logic trap_out
);

always_ff @(posedge clk)
begin
    if (rst)
    begin
        ctrl_word_out <= '0;
        pc_out <= '0;
        br_en_out <= '0;

        rs1_addr_out <= '0;
        rs2_addr_out <= '0;
        rd_out <= '0;
        cur_stall_out <= '0;

        rs1_data_out <= '0;
        rs2_data_out <= '0;

        mem_rdata_out <= '0;
        alu_res_out <= '0;
        bit_shift_out <= '0;

        u_imm_out <= '0;

        write_data_out <= '0;

        wmask_out <= '0;
        rmask_out <= '0;
        trap_out  <= '0;
    end

    else if (load)
    begin
        ctrl_word_out <= ctrl_word;
        pc_out <= pc;
        br_en_out <= br_en;

        rs1_addr_out <= rs1_addr;
        rs2_addr_out <= rs2_addr;
        rd_out <= rd;
        cur_stall_out <= cur_stall_in;

        rs1_data_out <= rs1_data;
        rs2_data_out <= rs2_data;

        mem_rdata_out <= mem_rdata;
        alu_res_out <= alu_res;
        bit_shift_out <= bit_shift;

        u_imm_out <= u_imm;

        write_data_out <= write_data;

        wmask_out <= wmask_in;
        rmask_out <= rmask_in;
        trap_out  <= trap_in;
    end
    
    else
    begin
        ctrl_word_out <= ctrl_word_out;
        pc_out <= pc_out;
        br_en_out <= br_en_out;

        rs1_addr_out <= rs1_addr_out;
        rs2_addr_out <= rs2_addr_out;
        rd_out <= rd_out;

        cur_stall_out <= cur_stall_out;

        rs1_data_out <= rs1_data_out;
        rs2_data_out <= rs2_data_out;

        mem_rdata_out <= mem_rdata_out;
        alu_res_out <= alu_res_out;
        bit_shift_out <= bit_shift_out;

        u_imm_out <= u_imm_out;

        write_data_out <= write_data_out;

        wmask_out <= wmask_out;
        rmask_out <= rmask_out;
        trap_out  <= trap_out;
    end
end

endmodule : reg_mem_wr

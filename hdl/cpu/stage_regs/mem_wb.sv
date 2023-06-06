module reg_mem_wb
import rv32i_types::*;
(
    input clk,
    input rst,
    input logic load,

    input rv32i_pc_word pc_mem,
    input rv32i_control_word ctrl_mem,
    input rv32i_reg_word regs_mem,
    input logic br_en_mem,

    input rv32i_word mem_rdata,
    input rv32i_word alu_res,       // represents original address or alu result
    input logic [1:0] bit_shift,

    input rv32i_word write_data,
    input logic [3:0] wmask_in,
    input logic [3:0] rmask_in,
    input logic trap_in,
    input logic cur_stall_in,

    output rv32i_pc_word pc_wb,
    output rv32i_control_word ctrl_wb,
    output rv32i_reg_word regs_wb,
    output logic br_en_wb,

    output rv32i_word mem_rdata_out,
    output rv32i_word alu_res_out,
    output logic [1:0] bit_shift_out,

    output rv32i_word write_data_out,
    output logic [3:0] wmask_out,
    output logic [3:0] rmask_out,
    output logic trap_out,
    output logic cur_stall_out
);

always_ff @(posedge clk)
begin
    if (rst)
    begin
        pc_wb <= '0;
        ctrl_wb <= '0;
        regs_wb <= '0;
        br_en_wb <= '0;

        mem_rdata_out <= '0;
        alu_res_out <= '0;
        bit_shift_out <= '0;

        write_data_out <= '0;
        wmask_out <= '0;
        rmask_out <= '0;
        trap_out  <= '0;
        cur_stall_out <= '0;
    end

    else if (load)
    begin
        pc_wb <= pc_mem;
        ctrl_wb <= ctrl_mem;
        regs_wb <= regs_mem;
        br_en_wb <= br_en_mem;

        mem_rdata_out <= mem_rdata;
        alu_res_out <= alu_res;
        bit_shift_out <= bit_shift;

        write_data_out <= write_data;
        wmask_out <= wmask_in;
        rmask_out <= rmask_in;
        trap_out  <= trap_in;
        cur_stall_out <= cur_stall_in;
    end
    
    else
    begin
        pc_wb <= pc_wb;
        ctrl_wb <= ctrl_wb;
        regs_wb <= regs_wb;
        br_en_wb <= br_en_wb;

        mem_rdata_out <= mem_rdata_out;
        alu_res_out <= alu_res_out;
        bit_shift_out <= bit_shift_out;

        write_data_out <= write_data_out;
        wmask_out <= wmask_out;
        rmask_out <= rmask_out;
        trap_out  <= trap_out;
        cur_stall_out <= cur_stall_out;
    end
end

endmodule : reg_mem_wb

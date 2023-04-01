module reg_ex_mem
import rv32i_types::*;

// ports
(
    input clk,
    input rst,
    input logic load,

    input rv32i_control_word ctrl_word,
    output rv32i_control_word ctrl_word_out,

    input rv32i_reg rd_in,
    output rv32i_reg rd_out,

    input rv32i_word pc_in,
    output rv32i_word pc_out,

    input rv32i_word addr_in,
    output rv32i_word addr_out,
    output logic [1:0] bit_shift,

    input logic br_en_in,
    output logic br_en_out,

    input rv32i_word write_data_in,
    output rv32i_word write_data_out,

    output logic [3:0] mem_byte_enable,

    input rv32i_word u_imm_in,
    output rv32i_word u_imm_out
);

logic [1:0] last2addr;
assign last2addr = addr_out[1:0];
logic [3:0] wmask;

always_comb
begin
    case(ctrl_word_out.funct3)
        sw: wmask = 4'b1111;
        
        sh: wmask = 4'b0011 << last2addr;
        
        sb: wmask = 4'b0001 << last2addr;
    endcase
end

always_ff @(posedge clk)
begin
    if (rst)
    begin
        write_data_out <= '0;
        addr_out <= '0;
        bit_shift <= '0;
        br_en_out <= '0;
        pc_out <= '0;
        rd_out <= '0;
        ctrl_word_out <= '0;
        mem_byte_enable <= '0;
        u_imm_out <= '0;
    end

    else if (load)
    begin
        write_data_out <= write_data_in;
        addr_out <= {addr_in[31:2],2'b00};
        bit_shift <= addr_in[1:0];
        br_en_out <= br_en_in;
        pc_out <= pc_in;
        rd_out <= rd_in;
        ctrl_word_out <= ctrl_word;
        mem_byte_enable <= wmask;
        u_imm_out <= u_imm_in;
    end
    
    else
    begin
        write_data_out <= write_data_out;
        addr_out <= addr_out;
        bit_shift <= bit_shift;
        br_en_out <= br_en_out;
        pc_out <= pc_out;
        rd_out <= rd_out;
        ctrl_word_out <= ctrl_word_out;
        mem_byte_enable <= mem_byte_enable;
        u_imm_out <= u_imm_out;
    end
end

endmodule : reg_ex_mem
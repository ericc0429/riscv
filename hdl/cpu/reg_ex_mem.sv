module reg_ex_mem
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

    input rv32i_word alu_res,               // represents new pc, address, or operation result
    input rv32i_word write_data,
    input rv32i_word u_imm,

    // outputs
    output rv32i_control_word ctrl_word_out,
    output rv32i_word pc_out,
    output logic br_en_out,

    output rv32i_reg rd_out,

    output rv32i_word addr_aligned,         // aligned to 4B (i.e. last 2 bits = 00)
    output rv32i_word alu_res_out,          // represents original addr or operation result
    output logic [1:0] bit_shift,
    output logic [3:0] mem_byte_enable,

    output rv32i_word write_data_out,
    output rv32i_word u_imm_out
);

rv32i_word addr;      // purely for clarity
assign addr = alu_res;

logic [1:0] last2addr;
assign last2addr = addr[1:0];
logic [3:0] wmask;

// determine byte enable based on last 2 bits of address
always_comb
begin
    case(ctrl_word.funct3)
        sw: wmask = 4'b1111;
        
        sh: wmask = 4'b0011 << last2addr;
        
        sb: wmask = 4'b0001 << last2addr;

        default: wmask = 4'b1111;
    endcase
end

always_ff @(posedge clk)
begin
    if (rst)
    begin
        ctrl_word_out <= '0;
        pc_out <= '0;
        br_en_out <= '0;

        rd_out <= '0;

        addr_aligned <= '0;
        alu_res_out <= '0;
        bit_shift <= '0;
        mem_byte_enable <= '0;

        write_data_out <= '0;
        u_imm_out <= '0;
    end

    else if (load)
    begin
        ctrl_word_out <= ctrl_word;
        pc_out <= pc;
        br_en_out <= br_en;

        rd_out <= rd;

        addr_aligned <= {addr[31:2],2'b00};
        alu_res_out <= alu_res;
        bit_shift <= addr[1:0];
        mem_byte_enable <= wmask;

        write_data_out <= write_data;
        u_imm_out <= u_imm;
    end
    
    else
    begin
        ctrl_word_out <= ctrl_word_out;
        pc_out <= pc_out;
        br_en_out <= br_en_out;

        rd_out <= rd_out;

        addr_aligned <= addr_aligned;
        alu_res_out <= alu_res_out;
        bit_shift <= bit_shift;
        mem_byte_enable <= mem_byte_enable;

        write_data_out <= write_data_out;
        u_imm_out <= u_imm_out;
    end
end

endmodule : reg_ex_mem

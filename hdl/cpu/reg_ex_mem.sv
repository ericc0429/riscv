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

    input rv32i_reg rs1_addr,
    input rv32i_reg rs2_addr,
    input rv32i_reg rd,
    input logic cur_stall_in,

    input rv32i_word rs1_data,
    input rv32i_word rs2_data,

    input rv32i_word alu_res,               // represents new pc, address, or operation result
    input rv32i_word write_data,
    input rv32i_word u_imm,

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

    output rv32i_word addr_aligned,         // aligned to 4B (i.e. last 2 bits = 00)
    output rv32i_word alu_res_out,          // represents original addr or operation result
    output logic [1:0] bit_shift,
    output logic [3:0] mem_byte_enable,

    output rv32i_word write_data_out,
    output rv32i_word u_imm_out,
    output logic [3:0] rmask_out,
    output logic trap_out

);

rv32i_word addr;      // purely for clarity
assign addr = alu_res;

logic [1:0] last2addr;
assign last2addr = addr[1:0];
logic [3:0] wmask, rmask;
logic trap;

// determine byte enable based on last 2 bits of address
always_comb
begin : trap_check
    trap = '0;
    rmask = '0;
    wmask = '0;

    case(ctrl_word.opcode)

        op_lui, op_auipc, op_imm, op_reg, op_jal, op_jalr:;

        op_br: begin
            case (branch_funct3_t'(ctrl_word.funct3))
                beq, bne, blt, bge, bltu, bgeu:;
                default: trap = '1;
            endcase
        end

        op_store: begin
            case(store_funct3_t'(ctrl_word.funct3))
                sw: wmask = 4'b1111;
                
                sh: wmask = 4'b0011 << last2addr;
                
                sb: wmask = 4'b0001 << last2addr;

                default: begin
                    trap = '1;
                end
            endcase
        end

        op_load: begin
            case(load_funct3_t'(ctrl_word.funct3))

                lw: rmask = 4'b1111;
                lh, lhu: rmask = 4'b0011 << last2addr /* Modify for MP1 Final */ ;
                lb, lbu: rmask = 4'b0001 << last2addr /* Modify for MP1 Final */ ;
                default: begin
                    trap = '1;
                end
            endcase
        end

        default: trap = '1;
    endcase
end

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

        addr_aligned <= '0;
        alu_res_out <= '0;
        bit_shift <= '0;
        mem_byte_enable <= '0;
        rmask_out <= '0;
        trap_out <= '0;

        write_data_out <= '0;
        u_imm_out <= '0;
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

        addr_aligned <= {addr[31:2],2'b00};
        alu_res_out <= alu_res;
        bit_shift <= addr[1:0];
        mem_byte_enable <= wmask;
        rmask_out <= wmask;
        trap_out <= trap;

        write_data_out <= write_data;
        u_imm_out <= u_imm;
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

        addr_aligned <= addr_aligned;
        alu_res_out <= alu_res_out;
        bit_shift <= bit_shift;
        mem_byte_enable <= mem_byte_enable;
        rmask_out <= rmask_out;
        trap_out <= trap_out;

        write_data_out <= write_data_out;
        u_imm_out <= u_imm_out;
    end
end

endmodule : reg_ex_mem

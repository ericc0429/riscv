module reg_ex_mem
import rv32i_types::*;
(
    input clk,
    input rst,
    input logic load,

    input rv32i_word pc_ex,
    input rv32i_control_word ctrl_ex,
    input rv32i_reg_word regs_ex,
    input logic br_en_ex,

    input rv32i_word alu_res,               // represents new pc, address, or operation result
    input rv32i_word write_data,

    input logic fwd_flag_ex,
    input rv32i_word regmux_ex,

    output rv32i_word           pc_mem,
    output rv32i_control_word   ctrl_mem,
    output rv32i_reg_word       regs_mem,
    output logic                br_en_mem,

    output rv32i_word           alu_res_out,          // represents original addr or operation result
    output rv32i_word           write_data_out,

    output logic                fwd_flag_mem,
    output rv32i_word           regmux_mem

    /* output rv32i_word addr_aligned,         // aligned to 4B (i.e. last 2 bits = 00)
    output logic [1:0] bit_shift,
    output logic [3:0] mem_byte_enable,
    output logic [3:0] rmask_out,
    output logic trap_out, */
    

);

/* rv32i_word addr;      // purely for clarity
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

    case(ctrl_ex.opcode)

        op_lui, op_auipc, op_imm, op_reg, op_jal, op_jalr:;

        op_br: begin
            case (branch_funct3_t'(ctrl_ex.funct3))
                beq, bne, blt, bge, bltu, bgeu:;
                default: trap = '1;
            endcase
        end

        op_store: begin
            case(store_funct3_t'(ctrl_ex.funct3))
                sw: wmask = 4'b1111;
                
                sh: wmask = 4'b0011 << last2addr;
                
                sb: wmask = 4'b0001 << last2addr;

                default: begin
                    trap = '1;
                end
            endcase
        end

        op_load: begin
            case(load_funct3_t'(ctrl_ex.funct3))

                lw: rmask = 4'b1111;
                lh, lhu: rmask = 4'b0011 << last2addr;
                lb, lbu: rmask = 4'b0001 << last2addr;
                default: begin
                    trap = '1;
                end
            endcase
        end

        default: trap = '1;
    endcase
end */

rv32i_word pc_d;
rv32i_control_word ctrl_d;
rv32i_reg_word regs_d;
logic br_en_d;

rv32i_word alu_res_d;
rv32i_word write_data_d;

logic fwd_flag_d;
rv32i_word regmux_d;

/* always_ff @(posedge clk)
begin
    if (rst)
    begin
        pc_mem <= '0;
        ctrl_mem <= '0;
        regs_mem <= '0;
        br_en_mem <= '0;

        alu_res_out <= '0;
        write_data_out <= '0;

        addr_aligned <= '0;
        bit_shift <= '0;
        mem_byte_enable <= '0;
        rmask_out <= '0;
        trap_out <= '0;

        fwd_flag_mem <= '0;
        regmux_mem <= '0;
    end

    else if (load)
    begin
        pc_mem <= pc_ex;
        ctrl_mem <= ctrl_ex;
        regs_mem <= regs_ex;
        br_en_mem <= br_en_ex;

        addr_aligned <= {addr[31:2],2'b00};
        alu_res_out <= alu_res;
        bit_shift <= addr[1:0];
        mem_byte_enable <= wmask;

        rmask_out <= rmask;
        trap_out <= trap;
        write_data_out <= write_data;

        fwd_flag_mem <= fwd_flag_ex;
        regmux_mem <= regmux_ex;
    end
    
    else
    begin
        pc_mem <= pc_mem;
        ctrl_mem <= ctrl_mem;
        regs_mem <= regs_mem;
        br_en_mem <= br_en_mem;

        addr_aligned <= addr_aligned;
        alu_res_out <= alu_res_out;
        bit_shift <= bit_shift;
        mem_byte_enable <= mem_byte_enable;

        rmask_out <= rmask_out;
        trap_out <= trap_out;
        write_data_out <= write_data_out;

        fwd_flag_mem <= fwd_flag_mem;
        regmux_mem <= regmux_mem;
    end
end */

always_ff @(posedge clk)
begin
    if (rst)
    begin
        pc_d <= '0;
        ctrl_d <= '0;
        regs_d <= '0;
        br_en_d <= '0;

        alu_res_d <= '0;
        write_data_d <= '0;

        fwd_flag_d <= '0;
        regmux_d <= '0;
    end

    else if (load)
    begin
        pc_d <= pc_ex;
        ctrl_d <= ctrl_ex;
        regs_d <= regs_ex;
        br_en_d <= br_en_ex;

        alu_res_d <= alu_res;
        write_data_d <= write_data;

        fwd_flag_d <= fwd_flag_ex;
        regmux_d <= regmux_ex;
    end
    
    else
    begin
        pc_d <= pc_d;
        ctrl_d <= ctrl_d;
        regs_d <= regs_d;
        br_en_d <= br_en_d;

        alu_res_d <= alu_res_d;
        write_data_d <= write_data_d;

        fwd_flag_d <= fwd_flag_d;
        regmux_d <= regmux_d;
    end
end

always_comb
begin
    pc_mem = pc_d;
    ctrl_mem = ctrl_d;
    regs_mem = regs_d;
    br_en_mem = br_en_d;
    alu_res_out  = alu_res_d;
    write_data_out = write_data_d;
    fwd_flag_mem = fwd_flag_d;
    regmux_mem = regmux_d;
end

endmodule : reg_ex_mem

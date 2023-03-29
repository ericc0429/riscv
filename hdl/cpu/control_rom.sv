import rv32i_types::*;

module control_rom (
    input rv32i_opcode opcode;
    input logic [2:0] funct3;
    input logic [6:0] funct7;
    output rv32i_control_word ctrl;
);

/* ======== Function Definitions ======== */
function void set_defaults();
    ctrl.opcode = opcode;
    ctrl.aluop = alu_ops'(funct3);
    ctrl.regfilemux_sel = regfilemux::alu_out;
    ctrl.load_regfile = 1'b0;
    ctrl.pcmux_sel = pcmux::pc_plus4;
    ctrl.load_pc = 1'b0;
    ctrl.alumux1_sel = alumux::rs1_out;
    ctrl.alumux2_sel = alumux::i_imm;
    ctrl.cmpop = branch_funct3_t'(funct3);
    ctrl.cmpmux_sel = cmpmux::rs2_out;
    ctrl.rdmux_sel = 1'b0;
    ctrl.mem_read = 1'b0;
    ctrl.mem_write = 1'b0;

    /* Unused signals from MP3 cpu control
    load_ir = 1'b0;
    load_mar = 1'b0;
    load_mdr = 1'b0;
    load_data_out = 1'b0;
    marmux_sel = marmux::pc_out;
    mem_byte_enable = 4'b1111;
    */
endfunction

function void loadRegfile(regfilemux::regfilemux_sel_t sel);
    load_regfile = 1'b1;
    regfilemux_sel = sel;
endfunction

function void setALU(alumux::alumux1_sel_t sel1, alumux::alumux2_sel_t sel2, logic setop, alu_ops op);
    /* Student code here */
    if (setop)
        begin
            ctrl.aluop = op; // else default value
            // We also have to set the alumux selector values
            ctrl.alumux1_sel = sel1;
            ctrl.alumux2_sel = sel2;
        end
endfunction

function automatic void setCMP(cmpmux::cmpmux_sel_t sel, logic setop, branch_funct3_t op);
    if (setop)
        begin
            ctrl.cmpop = op;
            ctrl.cmpmux_sel = sel;
        end
endfunction

/* ======== End Function Definitions ======== */

always_comb
begin
    /* Default assignments */
    set_defaults();

    /* Assign control signals based on opcode */
    case(opcode)

        op_lui:
        begin
            loadRegfile(regfilemux::u_imm);
        end

        op_auipc:
        begin
            setALU(alumux::pc_out, alumux::u_imm, 1'b1, alu_add);
            loadRegfile(regfilemux::alu_out);
        end

        op_jal:
        begin
            setALU(alumux::pc_out, alumux::j_imm, 1'b1, alu_add); // pc + j_imm
            loadRegfile(regfilemux::pc_plus4); // Write address of next instruction into rd
        end

        op_jalr:
        begin
            setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_add);
            loadRegfile(regfilemux::pc_plus4);
        end

        op_br:
        begin
            setALU(alumux::pc_out, alumux::b_imm, 1'b1, alu_add);
        end

        op_load:
        begin
            ctrl.mem_read = 1'b1;
            setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_add);
            case (load_funct3_t'(funct3))
                lb: loadRegfile(regfilemux::lb);
                lh: loadRegfile(regfilemux::lh);
                lw: loadRegfile(regfilemux::lw);
                lbu: loadRegfile(regfilemux::lbu);
                lhu: loadRegfile(regfilemux::lhu);
            endcase
        end

        op_store:
        begin
            setALU(alumux::rs1_out, alumux::s_imm, 1'b1, alu_add);
            mem_write = 1'b1;
        end

        op_imm:
        begin
            case (arith_funct3_t'(funct3))
                // add: // addi

                // sll: // slli

                slt: // slti
                begin
                    loadRegfile(regfilemux::br_en);
                    setCMP(cmpmux::i_imm, 1'b1, blt);
                end

                sltu: // sltiu
                begin
                    loadRegfile(regfilemux::br_en);
                    setCMP(cmpmux::i_imm, 1'b1, bltu);
                end

                // axor: // xori

                sr: // Check bit30 to determine if logical (0) or arithmetic (1)
                begin
                    loadRegfile(regfilemux::alu_out);
                    if (funct7[5]) setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_sra);
                    else setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_srl);
                end

                // aor: // ori

                // aand: // andi

                default:
                begin
                    loadRegfile(regfilemux::alu_out);
                    setALU(alumux::rs1_out, alumux::i_imm, 1'b1, alu_ops'(funct3));
                end
            endcase
        end

        op_reg:
        begin
            case (arith_funct3)
                // add/sub -- check bit30 for sub if op_reg opcode
                add:
                begin
                    loadRegfile(regfilemux::alu_out);
                    if (funct7[5]) setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_sub); // sub
                    else setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_add); // add
                end

                // sll:

                slt:
                begin
                    loadRegfile(regfilemux::br_en);
                    setCMP(cmpmux::rs2_out, 1'b1, blt);
                end

                sltu:
                begin
                    loadRegfile(regfilemux::br_en);
                    setCMP(cmpmux::rs2_out, 1'b1, bltu);
                end

                // axor: // xor

                sr: // Check bit30 to determine if logical (0) or arithmetic (1)
                begin
                    loadRegfile(regfilemux::alu_out);
                    if (funct7[5]) setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_sra);
                    else setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_srl);
                end

                // or:
                // and:

                default:
                begin
                    loadRegfile(regfilemux::alu_out);
                    setALU(alumux::rs1_out, alumux::rs2_out, 1'b1, alu_ops'(funct3));
                end
            endcase
        end

        default: begin
            ctrl = 0;   /* Unknown opcode, set control word to zero */
        end
    endcase

end

endmodule : control_rom
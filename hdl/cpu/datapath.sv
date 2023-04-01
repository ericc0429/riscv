module datapath
import rv32i_types::*;

(
    // Port Signals
    input clk,
    input rst,

    input rv32i_word    instr_mem_rdata,
    input rv32i_word    data_mem_rdata,

    output logic        instr_read,

    output logic 		data_read,
    output logic 		data_write,

    output rv32i_word   instr_mem_address,
    output rv32i_word   data_mem_address,

    output rv32i_word   data_mem_wdata,
    output logic [3:0]  data_mbe
);
// data_read
assign instr_read = '1;

/* ================ INTERNAL SIGNALS ================ */

// Default load signal
logic load = 1'b1;

// PC signals
rv32i_word pcmux_out;
rv32i_word pc_out;      // PC in IF stage
rv32i_word pc_id;       // PC in ID stage
rv32i_word pc_ex_in;    // Pre ALU_JUMP
rv32i_word pc_mem;
rv32i_word pc_wr;

// regfile signals
rv32i_word regfilemux_out;

rv32i_reg rs1;
rv32i_reg rs2;

rv32i_word rs1_out;
rv32i_word rs2_out;

rv32i_word rs1_ex;
rv32i_word rs2_ex;

rv32i_reg rd_id;
rv32i_reg rd_ex;
rv32i_reg rd_mem;
rv32i_reg rd_wr;

// Immediates
rv32i_word i_imm;   // inputs to id/ex
rv32i_word s_imm;
rv32i_word b_imm;
rv32i_word u_imm;
rv32i_word j_imm;

rv32i_word i_imm_ex;    // outputs from id/ex
rv32i_word s_imm_ex;
rv32i_word b_imm_ex;
rv32i_word u_imm_ex;
rv32i_word j_imm_ex;

rv32i_word u_imm_mem;

rv32i_word u_imm_wr;

// ALU signals
rv32i_word alupc_out;
rv32i_word alujump_out;

// rv32i_word alumux1_out;
rv32i_word alumux1_out;
rv32i_word alumux2_out;
rv32i_word alu_out;

// CMP signals
rv32i_word cmpmux_out;

// Control ROM signals 
rv32i_control_word ctrl;
rv32i_control_word ctrl_ex;
rv32i_control_word ctrl_mem;
rv32i_control_word ctrl_wr;

rv32i_opcode opcode;
logic [2:0] funct3;
logic [6:0] funct7;

// BR_en
logic br_en;
logic br_en_mem;
logic br_en_wr;

rv32i_word rdata_wr;
rv32i_word data_addr_wr;

rv32i_word wdata_out;

assign data_read = ctrl_mem.mem_read;
assign data_write = ctrl_mem.mem_write;
assign instr_mem_address = pc_out;

/* ================ REGISTERS ================ */

pc_register PC (
    .clk,
    .rst,
    .load,
    .in     (pcmux_out),
    .out    (pc_out)
);

regfile regfile (
    .clk,
    .rst,
    .load   (ctrl.load_regfile),
    .in     (regfilemux_out),
    .src_a  (rs1),
    .src_b  (rs2),
    .dest   (rd_wr),
    .reg_a  (rs1_out),
    .reg_b  (rs2_out)
);

/* ================ ALU / CMP ================ */

alu ALU_PC (
    .aluop  (alu_add),
    .a      (pc_out),
    .b      (32'd4),
    .f      (alupc_out)
);

// alu ALU_JUMP (
    // .aluop  (alu_add),
    // .a      (pc_ex_in),
    // .b      ({b_imm_ex[29:0], 2'b00}),
    // .f      (pc_ex_out)
// );

alu ALU (
    .aluop  (ctrl_ex.aluop),
    .a      (alumux1_out),
    .b      (alumux2_out),
    .f      (alu_out)
);

cmp CMP (
    .cmpop  (ctrl_ex.cmpop),
    .a      (rs1_ex),
    .b      (cmpmux_out),
    .f      (br_en)
);

/* ================ CONTROL ROM ================ */

control_rom CONTROL_ROM (.*);

/* ================ STAGE REGISTERS ================ */
reg_if_id IF_ID (
    // inputs
    .clk,
    .rst,
    .load,

    .instr(instr_mem_rdata),
    .pc_in(pc_out),

    // outputs
    .opcode,
    .funct3,
    .funct7,
 
    .pc_out(pc_id),
    
    .rs1,
    .rs2,
    .rd(rd_id),
    
    .i_imm,
    .s_imm,
    .b_imm,
    .u_imm,
    .j_imm   
);

reg_id_ex ID_EX (
    .clk,
    .rst,
    .load,

    .ctrl_word(ctrl),
    .pc(pc_id),

    .rs1(rs1_out),
    .rs2(rs2_out),
    .rd(rd_id),

    .i_imm,
    .s_imm,
    .b_imm,
    .u_imm,
    .j_imm, 

    // outputs
    .ctrl_word_out(ctrl_ex),
    .pc_out(pc_ex_in),
    // .b_imm_shifted,

    .rs1_out(rs1_ex),
    .rs2_out(rs2_ex),
    .rd_out(rd_ex),

    .i_imm_out(i_imm_ex),
    .s_imm_out(s_imm_ex),
    .b_imm_out(b_imm_ex),
    .u_imm_out(u_imm_ex),
    .j_imm_out(j_imm_ex)
);

reg_ex_mem EX_MEM (
    .clk,
    .rst,
    .load,

    .ctrl_word(ctrl_ex),
    .ctrl_word_out(ctrl_mem),

    .rd_in(rd_ex),
    .rd_out(rd_mem),

    .pc_in(alu_out),
    .pc_out(pc_mem),

    .addr_in(alu_out),
    .addr_out(data_mem_address),

    .br_en_in(br_en),
    .br_en_out(br_en_mem),

    .write_data_in(alumux2_out),
    .write_data_out(wdata_out),

    .mem_byte_enable(data_mbe),

    .u_imm_in(u_imm_ex),
    .u_imm_out(u_imm_mem)
);

reg_mem_wr MEM_WR (
    .clk,
    .rst,
    .load,

    .ctrl_word(ctrl_mem),
    .mem_addr(data_mem_address),
    .mem_rdata(data_mem_rdata),
    .rd(rd_mem),
    .br_en_in(br_en_mem),
    .u_imm_in(u_imm_mem),
    .pc_in(pc_mem),

    .ctrl_word_out(ctrl_wr),
    .mem_addr_out(data_addr_wr),
    .mem_rdata_out(rdata_wr),
    .rd_out(rd_wr),
    .br_en_out(br_en_wr),
    .u_imm_out(u_imm_wr),
    .pc_out(pc_wr)
);

rv32i_word shifted_val;
assign shifted_val = rdata_wr >> (8 * data_addr_wr[1:0]);


assign data_mem_wdata = wdata_out << (8 * data_addr_wr[1:0]);

logic pc_sel;
assign pc_sel = br_en & ctrl_ex.br_sel;

/* ================ MUXES ================ */
always_comb begin : MUXES

    // CMP MUX
    unique case (ctrl_ex.cmpmux_sel)
        cmpmux::rs2_out: cmpmux_out = rs2_ex;
        cmpmux::i_imm:   cmpmux_out = i_imm_ex;
    endcase

    // PC MUX
    unique case (pc_sel)
        1'b0: pcmux_out = alupc_out;
        1'b1: pcmux_out = alu_out;
        default: pcmux_out = alupc_out;
    endcase

    // ALU MUX #1
    unique case (ctrl_ex.alumux1_sel)
        alumux::rs1_out: alumux1_out = rs1_ex;
        alumux::pc_out: alumux1_out = pc_ex_in;
        default: alumux1_out = pc_ex_in;
    endcase
    
    // ALU MUX #2
    unique case (ctrl_ex.alumux2_sel)
        alumux::i_imm:      alumux2_out = i_imm_ex;
        alumux::u_imm:      alumux2_out = u_imm_ex;
        alumux::b_imm:      alumux2_out = b_imm_ex;
        alumux::s_imm:      alumux2_out = s_imm_ex;
        alumux::j_imm:      alumux2_out = j_imm_ex;
        alumux::rs2_out:    alumux2_out = rs2_ex;
        // alumux::b_imm_sl2:  alumux2_out = {b_imm_ex[29:0], 2'b00};
    endcase

    // REGFILE MUX
    unique case (ctrl_wr.regfilemux_sel)
        regfilemux::alu_out: regfilemux_out = data_addr_wr;
        regfilemux::br_en:   regfilemux_out = {31'b0, br_en_wr};
        regfilemux::u_imm:   regfilemux_out = u_imm_wr;
        regfilemux::lw:      regfilemux_out = rdata_wr;

        regfilemux::pc_plus4: regfilemux_out = pc_wr + 4;
        regfilemux::lb:     regfilemux_out = 32'(signed'(shifted_val[7:0]));
        regfilemux::lbu:    regfilemux_out = 32'(unsigned'(shifted_val[7:0]));
        regfilemux::lh:     regfilemux_out = 32'(signed'(shifted_val[15:0]));
        regfilemux::lhu:    regfilemux_out = 32'(unsigned'(shifted_val[15:0]));
    endcase
    
end

endmodule : datapath

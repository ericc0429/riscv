module datapath
import rv32i_types::*;
(
    input clk,
    input rst,

    // read data
    input rv32i_word    instr_mem_rdata,
    input rv32i_word    data_mem_rdata,

    // cache response
    input logic         instr_mem_resp,
    input logic         data_mem_resp,

    // control signals
    output logic        instr_read,
    output logic 		data_read,
    output logic 		data_write,

    // addresses
    output rv32i_word   instr_mem_address,
    output rv32i_word   data_mem_address,

    // write data
    output rv32i_word   data_mem_wdata,
    output logic [3:0]  data_mbe
);

/* ================ INTERNAL SIGNALS ================ */

rv32i_word addr_aligned_wr;

/* === MASKS AND TRAP === */
logic [3:0] wmask_wb;
logic [3:0] rmask_mem;
logic [3:0] rmask_wb;

logic trap_mem;
logic trap_wb;

/* === Stage Register Loads === */
logic load_if_id;
logic load_id_ex;
logic load_ex_mem;
logic load_mem_wb;

stall_debug sd; // Debug stalling

logic cur_stall;
logic cur_stall_wb;

/* === PC signals === */
logic load_pc;
rv32i_word pcmux_out;
rv32i_word pc_if;
rv32i_word pc_id;
rv32i_word pc_ex;
rv32i_word pc_wdata_ex;
rv32i_word pc_mem;
rv32i_word pc_wb;

/* === ir signals === */
logic load_ir;
assign load_ir = instr_mem_resp;

/* === Control ROM signals === */ 
rv32i_control_word ctrl;
rv32i_control_word ctrl_id;
rv32i_control_word ctrl_ex;
rv32i_control_word ctrl_mem;
rv32i_control_word ctrl_wb;
logic ctrlmux_sel;

/* === Reg Word === */
rv32i_reg_word regs_if;
rv32i_reg_word regs_id;
rv32i_reg_word regs_id_data;
rv32i_reg_word regs_ex;
rv32i_reg_word regs_ex_fwd;
rv32i_reg_word regs_mem;
rv32i_reg_word regs_wb;
/* === regfile signals === */
rv32i_word regfilemux_out;

// IF signals
rv32i_word instr_if;
rv32i_opcode opcode_if;
logic [2:0] funct3_if;
logic [6:0] funct7_if;

// ID signals
rv32i_word instr_id;
rv32i_opcode opcode;
logic [2:0] funct3;
logic [6:0] funct7;

// register data
rv32i_word rs1_data;    // Regfile output into regs_id_data
rv32i_word rs2_data;
rv32i_word rs1_data_final;      // either chose fwd val or curr reg val
rv32i_word rs2_data_final;

rv32i_word u_imm_mem;
rv32i_word u_imm_wb;

/* === ALU signals === */
rv32i_word alupc_out;
rv32i_word alu_out;
rv32i_word alu_out_mem;
rv32i_word alu_out_wb;
rv32i_word alumux1_out;
rv32i_word alumux2_out;

/* === CMP signals === */
rv32i_word cmpmux_out;

/* === branching === */
logic br_en;
logic br_en_mem;
logic br_en_wb;

/* === BIT SHIFTING/ALIGNMENT === */
rv32i_word wdata_out_final;
assign data_mem_wdata = wdata_out_final << (8 * alu_out_mem[1:0]);

logic pc_sel;   // br_en that only gets set if we are indeed branching
assign pc_sel = (br_en || (ctrl_ex.opcode == op_jal || ctrl_ex.opcode == op_jalr)) & ctrl_ex.br_sel;

/* ================ STALL LOGIC ================ */
logic stall_pipeline;
logic instr_stall;
logic data_stall;

assign instr_stall = !instr_mem_resp && (instr_read);
assign data_stall = !data_mem_resp && (data_read || data_write);
assign stall_pipeline = instr_stall || data_stall;


/* === branch prediction === */
logic flush;    // for branch prediction
// static branch-not-taken predictor logic -- if branch taken, flush IF/ID & ID/EX
assign flush = pc_sel && !stall_pipeline; 


/* === forwarding unit === */
logic rs1_fwdflag;
logic rs2_fwdflag;
logic rs2_fwdflag_mem1b;    // rs2_fwdflag_mem1behind
logic rs2_fwdflag_mem2b;    // rs2_fwdflag_mem2behind
logic rs2_2b_n1b;           // rs2_2behind_now1behind
rv32i_word regmux_mem;
rv32i_word rs1_fwd;
rv32i_word rs2_fwd;

/* ================ MEMORY SIGNALS ================ */
rv32i_word rdata_wb;
rv32i_word data_addr_wb;
rv32i_word orig_addr;
rv32i_word orig_addr_wb;
logic [1:0] bit_shift_mem;
logic [1:0] bit_shift_wb;

rv32i_word wdata_out;
rv32i_word wdata_wb;

assign instr_read = '1;     // just cp1 (always fetching)
assign data_read = ctrl_mem.mem_read;
assign data_write = ctrl_mem.mem_write;
assign instr_mem_address = pc_if;

/* ================ PREDICTOR ================ */
logic load_brp;
assign load_brp = (opcode_if == op_jal || opcode_if == op_br);
rv32i_brp_word brp_if;
rv32i_brp_word brp_id;
rv32i_brp_word brp_ex;

/*
brp PREDICTOR (
    .clk,
    .rst,
    .load   (load_brp),
    // Inputs
    .regs_if,

    // Outputs
    .brp_if
);
*/

/* ================ REGISTERS ================ */

pc_register PC (
    .clk,
    .rst,
    .load   (load_pc),
    .in     (pcmux_out),
    .out    (pc_if)
);

regfile regfile (
    .clk,
    .rst,
    .load   (ctrl_wb.load_regfile),
    .in     (regfilemux_out),
    .use_rd (ctrl_wb.use_rd),
    .src_a  (regs_id.rs1),
    .src_b  (regs_id.rs2),
    .dest   (regs_wb.rd),
    .reg_a  (rs1_data),
    .reg_b  (rs2_data)
);

ir IR(
    .clk    (clk),
    .rst    (rst || flush),
    .load   (load_if_id),
    .in     (instr_mem_rdata),

    .rs1    (regs_if.rs1),
    .rs2    (regs_if.rs2),
    .rd     (regs_if.rd),
    .i_imm  (regs_if.i_imm),
    .s_imm  (regs_if.s_imm),
    .b_imm  (regs_if.b_imm),
    .u_imm  (regs_if.u_imm),
    .j_imm  (regs_if.j_imm),
    .opcode (opcode_if),
    .funct3 (funct3_if),
    .funct7 (funct7_if)
);

/* ================ ALU / CMP ================ */

alu ALU_PC (
    .aluop  (alu_add),
    .a      (pc_if),
    .b      (32'd4),
    .f      (alupc_out)
);

alu ALU (
    .aluop  (ctrl_ex.aluop),
    .a      (alumux1_out),
    .b      (alumux2_out),
    .f      (alu_out)
);

cmp CMP (
    .cmpop  (ctrl_ex.cmpop),
    .a      (regs_ex_fwd.rs1_data/* rs1_data_final */),
    .b      (cmpmux_out),
    .f      (br_en)
);

/* ================ CONTROL ROM ================ */

control_rom CONTROL_ROM (.*);

/* ================ STAGE REGISTERS ================ */
reg_if_id IF_ID (
    .clk,
    .rst        (rst || flush),
    .load       (load_if_id),

    // Inputs
    .pc_if,
    .instr_if   (instr_mem_rdata),
    .opcode_if,
    .funct3_if,
    .funct7_if,
    .regs_if,

    // Outputs
    .pc_id,
    .instr_id,
    .opcode_id  (opcode),
    .funct3_id  (funct3),
    .funct7_id  (funct7),
    .regs_id
);

reg_id_ex ID_EX (
    .clk,
    .rst        (rst || flush),
    .load       (load_id_ex),

    // Inputs
    .pc_id,
    .ctrl_id,
    .regs_id    (regs_id_data),

    // Outputs
    .pc_ex,
    .ctrl_ex,
    .regs_ex
);

reg_ex_mem EX_MEM (
    .clk,
    .rst,
    .load       (load_ex_mem),

    // Inputs
    .pc_ex      (pc_wdata_ex),
    .ctrl_ex,
    .regs_ex    (regs_ex_fwd),
    .br_en_ex   (br_en),

    .alu_res        (alu_out),
    .write_data     (regs_ex.rs2_data),

    .fwd_flag_ex    (rs2_fwdflag_mem2b),
    .regmux_ex      (regfilemux_out),

    // Outputs
    .pc_mem,
    .ctrl_mem,
    .regs_mem,
    .br_en_mem,

    .addr_aligned   (data_mem_address),
    .alu_res_out    (alu_out_mem),
    .bit_shift      (bit_shift_mem),
    .mem_byte_enable(data_mbe),

    .rmask_out      (rmask_mem),
    .trap_out       (trap_mem),
    .write_data_out (wdata_out),

    .fwd_flag_mem   (rs2_2b_n1b),
    .regmux_mem
);

reg_mem_wb MEM_WB (
    .clk,
    .rst,
    .load           (load_mem_wb),

    .pc_mem,
    .ctrl_mem,
    .regs_mem,
    .br_en_mem,

    .mem_rdata      (data_mem_rdata),
    .alu_res        (alu_out_mem),
    .bit_shift      (bit_shift_mem),

    .write_data     (data_mem_wdata),
    .wmask_in       (data_mbe),
    .rmask_in       (rmask_mem),
    .trap_in        (trap_mem),
    .cur_stall_in   (cur_stall),
    .addr_aligned_in (data_mem_address),

    // outputs
    .pc_wb,
    .ctrl_wb,
    .regs_wb,
    .br_en_wb,

    .mem_rdata_out  (rdata_wb),
    .alu_res_out    (alu_out_wb),
    .bit_shift_out  (bit_shift_wb),

    .write_data_out (wdata_wb),
    .wmask_out      (wmask_wb),
    .rmask_out      (rmask_wb),
    .trap_out       (trap_wb),
    .addr_aligned_out (addr_aligned_wr),
    .cur_stall_out  (cur_stall_wb)
);

/* ================ FORWARDING UNIT ================ */
fwd_unit FWD_UNIT (
    // inputs
    .rd_wb  (regs_wb.rd),
    .rd_mem (regs_mem.rd),
    .rs1_ex (regs_ex.rs1),
    .rs2_ex (regs_ex.rs2),
    .rs2_mem(regs_mem.rs2),

    .use_rd_mem     (ctrl_mem.use_rd),
    .use_rd_wb      (ctrl_wb.use_rd),

    .rdata_wb,
    .alu_out_mem,     // alu output being propagated
    .regfilemux_out,

    .ctrl_ex,
    .ctrl_mem_opcode    (ctrl_mem.opcode),
    .data_mem_rdata,

    // outputs
    .rs1_fwdflag,
    .rs2_fwdflag,
    .rs2_fwdflag_mem1b,
    .rs2_fwdflag_mem2b,
    .rs1_fwd,
    .rs2_fwd
);

/* ================ HAZARD DETECTION UNIT ================ */
hdu HDU (
    .ctrl_ex,
    .rs1_id     (regs_id.rs1),
    .rs2_id     (regs_id.rs2),
    .rd_ex      (regs_ex.rd),
    .stall_pipeline,
    .ctrlmux_sel,
    .load_if_id,
    .load_id_ex,
    .load_ex_mem,
    .load_mem_wb,
    .load_pc,
    .cur_stall,

    .sd
);

always_comb begin : ID_REG_UPDATE
    // Passes values from regs_id to regs_id_data.
    // Allows us to have the regfile update the rs1_data and rs2_data fields.
    regs_id_data.rs1        = regs_id.rs1;
    regs_id_data.rs2        = regs_id.rs2;
    regs_id_data.rd         = regs_id.rd;
    regs_id_data.rs1_data   = rs1_data;
    regs_id_data.rs2_data   = rs2_data;
    regs_id_data.i_imm      = regs_id.i_imm;
    regs_id_data.s_imm      = regs_id.s_imm;
    regs_id_data.b_imm      = regs_id.b_imm;
    regs_id_data.u_imm      = regs_id.u_imm;
    regs_id_data.j_imm      = regs_id.j_imm;
end

always_comb begin : EX_REG_UPDATE
    regs_ex_fwd.rs1        = regs_ex.rs1;
    regs_ex_fwd.rs2        = regs_ex.rs2;
    regs_ex_fwd.rd         = regs_ex.rd;
    regs_ex_fwd.rs1_data   = rs1_data_final;
    regs_ex_fwd.rs2_data   = rs2_data_final;
    regs_ex_fwd.i_imm      = regs_ex.i_imm;
    regs_ex_fwd.s_imm      = regs_ex.s_imm;
    regs_ex_fwd.b_imm      = regs_ex.b_imm;
    regs_ex_fwd.u_imm      = regs_ex.u_imm;
    regs_ex_fwd.j_imm      = regs_ex.j_imm;
end

/* ================ MUXES ================ */
always_comb begin : MUXES

    // CTRL MUX -- either sends along control word from control ROM or 0
    unique case (ctrlmux_sel)
        1'b0: ctrl_id = ctrl;
        1'b1: ctrl_id = '0;
        default: ctrl_id = ctrl;
    endcase

    // FWD MUX #1 (choose forwarded or current value for rs1)
    unique case (rs1_fwdflag)
        1'b1: rs1_data_final = rs1_fwd;
        default: rs1_data_final = regs_ex.rs1_data;
    endcase

    // FWD MUX #2 (choose forwarded or current value for rs2)
    unique case (rs2_fwdflag)
        1'b1: rs2_data_final = rs2_fwd;
        default: rs2_data_final = regs_ex.rs2_data;
    endcase

    // FWD MUX #3 (choose which write data to send -> mem)
    unique case ({rs2_2b_n1b, rs2_fwdflag_mem1b})
        2'b01: wdata_out_final = regfilemux_out;
        2'b10: wdata_out_final = regmux_mem;
        default: wdata_out_final = wdata_out;
    endcase

    // CMP MUX #1
    unique case (ctrl_ex.cmpmux_sel)
        cmpmux::rs2_out: cmpmux_out = regs_ex_fwd.rs2_data;
        default: cmpmux_out = regs_ex_fwd.i_imm;
    endcase

    // PC MUX
    unique case (pc_sel)
        1'b1:
        begin
            pcmux_out = alu_out;
            pc_wdata_ex = alu_out;
        end
        default:
        begin
            pcmux_out = alupc_out;
            pc_wdata_ex = ctrl_ex.pc+4;
        end
    endcase

    // ALU MUX #1
    unique case (ctrl_ex.alumux1_sel)
        alumux::rs1_out: alumux1_out = regs_ex_fwd.rs1_data;
        alumux::pc_out: alumux1_out = pc_ex;
        default: alumux1_out = pc_ex;
    endcase
    
    // ALU MUX #2
    unique case (ctrl_ex.alumux2_sel)
        alumux::i_imm:      alumux2_out = regs_ex_fwd.i_imm;
        alumux::s_imm:      alumux2_out = regs_ex_fwd.s_imm;
        alumux::b_imm:      alumux2_out = regs_ex_fwd.b_imm;
        alumux::u_imm:      alumux2_out = regs_ex_fwd.u_imm;
        alumux::j_imm:      alumux2_out = regs_ex_fwd.j_imm;
        alumux::rs2_out:    alumux2_out = regs_ex_fwd.rs2_data;
        default:            alumux2_out = regs_ex_fwd.i_imm;
    endcase

    // REGFILE MUX
    unique case (ctrl_wb.regfilemux_sel)
        regfilemux::alu_out: regfilemux_out = alu_out_wb;
        regfilemux::br_en:   regfilemux_out = {31'b0, br_en_wb};
        regfilemux::u_imm:   regfilemux_out = regs_wb.u_imm;
        regfilemux::lw:      regfilemux_out = rdata_wb;

        regfilemux::pc_plus4: regfilemux_out = ctrl_wb.pc + 4;
        regfilemux::lb: begin
            case(bit_shift_wb)
                2'b00: regfilemux_out = {{24{rdata_wb[7]}},rdata_wb[7:0]};
                2'b01: regfilemux_out = {{24{rdata_wb[15]}},rdata_wb[15:8]};
                2'b10: regfilemux_out = {{24{rdata_wb[23]}},rdata_wb[23:16]};
                2'b11: regfilemux_out = {{24{rdata_wb[31]}},rdata_wb[31:24]};
                default: regfilemux_out = {{24{rdata_wb[7]}},rdata_wb[7:0]};
            endcase
        end
        regfilemux::lbu: begin
            case(bit_shift_wb)
                2'b00: regfilemux_out = {24'd0,rdata_wb[7:0]};
                2'b01: regfilemux_out = {24'd0,rdata_wb[15:8]};
                2'b10: regfilemux_out = {24'd0,rdata_wb[23:16]};
                2'b11: regfilemux_out = {24'd0,rdata_wb[31:24]};
                default: regfilemux_out = {24'd0,rdata_wb[7:0]};
            endcase
        end
        regfilemux::lh:  begin
            case(bit_shift_wb)
                2'b00: regfilemux_out = {{16{rdata_wb[15]}},rdata_wb[15:0]};
                2'b10: regfilemux_out = {{16{rdata_wb[31]}},rdata_wb[31:16]};
                default: regfilemux_out = {{16{rdata_wb[15]}},rdata_wb[15:0]};
            endcase
        end
        regfilemux::lhu: begin
            case(bit_shift_wb)
                2'b00: regfilemux_out = {16'd0,rdata_wb[15:0]};
                2'b10: regfilemux_out = {16'd0,rdata_wb[31:16]};
                default: regfilemux_out = {16'd0,rdata_wb[15:0]};
            endcase
        end

        default: regfilemux_out = alu_out;
    endcase
    
end

endmodule : datapath

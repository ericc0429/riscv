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

/* === Stage Register Loads === */
logic load_if_id;
logic load_id_ex;
logic load_ex_mem;
logic load_mem_wr;

stall_debug sd; // Debug stalling

// assign load_ex_mem = 1'b1;
// assign load_mem_wr = 1'b1;

/* === PC signals === */
logic load_pc;
// assign load_pc = 1'b1; // Comment out later
rv32i_word pcmux_out;
rv32i_word pc_if;
rv32i_word pc_id;
rv32i_word pc_ex;
rv32i_word pc_mem;
rv32i_word pc_wr;

/* === regfile signals === */
rv32i_word regfilemux_out;

// register indexes
rv32i_reg rs1;
rv32i_reg rs2;
rv32i_reg rs1_ex;
rv32i_reg rs2_ex;

rv32i_reg rd_id;
rv32i_reg rd_ex;
rv32i_reg rd_mem;
rv32i_reg rd_wr;

// register data
rv32i_word rs1_data;
rv32i_word rs2_data;
rv32i_word rs1_data_ex;
rv32i_word rs2_data_ex;
rv32i_word rs1_data_final;      // either chose fwd val or curr reg val
rv32i_word rs2_data_final;

/* === immediates === */
rv32i_word i_imm;
rv32i_word s_imm;
rv32i_word b_imm;
rv32i_word u_imm;
rv32i_word j_imm;

rv32i_word i_imm_ex;
rv32i_word s_imm_ex;
rv32i_word b_imm_ex;
rv32i_word u_imm_ex;
rv32i_word j_imm_ex;

rv32i_word u_imm_mem;
rv32i_word u_imm_wr;

/* === ALU signals === */
rv32i_word alupc_out;
rv32i_word alu_out;
rv32i_word alu_out_mem;
rv32i_word alu_out_wr;
rv32i_word alumux1_out;
rv32i_word alumux2_out;

/* === CMP signals === */
rv32i_word cmpmux_out;

/* === Control ROM signals === */ 
rv32i_control_word ctrl;
rv32i_control_word ctrl_id;
rv32i_control_word ctrl_ex;
rv32i_control_word ctrl_mem;
rv32i_control_word ctrl_wr;
logic ctrlmux_sel;

rv32i_opcode opcode;
logic [2:0] funct3;
logic [6:0] funct7;

/* === branching === */
logic br_en;
logic br_en_mem;
logic br_en_wr;

/* === BIT SHIFTING/ALIGNMENT === */
rv32i_word wdata_out_final;
assign data_mem_wdata = wdata_out_final << (8 * alu_out_mem[1:0]);

logic pc_sel;   // br_en that only gets set if we are indeed branching
assign pc_sel = br_en & ctrl_ex.br_sel;

/* === branch prediction === */
logic flush;    // for branch prediction
// static branch-not-taken predictor logic -- if branch taken, flush IF/ID & ID/EX
assign flush = pc_sel; 


/* === forwarding unit === */
logic rs1_fwdflag;
logic rs2_fwdflag;
logic rs2_fwdflag_mem;
rv32i_word rs1_fwd;
rv32i_word rs2_fwd;

/* ================ MEMORY SIGNALS ================ */
rv32i_word rdata_wr;
rv32i_word data_addr_wr;
rv32i_word orig_addr;
rv32i_word orig_addr_wr;
logic [1:0] bit_shift_mem;
logic [1:0] bit_shift_wr;

rv32i_word wdata_out;

assign instr_read = '1;     // just cp1 (always fetching)
assign data_read = ctrl_mem.mem_read;
assign data_write = ctrl_mem.mem_write;
assign instr_mem_address = pc_if;

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
    .load   (ctrl_wr.load_regfile),
    .in     (regfilemux_out),
    .src_a  (rs1),
    .src_b  (rs2),
    .dest   (rd_wr),
    .reg_a  (rs1_data),
    .reg_b  (rs2_data)
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
    .a      (rs1_data_final),
    .b      (cmpmux_out),
    .f      (br_en)
);

/* ================ CONTROL ROM ================ */

control_rom CONTROL_ROM (.*);

/* ================ STAGE REGISTERS ================ */
reg_if_id IF_ID (
    // inputs
    .clk,
    .rst    (rst || flush),
    .load   (load_if_id),

    .instr  (instr_mem_rdata),
    .pc_in  (pc_if),

    // outputs
    .opcode,
    .funct3,
    .funct7,
 
    .pc_out (pc_id),
    
    .rs1,
    .rs2,
    .rd     (rd_id),
    
    .i_imm,
    .s_imm,
    .b_imm,
    .u_imm,
    .j_imm   
);

reg_id_ex ID_EX (
    .clk,
    .rst        (rst || flush),
    .load       (load_id_ex),

    .ctrl_word  (ctrl_id),
    .pc         (pc_id),

    .rs1,                       // reg indexes
    .rs2,
    .rd         (rd_id),

    .rs1_data,                  // reg data
    .rs2_data,

    .i_imm,
    .s_imm,
    .b_imm,
    .u_imm,
    .j_imm, 

    // outputs
    .ctrl_word_out  (ctrl_ex),
    .pc_out         (pc_ex),

    .rs1_out        (rs1_ex),
    .rs2_out        (rs2_ex),
    .rd_out         (rd_ex),

    .rs1_data_out   (rs1_data_ex),
    .rs2_data_out   (rs2_data_ex),

    .i_imm_out  (i_imm_ex),
    .s_imm_out  (s_imm_ex),
    .b_imm_out  (b_imm_ex),
    .u_imm_out  (u_imm_ex),
    .j_imm_out  (j_imm_ex)
);

reg_ex_mem EX_MEM (
    .clk,
    .rst,
    .load           (load_ex_mem),

    .ctrl_word      (ctrl_ex),
    .pc             (alu_out),
    .br_en,

    .rd             (rd_ex),

    .alu_res        (alu_out),
    .write_data     (rs2_data_ex),
    .u_imm          (u_imm_ex),

    // outputs
    .ctrl_word_out  (ctrl_mem),
    .pc_out         (pc_mem),
    .br_en_out      (br_en_mem),

    .rd_out         (rd_mem),

    .addr_aligned   (data_mem_address),
    .alu_res_out    (alu_out_mem),
    .bit_shift      (bit_shift_mem),
    .mem_byte_enable(data_mbe),

    .write_data_out (wdata_out),
    .u_imm_out      (u_imm_mem)
);

reg_mem_wr MEM_WR (
    .clk,
    .rst,
    .load           (load_mem_wr),

    .ctrl_word      (ctrl_mem),
    .pc             (pc_mem),
    .br_en          (br_en_mem),

    .rd             (rd_mem),
    
    .mem_rdata      (data_mem_rdata),
    .alu_res        (alu_out_mem),
    .bit_shift      (bit_shift_mem),

    .u_imm          (u_imm_mem),

    // outputs
    .ctrl_word_out  (ctrl_wr),
    .pc_out         (pc_wr),
    .br_en_out      (br_en_wr),

    .rd_out         (rd_wr),

    .mem_rdata_out  (rdata_wr),
    .alu_res_out    (alu_out_wr),
    .bit_shift_out  (bit_shift_wr),

    .u_imm_out      (u_imm_wr)
);

/* ================ FORWARDING UNIT ================ */
fwd_unit FWD_UNIT (
    // inputs
    .rd_wr,
    .rd_mem,
    .rs1             (rs1_ex),
    .rs2             (rs2_ex),

    .rdata_wr,
    .alu_out_mem,     // alu output being propagated
    .regfilemux_out,

    .ctrl_ex,
    .ctrl_mem_opcode    (ctrl_mem.opcode),
    .data_mem_rdata,

    // outputs
    .rs1_fwdflag,
    .rs2_fwdflag,
    .rs2_fwdflag_mem,
    .rs1_fwd,
    .rs2_fwd
);

/* ================ STALL LOGIC ================ */
logic stall_pipeline;
logic instr_stall;
logic data_stall;

assign instr_stall = !instr_mem_resp && (instr_read);
assign data_stall = !data_mem_resp && (data_read || data_write);
assign stall_pipeline = instr_stall || data_stall;

/* ================ HAZARD DETECTION UNIT ================ */
hazard_detection_unit HDU (
    .ctrl_ex,
    .rs1,
    .rs2,
    .stall_pipeline,
    .rd_ex,
    .ctrlmux_sel,
    .load_if_id,
    .load_id_ex,
    .load_ex_mem,
    .load_mem_wr,
    .load_pc,

    .sd
);

/* ================ MUXES ================ */
always_comb begin : MUXES

    // CTRL MUX -- either sends along control word from control ROM or 0
    unique case (ctrlmux_sel)
        '0: ctrl_id = ctrl;
        '1: ctrl_id = '0;
        default: ctrl_id = ctrl;
    endcase

    // FWD MUX #1 (choose forwarded or current value for rs1)
    unique case (rs1_fwdflag)
        '0: rs1_data_final = rs1_data_ex;
        '1: rs1_data_final = rs1_fwd;
        default: rs1_data_final = rs1_data_ex;
    endcase

    // FWD MUX #2 (choose forwarded or current value for rs2)
    unique case (rs2_fwdflag)
        '0: rs2_data_final = rs2_data_ex;
        '1: rs2_data_final = rs2_fwd;
        default: rs2_data_final = rs2_data_ex;
    endcase

    // FWD MUX #3 (choose which write data to send -> mem)
    unique case (rs2_fwdflag_mem)
        '0: wdata_out_final = wdata_out; 
        '1: wdata_out_final = regfilemux_out;
        default: wdata_out_final = wdata_out;
    endcase

    // CMP MUX #1
    unique case (ctrl_ex.cmpmux_sel)
        cmpmux::rs2_out: cmpmux_out = rs2_data_final;
        cmpmux::i_imm:   cmpmux_out = i_imm_ex;
        default: cmpmux_out = i_imm_ex;
    endcase

    // PC MUX
    unique case (pc_sel)
        1'b0: pcmux_out = alupc_out;
        1'b1: pcmux_out = alu_out;
        default: pcmux_out = alupc_out;
    endcase

    // ALU MUX #1
    unique case (ctrl_ex.alumux1_sel)
        alumux::rs1_out: alumux1_out = rs1_data_final;
        alumux::pc_out: alumux1_out = pc_ex;
        default: alumux1_out = pc_ex;
    endcase
    
    // ALU MUX #2
    unique case (ctrl_ex.alumux2_sel)
        alumux::i_imm:      alumux2_out = i_imm_ex;
        alumux::u_imm:      alumux2_out = u_imm_ex;
        alumux::b_imm:      alumux2_out = b_imm_ex;
        alumux::s_imm:      alumux2_out = s_imm_ex;
        alumux::j_imm:      alumux2_out = j_imm_ex;
        alumux::rs2_out:    alumux2_out = rs2_data_final;
        default: alumux2_out = i_imm_ex;
    endcase

    // REGFILE MUX
    unique case (ctrl_wr.regfilemux_sel)
        regfilemux::alu_out: regfilemux_out = alu_out_wr;
        regfilemux::br_en:   regfilemux_out = {31'b0, br_en_wr};
        regfilemux::u_imm:   regfilemux_out = u_imm_wr;
        regfilemux::lw:      regfilemux_out = rdata_wr;

        regfilemux::pc_plus4: regfilemux_out = pc_wr + 4;
        regfilemux::lb: begin
            case(bit_shift_wr)
                2'b00: regfilemux_out = {{24{rdata_wr[7]}},rdata_wr[7:0]};
                2'b01: regfilemux_out = {{24{rdata_wr[15]}},rdata_wr[15:8]};
                2'b10: regfilemux_out = {{24{rdata_wr[23]}},rdata_wr[23:16]};
                2'b11: regfilemux_out = {{24{rdata_wr[31]}},rdata_wr[31:24]};
                default: regfilemux_out = {{24{rdata_wr[7]}},rdata_wr[7:0]};
            endcase
        end
        regfilemux::lbu: begin
            case(bit_shift_wr)
                2'b00: regfilemux_out = {24'd0,rdata_wr[7:0]};
                2'b01: regfilemux_out = {24'd0,rdata_wr[15:8]};
                2'b10: regfilemux_out = {24'd0,rdata_wr[23:16]};
                2'b11: regfilemux_out = {24'd0,rdata_wr[31:24]};
                default: regfilemux_out = {24'd0,rdata_wr[7:0]};
            endcase
        end
        regfilemux::lh:  begin
            case(bit_shift_wr)
                2'b00: regfilemux_out = {{16{rdata_wr[15]}},rdata_wr[15:0]};
                2'b10: regfilemux_out = {{16{rdata_wr[31]}},rdata_wr[31:16]};
                default: regfilemux_out = {{16{rdata_wr[15]}},rdata_wr[15:0]};
            endcase
        end
        regfilemux::lhu: begin
            case(bit_shift_wr)
                2'b00: regfilemux_out = {16'd0,rdata_wr[15:0]};
                2'b10: regfilemux_out = {16'd0,rdata_wr[31:16]};
                default: regfilemux_out = {16'd0,rdata_wr[15:0]};
            endcase
        end

        default: regfilemux_out = alu_out;
    endcase
    
end

endmodule : datapath

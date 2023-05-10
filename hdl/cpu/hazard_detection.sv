module hdu
import rv32i_types::*;
(
    input rv32i_control_word ctrl_id,
    input rv32i_control_word ctrl_ex,
    input rv32i_reg_word regs_if,
    input rv32i_reg_word regs_id,
    input rv32i_reg_word regs_ex,
    input logic stall_pipeline,
    output logic ctrlmux_sel,
    output logic load_if_id,
    output logic load_id_ex,
    output logic load_ex_mem,
    output logic load_mem_wb,
    output logic load_pc,
    output logic cur_stall,
    output stall_debug sd   // Stall debug
);


function void set_defaults();
    ctrlmux_sel = '0;
    load_if_id = '1;
    load_id_ex = '1;
    load_ex_mem = '1;
    load_mem_wb = '1;
    load_pc = '1;
    sd = no_stall;
    cur_stall = '0;
endfunction

function void stall(); // Read after load stall
    ctrlmux_sel = '1;
    load_if_id = '0;
    load_pc = '0;
    sd = read_after_load;
endfunction

/* function void brp_stall(); // Branching immediately after modifying one of its sources
    ctrlmux_sel = '1;
    load_if_id = '0;
    load_pc = '0;
endfunction */

function void mem_stall ();
    load_pc     = '0;
    load_if_id  = '0;
    load_id_ex  = '0;
    load_ex_mem = '0;
    load_mem_wb = '0;
    sd = mem_delay_stall;
    cur_stall = '1;
endfunction

always_comb
begin
    set_defaults();
    
    if ((regs_ex.rd != '0)
        && ((regs_ex.rd == regs_id.rs1) || (regs_ex.rd == regs_id.rs2))
        && (ctrl_ex.opcode == op_load || ctrl_ex.opcode == op_lui))
    begin
        stall();
    end
    /* if ((regs_id.rd != '0)
        && ((regs_id.rd == regs_if.rs1) || (regs_id.rd == regs_if.rs2))
        && ((ctrl_id.opcode == op_load || ctrl_id.opcode == op_lui)))
    begin
        stall();
    end */
    if (stall_pipeline)
    begin
        mem_stall();
    end
end

endmodule : hdu

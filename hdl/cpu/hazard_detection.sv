module hdu
import rv32i_types::*;
(
    input rv32i_control_word ctrl_ex,
    input rv32i_reg rs1_id,
    input rv32i_reg rs2_id,
    input rv32i_reg rd_ex,
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
    
    if((rd_ex != '0) && ((rd_ex == rs1_id) || (rd_ex == rs2_id)) && (ctrl_ex.opcode == op_load))
    begin
        stall();
    end
    if (stall_pipeline)
    begin
        mem_stall();
    end
end

endmodule : hdu

module reg_id_ex
import rv32i_types::*;
(
    input clk,
    input rst,
    input logic load,

    input rv32i_pc_word pc_id,
    input rv32i_control_word ctrl_id,
    input rv32i_reg_word regs_id,
    input rv32i_brp_word brp_id,

    output rv32i_pc_word pc_ex,
    output rv32i_control_word ctrl_ex,
    output rv32i_reg_word regs_ex,
    output rv32i_brp_word brp_ex
);

// assignments
always_ff @(posedge clk)
begin
    if (rst)
    begin
        pc_ex <= '0;
        ctrl_ex <= '0;
        regs_ex <= '0;
        brp_ex <= '0;
    end

    else if (load)
    begin
        pc_ex <= pc_id;
        ctrl_ex <= ctrl_id;
        regs_ex <= regs_id;
        brp_ex <= brp_id;
    end

    else
    begin
        pc_ex <= pc_ex;
        ctrl_ex <= ctrl_ex;
        regs_ex <= regs_ex;
        brp_ex <= brp_ex;
    end
end

endmodule : reg_id_ex
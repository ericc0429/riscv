module reg_id_ex
import rv32i_types::*;
(
    input clk,
    input rst,
    input logic load,

    input rv32i_word pc_id,
    input rv32i_control_word ctrl_id,
    input rv32i_reg_word regs_id,

    output rv32i_word pc_ex,
    output rv32i_control_word ctrl_ex,
    output rv32i_reg_word regs_ex
);

// assignments
always_ff @(posedge clk)
begin
    if (rst)
    begin
        pc_ex <= '0;
        ctrl_ex <= '0;
        regs_ex <= '0;
    end

    else if (load)
    begin
        pc_ex <= pc_id;
        ctrl_ex <= ctrl_id;
        regs_ex <= regs_id;
    end

    else
    begin
        pc_ex <= pc_ex;
        ctrl_ex <= ctrl_ex;
        regs_ex <= regs_ex;
    end
end

endmodule : reg_id_ex
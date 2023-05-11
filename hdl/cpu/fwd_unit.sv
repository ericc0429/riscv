module fwd_unit
import rv32i_types::*;
(
    // inputs
    input rv32i_reg rd_wb,
    input rv32i_reg rd_mem,
    input rv32i_reg rs1_ex,
    input rv32i_reg rs2_ex,
    input rv32i_reg rs2_mem,

    input logic use_rd_mem,
    input logic use_rd_wb,

    input rv32i_word rdata_wb,
    input rv32i_word alu_out_mem,     // alu output being propagated
    input rv32i_word regfilemux_out,

    input rv32i_control_word ctrl_ex,
    input rv32i_opcode ctrl_mem_opcode,
    input rv32i_word data_mem_rdata,

    // outputs
    output logic rs1_fwdflag,
    output logic rs2_fwdflag,
    output logic rs2_fwdflag_mem1b,
    output logic rs2_fwdflag_mem2b,
    output rv32i_word rs1_fwd,
    output rv32i_word rs2_fwd
);

// see if we stalled (have a bubble)
logic fwd_data;

always_comb
begin : read_after_load
    if (ctrl_ex == '0 && ctrl_mem_opcode == op_load)    // stall occurring
        fwd_data = '1;
    else
        fwd_data = '0;
end

always_comb
begin : rs1_forwarding
    rs1_fwdflag = '0;
    rs1_fwd = '0;

    if (rs1_ex != '0)      //check for hazard
    begin
        if(rs1_ex == rd_mem && use_rd_mem)     // rs1 changes in mem stage (priority)
        begin
            rs1_fwd = alu_out_mem;
            rs1_fwdflag  = '1; 
        end
        else if(rs1_ex == rd_wb && use_rd_wb)  // rs1 changes in wr stage
        begin
            if (fwd_data)
                rs1_fwd = rdata_wb;
            else
                rs1_fwd = regfilemux_out;

            rs1_fwdflag = '1;
        end
    end
end

always_comb
begin : rs2_forwarding
    rs2_fwdflag = '0;
    rs2_fwd = '0;

    if (rs2_ex != '0)
    begin
        if(rs2_ex == rd_mem && use_rd_mem)     // rs2 changes in the mem stage (priority)
        begin
            rs2_fwd = alu_out_mem;
            rs2_fwdflag = '1;
        end
        else if(rs2_ex == rd_wb && use_rd_wb)  // rs2 changes in the wr stage
        begin
            if (fwd_data) rs2_fwd = rdata_wb;
            else rs2_fwd = regfilemux_out;
            rs2_fwdflag = '1;
        end
    end
end

always_comb
begin : rs2_forwarding_mem
    rs2_fwdflag_mem1b = '0;
    rs2_fwdflag_mem2b = '0;

    if (rd_wb != '0)
    begin
        if (rd_wb == rs2_mem && use_rd_wb) begin
            rs2_fwdflag_mem1b = '1;
        end

        if (rd_wb == rs2_ex && use_rd_wb) begin
            rs2_fwdflag_mem2b = '1;
        end
    end
end

endmodule : fwd_unit

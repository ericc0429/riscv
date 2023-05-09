module fwd_unit
import rv32i_types::*;
(
    // inputs
    input rv32i_reg rd_wb,
    input rv32i_reg rd_mem,
    input rv32i_reg rs1,
    input rv32i_reg rs2,

    input rv32i_word rdata_wb,
    input rv32i_word alu_out_mem,     // alu output being propagated
    input rv32i_word regfilemux_out,

    input rv32i_control_word ctrl_ex,
    input rv32i_opcode ctrl_mem_opcode,
    input rv32i_word data_mem_rdata,

    // outputs
    output logic rs1_fwdflag,
    output logic rs2_fwdflag,
    output logic rs2_fwdflag_mem,
    output rv32i_word rs1_fwd,
    output rv32i_word rs2_fwd
);

// see if we stalled (have a bubble)
logic fwd_data;
always_comb begin : read_after_load
    if (ctrl_ex == '0 && ctrl_mem_opcode == op_load)    // stall occurring
        fwd_data = '1;
    else
        fwd_data = '0;
end

always_comb begin : rs1_forwarding
    rs1_fwdflag = '0;

    //check for hazard
    if (rs1 != '0) begin
    
        if(rs1 == rd_mem) begin              // rs1 changes in mem stage (priority)
            rs1_fwd = alu_out_mem;
            rs1_fwdflag  = '1; 
        end
        else if(rs1 == rd_wb) begin          // rs1 changes in wr stage
            if (fwd_data)
                rs1_fwd = rdata_wb;
            else
                rs1_fwd = regfilemux_out;

            rs1_fwdflag = '1;
        end

    end
end

always_comb begin : rs2_forwarding
    rs2_fwdflag = '0;

    if (rs2 != '0) begin

        if(rs2 == rd_mem) begin              // rs2 changes in the mem stage (priority)
            rs2_fwd = alu_out_mem;
            rs2_fwdflag = '1;
        end
        else if(rs2 == rd_wb) begin          // rs2 changes in the wr stage
            if (fwd_data)
                rs2_fwd = rdata_wb;
            else
                rs2_fwd = regfilemux_out;

            rs2_fwdflag = '1;
        end

    end
end

always_comb begin : rs2_forwarding_mem
    rs2_fwdflag_mem = '0;

    if ((rd_mem != '0) && (rd_mem == rd_wb)) begin
        rs2_fwdflag_mem = '1;
    end
end

endmodule : fwd_unit

module brp
import rv32i_types::*;
(
    input logic clk,
    input logic rst,
    input logic load,

    input rv32i_control_word ctrl,
    
    // For accuracy counter
    input rv32i_control_word ctrl_ex,
    input logic br_prediction_ex,       // Previous prediction
    input logic br_en,                  // Whether branch was taken from previous prediction
    

    // Offsets
    input rv32i_word b_imm,
    input rv32i_word j_imm,
    input rv32i_opcode opcode_if,
    input rv32i_brp_word brp_ex,
    input rv32i_reg_word regs_if,

    // Outputs the branch target address depending on the prediction, as well as a return target to return to if the prediction was incorrect.
    output logic prediction,
    output rv32i_word br_target,
    output rv32i_word br_ret_target
    output rv32i_brp_word brp_if
);

rv32i_word pc_if;
logic brp_prediction;
rv32i_brp_word brp;
int c_total;    // Total #predictions counter
int c_correct;  // #correct predictions counter

logic bp_correctness;
assign bp_correctness = (br_en == br_prediction_ex);

brp_bimodal bimodal (
    .clk,
    .rst,
    .load (opcode_if == op_br), // Only use predictor when it's a branch instruction
    .bp_correctness,

    .br_prediction (brp_prediction)
);

function void set_defaults ();
    brp.predicted = '0;
    brp.prediction = '0;
    brp.brp_target = '0;
    brp.brp_alt = '0;
    brp.mispredicted = '0;
endfunction

function void predict(logic _prediction, rv32i_word base_addr, rv32i_word offset);
    // brp_if.predicted = '1;
    brp.prediction = _prediction;

    brp.brp_target = (_prediction) ? (base_addr + offset) : (base_addr + 4);
    brp.brp_alt = !(_prediction) ? (base_addr + offset) : (base_addr + 4);
endfunction

/* always_ff @(posedge clk)
begin
    // Reset Signal
    if (rst)
    begin
        br_target <= ctrl.pc + 4;
        br_ret_target <= ctrl.pc + 4;
        prediction <= 1'b0;
    end

    // INSTR is a branch/jal, so we want prediction output
    // Not handling JALR instructions because of potential for hazards in RS1
    else if (load)
    begin
        case (ctrl.opcode)
            op_jal:
            begin
                br_target <= ctrl.pc + j_imm;
                br_ret_target <= ctrl.pc + 4;
                prediction <= 1'b1;
            end
            
            op_br:
            begin
                prediction <= bp_prediction;

                if (bp_prediction)
                begin
                    br_target <= ctrl.pc + b_imm;
                    br_ret_target <= ctrl.pc + 4;
                end

                else
                begin
                    br_target <= ctrl.pc + 4;
                    br_ret_target <= ctrl.pc + b_imm;
                end
            end
        endcase
    end

    else
    begin
        br_target <= br_target;
        br_ret_target <= br_ret_target;
        prediction <= prediction;
    end
end */

always_comb
begin : predictor
    set_defaults();

    // INSTR is a branch/jal, so we want prediction output
    // Not handling JALR instructions because of potential for hazards in RS1
    case (opcode_if)
        op_jal:
        begin
            brp.prediction = '0;
            predict(1'b1, pc_if, regs_if.j_imm);
        end
        
        op_br:
        begin
            brp.prediction = '1;
            predict(brp_prediction, pc_if, regs_if.b_imm);
        end
    endcase
end

always_ff @(posedge clk)
begin : output_assignment
    if (rst) brp_if <= '0;
    else if (load) brp_if <= brp;
    else brp_if <= '0;

end

always_ff @(posedge clk)
begin : correctness
    // Reset signal
    if (rst)
    begin
        c_total <= 0;
        c_correct <= 0;
    end
    else if (brp_ex.mispredicted)
    begin
        c_total <= c_total + 1;
        if (bp_correctness) c_correct <= c_correct + 1;
    end
end


endmodule : brp

module datapath

// ports
(

);

// modules
alu_pc alu (

);

alu_jump alu (
    
);

ALU alu (

);

CMP cmp (

);

PC pc_reg (

);

control_rom control_rom (

);

regfile regfile (

);

// intermediate stage registers 
if_id_reg
id_ex_reg
ex_mem_reg
mem_wr_reg

always_comb begin: MUXES

end

endmodule : datapath

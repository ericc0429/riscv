module mp4_tb;
import rv32i_types::*;
`timescale 1ns/10ps

/********************* Do not touch for proper compilation *******************/
// Instantiate Interfaces
tb_itf itf();
rvfi_itf rvfi(itf.clk, itf.rst);

// Instantiate Testbench
source_tb tb(
    .magic_mem_itf(itf),
    .mem_itf(itf),
    .sm_itf(itf),
    .tb_itf(itf),
    .rvfi(rvfi)
);

// Dump signals
initial begin
    $fsdbDumpfile("dump.fsdb");
    $fsdbDumpvars(0, mp4_tb, "+all");
end
/****************************** End do not touch *****************************/



/***************************** Spike Log Printer *****************************/
// Can be enabled for debugging
spike_log_printer printer(.itf(itf), .rvfi(rvfi));
/*************************** End Spike Log Printer ***************************/


/************************ Signals necessary for monitor **********************/
// This section not required until CP2

assign rvfi.commit = ((dut.datapath.ctrl_wr.valid) && ~dut.datapath.cur_stall_wr); // Set high when a valid instruction is modifying regfile or PC
assign rvfi.halt = (rvfi.commit && (rvfi.pc_rdata == rvfi.pc_wdata)); // Set high when target PC == Current PC for a branch
initial rvfi.order = 0;
always @(posedge itf.clk iff rvfi.commit) rvfi.order <= rvfi.order + 1; // Modify for OoO

assign rvfi.clk = itf.clk;
assign rvfi.rst = itf.rst;

function void set_defaults();
    // Instruction and trap:
    rvfi.inst = dut.datapath.ctrl_wr.instr;
    rvfi.trap = dut.datapath.trap_wr;

    // Regfile:
    rvfi.rs1_addr = dut.datapath.rs1_addr_wr;
    rvfi.rs2_addr = dut.datapath.rs2_addr_wr;
    rvfi.rs1_rdata = dut.datapath.rs1_data_wr;
    rvfi.rs2_rdata = dut.datapath.rs2_data_wr;

    rvfi.load_regfile = dut.datapath.ctrl_wr.load_regfile;
    rvfi.rd_addr = dut.datapath.rd_wr;
    rvfi.rd_wdata = dut.datapath.regfilemux_out;

    // PC:
    rvfi.pc_rdata = dut.datapath.ctrl_wr.pc;
    rvfi.pc_wdata = dut.datapath.pc_wr;

    // Memory:
    rvfi.mem_addr = dut.datapath.alu_out_wr;
    rvfi.mem_rmask = dut.datapath.rmask_wr;
    rvfi.mem_wmask = dut.datapath.wmask_wr;
    rvfi.mem_rdata = dut.datapath.rdata_wr;
    rvfi.mem_wdata = dut.datapath.wdata_wr; 

endfunction

// Set signals that could possible be used in some instructions
// i.e. rs2_addr
always_comb begin

    set_defaults();

    case(dut.datapath.ctrl_wr.opcode)
        
        op_reg, op_store, op_br: ;

        op_lui, op_auipc, op_jal: begin
            rvfi.rs1_addr = '0;
            rvfi.rs2_addr = '0;
            rvfi.rs1_rdata = 32'd0;
            rvfi.rs2_rdata = 32'd0;
        end 
        
        op_jalr, op_imm, op_load: begin
            rvfi.rs2_addr = '0;
            rvfi.rs2_rdata = 32'd0;
        end
    endcase

    if(!dut.datapath.ctrl_wr.mem_read)begin
        rvfi.mem_rmask = 4'h0;
    end
    if(!dut.datapath.ctrl_wr.mem_write)begin
        rvfi.mem_wmask = 4'h0;
    end

    if(dut.datapath.rd_wr == '0)begin
        rvfi.rd_wdata = 32'd0;
    end

end



// Please refer to rvfi_itf.sv for more information.


/**************************** End RVFIMON signals ****************************/



/********************* Assign Shadow Memory Signals Here *********************/
// This section not required until CP2
/*
The following signals need to be set:
icache signals:
assign itf.inst_read
assign itf.inst_addr
assign itf.inst_resp
assign itf.inst_rdata

dcache signals:
assign itf.data_read
assign itf.data_write
assign itf.data_mbe
assign itf.data_addr
assign itf.data_wdata
assign itf.data_resp
assign itf.data_rdata

Please refer to tb_itf.sv for more information.
*/

assign itf.inst_read    = dut.instr_read;
assign itf.inst_addr    = dut.instr_mem_address;
assign itf.inst_resp    = dut.instr_mem_resp;
assign itf.inst_rdata   = dut.instr_mem_rdata;

assign itf.data_read    = dut.data_read;
assign itf.data_write   = dut.data_write;
assign itf.data_mbe     = dut.data_mbe;
assign itf.data_addr    = dut.data_mem_address;
assign itf.data_wdata   = dut.data_mem_wdata;
assign itf.data_resp    = dut.data_mem_resp;
assign itf.data_rdata   = dut.data_mem_rdata;

/*********************** End Shadow Memory Assignments ***********************/

// Set this to the proper value
// assign itf.registers = '{default: '0};
assign itf.registers = dut.datapath.regfile.data;

/*********************** Instantiate your design here ************************/

/*
The following signals need to be connected to your top level for CP2:
Burst Memory Ports:
    itf.mem_read
    itf.mem_write
    itf.mem_wdata
    itf.mem_rdata
    itf.mem_addr
    itf.mem_resp

Please refer to tb_itf.sv for more information.
*/


mp4 dut(
    .clk(itf.clk),
    .rst(itf.rst),
    
    // Remove after CP1
    /* .instr_mem_resp(itf.inst_resp),
    .instr_mem_rdata(itf.inst_rdata),
	.data_mem_resp(itf.data_resp),
    .data_mem_rdata(itf.data_rdata),
    .instr_read(itf.inst_read),
	.instr_mem_address(itf.inst_addr),
    .data_read(itf.data_read),
    .data_write(itf.data_write),
    .data_mbe(itf.data_mbe),
    .data_mem_address(itf.data_addr),
    .data_mem_wdata(itf.data_wdata) */

    // Use for CP2 onwards
    .pmem_read(itf.mem_read),
    .pmem_write(itf.mem_write),
    .pmem_wdata(itf.mem_wdata),
    .pmem_rdata(itf.mem_rdata),
    .pmem_address(itf.mem_addr),
    .pmem_resp(itf.mem_resp)
   
);

/***************************** End Instantiation *****************************/

endmodule

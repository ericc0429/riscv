module mp4_tb;
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
//spike_log_printer printer(.itf(itf), .rvfi(rvfi));
/*************************** End Spike Log Printer ***************************/


/************************ Signals necessary for monitor **********************/
// This section not required until CP2

assign rvfi.commit = 0; // Set high when a valid instruction is modifying regfile or PC
assign rvfi.halt = 0; // Set high when target PC == Current PC for a branch
initial rvfi.order = 0;
always @(posedge itf.clk iff rvfi.commit) rvfi.order <= rvfi.order + 1; // Modify for OoO

/*
Instruction and trap:
    rvfi.inst
    rvfi.trap

Regfile:
    rvfi.rs1_addr
    rvfi.rs2_addr
    rvfi.rs1_rdata
    rvfi.rs2_rdata
    rvfi.load_regfile
    rvfi.rd_addr
    rvfi.rd_wdata

PC:
    rvfi.pc_rdata
    rvfi.pc_wdata

Memory:
    rvfi.mem_addr
    rvfi.mem_rmask
    rvfi.mem_wmask
    rvfi.mem_rdata
    rvfi.mem_wdata

Please refer to rvfi_itf.sv for more information.
*/

/**************************** End RVFIMON signals ****************************/



/********************* Assign Shadow Memory Signals Here *********************/
// This section not required until CP2
/*
The following signals need to be set:
icache signals:
    itf.inst_read
    itf.inst_addr
    itf.inst_resp
    itf.inst_rdata

dcache signals:
    itf.data_read
    itf.data_write
    itf.data_mbe
    itf.data_addr
    itf.data_wdata
    itf.data_resp
    itf.data_rdata

Please refer to tb_itf.sv for more information.
*/

/* 
assign itf.inst_read    = dut.Icache_read;
assign itf.inst_addr    = dut.Icache_address;
assign itf.inst_resp    = dut.Icache_resp;
assign itf.inst_rdata   = dut.Icache_rdata;

assign itf.data_read    = dut.Dcache_read;
assign itf.data_write   = dut.Dcache_write;
assign itf.data_mbe     = dut.data_mbe;
assign itf.data_addr    = dut.Dcache_address;
assign itf.data_wdata   = dut.Dcache_wdata;
assign itf.data_resp    = dut.Dcache_resp;
assign itf.data_rdata   = dut.Dcache_rdata;
*/

/*********************** End Shadow Memory Assignments ***********************/

// Set this to the proper value
assign itf.registers = '{default: '0};

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
    
    .instr_mem_resp(itf.inst_resp),
    .instr_mem_rdata(itf.inst_rdata),
	.data_mem_resp(itf.data_resp),
    .data_mem_rdata(itf.data_rdata),
    .instr_read(itf.inst_read),
	.instr_mem_address(itf.inst_addr),
    .data_read(itf.data_read),
    .data_write(itf.data_write),
    .data_mbe(itf.data_mbe),
    .data_mem_address(itf.data_addr),
    .data_mem_wdata(itf.data_wdata)
    


    // Use for CP2 onwards
    // .pmem_read(itf.mem_read),
    // .pmem_write(itf.mem_write),
    // .pmem_wdata(itf.mem_wdata),
    // .pmem_rdata(itf.mem_rdata),
    // .pmem_address(itf.mem_addr),
    // .pmem_resp(itf.mem_resp)
   
);

// riscv_formal_monitor_rv32imc monitor (
//   .clock,
//   .reset,
//   .rvfi_valid,
//   .rvfi_order,
//   .rvfi_insn,
//   .rvfi_trap,
//   .rvfi_halt,
//   .rvfi_intr,
//   .rvfi_mode,
//   .rvfi_rs1_addr,
//   .rvfi_rs2_addr,
//   .rvfi_rs1_rdata,
//   .rvfi_rs2_rdata,
//   .rvfi_rd_addr,
//   .rvfi_rd_wdata,
//   .rvfi_pc_rdata,
//   .rvfi_pc_wdata,
//   .rvfi_mem_addr,
//   .rvfi_mem_rmask,
//   .rvfi_mem_wmask,
//   .rvfi_mem_rdata,
//   .rvfi_mem_wdata,
//   .rvfi_mem_extamo,
//   .errcode
// );

/***************************** End Instantiation *****************************/

endmodule

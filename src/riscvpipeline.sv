`timescale 1ns/1ps
module riscvpipeline (
    input logic clk,
    input logic rst
);

    //  Performance Counters
    reg [31:0] cycle_count;
    reg [31:0] instr_retired;

    // Memory Interface Wires 
    logic [31:0] imem_addr;
    logic [31:0] imem_rdata;
    logic [31:0] dmem_addr;
    logic [31:0] dmem_wdata;
    logic [31:0] dmem_rdata;
    logic        dmem_wen;
    logic        dmem_ren;

    //  Control Wires 
    logic [6:0] idex_opcode;
    logic [2:0] idex_funct3;
    logic [6:0] idex_funct7;
    logic [4:0] idex_rd;
    logic [4:0] idex_rs1;
    logic [4:0] idex_rs2;
    logic [31:0] ifid_instr;
    logic [4:0]  exmem_rd;
    logic        exmem_regwrite;
    logic [4:0]  memwb_rd;
    logic        memwb_regwrite;
    logic        memwb_memtoreg;
    logic        ctrl_regwrite;
    logic        ctrl_memread;
    logic        ctrl_memwrite;
    logic        ctrl_memtoreg;
    logic        ctrl_alusrc;
    logic [3:0]  ctrl_aluctrl; 
    logic        ctrl_branch;
    logic [1:0] fwd_forwardA;
    logic [1:0] fwd_forwardB;
    logic       hz_if_id_write;
    logic       hz_pc_write;
    logic       hz_id_ex_flush;
    logic       branch_taken;
    // ID/EX-latched control outputs from datapath
    logic        idex_regwrite_out;
    logic        idex_memread_out;
    logic        idex_memwrite_out;
    logic        idex_memtoreg_out;
    logic        idex_alusrc_out;
    logic [3:0]  idex_aluctrl_out;
    logic        idex_branch_out;

    // Memory Instantiations 
    
    // Instruction memory 
    reg [31:0] imem [0:511];
    assign imem_rdata = imem[imem_addr >> 2];

    // Data Memory (Synchronous Read/Write)
    simple_ram dmem_inst (
        .clk(clk),
        .wen(dmem_wen),
        .ren(dmem_ren),
        .addr(dmem_addr),
        .wdata(dmem_wdata),
        .rdata(dmem_rdata)
    );

    // Module Instantiations

    datapath datapath_inst (
        .clk(clk),
        .rst(rst),
        // Memory Interface
        .imem_addr(imem_addr),
        .imem_rdata(imem_rdata),
        .dmem_addr(dmem_addr),
        .dmem_wdata(dmem_wdata),
        .dmem_wen(dmem_wen),
        .dmem_ren(dmem_ren),
        .dmem_rdata(dmem_rdata),
        // IF/ID
        .ifid_instr(ifid_instr),
        // ID/EX (to control)
        .idex_rs1(idex_rs1),
        .idex_rs2(idex_rs2),
        .idex_rd(idex_rd),
        .idex_opcode(idex_opcode),
        .idex_funct3(idex_funct3),
        .idex_funct7(idex_funct7),
    // ID/EX-latched control outputs (from datapath)
    .idex_regwrite_out(idex_regwrite_out),
    .idex_memread_out(idex_memread_out),
    .idex_memwrite_out(idex_memwrite_out),
    .idex_memtoreg_out(idex_memtoreg_out),
    .idex_alusrc_out(idex_alusrc_out),
    .idex_aluctrl_out(idex_aluctrl_out),
    .idex_branch_out(idex_branch_out),
    // Control (registered into ID/EX-aligned signals)
    .idex_regwrite(ctrl_regwrite),
    .idex_memread(ctrl_memread),
    .idex_memwrite(ctrl_memwrite),
    .idex_memtoreg(ctrl_memtoreg),
    .idex_alusrc(ctrl_alusrc),
    .idex_aluctrl(ctrl_aluctrl),
    .idex_branch(ctrl_branch),
        // EX/MEM (to forwarding)
        .exmem_rd(exmem_rd),
        .exmem_regwrite(exmem_regwrite),
        // MEM/WB (to forwarding)
        .memwb_rd(memwb_rd),
        .memwb_regwrite(memwb_regwrite),
        .memwb_memtoreg(memwb_memtoreg),
        // Forwarding (from fwd unit)
        .forwardA(fwd_forwardA),
        .forwardB(fwd_forwardB),
        // Hazard (from hazard unit)
        .if_id_write(hz_if_id_write),
        .pc_write(hz_pc_write),
        .id_ex_flush(hz_id_ex_flush),
        // Branch
        .branch_taken(branch_taken),
        .if_id_flush(branch_taken) // Flush IF/ID if branch is taken
    );

    // Controller
  
    controller controller_inst (
        .opcode(ifid_instr[6:0]),
        .funct3(ifid_instr[14:12]),
        .funct7(ifid_instr[31:25]),
        .RegWrite(ctrl_regwrite),
        .MemRead(ctrl_memread),
        .MemWrite(ctrl_memwrite),
        .MemToReg(ctrl_memtoreg),
        .ALUSrc(ctrl_alusrc),
        .Branch(ctrl_branch),
        .ALUctrl(ctrl_aluctrl)
    );

    
    // Hazard Unit
    
    hazard_unit hazard_unit_inst (
        .idex_memread(idex_memread_out),    // From datapath-latched ID/EX controls
        .idex_rd(idex_rd),              // From datapath
        .ifid_rs1(ifid_instr[19:15]),   // Decode rs1 from IF/ID instr
        .ifid_rs2(ifid_instr[24:20]),   // Decode rs2 from IF/ID instr
        .stall(),                       // Unused output
        .if_id_write(hz_if_id_write),
        .pc_write(hz_pc_write),
        .id_ex_flush(hz_id_ex_flush)
    );


    // Forwarding Unit
    
    forwarding_unit forwarding_unit_inst (
        .exmem_regwrite(exmem_regwrite),
        .exmem_rd(exmem_rd),
        .memwb_regwrite(memwb_regwrite),
        .memwb_rd(memwb_rd),
        .idex_rs1(idex_rs1),
        .idex_rs2(idex_rs2),
        .forwardA(fwd_forwardA),
        .forwardB(fwd_forwardB)
    );

    // Performance Counters 
    always @(posedge clk) begin
        if (rst) begin
            cycle_count <= 0;
            instr_retired <= 0;
        end else begin
            cycle_count <= cycle_count + 1;
            if (memwb_regwrite && (memwb_rd != 0)) begin
                instr_retired <= instr_retired + 1;
            end
        end
    end
    
    always @(posedge clk) begin
        if (!rst) begin
            if (dmem_wen) begin
                $display("TOP DMEM WRITE: time=%0t addr=0x%0h data=0x%0h", $time, dmem_addr, dmem_wdata);
            end
            if (memwb_regwrite) begin
                $display("TOP MEMWB: time=%0t memwb_regwrite=%0b memwb_rd=%0d", $time, memwb_regwrite, memwb_rd);
            end
        end
    end

    always @(posedge clk) begin
        if (!rst) begin
            $display("TOP CTRL: time=%0t idex_opcode=0b%b ctrl_regwrite=%0b ctrl_memread=%0b ctrl_memwrite=%0b ctrl_alusrc=%0b",
                     $time, idex_opcode, ctrl_regwrite, ctrl_memread, ctrl_memwrite, ctrl_alusrc);
            $display("TOP DBG: time=%0t ctrl=(wr=%0b rd=%0b we=%0b m2r=%0b a_src=%0b aluctrl=0x%0h branch=%0b) idex_reg_out=(wr=%0b rd=%0b we=%0b m2r=%0b a_src=%0b aluctrl=0x%0h branch=%0b) hazard=(if_id_write=%0b pc_write=%0b id_ex_flush=%0b)",
                     $time, ctrl_regwrite, ctrl_memread, ctrl_memwrite, ctrl_memtoreg, ctrl_alusrc, ctrl_aluctrl, ctrl_branch,
                     idex_regwrite_out, idex_memread_out, idex_memwrite_out, idex_memtoreg_out, idex_alusrc_out, idex_aluctrl_out, idex_branch_out,
                     hz_if_id_write, hz_pc_write, hz_id_ex_flush);
        end
    end
    
endmodule

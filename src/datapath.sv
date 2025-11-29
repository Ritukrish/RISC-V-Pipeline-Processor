module datapath (
    input  logic clk,
    input  logic rst,

    output logic [31:0] imem_addr,
    input  logic [31:0] imem_rdata,

    // Data Memory 
    output logic [31:0] dmem_addr,
    output logic [31:0] dmem_wdata,
    output logic        dmem_wen,
    output logic        dmem_ren,
    input  logic [31:0] dmem_rdata,
    
    // IF/ID outputs 
    output logic [31:0] pc,
    output logic [31:0] ifid_pc,
    output logic [31:0] ifid_instr,

    // ID/EX pipeline regs 
    output logic [31:0] idex_pc,
    output logic [31:0] idex_rs1_data,
    output logic [31:0] idex_rs2_data,
    output logic [31:0] idex_imm,
    output logic [4:0]  idex_rs1,
    output logic [4:0]  idex_rs2,
    output logic [4:0]  idex_rd,
    output logic [6:0]  idex_opcode,
    output logic [2:0]  idex_funct3,
    output logic [6:0]  idex_funct7,

    // ID/EX-aligned control outputs 
    output logic        idex_regwrite_out,
    output logic        idex_memread_out,
    output logic        idex_memwrite_out,
    output logic        idex_memtoreg_out,
    output logic        idex_alusrc_out,
    output logic [3:0]  idex_aluctrl_out,
    output logic        idex_branch_out,

    // control signals that are set by top-level controller 
    input  logic idex_regwrite,
    input  logic idex_memread,
    input  logic idex_memwrite,
    input  logic idex_memtoreg,
    input  logic idex_alusrc,
    input  logic [3:0] idex_aluctrl,
    input  logic idex_branch,

    // EX/MEM
    output logic [31:0] exmem_pc,
    output logic [31:0] exmem_alu_out,
    output logic [31:0] exmem_rs2_data,
    output logic [4:0]  exmem_rd,
    output logic        exmem_regwrite,
    output logic        exmem_memread,
    output logic        exmem_memwrite,
    output logic        exmem_memtoreg,
    output logic        exmem_branch,
    output logic        exmem_zero,

    // MEM/WB
    output logic [31:0] memwb_pc,
    output logic [31:0] memwb_alu_out,
    output logic [31:0] memwb_mem_data,
    output logic [4:0]  memwb_rd,
    output logic        memwb_regwrite,
    output logic        memwb_memtoreg,

    // forwarding controls
    input logic [1:0] forwardA,
    input logic [1:0] forwardB,

    // hazard controls into the datapath
    input logic if_id_write,
    input logic pc_write,
    input logic id_ex_flush,
    input logic if_id_flush,

    // branch outputs
    output logic [31:0] branch_target,
    output logic branch_taken
);

    // Program counter
    logic [31:0] PC;
    assign pc = PC;

    // Register file
    logic [31:0] regfile [0:31];
    integer i;

    //  WB Stage
    always_ff @(posedge clk) begin
        if (rst) begin
            // Initialize all registers to 0 on reset
            for (i = 0; i < 32; i = i + 1)
                regfile[i] <= 32'd0;
        end else begin
            // Normal write-back logic
            if (memwb_regwrite_r && (memwb_rd_r != 0)) begin
                if (memwb_memtoreg_r) begin
                    regfile[memwb_rd_r] <= memwb_mem_r;
                end else begin
                    regfile[memwb_rd_r] <= memwb_alu_r;
                end
            end
        end
    end

    // Internal IF/ID registers
    logic [31:0] if_pc_reg;
    logic [31:0] if_instr_reg;

    // ID/EX
    logic [31:0] idex_pc_r;
    logic [31:0] idex_rs1_r, idex_rs2_r;
    logic [31:0] idex_imm_r;
    logic [4:0]  idex_rs1_rn, idex_rs2_rn, idex_rd_rn;
    logic [6:0]  idex_opcode_r;
    logic [2:0]  idex_funct3_r;
    logic [6:0]  idex_funct7_r;
    // ID/EX latched control signals (owned by datapath)
    logic        idex_regwrite_r;
    logic        idex_memread_r;
    logic        idex_memwrite_r;
    logic        idex_memtoreg_r;
    logic        idex_alusrc_r;
    logic [3:0]  idex_aluctrl_r;
    logic        idex_branch_r;

    // EX/MEM 
    logic [31:0] exmem_pc_r;
    logic [31:0] exmem_alu_r;
    logic [31:0] exmem_rs2_r;
    logic [4:0]  exmem_rd_r;
    logic        exmem_regwrite_r, exmem_memread_r, exmem_memwrite_r, exmem_memtoreg_r, exmem_branch_r;
    logic        exmem_zero_r;

    // MEM/WB 
    logic [31:0] memwb_pc_r;
    logic [31:0] memwb_alu_r;
    logic [31:0] memwb_mem_r;
    logic [4:0]  memwb_rd_r;
    logic        memwb_regwrite_r, memwb_memtoreg_r;

    //  IF Stage
    assign imem_addr = PC; // Connect PC to instruction memory address
    
    always_ff @(posedge clk) begin
        if (rst) begin
            PC <= 0;
            if_pc_reg <= 0;
            if_instr_reg <= 32'h00000013; // NOP
        end else begin
            if (!pc_write) begin
                if (branch_taken) begin
                    PC <= branch_target;
                end else begin
                    PC <= PC + 4;
                end
            end else begin
                PC <= PC; // stall
            end

            // IF/ID pipeline register handling
            if (!if_id_write) begin
                if_pc_reg <= PC;
                if_instr_reg <= imem_rdata; // Fetch from imem port
            end else begin
                if_pc_reg <= if_pc_reg; // freeze
                if_instr_reg <= if_instr_reg;
            end

            if (if_id_flush) begin
                if_instr_reg <= 32'h00000013; // flush to nop
            end
        end
    end

    always @(posedge clk) begin
        if (!rst) begin
            $display("DP_IFID: time=%0t PC=0x%0h if_pc_reg=0x%0h if_instr_reg=0x%0h imem_rdata=0x%0h",
                     $time, PC, if_pc_reg, if_instr_reg, imem_rdata);
        end
    end

    // Expose IF/ID
    assign ifid_pc = if_pc_reg;
    assign ifid_instr = if_instr_reg;

    //  ID Stage
 
    logic [6:0] opcode;
    logic [4:0] rd;
    logic [2:0] funct3;
    logic [4:0] rs1;
    logic [4:0] rs2;
    logic [6:0] funct7;

    // Continuous assignments from IF/ID register fields
    assign opcode = if_instr_reg[6:0];
    assign rd     = if_instr_reg[11:7];
    assign funct3 = if_instr_reg[14:12];
    assign rs1    = if_instr_reg[19:15];
    assign rs2    = if_instr_reg[24:20];
    assign funct7 = if_instr_reg[31:25];

    // Immediate generation functions
    function automatic logic [31:0] signext_imm_i(input logic [11:0] imm);
        signext_imm_i = {{20{imm[11]}}, imm};
    endfunction
    function automatic logic [31:0] signext_imm_s(input logic [11:0] imm);
        signext_imm_s = {{20{imm[11]}}, imm};
    endfunction
    function automatic logic [31:0] signext_imm_b(input logic [12:0] imm);
        // imm is 13 bits: imm[12:0]. The branch immediate in RISC-V uses bits [12:1]
        // (imm[0] is always 0). Sign-extend imm[12] and place imm[12:1] and a 0 LSB to make a 32-bit signed branch offset.
        
        signext_imm_b = {{19{imm[12]}}, imm[12:1], 1'b0};
    endfunction

    logic [31:0] imm_i, imm_s, imm_b;
    always_comb begin
        imm_i = signext_imm_i(if_instr_reg[31:20]);
        imm_s = signext_imm_s({if_instr_reg[31:25], if_instr_reg[11:7]});
        imm_b = signext_imm_b({if_instr_reg[31], if_instr_reg[7], if_instr_reg[30:25], if_instr_reg[11:8]});
    end

    //  ID/EX Pipeline Register 
    always_ff @(posedge clk) begin
        if (rst) begin
            idex_pc_r <= 0;
            idex_rs1_r <= 0;
            idex_rs2_r <= 0;
            idex_imm_r <= 0;
            idex_rs1_rn <= 0;
            idex_rs2_rn <= 0;
            idex_rd_rn <= 0;
            idex_opcode_r <= 7'h0;
            idex_funct3_r <= 3'h0;
            idex_funct7_r <= 7'h0;
            // reset latched controls
            idex_regwrite_r <= 0;
            idex_memread_r  <= 0;
            idex_memwrite_r <= 0;
            idex_memtoreg_r <= 0;
            idex_alusrc_r   <= 0;
            idex_aluctrl_r  <= 4'd0;
            idex_branch_r   <= 0;
        end else begin
            if (id_ex_flush) begin
                // insert bubble
                idex_pc_r <= 0;
                idex_rs1_r <= 0;
                idex_rs2_r <= 0;
                idex_imm_r <= 0;
                idex_rs1_rn <= 0;
                idex_rs2_rn <= 0;
                idex_rd_rn <= 0;
                idex_opcode_r <= 7'h13; // an I-type NOP-like opcode
                idex_funct3_r <= 3'h0;
                idex_funct7_r <= 7'h0;
            end else begin
                // Normal operation
                idex_pc_r <= if_pc_reg;
                idex_rs1_r <= regfile[rs1]; 
                idex_rs2_r <= regfile[rs2]; 
                
                // Select immediate
                if (opcode == 7'b0000011) begin // lw
                    idex_imm_r <= imm_i;
                end else if (opcode == 7'b0100011) begin // sw
                    idex_imm_r <= imm_s;
                end else if (opcode == 7'b1100011) begin // branch
                    idex_imm_r <= imm_b;
                end else begin // I-type, R-type (imm not used)
                    idex_imm_r <= imm_i;
                end
                
                // Pass register numbers
                idex_rs1_rn <= rs1;
                idex_rs2_rn <= rs2;
                idex_rd_rn <= rd;
                
                // Pass control to the next stage 
                idex_opcode_r <= opcode;
                idex_funct3_r <= funct3;
                idex_funct7_r <= funct7;
                // latch control inputs into ID/EX-aligned control regs
                idex_regwrite_r <= idex_regwrite;
                idex_memread_r  <= idex_memread;
                idex_memwrite_r <= idex_memwrite;
                idex_memtoreg_r <= idex_memtoreg;
                idex_alusrc_r   <= idex_alusrc;
                idex_aluctrl_r  <= idex_aluctrl;
                idex_branch_r   <= idex_branch;
            end
        end
    end

    // Debug ID/EX contents
    always @(posedge clk) begin
        if (!rst) begin
            $display("DP_IDEX: time=%0t idex_opcode=0b%b idex_rd=%0d idex_rs1=%0d idex_rs2=%0d idex_imm=0x%0h",
                     $time, idex_opcode_r, idex_rd_rn, idex_rs1_rn, idex_rs2_rn, idex_imm_r);
            // Show the idex control inputs (from top) next to the latched ID/EX fields
            $display("DP_IDEX_CTRL: time=%0t idex_regwrite(in)=%0b idex_memread(in)=%0b idex_memwrite(in)=%0b idex_memtoreg(in)=%0b idex_alusrc(in)=%0b idex_aluctrl(in)=0x%0h idex_branch(in)=%0b",
                     $time, idex_regwrite, idex_memread, idex_memwrite, idex_memtoreg, idex_alusrc, idex_aluctrl, idex_branch);
            
            $display("DP_IDEX_REGS: time=%0t idex_opcode_r=0b%b idex_f3=0b%b idex_f7=0x%0h",
                     $time, idex_opcode_r, idex_funct3_r, idex_funct7_r);
        end
    end

    // Expose ID/EX
    assign idex_pc = idex_pc_r;
    assign idex_rs1_data = idex_rs1_r;
    assign idex_rs2_data = idex_rs2_r;
    assign idex_imm = idex_imm_r;
    assign idex_rs1 = idex_rs1_rn;
    assign idex_rs2 = idex_rs2_rn;
    assign idex_rd = idex_rd_rn;
    assign idex_opcode = idex_opcode_r;
    assign idex_funct3 = idex_funct3_r;
    assign idex_funct7 = idex_funct7_r;

    // Expose latched control signals
    assign idex_regwrite_out = idex_regwrite_r;
    assign idex_memread_out  = idex_memread_r;
    assign idex_memwrite_out = idex_memwrite_r;
    assign idex_memtoreg_out = idex_memtoreg_r;
    assign idex_alusrc_out   = idex_alusrc_r;
    assign idex_aluctrl_out  = idex_aluctrl_r;
    assign idex_branch_out   = idex_branch_r;

    // EX Stage
    logic [31:0] alu_in1, alu_in2;
    logic [31:0] forwarded_a, forwarded_b;
    logic [31:0] memwb_data_for_fwd;

    // Data from MEM/WB stage for forwarding
    assign memwb_data_for_fwd = memwb_memtoreg_r ? memwb_mem_r : memwb_alu_r;

    always_comb begin
        // Default: get data from ID/EX register
        forwarded_a = idex_rs1_r;
        forwarded_b = idex_rs2_r;

        case (forwardA)
            2'b00: forwarded_a = idex_rs1_r;
            2'b10: forwarded_a = exmem_alu_r;
            2'b01: forwarded_a = memwb_data_for_fwd;
            default: forwarded_a = idex_rs1_r;
        endcase
        case (forwardB)
            2'b00: forwarded_b = idex_rs2_r;
            2'b10: forwarded_b = exmem_alu_r;
            2'b01: forwarded_b = memwb_data_for_fwd;
            default: forwarded_b = idex_rs2_r;
        endcase
    end

    assign alu_in1 = forwarded_a;
    assign alu_in2 = idex_alusrc ? idex_imm_r : forwarded_b;

    // ALU implementation
    function automatic logic [31:0] ALU(input logic [3:0] op, input logic [31:0] a, input logic [31:0] b, input logic [2:0] f3, input logic [6:0] f7);
        logic [31:0] res;
        begin
            res = 32'd0;
            unique case (op)
                4'd0: res = a + b; // ADD
                4'd1: res = a - b; // SUB
                4'd2: res = a & b; // AND
                4'd3: res = a | b; // OR
                4'd4: res = a ^ b; // XOR
                4'd5: res = a << (b[4:0]); // SLL
                4'd6: res = a >> (b[4:0]); // SRL
                4'd7: res = ($signed(a)) >>> (b[4:0]); // SRA
                4'd8: res = a & ~b; // ANDN
                4'd9: res = a | ~b; // ORN
                4'd10: res = ~(a ^ b); // XNOR
                4'd11: res = ($signed(a) < $signed(b)) ? a : b; // MIN
                4'd12: res = ($signed(a) > $signed(b)) ? a : b; // MAX
                4'd13: res = (a < b) ? a : b; // MINU
                4'd14: res = (a > b) ? a : b; // MAXU
                4'd15: begin
                    case (f7)
                        7'h2A: res = (a[31]) ? -a : a; // ABS
                        7'h28: res = (a << (b[4:0])) | (a >> (32 - (b[4:0]))); // ROL
                        7'h29: res = (a >> (b[4:0])) | (a << (32 - (b[4:0]))); // ROR
                        default: res = a;
                    endcase
                end
                default: res = a + b;
            endcase
            ALU = res;
        end
    endfunction

    logic [31:0] alu_result;
    always_comb begin
        alu_result = ALU(idex_aluctrl, alu_in1, alu_in2, idex_funct3_r, idex_funct7_r);
    end

    // Branch decision
    always_comb begin
        branch_taken = 0;
        branch_target = 0;
        if (idex_branch && (idex_opcode_r == 7'b1100011)) begin // Branch
            if (idex_funct3_r == 3'b000) begin // BEQ
                if (forwarded_a == forwarded_b) begin // Use forwarded values
                    branch_taken = 1;
                    branch_target = idex_pc_r + idex_imm_r;
                end
            end
            
        end
    end

    //  EX/MEM Pipeline Register 
    always_ff @(posedge clk) begin
        if (rst) begin
            exmem_pc_r <= 0;
            exmem_alu_r <= 0;
            exmem_rs2_r <= 0;
            exmem_rd_r <= 0;
            exmem_regwrite_r <= 0;
            exmem_memread_r <= 0;
            exmem_memwrite_r <= 0;
            exmem_memtoreg_r <= 0;
            exmem_branch_r <= 0;
            exmem_zero_r <= 0;
        end else begin
            exmem_pc_r <= idex_pc_r;
            exmem_alu_r <= alu_result;
            exmem_rs2_r <= forwarded_b; 
            exmem_rd_r <= idex_rd_rn;
            exmem_regwrite_r <= idex_regwrite_r;
            exmem_memread_r  <= idex_memread_r;
            exmem_memwrite_r <= idex_memwrite_r;
            exmem_memtoreg_r <= idex_memtoreg_r;
            exmem_branch_r   <= idex_branch_r;
            exmem_zero_r     <= (alu_result == 0);
        end
    end

    // Debug EX/MEM contents
    always @(posedge clk) begin
        if (!rst) begin
            $display("DP_EXMEM: time=%0t exmem_rd=%0d exmem_alu=0x%0h exmem_rs2=0x%0h exmem_memwrite=%0b exmem_memread=%0b",
                     $time, exmem_rd_r, exmem_alu_r, exmem_rs2_r, exmem_memwrite_r, exmem_memread_r);
        end
    end

    // MEM Stage
    
    assign dmem_addr = exmem_alu_r;    // Address comes from ALU result
    assign dmem_wdata = exmem_rs2_r;   // Write data comes from forwarded rs2
    assign dmem_wen = exmem_memwrite_r; // Write enable from control
    assign dmem_ren = exmem_memread_r;  // Read enable from control

    //  MEM/WB Pipeline Register 
    
    always_ff @(posedge clk) begin
        if (rst) begin
            memwb_pc_r <= 0;
            memwb_alu_r <= 0;
            memwb_mem_r <= 0;
            memwb_rd_r <= 0;
            memwb_regwrite_r <= 0;
            memwb_memtoreg_r <= 0;
        end else begin
            memwb_pc_r <= exmem_pc_r;
            memwb_alu_r <= exmem_alu_r;
            memwb_mem_r <= dmem_rdata; 
            memwb_rd_r <= exmem_rd_r;
            memwb_regwrite_r <= exmem_regwrite_r;
            memwb_memtoreg_r <= exmem_memtoreg_r;
        end
    end

    // Debug MEM/WB contents
    always @(posedge clk) begin
        if (!rst) begin
            $display("DP_MEMWB: time=%0t memwb_rd=%0d memwb_regwrite=%0b memwb_mem=0x%0h memwb_alu=0x%0h",
                     $time, memwb_rd_r, memwb_regwrite_r, memwb_mem_r, memwb_alu_r);
        end
    end

    // Expose EX/MEM and MEM/WB
    assign exmem_pc = exmem_pc_r;
    assign exmem_alu_out = exmem_alu_r;
    assign exmem_rs2_data = exmem_rs2_r;
    assign exmem_rd = exmem_rd_r;
    assign exmem_regwrite = exmem_regwrite_r;
    assign exmem_memread = exmem_memread_r;
    assign exmem_memwrite = exmem_memwrite_r;
    assign exmem_memtoreg = exmem_memtoreg_r;
    assign exmem_branch = exmem_branch_r;
    assign exmem_zero = exmem_zero_r;

    assign memwb_pc = memwb_pc_r;
    assign memwb_alu_out = memwb_alu_r;
    assign memwb_mem_data = memwb_mem_r;
    assign memwb_rd = memwb_rd_r;
    assign memwb_regwrite = memwb_regwrite_r;
    assign memwb_memtoreg = memwb_memtoreg_r;

endmodule

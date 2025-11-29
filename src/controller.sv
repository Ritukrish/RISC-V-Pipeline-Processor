module controller (
    input  logic [6:0] opcode,
    input  logic [2:0] funct3,
    input  logic [6:0] funct7,

    // Main control outputs
    output logic       RegWrite,
    output logic       MemRead,
    output logic       MemWrite,
    output logic       MemToReg,
    output logic       ALUSrc,
    output logic       Branch,
    
    // ALU control output
    output logic [3:0] ALUctrl
);

    // ALUctrl encoding:
    // 0: ADD, 1: SUB, 2: AND, 3: OR, 4: XOR, 5: SLL, 6: SRL, 7: SRA
    // 8: ANDN, 9: ORN, 10: XNOR, 11: MIN, 12: MAX, 13: MINU, 14: MAXU
    // 15: ABS/ROL/ROR (disambiguated by funct7)
    
    logic [3:0] alu_op_r_type;

    // Combinational block for all control signals
    always_comb begin
        // 1. Main Control Logic
        // Default values (NOP)
        RegWrite = 0;
        MemRead  = 0;
        MemWrite = 0;
        MemToReg = 0;
        ALUSrc   = 0;
        Branch   = 0;

        case (opcode)
            7'b0110011: begin // R-type (and RVX10)
                RegWrite = 1;
                ALUSrc   = 0;
            end
            7'b0010011: begin // I-type ALU (addi)
                RegWrite = 1;
                ALUSrc   = 1;
            end
            7'b0000011: begin // LW
                RegWrite = 1;
                MemRead  = 1;
                MemToReg = 1;
                ALUSrc   = 1;
            end
            7'b0100011: begin // SW
                MemWrite = 1;
                ALUSrc   = 1;
            end
            7'b1100011: begin // BEQ
                Branch   = 1;
                ALUSrc   = 0;
            end
            default: begin
                // All signals remain 0 (NOP)
            end
        endcase

        //  2. ALU Control Logic
        
        unique case (funct7)
            7'h00: begin // basic R-type
                case (funct3)
                    3'b000: alu_op_r_type = 4'd0;  // ADD
                    3'b100: alu_op_r_type = 4'd4;  // XOR
                    3'b110: alu_op_r_type = 4'd3;  // OR
                    3'b111: alu_op_r_type = 4'd2;  // AND
                    3'b001: alu_op_r_type = 4'd5;  // SLL
                    3'b101: alu_op_r_type = 4'd6;  // SRL
                    default: alu_op_r_type = 4'd0; // default to ADD
                endcase
            end
            7'h20: alu_op_r_type = 4'd1;  // SUB
            7'h2A: alu_op_r_type = 4'd15; // ABS
            7'h21: alu_op_r_type = 4'd8;  // ANDN
            7'h22: alu_op_r_type = 4'd9;  // ORN
            7'h23: alu_op_r_type = 4'd10; // XNOR
            7'h24: alu_op_r_type = 4'd11; // MIN
            7'h25: alu_op_r_type = 4'd12; // MAX
            7'h26: alu_op_r_type = 4'd13; // MINU
            7'h27: alu_op_r_type = 4'd14; // MAXU
            7'h28: alu_op_r_type = 4'd15; // ROL
            7'h29: alu_op_r_type = 4'd15; // ROR
            default: alu_op_r_type = 4'd0; // default to ADD
        endcase

        //  3. Final ALUctrl Mux
        if (opcode == 7'b0110011) begin       // R-type
            ALUctrl = alu_op_r_type;
        end else if (opcode == 7'b1100011) begin // BEQ
            ALUctrl = 4'd1; // SUB (for comparison)
        end else begin                         // LW, SW, ADDI
            ALUctrl = 4'd0; // ADD (for addr calc or addi)
        end
    end
endmodule

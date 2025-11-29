module forwarding_unit (
    input  logic exmem_regwrite,
    input  logic [4:0] exmem_rd,
    input  logic memwb_regwrite,
    input  logic [4:0] memwb_rd,
    input  logic [4:0] idex_rs1,
    input  logic [4:0] idex_rs2,
    output logic [1:0] forwardA,
    output logic [1:0] forwardB
);
    always_comb begin
        // default
        forwardA = 2'b00;
        forwardB = 2'b00;

        // EX hazard
        if (exmem_regwrite && (exmem_rd != 0) && (exmem_rd == idex_rs1))
            forwardA = 2'b10;
        if (exmem_regwrite && (exmem_rd != 0) && (exmem_rd == idex_rs2))
            forwardB = 2'b10;

        // MEM hazard 
        if (memwb_regwrite && (memwb_rd != 0) && !(exmem_regwrite && (exmem_rd != 0) && (exmem_rd == idex_rs1)) && (memwb_rd == idex_rs1))
            forwardA = 2'b01;
        if (memwb_regwrite && (memwb_rd != 0) && !(exmem_regwrite && (exmem_rd != 0) && (exmem_rd == idex_rs2)) && (memwb_rd == idex_rs2))
            forwardB = 2'b01;
    end
endmodule

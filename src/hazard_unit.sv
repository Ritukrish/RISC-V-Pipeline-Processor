module hazard_unit (
    input  logic idex_memread,
    input  logic [4:0] idex_rd,
    input  logic [4:0] ifid_rs1,
    input  logic [4:0] ifid_rs2,
    output logic stall,
    output logic if_id_write,
    output logic pc_write,
    output logic id_ex_flush
);
    always_comb begin
        if (idex_memread && ((idex_rd == ifid_rs1) || (idex_rd == ifid_rs2)) && (idex_rd != 0)) begin
            stall = 1;
            if_id_write = 1; 
            pc_write = 1;    
            id_ex_flush = 1; 
        end else begin
            stall = 0;
            if_id_write = 0;
            pc_write = 0;
            id_ex_flush = 0;
        end
    end
endmodule

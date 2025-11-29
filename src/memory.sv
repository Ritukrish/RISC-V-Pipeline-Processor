module simple_ram (
    input  logic        clk,
    input  logic        wen,     // write enable
    input  logic        ren,     // read enable
    input  logic [31:0] addr,    // byte address
    input  logic [31:0] wdata,   // write data
    output logic [31:0] rdata    // read data (registered)
);
   
    logic [31:0] mem [0:1023];

    always_ff @(posedge clk) begin
        if (wen) begin
            mem[addr >> 2] <= wdata;
            $display("DMEM WRITE: time=%0t addr=0x%0h data=0x%0h", $time, addr, wdata);
        end
        if (ren) rdata <= mem[addr >> 2];
    end
endmodule

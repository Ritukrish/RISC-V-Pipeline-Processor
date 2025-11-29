`timescale 1ns/1ps
module tb;
   
    reg clk;
    reg rst;

    riscvpipeline uut(.clk(clk), .rst(rst));

    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10 ns period
    end
    
  
    initial begin
        // performance counters / CPI locals (declare at start of block)
        integer cycles;
        integer instrs;
        real measured_cpi;
        real single_cycle_cpi;
        real ideal_pipeline_cpi;

        $dumpfile("dump.vcd");
        $dumpvars(0, uut);

        // assert reset
        rst = 1;
        $display("TB: reset asserted at time=%0t", $time);

        // Wait for 2 full clock cycles while reset is high to ensures all registers are properly reset.
        @(posedge clk);
        @(posedge clk);

        // De-assert reset *between* clock edges (1ns after the posedge)
        #1;
        $display("TB: releasing reset at time=%0t", $time);
        rst = 0;

        for (integer i = 0; i < 64; i = i + 1) begin
            uut.imem[i] = 32'h00000013; // NOP
        end
        $readmemh("tests/rvx10_pipeline.hex", uut.imem);

        // Initialize dmem (access the internal mem array of the simple_ram instance)
        for (integer i = 0; i < 64; i = i + 1) begin
            uut.dmem_inst.mem[i] = 32'd0;
        end

        #1000;

        // check memory[100] (word index 100/4 = 25)
    if (uut.dmem_inst.mem[100 >> 2] == 25) 
    	$display("TEST PASSED: dmem[100] == 25");
    else 
     	$display("TEST FAILED: dmem[100] = %0d (expected 25)", uut.dmem_inst.mem[100 >> 2]);

        // Check performance counters and print CPI summary
        cycles = uut.cycle_count;
        instrs = uut.instr_retired;
        $display("cycle_count = %0d", cycles);
        $display("instr_retired = %0d", instrs);
        if (instrs != 0) begin
            measured_cpi = $itor(cycles) / $itor(instrs);
            // Print measured CPI 
            $display("Measured CPI = %0.2f (cycles/instr)", measured_cpi);
            // Baselines: single-cycle implementation (CPI ~= 5.0 for a 5-stage non-pipelined) and ideal pipelined (CPI = 1.0)
            single_cycle_cpi = 5.0;
            ideal_pipeline_cpi = 1.0;
            $display("Baseline single-cycle CPI = %0.2f => expected cycles = %0.2f", single_cycle_cpi, single_cycle_cpi * $itor(instrs));
            $display("Ideal pipelined CPI = %0.2f => expected cycles = %0.2f", ideal_pipeline_cpi, ideal_pipeline_cpi * $itor(instrs));
            $display("Measured cycles = %0d (%.2f%% of single-cycle expected, %.2f%% over ideal)", cycles,
                     100.0 * (measured_cpi / single_cycle_cpi),
                     100.0 * ((measured_cpi - ideal_pipeline_cpi) / ideal_pipeline_cpi));
        end

        $finish;
    end

    // Optional monitor
    always @(posedge clk) begin
        if (!rst)
            $display("TB_MON: time=%0t cycle=%0d PC=%0h ifid_instr=%h retired=%0d",
                     $time, uut.cycle_count, uut.imem_addr, uut.ifid_instr, uut.instr_retired);
    end
    
endmodule

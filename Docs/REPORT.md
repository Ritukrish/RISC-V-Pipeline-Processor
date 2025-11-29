# RVX10p — Project Report

## Executive Summary
This document collects the design notes, test procedure, results, observations, and known issues for the RVX10p pipelined CPU . The design implements a pipelined RV32I-like datapath with control logic, forwarding, hazard detection, and memory modules. The testbench exercises typical pipeline behaviors: ALU operations, loads/stores, branch control, forwarding, and load-use stalls. Waveforms and simulation artifacts are included in the Docs directory (wave.png) for visual inspection.

## Project Objectives
- Implement a 5-stage pipelined CPU datapath (IF, ID, EX, MEM, WB) supporting a subset of RV32I.
- Provide control logic to assert appropriate control signals per instruction.
- Implement forwarding and a hazard detection unit to resolve data hazards and generate stalls where necessary.
- Verify correctness using an Icarus Verilog testbench and examine timing/behavior with GTKWave.

## Repository Layout (relevant files)
- RVX10p/README.md — Project overview and instructions (kept at RVX10p/).
- tb/tb_pipeline.sv — Top-level testbench (simulation stimulus).
- src/datapath.sv — Datapath top and stage registers (IF/ID, ID/EX, EX/MEM, MEM/WB).
- src/controller.sv — Control unit that decodes instructions and generates control signals.
- src/forwarding_unit.sv — Forwarding mux control logic for EX stage operands.
- src/hazard_unit.sv — Load-use and other stall/flush logic.
- src/memory.sv — Data memory / instruction memory modules used by the testbench.
- src/riscvpipeline.sv — Top-level CPU module instantiated by tb_pipeline.sv.
- Docs/wave.png — Waveform screenshot (visual reference).
- Docs/report.md — This file.

## Design Overview

### Datapath
- IF stage: Fetches instruction using PC, increments PC (pc + 4) by default, and provides instr/pc to IF/ID register.
- ID stage: Decodes instruction, reads register file, sign-extends immediates, detects branches.
- EX stage: ALU operations happen here. Operand A/B come from ID/EX or forwarded values from later pipeline stages using forwarding_unit.
- MEM stage: Accesses data memory for loads/stores. Generates mem-to-reg values for WB.
- WB stage: Writes back to the register file when RegWrite is asserted.

### Control
- controller.sv decodes opcode/funct fields and generates standard signals:
  - RegWrite, MemRead, MemWrite, MemToReg, ALU control, Branch, ALUSrc, etc.
- Control signals are propagated through pipeline registers; hazard_unit may override or insert bubbles when necessary.

### Forwarding & Hazard Unit
- forwarding_unit.sv drives forwardA/forwardB control so EX stage selects ALU operands from EX/MEM or MEM/WB results when appropriate.
- hazard_unit.sv detects load-use hazards (when EX-stage is a load and ID stage needs the loaded register) and generates stall/flush signals (e.g., stall PC, insert bubble into ID/EX).
- Branch handling: On taken branches, pipeline flushes/inserts as implemented in the design (verify flush signals in waveform).

## Testbench and Simulation
- Compile and run using Icarus Verilog:
  - iverilog -g2012 -o pipeline_sim tb/tb_pipeline.sv src/datapath.sv src/controller.sv src/forwarding_unit.sv src/hazard_unit.sv src/memory.sv src/riscvpipeline.sv
  - vvp pipeline_sim
- Generated waveform: `pipeline.vcd` (inspect with GTKWave)
  - gtkwave pipeline.vcd


## Waveform / Results / What to inspect
Open the waveform (Docs/wave.png or pipeline.vcd) and verify:
- Clock and cycle boundaries — how control signals propagate each cycle.
- Forwarding events: when an ALU operand is needed one cycle after it is calculated in EX/MEM or MEM/WB, forwarding muxes should select forwarded values (check forwardA/forwardB).
- Load-use stalls: when the instruction in EX is a load and the next instruction uses that register, the hazard unit should assert a stall for one cycle (PC/IF/ID should stall or ID/EX should be bubbled).
- Branch behavior: when Branch is taken, PC should update with branch target and wrong-path instructions should be flushed (control signals zeroed for flushed stage).
- Register writeback and memory accesses: confirm MemRead/MemWrite, RegWrite, and MemToReg behavior matches expected instruction semantics.

Key signals to add to GTKWave:
- Pipeline stage registers (IF/ID.*, ID/EX.*, EX/MEM.*, MEM/WB.*)
- Control signals: RegWrite, MemRead, MemWrite, MemToReg, ALUSrc, Branch
- Forwarding and hazard signals: forwardA, forwardB, stall, flush, pc_write
- ALU result, rdata1/rdata2, wdata, mem_rdata, pc, instruction bytes

## Observations (example expected)
- Forwarding resolves most dependent-ALU hazards (no stall when EX or MEM stage has the needed result).
- Load-use requires a 1-cycle stall: after a load, the following dependent instruction is delayed until the loaded data reaches MEM/WB or is forwarded.
- Branch resolution often occurs in ID or EX depending on implementation — expect 1-2 cycle penalty if branch decision is computed in EX.
- Verify register file writes occur in WB stage and are visible in subsequent cycles after writeback.

## Testcases covered
- Series of ALU-only dependent instructions (verify forwarding).
- Load followed by dependent instruction (verify load-use stall).
- Branch instructions exercising taken and not-taken paths (verify flush).
- Store and load operations to/from memory (verify MemWrite / MemRead).

## Known Issues / Limitations
- Limited instruction subset: only a portion of RV32I may be implemented.
- Branch prediction: none implemented — branch penalty is paid on taken branches.
- Memory model: simple synchronous/asynchronous memory used for simulation — adjust timing if co-simulation with a different memory model.
- Waveform readability: long traces can be dense. Use GTKWave time cursors and signal grouping.

## Future Work
- Add more RV32I instructions (CSR, atomic, multiply/divide).
- Add branch prediction or branch target buffer to reduce branch penalties.
- Expand testbench to random/auto-checking tests and include self-checking assertions.
- Add CI to automatically run simulations and produce artifact waveforms on push.


## Appendices

### A. Typical simulation commands
- Compile:
  - iverilog -g2012 -o pipeline_sim tb/tb_pipeline.sv src/datapath.sv src/controller.sv src/forwarding_unit.sv src/hazard_unit.sv src/memory.sv src/riscvpipeline.sv
- Run:
  - vvp pipeline_sim
- View:
  - gtkwave pipeline.vcd


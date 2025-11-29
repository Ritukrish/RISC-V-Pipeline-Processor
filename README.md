# RISC-V-Pipeline-Processor

This directory contains my work for Custom Pipelined CPU (RISC-V-Pipeline-Processor). Below you will find the purpose of the assignment, what's included, how to run the simulation, and links to the waveform and full project report.

---

## Objective
- Implement a pipelined RV32I-style datapath with control logic and forwarding/hazard handling.
- Verify the design using a testbench and inspect the signals with GTKWave to confirm correct pipeline behavior (stalling, forwarding, hazard resolution, PC updates, and register/write-back correctness).

---

## Files in this folder
- `README.md` — this file (overview & instructions).
- `waveform.png` — waveform screenshot for the pipelined design (GTKWave view).
- `Docs/REPORT.md` — detailed project report and design notes (see path below).
- (other source files / testbenches) — your Verilog/SystemVerilog sources, testbenches, scripts, and any other support files should also be placed in the repo (see `src/`, `tb/`).

Important paths (as shown in the repository file tree):
- Project report: RISC-V-Pipeline-Processor/Docs/REPORT.md
  - GitHub permalink: RISC-V-Pipeline-Processor/blob/main/Docs/REPORT.md
- Waveform image in Docs: RISC-V-Pipeline-Processor/Docs/wave.png
  - GitHub permalink: RISC-V-Pipeline-Processor/blob/main/Docs/wave.png
- Sources: 
  - `src/datapath.sv`
  - `src/controller.sv`
  - `src/forwarding_unit.sv`
  - `src/hazard_unit.sv`
  - `src/memory.sv`
  - `src/riscvpipeline.sv`
- Testbenches:
  - `tb/tb_pipeline.sv`
  - `tb/tb_modular.sv`

---

## How to run the simulation (Icarus Verilog + GTKWave)
1. From the repository root, compile:
   iverilog -g2012 -o pipeline_sim tb/tb_pipeline.sv src/datapath.sv src/controller.sv src/forwarding_unit.sv src/hazard_unit.sv src/memory.sv src/riscvpipeline.sv

2. Run the simulation:
   vvp pipeline_sim

3. View the waveform:
   gtkwave pipeline.vcd

---

## Report and Waveform (Docs)
A comprehensive project report and the waveform image are stored under the Docs directory inside this RISC-V-Pipeline-Processor folder. You can view them on GitHub or open locally:

- REPORT: RISC-V-Pipeline-Processor/Docs/REPORT.md
  - Use the GitHub link above or open the file locally.
- Waveform image: RISC-V-Pipeline-Processor/Docs/wave.png

These files include design rationale, test procedures, waveform screenshots, and notes about forwarding, hazards, and observed behavior in simulation.

---

## Notes & Tips for GTKWave inspection
- Add these signals to the waveform view:
  - Pipeline registers between stages (IF/ID, ID/EX, EX/MEM, MEM/WB)
  - Control signals: RegWrite, MemRead, MemWrite, MemToReg, ALU control, Branch
  - Forwarding signals: forwardA, forwardB
  - Hazard signals: stall, pc_write, flush
  - ALU results, register file read data, memory rdata/wdata, PC, instruction fields
- Use GTKWave cursors to compare values at cycle boundaries and to inspect forwarding/stall cycles..

---

## Author / Submission Info
Name: Ritu Kumari
Designation: Research Scholar, CSE, IITG

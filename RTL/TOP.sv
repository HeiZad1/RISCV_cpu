

// RISC-V  processor
// 2024.2
// songwenfei999@gmail.com

// run 210
// Expect simulator to print "Simulation succeeded"
// when the value 25 (0x19) is written to address 100 (0x64)





module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic [31:0] PC, Instr, ReadData;
  
  // instantiate processor and memories
  riscv rv(clk, reset, PC, Instr, MemWrite, DataAdr, 
                       WriteData, ReadData);
  imem imem(PC, Instr);
  dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

module riscv(input  logic        clk, reset,
                   output logic [31:0] PCF,
                   input  logic [31:0] Instr,
                   output logic        MemWrite,
                   output logic [31:0] ALUResultM, WriteDataM,
                   input  logic [31:0] ReadData);

  logic       ALUSrcE, RegWriteW,RegWriteM, JumpE, Zero,stallD,stallF,FlushD,FlushE,ResultSrcE0;
  logic [1:0] ResultSrcW, ImmSrcD;
  logic [2:0] ALUControlE;
  logic [1:0] ForWordAE,ForWordBE;
  logic [4:0] RdW,RdM,RdE,Rs1D,Rs2D,Rs1E,Rs2E;
  logic [31:0] InstrD;

  controller c(clk,reset,InstrD[6:0], InstrD[14:12], InstrD[30], Zero,ResultSrcE0,
               ResultSrcW, MemWrite, PCSrcE,
               ALUSrcE, RegWriteW, JumpE,RegWriteM,
               ImmSrcD, ALUControlE,FlushE);
  datapath dp(clk, reset, ResultSrcW, PCSrcE,
              ALUSrcE, RegWriteW,
              ImmSrcD, ALUControlE,
              Zero, PCF, Instr,
              ALUResultM,WriteDataM, ReadData,
			  stallD,stallF,FlushD,FlushE,ForWordAE,ForWordBE,
			  RdW,RdM,RdE,
			  Rs1D,Rs2D,Rs1E,Rs2E,InstrD
			  );
			  
  HazardUnit hu(RegWriteW,RegWriteM,RdW,RdM,RdE,
			    ResultSrcE0,PCSrcE,  
			    Rs1E,Rs2E,Rs1D,Rs2D,
			    ForWordAE,ForWordBE,
				FlushE,FlushD,stallF,stallD
				);
endmodule


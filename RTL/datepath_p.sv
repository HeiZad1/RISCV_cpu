

// RISC-V  processor
// 2024.2
// songwenfei999@gmail.com


// run 210
// Expect simulator to print "Simulation succeeded"
// when the value 25 (0x19) is written to address 100 (0x64)



// Implements a subset of the base integer instructions:
//    lw, sw
//    add, sub, and, or, slt, 
//    addi, andi, ori, slti
//    beq
//    jal
// Exceptions, traps, and interrupts not implemented
// little-endian memory

// 31 32-bit registers x1-x31, x0 hardwired to 0
// R-Type instructions
//   add, sub, and, or, slt ,xor
//   INSTR rd, rs1, rs2
//   Instr[31:25] = funct7 (funct7b5 & opb5 = 1 for sub, 0 for others)
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// I-Type Instructions
//   lw, I-type ALU (addi, andi, ori, slti)
//   lw:         INSTR rd, imm(rs1)
//   I-type ALU: INSTR rd, rs1, imm (12-bit signed)
//   Instr[31:20] = imm[11:0]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// S-Type Instruction
//   sw rs2, imm(rs1) (store rs2 into address specified by rs1 + immm)
//   Instr[31:25] = imm[11:5] (offset[11:5])
//   Instr[24:20] = rs2 (src)
//   Instr[19:15] = rs1 (base)
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:0]  (offset[4:0])
//   Instr[6:0]   = opcode
// B-Type Instruction
//   beq rs1, rs2, imm (PCTarget = PC + (signed imm x 2))
//   Instr[31:25] = imm[12], imm[10:5]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:1], imm[11]
//   Instr[6:0]   = opcode
// J-Type Instruction
//   jal rd, imm  (signed imm is multiplied by 2 and added to PC, rd = PC+4)
//   Instr[31:12] = imm[20], imm[10:1], imm[11], imm[19:12]
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// u-Type Instruction
//	 lui          
//	 Instr[31:12] = imm[31:12]
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode


//   Instruction  opcode    funct3    funct7
//   add          0110011   000       0000000
//   sub          0110011   000       0100000
//   and          0110011   111       0000000
//   or           0110011   110       0000000
//   slt          0110011   010       0000000
//   addi         0010011   000       immediate
//   andi         0010011   111       immediate
//   ori          0010011   110       immediate
//   slti         0010011   010       immediate
//   beq          1100011   000       immediate
//   lw	          0000011   010       immediate
//   sw           0100011   010       immediate
//   jal          1101111   immediate immediate
//   lui          0110111   immediate imm
//   xor          0110011   100       0000000










module datapath(input  logic        clk, reset,
                input  logic [1:0]  ResultSrc, 
                input  logic        PCSrc, ALUSrc,
                input  logic        RegWrite,
                input  logic [1:0]  ImmSrc,
                input  logic [2:0]  ALUControl,
                output logic        Zero,
                output logic [31:0] PCF,
                input  logic [31:0] Instr,
                output logic [31:0] ALUResultM,WriteDataM,
                input  logic [31:0] ReadData,
				input  logic        stallD,stallF,
				input  logic        FlushD,FlushE,
				input  logic [ 1:0] ForWordAE,ForWordBE,

				output logic [ 4:0] RdW,RdM,RdE,
				output logic [ 4:0] Rs1D,Rs2D,Rs1E,Rs2E,
				output logic [31:0] InstrD	
				);

  logic [31:0] PCFNext, PCPlus4F, PCTargetE,PCD,PCPlus4D,PCE,PCPlus4E,PCPlus4M,PCPlus4W;
  logic [31:0] ImmExtD,ImmExtE,ALUResult,ALUResultW;
  logic [31:0] SrcAE, SrcBE,RD1D,RD2D,RD1E,RD2E,RdD;
  logic [31:0] ResultW,WriteDataE,ReadDataW;

  assign       Rs1D = InstrD[19:15];
  assign       Rs2D = InstrD[24:20];
  assign       RdD  = InstrD[11: 7];
  
  

  // next PC logic
  enflopr #(32) pcreg(clk, reset, ~stallF,PCFNext, PCF);  
  adder         pcadd4(PCF, 32'd4, PCPlus4F);
  adder         pcaddbranch(PCE, ImmExtE, PCTargetE);
  mux2 #(32)    pcmux(PCPlus4F, PCTargetE, PCSrc, PCFNext);
 
  // register file logic
  regfile     rf(~clk, RegWrite, InstrD[19:15], InstrD[24:20], //接口待修改
                 RdW, ResultW,RD1D ,RD2D );
  extend      ext(InstrD[31:7], ImmSrc, ImmExtD);
 // mux2 #(32)    fwdmux(RD2, ResultW,ForWordD , RD2D);
  
  

  // ALU logic
  mux3 #(32)  srcAmux(RD1E, ResultW , ALUResultM, ForWordAE, SrcAE);
  mux3 #(32)  srcBmux(RD2E, ResultW, ALUResultM, ForWordBE, WriteDataE);
  mux2 #(32)  srcbmux(WriteDataE, ImmExtE, ALUSrc, SrcBE);
  alu         alu(SrcAE, SrcBE, ALUControl, ALUResult, Zero);   //ALUResult输出端口
  
  
  // regD
	enfloprsind #(32) Dreg1(clk, open ,FlushD, ~stallD,Instr, InstrD);
	enfloprs #(32) Dreg2(clk,  reset,FlushD , ~stallD,PCF, PCD);
	enfloprspc4 #(32) Dreg3(clk, open ,FlushD, ~stallD,PCPlus4F, PCPlus4D);
	
	//regE
	floprs #(32) Ereg1(clk, reset, FlushE, PCD, PCE);
	floprs #( 5) Ereg2(clk,  reset, FlushE, Rs1D, Rs1E);
	floprs #( 5) Ereg3(clk, reset, FlushE, Rs2D, Rs2E);
	floprs #( 5) Ereg4(clk,  reset, FlushE, RdD, RdE );
	floprs #(32) Ereg5(clk,  reset, FlushE, ImmExtD, ImmExtE);
	floprs #(32) Ereg6(clk,  reset, FlushE, PCPlus4D, PCPlus4E);
	floprs #(32) Ereg7(clk,  reset, FlushE, RD2D , RD2E);
	floprs #(32) Ereg8(clk,  reset, FlushE, RD1D, RD1E);
	
	//regM
	flopr #(32) Mreg1(clk, reset , ALUResult, ALUResultM);
	flopr #(32) Mreg2(clk, reset , WriteDataE,WriteDataM);
	flopr #( 5) Mreg3(clk, reset , RdE, RdM);
	flopr #(32) Mreg4(clk, reset , PCPlus4E, PCPlus4M);
	
	//RegW
	flopr #(32) Wreg1(clk, reset , ALUResultM, ALUResultW);
	flopr #(32) Wreg2(clk, reset , ReadData, ReadDataW);
	flopr #( 5) Wreg3(clk, reset , RdM, RdW);
	flopr #(32) Wreg4(clk, reset , PCPlus4M, PCPlus4W);
	
	//RES
	mux3 #(32)  resultmux(ALUResultW, ReadDataW, PCPlus4W, ResultSrc, ResultW);
	
  
endmodule

module regfile(input  logic        clk,
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(negedge clk)
    if (we3 & (a3 !=0)) rf[a3] <= wd3;	
  assign rd1 = ((we3 && (a1 == a3)) ? wd3 : (a1 != 0) ? rf[a1] : 0);
  assign rd2 = ((we3 && (a2 == a3)) ? wd3 : (a2 != 0) ? rf[a2] : 0);
	 
    
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
               // I-type 
      2'b00:   immext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
               // B-type (branches)
      2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
               // J-type (jal)
      2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
      default: immext = 32'bx; // undefined
    endcase             
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module floprs #(parameter WIDTH = 8)
              (input  logic             clk, reset,clr,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
	else if (clr) q <= 0;
    else       q <= d;
endmodule

module enflopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,en,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)         q <= 0;
    else if (en)       q <= d;
endmodule



module enfloprs #(parameter WIDTH = 8)
              (input  logic             clk, reset,clr,en,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)         q <= 0;
	else if(clr)		q <= 0;
    else if (en)       q <= d;
endmodule

module enfloprsind #(parameter WIDTH = 8)
              (input  logic             clk, reset,clr,en,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)         q <= 0;
	else if(clr)		q <= 32'H00000013;
    else if (en)       q <= d;
endmodule

module enfloprspc4 #(parameter WIDTH = 8)
              (input  logic             clk, reset,clr,en,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset)         q <= 32'H00000004;
	else if(clr)		q <= 32'H00000004;
    else if (en)       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module imem(input  logic [31:0] a,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  initial
      $readmemh("riscvtest.txt",RAM);

  assign rd = RAM[a[31:2]]; // word aligned
endmodule

module dmem(input  logic        clk, we,
            input  logic [31:0] a, wd,
            output logic [31:0] rd);

  logic [31:0] RAM[63:0];

  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;
  logic        v;              // overflow
  logic        isAddSub;       // true when is add or subtract operation

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];
  assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                    ~alucontrol[1] & alucontrol[0];

  always_comb
    case (alucontrol)
      3'b000:  result = sum;         // add
      3'b001:  result = sum;         // subtract
      3'b010:  result = a & b;       // and
      3'b011:  result = a | b;       // or
      3'b100:  result = a ^ b;       // xor
      3'b101:  result = sum[31] ^ v; // slt
      3'b110:  result = a << b[4:0]; // sll
      3'b111:  result = a >> b[4:0]; // srl
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
  assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
  
endmodule



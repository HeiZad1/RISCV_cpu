module controller(
				  input  logic       clk,reset,
				  input  logic [6:0] op,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       ZeroE,
				  output logic       ResultSrcE0,
                  output logic [1:0] ResultSrcW,
                  output logic       MemWriteM,
                  output logic       PCSrcE, ALUSrcE,
                  output logic       RegWriteW, JumpE,RegWriteM,   //?????
                  output logic [1:0] ImmSrcD,
                  output logic [2:0] ALUControlE,
				  input  logic       FlushE);

    logic [1:0] ALUOp,ResultSrcD,ResultSrcE,ResultSrcM;
    logic       BranchD,JumpD,ALUSrcD,BranchE;
	logic [2:0] ALUControlD;
	logic       RegWriteE,RegWriteD;
	logic       MemWriteE,MemWriteD;
	
	
	
  maindec md(op, ResultSrcD, MemWriteD, BranchD,
             ALUSrcD, RegWriteD, JumpD, ImmSrcD, ALUOp);
  aludec  ad(op[5], funct3, funct7b5, ALUOp, ALUControlD);
  
  //REGE
  
  floprs #( 1) Ereg9 (clk, reset, FlushE, RegWriteD, RegWriteE);
  floprs #( 2) Ereg10(clk, reset, FlushE , ResultSrcD, ResultSrcE);
  
  
  floprs #( 1) Ereg11(clk,  reset, FlushE, MemWriteD, MemWriteE);
  floprs #( 1) Ereg12(clk,  reset, FlushE, JumpD, JumpE);
  floprs #( 1) Ereg13(clk,  reset, FlushE, BranchD, BranchE);
  floprs #( 3) Ereg14(clk,  reset, FlushE, ALUControlD, ALUControlE);
  floprs #( 1) Ereg15(clk,  reset, FlushE, ALUSrcD, ALUSrcE);
  
  //REGM
  flopr #( 1) Mreg5 (clk, reset , RegWriteE, RegWriteM);
  flopr #( 2) Mreg6 (clk, reset , ResultSrcE, ResultSrcM);
  flopr #( 1) Mreg7 (clk, reset , MemWriteE, MemWriteM);
  
  //REGW
  flopr #( 1) Wreg5 (clk, reset , RegWriteM, RegWriteW);
  flopr #( 2) Wreg6 (clk, reset , ResultSrcM, ResultSrcW);
  
  

  assign PCSrcE = (BranchE & ZeroE) | JumpE;
  assign      ResultSrcE0 = ResultSrcE[0];
endmodule


module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrcD,
               output logic       MemWriteD,
               output logic       BranchD, ALUSrcD,
               output logic       RegWriteD, JumpD,
               output logic [1:0] ImmSrcD,
               output logic [1:0] ALUOp);

  logic [10:0] controls;

  assign {RegWriteD, ImmSrcD, ALUSrcD, MemWriteD,
          ResultSrcD, BranchD, ALUOp, JumpD} = controls;

  always_comb
    case(op)
    // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
      7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
      7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
      7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type 
      7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
      7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
      7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
      default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // non-implemented instruction
    endcase
endmodule

module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [2:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 3'b000; // addition
      2'b01:                ALUControl = 3'b001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl = 3'b001; // sub
                          else          
                            ALUControl = 3'b000; // add, addi
                 3'b010:    ALUControl = 3'b101; // slt, slti
				 3'b100:    ALUControl = 3'b100; // xor
                 3'b110:    ALUControl = 3'b011; // or, ori
                 3'b111:    ALUControl = 3'b010; // and, andi
                 default:   ALUControl = 3'bxxx; // ???
               endcase
    endcase
endmodule
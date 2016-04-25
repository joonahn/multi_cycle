`timescale 1ns/1ns
`define WORD_SIZE 16    // data and address word size			  
//JYP

module cpu(Clk, Reset_N, readM, writeM, address, data, num_inst, output_port, is_halted);
   input Clk;
   wire Clk;
   input Reset_N;
   wire Reset_N;
   

   output readM;
   wire readM;
   output writeM;
   wire writeM;
   output [`WORD_SIZE-1:0] address;
   wire [`WORD_SIZE-1:0] address;

   inout [`WORD_SIZE-1:0] data;
   wire [`WORD_SIZE-1:0] data;

   output [`WORD_SIZE-1:0] num_inst;
   wire [`WORD_SIZE-1:0] num_inst;
   output [`WORD_SIZE-1:0] output_port;
   wire [`WORD_SIZE-1:0] output_port;
   output is_halted;
   wire is_halted;

   Control congen(Clk, instr, MemRead, REgDst, SavePc, RegWrite, ExtWay, AlUSrc, MemRead, MemWrite, MemtoReg, Branch, JRLJPR, Jump, IorD, PVSWriteEnPC, PVSWriteEnReg, PVSWriteEnMem, Reset_N, is_halted);
   Datapath dpth(Clk, Reset_N, PVSWriteEnMem, PVSWriteEnReg, PVSWriteEnPC, RegWrite, RegDst, IorD, MemRead, MemWrite, ALUop, ALUSrc, SavePC, MemtoReg, ExtWay, Branch, JRLJPR, Jump, inst, output_port);

   // TODO : Implement your multi-cycle CPU!

endmodule


module ALU(A, B,ALUcontrol, out, bcond);
   input wire[`WORD_SIZE-1:0] A;
   input wire[`WORD_SIZE-1:0] B;
   input wire[5:0] ALUcontrol;
   output reg[`WORD_SIZE-1:0] out;
   output reg bcond;

   always @(A or B) begin
      case (ALUcontrol)
         0: begin
            out <= A+B;
         end
         1: begin
            out <= A + (~B) + 1;
         end
         2: begin
            out <= A & B;
         end
         3: begin
            out <= A | B;
         end
         4: begin
            out <= (~A);
         end
         5: begin
            out <= (~A) + 1;
         end
         6: begin
            out <= A << 1;
         end
         7: begin
            out <= A >> 1;
         end
         8: begin
            out <= A << 8;
         end
         9: begin
            out <= A;
         end
         10: begin
            bcond <= (A!=B);
         end
         11: begin
            bcond <= (A==B);
         end
         12: begin
            bcond <= (A>0);
         end
         13: begin
            bcond <= (A<0);
         end
      endcase
   end

endmodule

module NumberExtender(in, SignExt, ZeroExt);
   input wire[(`WORD_SIZE/2)-1:0] in;
   output wire[`WORD_SIZE-1:0] SignExt;
   output wire[`WORD_SIZE-1:0] ZeroExt;

   assign SignExt = (in[(`WORD_SIZE/2)-1] == 1) ? {8'hFFFF, in} : {8'h0000, in};
   assign ZeroExt = {8'h0000, in};

endmodule

module RF(readregA, readregB, writereg, writedata, RegWrite, regoutA, regoutB, PVSWriteEn);

   input [1:0]readregA;
   input [1:0]readregB;
   input [1:0]writereg;
   input [`WORD_SIZE-1:0]writedata;
   input RegWrite;
   input PVSWriteEn;
   output reg [`WORD_SIZE-1:0]regoutA;
   output reg [`WORD_SIZE-1:0]regoutB;
   reg [`WORD_SIZE-1:0]r0;
   reg [`WORD_SIZE-1:0]r1;
   reg [`WORD_SIZE-1:0]r2;
   reg [`WORD_SIZE-1:0]r3;

   always @(readregA, readregB)
   begin
      case(readregA)
      0:  regoutA=r0;
      1:  regoutA=r1;
      2:  regoutA=r2;
      3:  regoutA=r3;
      endcase

      case(readregB)
      0:  regoutB=r0;
      1:  regoutB=r1;
      2:  regoutB=r2;
      3:  regoutB=r3;
      endcase
   end

   always @(writereg)
   begin
      if(RegWrite==1)
      begin
         @(posedge PVSWriteEn)
         begin
            case(writereg)
            0:  r0=writedata;
            1:  r1=writedata;
            2:  r2=writedata;
            3:  r3=writedata;
            endcase
         end

      end
   end

endmodule

module PC(in, out, PVSWriteEn);
   input wire [`WORD_SIZE-1:0] in;
   input wire PVSWriteEn;
   output reg [`WORD_SIZE-1:0] out;

   always @(PVSWriteEn)
   begin
      out <= in;
   end

endmodule

module Control(clk, instr,
   MemRead, RegDst, SavePC, RegWrite, ExtWay, ALUSrc, MemRead, MemWrite, MemtoReg, Branch, JRLJPR, Jump, IorD,
   PVSWriteEnPC, PVSWriteEnReg, PVSWriteEnMem, Reset_N, is_halted);
   input wire clk;
   input wire[`WORD_SIZE-1:0] instr;
   input wire Reset_N;
   output reg is_halted;
   output reg MemRead;
   output reg RegDst;
   output reg SavePC;
   output reg RegWrite;
   output reg ExtWay;
   output reg ALUSrc;
   output reg MemWrite;
   output reg MemtoReg;
   output reg Branch;
   output reg JRLJPR;
   output reg Jump;
   output reg IorD;

   output reg PVSWriteEnPC;
   output reg PVSWriteEnReg;
   output reg PVSWriteEnMem;

   reg [5:0]state;

   // Control state initialization
   always @(negedge Reset_N)
   begin
      state <= 0;
      MemRead <= 0;
      RegDst <= 0;
      SavePC <= 0;
      RegWrite <= 0;
      ExtWay <= 0;
      ALUSrc <= 0;
      MemWrite <= 0;
      MemtoReg <= 0;
      Branch <= 0;
      JRLJPR <= 0;
      Jump <= 0;
      IorD <= 0;
      is_halted <= 0;
   end

   // Moore Machine state output
   always @(state)
   begin
      case(state)
         0: begin
            MemRead <=1;
            IorD <= 1;
            MemWrite <= 0;
            RegWrite <= 0;
            Branch <= 0;
            JRLJPR <= 0;
            Jump <= 0;
         end
         1: begin
            // R-type ID
            RegDst <= 1;
            SavePC <= 1;
         end
         2: begin
            // R-type EX
            ALUSrc <= 1;
         end
         3: begin
            // R-type WB
            MemtoReg <= 0;
            RegWrite <= 1;
         end
         4: begin
            // I-type ID
            RegDst <= 0;
            SavePC <=0;
         end
         5: begin
            // I-type EX
            if(instr[`WORD_SIZE-1:`WORD_SIZE-4] == 5)
            begin
               ExtWay <= 0;
            end
            else begin
               ExtWay <= 1;
            end
            ALUSrc <= 1;
         end
         6: begin
            // I-type WB
            MemtoReg <= 0;
            RegWrite <= 1;
         end
         7: begin
            // LW ID
            RegDst <= 0;
            SavePC <= 0;
         end
         8: begin
            // LW Ex
            ExtWay <= 1;
            ALUSrc <= 1;
         end
         9: begin
            // LW MEM
            MemRead <= 1;
         end
         10: begin
            // LW WB
            MemtoReg <= 1;
            RegWrite <= 1;
         end
         11: begin
            // WWD EX
         end
         12: begin
            // SW ID
            RegDst <= 0;
            SavePC <= 0;
         end
         13: begin
            // SW EX
            ExtWay <= 1;
            ALUSrc <= 1;
         end
         14: begin
            // SW MEM
            MemWrite <= 1;
         end
         15: begin
            // BR EX
            ExtWay <= 1;
            ALUSrc <= 0;
            Branch <= 1;
            JRLJPR <= 0;
            Jump <= 0;
         end
         16: begin
            // JAL WB
            SavePC <= 1;
            RegWrite <= 1;
         end
         17: begin
            // JRL WB
            SavePC <= 1;
            RegWrite <= 1;
            JRLJPR <= 1;
            Jump <= 0;
         end
      endcase
   end

   // PVSWriteEn signal delay
   always @(PVSWriteEnMem or PVSWriteEnReg or PVSWriteEnPC) begin
      #5;
      PVSWriteEnPC <= 0;
      PVSWriteEnReg <= 0;
      PVSWriteEnMem <= 0;
   end

   // Transition
   always @(posedge clk) begin
      IorD <= 0;
      case (state)
         0: begin
         // IF stage of ALL INSTRUCTIONS
            case (instr[`WORD_SIZE-1:`WORD_SIZE-4] == 5)
               0: begin
                  // Branch
                  state <= 15;
                  end
               1: begin
                  // Branch
                  state <= 15;
                  end
               2: begin
                  // Branch
                  state <= 15;
                  end
               3: begin
                  // Branch
                  state <= 15;
                  end
               4:
                begin
                  // I-type ALU
                  state <= 4;
               end
               5:
                begin
                  // I-type ALU
                  state <= 4;
               end
               6:
                begin
                  // I-type ALU
                  state <= 4;
               end
               7: begin
                  // LWD
                  state <= 7;
               end
               8: begin
                  // SWD
                  state <= 8;
               end
               9: begin
                  // JMP
                  PVSWriteEnPC <= 1;
               end
               10: begin
                  // JAL
                  state <= 16;
               end
               15: begin
                  // switch by FUNCT
                  case(instr[5:0])
                     0:
                     begin
                        // R-type ALU
                        state <= 1; 
                     end
                     1:
                     begin
                        // R-type ALU
                        state <= 1; 
                     end
                     2:
                     begin
                        // R-type ALU
                        state <= 1; 
                     end
                     3:
                     begin
                        // R-type ALU
                        state <= 1; 
                     end
                     4:
                     begin
                        // R-type ALU
                        state <= 1; 
                     end
                     5:
                     begin
                        // R-type ALU
                        state <= 1; 
                     end
                     6:
                     begin
                        // R-type ALU
                        state <= 1; 
                     end
                     7: 
                     begin
                        // R-type ALU
                        state <= 1; 
                     end
                     26: 
                     begin
                        // JRL
                        state <=17;
                     end
                     29: begin
                        // SYSTEM HALTS
                        is_halted <= 1;
                     end
                     28: 
                     begin
                        // WWD
                        PVSWriteEnPC <= 1;
                     end

                  endcase
               end

            endcase
         end
         1: begin
            state <= 2;
         end
         2: begin
            state <= 3;
         end
         3: begin
            // R-type ALU WB -> IF
            state <= 0;
            PVSWriteEnReg <= 1;
            PVSWriteEnPC <= 1;
         end
         4: begin
            // I-type ALU ID -> EX
            state <= 5;
         end
         5: begin
            state <= 6;
         end
         6: begin
            // I-type ALU WB -> IF
            state <= 0;
            PVSWriteEnReg <= 1;
            PVSWriteEnPC <= 1;
         end
         7: begin
            // LW ID -> EX
            state <= 8;
         end
         8: begin
            state <= 9;
         end
         9: begin
            state <= 10;
         end
         10: begin
            PVSWriteEnReg <= 1;
            PVSWriteEnPC <= 1;
            state <= 0;
         end
         11: begin
            state <= 0;
         end
         12: begin
            state <= 13;
         end
         13: begin
            state <= 14;
         end
         14: begin
            PVSWriteEnMem <= 1;
            PVSWriteEnPC <= 1;
            state <= 0;
         end
         15: begin
            // Br EX -> IF
            PVSWriteEnPC <= 1;
            state <= 0;
         end
         16: begin
            // JAL WB -> IF
            PVSWriteEnReg <= 1;
            PVSWriteEnPC <= 1;
            state <= 0;
         end
         17: begin
            // JRL WB -> IF
            PVSWriteEnReg <= 1;
            state <= 0;
         end
      endcase
   end

endmodule

module ALUcontrol(instr, out);

   input wire[`WORD_SIZE-1:0] instr;
   output reg[3:0] out;

   wire[3:0] OPcode;
   wire[5:0] Funct;

   assign OPcode = instr[`WORD_SIZE-1:`WORD_SIZE-4];
   assign Funct = instr[5:0];

   always @(OPcode or Funct) begin
      case (OPcode)
         0: begin
            // BNE
            out <= 10;
         end
         1: begin
            // BEQ
            out <= 11;
         end
         2: begin
            // BGZ
            out <= 12;
         end
         3: begin
            // BLZ
            out <= 13;
         end
         4: begin
            // ADI
            out <= 0;
         end
         5: begin
            // ORI
            out <= 3;
         end
         6: begin
            // LHI
            out <= 8;
         end
         7: begin
            // LWD
            out <= 0;
         end
         8: begin
            // SWD
            out <= 0;
         end
         15: begin
            out <= Funct;
         end
      endcase
   end

endmodule

module Datapath(clk, Reset_N, PVSWriteEnMem, PVSWriteEnReg, PVSWriteEnPC,
   RegWrite, RegDst, IorD, MemRead, MemWrite, ALUop, ALUSrc, SavePC, MemtoReg, ExtWay, Branch, JRLJPR, Jump, inst, output_port);

   input wire clk;
   input wire Reset_N;
   input wire PVSWriteEnMem;
   input wire PVSWriteEnReg;
   input wire PVSWriteEnPC;
   input wire RegWrite;
   input wire RegDst;
   input wire IorD;
   input wire MemRead;
   input wire MemWrite;
   input wire ALUop;
   input wire ALUSrc;
   input wire SavePC;
   input wire MemtoReg;
   input wire ExtWay;
   input wire Branch;
   input wire JRLJPR;
   input wire Jump;
   output wire [`WORD_SIZE-1:0]inst;
   output reg output_port;
   
   wire [`WORD_SIZE-1:0]PCOut;
   wire [`WORD_SIZE-1:0]PCplus4;
   wire [`WORD_SIZE-1:0]branchPC;
   wire [`WORD_SIZE-1:0]RSneqPC;
   wire [`WORD_SIZE-1:0]RSeqPC;
   wire [`WORD_SIZE-1:0]jumpPC;
   wire [`WORD_SIZE-1:0]njumpPC;
   wire [`WORD_SIZE-1:0]PCIn;
   wire [`WORD_SIZE-1:0]MemAdrsI;
   wire [`WORD_SIZE-1:0]MemAdrsD;
   wire [`WORD_SIZE-1:0]MemAdrsSel;
   wire [`WORD_SIZE-1:0]MemData;
   wire [1:0]RFRName1;
   wire [1:0]RFRName2;
   wire [1:0]RFWNotSavePC;
   wire [1:0]RFWName;
   wire [`WORD_SIZE-1:0]RFWData;
   wire [`WORD_SIZE-1:0]RFRData1;
   wire [`WORD_SIZE-1:0]RFRData2;
   wire [`WORD_SIZE-1:0]signExt;
   wire [`WORD_SIZE-1:0]zeroExt;
   wire [`WORD_SIZE-1:0]ExtendedNum;
   wire [`WORD_SIZE-1:0]ALUinputA;
   wire [`WORD_SIZE-1:0]ALUinputB;
   wire [`WORD_SIZE-1:0]ALUoutput;
   wire [`WORD_SIZE-1:0]WBData;
   wire [3:0]ALUConOut;

   reg [`WORD_SIZE-1:0]instbuf;

   assign PCplus4=PCOut+4;
   assign branchPC=PCplus4+ExtendedNum;
   assign RSneqPC=(Branch&&bcond)? branchPC : PCplus4;
   assign RSeqPC=RFRData1;
   assign jumpPC=PCplus4[15:12]||inst[11:0];
   assign njumpPC=JRLJPR ? RSeqPC : RSneqPC;
   assign PCIn=Jump ? jumpPC : njumpPC;
   assign MemAdrsI=PCOut;
   assign RFRName1=inst[11:10];
   assign RFRName2=inst[9:8];
   assign RFWNotSavePC=RegDst ? inst[7:6] : inst[9:8];
   assign RFWName=SavePC ? 2 : RFWNotSavePC;
   assign RFWData=SavePC ? PCplus4 : WBData;
   assign ExtendedNum=ExtWay ? signExt : zeroExt;
   assign ALUinputA=RFRData1;
   assign ALUinputB=ALUSrc ? ExtendedNum : RFRData2;
   assign MemAdrsD=ALUoutput;
   assign MemAdrsSel=IorD ? MemAdrsI : MemAdrsD;
   assign MemData=IorD ? 16'hzzzz : RFRData2;
   assign inst = IorD ? MemData : 16'hzzzz;
   assign WB = IorD ? 16'hzzzz : MemData;

   always @(inst) begin
      instbuf=inst;
   end

   PC pcounter(PCIn, PCOut, PVSWriteEnPC);
   Memory memory(clk, Reset_N, MemRead, MemWrite, MemAdrsSel, MemData);
   ALUcontrol aluCon(inst, ALUConOut);
   ALU alu(ALUinputA, ALUinputB, ALUConOut, bcond);
   NumberExtender ext(inst[7:0], signExt, zeroExt);
   RF regfile(RFRName1, RFRName2, RFWName, RFWData, RegWrite, RFRData1, RFRData2, PVSWriteEnReg);
endmodule
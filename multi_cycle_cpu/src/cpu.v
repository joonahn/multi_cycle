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

   wire [`WORD_SIZE-1:0] inst;
   wire [3:0] ALUop;


   // Control Module Definition
   Control congen(Clk, inst, readM, RegDst, SavePC, RegWrite, ExtWay, ALUSrc,
      writeM, MemtoReg, Branch, JRLJPR, Jump, IorD, 
      PVSWriteEnPC, PVSWriteEnReg, PVSWriteEnMem, Reset_N, is_halted, num_inst);

   // Datapath Module Definition
   Datapath dpth(Clk, Reset_N, PVSWriteEnMem, PVSWriteEnReg, PVSWriteEnPC, 
      RegWrite, RegDst, IorD, readM, writeM, ALUop, ALUSrc, SavePC, 
      MemtoReg, ExtWay, Branch, JRLJPR, Jump, inst, output_port, address, data);
endmodule


module ALU(A, B,ALUcontrol, out, bcond);
   input wire[`WORD_SIZE-1:0] A;
   input wire[`WORD_SIZE-1:0] B;
   input wire[3:0] ALUcontrol;
   output wire[`WORD_SIZE-1:0] out;
   output wire bcond;

   assign out = 
      (ALUcontrol==0)? A+B :
      (ALUcontrol==1)? A + (~B) + 1 :
      (ALUcontrol==2)? A & B :
      (ALUcontrol==3)? A | B :
      (ALUcontrol==4)? (~A) :
      (ALUcontrol==5)? (~A) + 1 :
      (ALUcontrol==6)? A << 1 :
      (ALUcontrol==7)? A >> 1 :
      (ALUcontrol==8)? B << 8 :
      (ALUcontrol==9)? A : 0;

   assign bcond =
      (ALUcontrol==10)? (A!=B) :
      (ALUcontrol==11)? (A==B) :
      (ALUcontrol==12)? (A<16'h8000 && A>0) :
      (ALUcontrol==13)? (A>16'h7FFF) : 0;

endmodule

module NumberExtender(in, SignExt, ZeroExt);
   input wire[(`WORD_SIZE/2)-1:0] in;
   output wire[`WORD_SIZE-1:0] SignExt;
   output wire[`WORD_SIZE-1:0] ZeroExt;

   assign SignExt = (in[(`WORD_SIZE/2)-1] == 1) ? {8'hFF, in} : {8'h00, in};
   assign ZeroExt = {8'h00, in};
endmodule

module RF(readregA, readregB, writereg, writedata, RegWrite, regoutA, regoutB, PVSWriteEn, clk);

   input [1:0]readregA;
   input [1:0]readregB;
   input [1:0]writereg;
   input [`WORD_SIZE-1:0]writedata;
   input RegWrite;
   input PVSWriteEn;
   input wire clk;
   output wire [`WORD_SIZE-1:0]regoutA;
   output wire [`WORD_SIZE-1:0]regoutB;
   reg [`WORD_SIZE-1:0]r0;
   reg [`WORD_SIZE-1:0]r1;
   reg [`WORD_SIZE-1:0]r2;
   reg [`WORD_SIZE-1:0]r3;

   assign regoutA = (readregA==0)?r0:((readregA==1)?r1:((readregA==2)?r2:r3));
   assign regoutB = (readregB==0)?r0:((readregB==1)?r1:((readregB==2)?r2:r3));

   always @(posedge clk) begin
      if(PVSWriteEn)
      begin
         case(writereg)
         0:  r0=writedata;
         1:  r1=writedata;
         2:  r2=writedata;
         3:  r3=writedata;
         endcase
      end
   end

endmodule

module PC(in, out, PVSWriteEn, Reset_N, clk);
   input wire [`WORD_SIZE-1:0] in;
   input wire PVSWriteEn;
   input wire Reset_N;
   input wire clk;
   output reg [`WORD_SIZE-1:0] out;

   always @(posedge Reset_N)
   begin
      out <= 0;
   end


   always @(posedge clk)
   begin
      if(PVSWriteEn)
      begin
         out <= in;
      end
   end
endmodule

module Control(clk, instr,
   MemRead, RegDst, SavePC, RegWrite, ExtWay, ALUSrc, MemWrite, MemtoReg, Branch, JRLJPR, Jump, IorD,
   PVSWriteEnPC, PVSWriteEnReg, PVSWriteEnMem, Reset_N, is_halted, num_inst);
   input wire clk;
   input wire[`WORD_SIZE-1:0] instr;
   input wire Reset_N;
   output reg is_halted;
   output wire MemRead;
   output reg RegDst;
   output reg SavePC;
   output reg RegWrite;
   output reg ExtWay;
   output reg ALUSrc;
   output reg MemWrite;
   output reg MemtoReg;
   output reg Branch;
   output wire JRLJPR;
   output wire Jump;
   output reg IorD;
   output wire [`WORD_SIZE-1:0]num_inst;

   output wire PVSWriteEnPC;
   output wire PVSWriteEnReg;
   output wire PVSWriteEnMem;

   reg [5:0]state;
   reg [`WORD_SIZE-1:0] inst_counter;
   wire[3:0]OPcode;
   wire[5:0]Funct;

   // Assign Instruction counter
   assign num_inst = inst_counter;

   // Assign IF-finished instructions
   assign OPcode = instr[`WORD_SIZE-1:`WORD_SIZE-4];
   assign Funct = instr[5:0];
   assign Jump = (state==0 && OPcode==9) || (state==16);
   assign JRLJPR = (Funct==25)||(Funct==26);
   assign MemRead = (state==10) || (state==9) || (state==0);

   // Assign PVSWriteEn Signals
   assign PVSWriteEnMem = (state==18)?0:(OPcode==8);
   // assign PVSWriteEnReg = (state==18)?0:((OPcode==15 && ((Funct>-1 && Funct<8) || Funct == 26)) || (OPcode>3 && OPcode<8));
   assign PVSWriteEnReg = state==3 || state==6 || state == 10 || state==16 || state== 17;
   assign PVSWriteEnPC = (OPcode==9 || (OPcode==15 && (Funct==25 || Funct==28))) || 
                           state==3 || state==6 || state==10 || state==14 || state==15 || state==16 || state==17;

   // Control state initialization
   always @(posedge Reset_N)
   begin
      state <= 0;
      RegDst <= 0;
      SavePC <= 0;
      RegWrite <= 0;
      ExtWay <= 0;
      ALUSrc <= 0;
      MemWrite <= 0;
      MemtoReg <= 0;
      Branch <= 0;
      IorD <= 1;
      is_halted <= 0;
      inst_counter <= 1;
   end

   // Moore Machine state output
   always @(state)
   begin
   
      // Memory Selection 
      case(state)
         0: begin
            IorD<=1;
         end
         default: begin
            IorD<=0;
         end
      endcase

      // Control Signal
      case(state)
         0: begin
            MemWrite <= 0;
            RegWrite <= 0;
            Branch <= 0;
         end
         1: begin
            // R-type ID
            RegDst <= 1;
            SavePC <= 0;
         end
         2: begin
            // R-type EX
            ALUSrc <= 0;
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
         end
         18: begin
            // System HALTS
            state <= 0;
            RegDst <= 0;
            SavePC <= 0;
            RegWrite <= 0;
            ExtWay <= 0;
            ALUSrc <= 0;
            MemWrite <= 0;
            MemtoReg <= 0;
            Branch <= 0;
            IorD <= 0;
            is_halted <= 1;
         end
      endcase
   end

   // Transition
   always @(posedge clk) begin
      case (state)
         0: begin
         // IF stage of ALL INSTRUCTIONS
            case (instr[`WORD_SIZE-1:`WORD_SIZE-4])
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
                  state <= 12;
               end
               9: begin
                  // JMP
                  inst_counter <= inst_counter + 1;
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
                     25:
                     begin
                        inst_counter <= inst_counter + 1;
                     end
                     26: 
                     begin
                        // JRL
                        state <=17;
                     end
                     29: begin
                        // SYSTEM HALTS
                        state <= 18;
                     end
                     28: 
                     begin
                        // WWD
                        inst_counter <= inst_counter + 1;
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
            inst_counter <= inst_counter + 1;
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
            inst_counter <= inst_counter + 1;
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
            state <= 0;
            inst_counter <= inst_counter + 1;
         end
         11: begin
            state <= 0;
            inst_counter <= inst_counter + 1;
         end
         12: begin
            state <= 13;
         end
         13: begin
            state <= 14;
         end
         14: begin
            state <= 0;
            inst_counter <= inst_counter + 1;
         end
         15: begin
            // Br EX -> IF
            state <= 0;
            inst_counter <= inst_counter + 1;
         end
         16: begin
            // JAL WB -> IF
            state <= 0;
            inst_counter <= inst_counter + 1;
         end
         17: begin
            // JRL WB -> IF
            state <= 0;
            inst_counter <= inst_counter + 1;
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
   RegWrite, RegDst, IorD, MemRead, MemWrite, ALUop, ALUSrc, SavePC, MemtoReg, ExtWay, Branch, JRLJPR, Jump, inst_stabil, output_port, MemAdrsSel, MemData);

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
   input wire [3:0]ALUop;
   input wire ALUSrc;
   input wire SavePC;
   input wire MemtoReg;
   input wire ExtWay;
   input wire Branch;
   input wire JRLJPR;
   input wire Jump;
   output wire [`WORD_SIZE-1:0]inst_stabil;
   output wire [`WORD_SIZE-1:0]output_port;

   output wire [`WORD_SIZE-1:0]MemAdrsSel;
   output wire [`WORD_SIZE-1:0]MemData;
   
   wire [`WORD_SIZE-1:0]inst;
   wire [`WORD_SIZE-1:0]PCOut;
   wire [`WORD_SIZE-1:0]PCplus1;
   wire [`WORD_SIZE-1:0]branchPC;
   wire [`WORD_SIZE-1:0]RSneqPC;
   wire [`WORD_SIZE-1:0]RSeqPC;
   wire [`WORD_SIZE-1:0]jumpPC;
   wire [`WORD_SIZE-1:0]njumpPC;
   wire [`WORD_SIZE-1:0]PCIn;
   wire [`WORD_SIZE-1:0]MemAdrsI;
   wire [`WORD_SIZE-1:0]MemAdrsD;
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
   wire [`WORD_SIZE-1:0]WBMem;
   wire [3:0]ALUConOut;

   reg [`WORD_SIZE-1:0]instbuf;

   assign PCplus1=PCOut+1;
   assign branchPC=PCplus1+ExtendedNum;
   assign RSneqPC=(Branch&&bcond)? branchPC : PCplus1;
   assign RSeqPC=RFRData1;
   assign jumpPC={PCplus1[15:12],inst_stabil[11:0]};
   assign njumpPC=JRLJPR ? RSeqPC : RSneqPC;
   assign PCIn=Jump ? jumpPC : njumpPC;
   assign MemAdrsI=PCOut;
   assign RFRName1=inst_stabil[11:10];
   assign RFRName2=inst_stabil[9:8];
   assign RFWNotSavePC=RegDst ? inst_stabil[7:6] : inst_stabil[9:8];
   assign RFWName=SavePC ? 2 : RFWNotSavePC;
   assign RFWData = SavePC ? PCplus1 : WBData;
   assign ExtendedNum=ExtWay ? signExt : zeroExt;
   assign ALUinputA=RFRData1;
   assign ALUinputB=ALUSrc ? ExtendedNum : RFRData2;
   assign MemAdrsD=ALUoutput;
   assign MemAdrsSel=IorD ? MemAdrsI : MemAdrsD;
   assign MemData= (IorD==0 && MemWrite) ? RFRData2 : 16'hzzzz ;
   assign inst = IorD ? MemData : 16'hzzzz;
   assign WBMem = IorD ? 16'hzzzz : MemData;
   assign WBData = MemtoReg ? WBMem : ALUoutput;
   assign inst_stabil=instbuf;

   assign output_port = RFRData1;

   // always @(RFRData1) begin
   //    output_port=RFRData1;   
   // end
   
   always @(inst) begin
      if(inst!==16'hzzzz)	
		 begin
            instbuf=inst;
		end
   end

   PC pcounter(PCIn, PCOut, PVSWriteEnPC, Reset_N, clk);
   ALUcontrol aluCon(inst_stabil, ALUConOut);
   ALU alu(ALUinputA, ALUinputB, ALUConOut, ALUoutput, bcond);
   NumberExtender ext(inst_stabil[7:0], signExt, zeroExt);
   RF regfile(RFRName1, RFRName2, RFWName, RFWData, RegWrite, RFRData1, RFRData2, PVSWriteEnReg, clk);
   //module RF(readregA, readregB, writereg, writedata, RegWrite, regoutA, regoutB, PVSWriteEn, clk);

endmodule
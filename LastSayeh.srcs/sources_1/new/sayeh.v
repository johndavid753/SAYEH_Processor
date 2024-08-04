`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/01/2024 10:38:15 PM
// Design Name: 
// Module Name: sayeh
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module addresslogic ( input [15:0] PCside, Rside,
input [7:0] Iside,
input ResetPC, PCplusI, PCplus1, RplusI, Rplus0,
output reg [15:0] ALout );
always @ (*)
case ({ResetPC, PCplusI, PCplus1, RplusI, Rplus0})
5'b10000: ALout = 0;
5'b01000: ALout = PCside + Iside;
5'b00100: ALout = PCside + 1;
5'b00010: ALout = Rside + Iside;
5'b00001: ALout = Rside;
default: ALout = PCside;
endcase
endmodule
//Program Counter
module programcounter(
input [15:0] in, input enable, clk, output reg [15:0] out);
always @ (negedge clk) 
    if (enable) out <= in;
endmodule
//Overall Addressing unit...
module AddressingUnit(input [15:0]Rside, input [7:0] Iside, output [15:0]Address,
input clk, ResetPC, PCplusI, PCplus1, RplusI, Rplus0, PCenable);
wire [15:0] PCout;
programcounter PC (Address, PCenable, clk, PCout);
addresslogic AL (PCout, Rside, Iside, ResetPC, PCplusI, PCplus1, RplusI, Rplus0,Address);
endmodule

// Status Register
module StatusRegister(SRCin,SRZin,SRload,clk,Cset,Creset,Zset,Zreset,SRCout,SRZout);
input SRCin,SRZin,clk,Cset,Creset,Zset,Zreset,SRload;
output SRCout,SRZout;
reg SRCout;
reg SRZout;
always@ (negedge clk)begin
if(SRload) begin
    SRCout<= (SRCin|Cset)&(~Creset);
    SRZout<= (SRZin|Zset)&(~Zreset);
        end
    end
endmodule

//Arithmetic Logic Unit.....
`define B15to0H 10'b1000000000
`define AandBH 10'b0100000000
`define AorBH 10'b0010000000
`define notBH 10'b0001000000
`define shlBH 10'b0000100000
`define shrBH 10'b0000010000
`define AaddBH 10'b0000001000
`define AsubBH 10'b0000000100
`define AmulBH 10'b0000000010
`define AcmpBH 10'b0000000001

module ArithmeticUnit ( A, B, B15to0, AandB, AorB, notB, shlB, shrB, AaddB, AsubB, AmulB, AcmpB, aluout, cin, zout, cout);
    input [15:0] A, B;
    input B15to0, AandB, AorB, notB, shlB, shrB, AaddB, AsubB, AmulB, AcmpB;
    input cin;
    output [15:0] aluout;
    output zout, cout;
    reg [15:0] aluout;
    reg zout, cout;
always @(*)
    begin
            zout = 0; cout = 0; aluout = 0;
    case ({B15to0, AandB, AorB, notB, shlB,shrB, AaddB, AsubB, AmulB, AcmpB})
    `B15to0H:aluout = B;
    `AandBH: aluout = A & B;
    `AorBH: aluout = A | B;
    `notBH: aluout = ~B;
    `shlBH: aluout = {B[15:0], B[0]};
    `shrBH: aluout = {B[15], B[15:1]};
    `AaddBH: {cout, aluout} = A + B + cin;
    `AsubBH: {cout, aluout} = A - B - cin;
    `AmulBH: aluout = A[7:0] * B[7:0];
    `AcmpBH: begin
            aluout = A;
            if (A> B) cout = 1;
            else cout = 0;
            if (A==B) zout = 1;
            else zout = 0;
    end
    default: aluout = 0;
    endcase
    
    if (aluout == 0)
        zout = 1'b1;
    end
endmodule

/// Window Pointer....
module WindowPointer(IRout[2:0],clk,WPreset,WPadd,WPout);
input [2:0]IRout;
input WPreset,WPadd,clk;
output reg [2:0]WPout;
always@(negedge clk)
begin 
      if(WPreset)
        WPout<=3'b000;
      else if(WPadd)
        WPout<= WPout+IRout;
end
endmodule

// Instruction Register
module InstrunctionRegister (in, IRload, clk, out);
    input [15:0] in;
    input IRload, clk;
    output [15:0] out;
    reg [15:0] out;
    always @(negedge clk) 
        if (IRload == 1) 
            out <= in;
    endmodule   

// Register FIle......
module RegisterFile ( input [15:0] in,input clk,input [1:0] Laddr, Raddr, input [2:0] Base,input RFLwrite, RFHwrite,output [15:0] Lout, Rout );
    reg [15:0] MemoryFile [0:7];
    wire [2:0] Laddress = Base + Laddr;
    wire [2:0] Raddress = Base + Raddr;
    
assign Lout = MemoryFile [Laddress];
assign Rout = MemoryFile [Raddress];
    reg [15:0] TempReg;
    always @(negedge clk) begin
        TempReg = MemoryFile [Laddress];
            if (RFLwrite) TempReg [7:0] = in [7:0];
            if (RFHwrite) TempReg [15:8] = in [15:8];
            MemoryFile [Laddress] = TempReg;
        end
endmodule

module datapath(clk, Databus, Addressbus,ResetPC, PCplusI, PCplus1, RplusI, Rplus0,Rs_on_AddressUnitRSide, Rd_on_AddressUnitRSide, EnablePC,B15to0, AandB, AorB, notB,
                shlB, shrB,AaddB, AsubB, AmulB, AcmpB,RFLwrite, RFHwrite,WPreset, WPadd, IRload, SRload, Address_on_Databus,ALU_on_Databus,IR_on_LOpndBus, 
                IR_on_HOpndBus, RFright_on_OpndBus,Cset, Creset, Zset, Zreset, Shadow, Instruction, Cout, Zout );
                
input clk;
inout [15:0] Databus;
output [15:0] Addressbus, Instruction;
output Cout, Zout;
input  ResetPC, PCplusI, PCplus1, RplusI, Rplus0,Rs_on_AddressUnitRSide, Rd_on_AddressUnitRSide, EnablePC,B15to0, AandB, AorB, notB, shlB, shrB,AaddB, AsubB,
       AmulB, AcmpB,RFLwrite, RFHwrite, WPreset, WPadd, IRload, SRload,Address_on_Databus, ALU_on_Databus, IR_on_LOpndBus,IR_on_HOpndBus, RFright_on_OpndBus,
        Cset, Creset, Zset, Zreset, Shadow;
        
wire [15:0] Right, Left, OpndBus, ALUout, IRout, Address,AddressUnitRSideBus;
wire SRCin, SRZin, SRZout, SRCout;
wire [2:0] WPout;
wire [1:0] Laddr, Raddr;

AddressingUnit AU (AddressUnitRSideBus, IRout[7:0], Address,clk, ResetPC, PCplusI, PCplus1, RplusI,Rplus0, EnablePC);
ArithmeticUnit AL (Left, OpndBus, B15to0, AandB, AorB, notB,shlB, shrB, AaddB, AsubB, AmulB, AcmpB,ALUout, SRCout, SRZin, SRCin);
RegisterFile RF (Databus, clk, Laddr, Raddr, WPout, RFLwrite,RFHwrite, Left, Right);
InstrunctionRegister IR (Databus, IRload, clk, IRout);
StatusRegister SR (SRCin, SRZin, SRload, clk, Cset, Creset,Zset, Zreset, SRCout, SRZout);
WindowPointer WP (IRout[2:0], clk, WPreset, WPadd, WPout);


assign AddressUnitRSideBus = (Rs_on_AddressUnitRSide) ? Right:(Rd_on_AddressUnitRSide) ?Left :16'bZZZZZZZZZZZZZZZZ;
assign Addressbus = Address;
assign Databus = (Address_on_Databus) ? Address :((ALU_on_Databus) ? ALUout :16'bZZZZZZZZZZZZZZZZ);
assign OpndBus[07:0] = IR_on_LOpndBus == 1 ? IRout[7:0] :8'bZZZZZZZZ;
assign OpndBus[15:8] = IR_on_HOpndBus == 1 ? IRout[7:0] :8'bZZZZZZZZ;
assign OpndBus = RFright_on_OpndBus == 1 ? Right :16'bZZZZZZZZZZZZZZZZ;
assign Zout = SRZout;
assign Cout = SRCout;
assign Instruction = IRout[15:0];
assign Laddr = (~Shadow) ? IRout[11:10] : IRout[3:2];
assign Raddr = (~Shadow) ? IRout[09:08] : IRout[1:0];

endmodule


//COntroller
module controller( Instruction,ExternalReset,MemDataReady, clk,Cflag, Zflag,ResetPC, PCplusI, PCplus1, RplusI, Rplus0,Rs_on_AddressUnitRSide, Rd_on_AddressUnitRSide, EnablePC,
    B15to0, AandB, AorB, notB, shlB, shrB,AaddB, AsubB, AmulB, AcmpB,RFLwrite, RFHwrite,WPreset, WPadd, IRload, SRload, Address_on_Databus,ALU_on_Databus,
    IR_on_LOpndBus, IR_on_HOpndBus, RFright_on_OpndBus,
    Cset, Creset, Zset, Zreset,ReadMem,WriteMem,Shadow );

input ExternalReset,clk,Cflag, Zflag,MemDataReady;
input [15:0] Instruction;

output ResetPC, PCplusI, PCplus1, RplusI, Rplus0, Rs_on_AddressUnitRSide, Rd_on_AddressUnitRSide, EnablePC,
    B15to0, AandB, AorB, notB, shlB, shrB,AaddB, AsubB, AmulB, AcmpB,
    RFLwrite, RFHwrite,WPreset, WPadd, IRload, SRload, 
    Address_on_Databus,ALU_on_Databus,IR_on_LOpndBus, IR_on_HOpndBus, RFright_on_OpndBus,
    Cset, Creset, Zset, Zreset,ReadMem,WriteMem,Shadow;
reg ResetPC, PCplusI, PCplus1, RplusI, Rplus0, Rs_on_AddressUnitRSide, Rd_on_AddressUnitRSide, EnablePC,
    B15to0, AandB, AorB, notB, shlB, shrB,AaddB, AsubB, AmulB, AcmpB,
    RFLwrite, RFHwrite,WPreset, WPadd, IRload, SRload, 
    Address_on_Databus,ALU_on_Databus,IR_on_LOpndBus, IR_on_HOpndBus, RFright_on_OpndBus,
    Cset, Creset, Zset, Zreset, Shadow,ReadMem,WriteMem;
    
parameter [3:0] reset = 0, halt = 1, fetch = 2, memread = 3, exec1 = 4, exec2 = 5, exec1lda = 6, exec2lda = 7, incpc = 8;

reg [3:0] Pstate, Nstate;

//output ShadowEn ;
wire ShadowEn = ~(Instruction[7:0] == 8'b00001111);
parameter hlt = 4'b0001;
parameter nop = 4'b0000;
parameter szf = 4'b0010;
parameter czf = 4'b0011;
parameter scf = 4'b0100;
parameter ccf = 4'b0101;
parameter cwp = 4'b0110;

parameter jpr = 4'b0111;
parameter brz = 4'b1000;
parameter brc = 4'b1001;
parameter awp = 4'b1010;


parameter mvr = 4'b0001;
parameter lda = 4'b0010;
parameter sta = 4'b0011;
parameter inp = 4'b0100;
parameter oup = 4'b0101;
parameter andd = 4'b0110;
parameter orr = 4'b0111;
parameter nott = 4'b1000;
parameter shl = 4'b1001;
parameter shr = 4'b1010;
parameter add = 4'b1011;
parameter sub = 4'b1100;
parameter mul = 4'b1101;
parameter cmp = 4'b1110;

parameter mil = 2'b00;
parameter mih = 2'b01;
parameter spc = 2'b10;
parameter jpa = 2'b11;
 
always @( negedge clk ) begin
    if( ExternalReset ) 
        Pstate <= reset;
    else 
        Pstate <= Nstate;
    end

always @(*)
 begin
    ResetPC = 1'b0; PCplusI=1'b0; PCplus1=1'b0; RplusI= 1'b0; Rplus0= 1'b0; Rs_on_AddressUnitRSide= 1'b0; Rd_on_AddressUnitRSide= 1'b0; EnablePC= 1'b0;
    B15to0= 1'b0; AandB= 1'b0; AorB= 1'b0; notB= 1'b0; shlB= 1'b0; shrB= 1'b0; AaddB= 1'b0; AsubB= 1'b0; AmulB= 1'b0; AcmpB= 1'b0;
    RFLwrite= 1'b0; RFHwrite= 1'b0; WPreset= 1'b0; WPadd= 1'b0; IRload= 1'b0; SRload= 1'b0; 
    Address_on_Databus= 1'b0; ALU_on_Databus= 1'b0;IR_on_LOpndBus= 1'b0; IR_on_HOpndBus= 1'b0; RFright_on_OpndBus= 1'b0;
    Cset= 1'b0; Creset= 1'b0; Zset= 1'b0; Zreset= 1'b0; Shadow= 1'b0; ReadMem = 1'b0; WriteMem = 1'b0;

    case(Pstate) 
        reset: begin
            ResetPC = 1'b1;
            EnablePC = 1'b1;
            SRload = 1'b1;
            Creset = 1'b1;
            Zreset = 1'b1;
            WPreset = 1'b1;
            Nstate=fetch;
            end
            //End Reset
        halt:begin
            Nstate = halt;
            end
        
        fetch: begin
            ReadMem = 1'b1;
            Nstate=memread;
            end
            //End fetch
        
        memread: begin
            if(MemDataReady == 1'b1) begin
                IRload = 1'b1;
                Nstate=exec1;
                end
            else begin
                ReadMem = 1'b1;
                Nstate = memread;
                end
            end
            //End readmem
            
        exec1:
            if(Instruction[15:12] == 4'b0000) begin
                case(Instruction[11:8])
                    nop : begin
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                     hlt: begin
                        PCplus1 = 1'b1;
                        EnablePC=1'b1;
                        Nstate = halt;
                        end 
                         
                     scf : begin
                         Cset = 1'b1;
                         SRload = 1'b1;
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                    ccf : begin
                        Creset = 1'b1;
                        SRload = 1'b1;
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                    szf : begin
                        Zset = 1'b1;
                        SRload = 1'b1;                        
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                    czf : begin
                        Zreset = 1'b1;
                        SRload = 1'b1;
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                    cwp : begin
                        WPreset = 1'b1;
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                    // 16 bit instruction with immediate data   
                    jpr : begin
                        PCplusI=1'b1;
                        EnablePC=1'b1;
                        Nstate = fetch;
                        end
                        
                    brz : begin
                        if(Zflag == 1) begin
                            PCplusI=1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        else begin
                            PCplus1=1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                    brc : begin
                        if(Cflag == 1) begin
                            PCplusI=1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        else begin
                            PCplus1=1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                    awp : begin
                        WPadd = 1'b1;
                        PCplus1=1'b1;
                        EnablePC=1'b1;
                        Nstate = fetch;
                        end                                               
                endcase
            end// if END
            
            else if (Instruction[15:12] == 4'b1111) begin
                case(Instruction[9:8])
                    mil : begin
                        
                        IR_on_LOpndBus= 1'b1;
                        B15to0= 1'b1;
                        ALU_on_Databus= 1'b1; 
                        RFLwrite = 1'b1;
                        
                        
                        PCplus1 = 1'b1;
                        EnablePC=1'b1;
                        Nstate = fetch;
                        end
                    mih : begin
                        IR_on_HOpndBus= 1'b1;
                        B15to0= 1'b1;
                        ALU_on_Databus= 1'b1; 
                        RFHwrite = 1'b1;
                        
                        PCplus1 = 1'b1;
                        EnablePC=1'b1;
                        Nstate = fetch;
                        end
                        
                    spc : begin
                        PCplusI = 1'b1;
                        
                        Address_on_Databus= 1'b1;
                        
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        Nstate = incpc;                                          
                        end
                        
                    jpa : begin
                        Rd_on_AddressUnitRSide= 1'b1;
                        RplusI= 1'b1;
                        EnablePC=1'b1;
                        Nstate=fetch;
                        end   
                    endcase
                end //elseif END
                
                else  begin
                case(Instruction[15:12])
                
                    lda : begin  
                        Rs_on_AddressUnitRSide = 1'b1;
                        Rplus0 = 1'b1;
                        //PCplusI=1'b0;
                        ReadMem = 1'b1;
                        Nstate = exec1lda;
                        end
                       
                        
                    sta : begin
                        RFright_on_OpndBus = 1'b1;
                        B15to0 = 1'b1;
                        ALU_on_Databus = 1'b1;
                        
                        Rplus0 = 1'b1;
                        Rd_on_AddressUnitRSide = 1'b1;
                        WriteMem = 1'b1;
                        
                        Nstate = incpc;
          
                        end
                    mvr : begin
                        RFright_on_OpndBus = 1'b1;
                        B15to0 = 1'b1;
                        ALU_on_Databus = 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                    andd : begin
                        
                        RFright_on_OpndBus= 1'b1;
                        AandB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                   orr : begin
                        RFright_on_OpndBus= 1'b1;
                        AorB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                   nott : begin
                        RFright_on_OpndBus= 1'b1;
                        notB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                   shl : begin
                        RFright_on_OpndBus= 1'b1;
                        shlB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                   shr : begin
                        RFright_on_OpndBus= 1'b1;
                        shrB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1= 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                   add : begin
                        
                        RFright_on_OpndBus = 1'b1;
                        AaddB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        SRload = 1'b1;
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                   sub : begin
                        RFright_on_OpndBus= 1'b1;
                        AsubB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        SRload = 1'b1;
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                   mul : begin
                        RFright_on_OpndBus= 1'b1;
                        AmulB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        SRload = 1'b1;
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                   cmp : begin
                        RFright_on_OpndBus= 1'b1;
                        AcmpB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        SRload = 1'b1;
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            end
                        end
                        
                        
                        
                        
                        
             
                  endcase
              end //else END
              
        exec1lda : begin
                    if(MemDataReady == 1'b1) begin  
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        if (ShadowEn==1'b1)
                            Nstate = exec2;
                        else begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;   
                        end 
                     end
                     else begin
                        ReadMem = 1'b1;
                        Rplus0 = 1'b1;
                        Rs_on_AddressUnitRSide = 1'b1;
                        Nstate = exec1lda;
                     end
                end
                
         exec2 : begin
                Shadow = 1'b1;
                if(Instruction[7:4] == 4'b0000) begin
                case(Instruction[3:0])
                    nop : begin
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch; 
                        end
                     hlt: begin
                        PCplus1 = 1'b1;
                        EnablePC=1'b1;
                        Nstate = halt;
                        end  
                        
                     scf : begin
                         Cset = 1'b1;
                         SRload = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                    ccf : begin
                        Creset = 1'b1;
                        SRload = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                    szf : begin
                        Zset = 1'b1;
                        SRload = 1'b1;                        
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                    czf : begin
                        Zreset = 1'b1;
                        SRload = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                    cwp : begin
                        WPreset = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch; 
                        end
                                               
                endcase
            end// if END
            
                
                else  begin
                case(Instruction[7:4])
                    lda : begin
                        
                        Rs_on_AddressUnitRSide = 1'b1;
                        Rplus0 = 1'b1;
                        ReadMem = 1'b1;
                        Nstate = exec2lda;
                        end
                        
                    sta : begin
                        RFright_on_OpndBus = 1'b1;
                        B15to0 = 1'b1;
                        ALU_on_Databus = 1'b1;
                        
                        Rplus0 = 1'b1;
                        Rd_on_AddressUnitRSide = 1'b1;
                        WriteMem = 1'b1;
                        
                        Nstate = fetch;
                        end
                    mvr : begin
                        RFright_on_OpndBus = 1'b1;
                        B15to0 = 1'b1;
                        ALU_on_Databus = 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1; 
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                    andd : begin
                        
                        RFright_on_OpndBus= 1'b1;
                        AandB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                   orr : begin
                        RFright_on_OpndBus= 1'b1;
                        AorB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;  
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate <= fetch;
                           
                        end
                        
                   nott : begin
                        RFright_on_OpndBus= 1'b1;
                        notB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                   shl : begin
                        RFright_on_OpndBus= 1'b1;
                        shlB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                   shr : begin
                        RFright_on_OpndBus= 1'b1;
                        shrB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                   add : begin
                        
                        RFright_on_OpndBus = 1'b1;
                        AaddB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        SRload = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                   sub : begin
                        RFright_on_OpndBus= 1'b1;
                        AsubB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        SRload = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                   mul : begin
                        RFright_on_OpndBus= 1'b1;
                        AmulB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        SRload = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end
                        
                   cmp : begin
                        RFright_on_OpndBus= 1'b1;
                        AcmpB = 1'b1;
                        ALU_on_Databus= 1'b1;
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                        SRload = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;
                            
                        end       
                  endcase
              end //else END          
         end  //exec2
           
       exec2lda:
                 begin
                    if(MemDataReady == 1'b1) begin  
                        RFLwrite = 1'b1;
                        RFHwrite = 1'b1;
                            PCplus1 = 1'b1;
                            EnablePC=1'b1;
                            Nstate = fetch;   
                        
                     end
                end//END of Exedc2lda
                
       incpc : begin
            PCplus1 = 1'b1;
            EnablePC=1'b1;
            Nstate = fetch; 
            end
            
       default : Nstate=reset;
    endcase
    
  end
endmodule


module Sayeh ( clk, ReadMem, WriteMem,Databus, Addressbus, ExternalReset,MemDataReady);

input clk;
output ReadMem, WriteMem;
inout [15: 0] Databus;
output [15: 0] Addressbus;
input ExternalReset,MemDataReady;
wire [15:0]Instruction;

wire ResetPC, PCplusI, PCplus1, RplusI, Rplus0, Rs_on_AddressUnitRSide, Rd_on_AddressUnitRSide, EnablePC,
    B15to0, AandB, AorB, notB, shlB, shrB,AaddB, AsubB, AmulB, AcmpB,
    RFLwrite, RFHwrite,WPreset, WPadd, IRload, SRload, 
    Address_on_Databus,ALU_on_Databus,IR_on_LOpndBus, IR_on_HOpndBus, RFright_on_OpndBus,
    Cset, Creset, Zset, Zreset, Shadow,ReadMem,WriteMem,Cout,Zout;
    
//wire Cout,Zout;


datapath dp(clk, Databus, Addressbus,ResetPC, PCplusI, PCplus1, RplusI, Rplus0,Rs_on_AddressUnitRSide, Rd_on_AddressUnitRSide, EnablePC,B15to0, AandB, AorB, notB,
                shlB, shrB,AaddB, AsubB, AmulB, AcmpB,RFLwrite, RFHwrite,WPreset, WPadd, IRload, SRload, Address_on_Databus,ALU_on_Databus,IR_on_LOpndBus, 
                IR_on_HOpndBus, RFright_on_OpndBus,Cset, Creset, Zset, Zreset, Shadow, Instruction, Cout, Zout);

controller ch( Instruction,ExternalReset,MemDataReady, clk,Cout, Zout,ResetPC, PCplusI, PCplus1, RplusI, Rplus0,Rs_on_AddressUnitRSide, Rd_on_AddressUnitRSide, EnablePC,
                B15to0, AandB, AorB, notB, shlB, shrB,AaddB, AsubB, AmulB, AcmpB,RFLwrite, RFHwrite,WPreset, WPadd, IRload, SRload, Address_on_Databus,
                ALU_on_Databus,IR_on_LOpndBus, IR_on_HOpndBus, RFright_on_OpndBus,Cset, Creset, Zset, Zreset,ReadMem,WriteMem,Shadow);


endmodule


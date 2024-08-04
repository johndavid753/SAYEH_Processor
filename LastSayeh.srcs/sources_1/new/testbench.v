`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/02/2024 12:59:38 AM
// Design Name: 
// Module Name: testbench
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

module testbench;

reg  ExternalReset=1, clk=1;
reg MemDataReady;
wire ReadMem, WriteMem;
wire [15:0] Addressbus;
wire [15:0] Databus;
reg [15:0] memory[0:63];  // Declare memory array
reg [15:0]mem_data;


   initial begin
      MemDataReady=0;
      mem_data=16'bz;
   // Read the memory file into the array
      $readmemh("instructions.mem", memory);  
      #50 ExternalReset=1'b0;
    end

 
Sayeh UUT(clk, ReadMem, WriteMem,Databus, Addressbus, ExternalReset,MemDataReady);

always #10 clk = ~clk;


//reg control=0;


always@(posedge clk) begin : Memory_Read_Write
    if(ReadMem) begin
        MemDataReady <= 1;
        mem_data<=memory[Addressbus];
        end
     else begin
        MemDataReady <= 0;
        mem_data <= 16'bz;
     end
     
    if(WriteMem) begin
       #1 memory[Addressbus]=Databus;
       end 
end

// Assigning( Copying) the mem_data to the data_bus
assign Databus = mem_data;

//initial begin
//#20
//$stop;
//end
    
endmodule


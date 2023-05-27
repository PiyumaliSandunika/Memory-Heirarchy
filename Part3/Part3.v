// Computer Architecture (CO224) - Lab 05
// Design: Testbench of Integrated CPU of Simple Processor


`include "alu.v"
`include "reg_file.v"
`include "data_memory.v"
`include "data_cache.v"
`include "instr_memory.v"
`include "instr_cache.v"
`timescale 1ns/100ps

module cpu_tb;

    reg CLK, RESET;
	wire WRITE,READ,BUSYWAIT,INSTRCACHE_BUSYWAIT,DCACHE_BUSYWAIT;
	wire [7:0] READDATA,WRITEDATA,ADDRESS;
    wire [31:0] PC;
    wire [31:0] INSTRUCTION;
    wire [31:0] MEM_WRITEDATA,MEM_READDATA;
	wire [5:0] MEM_ADDRESS,IMEM_PC;
	wire MEM_READ,MEM_WRITE,MEM_BUSYWAIT,IMEM_BUSYWAIT,IMEM_READ;
	wire [127:0] IMEM_READDATA;
    /* 
    ------------------------
     SIMPLE INSTRUCTION MEM
    ------------------------
    */
    
    // TODO: Initialize an array of registers (8x1024) named 'instr_mem' to be used as instruction memory
    //reg [7:0]instr_mem [1023:0];
    // TODO: Create combinational logic to support CPU instruction fetching, given the Program Counter(PC) value 
    //       (make sure you include the delay for instruction fetching here)
   /* always @(PC) begin
		#2
		INSTRUCTION = {instr_mem[PC+3],instr_mem[PC+2],instr_mem[PC+1],instr_mem[PC]};
	end
        
    initial
    begin

        
        //loading instr_mem content from instr_mem.mem file
        $readmemb("instr_mem.mem", instr_mem);
    end*/
    
    /* 
    -----
     CPU
    -----
    */
	assign BUSYWAIT = (INSTRCACHE_BUSYWAIT || DCACHE_BUSYWAIT);

	//instantiate cpu and memory moodules and cache
    cpu mycpu(PC,WRITEDATA,WRITE,READ,ADDRESS,INSTRUCTION,CLK,RESET,BUSYWAIT,READDATA);
	
	dcache mydcache(CLK,RESET,DCACHE_BUSYWAIT,READ,WRITE,WRITEDATA,READDATA,ADDRESS,MEM_BUSYWAIT,MEM_READ,MEM_WRITE,MEM_WRITEDATA,MEM_READDATA,MEM_ADDRESS); //chamudi

	data_memory mydatamem(CLK,RESET,MEM_READ,MEM_WRITE,MEM_ADDRESS,MEM_WRITEDATA,MEM_READDATA,MEM_BUSYWAIT);

	icache myicache(CLK,RESET,PC[9:0],INSTRUCTION,INSTRCACHE_BUSYWAIT,IMEM_READ,IMEM_READDATA,IMEM_PC,IMEM_BUSYWAIT);
	//instructcache myicache(CLK,RESET,PC[10:0],INSTRCACHE_BUSYWAIT,IMEM_READ,IMEM_PC,IMEM_READDATA,IMEM_BUSYWAIT,INSTRUCTION);

	instruction_memory myinstrmem(CLK,IMEM_READ,IMEM_PC,IMEM_READDATA,IMEM_BUSYWAIT);
	
	
	initial
    begin
    
        // generate files needed to plot the waveform using GTKWave
        $dumpfile("waveform.vcd");
		$dumpvars(0, cpu_tb);
        
        CLK = 1'b0;
        RESET = 1'b0;
        
        // TODO: Reset the CPU (by giving a pulse to RESET signal) to start the program execution
        RESET = 1'b1;
        #8
        RESET = 1'b0;
        // finish simulation after some time
        #1500
        $finish;
        

    end
	
    // clock signal generation
    always
        #4 CLK = ~CLK;
   
endmodule

//module to caculate twos complement
module twosComp(TWOSIN,TWOSOUT);
	input[7:0]TWOSIN;     //8 bit input
	output[7:0]TWOSOUT;   //8 bit output
	
	assign #1 TWOSOUT = ~TWOSIN + 8'b00000001;  //twos complement of the input assigned to output
endmodule //twosComp

//module for a mux
module mux08(MUXIN1,MUXIN2,MUXSELECT,MUXOUT);
	input [7:0]MUXIN1;       //8 bit input
	input [7:0]MUXIN2;       //8 bit input
	input MUXSELECT;         //input
	output reg [7:0]MUXOUT;  //8 bit output register
	
	//Get executed whenever the values of SELECT,IN1,IN2 changes
	always @(MUXSELECT,MUXIN1,MUXIN2) begin
	//Choose the output according to the value of select 
	case(MUXSELECT)
		1'b0 : MUXOUT = MUXIN1;
		1'b1 : MUXOUT = MUXIN2;
	endcase
	end
endmodule 

module mux32(MUXIN1,MUXIN2,MUXSELECT,MUXOUT);
	input [31:0]MUXIN1;       //32 bit input
	input [31:0]MUXIN2;       //32 bit input
	input MUXSELECT;         //input
	output reg [31:0]MUXOUT;  //32 bit output register
	
	//Get executed whenever the values of SELECT,IN1,IN2 changes
	always @(MUXSELECT,MUXIN1,MUXIN2) begin
	//Choose the output according to the value of select 
	case(MUXSELECT)
		1'b1 : MUXOUT = MUXIN1;
		1'b0 : MUXOUT = MUXIN2;
	endcase
	end
endmodule 

//module for a adder to increment the pc value
module PCADDER (PCINPUT,PCNEXT,BUSY);
	input [31:0] PCINPUT;    //32 bit input
	output reg [31:0] PCNEXT; //32 bit output register
	input BUSY;		//input 1 bit busy signal
	
	always @(*) begin
	if(BUSY == 1'b0) begin		//assign next PC value only when busy signalis deasserted
	#1 PCNEXT = PCINPUT + 4;  //has a delay of 1 unit
	end
	end
endmodule //PCADDER

//module to determine PC value in a j/beq instruction
module ADDER(PCINPUT2,INCREMENT,PCNEXT2,BUSY);
	input signed [31:0] PCINPUT2;    //32 bit input
	input signed [7:0] INCREMENT;
	output reg signed[31:0] PCNEXT2; //32 bit output register
	input BUSY;
	
	always @(*) begin
	if(BUSY == 1'b0) begin
	//get executed whenever the value of PCINPUT changes
	#2 PCNEXT2 = PCINPUT2 + (INCREMENT<<2);  //add offset to PC and assign after delay of 2 time units
	end
	end
endmodule

//module for the control unit
module control(OP,INSTRUCTION,WRITEENABLE,ALUOP,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,BUSYWAIT,SIGNAL);
	input[7:0] OP ;  	//input signals
	input [31:0]INSTRUCTION;
	input [0:0] BUSYWAIT;                          
	output reg[0:0] WRITEENABLE,ISMINUS,ISIMM,BRANCH,JUMP,READ,WRITE,SIGNAL; //output registers
	output reg [2:0] ALUOP;   //3 bit output register
	
	//
	always@(negedge BUSYWAIT) begin
		READ=0;
		WRITE=0;
	end
	
    //Get executed whenever the OP changes
	always @(INSTRUCTION) begin
		//Generating the signals according to the opcode
		//with a delay of 1 unit
		#1 case(OP)
			8'b0000_0000 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b000_1_0_1_0_0_0_0_0;	//loadi
			8'b0000_0001 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b000_1_0_0_0_0_0_0_0;	//mov
			8'b0000_0010 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b001_1_0_0_0_0_0_0_0;	//add
			8'b0000_0011 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b001_1_1_0_0_0_0_0_0;	//sub
			8'b0000_0100 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b010_1_0_0_0_0_0_0_0;	//and
			8'b0000_0101 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b011_1_0_0_0_0_0_0_0;	//or
			8'b0000_0110 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'bxxx_0_0_0_1_0_0_0_0;	//j
			8'b0000_0111 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b001_0_1_0_0_1_0_0_0;	//beq
			8'b0000_1000 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b000_1_0_0_0_0_1_0_1;	//lwd - reg direct addressing
			8'b0000_1001 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b000_1_0_1_0_0_1_0_1;	//lwi - immediate addressing
			8'b0000_1010 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b000_0_0_0_0_0_0_1_0;	//swd
			8'b0000_1011 : 
				{ALUOP,WRITEENABLE,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,SIGNAL} = 11'b000_0_0_1_0_0_0_1_0;	//swi
		endcase
	end	
endmodule 

//module for CPU
module cpu(PC,WRITEDATA,WRITE,READ,ADDRESS,INSTRUCTION,CLK,RESET,BUSYWAIT,READDATA);
	
	input [31:0]INSTRUCTION;
	input  CLK,RESET,BUSYWAIT;       
	output reg[31:0]PC;      		
	output WRITE,READ;		
	input [7:0] READDATA;
	output [7:0] WRITEDATA;
	output [7:0]ADDRESS;
	

	reg [7:0]OPCODE,IMMEDIATE;
  	reg signed [7:0] INCREMENT;				
	reg[2:0]WRITEREG,READREG1,READREG2;  //3bit registers                                  
	wire[7:0]ALURESULT,REGOUT1,REGOUT2,MUX1OUT,MUX2OUT,TWOSRESULT,MXR_OUT;//,READDATA;     //6 8-bit wires
	wire[2:0]ALUOP;                                                    //3 bit wire
	wire ISMINUS,ISIMM,WRITEENABLE,ZERO,JUMP,BRANCH,ANDOUT,OROUT,SIGNAL;            //7 wires
	wire [31:0] PCRESULT,OFFSET,FINALPC;                          //32 bit wire
	wire [1:0] SEL;		//2 bit wire
 
 
	and a1 (ANDOUT,BRANCH,ZERO);  
	or o1 (OROUT,ANDOUT,JUMP);
		
	//instantiate PCADDER module
	PCADDER adder1(.PCINPUT(PC),.PCNEXT(PCRESULT),.BUSY(BUSYWAIT)); 

	//instantiate adder to increase the value in j/beq instruction	
	ADDER adder2(.PCINPUT2(PCRESULT),.INCREMENT(INCREMENT),.PCNEXT2(OFFSET),.BUSY(BUSYWAIT));  

	//Instantiate mux32 module to choose between jump and normal increment of pc
	mux32 mux01(.MUXIN1(OFFSET),.MUXIN2(PCRESULT),.MUXSELECT(OROUT),.MUXOUT(FINALPC));
	
	//Get executed at the positive edge of the clock
	always @(posedge CLK) begin
		if(BUSYWAIT == 1'b0) begin	//if busy signal is deasserted
			//if RESET is high assign 0 to the PC
			if(RESET) begin
				#1 PC = 0;
			//else assign the incremented value to the PC
			end else begin
				#1 PC = FINALPC;
			end
		end
	end
	
	//decoding
	always @(INSTRUCTION) begin
		OPCODE = INSTRUCTION[31:24];        //24-31 bits -> Opcode
		WRITEREG = INSTRUCTION[18:16];      //16-18 bits -> Destination register
		READREG1 = INSTRUCTION[10:8];       //8-10 bits -> Source register 1
		READREG2 = INSTRUCTION[2:0];        //0-7 bits -> Source register 2 
		IMMEDIATE = INSTRUCTION[7:0];       //0-7 bits -> Immediate value
		INCREMENT = INSTRUCTION[23:16];     //16-23 bits -> Increment value
	end
	
	
	//instantiate control module
	control controlUnit1(OPCODE,INSTRUCTION,WRITEENABLE,ALUOP,ISMINUS,ISIMM,JUMP,BRANCH,READ,WRITE,BUSYWAIT,SIGNAL);
	//instantiate alu module
	alu alu1(.DATA1(WRITEDATA),.DATA2(MUX2OUT),.ZERO(ZERO),.RESULT(ADDRESS),.SELECT(ALUOP));
	//instantiate mux module which is between reg file and control unit
	mux08 muxtoreg (.MUXIN1(ADDRESS),.MUXIN2(READDATA),.MUXOUT(MXR_OUT),.MUXSELECT(SIGNAL));
	//instantiate reg_file module
	reg_file regfile1(.IN(MXR_OUT),.OUT1(WRITEDATA),.OUT2(REGOUT2),.INADDRESS(WRITEREG),.OUT1ADDRESS(READREG1),.OUT2ADDRESS(READREG2),.WRITE(WRITEENABLE),.CLK(CLK),.RESET(RESET));
	//instantiate mux08 module for twos complement
	mux08 mux1twos(.MUXIN1(REGOUT2),.MUXIN2(TWOSRESULT),.MUXOUT(MUX1OUT),.MUXSELECT(ISMINUS));
	//instantiate mux08 module for immidiate value
	mux08 mux2imm(.MUXIN1(MUX1OUT),.MUXIN2(IMMEDIATE),.MUXOUT(MUX2OUT),.MUXSELECT(ISIMM));
	//instantiate twosComp module
	twosComp twosComp1(.TWOSIN(REGOUT2),.TWOSOUT(TWOSRESULT));	
	
endmodule //cpu


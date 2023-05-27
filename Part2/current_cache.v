/*
Module  : Data Cache 
Group : 22
*/
`timescale 1ns/100ps
module dcache (
    clock,
    reset,
    busywait,
    read,
    write,
    writedata,
    readdata,
    address,
    mem_busywait,
    mem_read,
    mem_write,
    mem_writedata,
    mem_readdata,
    mem_address
);

input         clock;
input         reset;
input         read;             //from cpu controller
input         write;            //signal from cpu controller
input         mem_busywait;     //signal from memory 
input[7:0]    writedata;        //signal from register file
input[7:0]    address;          //from ALU
input[31:0]   mem_readdata;     //read data from memory 

output reg         busywait;        //signal to controller
output reg         mem_read;        //signal to memory
output reg         mem_write;          //from memory
output reg [5:0]   mem_address;
output reg [7:0]       readdata;
output reg [31:0]  mem_writedata;

reg [31:0] dataCache[0:7]; //8 datablocks -> 32bits * 8
reg [2:0]  cacheTagArr[0:7];
reg [7:0]  validArr;
reg [7:0]  dirtyArr;
reg [31:0] dataBlock;
wire [2:0] index;
wire [2:0] tag;
wire [2:0] dataBlockTag;
wire [1:0] offset;
reg [7:0] data;
wire hit,valid,dirty,tagMatch;

	/*
    Combinational part for indexing, tag comparison for hit deciding, etc.
    ...
    ...
	*/

	//asserting the busywait signal when read/write signal is detected
	always@ (read,write)
        busywait = (read || write)? 1 : 0;

	//splitting the address into index,tag and offset
    assign index = address[7:2] % 8;
    assign tag = address[7:5];
    assign offset = address[1:0];

	//extracting current datablock
	always@ (*)
    begin
        #1
        dataBlock = dataCache[index];        
    end

	//extracting the tag,valid,dirty values corresponding to the current datablock
	assign #1 dataBlockTag = cacheTagArr[index];               
    assign #1 valid = validArr[index];
    assign #1 dirty = dirtyArr[index];


	//tag comparison and validation
	assign #0.9 tagMatch = (dataBlockTag == tag) ? 1 : 0;       //check whether tag and current address tag match
    assign hit = valid && tagMatch;
	
    
    always @(offset,read,hit)
    begin
        #1
        case(offset)
                2'b00: data = dataBlock[7:0];
                2'b01: data = dataBlock[15:8];
                2'b10: data = dataBlock[23:16];
                2'b11: data = dataBlock[31:24];
        endcase
        if(read && hit)
        begin
            readdata =  data;
            busywait = 1'b0;
        end
    end

	//writehit
	always @(posedge clock)
    begin
        if(write==1 && hit==1)
        begin
            #1
            case(offset)
                2'b00: dataCache[index][7:0] = writedata;
                2'b01: dataCache[index][15:8] = writedata;
                2'b10: dataCache[index][23:16] = writedata;
                2'b11: dataCache[index][31:24] =writedata;
            endcase

            validArr[index] = 1;
            dirtyArr[index] = 1;
        end
    end

    integer i;

    //Reset cache blocks 
    always @(posedge reset)
    begin
        for (i=0;i<8; i=i+1)
        begin
            dataBlock[i] = 32'dx;      //set to unknown values
            cacheTagArr[i] = 3'dx;        //set to unknown values
            validArr[i] = 0;
            dirtyArr[i] = 0;
        end
    end

//at posedge clock if its a hit de-assert the busywait so the CPU runs without stalling
	always@ (posedge clock)
    begin
      if (hit) begin            
          busywait = 0;     
      end
    end
    


	/* Cache Controller FSM Start */

    parameter IDLE = 3'b000, MEM_READ = 3'b001 ,MEM_WRITE = 3'b010 ,CACHE_UPDATE = 3'b011;
    reg [2:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if ((read || write) && !dirty && !hit)  //readmiss, writemiss with dirty=0
                    next_state = MEM_READ;
                else if ((read || write) && dirty && !hit) //readmiss, writemiss with dirty=1
                    next_state =  MEM_WRITE;
                else
                    next_state = IDLE;
            
            MEM_READ:
                if (!mem_busywait)
                    next_state = CACHE_UPDATE;
                else    
                    next_state = MEM_READ;
            
            MEM_WRITE:
                if (!mem_busywait)
                    next_state = MEM_READ;
                else    
                    next_state = MEM_WRITE;
            
            CACHE_UPDATE:
                next_state = IDLE;    
        endcase
    end

	//combinational output logic
    always @(state)
    begin
        case(state)
            IDLE:
            begin
                mem_read = 0;
                mem_write = 0;
                mem_address = 5'dx;
                mem_writedata = 32'dx;
                busywait = 0;
            end
         
            MEM_READ: 
            begin
                mem_read = 1;
                mem_write = 0;
                mem_address = {tag, index};
                mem_writedata = 32'dx;
                busywait=1;
            end

            MEM_WRITE:
            begin
                mem_read = 0;
                mem_write = 1;
                mem_address = {dataBlockTag,index};
                mem_writedata = dataBlock;
                busywait=1;
            end

            CACHE_UPDATE:
            begin
                mem_read = 0;
                mem_write = 0;
                mem_address = 5'dx;
                mem_writedata = 32'dx;

				#1
                cacheTagArr[index] = tag;
                validArr[index] = 1;
                dataCache[index] = mem_readdata;
                dirtyArr[index] = 0;
               // busywait=0;
            end
        endcase
    end

	//sequential logic for state transitioning 
    always @(posedge clock, reset)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

    /* Cache Controller FSM End */

endmodule
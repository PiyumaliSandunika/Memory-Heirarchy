module icache(clock,reset,i_pc,instruction,i_busywait,imem_read,imem_readdata,imem_pc,imem_busywait);

input clock,reset;
input[9:0]i_pc;                     //cpu accessses single instr using 10 bit word addr
input imem_busywait;                //to notify cache that mem is reading instructions
input [127:0] imem_readdata;        //fetch 16 byte(128 bits) block wise

output reg [5:0] imem_pc;
output reg [31:0]instruction;        //output relevant 32 bit instruction
output reg imem_read,i_busywait;     //notify instr_mem to read instr

reg [127:0] iblock [0:7];        //8 blocksof 16 byte(128 bits)
reg [2:0]itagArray[0:7];
reg validArray[0:7];

reg [2:0]correct_itag,correct_validbit;
wire[2:0]tag,index;
wire [1:0] offset;
reg [127:0] correct_iblock;
reg itagMatch,hit;
reg cacheUpdate;
integer i;

//----------------------- Combinational part for indexing, tag comparison for hit deciding, etc.------------------------
initial begin
  i_busywait = 0;
end

//spliting the address into tag,index,offset
// [9:7]---> tag [6:4] ---->index  [3:2]--->offset  [1:0]---> always 00 represents a 4byte word
assign tag = i_pc[9:7];
assign index = i_pc[6:4];
assign offset = i_pc[3:2];

always @(i_pc,hit)
begin
    if(hit==0)
    i_busywait = 1;
end


always@(*)begin
    //extracting stored datablock,tag,valid bit
    #1
    correct_iblock = iblock[index];
    correct_itag = itagArray[index];
    correct_validbit = validArray[index];

    //tag comparison and validation
    #0.9
    itagMatch = (tag == correct_itag) ? 1 : 0;       //check whether tag and current address tag match
    hit = correct_validbit && itagMatch;
end


//read data from cache to cpu (handling read hits)
    always @(*) begin
        if(hit == 1) begin
			i_busywait = 0;

        #1 case(offset)
            2'b00: instruction = correct_iblock[31:0];
            2'b01: instruction = correct_iblock[63:32];
            2'b10: instruction = correct_iblock[95:64];
            2'b11: instruction = correct_iblock[127:96];
        endcase
        end
    end

	/* Cache Controller FSM Start */

    parameter IDLE = 3'b000, IMEM_READ = 3'b001;
    reg [2:0] state, next_state;

    // combinational next state logic
    always @(*)
    begin
        case (state)
            IDLE:
                if (hit==0)  //if read miss 
                    next_state = IMEM_READ;
                else                                   
                    next_state = IDLE;
            
            IMEM_READ:
                if (!imem_busywait)          //finished mem reading
                    next_state = IDLE;
                else    
                    next_state = IMEM_READ;  
        endcase
    end

	// combinational output logic
    always @(state)begin
        case(state)
            IDLE:
            begin
                imem_read = 0;
                imem_pc = 6'bxxxxxx;
				cacheUpdate = 0;
            end
         
            IMEM_READ:
            begin
                imem_read = 1;
                imem_pc = {tag, index};
				cacheUpdate = 1;
            end
        endcase
    end

    //update the cache,validbit,tag when reading from the memory is over
	always @(negedge cacheUpdate)
	begin
		#1
        iblock[index] = imem_readdata;
        validArray[index] = 1;
        itagArray[index] = tag;
	end

//sequential logic for state transitioning

always @(posedge clock,reset)
begin
    if(reset) 
        state = IDLE;
    else
        state = next_state;
end

always @(posedge reset)
begin
    for (i=0;i<8; i=i+1)
    begin
        validArray[i] = 0;       //assigning value zero to valid bit array
        iblock[i] = 128'dx;      //set to unknown values
        itagArray[i] = 3'dx;     //set to unknown values
    end
end

endmodule
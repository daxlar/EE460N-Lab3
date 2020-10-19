/*
    Name 1: Your full name
    UTEID 1: Your UT EID
*/

/***************************************************************/
/*                                                             */
/*   LC-3b Simulator                                           */
/*                                                             */
/*   EE 460N                                                   */
/*   The University of Texas at Austin                         */
/*                                                             */
/***************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/***************************************************************/
/*                                                             */
/* Files:  ucode        Microprogram file                      */
/*         isaprogram   LC-3b machine language program file    */
/*                                                             */
/***************************************************************/

/***************************************************************/
/* These are the functions you'll have to write.               */
/***************************************************************/

void eval_micro_sequencer();
void cycle_memory();
void eval_bus_drivers();
void drive_bus();
void latch_datapath_values();

/***************************************************************/
/* A couple of useful definitions.                             */
/***************************************************************/
#define FALSE 0
#define TRUE  1

/***************************************************************/
/* Use this to avoid overflowing 16 bits on the bus.           */
/***************************************************************/
#define Low16bits(x) ((x) & 0xFFFF)

/***************************************************************/
/* Definition of the control store layout.                     */
/***************************************************************/
#define CONTROL_STORE_ROWS 64
#define INITIAL_STATE_NUMBER 18

/***************************************************************/
/* Definition of bit order in control store word.              */
/***************************************************************/
enum CS_BITS {                                                  
    IRD,
    COND1, COND0,
    J5, J4, J3, J2, J1, J0,
    LD_MAR,
    LD_MDR,
    LD_IR,
    LD_BEN,
    LD_REG,
    LD_CC,
    LD_PC,
    GATE_PC,
    GATE_MDR,
    GATE_ALU,
    GATE_MARMUX,
    GATE_SHF,
    PCMUX1, PCMUX0,
    DRMUX,
    SR1MUX,
    ADDR1MUX,
    ADDR2MUX1, ADDR2MUX0,
    MARMUX,
    ALUK1, ALUK0,
    MIO_EN,
    R_W,
    DATA_SIZE,
    LSHF1,
    CONTROL_STORE_BITS
} CS_BITS;

/***************************************************************/
/* Functions to get at the control bits.                       */
/***************************************************************/
int GetIRD(int *x)           { return(x[IRD]); }
int GetCOND(int *x)          { return((x[COND1] << 1) + x[COND0]); }
int GetJ(int *x)             { return((x[J5] << 5) + (x[J4] << 4) +
				      (x[J3] << 3) + (x[J2] << 2) +
				      (x[J1] << 1) + x[J0]); }
int GetLD_MAR(int *x)        { return(x[LD_MAR]); }
int GetLD_MDR(int *x)        { return(x[LD_MDR]); }
int GetLD_IR(int *x)         { return(x[LD_IR]); }
int GetLD_BEN(int *x)        { return(x[LD_BEN]); }
int GetLD_REG(int *x)        { return(x[LD_REG]); }
int GetLD_CC(int *x)         { return(x[LD_CC]); }
int GetLD_PC(int *x)         { return(x[LD_PC]); }
int GetGATE_PC(int *x)       { return(x[GATE_PC]); }
int GetGATE_MDR(int *x)      { return(x[GATE_MDR]); }
int GetGATE_ALU(int *x)      { return(x[GATE_ALU]); }
int GetGATE_MARMUX(int *x)   { return(x[GATE_MARMUX]); }
int GetGATE_SHF(int *x)      { return(x[GATE_SHF]); }
int GetPCMUX(int *x)         { return((x[PCMUX1] << 1) + x[PCMUX0]); }
int GetDRMUX(int *x)         { return(x[DRMUX]); }
int GetSR1MUX(int *x)        { return(x[SR1MUX]); }
int GetADDR1MUX(int *x)      { return(x[ADDR1MUX]); }
int GetADDR2MUX(int *x)      { return((x[ADDR2MUX1] << 1) + x[ADDR2MUX0]); }
int GetMARMUX(int *x)        { return(x[MARMUX]); }
int GetALUK(int *x)          { return((x[ALUK1] << 1) + x[ALUK0]); }
int GetMIO_EN(int *x)        { return(x[MIO_EN]); }
int GetR_W(int *x)           { return(x[R_W]); }
int GetDATA_SIZE(int *x)     { return(x[DATA_SIZE]); } 
int GetLSHF1(int *x)         { return(x[LSHF1]); }

/***************************************************************/
/* The control store rom.                                      */
/***************************************************************/
int CONTROL_STORE[CONTROL_STORE_ROWS][CONTROL_STORE_BITS];

/***************************************************************/
/* Main memory.                                                */
/***************************************************************/
/* MEMORY[A][0] stores the least significant byte of word at word address A
   MEMORY[A][1] stores the most significant byte of word at word address A 
   There are two write enable signals, one for each byte. WE0 is used for 
   the least significant byte of a word. WE1 is used for the most significant 
   byte of a word. */

#define WORDS_IN_MEM    0x08000 
#define MEM_CYCLES      5
int MEMORY[WORDS_IN_MEM][2];

/***************************************************************/

/***************************************************************/

/***************************************************************/
/* LC-3b State info.                                           */
/***************************************************************/
#define LC_3b_REGS 8

int RUN_BIT;	/* run bit */
int BUS;	/* value of the bus */

typedef struct System_Latches_Struct{

int PC,		/* program counter */
    MDR,	/* memory data register */
    MAR,	/* memory address register */
    IR,		/* instruction register */
    N,		/* n condition bit */
    Z,		/* z condition bit */
    P,		/* p condition bit */
    BEN;        /* ben register */

int READY;	/* ready bit */
  /* The ready bit is also latched as you dont want the memory system to assert it 
     at a bad point in the cycle*/

int REGS[LC_3b_REGS]; /* register file. */

int MICROINSTRUCTION[CONTROL_STORE_BITS]; /* The microintruction */

int STATE_NUMBER; /* Current State Number - Provided for debugging */ 
} System_Latches;

/* Data Structure for Latch */

System_Latches CURRENT_LATCHES, NEXT_LATCHES;

/***************************************************************/
/* A cycle counter.                                            */
/***************************************************************/
int CYCLE_COUNT;

/***************************************************************/
/*                                                             */
/* Procedure : help                                            */
/*                                                             */
/* Purpose   : Print out a list of commands.                   */
/*                                                             */
/***************************************************************/
void help() {                                                    
    printf("----------------LC-3bSIM Help-------------------------\n");
    printf("go               -  run program to completion       \n");
    printf("run n            -  execute program for n cycles    \n");
    printf("mdump low high   -  dump memory from low to high    \n");
    printf("rdump            -  dump the register & bus values  \n");
    printf("?                -  display this help menu          \n");
    printf("quit             -  exit the program                \n\n");
}

/***************************************************************/
/*                                                             */
/* Procedure : cycle                                           */
/*                                                             */
/* Purpose   : Execute a cycle                                 */
/*                                                             */
/***************************************************************/
void cycle() {                                                

  eval_micro_sequencer();   
  cycle_memory();
  eval_bus_drivers();
  drive_bus();
  latch_datapath_values();

  CURRENT_LATCHES = NEXT_LATCHES;

  CYCLE_COUNT++;
}

/***************************************************************/
/*                                                             */
/* Procedure : run n                                           */
/*                                                             */
/* Purpose   : Simulate the LC-3b for n cycles.                 */
/*                                                             */
/***************************************************************/
void run(int num_cycles) {                                      
    int i;

    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating for %d cycles...\n\n", num_cycles);
    for (i = 0; i < num_cycles; i++) {
	if (CURRENT_LATCHES.PC == 0x0000) {
	    RUN_BIT = FALSE;
	    printf("Simulator halted\n\n");
	    break;
	}
	cycle();
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : go                                              */
/*                                                             */
/* Purpose   : Simulate the LC-3b until HALTed.                 */
/*                                                             */
/***************************************************************/
void go() {                                                     
    if (RUN_BIT == FALSE) {
	printf("Can't simulate, Simulator is halted\n\n");
	return;
    }

    printf("Simulating...\n\n");
    while (CURRENT_LATCHES.PC != 0x0000)
	cycle();
    RUN_BIT = FALSE;
    printf("Simulator halted\n\n");
}

/***************************************************************/ 
/*                                                             */
/* Procedure : mdump                                           */
/*                                                             */
/* Purpose   : Dump a word-aligned region of memory to the     */
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void mdump(FILE * dumpsim_file, int start, int stop) {          
    int address; /* this is a byte address */

    printf("\nMemory content [0x%.4x..0x%.4x] :\n", start, stop);
    printf("-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	printf("  0x%.4x (%d) : 0x%.2x%.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    printf("\n");

    /* dump the memory contents into the dumpsim file */
    fprintf(dumpsim_file, "\nMemory content [0x%.4x..0x%.4x] :\n", start, stop);
    fprintf(dumpsim_file, "-------------------------------------\n");
    for (address = (start >> 1); address <= (stop >> 1); address++)
	fprintf(dumpsim_file, " 0x%.4x (%d) : 0x%.2x%.2x\n", address << 1, address << 1, MEMORY[address][1], MEMORY[address][0]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : rdump                                           */
/*                                                             */
/* Purpose   : Dump current register and bus values to the     */   
/*             output file.                                    */
/*                                                             */
/***************************************************************/
void rdump(FILE * dumpsim_file) {                               
    int k; 

    printf("\nCurrent register/bus values :\n");
    printf("-------------------------------------\n");
    printf("Cycle Count  : %d\n", CYCLE_COUNT);
    printf("PC           : 0x%.4x\n", CURRENT_LATCHES.PC);
    printf("IR           : 0x%.4x\n", CURRENT_LATCHES.IR);
    printf("STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    printf("BUS          : 0x%.4x\n", BUS);
    printf("MDR          : 0x%.4x\n", CURRENT_LATCHES.MDR);
    printf("MAR          : 0x%.4x\n", CURRENT_LATCHES.MAR);
    printf("CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    printf("Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	printf("%d: 0x%.4x\n", k, CURRENT_LATCHES.REGS[k]);
    printf("\n");

    /* dump the state information into the dumpsim file */
    fprintf(dumpsim_file, "\nCurrent register/bus values :\n");
    fprintf(dumpsim_file, "-------------------------------------\n");
    fprintf(dumpsim_file, "Cycle Count  : %d\n", CYCLE_COUNT);
    fprintf(dumpsim_file, "PC           : 0x%.4x\n", CURRENT_LATCHES.PC);
    fprintf(dumpsim_file, "IR           : 0x%.4x\n", CURRENT_LATCHES.IR);
    fprintf(dumpsim_file, "STATE_NUMBER : 0x%.4x\n\n", CURRENT_LATCHES.STATE_NUMBER);
    fprintf(dumpsim_file, "BUS          : 0x%.4x\n", BUS);
    fprintf(dumpsim_file, "MDR          : 0x%.4x\n", CURRENT_LATCHES.MDR);
    fprintf(dumpsim_file, "MAR          : 0x%.4x\n", CURRENT_LATCHES.MAR);
    fprintf(dumpsim_file, "CCs: N = %d  Z = %d  P = %d\n", CURRENT_LATCHES.N, CURRENT_LATCHES.Z, CURRENT_LATCHES.P);
    fprintf(dumpsim_file, "Registers:\n");
    for (k = 0; k < LC_3b_REGS; k++)
	fprintf(dumpsim_file, "%d: 0x%.4x\n", k, CURRENT_LATCHES.REGS[k]);
    fprintf(dumpsim_file, "\n");
    fflush(dumpsim_file);
}

/***************************************************************/
/*                                                             */
/* Procedure : get_command                                     */
/*                                                             */
/* Purpose   : Read a command from standard input.             */  
/*                                                             */
/***************************************************************/
void get_command(FILE * dumpsim_file) {                         
    char buffer[20];
    int start, stop, cycles;

    printf("LC-3b-SIM> ");

    scanf("%s", buffer);
    printf("\n");

    switch(buffer[0]) {
    case 'G':
    case 'g':
	go();
	break;

    case 'M':
    case 'm':
	scanf("%i %i", &start, &stop);
	mdump(dumpsim_file, start, stop);
	break;

    case '?':
	help();
	break;
    case 'Q':
    case 'q':
	printf("Bye.\n");
	exit(0);

    case 'R':
    case 'r':
	if (buffer[1] == 'd' || buffer[1] == 'D')
	    rdump(dumpsim_file);
	else {
	    scanf("%d", &cycles);
	    run(cycles);
	}
	break;

    default:
	printf("Invalid Command\n");
	break;
    }
}

/***************************************************************/
/*                                                             */
/* Procedure : init_control_store                              */
/*                                                             */
/* Purpose   : Load microprogram into control store ROM        */ 
/*                                                             */
/***************************************************************/
void init_control_store(char *ucode_filename) {                 
    FILE *ucode;
    int i, j, index;
    char line[200];

    printf("Loading Control Store from file: %s\n", ucode_filename);

    /* Open the micro-code file. */
    if ((ucode = fopen(ucode_filename, "r")) == NULL) {
	printf("Error: Can't open micro-code file %s\n", ucode_filename);
	exit(-1);
    }

    /* Read a line for each row in the control store. */
    for(i = 0; i < CONTROL_STORE_ROWS; i++) {
	if (fscanf(ucode, "%[^\n]\n", line) == EOF) {
	    printf("Error: Too few lines (%d) in micro-code file: %s\n",
		   i, ucode_filename);
	    exit(-1);
	}

	/* Put in bits one at a time. */
	index = 0;

	for (j = 0; j < CONTROL_STORE_BITS; j++) {
	    /* Needs to find enough bits in line. */
	    if (line[index] == '\0') {
		printf("Error: Too few control bits in micro-code file: %s\nLine: %d\n",
		       ucode_filename, i);
		exit(-1);
	    }
	    if (line[index] != '0' && line[index] != '1') {
		printf("Error: Unknown value in micro-code file: %s\nLine: %d, Bit: %d\n",
		       ucode_filename, i, j);
		exit(-1);
	    }

	    /* Set the bit in the Control Store. */
	    CONTROL_STORE[i][j] = (line[index] == '0') ? 0:1;
	    index++;
	}

	/* Warn about extra bits in line. */
	if (line[index] != '\0')
	    printf("Warning: Extra bit(s) in control store file %s. Line: %d\n",
		   ucode_filename, i);
    }
    printf("\n");
}

/************************************************************/
/*                                                          */
/* Procedure : init_memory                                  */
/*                                                          */
/* Purpose   : Zero out the memory array                    */
/*                                                          */
/************************************************************/
void init_memory() {                                           
    int i;

    for (i=0; i < WORDS_IN_MEM; i++) {
	MEMORY[i][0] = 0;
	MEMORY[i][1] = 0;
    }
}

/**************************************************************/
/*                                                            */
/* Procedure : load_program                                   */
/*                                                            */
/* Purpose   : Load program and service routines into mem.    */
/*                                                            */
/**************************************************************/
void load_program(char *program_filename) {                   
    FILE * prog;
    int ii, word, program_base;

    /* Open program file. */
    prog = fopen(program_filename, "r");
    if (prog == NULL) {
	printf("Error: Can't open program file %s\n", program_filename);
	exit(-1);
    }

    /* Read in the program. */
    if (fscanf(prog, "%x\n", &word) != EOF)
	program_base = word >> 1;
    else {
	printf("Error: Program file is empty\n");
	exit(-1);
    }

    ii = 0;
    while (fscanf(prog, "%x\n", &word) != EOF) {
	/* Make sure it fits. */
	if (program_base + ii >= WORDS_IN_MEM) {
	    printf("Error: Program file %s is too long to fit in memory. %x\n",
		   program_filename, ii);
	    exit(-1);
	}

	/* Write the word to memory array. */
	MEMORY[program_base + ii][0] = word & 0x00FF;
	MEMORY[program_base + ii][1] = (word >> 8) & 0x00FF;
	ii++;
    }

    if (CURRENT_LATCHES.PC == 0) CURRENT_LATCHES.PC = (program_base << 1);

    printf("Read %d words from program into memory.\n\n", ii);
}

/***************************************************************/
/*                                                             */
/* Procedure : initialize                                      */
/*                                                             */
/* Purpose   : Load microprogram and machine language program  */ 
/*             and set up initial state of the machine.        */
/*                                                             */
/***************************************************************/
void initialize(char *ucode_filename, char *program_filename, int num_prog_files) { 
    int i;
    init_control_store(ucode_filename);

    init_memory();
    for ( i = 0; i < num_prog_files; i++ ) {
	load_program(program_filename);
	while(*program_filename++ != '\0');
    }
    CURRENT_LATCHES.Z = 1;
    CURRENT_LATCHES.STATE_NUMBER = INITIAL_STATE_NUMBER;
    memcpy(CURRENT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[INITIAL_STATE_NUMBER], sizeof(int)*CONTROL_STORE_BITS);

    NEXT_LATCHES = CURRENT_LATCHES;

    RUN_BIT = TRUE;
}

/***************************************************************/
/*                                                             */
/* Procedure : main                                            */
/*                                                             */
/***************************************************************/
int main(int argc, char *argv[]) {                            
    FILE * dumpsim_file;

    /* Error Checking */
    if (argc < 3) {
	printf("Error: usage: %s <micro_code_file> <program_file_1> <program_file_2> ...\n",
	       argv[0]);
	exit(1);
    }

    printf("LC-3b Simulator\n\n");

    initialize(argv[1], argv[2], argc - 2);

    if ( (dumpsim_file = fopen( "dumpsim", "w" )) == NULL ) {
	printf("Error: Can't open dumpsim file\n");
	exit(-1);
    }

    while (1)
	get_command(dumpsim_file);

}

/***************************************************************/
/* Do not modify the above code.
   You are allowed to use the following global variables in your
   code. These are defined above.

   CONTROL_STORE
   MEMORY
   BUS

   CURRENT_LATCHES
   NEXT_LATCHES

   You may define your own local/global variables and functions.
   You may use the functions to get at the control bits defined
   above.

   Begin your code here 	  			       */
/***************************************************************/


void eval_micro_sequencer() {

  /* 
   * Evaluate the address of the next state according to the 
   * micro sequencer logic. Latch the next microinstruction.
   */

  int irdBits = GetIRD(CURRENT_LATCHES.MICROINSTRUCTION);
  int condBits = GetCOND(CURRENT_LATCHES.MICROINSTRUCTION);
  int jBits = GetJ(CURRENT_LATCHES.MICROINSTRUCTION);

  int nextMicroinstructionAddress = 0;

  if(irdBits){
      // get the opCode bits for next iteration
      int instructionBits = ((CURRENT_LATCHES.IR >> (11)) & 0x000F);
      nextMicroinstructionAddress = instructionBits;
  }else{
      if(condBits == 1){
          // couple the R bit to jBits
          nextMicroinstructionAddress = jBits || (CURRENT_LATCHES.READY << 1);
      }else if(condBits == 2){
          // couple the BEN bit to jBits
          nextMicroinstructionAddress = jBits || (CURRENT_LATCHES.BEN << 2);
      }else if(condBits == 3){
          // couple of IR[11] bit to jBits
          nextMicroinstructionAddress = jBits || ((CURRENT_LATCHES.IR >> 11) & 0x0001);
      }
  }

  memcpy(NEXT_LATCHES.MICROINSTRUCTION, CONTROL_STORE[nextMicroinstructionAddress], sizeof(int)*CONTROL_STORE_BITS);
}


void cycle_memory() {
 
  /* 
   * This function emulates memory and the WE logic. 
   * Keep track of which cycle of MEMEN we are dealing with.  
   * If fourth, we need to latch Ready bit at the end of 
   * cycle to prepare microsequencer for the fifth cycle.  
   */

  // check MIO.EN
  static int cycleNumber = 1;

  if(cycleNumber == 4){
    int rwBit = GetR_W(CURRENT_LATCHES.MICROINSTRUCTION);
    int marZeroVal = CURRENT_LATCHES.MAR & 0x0001;
    int datasizeVal = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);
    int memoryAddress = CURRENT_LATCHES.MAR >> 1;
    if(rwBit){
        // write
        if(datasizeVal){
            MEMORY[memoryAddress][1] = (CURRENT_LATCHES.MDR & 0xFF00) >> 8;
            MEMORY[memoryAddress][0] = (CURRENT_LATCHES.MDR & 0x00FF); 
        }else{
            if(marZeroVal){
                MEMORY[memoryAddress][1] = (CURRENT_LATCHES.MDR & 0xFF00) >> 8;
            }else{
                MEMORY[memoryAddress][0] = (CURRENT_LATCHES.MDR & 0x00FF); 
            }
        }
    }else{
        // read
        if(datasizeVal){
            NEXT_LATCHES.MDR = MEMORY[memoryAddress][1] << 8 | MEMORY[memoryAddress][0];
        }else{
            if(marZeroVal){
                NEXT_LATCHES.MDR = MEMORY[memoryAddress][1] >> 8;
            }else{
                NEXT_LATCHES.MDR = MEMORY[memoryAddress][0]; 
            }
        }
    }
    NEXT_LATCHES.READY = 1;
  }else{
    NEXT_LATCHES.READY = 0;
  }

  cycleNumber = (cycleNumber + 1) % 4;
}

int gateAluVal = 0;
int gateMarmuxVal = 0;
int gateMdrVal = 0;
int gatePcVal = 0;
int gateShfVal = 0;

int aluVal = 0;
int marmuxVal = 0;
int mdrVal = 0;
int pcVal = 0;
int shfVal = 0;

void eval_bus_drivers() {


  /* 
   * Datapath routine emulating operations before driving the bus.
   * Evaluate the input of tristate drivers 
   *             Gate_MARMUX,
   *		 Gate_PC,
   *		 Gate_ALU,
   *		 Gate_SHF,
   *		 Gate_MDR.
   */

  gateMarmuxVal = GetGATE_MARMUX(CURRENT_LATCHES.MICROINSTRUCTION);
  gatePcVal = GetGATE_PC(CURRENT_LATCHES.MICROINSTRUCTION);
  gateAluVal = GetGATE_ALU(CURRENT_LATCHES.MICROINSTRUCTION);
  gateShfVal = GetGATE_SHF(CURRENT_LATCHES.MICROINSTRUCTION);
  gateMdrVal = GetGATE_MDR(CURRENT_LATCHES.MICROINSTRUCTION);

  if(gateAluVal){
    int alukVal = GetALUK(CURRENT_LATCHES.MICROINSTRUCTION);
    int sr1MuxVal = GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
    int inputARegister = 0;
    if(sr1MuxVal){
        inputARegister = (CURRENT_LATCHES.IR >> 6) & 0x0007;
    }else{
        inputARegister = (CURRENT_LATCHES.IR >> 9) & 0x0007;
    }
    int inputAVal = CURRENT_LATCHES.REGS[inputARegister];
    
    int sr2Mux = (CURRENT_LATCHES.IR >> 5) & 0x0001;
    int inputBVal = 0;
    if(sr2Mux){
        // take an imm5 val from IR
        int imm5Val = (CURRENT_LATCHES.IR & 0x001F);
        if((imm5Val >> 4)){
            // sign extend imm5Val
            inputBVal = 0xFF00 & imm5Val;
        }else{
            inputBVal = imm5Val;
        }
    }else{
        // take a value from IR register bits[2:0]
        int sr2RegNum = CURRENT_LATCHES.IR & 0x0007;
        inputBVal = CURRENT_LATCHES.REGS[sr2RegNum];
    }

    if(alukVal == 0){
        //ADD
        gateAluVal = (inputBVal + inputAVal) & 0xFFFF;
    }else if(alukVal == 1){
        //AND
        gateAluVal = (inputBVal & inputAVal) & 0xFFFF;
    }else if(alukVal == 2){
        //XOR
        gateAluVal = (inputBVal ^ inputAVal) & 0xFFFF;
    }else if(alukVal == 3){
        //PASSA
        gateAluVal = (inputAVal) & 0xFFFF;
    }
  }else if(gateMarmuxVal){
    int addr1muxVal = GetADDR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
    int sr1muxVal = GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
    int addr2muxVal = GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION);
    int lshf1Val = GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION);
    int marmuxVal = GetMARMUX(CURRENT_LATCHES.MICROINSTRUCTION);

    if(marmuxVal){
        // select LSHF(ZEXT[IR[7:0]],1)
        marmuxVal = (CURRENT_LATCHES.IR & 0x00FF) << 1;
    }else{
        // output of address adder
        int addressAdderRightInput = 0;
        int addressAdderLeftInput = 0;
        if(addr1muxVal){
            // use BaseR
            int baseRegNum = 0;
            if(sr1muxVal){
                // IR[8:6]
                baseRegNum = (CURRENT_LATCHES.IR >> 6) & 0x0007;
            }else{
                // IR[11:9]
                baseRegNum = (CURRENT_LATCHES.IR >> 9) & 0x0007;
            }
            addressAdderRightInput = CURRENT_LATCHES.REGS[baseRegNum];
        }else{
            // use PC
            addressAdderRightInput = CURRENT_LATCHES.PC;
        }

        int addressLeftInputTempVal = 0;
        if(addr2muxVal == 0){
            addressLeftInputTempVal = 0;
        }else if(addr2muxVal == 1){
            //offset6
            addressLeftInputTempVal = CURRENT_LATCHES.IR & 0x003F;
            if(addressLeftInputTempVal >> 5){
                addressLeftInputTempVal |= 0xFF40;
            }
        }else if(addr2muxVal == 2){
            //offset9
            addressLeftInputTempVal = CURRENT_LATCHES.IR & 0x01FF;
            if(addressLeftInputTempVal >> 8){
                addressLeftInputTempVal |= 0xFE00;
            }
        }else if(addr2muxVal == 3){
            //offset11
            addressLeftInputTempVal = CURRENT_LATCHES.IR & 0x07FF;
            if(addressLeftInputTempVal >> 10){
                addressLeftInputTempVal |= 0xF800;
            }
        }

        if(lshf1Val){
            addressLeftInputTempVal = addressLeftInputTempVal << 1;
            addressLeftInputTempVal &= 0xFFFF;
        }

        addressAdderLeftInput = addressLeftInputTempVal;
        marmuxVal = addressAdderLeftInput + addressAdderRightInput;
    }
  }else if(gateMdrVal){
    int datasizeVal = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);
    int marZeroVal = CURRENT_LATCHES.MAR & 0x0001;
    int currentMdrVal = CURRENT_LATCHES.MDR;
    if(datasizeVal){
        mdrVal = currentMdrVal;
    }else{
        if(marZeroVal == 0){
            mdrVal = (currentMdrVal & 0x00FF);
            if(mdrVal & 0x0080){
                mdrVal |= 0xFF;
            }
        }else{
            mdrVal = (currentMdrVal & 0xFF00) >> 8;
            if(mdrVal & 0x0080){
                mdrVal |= 0xFF;
            }
        }
    }
  }else if(gatePcVal){
    pcVal = CURRENT_LATCHES.PC;
  }else if(gateShfVal){
    int sr1muxVal = GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
    int sr1RegNum = 0;
    if(sr1muxVal){
        //IR[8:6]
        sr1RegNum = (CURRENT_LATCHES.IR >> 6) & 0x0007;
    }else{
        //IR[11:9]
        sr1RegNum = (CURRENT_LATCHES.IR >> 9) & 0x0007;
    }
    int shfNum = (CURRENT_LATCHES.IR & 0x003F);
    int lshf = shfNum >> 4;
    if(!lshf){
        shfVal = (CURRENT_LATCHES.REGS[sr1RegNum] << shfNum) & 0xFFFF;
    }else{
        int arithmeticVal = lshf;
        if(arithmeticVal == 1){
            shfVal = (CURRENT_LATCHES.REGS[sr1RegNum] >> shfNum) & 0xFFFF;
        }else{
            int arithmeticBit = CURRENT_LATCHES.REGS[sr1RegNum] >> 15;
            int mask = 0xFFFF << (15-shfNum);
            if(arithmeticBit){
                shfVal = (CURRENT_LATCHES.REGS[sr1RegNum] >> shfNum) | mask;
            }else{
                shfVal = (CURRENT_LATCHES.REGS[sr1RegNum] >> shfNum) & 0xFFFF;
            }
            shfVal = shfVal & 0xFFFF;
        }
    }
  }
}


void drive_bus() {

  /* 
   * Datapath routine for driving the bus from one of the 5 possible 
   * tristate drivers. 
   */
  if(gateAluVal){
    BUS = aluVal;
  }else if(gateMarmuxVal){
    BUS = marmuxVal;
  }else if(gateMdrVal){
    BUS = mdrVal;
  }else if(gatePcVal){
    BUS = pcVal;
  }else if(gateShfVal){
    BUS = shfVal;
  }
}


void latch_datapath_values() {

  /* 
   * Datapath routine for computing all functions that need to latch
   * values in the data path at the end of this cycle.  Some values
   * require sourcing the bus; therefore, this routine has to come 
   * after drive_bus.
   */       
  
  int ldMarVal = GetLD_MAR(CURRENT_LATCHES.MICROINSTRUCTION);
  int ldMdrVal = GetLD_MDR(CURRENT_LATCHES.MICROINSTRUCTION);
  int ldIrVal = GetLD_IR(CURRENT_LATCHES.MICROINSTRUCTION);
  int ldBenVal = GetLD_BEN(CURRENT_LATCHES.MICROINSTRUCTION);
  int ldRegVal = GetLD_REG(CURRENT_LATCHES.MICROINSTRUCTION);
  int ldCcVal = GetLD_CC(CURRENT_LATCHES.MICROINSTRUCTION);
  int ldPcVal = GetLD_PC(CURRENT_LATCHES.MICROINSTRUCTION);

  if(ldMarVal){
    NEXT_LATCHES.MAR = BUS;
  }

  // come back to this one
  if(ldMdrVal){
    int datasizeVal = GetDATA_SIZE(CURRENT_LATCHES.MICROINSTRUCTION);
    int marZeroVal = CURRENT_LATCHES.MDR & 0x0001;
    int mioEnVal = GetMIO_EN(CURRENT_LATCHES.MICROINSTRUCTION);

    if(!mioEnVal){
        // load MDR from bus
        if(!marZeroVal){
            NEXT_LATCHES.MDR = BUS;
        }else{
            int busSevenZero = BUS & 0x00FF;
            NEXT_LATCHES.MDR = (busSevenZero << 8) | busSevenZero;
        }
    }else{
        // load MDR from memory
        int memoryLocation = CURRENT_LATCHES.MAR >> 1;
        int lowerByte = MEMORY[memoryLocation][0];
        int upperByte = MEMORY[memoryLocation][1];
        if(datasizeVal){
            // get 16 bits
            NEXT_LATCHES.MDR = (upperByte << 8) | lowerByte;
        }else{
            if(marZeroVal){
                NEXT_LATCHES.MDR = upperByte;
                if(upperByte & 0x0080){
                    NEXT_LATCHES.MDR |= 0xFF00;
                }
            }else{
                NEXT_LATCHES.MDR = lowerByte;
                if(upperByte & 0x0080){
                    NEXT_LATCHES.MDR |= 0xFF00;
                }
            }
        }
    }


  }

  if(ldIrVal){
    NEXT_LATCHES.IR = BUS;
  }

  if(ldBenVal){
    int nBit = CURRENT_LATCHES.IR >> 11;
    int zBit = CURRENT_LATCHES.IR >> 10;
    int pBit = CURRENT_LATCHES.IR >> 9;

    NEXT_LATCHES.BEN = (CURRENT_LATCHES.N && nBit) || (CURRENT_LATCHES.Z && zBit) || (CURRENT_LATCHES.P && pBit);
  }

  if(ldRegVal){
    int drMuxVal = GetDRMUX(CURRENT_LATCHES.MICROINSTRUCTION);
    int drRegNum = 0;
    // 0 is IR[11:9]
    // 1 is 7
    if(drMuxVal){
        drRegNum = 7;
    }else{
        drRegNum = (CURRENT_LATCHES.IR >> 9) & 0x0007;
    }
    NEXT_LATCHES.REGS[drRegNum] = BUS;

  }

  if(ldCcVal){
    NEXT_LATCHES.N = 0;
    NEXT_LATCHES.Z = 0;
    NEXT_LATCHES.P = 0;
    if(BUS == 0){
        NEXT_LATCHES.Z = 1;
    }else if(BUS > 0){
        NEXT_LATCHES.P = 1;
    }else if(BUS < 0){
        NEXT_LATCHES.N = 1;
    }
  }

  if(ldPcVal){
    int pcMuxVal = GetPCMUX(CURRENT_LATCHES.MICROINSTRUCTION);
    // 0 is PC + 2
    // 1 is BUS
    // 2 is ADDER

    if(pcMuxVal == 0){
        NEXT_LATCHES.PC = CURRENT_LATCHES.PC + 2;
    }else if(pcMuxVal == 1){
        NEXT_LATCHES.PC = BUS;
    }else if(pcMuxVal == 2){
        int addr1muxVal = GetADDR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
        int sr1muxVal = GetSR1MUX(CURRENT_LATCHES.MICROINSTRUCTION);
        int addr2muxVal = GetADDR2MUX(CURRENT_LATCHES.MICROINSTRUCTION);
        int lshf1Val = GetLSHF1(CURRENT_LATCHES.MICROINSTRUCTION);
        int marmuxVal = GetMARMUX(CURRENT_LATCHES.MICROINSTRUCTION);
        // output of address adder
        int addressAdderRightInput = 0;
        int addressAdderLeftInput = 0;
        if(addr1muxVal){
            // use BaseR
            int baseRegNum = 0;
            if(sr1muxVal){
                // IR[8:6]
                baseRegNum = (CURRENT_LATCHES.IR >> 6) & 0x0007;
            }else{
                // IR[11:9]
                baseRegNum = (CURRENT_LATCHES.IR >> 9) & 0x0007;
            }
            addressAdderRightInput = CURRENT_LATCHES.REGS[baseRegNum];
        }else{
            // use PC
            addressAdderRightInput = CURRENT_LATCHES.PC;
        }

        int addressLeftInputTempVal = 0;
        if(addr2muxVal == 0){
            addressLeftInputTempVal = 0;
        }else if(addr2muxVal == 1){
            //offset6
            addressLeftInputTempVal = CURRENT_LATCHES.IR & 0x003F;
            if(addressLeftInputTempVal >> 5){
                addressLeftInputTempVal |= 0xFF40;
            }
        }else if(addr2muxVal == 2){
            //offset9
            addressLeftInputTempVal = CURRENT_LATCHES.IR & 0x01FF;
            if(addressLeftInputTempVal >> 8){
                addressLeftInputTempVal |= 0xFE00;
            }
        }else if(addr2muxVal == 3){
            //offset11
            addressLeftInputTempVal = CURRENT_LATCHES.IR & 0x07FF;
            if(addressLeftInputTempVal >> 10){
                addressLeftInputTempVal |= 0xF800;
            }
        }

        if(lshf1Val){
            addressLeftInputTempVal = addressLeftInputTempVal << 1;
            addressLeftInputTempVal &= 0xFFFF;
        }

        addressAdderLeftInput = addressLeftInputTempVal;
        NEXT_LATCHES.PC = addressAdderLeftInput + addressAdderRightInput;
    }
  }


}
/* Header for the LittleComputer-3 VM
 *
 * AUTHOR: Pierre-Charles Dussault
 * SINCE: 2021/10/26
 * */
#ifndef LC3_FUNCS_14234234
#define LC3_FUNCS_14234234

/* Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
/* Unix specific */
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/mman.h>


/* Function declarations */
uint16_t sign_extend(uint16_t x, int bit_count);
void update_flags(uint16_t r);
void handle_ADD(uint16_t instruction, uint16_t regs[]);
void handle_AND(uint16_t instruction, uint16_t regs[]);
void handle_NOT(uint16_t instruction, uint16_t regs[]);
void handle_BR(uint16_t instruction, uint16_t regs[]);
void handle_LD(uint16_t instruction, uint16_t regs[]);
void handle_LDI(uint16_t instruction, uint16_t regs[]);
void handle_LDR(uint16_t instruction, uint16_t regs[]);
void handle_LEA(uint16_t instruction, uint16_t regs[]);
void handle_JMP(uint16_t instruction, uint16_t regs[]);
void handle_JSR(uint16_t instruction, uint16_t regs[]);
void handle_ST(uint16_t instruction, uint16_t regs[]);
void handle_STI(uint16_t instruction, uint16_t regs[]);
void handle_STR(uint16_t instruction, uint16_t regs[]);
void handle_TRAP(uint16_t instruction, uint16_t regs[], int* running);
void handle_RES(uint16_t instruction, uint16_t regs[]);
void handle_RTI(uint16_t instruction, uint16_t regs[]);
void handle_BAD_OPCODE(uint16_t instruction, uint16_t regs[]);
void handle_TRAP_GETC();
void handle_TRAP_OUT();
void handle_TRAP_PUTS();
void handle_TRAP_IN();
void handle_TRAP_PUTSP();
void handle_TRAP_HALT(int* running);
uint16_t swap16(uint16_t to_swap);
void read_image_file(FILE* file);
int read_image(const char* image_path);
uint16_t mem_read(uint16_t address);
void mem_write(uint16_t address, uint16_t value);
uint16_t check_key();
void disable_input_buffering();
void restore_input_buffering();
void handle_interrupt(int signal);


/* ----- VM Setup ----- */

/* Maximum number of 16bit blocks for the memory */
uint16_t memory[UINT16_MAX];

/* Registers
 * The registers used are not actually referenced by their actual address, but
 * by their position inside the register array. The value for each of these
 * constants is that registers index position inside regs[] (see its
 * declaration right below the enum) */
enum {
  R_R0 = 0,
  R_R1,
  R_R2,
  R_R3,
  R_R4,
  R_R5,
  R_R6, /* Stack pointer */
  R_R7, /* Links back to instruction after a subroutine returns */
  R_PC, /* Program counter */
  R_COND, /* Info about previous calculation */
  R_COUNT /* Total number of registers, including itself */
};

/* Store the registers inside an array */
uint16_t regs[R_COUNT];

/* Opcodes for the instruction set, with their respective instruction in
 * comments in the following format:
 *
 *      / <instruction> <description> /
 * */
enum {
  OP_BR = 0, /* BR Branch*/
  OP_ADD, /* ADD Add */
  OP_LD, /* LD Load */
  OP_ST, /* ST Store */
  OP_JSR, /* JSR Jump to subroutine */
  OP_AND, /* AND And */
  OP_LDR, /* LDR Load register */
  OP_STR, /* STR Store register */
  OP_RTI, /* RTI Unused */
  OP_NOT, /* NOT Not */
  OP_LDI, /* LDI Load indirect */
  OP_STI, /* STI Store indirect */
  OP_JMP, /* JMP Jump */
  OP_RES, /* RES Reserved (unused) */
  OP_LEA, /* LEA Load effective address */
  OP_TRAP /* TRAP Execute trap */
};

/* Condition flags */
enum {
  FL_P = 1 << 0, /* P Positive [001] */
  FL_Z = 1 << 1, /* Z Zero [010] */
  FL_N = 1 << 2 /* N Negative [100] */
};

/* Codes for TRAP routines */
enum {
  TRAP_GETC = 0x20, /* Get a char from keyboard, not echoed on the terminal */
  TRAP_OUT = 0x21, /* Output a char */
  TRAP_PUTS = 0x22, /* Output a word string */
  TRAP_IN = 0x23, /* Get a char from keyboard, echoed on the terminal */
  TRAP_PUTSP = 0x24, /* Output a byte string */
  TRAP_HALT = 0x25 /* Halt the program */
};

/* Memory-mapped registers for the LittleComputer-3 */
enum {
  MR_KBSR = 0xFE00, /* Keyboard status (was a key pressed?) */
  MR_KBDR = 0xFE02 /* Keyboard data (which key was it?) */
};

/* Data structure for use with terminal access in Unix */
struct termios original_tio;
/* end of VM Setup */


/* ----- Helper functions ----- */

uint16_t sign_extend(uint16_t x, int bit_count) {
  if ((x >> (bit_count - 1)) & 0x1) {
    /* The condition is triggered if the last bit == 1, indicating a negative
     * number */
    x |= (0xFFFF << bit_count);
  }

  /* The system will automatically fill the bit vector with left-most zeros if
   * we do not handle the bit vector extension ourselves, so we don't explicitely
   * need to account for the case where the number is positive */
  return x;
}

void update_flags(uint16_t r_index) {
  if (regs[r_index] == 0) {
    regs[R_COND] = FL_Z;
  } else if (regs[r_index] >> 15) { /* 1 in left-most bit means a negative number */
    regs[R_COND] = FL_N;
  } else {
    regs[R_COND] = FL_P;
  }
}

void handle_ADD(uint16_t instruction, uint16_t regs[]) {
  /* Destionation register (DR) at bits [9:11] in the bit vector. Shorten the
   * 16bit vector so that it begins with bit [9]. Then filter out the bits
   * over the bit [11] using an AND with a 111 bitmask, since bits [9 : 11]
   * will be the first 3 bits */
  uint16_t dr_index = (instruction >> 9) & 0x7;
  /* First operand (SR1) */
  uint16_t sr1_index = (instruction >> 6) & 0x7;
  /* Whether we are in immediate mode */
  uint16_t imm_flag = (instruction >> 5) & 0x1;

  if (imm_flag) {
    uint16_t imm5 = sign_extend(instruction & 0x1F, 5);
    regs[dr_index] = regs[sr1_index] + imm5;
  } else {
    uint16_t sr2_index = instruction & 0x7;
    /* This sum must be representable by 16bits or bits will fall off. */
    regs[dr_index] = regs[sr1_index] + regs[sr2_index];
  }

  update_flags(dr_index);
}

void handle_AND(uint16_t instruction, uint16_t regs[]) {
  /* Gather instruction and filter with bitmask */
  uint16_t dr_index = (instruction >> 9) & 0x7;
  uint16_t sr1_index = (instruction >> 6) & 0x7;

  uint16_t imm_flag = instruction & 0x20; /* 100000 bitmask, alternative method
                                           * to obtain the desired bit*/

  if (imm_flag) {
    /* If bit [5] == 1, DR = SR1 AND imm5 */
    uint16_t imm5 = sign_extend(instruction & 0x1F, 5);
    regs[dr_index] = regs[sr1_index] & imm5;

  } else {
    /* If bit [5] == 0, DR = SR1 AND SR2 */
    uint16_t sr2_index = instruction & 0x7;
    regs[dr_index] = regs[sr1_index] & regs[sr2_index];
  }

  update_flags(dr_index);
}

void handle_NOT(uint16_t instruction, uint16_t regs[]) {
  uint16_t dr_index = (instruction >> 9) & 0x7;
  uint16_t sr_index = (instruction >> 6) & 0x7;
  regs[dr_index] = ~regs[sr_index];
  update_flags(dr_index);
}

void handle_BR(uint16_t instruction, uint16_t regs[]) {
  uint16_t offset = sign_extend(instruction & 0x1FF, 9);
  uint16_t cond_flag = (instruction >> 9) & 0x7;

  /* Here, we are deliberately not fully implementing the complete BRANCH
   * specification, omitting the case of unconditional branching, when the
   * cond_flag vector is [000] */
  if (cond_flag & regs[R_COND])
    regs[R_PC] += offset;
}

void handle_JMP(uint16_t instruction, uint16_t regs[]) {
  uint16_t baseR_index = (instruction >> 6) & 0x7;
  regs[R_PC] = regs[baseR_index];
}

void handle_JSR(uint16_t instruction, uint16_t regs[]) {
  uint16_t flag = instruction & 0x800; /* Again the alternative way to gather
                                        * the bit using only a bitmask without
                                        * using a bit shift */
  regs[R_R7] = regs[R_PC];
  if (flag) {
    uint16_t offset = sign_extend(instruction & 0x7FF, 11);
    regs[R_PC] += offset; /* JSR */
  } else {
    uint16_t baseR_index = (instruction >> 6) & 0x7;
    regs[R_PC] = regs[baseR_index]; /* JSRR */
  }
}

void handle_LD(uint16_t instruction, uint16_t regs[]) {
  uint16_t dr_index = (instruction >> 9) & 0x7;
  uint16_t offset = sign_extend(instruction & 0x1FF, 9);
  regs[dr_index] = mem_read(regs[R_PC] + offset);
  update_flags(dr_index);
}

void handle_LDI(uint16_t instruction, uint16_t regs[]) {
  /* Get the DR (bits [9:11]) by moving starting bit to bit 9, and then use
   * bitmask to filter out exceeding bits */
  uint16_t dr_index = (instruction >> 9) & 0x7;
  /* Bitmask to only keep bits [0:8] (the contents of PCoffset9) */
  uint16_t relevant_instr = instruction & 0x1FF;
  /* Gather first memory address from PCoffset9 by sign-extending it*/
  uint16_t pc_offset = sign_extend(relevant_instr, 9);
  /* Gather the second memory address (to be stored in the DR) at the memory
   * address obtained by adding the one we just found with the address of the
   * PC. */
  uint16_t sign_extended_reference = regs[R_PC] + pc_offset;
  uint16_t second_reference = mem_read(sign_extended_reference);
  uint16_t real_reference_to_data = mem_read(second_reference);
  regs[dr_index] = real_reference_to_data;
  update_flags(dr_index);
}

void handle_LDR(uint16_t instruction, uint16_t regs[]) {
  uint16_t dr_index = (instruction >> 9) & 0x7;
  uint16_t baseR_index = (instruction >> 6) & 0x7;
  uint16_t offset = sign_extend(instruction & 0x3F, 6);
  regs[dr_index] = mem_read(regs[baseR_index] + offset);
  update_flags(dr_index);
}

void handle_LEA(uint16_t instruction, uint16_t regs[]) {
  uint16_t dr_index = (instruction >> 9) & 0x7;
  uint16_t offset = sign_extend(instruction & 0x1FF, 9);
  regs[dr_index] = regs[R_PC] + offset;
  update_flags(dr_index);
}

void handle_ST(uint16_t instruction, uint16_t regs[]) {
  uint16_t sr_index = (instruction >> 9) & 0x7;
  uint16_t offset = sign_extend(instruction & 0x1FF, 9);
  mem_write(regs[R_PC] + offset, regs[sr_index]);
}

void handle_STI(uint16_t instruction, uint16_t regs[]) {
  uint16_t sr_index = (instruction >> 9) & 0x7;
  uint16_t offset = sign_extend(instruction & 0x1FF, 9);
  mem_write(mem_read(regs[R_PC] + offset), regs[sr_index]);
}

void handle_STR(uint16_t instruction, uint16_t regs[]) {
  uint16_t sr_index = (instruction >> 9) & 0x7;
  uint16_t baseR_index = (instruction >> 6) & 0x7;
  uint16_t offset = sign_extend(instruction & 0x3F, 6); 
  mem_write(regs[baseR_index] + offset, regs[sr_index]);
}

void handle_TRAP(uint16_t instruction, uint16_t regs[], int* running) {
  uint16_t trap_code = instruction & 0xFF;
  switch(trap_code) {
    case TRAP_GETC:
      handle_TRAP_GETC();
      break;
    case TRAP_OUT:
      handle_TRAP_OUT();
      break;
    case TRAP_PUTS:
      handle_TRAP_PUTS();
      break;
    case TRAP_IN:
      handle_TRAP_IN();
      break;
    case TRAP_PUTSP:
      handle_TRAP_PUTSP();
      break;
    case TRAP_HALT:
      handle_TRAP_HALT(running);
      break;
  }
}

void handle_RTI(uint16_t instruction, uint16_t regs[]) {
  /* RTI is unused in this VM */
  handle_BAD_OPCODE(instruction, regs);
}

void handle_RES(uint16_t instruction, uint16_t regs[]) {
  /* RES is unused in this VM */
  handle_BAD_OPCODE(instruction, regs);
}

void handle_BAD_OPCODE(uint16_t instruction, uint16_t regs[]) {
  /* Get arguments in case we could ever implement them in the bad opcode
   * subroutine */
  abort();
}

void handle_TRAP_GETC() {
  regs[R_R0] = (uint16_t) getchar();
}

void handle_TRAP_OUT() {
  putc((char) regs[R_R0], stdout);
  fflush(stdout);
}

void handle_TRAP_PUTS() {
  /* In memory, treat the characters as uint16_t values */
  /* The string start at memory_start + R0 */
  uint16_t *c = memory + regs[R_R0];

  /* The string terminates with the occurence of 0x0000 in a memory location
   * per the specification, so we can use that to knw when to stop the loop */
  while(*c) {
    /* Cast the char values to char type (8bit) in order to use the putc()
     * function, as chars in C are 8bits, but here we stored them as 16bits */
    putc((char) *c, stdout);
    c++;
  }

  fflush(stdout);
}

void handle_TRAP_IN() {
  printf("Enter a character: ");
  char c = getchar();
  putc(c, stdout);
  regs[R_R0] = (uint16_t) c;
  fflush(stdout);
}

void handle_TRAP_PUTSP() {
  /* This stores one char per byte, as opposed to one char per 16bit vector,
   * so each address would contain 2 chars instead of one */

  uint16_t *c = memory + regs[R_R0];
  uint16_t current;
  /* End loop when *c == 0x0000 */
  while (*c) {
    current = *c & 0xFF; /* Get the first 8 bits */
    putc((char) current, stdout);
    current = *c & 0xFF00; /* Get the last 8 bits */
    /* In case of odd number of chars, with the last 8 bits being equal to 0x00,
     * it doesn't matter, as it will output NULL and it won't affect the string
     * */
    putc((char) current, stdout);
    c++;
  }

  fflush(stdout);
}

void handle_TRAP_HALT(int* running) {
  puts("HALT");
  fflush(stdout);
  *running = 0;
}

uint16_t swap16(uint16_t to_swap) {
  /* LittleComputer-3 programs are BigEndian, so in order for my consumer PC
   * (which is LittleEndian) to read the byte-order of values correctly, we
   * need to switch them into LittleEndian, because the program will be
   * providing them as BigEndian initially */
  return (to_swap << 8) | (to_swap >> 8);
}

void read_image_file(FILE* file) {
  /* Read a file from the origin until we hit the max value possible to read
   * given the size of our memory's array */
  
  uint16_t origin;
  /* Load only the first 16bits in the file */
  fread(&origin, sizeof(origin), 1, file);
  /* Convert the first 16bits to Little Endian */
  /* Here 'origin' is the actual data, and '&origin' is the pointer */
  origin = swap16(origin);

  /* Determine the maxmimum number of elements to load from the file */
  uint16_t max_read = UINT16_MAX - origin;
  
  /* Set pointer at memory location for the start the program */
  uint16_t* p = memory + origin;
  /* Read a number of instructions equal to the maximum that our memory array
   * can handle */
  size_t read = fread(p, sizeof(uint16_t), max_read, file);

  /* Swap all 16bit vectors to Little Endian */
  while (read > 0) {
    *p = swap16(*p); /* Swap the current 16bit vector */
    p++; /* Move to the next vector */
    read--; /* Decrement the number of remaining elements to swap */
  }
}

int read_image(const char* image_path) {
  /* Convenience wrapper function, which also reports the operation status */

  FILE* file = fopen(image_path, "rb");
  if (!file)
    return 0; /* No file loaded */

  read_image_file(file);
  fclose(file);
  return 1; /* A file was loaded */
}

uint16_t mem_read(uint16_t address) {
  /* Getter: get a value from the block at a memory address */
  
  if (address == MR_KBSR) {
    if (check_key()) {
      memory[MR_KBSR] = 1 << 15;
      memory[MR_KBDR] = getchar();
    } else {
      memory[MR_KBSR] = 0;
    }
  }

  return memory[address];
}

void mem_write(uint16_t address, uint16_t value) {
  /* Setter: set a memory block at a memory address to a given value */
  memory[address] = value;
}

uint16_t check_key() {
  /* Unix specific for keyboard access */
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(STDIN_FILENO, &readfds);

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;
  return select(1, &readfds, NULL, NULL, &timeout) != 0;
}

void disable_input_buffering() {
  /* Unix specific for terminal input */
  tcgetattr(STDIN_FILENO, &original_tio);
  struct termios new_tio = original_tio;
  new_tio.c_lflag &= ~ICANON & ~ECHO;
  tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering() {
  /* Unix specific for terminal input */
  tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

void handle_interrupt(int signal) {
  /* Unix specific for terminal input */
  restore_input_buffering();
  printf("\n");
  exit(-2);
}

#endif /* LC3_FUNCS_14234234 */

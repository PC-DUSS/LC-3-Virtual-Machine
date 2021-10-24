/*
 * Elementary virtual machine to understand instruction sets, parsing
 * and gain a better general understanding of computers.
 *
 * AUTHOR: Pierre-Charles Dussault
 * SINCE: 2021/10/21
 * */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/*
#include <string.h>
#include <signal.h>

#include <unistd.h>
#include <fcntl.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/mman.h>
*/

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
void handle_TRAP(uint16_t instruction, uint16_t regs[]);
void handle_BAD_OPCODE(uint16_t instruction, uint16_t regs[]);

/* ----- VM Setup ----- */

/* Memory array size, and individual block size */
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
  R_R7, /* Links back to instruction after a subroutine call */
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
  FL_P = 1 << 0, /* P Positive */
  FL_Z = 1 << 1, /* Z Zero */
  FL_N = 1 << 2 /* N Negative */
};
/* end of VM Setup */


/* ----- Main Program ----- */

int main(int argc, const char* argv[]) {
  /* Input validation */
  if (argc < 2) {
    printf("Correct usage:\nlc3 [<image-file1>, <image-file2>, ...]\n");
    exit(2);
  }

  for (int index = 0; index < argc; index++) {
    if (!read_image(argv[index])) {
      printf("failed to load image: %s\n", argv[index]);
      exit(1);
    }
  } /* end of Input validation */

  /* Set a constant for the address where the program starts, and store it in
   * the Program Counter register */
  enum {PC_START = 0x3000};
  regs[R_PC] = PC_START;

  int running = 1;
  while (running) {
    /* Read the current instruction, and determine what operation to do */
    uint16_t instruction = mem_read(regs[R_PC]++);
    uint16_t operation = instruction >> 12;
    switch (operation) {
      case OP_ADD:
        handle_ADD(instruction, regs);
        break;
      case OP_AND:
        handle_AND(instruction, regs);
        break;
      case OP_NOT:
        handle_NOT(instruction, regs);
        break;
      case OP_BR:
        handle_BR(instruction, regs);
        break;
      case OP_JMP:
        handle_JMP(instruction, regs);
        break;
      case OP_JSR:
        handle_JSR(instruction, regs);
        break;
      case OP_LD:
        handle_LD(instruction, regs);
        break;
      case OP_LDI:
        handle_LDI(instruction, regs);
        break;
      case OP_LDR:
        handle_LDR(instruction, regs);
        break;
      case OP_LEA:
        handle_LEA(instruction, regs);
        break;
      case OP_ST:
        handle_ST(instruction, regs);
        break;
      case OP_STI:
        handle_STI(instruction, regs);
        break;
      case OP_STR:
        handle_STR(instruction, regs);
        break;
      case OP_TRAP:
        handle_TRAP(instruction, regs);
        break;
      case OP_RES:
        /* Unused */
      case OP_RTI:
        /* Unused */
      default:
        handle_BAD_OPCODE(instruction, regs);
        break;
    }
  }
} /* end of Main Program */



/* ----- Helper functions ----- */

uint16_t sign_extend(uint16_t x, int bit_count) {
  /* Get the last bit from the binary_num, and pass it through bitwise
   * AND with a bit of 1. 
   *
   * Here, the bit_count parameter is the number of bits present in the
   * passed value x
   * */

  if ((x >> (bit_count - 1)) & 0b1) {
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
   * passed the bit [11] using an AND with a 111 bitmask, since bits [9 : 11]
   * will be the first 3 bits */
  uint16_t dr_index = (instruction >> 9) & 0b111;
  /* First operand (SR1) */
  uint16_t sr1_index = (instruction >> 6) & 0b111;
  /* Whether we are in immediate mode */
  uint16_t imm_flag = (instruction >> 5) & 0b1;

  if (imm_flag) {
    uint16_t imm5 = sign_extend(instruction & 0b11111, 5);
    /* Add the value stored at the address of sr1_index (which can be
     * +/- 2^15) with the immediately value (which must be in range
     * [-16, 15]) */
    regs[dr_index] = regs[sr1_index] + imm5;
  } else {
    uint16_t sr2_index = instruction & 0b111;
    /* This sum must be representable by 16bits or bits will fall off. */
    regs[dr_index] = regs[sr1_index] + regs[sr2_index];
  }

  /* Every time an instruction modifies a register, we need to update the
   * condition flag */
  update_flags(dr_index);
}

void handle_AND(uint16_t instruction, uint16_t regs[]) {
  /* Gather instruction and filter with bitmask */
  uint16_t dr_index = instruction >> 9 & 0b111;
  uint16_t sr1_index = instruction >> 6 & 0b111;

  if (instruction >> 5 & 0b1) {
    /* If bit [5] == 1, DR = SR1 AND imm5 */
    uint16_t imm5 = sign_extend(instruction & 0x1F, 5);
    regs[dr_index] = mem_read(sr1_index) & imm5;

  } else {
    /* If bit [5] == 0, DR = SR1 AND SR2 */
    uint16_t sr2_index = instruction & 0b111;
    regs[dr_index] = mem_read(sr1_index) & mem_read(sr2_index);
  }

  /* As always, update condition flag */
  update_flags(dr_index);
}

void handle_NOT(uint16_t instruction, uint16_t regs[]) {
  uint16_t dr_index = instruction >> 9 & 0b111;
  uint16_t sr_index = instruction >> 6 & 0b111;
  regs[dr_index] = ~mem_read(SR);
  update_flags(dr_index);
}

void handle_BR(uint16_t instruction, uint16_t regs[]) {
  /* BR stands for Branch */

  /* Local flag to determine if the branch should be executed. Turn this to 1
   * if any tested condition is met */
  uint16_t local_flag = 0;

  /* Local flag to determine if a condition should be checked */
  uint16_t n_flag = instruction >> 11 & 0b1;
  uint16_t z_flag = instruction >> 10 & 0b1;
  uint16_t p_flag = instruction >> 9 & 0b1;

  /* If no condition is checked, set local_flag to 1 unconditionally */
  if (!(n_flag || z_flag || p_flag))
    local_flag = 1;

  /* If N's bit [1] is set to 1, test for N */
  if (n_flag)
    if (regs[R_COND] == FL_N)
      local_flag = 1;
  
  /* If Z's bit [10] is set to 1, test for Z */
  if (z_flag)
    if (regs[R_COND] == FL_Z)
      local_flag = 1;
  
  /* If P's bit [10] is set to 1, test for P */
  if (p_flag)
    if (regs[R_COND] == FL_P)
      local_flag = 1;
  
  if (local_flag) {
    regs[R_PC] += sign_extend(instruction & 0x1FF, 9); /* bitmask 111111111 */
    /*  TODO: Here, do we update_flags with the modified register R_PC? Or do we
     *  set it manually depending on the boolean outcome of the Branch
     *  instruction */
    regs[R_COND] = FL_P;
  } else {
    regs[R_COND] = FL_N;
  }
}

void handle_JMP(uint16_t instruction, uint16_t regs[]) {
  uint16_t baseR_index = instruction >> 6 & 0b111;
  regs[R_PC] = mem_read(regs[baseR_index]);
  
  /* TODO: Here, I am unsure if jumping back to instruction after the end of the
   * subroutine counts as a positive flag or negative flag
   * */
}

void handle_JSR(uint16_t instruction, uint16_t regs[]) {
  /* Jump to subsroutine */
  regs[R_R7] = regs[R_PC];
  if (instruction >> 11 & 0b1) {
    regs[R_PC] += sign_extend(instruction & 0x7FF, 11);
  } else {
    regs[R_PC] = mem_read(instruction >> 6 & 0x7);
  }
  
  /* TODO: Conditional flags, how? */
}

void handle_LD(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_LDI(uint16_t instruction, uint16_t regs[]) {
  /* Get the DR (bits [9:11]) by moving starting bit to bit 9, and then use
   * bitmask to filter out exceeding bits */
  uint16_t dr_index = instruction >> 9 & 0b111;
  /* Bitmask to only keep bits [0:8] (the contents of PCoffset9) */
  uint16_t relevant_instr = instruction & 0x1FF;
  /* Gather first memory address from PCoffset9 by sign-extending it*/
  uint16_t pc_offset = sign_extend(relevant_instr, 9);
  /* Gather the second reference (to be stored in the DR) at the memory address
   * obtained by adding the one we just found with the address of the PC. 
   * I am making all the steps clear here, but you could refactor in less lines
   * for better efficiency

  uint16_t sign_extended_reference = regs[R_PC] + pc_offset
  uint16_t second_reference = mem_read(sign_extended_reference);
  uint16_t real_reference_to_data = mem_read(second_reference);
  regs[dr_index] = real_reference_to_data;
  
   * Can be refactored to:
   */
  regs[dr_index] = mem_read(mem_read(regs[R_PC] + pc_offset));

  /* Finally, don't forget to update flags after each instruction! */
  update_flags(dr_index);
}

void handle_LDR(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_LEA(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_ST(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_STI(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_STR(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_TRAP(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_BAD_OPCODE(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

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
#include <string.h>
#include <signal.h>
/* unix specific */
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
void handle_TRAP(uint16_t instruction, uint16_t regs[]);


/* ----- VM Setup ----- */

/* Memory array size, and individual block size */
uint16_t memory[UINT16_MAX];

/* Registers */
enum {
  R_R0 = 0,
  R_R1,
  R_R2,
  R_R3,
  R_R4,
  R_R5,
  R_R6, /* Stack pointer */
  R_R7,
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
  OP_JSR, /* JSR Jump register */
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
  FL_POS = 1 << 0, /* P Positive */
  FL_ZRO = 1 << 1, /* Z Zero */
  FL_NEG = 1 << 2 /* N Negative */
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

    /* See 'Helper functions' for the behaviour of each handling function */
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
        {BAD_OPCODE, 7}
        break;
    }
  }
} /* end of Main Program */



/* ----- Helper functions ----- */

uint16_t sign_extend(uint16_t x, int bit_count) {
  /* Get the right-most bit from the binary_num, and pass it through bitwise
   * AND with a bit of 1. This gives 1 if the right-most bit is equal to 1
   * (negative), or 0 if the right-most bit is equal to 0 (positive) */

  if ((x >> (bit_count - 1)) & 1) {
  /* The if condition is triggered if the right-most bit indicates a negative
   * number */

    /* This does bitwise OR with x and a 16-bit number with the (bit_count)
     * left-most bits as zeros */
    x |= (0xFFFF << bit_count);
  }

  return x;
}

void update_flags(uint16_t r) {
  if (regs[r] == 0) {
    regs[R_COND] = FL_ZRO;
  } else if (regs[r] >> 15) { /* 1 in left-most bit means a negative number */
    regs[R_COND] = FL_NEG;
  } else {
    regs[R_COND] = FL_POS;
  }
}

void handle_ADD(uint16_t instruction, uint16_t regs[]) {
  /* Destination register (DR) */
  uint16_t r0 = (instruction >> 9) & 0b111; /* this removes left-trailing bits */
  /* First operand (SR1) */
  uint16_t r1 = (instruction >> 6) & 0b111;
  /* Whether we are in immediate mode */
  uint16_t imm_flag = (instruction >> 5) & 0b1;

  if (imm_flag) {
    uint16_t imm5 = sign_extend(instruction & 0b11111, 5);
    /* Add the value stored at the address of r1 (which can be
     * +/- 2^15) with the immediately value (which must be in range
     * [-16, 15]) */
    regs[r0] = regs[r1] + imm5;
  } else {
    uint16_t r2 = instruction & 0b111;
    /* Remember this total must be representable by 16bits or bits will fall off. */
    regs[r0] = regs[r1] + regs[r2];
  }

  update_flags(r0);
}

void handle_AND(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_NOT(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_BR(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_JMP(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_JSR(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_LD(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
}

void handle_LDI(uint16_t instruction, uint16_t regs[]) {
  /* TODO */
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


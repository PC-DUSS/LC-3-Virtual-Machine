/*
 * Elementary virtual machine to understand instruction sets, parsing
 * and gain a better general understanding of computers.
 * Made alongside the exercise created by Justin Meiners and Ryan Pendleton,
 * 'Write Your Own Virtual Machine'
 *
 * AUTHOR: Pierre-Charles Dussault
 * SINCE: 2021/10/21
 * */

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
/* Custom headers */
#include "lc3_funcs.h"


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

  /* Disable keyboard input during the running of the program */
  signal(SIGINT, handle_interrupt); /* handle_interrupt function as argument */
  disable_input_buffering();

  /* Set a constant for the address where the program starts, and store it in
   * the Program Counter register */
  enum {PC_START = 0x3000};
  regs[R_PC] = PC_START;

  int state = 1;
  int *running = &state;

  while (*running) {
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
        handle_TRAP(instruction, regs, running);
        break;
      case OP_RES:
        handle_RES(instruction, regs);
      case OP_RTI:
        handle_RTI(instruction, regs);
      default:
        handle_BAD_OPCODE(instruction, regs);
        break;
    }
  }

  /* Restore keyboard input at end of the program */
  restore_input_buffering();
} /* end of Main Program */

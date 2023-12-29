#pragma once
#include <stdint.h>

// memory operations funct3
#define MEM_BYTE 0x0
#define MEM_HALFWORD 0x1
#define MEM_WORD 0x2
#define MEM_DWORD 0x3
#define MEM_UBYTE 0x4
#define MEM_UHALFWORD 0x5
#define MEM_UWORD 0x6

// memory operation: match PAGEFAULT interrupt
#define MEM_INSTR 0 
#define MEM_READ 1
#define MEM_WRITE 3


// usable memory size: 256M
#define MEMSIZE 256*1024*1024 

extern uint64_t pc;
extern uint64_t no_cycles;
extern uint8_t mem[];   // main memory

extern int init_cpu(uint64_t start_pc);
extern int pa_mem_interface(uint32_t mem_mode, unsigned int addr, int size, unsigned int* data);


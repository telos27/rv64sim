#pragma once

// config options
// CONFIG_RV64, CONFIG_M, CONFIG_A, CONFIG_C

#include <stdint.h>

#ifdef CONFIG_RV64
#define XLEN 64
typedef uint64_t reg_type;			// unsigned register-sized integer
typedef int64_t signed_reg_type;	// signed register-sized integer
#define MEM_REG_SIZE MEM_DWORD		// size for memory access
#define SIGNED_REG_MIN INT64_MIN
#define SHIFT_MASK 0x3f
#define SHIFT_REST_MASK 0x7e	// sub7 with one bit consumed by shift amount
#define INTR_NONE 0xffffffffffffffff
#define CSR_SATP_MODE (0xf<<60)
#define CSR_SATP_PPN 0xfffffffffff	// bits 43:0
#else
#define XLEN 32
typedef uint32_t reg_type;
typedef int32_t signed_reg_type;
#define SIGNED_REG_MIN INT32_MIN
#define MEM_REG_SIZE MEM_WORD
#define SHIFT_MASK 0x1f
#define SHIFT_REST_MASK 0x7f
#define INTR_NONE 0xffffffff
#define CSR_SATP_MODE 0x80000000
#define CSR_SATP_PPN 0x3fffff
#endif


// privilege levles
#define MODE_M 0x3
#define MODE_S 0x1
#define MODE_U 0x0

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


// usable memory size: 128M
#define MEMSIZE 128*1024*1024 

extern reg_type pc;
extern unsigned int mode;
extern reg_type no_cycles;
extern uint8_t mem[];   // main memory

extern int init_cpu(reg_type start_pc);
extern int pa_mem_interface(int mem_mode, reg_type addr, int size, reg_type* data , reg_type* interrupt);
extern reg_type read_reg(int reg_no);
extern int write_reg(int reg_no, reg_type data);
extern reg_type read_CSR(int CSR_no);
extern reg_type write_CSR(int CSR_no, reg_type value);
extern int execute_code();


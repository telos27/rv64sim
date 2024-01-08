#include <stdint.h>
#include <inttypes.h>
#include <assert.h>
#include <stdio.h>

#include "cpu.h"
#include "soc.h"


// instruction decoding： bitfields of an instruction; could use an array
#define OPCODE_MASK 0x7f    // bits [6:0]
#define OPCODE_SHIFT 0
#define FUNCT7_MASK 0xfe000000  // bits [31:25]
#define FUNCT7_SHIFT 25
#define FUNCT3_MASK  0x7000   // bits [14:12]
#define FUNCT3_SHIFT 12
#define RD_MASK 0xf80       // bits [11:7]
#define RD_SHIFT 7
#define RS1_MASK 0xf8000    // bits [19:15]
#define RS1_SHIFT 15
#define RS2_MASK 0x1f00000   // bits [24:20]
#define RS2_SHIFT 20
#define IMM20_MASK 0xfffff000   // bits [31:12]
#define IMM20_SHIFT 12
#define IMM5_MASK RD_MASK
#define IMM5_SHIFT RD_SHIFT
#define IMM7_MASK FUNCT7_MASK
#define IMM7_SHIFT FUNCT7_SHIFT
#define IMM12_MASK 0xfff00000   // bits [31:20]
#define IMM12_SHIFT 20

// opcodes: RV64IMA_Zicsr_Zifenci, mret, wfi
#define OP_ADD    0b0110011
#define OP_ADDW   0b0111011
#define OP_ADDI   0b0010011
#define OP_ADDIW  0b0011011
#define OP_LB     0b0000011
#define OP_SB     0b0100011
#define OP_BEQ    0b1100011
#define OP_JAL    0b1101111
#define OP_JALR   0b1100111
#define OP_LUI    0b0110111
#define OP_AUIPC  0b0010111
#define OP_ECALL  0b1110011
#define OP_FENCEI 0b0001111
#define OP_A      0b0101111

// R/I instruction funct3
#define ALU_ADD 0x0     // SUB funct7=0x20
#define ALU_XOR 0x4
#define ALU_OR 0x6
#define ALU_AND 0x7
#define ALU_SLL 0x1
#define ALU_SRL 0x5     // SRA/SRAI funct7=0x20
#define ALU_SLT 0x2
#define ALU_SLTU 0x3

// RV-M funct3
#define MUL 0x0
#define MULH 0x1
#define MULHSU 0x2
#define MULHU 0x3
#define DIV 0x4
#define DIVU 0x5
#define REM 0x6
#define REMU 0x7

// funct7: SUB & SRA
#define NORMAL 0x0
#define SUB 0x20
#define SRA 0x20

// branch funct3
#define BRANCH_EQ 0x0
#define BRANCH_NE 0x1
#define BRANCH_LT 0x4
#define BRANCH_GE 0x5
#define BRANCH_LTU 0x6
#define BRANCH_GEU 0x7

// ecall funct3
#define SYSTEM_ECALL 0x0
#define SYSTEM_CSRRW 0x1
#define SYSTEM_CSRRS 0x2
#define SYSTEM_CSRRC 0x3
#define SYSTEM_CSRRWI 0x5
#define SYSTEM_CSRRSI 0x6
#define SYSTEM_CSRRCI 0x7

// SYSTEM_ECALL imm12
#define ECALL_ECALL 0x0
#define ECALL_EBREAK 0x1
#define ECALL_MRET 0x302
#define ECALL_SRET 0x102
#define ECALL_WFI 0x105
#define ECALL_SFENCEVMA 0x120

// AMO funct7[6:4]
#define AMO_ADD 0x0
#define AMO_XOR 0x1
#define AMO_AND 0x3
#define AMO_OR 0x2
#define AMO_MIN 0x4
#define AMO_MAX 0x5
#define AMO_MINU 0x6
#define AMO_MAXU 0x7

// AMO funct7[3:2] for ADD
#define AMO_ADD_ADD 0x0
#define AMO_ADD_SWAP 0x1
#define AMO_ADD_LR 0x2
#define AMO_ADD_SC 0x3

// AMO funct3
#define AMO_D 0x3
#define AMO_W 0x2


// Sv39 virtual memory
#define PTE_LEVELS 3
#define VPN_SEG1 0x7fc0000	// bits [26:18] of vpn
#define VPN_SEG2 0x3fe00	// bits[17:9]
#define VPN_SEG3 0x1ff // bits[8:0]
#define PTE_V 0x1
#define PTE_R 0x2
#define PTE_W 0x4
#define PTE_X 0x8
#define PTE_U 0x10
#define PTE_G 0x20
#define PTE_A 0x40
#define PTE_D 0x80
#define PTE_PPN 0x3ffffffffffc00


#define BOOL int8_t
#define TRUE 1
#define FALSE 0 
#define REGISTER uint64_t   // enough to hold the content of a single register

// external memory
uint8_t mem[MEMSIZE];   // main memory

// TODO: leftover from 32-bit impl, clean up
// MEMIO range: currently CLINT & UART, and debug syscall
#define MEMIO_START 0x10000000
#define MEMIO_END 0x12000000    // exclusive

#define NO_CSRS 4096

// CPU internal state
static REGISTER regs[32];
static uint64_t CSRs[NO_CSRS];  // explicit init?
uint64_t pc ;       // 64-bit PC
unsigned int mode;      // privilege mode: currently M only
static uint64_t reservation;   // address for lr/sc ; top 61 bits
unsigned int wfi = 0;    // WFI flag 
uint64_t no_cycles; // execution cycles; currently always 1 cycle/instruction
static uint64_t interrupt;  // interrupt type

static unsigned int trace = 0;  // trace every instruction


// read register
// can optimize by always 0 in x0
REGISTER read_reg(int reg_no)
{
    assert(reg_no < 32);
    return (reg_no == 0) ? 0 : regs[reg_no];
}

int write_reg(int reg_no, REGISTER data)
{
    assert(reg_no < 32);
   if (reg_no != 0) {
      regs[reg_no] = data;
   }
   return reg_no;  // not used yet
}


// TODO: implement CSR R/W rules according to spec
uint64_t read_CSR(int CSR_no)
{
    assert(CSR_no < 4096);
    if (CSR_no == CSR_MISA)
        return 0x40401101;  // TODO
    else if (CSR_no == CSR_MVENDORID)
        return 0xff0fff0f;  // TODO
    else if (CSR_no == CSR_CYCLE)
        return no_cycles;
    else if (CSR_no & 0xc00) {
        //printf("read_CSR: 0x%x\n", CSR_no);
    }
    return CSRs[CSR_no];
}

uint64_t write_CSR(int CSR_no, uint64_t value)
{
    assert(CSR_no < 4096);
    CSRs[CSR_no] = value;
    if (CSR_no == CSR_MTVEC) {
 //       printf("write_CSR: mtvec = %llx, pc=%llx\n", value, pc);
    }
    if (CSR_no == CSR_STVEC) {
 //       printf("write_CSR: stvec = %llx, pc=%llx\n", value, pc);
    }
    if (CSR_no == CSR_SATP) {
 //       printf("write_CSR: satp = %llx, pc=%llx\n", value, pc);
    }
    if (CSR_no == CSR_SIP) {
//        printf("write_CSR: sip = %llx, pc=%lx\n", value, pc);
    }
    return 1;
}


// VM handling

typedef uint64_t PTE;

// translate 27-bit vpn va to 44-bit ppn
// mem_access_mode: instruction read, data read, data write
// returns 0 if there is any kind of fault, which is stored in interrupt
uint64_t vpn2ppn(uint64_t vpn, uint64_t mem_access_mode, uint64_t* interrupt)
{
    uint64_t ppn = read_CSR(CSR_SATP) & CSR_SATP_PPN;
    uint64_t vpn_segment[] = { vpn & VPN_SEG1 , vpn & VPN_SEG2 , vpn & VPN_SEG3};	// three segements corresponding to three-level page table

    // loop three levels for Sv39
    for (int i = 0; i < PTE_LEVELS; i++)
    {
        PTE pte;

        pa_mem_interface(MEM_READ, (ppn << 12) | ((vpn_segment[i]>>(9*(PTE_LEVELS-i-1)))<<3), MEM_DWORD, &pte, interrupt);	// physical address, no translation
        if (*interrupt != 0xffffffffffffffff) return 0;

        if ((pte & PTE_V) == 0 || ((pte & PTE_W) && !(pte & PTE_R))) {
            *interrupt = INT_INSTR_PAGEFAULT + mem_access_mode;	// invalid PTE or RW reserved combination
            return 0;
        }

        if (pte & PTE_R || pte & PTE_X) { // leaf pte
            uint64_t result = (pte & PTE_PPN)>>10 ;
            // TODO: check U & SUM & MXR

            // check superpage alignment: the last segments of the PPN should all be 0
            if (((i == 0) && (((result & VPN_SEG2) != 0) || ((result & VPN_SEG3) != 0))) || 
                ((i==1) && ((result & VPN_SEG3) !=0))) {
                *interrupt = INT_INSTR_PAGEFAULT + mem_access_mode;
                return 0;
            }

            /* TODO: no need to check?  xv6 doesn't use them, how about Linux?
            // A or D bit doesn't match
            if ((pte & PTE_A) == 0 || (mem_access_mode == MEM_WRITE && ((pte & PTE_D) == 0))) {
                //or ACCESS if pte PMP/PMA
                *interrupt = INT_INSTR_PAGEFAULT + mem_access_mode;
                return 0;
            }
            */
            // TODO: handle A&D update; set A, set D if write; should not update for xv6?
            if (i == 0) {	// gigapage: use segment1 from VPN
                result |= vpn_segment[1];
            }
            if (i<=1) {     // megapage and gigapage: use segmenet2 from VPN
                result |= vpn_segment[2];
            }
            return result;
        }
        else {
            ppn = (pte & PTE_PPN)>>10;		// next-level PTE; should be good enough?
        }
    }
    // TODO: how to avoid two addr translations for a single AMO? should be correct even if we don't do anything? but should optimize
    *interrupt = mem_access_mode + INT_INSTR_PAGEFAULT;	//no leaf pte
    return 0;
}


// physical address memory operation, no sign extension done here
// memory address start at INITIAL_PC, no memory storage below that address
// can improve perf by handling multiple bytes at once
int pa_mem_interface(uint64_t mem_mode, uint64_t addr, int size, uint64_t* data, uint64_t *interrupt)
{
    assert(addr >= INITIAL_PC);
    // TODO: PMP & PMA checks 
    addr -= INITIAL_PC;
    assert(addr < MEMSIZE);
    assert(size == MEM_BYTE || size == MEM_HALFWORD || size == MEM_WORD || size==MEM_DWORD ||
            size==MEM_UBYTE || size==MEM_UHALFWORD || size==MEM_UWORD);
    if (mem_mode == MEM_WRITE) {
        switch (size) {
        case MEM_BYTE:
            mem[addr] = (*data) & 0xff;
            break;
        case MEM_HALFWORD:
            mem[addr] = (*data) & 0xff;
            mem[addr + 1] = ((*data) & 0xff00) >> 8;
            break;
        case MEM_WORD:
            mem[addr] = (*data) & 0xff;
            mem[addr + 1] = ((*data) & 0xff00) >> 8;
            mem[addr + 2] = ((*data) & 0xff0000) >> 16;
            mem[addr + 3] = ((*data) & 0xff000000) >> 24;
            break;
        case MEM_DWORD:
            mem[addr] = (*data) & 0xff;
            mem[addr + 1] = ((*data) & 0xff00) >> 8;
            mem[addr + 2] = ((*data) & 0xff0000) >> 16;
            mem[addr + 3] = ((*data) & 0xff000000) >> 24;
            mem[addr + 4] = ((*data) & 0xff00000000) >> 32;
            mem[addr + 5] = ((*data) & 0xff0000000000) >> 40;
            mem[addr + 6] = ((*data) & 0xff000000000000) >> 48;
            mem[addr + 7] = ((*data) & 0xff00000000000000) >> 56;
            break;
        default: *interrupt=INT_ILLEGAL_INSTR ; // unspported size
        }

    } else { // both instruction and data read
        switch (size) {
        case MEM_BYTE: case MEM_UBYTE:
            // NOTE: no sign extension
            *data = mem[addr];
            break;
        case MEM_HALFWORD: case MEM_UHALFWORD:
            // NOTE: no sign extension
            *data = mem[addr] | ((uint64_t)mem[addr + 1]) << 8 ;
            break;
        case MEM_WORD: case MEM_UWORD:
            *data = mem[addr] | ((uint64_t)mem[addr + 1]) << 8 |
                ((uint64_t)mem[addr + 2]) << 16 | ((uint64_t)mem[addr + 3]) << 24;
            break;
        case MEM_DWORD:
            *data = mem[addr] | ((uint64_t)mem[addr + 1]) << 8 |
                ((uint64_t)mem[addr + 2]) << 16 | ((uint64_t)mem[addr + 3]) << 24 |
            ((uint64_t)mem[addr + 4]) << 32 | ((uint64_t)mem[addr + 5]) << 40 |
            ((uint64_t)mem[addr + 6]) << 48 | ((uint64_t)mem[addr + 7]) << 56;
            break;

        default:
            *interrupt = INT_ILLEGAL_INSTR ; // unsupported size
        }
    }
    return TRUE;
}


// read/write memory, instruction semantics, which includes sign extension in some cases
// include memory-mapped I/O in specificed address range, which goes through MMU as well
// NOTE: little-endian
// TODO: mstatus.mprv
// TODO: cleaner logic for mem I/O vs real memory
int rw_memory(uint64_t mem_mode, uint64_t addr, int sub3, uint64_t* data)
{
    // address translation here ;
    uint64_t satp = read_CSR(CSR_SATP);

    // TODO: trap for other modes
    if (mode!=MODE_M && ((satp>>60) == CSR_SATP_MODE_SV39)) {
        addr = (vpn2ppn(addr>>12 , mem_mode , &interrupt) << 12) | (addr & 0xfff);  // 56 bit pa
        if (interrupt!=0xffffffffffffffff) return -1;
    }
    if (mem_mode==MEM_WRITE) {
        if (addr >= MEMIO_START && addr < MEMIO_END ||
            addr >= IO_CLINT_START && addr<IO_CLINT_END ||
            addr >= IO_UART_START && addr < IO_UART_END ||
            addr >= IO_PLIC_START && addr < IO_PLIC_END ||
            addr >= IO_VIRTIO_START && addr < IO_VIRTIO_END) {
            return io_write(addr, data);
        }
        else {
            return pa_mem_interface(mem_mode, addr, sub3, data , &interrupt);
        }
    } else {    // both instruction and data read
        uint64_t read_data;
        int result;
        if (addr >= MEMIO_START && addr < MEMIO_END ||
            addr >= IO_CLINT_START && addr < IO_CLINT_END ||
            addr >= IO_UART_START && addr < IO_UART_END ||
            addr >= IO_PLIC_START && addr < IO_PLIC_END ||
            addr >= IO_VIRTIO_START && addr < IO_VIRTIO_END) {
            result = io_read(addr, &read_data) ;  // TODO: what do we do about non-word-sized I/O？
        }
        else {
            result = pa_mem_interface(mem_mode, addr, sub3, &read_data , &interrupt);
            if (interrupt != 0xffffffffffffffff) return -1;
        }
        switch (sub3) {
            case MEM_BYTE: 
                // NOTE: sign extension
                *data = (int64_t)((int8_t)read_data) ; 
                break ;
            case MEM_HALFWORD: 
                // NOTE: sign extension
                *data = (int64_t) ((int16_t)read_data)  ;
                break;
            case MEM_WORD: 
                // NOTE: sign extension
                *data = (int64_t)((int32_t)read_data);
                break;
            case MEM_UBYTE:
            case MEM_UHALFWORD:
            case MEM_UWORD:
            case MEM_DWORD:
                *data = read_data ;
                break ;
            default: 
                interrupt = INT_ILLEGAL_INSTR; // undefined sub3
        }
        return result;

    }
}

static inline void umul64wide(uint64_t a, uint64_t b, uint64_t* hi, uint64_t* lo)
{
    uint64_t a_lo = (uint32_t)a;
    uint64_t a_hi = a >> 32;
    uint64_t b_lo = (uint32_t)b;
    uint64_t b_hi = b >> 32;

    uint64_t p0 = a_lo * b_lo;
    uint64_t p1 = a_lo * b_hi;
    uint64_t p2 = a_hi * b_lo;
    uint64_t p3 = a_hi * b_hi;

    uint32_t cy = (uint32_t)(((p0 >> 32) + (uint32_t)p1 + (uint32_t)p2) >> 32);

    *lo = p0 + (p1 << 32) + (p2 << 32);
    *hi = p3 + (p1 >> 32) + (p2 >> 32) + cy;
}

static inline void mul64wide(int64_t a, int64_t b, int64_t* hi, int64_t* lo)
{
    umul64wide((uint64_t)a, (uint64_t)b, (uint64_t*)hi, (uint64_t*)lo);
    if (a < 0LL) *hi -= b;
    if (b < 0LL) *hi -= a;
}

static inline void mulhsu64wide(int64_t a, uint64_t b, int64_t* hi, int64_t* lo)
{
    umul64wide((uint64_t)a, (uint64_t)b, (uint64_t*)hi, (uint64_t*)lo);
    if (a < 0LL) *hi -= b;
}


// NOTE: shift amount only uses 6-bits
int reg_op(int rd , int rs1 , int rs2 , int sub3 , int sub7)
{
    if ((sub7 & 1) != 1) {
        switch (sub3) {
        case ALU_ADD:
            if (sub7 == NORMAL) {
                write_reg(rd, read_reg(rs1) + read_reg(rs2));
            }
            else if (sub7 == SUB) {
                write_reg(rd, read_reg(rs1) - read_reg(rs2));
            }
            else {
                interrupt = INT_ILLEGAL_INSTR;  // nonexistent sub7
            }
            break;
        case ALU_XOR:
            write_reg(rd, read_reg(rs1) ^ read_reg(rs2));
            break;
        case ALU_OR:
            write_reg(rd, read_reg(rs1) | read_reg(rs2));
            break;
        case ALU_AND:
            write_reg(rd, read_reg(rs1) & read_reg(rs2));
            break;
        case ALU_SLL:
            write_reg(rd, read_reg(rs1) << (read_reg(rs2) & 0x3f));
            break;
        case ALU_SRL:
            if ((sub7&0xfe) == NORMAL) {    // only check 6 bits
                write_reg(rd, read_reg(rs1) >> (read_reg(rs2) & 0x3f));
            } else if ((sub7&0xfe) == SRA) {    // check 6 bits
                // signed right shift
                write_reg(rd, ((int64_t)read_reg(rs1)) >> (read_reg(rs2) & 0x3f));
            }
            else {
                interrupt = INT_ILLEGAL_INSTR;  // nonexistent sub7
            }
            break;
        case ALU_SLT:
            write_reg(rd, ((int64_t)read_reg(rs1) < (int64_t)read_reg(rs2)) ? 1 : 0);
            break;
        case ALU_SLTU:
            write_reg(rd, (read_reg(rs1) < read_reg(rs2)) ? 1 : 0);
            break ;
        default: interrupt = INT_ILLEGAL_INSTR ; // unknown sub3
        }
    } else { // MULDIV
        // NOTE: signedness, special cases for division/remainder of certain numbers
        uint64_t n1 = read_reg(rs1);
        uint64_t n2 = read_reg(rs2);
        uint64_t hi=0 , lo=0, result = 0;
        switch (sub3) {
        case MUL: result = ((int64_t)n1) * ((int64_t)n2); break;
/* copy zap8600/rv64i-emu: the idea is to trap to emulate?
        case MULH: result = (((int64_t)(int32_t)n1) * ((int64_t)(int32_t)n2)) >> 32; break;
        case MULHSU: result = (((int64_t)(int32_t)n1) * ((uint64_t)n2)) >> 32; break;
        */
        case MULHU: umul64wide(n1, n2, &result, &lo); break;     
        case DIV: if (n2 == 0) result = -1; else
            result = ((((int64_t)n1) == INT64_MIN) && ((int64_t)n2) == -1) ? n1 : (((int64_t)n1) / (int64_t)n2); break;
        case DIVU: result = (n2 == 0) ? -1 : (n1 / n2); break;
        case REM: if (n2 == 0) result = n1; else
            result = ((((int64_t)n1) == INT64_MIN) && (((int64_t)n2) == -1)) ? 0 : 
                (uint64_t)((int64_t)n1 % (int64_t)n2); 
            break;
        case REMU: result = (n2 == 0) ? n1 : (n1 % n2); break;

        default: interrupt = INT_ILLEGAL_INSTR; break;  // unknown sub3
        }
        write_reg(rd, result);
    }
    return TRUE;
}



// NOTE: shift amount only uses 5-bits
// operation on 32 bits only
int reg32_op(int rd, int rs1, int rs2, int sub3, int sub7)
{
    if ((sub7 & 1) != 1) {
        switch (sub3) {
        case ALU_ADD:
            if (sub7 == NORMAL) {
                write_reg(rd, (int64_t)((uint32_t)read_reg(rs1) + (uint32_t)read_reg(rs2)));
            }
            else if (sub7 == SUB) {
                write_reg(rd, (int64_t)((uint32_t)read_reg(rs1) - (uint32_t)read_reg(rs2)));
            }
            else {
                interrupt = INT_ILLEGAL_INSTR;  // nonexistent sub7
            }
            break;
        case ALU_SLL:
            write_reg(rd, (int64_t)((uint32_t)read_reg(rs1) << (read_reg(rs2) & 0x1f)));
            break;
        case ALU_SRL:
            if (sub7 == NORMAL) {
                write_reg(rd, (int64_t)((uint32_t)read_reg(rs1) >> (read_reg(rs2) & 0x1f)));
            }
            else if (sub7 == SRA) {
                // signed right shift
                write_reg(rd, (int64_t)(((int32_t)read_reg(rs1)) >> (read_reg(rs2) & 0x1f)));
            }
            else {
                interrupt = INT_ILLEGAL_INSTR;  // nonexistent sub7
            }
            break;
        default: interrupt = INT_ILLEGAL_INSTR; // unknown sub3
        }
    }
    else { // MULDIV
        // NOTE: signedness, special cases for division/remainder of certain numbers
        uint32_t n1 = (uint32_t) read_reg(rs1);
        uint32_t n2 = (uint32_t) read_reg(rs2);
        uint32_t result = 0;
        switch (sub3) {
        case MUL: result = ((int32_t)n1) * ((int32_t)n2); break;
        case DIV: if (n2 == 0) result = -1; else
            result = ((((int32_t)n1) == INT32_MIN) && ((int32_t)n2) == -1) ? n1 : (((int32_t)n1) / (int32_t)n2); break;
        case DIVU: result = (n2 == 0) ? 0xffffffff : (n1 / n2); break;
        case REM: if (n2 == 0) result = n1; else
            result = ((((int32_t)n1) == INT32_MIN) && (((int32_t)n2) == -1)) ? 0 :
            (uint32_t)((int32_t)n1 % (int32_t)n2);
            break;
        case REMU: result = (n2 == 0) ? n1 : (n1 % n2); break;
        default: interrupt = INT_ILLEGAL_INSTR; break;  // unknown sub3
        }
        write_reg(rd, (int64_t)(int32_t)result);
    }
    return TRUE;
}


// NOTE: imm is already sign-extended; shift amount only uses 5 bits
int imm_op(int rd , int rs1 , int sub3 , int sub7 , uint64_t imm)
{
    switch (sub3) {
    case ALU_ADD: write_reg(rd, read_reg(rs1) + (int32_t) imm); break;
    case ALU_XOR: write_reg(rd, read_reg(rs1) ^ imm); break;
    case ALU_OR: write_reg(rd, read_reg(rs1) | imm); break;
    case ALU_AND: write_reg(rd, read_reg(rs1) & imm); break;
    case ALU_SLL: write_reg(rd, read_reg(rs1) << (imm & 0x3f)); break;
    case ALU_SRL:
        if ((sub7 & 0xfe) == NORMAL) {  // check 6 bits
            write_reg(rd, read_reg(rs1) >> (imm & 0x3f));
        }
        else if ((sub7&0xfe) == SRA) {  // check 6 bits
            // signed right shift
            write_reg(rd, ((int64_t)read_reg(rs1)) >> (imm & 0x3f));
        }
        else {
            interrupt = INT_ILLEGAL_INSTR; // nonexistent sub7
        }
        break;
    case ALU_SLT:
        write_reg(rd, ((int64_t)read_reg(rs1) < (int64_t)imm) ? 1 : 0);
        break;
    case ALU_SLTU:
        write_reg(rd, (read_reg(rs1) < imm) ? 1 : 0);
        break;
    default: interrupt = INT_ILLEGAL_INSTR; // nonexistent sub3
    }
    return TRUE;
}


// NOTE: imm is already sign-extended; shift amount only uses 5 bits
// only uses 32 bits from rs1, imm is 32-bit
int imm32_op(int rd, int rs1, int sub3, int sub7, unsigned int imm)
{
    switch (sub3) {
    case ALU_ADD: write_reg(rd, (int64_t)((int32_t)read_reg(rs1) + (int32_t)imm)); break;
    case ALU_SLL: write_reg(rd, (int64_t)((uint32_t)read_reg(rs1) << (imm & 0x1f))); break;
    case ALU_SRL:
        if (sub7 == NORMAL) {
            write_reg(rd, (int64_t)((uint32_t)read_reg(rs1) >> (imm & 0x1f)));
        }
        else if (sub7 == SRA) {
            // signed right shift
            write_reg(rd, (int64_t)((int32_t)read_reg(rs1)) >> (imm & 0x1f));
        }
        else {
            interrupt = INT_ILLEGAL_INSTR; // nonexistent sub7
        }
        break;
    default: interrupt = INT_ILLEGAL_INSTR; // nonexistent sub3
    }
    return TRUE;
}


// return a sign-extended version of a number with no_bits
uint64_t sign_extend(uint64_t n , int no_bits)
{
    // TODO: is this portable?
    return (((int64_t)n) << (64 - no_bits)) >> (64 - no_bits);
}


// return next PC, -1 if we don't branch
int64_t branch_op(int rs1 , int rs2 , int sub3 , unsigned int imm5 , unsigned int imm7)
{
    BOOL do_branch = FALSE ;

    switch (sub3) {
    case BRANCH_EQ: do_branch = read_reg(rs1) == read_reg(rs2); break ;
    case BRANCH_NE: do_branch = read_reg(rs1) != read_reg(rs2); break;
    case BRANCH_LT: do_branch = (int64_t)read_reg(rs1) < (int64_t) read_reg(rs2); break ;
    case BRANCH_GE: do_branch = (int64_t)read_reg(rs1) >= (int64_t) read_reg(rs2); break;
    case BRANCH_LTU: do_branch = read_reg(rs1) < read_reg(rs2); break;
    case BRANCH_GEU: do_branch = read_reg(rs1) >= read_reg(rs2); break;
    default: interrupt = INT_ILLEGAL_INSTR; // error
    }

    if (do_branch) {
        // NOTE: order of bits, added a zero bit at the end; signed offset
        unsigned int unsigned_offset = ((imm7 & 0x40) << 6) | ((imm5 & 0x1) << 11) |
            ((imm7 & 0x3f) << 5) | (imm5 & 0x1e) ;  
        uint64_t offset = sign_extend(unsigned_offset , 13);
        uint64_t ret =  pc+offset ;
    //    printf("B: pc=%x , sub3=%x , rs1=%x , rs2=%x , imm5=%x , imm7=%x , offset=%x , ret=%x\n", pc , sub3, rs1, rs2, imm5,
     //       imm7, offset, ret);
        return ret;
    } else {
        return -1 ;
    }
}

// JAL opcode: returns next PC
uint64_t jal_op(int rd, unsigned int imm)
{
    write_reg(rd, pc + 4);
    // NOTE: bit position, add 0 bit , sign extend
    uint64_t ret = pc + sign_extend(((imm & 0x80000) | ((imm & 0xff) << 11) | 
        ((imm & 0x100) << 2) | ((imm & 0x7fe00) >> 9))<<1, 21);
   // printf("JAL: pc=%x , rd=%x , imm=%x , next=%x\n", pc , rd, imm, ret);
    return ret;
}


// JALR opcode: returns next PC
uint64_t jalr_op(int rd, int rs1, unsigned int imm12)
{
    // NOTE: write register afterwards, rd could be same as rs1
    uint64_t saved_pc = pc + 4;
    // NOTE: sign extend , zero LSB
    uint64_t ret =  (read_reg(rs1) + sign_extend(imm12, 12)) & 0xfffffffffffffffe;
    //printf("JALR: pc=%x , rd=%x , rs1=%x , imm12=%x , next=%x\n", pc , rd, rs1, imm12 ,  ret);
    write_reg(rd, saved_pc);
    return ret;
}

// AUIPC opcode
// NOTE: sign extend to 32 bits
int auipc_op(int rd, unsigned int imm20)
{
    write_reg(rd, pc + sign_extend(imm20 << 12 , 32));
    return 0;
}

// LUI opcode
// NOTE: sign extend to 32 bits
int lui_op(int rd, unsigned int imm20)
{
    write_reg(rd, sign_extend(imm20 << 12 , 32));
    return 0;
}


// CSR, ecall/ebreak, mret/sret, wfi
// return next_pc ;
uint64_t ecall_op(int sub3 , int sub7 , uint64_t rs1 , uint64_t rd , uint64_t imm12)
{
    switch (sub3) {
    case SYSTEM_ECALL: {    // ecall , ebreak , mret, sret
        switch (imm12) {    // NOTE: use entire 12-bit to distinguish among the different instructions
        case ECALL_ECALL: {
            interrupt = mode+INT_UCALL;   // interrupt number is different for different modes
            break;
        }
        case ECALL_EBREAK: {
            interrupt = INT_BREAKPOINT;
            //printf("ebreak: pc=0x%x , cycle=0x%x\n", pc, no_cycles);
            break;
        }
        case ECALL_MRET: {
            uint64_t mstatus = read_CSR(CSR_MSTATUS);
            int prev_mode = mode;  // we are swapping mode and mstatus.mpp here
            mode = (mstatus & CSR_MSTATUS_MPP) >> 11;  // restore CPU mode from MPP ;
            // mie = mpie ; mpie=1 ; mpp = mode
            write_CSR(CSR_MSTATUS, (prev_mode << 11) | CSR_MSTATUS_MPIE | ((mstatus & CSR_MSTATUS_MPIE) >> 4));
            uint64_t next_pc = read_CSR(CSR_MEPC);
 //           printf("MRET: pc=0x%x, cycles=0x%x , next_pc=0x%x\n", pc, no_cycles , next_pc);
            return next_pc;
        }
        case ECALL_SRET: {
            uint64_t sstatus = read_CSR(CSR_SSTATUS);
            int prev_mode = mode;  // we are swapping mode and mstatus.mpp here
            mode = (sstatus & CSR_SSTATUS_SPP) >> 8;  // restore CPU mode from SPP (NOTE: one bit only) ;
            // mie = mpie ; mpie=1 ; spp = mode[0]
            write_CSR(CSR_SSTATUS, ((prev_mode&1) << 8) | CSR_SSTATUS_SPIE | ((sstatus & CSR_SSTATUS_SPIE) >> 4));
            uint64_t next_pc = read_CSR(CSR_SEPC);
 //         printf("SRET: pc=0x%llx, cycles=0x%llx , next_pc=0x%llx\n", pc, no_cycles , next_pc);
            return next_pc;
        }
        case ECALL_WFI: {
            write_CSR(CSR_MSTATUS, read_CSR(CSR_MSTATUS) | CSR_MSTATUS_MIE);
            wfi = 1;
            break;
        }
        case ECALL_SFENCEVMA: break;    // TODO: invalidate TLB etc.
        default: interrupt = INT_ILLEGAL_INSTR; break;
        }
        break;
    }      
    // NOTE: order of register read/write as rd&rs1 can be the same register; special cases when register number is 0
    // atomic read CSR into rd and write CSR from rs1
    case SYSTEM_CSRRW: {
        uint64_t new_value = read_reg((int)rs1);
        // if x0, do not read CSR, but still write CSR
        if (rd != 0) {
            write_reg((int)rd, read_CSR((int)imm12));
        }
        uint64_t time;
        io_read(IO_CLINT_TIMERL, &time);
        //printf("%ld:CSRRW@0x%lx ,  [0x%x]=0x%x , prev=0x%x\n", time , pc , imm12, new_value , regs[rd]);
        write_CSR((int)imm12, new_value);
        break;
    }
    case SYSTEM_CSRRS: {
        uint64_t new_value = read_reg((int)rs1);
        uint64_t value = read_CSR((int)imm12);
        write_reg((int)rd, value);
        if (rs1 != 0) {
            write_CSR((int)imm12, value | new_value);    // TODO: unsettable bits?
        }
        break;
    }
    case SYSTEM_CSRRC: {
        uint64_t new_value = read_reg((int)rs1);
        uint64_t value = read_CSR((int)imm12);
        write_reg((int)rd, value);
        if (rs1 != 0) {
            write_CSR((int)imm12, value & ~new_value);    // TODO: unsettable bits?
        }
        break;
    }
    case SYSTEM_CSRRWI: {
        // if x0, do not read CSR, but still write CSR
        if (rd != 0) {
            write_reg((int)rd, read_CSR((int)imm12));
        }
        write_CSR((int)imm12, rs1);  // rs1 is imm5
        break;
    }
    case SYSTEM_CSRRSI: {
        uint64_t value = read_CSR((int)imm12);
        write_reg((int)rd, value);
        if (rs1 != 0) {
            write_CSR((int)imm12, value | rs1);    // TODO: unsettable bits?
        }
        break;
    }
    case SYSTEM_CSRRCI: {
        uint64_t value = read_CSR((int)imm12);
            write_reg((int)rd, value);
        if (rs1 != 0) {
            write_CSR((int)imm12, value & ~rs1);    // TODO: unsettable bits?
        }
        break;
    }
    default: interrupt = INT_ILLEGAL_INSTR ;    // undefined sub3
    }
    return -1;  // only MRET/SRET does a jump
}


// atomic memory access instructions
void atomic_op(int sub7 , int rd , int rs1 , int rs2)
{
    uint64_t data, data2 ,  addr;

    int lrsc = (sub7 & 0x8);    //bit 28 (bit 3 of sub7) covers both instructions ;

    // NOTE: register access order critical as rd&rs1 can be the same register
    addr = read_reg(rs1);   // memory address
    if (!lrsc) {
        rw_memory(MEM_READ, addr, MEM_DWORD, &data);    // one piece of data in memory 
        data2 = read_reg(rs2);  // the other piece in register
    }

    switch (sub7 >> 4) {
    case AMO_ADD: 
        switch ((sub7 & 0xc)>>2) {
        case AMO_ADD_ADD: data2 += data; break;
        case AMO_ADD_SWAP: break;
        case AMO_ADD_LR: reservation = addr>>3; rw_memory(MEM_READ, addr, MEM_DWORD, &data); write_reg(rd, data); break;
        case AMO_ADD_SC: 
            if (reservation == (addr >>3)) {    // check to see if lr & sc match on address, i.e. reservation still there
                data = read_reg(rs2);
                rw_memory(MEM_WRITE, addr, MEM_DWORD, &data);
                write_reg(rd, 0);
                reservation = 0;
            } else {
                write_reg(rd, 1);   // TODO: do we need to reset reservation?
            } break;
        default: interrupt = INT_ILLEGAL_INSTR; break;
        } 
        break ;
    case AMO_XOR: data2 ^= data ; break;
    case AMO_AND: data2 &= data; break;
    case AMO_OR: data2 |= data; break;
    case AMO_MIN: data2 = ((int64_t)data < (int64_t)data2) ? data : data2; break;
    case AMO_MAX: data2 = ((int64_t)data < (int64_t)data2) ? data2:data;  break;
    case AMO_MINU: data2=(data<data2)?data:data2;  break;
    case AMO_MAXU: data2=(data<data2)?data2:data; break;
    default: interrupt = INT_ILLEGAL_INSTR; break;
    }

    if (!lrsc) {
        // after arithmetic operation, update memory and register
        rw_memory(MEM_WRITE, addr, MEM_DWORD, &data2);
        write_reg(rd, data);
    }
}


// 32bit atomic memory access instructions
void atomic32_op(int sub7, int rd, int rs1, int rs2)
{
    uint32_t data, data2;
    uint64_t addr , mem_data;

    int lrsc = (sub7 & 0x8);    //bit 28 (bit 3 of sub7) covers both instructions ;

    // NOTE: register access order critical as rd&rs1 can be the same register
    addr = read_reg(rs1);   // memory address
    if (!lrsc) {
        rw_memory(MEM_READ, addr, MEM_WORD, &mem_data);    // one piece of data in memory 
        data = (uint32_t)mem_data;
        data2 = (uint32_t)read_reg(rs2);  // the other piece in register
    }

    switch (sub7 >> 4) {
    case AMO_ADD:
        switch ((sub7 & 0xc) >> 2) {
        case AMO_ADD_ADD: data2 += data; break;
        case AMO_ADD_SWAP: break;
        case AMO_ADD_LR: {
            reservation = addr >> 3;
            rw_memory(MEM_READ, addr, MEM_WORD, &mem_data);
            data = (uint32_t)mem_data;
            write_reg(rd, (int64_t)(int32_t)data);
            break;
        }
        case AMO_ADD_SC:
            // TODO: mix of 32-bit and 64-bit LR/SC?
            if (reservation == (addr >> 3)) {    // check to see if lr & sc match on address, i.e. reservation still there
                data = (int32_t) read_reg(rs2);
                mem_data = data;
                rw_memory(MEM_WRITE, addr, MEM_WORD, &mem_data);
                write_reg(rd, 0);
                reservation = 0;
            }
            else {
                write_reg(rd, 1);   // TODO: do we need to reset reservation?
            } break;
        default: interrupt = INT_ILLEGAL_INSTR; break;
        }
        break;
    case AMO_XOR: data2 ^= data; break;
    case AMO_AND: data2 &= data; break;
    case AMO_OR: data2 |= data; break;
    case AMO_MIN: data2 = ((int32_t)data < (int32_t)data2) ? data : data2; break;
    case AMO_MAX: data2 = ((int32_t)data < (int32_t)data2) ? data2 : data;  break;
    case AMO_MINU: data2 = (data < data2) ? data : data2;  break;
    case AMO_MAXU: data2 = (data < data2) ? data2 : data; break;
    default: interrupt = INT_ILLEGAL_INSTR; break;
    }

    if (!lrsc) {
        // after arithmetic operation, update memory and register
        mem_data = data2;
        rw_memory(MEM_WRITE, addr, MEM_WORD, &mem_data);
        write_reg(rd, (int64_t)(int32_t)data);
    }
}


uint64_t execute_one_instruction()
{
    uint32_t instr , opcode, sub3, sub7, rs1, rs2, rd, imm12, imm5, imm7, imm20;
    int64_t next_pc = -1;
    uint64_t mem_data;

 //   if (pc == 0x800017c4) DebugBreak();

    // fetch instruction 
    if ((pc & 0x3) != 0) {
        interrupt = INTR_INSTR_MISALIGN;
        return 1;
    }
    rw_memory(MEM_INSTR, pc, MEM_WORD, &mem_data);
    instr = (uint32_t) mem_data;
   // printf("pc=0x%lx , instr=0x%x\n", pc, instr);

    // decode instruction
    opcode = (instr & OPCODE_MASK) >> OPCODE_SHIFT;
    sub3 = (instr & FUNCT3_MASK) >> FUNCT3_SHIFT;
    sub7 = (instr & FUNCT7_MASK) >> FUNCT7_SHIFT;
    rs1 = (instr & RS1_MASK) >> RS1_SHIFT;
    rs2 = (instr & RS2_MASK) >> RS2_SHIFT;
    imm12 = (instr & IMM12_MASK) >> IMM12_SHIFT;
    imm5 = (instr & IMM5_MASK) >> IMM5_SHIFT;
    imm7 = (instr & IMM7_MASK) >> IMM7_SHIFT;
    imm20 = (instr & IMM20_MASK) >> IMM20_SHIFT;
    rd = (instr & RD_MASK) >> RD_SHIFT;

    switch (opcode) {
    case OP_ADD: reg_op(rd, rs1, rs2, sub3, sub7); break;
    case OP_ADDW: reg32_op(rd, rs1, rs2, sub3, sub7); break;
    case OP_ADDI: imm_op(rd, rs1, sub3, sub7, sign_extend(imm12, 12)); break;
    case OP_ADDIW: imm32_op(rd, rs1, sub3, sub7, (uint32_t)sign_extend(imm12, 12)); break;
        // NOTE: memory address offset is signed
    case OP_LB:
        rw_memory(MEM_READ, read_reg(rs1) + sign_extend(imm12, 12), sub3, &mem_data);
        write_reg(rd, mem_data);
        break;
    case OP_SB:
        mem_data = read_reg(rs2);
        rw_memory(MEM_WRITE, read_reg(rs1) + sign_extend((imm7 << 5) | imm5, 12), sub3, &mem_data);
        break;
    case OP_BEQ: next_pc = branch_op(rs1, rs2, sub3, imm5, imm7); break;
    case OP_JAL: next_pc = jal_op(rd, imm20); break;
    case OP_JALR: next_pc = jalr_op(rd, rs1, imm12);  break;
    case OP_AUIPC: auipc_op(rd, imm20);  break;
    case OP_LUI: lui_op(rd, imm20);  break;
    case OP_ECALL: next_pc = ecall_op(sub3, sub7, rs1, rd, imm12); break;
    case OP_FENCEI: break; // TODO: don't need to anything until we have cache or pipeline
    case OP_A: 
        if (sub3 == AMO_D) atomic_op(sub7, rd, rs1, rs2); 
        else atomic32_op(sub7, rd, rs1, rs2); break;  // fault for other sub3
    default: interrupt = INT_ILLEGAL_INSTR; break;  // invalid opcode
    }
    if (next_pc == -1) next_pc = pc + 4;    // next instruction if there is no jump
    return next_pc;
}


// check to see if what kind of interrupt we should generate
void check_interrupt()
{
    uint64_t mip, mie, mstatus;

    mstatus = read_CSR(CSR_MSTATUS);
    mip = read_CSR(CSR_MIP);
    mie = read_CSR(CSR_MIE);
    // generate interrupt only if all three conditions are met:
    // MIP.MTIP , MIE.MTIE , MSTATUS.MIE
    if ((mip & CSR_MIP_MTIP) && (mie & CSR_MIE_MTIE) && ((mstatus & CSR_MSTATUS_MIE) || mode != MODE_M)) {
        interrupt = INTR_MTIMER;
        return ;
    }

    // SSI
    uint64_t sstatus = read_CSR(CSR_SSTATUS);
    uint64_t sip = read_CSR(CSR_SIP);
    uint64_t sie = read_CSR(CSR_SIE);

    // generate interrupt only if all three conditions are met:
    // SIP.SSIP , SIE.SSIE , SSTATUS.SIE
    if ((sip & CSR_SIP_SSIP) && (sie & CSR_SIE_SSIE) && ((sstatus & CSR_SSTATUS_SIE) && mode != MODE_M)) {
        interrupt = INTR_SSOFTWARE;
        return ;
    }

    // SEI
    // generate interrupt only if all three conditions are met:
    // SIP.SEIP , SIE.SEIE , SSTATUS.SIE
    if ((sip & CSR_SIP_SEIP) && (sie & CSR_SIE_SEIE) && ((sstatus & CSR_SSTATUS_SIE) && mode != MODE_M)) {
        interrupt = INTR_SEXTERNAL;
        return;
    }
}


// return next_pc; -1 if we don't take the interrupt for some reason (currently not possible)
// possible interrupts: timer + exceptions
uint64_t execute_interrupt(uint64_t interrupt)
{
    uint64_t result = -1;

    uint64_t mstatus = read_CSR(CSR_MSTATUS);
    uint64_t exception_delegate = read_CSR(CSR_MEDELEG);
    uint64_t interrupt_delegate = read_CSR(CSR_MIDELEG);
    int delegated = 0;


    // TODO: delegated interrupts should check sie?
    if (interrupt & 0x8000000000000000) {
        int interrupt_no = interrupt & 0x7fffffff;
        if (interrupt_no!=3 && interrupt_no!=7 && interrupt_no!=11) 
            delegated = (interrupt_no < 32) && (interrupt_delegate & (1LL << interrupt_no));
    }
    else {
        delegated = (interrupt < 32) && (exception_delegate & (1LL << interrupt));
    }

    // default is M mode
    if (mode==MODE_M || !delegated) {
        write_CSR(CSR_MCAUSE, interrupt);
        // mstatus: copy MIE to MPIE, clear MIE, copy current mode into MPP
        write_CSR(CSR_MSTATUS, ((read_CSR(CSR_MSTATUS) & CSR_MSTATUS_MIE) << 4) | (mode << 11));
        write_CSR(CSR_MTVAL, (interrupt & 0x8000000000000000) ? 0 : pc);   // TODO: can provide diff info for certain types of interrupts 
        write_CSR(CSR_MEPC, pc);    // NOTE: interrupt and exception cases are different, but both should save the current pc
        mode = MODE_M; // switch to M mode ;
        result = read_CSR(CSR_MTVEC);  // jump to interrupt routine, no vectoring support yet
 //       printf("[time=0x%lldus]M INTR: pc=%lx , interrupt=%x:%x, next = % lx\n", 
 //         (uint64_t)get_microseconds() , pc, interrupt>>32 , interrupt&0xffffffff, result);
    }
    else {  // trap to S mode
        write_CSR(CSR_SCAUSE, interrupt);
        // mstatus: copy SIE to SPIE, clear SIE, copy current mode(one bit) into SPP
        write_CSR(CSR_SSTATUS, ((read_CSR(CSR_SSTATUS) & CSR_SSTATUS_SIE) << 4) | ((mode & 1) << 8));
        write_CSR(CSR_STVAL, (interrupt & 0x8000000000000000) ? 0 : pc);   // TODO: can provide diff info for certain types of interrupts 
        write_CSR(CSR_SEPC, pc);    // NOTE: interrupt and exception cases are different, but both should save the current pc
        mode = MODE_S; // switch to S mode ;
        result = read_CSR(CSR_STVEC);  // jump to interrupt routine, no vectoring support yet
//      printf("[time=0x%lldus]S INTR: pc=%llx , interrupt=%x:%x , next=%llx\n", (uint64_t)get_microseconds() , pc,
//            interrupt >> 32, interrupt & 0xffffffff, result);
    }
    return result;
}


int execute_code()
{ 
    uint64_t next_pc;

    for (;;) {
        next_pc = pc ;  // assume no jump
        interrupt = INTR_NONE;     // NOTE: 0 is valid interrupt
        no_cycles++;

        // run SoC every 1024 instructions
        if ((no_cycles & 0x3ff) == 0) {
            soc_tick();
        } 

        check_interrupt();

        // 4 combination of interrupt & wfi  
        if (interrupt==INTR_NONE) {   // if no interrupt; if there is interrupt, will execute the interrupt-handling code following this if
            if (!wfi) next_pc = execute_one_instruction();    // does not change any state except for what is defined in the instruction
        }
        if (interrupt!=INTR_NONE) {
            next_pc = execute_interrupt(interrupt);
        }
   
        pc = next_pc;
    }
}

int init_cpu(uint64_t start_pc)
{
    // initialize CPU state
    pc = start_pc ;
    mode = MODE_M; // Machine-mode.
    no_cycles = 0;
    return 0;
}
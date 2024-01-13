#pragma once

#define INITIAL_PC 0x80000000



// machine-mode CSR, only defined the ones we currently use
#define CSR_MSTATUS 0x300
#define CSR_MISA 0x301
#define CSR_MEDELEG 0x302
#define CSR_MIDELEG 0x303
#define CSR_MIE 0x304
#define CSR_MTVEC 0x305
#define CSR_MSCRATCH 0x340
#define CSR_MEPC 0x341
#define CSR_MCAUSE 0x342
#define CSR_MTVAL 0x343
#define CSR_MIP 0x344
#define CSR_MTINST 0x34A
#define CSR_MTVAL2 0x34B
#define CSR_MVENDORID 0xf11
#define CSR_PMPCFG0 0x3A0
#define CSR_PMPADDR0 0x3B0

#define CSR_SSTATUS 0x100
#define CSR_SIE 0x104
#define CSR_STVEC 0x105
#define CSR_SSCRATCH 0x140
#define CSR_SEPC 0x141
#define CSR_SCAUSE 0x142
#define CSR_STVAL 0x143
#define CSR_SIP 0x144
#define CSR_SATP 0x180

#define CSR_CYCLE 0xc00

// bit masks for fields in CSRs
#define CSR_MSTATUS_MIE 0x8
#define CSR_MSTATUS_MPIE 0x80
#define CSR_MSTATUS_MPP 0x1800
#define CSR_MIP_MTIP 0x80
#define CSR_MIP_MEIP (0x1<<11)
#define CSR_MIE_MTIE 0x80
#define CSR_MIE_MEIE (0x1<<11)
#define CSR_SSTATUS_SIE 0x2
#define CSR_SSTATUS_SPIE 0x20
#define CSR_SSTATUS_SPP 0x100
#define CSR_SIP_SSIP 0x2
#define CSR_SIP_SEIP 0x200
#define CSR_SIE_SSIE 0x2
#define CSR_SIE_SEIE 0x200
#ifdef CONFIG_RV64
#define CSR_SATP_MODE_SV39 0x8
#else
#define CSR_SATP_MODE_SV32 0x1
#endif

// interrupt numbers
// TODO: separate out 1<<(xlen-1)
#define INTR_SSOFTWARE ((1LL<<(XLEN-1))|1)
#define INTR_SEXTERNAL ((1LL<<(XLEN-1))|9)
#define INTR_MTIMER ((1LL<<(XLEN-1)) | 7)
#define INTR_MEXTERNAL ((1LL<<(XLEN-1)) | 0xb)
#define INTR_INSTR_MISALIGN 0x0
#define INT_INSTR_ACCESS 0x1
#define INT_ILLEGAL_INSTR 0x2
#define INT_BREAKPOINT 0x3
#define INT_LOAD_MISALIGN 0x4
#define INT_LOAD_ACCESS 0x5
#define INT_STORE_MISALIGN 0x6
#define INT_STORE_ACCES 0x7
#define INT_UCALL 0x8
#define INT_SCALL 0x9
#define INT_MCALL 11
#define INT_INSTR_PAGEFAULT 12
#define INT_LOAD_PAGEFAULT 13
#define INT_STORE_PAGEFAULT 15


// MEMIO addresses
// on real systems these would be actual devices
// HACK: clint address different from default Linux dtb, change for xv6 first
#define IO_CLINT_START 0x02004000
#define IO_CLINT_END   0x0200c000
#define IO_CLINT_TIMERL 0x0200bff8
#define IO_CLINT_TIMERMATCHL 0x02004000
/*
#define IO_UART_START 0x10000000			// xv6 expects it at 0x10000000
#define IO_UART_END   0x10000007
#define IO_UART_DATA       0x10000000
#define IO_UART_INTRENABLE 0x10000001
#define IO_UART_FIFOCTRL   0x10000002
#define IO_UART_INTRSTATUS 0x10000002	// same addr
#define IO_UART_LINECTRL   0x10000003
#define IO_UART_MODEMCTRL  0x10000004
#define IO_UART_READY      0x10000005
#define IO_UART_MODEMSTATUS 0x10000006
*/
#define IO_UART_START 0x3000000			// xv6 expects it at 0x10000000
#define IO_UART_END   0x3000007
#define IO_UART_DATA       0x3000000
#define IO_UART_INTRENABLE 0x3000001
#define IO_UART_FIFOCTRL   0x3000002
#define IO_UART_INTRSTATUS 0x3000002	// same addr
#define IO_UART_LINECTRL   0x3000003
#define IO_UART_MODEMCTRL  0x3000004
#define IO_UART_READY      0x3000005
#define IO_UART_MODEMSTATUS 0x3000006

#define IO_PLIC_START	0x0c000000
#define IO_PLIC_END 0x0c203008
#define PLIC_PRIORITY_START 0x0c000004
#define PLIC_PRIORITY_END   0x0c000200
#define PLIC_PENDING_START 0x0c001000
#define PLIC_PENDING_END 0x0c001010
#define PLIC_ENABLE_START 0x0c002080
#define PLIC_ENABLE_END 0x0c002090
#define PLIC_THRESHOLD 0x0c201000
#define PLIC_CLAIM 0x0c201004

#define IO_VIRTIO_START 0x10001000
#define IO_VIRTIO_END   0x100010a8
// offsets
// virtio mmio control registers, mapped starting at 0x10001000.
// from qemu virtio_mmio.h
#define VIRTIO_MAGIC_VALUE		0x000 // 0x74726976
#define VIRTIO_VERSION		0x004 // version; 1 is legacy
#define VIRTIO_DEVICE_ID		0x008 // device type; 1 is net, 2 is disk
#define VIRTIO_VENDOR_ID		0x00c // 0x554d4551
#define VIRTIO_DEVICE_FEATURES	0x010
#define VIRTIO_DRIVER_FEATURES	0x020
#define VIRTIO_QUEUE_SEL		0x030 // select queue, write-only
#define VIRTIO_QUEUE_NUM_MAX	0x034 // max size of current queue, read-only
#define VIRTIO_QUEUE_NUM		0x038 // size of current queue, write-only
#define VIRTIO_QUEUE_ALIGN		0x03c // used ring alignment, write-only
#define VIRTIO_QUEUE_READY		0x044 // ready bit
#define VIRTIO_QUEUE_NOTIFY	0x050 // write-only
#define VIRTIO_INTERRUPT_STATUS	0x060 // read-only
#define VIRTIO_INTERRUPT_ACK	0x064 // write-only
#define VIRTIO_STATUS		0x070 // read/write
#define VIRTIO_QUEUE_DESC_LOW	0x080 // physical address for descriptor table, write-only
#define VIRTIO_QUEUE_DESC_HIGH	0x084
#define VIRTIO_DRIVER_DESC_LOW	0x090 // physical address for available ring, write-only
#define VIRTIO_DRIVER_DESC_HIGH	0x094
#define VIRTIO_DEVICE_DESC_LOW	0x0a0 // physical address for used ring, write-only
#define VIRTIO_DEVICE_DESC_HIGH	0x0a4


// debugging use only, don't use in production code
#define IO_DEBUG 0x11200000

// part of CPU state
extern uint32_t wfi;
extern uint8_t* vio_disk;

uint32_t io_read(uint64_t addr, uint64_t* data);
uint32_t io_write(uint64_t addr, uint64_t* data);

uint32_t init_soc();
uint64_t soc_tick();
uint64_t get_microseconds();

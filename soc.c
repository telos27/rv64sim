// clint.c: CLINT, UART, PLIC, virtio emulation
#include <stdint.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#if defined(WINDOWS) || defined(WIN32) || defined(_WIN32)
#include <conio.h>
#include <windows.h>
#endif

#include "cpu.h"
#include "soc.h"

static int vio_disk_access();

static int IsKBHit();
static int ReadKBByte();

// CLINT I/O register states
static uint64_t timer = 0;	// part of CLINT, mtime in SiFive doc
static uint64_t timer_match = 0;	// part of CLINT, mtimecmp in SiFive doc

// UART state
static uint32_t uart_interrupt = 0;
static uint32_t uart_interrupt_pending;

// virtio state
static uint64_t virtio_interrupt_pending;
// virtio implementation here is written to fit with xv6 and legacy mode in qemu?
// biggest difference with recent virtio spec is single pfn vs. three descriptor addresses
#define VIO_QUEUE_SIZE 8
#define SECTOR_SIZE 512

#define VRING_DESC_F_NEXT  1 // chained with another descriptor
#define VRING_DESC_F_WRITE 2 // device writes (vs read)

static uint32_t vio_queue_num;
static reg_type vio_queue_desc, vio_driver_desc, vio_device_desc;
static uint32_t vio_queue_align;
static uint32_t vio_status;
static uint32_t vio_queue_notify = UINT32_MAX;
static uint32_t vio_queue_ready;
static reg_type vio_used_idx;	// matches virtq.used.idx
uint8_t* vio_disk;	// disk space, allocated in init

// PLIC state
static uint32_t plic_priority[128];
static uint32_t plic_pending[4];
static uint32_t plic_enable[4];
static uint32_t plic_threshold, plic_claim;



// debugging syscall on I/O write: use x5: 1 halt 10 print all registers 11 print string (ptr in x6) 12 print integer (value in x6) 
void debug_syscall()
{

	uint64_t call_no = read_reg(5);

	switch (call_no) {
	case 1: printf("total_cycles = %lld", no_cycles); exit(0); break;
	case 10:
		printf("0x%llx ", pc);
		for (int i = 1; i < 32; i++) {
			printf("0x%llx ", read_reg(i));
		}
		printf("\n");
		break;
	case 11: printf("%s", (char*)(read_reg(6))); break;
	case 12:printf("0x%lx", read_reg(6)); break;
	default: assert(0);     // unsupported system call
		break;
	}
}





uint32_t plic_read(reg_type addr, reg_type* data)
{
	if (addr >= PLIC_PRIORITY_START && addr < PLIC_PRIORITY_END) {
		*data = plic_priority[addr - PLIC_PRIORITY_START];
	} else if (addr >= PLIC_PENDING_START && addr < PLIC_PENDING_END) {
		*data = plic_pending[addr - PLIC_PENDING_START];
	} else if (addr >= PLIC_ENABLE_START && addr < PLIC_ENABLE_END) {
		*data = plic_enable[addr - PLIC_ENABLE_START];
	} else if (addr == PLIC_THRESHOLD) {
		*data = plic_threshold;
	} else if (addr == PLIC_CLAIM) {
		*data = plic_claim;
	} else {
		assert(0);
	}
	return 0;
}


uint32_t plic_write(reg_type addr, reg_type* data)
{
	if (addr >= PLIC_PRIORITY_START && addr < PLIC_PRIORITY_END) {
		plic_priority[addr - PLIC_PRIORITY_START] = (uint32_t) *data;
	} else if (addr >= PLIC_PENDING_START && addr < PLIC_PENDING_END) {
		plic_pending[addr - PLIC_PENDING_START] = (uint32_t) *data;
	} else if (addr >= PLIC_ENABLE_START && addr < PLIC_ENABLE_END) {
		plic_enable[addr - PLIC_ENABLE_START] = (uint32_t)*data ;
	} else if (addr == PLIC_THRESHOLD) {
		plic_threshold = (uint32_t)*data;
	} else if (addr == PLIC_CLAIM) {
		// clear corresponding pending bit
		uint32_t* p = &plic_pending[(*data) >> 5];
		assert(*data < 128);
		*p &= ~(1 << (*data & 0x1f));
		uint64_t sip = read_CSR(CSR_SIP);
		write_CSR(CSR_SIP, sip & (~CSR_SIP_SEIP));
		//printf("write PLIC_COMPLETE = %d\n", *data);
		plic_claim = 0 ;		// TODO: should be next pending interrupt? current way only works if there is single pending interrupt
	} else {
		assert(0);
	}
	return 0;
}

static uint64_t last_time = 0;		// last time when we updated the timer, initialized in init_clint()

void timer_tick()
{
	// update timer based on the current time
	uint64_t elapsed_time = get_microseconds() - last_time;
	last_time += elapsed_time;
	timer += elapsed_time;
	//	printf("timer=%ld\n", timer);

	// compare timer and set interrupt pending info
	// MIP.MTIP is always updated: clear WFI & set MIP or clear MIP 
	reg_type mip = read_CSR(CSR_MIP);
	if (timer > timer_match) {	// timer_match set (if not set, >= would sitll hold) and current time>=timer_match
		wfi = 0;
		write_CSR(CSR_MIP, mip | CSR_MIP_MTIP);
	}
	else {
		// TODO: ok to reset here? 
		write_CSR(CSR_MIP, mip & ~((reg_type)CSR_MIP_MTIP));
	}
}

// valid when uart_interrupt_pending
static char uart_saved_char;
static unsigned int uart_tick_count;

int uart_tick()
{
	/*
	// do read when the buffer is empty
	if (!uart_interrupt_pending) {
		if (((uart_tick_count&0xff)==0) &&IsKBHit()) {
			uart_saved_char = ReadKBByte();
			uart_interrupt_pending = 1;
		}
	}
	uart_tick_count++;
	*/
	return 0;
}


// If there has been a notify, generate an interrupt and process the request
uint32_t vio_interrupt_pending()
{
	if (vio_queue_notify != UINT_MAX) {
		vio_queue_notify = UINT_MAX;
		return TRUE;
	}
	else {
		return FALSE;
	}
}

int plic_tick()
{
	// rvemu seems to assume a single priority pending, it should calculate according to priority and pending
	if (uart_interrupt_pending) {
		plic_pending[0] |= 1 << 10;
		plic_claim = 10;
	}
	else if (vio_interrupt_pending()) {
		vio_disk_access();
		plic_pending[0] |= 1 << 1;
		plic_claim = 1;
	}
	if (plic_claim > 0) {
		reg_type sip = read_CSR(CSR_SIP);
		write_CSR(CSR_SIP, sip | CSR_SIP_SEIP);
	}
	return 0;
}


// TODO: check vio for 32-bit mode

uint32_t init_vio()
{
	// empty right now
	vio_disk = malloc(64 * 1024 * 1024);	// 64M disk
	return 0;
}

uint32_t vio_read(reg_type addr, reg_type* data)
{
	uint32_t offset = (uint32_t) (addr - IO_VIRTIO_START);
	switch (offset) {
	case VIRTIO_STATUS: *data = vio_status; break;
	case VIRTIO_MAGIC_VALUE: *data = 0x74726976; break;
	case VIRTIO_VERSION: *data = 0x2; break;	// to make xv6 happy
	case VIRTIO_DEVICE_ID: *data = 0x2; break;
	case VIRTIO_VENDOR_ID: *data = 0x554d4551; break;
	case VIRTIO_DEVICE_FEATURES: *data = 0; break;	// appears unused
	case VIRTIO_QUEUE_NUM_MAX: *data = VIO_QUEUE_SIZE; break;
	case VIRTIO_QUEUE_READY: *data = vio_queue_ready; break;	
	case VIRTIO_INTERRUPT_STATUS: *data = 1; break;		// should be ok
	default: assert(0);		// unsupported I/O register
	}
	return 0;
}


uint32_t vio_write(reg_type addr, reg_type* data)
{
	uint32_t offset = (uint32_t) (addr - IO_VIRTIO_START);
	switch (offset) {
	case VIRTIO_STATUS: {
		vio_status = (uint32_t) *data;	// vs. rvemu; I don't think needed for xv6
		break;
	}
	case VIRTIO_DRIVER_FEATURES: break;	// appears unused
	case VIRTIO_QUEUE_SEL: assert(*data == 0); break;	// only single queue
	case VIRTIO_QUEUE_NUM: vio_queue_num = (uint32_t)*data; break;
	case VIRTIO_QUEUE_ALIGN: vio_queue_align = (uint32_t)*data; break;
	case VIRTIO_INTERRUPT_ACK: break; // don't think we need to do anything yet, as queue_notify is updated already
	case VIRTIO_QUEUE_DESC_LOW: vio_queue_desc = (vio_queue_desc & 0xffffffff00000000) | *data; break;
	case VIRTIO_QUEUE_DESC_HIGH: vio_queue_desc = (vio_queue_desc & 0xffffffff) | (*data << 32); break;
	case VIRTIO_DRIVER_DESC_LOW: vio_driver_desc = (vio_driver_desc & 0xffffffff00000000) | *data; break;
	case VIRTIO_DRIVER_DESC_HIGH: vio_driver_desc = (vio_driver_desc & 0xffffffff) | (*data << 32); break;	
	case VIRTIO_DEVICE_DESC_LOW: vio_device_desc = (vio_device_desc & 0xffffffff00000000) | *data; break;
	case VIRTIO_DEVICE_DESC_HIGH: vio_device_desc = (vio_device_desc & 0xffffffff) | (*data << 32); break;
	case VIRTIO_QUEUE_READY: vio_queue_ready = (uint32_t)*data; break;
	case VIRTIO_QUEUE_NOTIFY: vio_queue_notify = (uint32_t)*data; /*printf("virtio notify\n"); */ break;
	default: assert(0);		// unsupported I/O register
	}
	return 0;
}



static int vio_disk_access()
{
	reg_type mem_data, interrupt;
	// read 3 descriptors from virtual queue

	// address of virt queue
	reg_type virtq = vio_queue_desc ;

	// address of avail 
	reg_type avail = vio_driver_desc;	
	uint16_t avail_idx;
	pa_mem_interface(MEM_READ, avail + 2, MEM_HALFWORD, &mem_data , &interrupt);	// read avail.idx
	avail_idx = ((uint16_t)mem_data-1) % VIO_QUEUE_SIZE;
	//printf("avail_idx=%d\n", avail_idx);

	reg_type head_index;
	pa_mem_interface(MEM_READ , avail+4 + avail_idx*2 , MEM_HALFWORD , &head_index , &interrupt);	

	reg_type desc0_addr;
	// TODO: 32-bit
	pa_mem_interface(MEM_READ, virtq + 16 * head_index, MEM_DWORD, &desc0_addr, &interrupt);

	uint64_t sector;
	// TODO: 32-bit
	pa_mem_interface(MEM_READ, desc0_addr + 8, MEM_DWORD, &sector, &interrupt);		// LSB

	reg_type desc0_next;	// index for desc1
	pa_mem_interface(MEM_READ, virtq + 16 * head_index + 14, MEM_HALFWORD, &desc0_next, &interrupt);

	reg_type desc1_addr;
	// TODO: 32-bit
	pa_mem_interface(MEM_READ, virtq + 16 * desc0_next , MEM_DWORD, &desc1_addr, &interrupt);

	reg_type desc1_len;
	pa_mem_interface(MEM_READ, virtq + 16 * desc0_next + 8, MEM_WORD, &desc1_len, &interrupt);

	reg_type desc1_flags;
	pa_mem_interface(MEM_READ, virtq + 16 * desc0_next + 12, MEM_HALFWORD, &desc1_flags, &interrupt);

	reg_type desc1_next;
	pa_mem_interface(MEM_READ, virtq + 16 * desc0_next + 14, MEM_HALFWORD, &desc1_next, &interrupt);

	reg_type data;	// only 1 byte used

	assert(sector>0 && sector * SECTOR_SIZE + desc1_len < 64 * 1024 * 1024);

	// NOTE: write means device writes to buffer, which is actually a disk read
	if ((desc1_flags & VRING_DESC_F_WRITE)==0) {
		for (int i = 0; i < (signed_reg_type) desc1_len; i++) {
			pa_mem_interface(MEM_READ, desc1_addr + i, MEM_BYTE , &data, &interrupt);
			vio_disk[sector * SECTOR_SIZE + i] = (uint8_t) data;
			
		}
//		printf("vio: [0x%lx] -> sector %ld, len=%d\n", desc1_addr, sector, desc1_len);
	}
	else {
		for (int i = 0; i < (signed_reg_type)desc1_len; i++) {
			data = vio_disk[sector * SECTOR_SIZE + i];
			pa_mem_interface(MEM_WRITE, desc1_addr + i, MEM_BYTE, &data, &interrupt);
//			printf("vio read: mem[%llx] = %d\n", desc1_addr + i, (uint32_t)data);
		}
//		printf("vio: sector %ld -> [0x%llx], len=%d\n", sector , desc1_addr, desc1_len);
	}

	// set desc2's block to zero to mean completion
	reg_type desc2_addr;
	// TODO: 32-bit
	pa_mem_interface(MEM_READ, virtq + 16 * desc1_next, MEM_DWORD, &desc2_addr, &interrupt);

	data = 0;
	pa_mem_interface(MEM_WRITE, desc2_addr, MEM_BYTE, &data, &interrupt);

	// update used
	// address of used
	reg_type used = vio_device_desc ;	

	// update used.idx; 
	// NOTE: need to take mod
	pa_mem_interface(MEM_WRITE, used + 4 + 8 * (vio_used_idx%VIO_QUEUE_SIZE), MEM_WORD, &head_index, &interrupt);

	// update used.idx; vio_used_idx is same as used.idx
	// TODO: 16-bit wraparound?
	vio_used_idx = (vio_used_idx + 1);
	pa_mem_interface(MEM_WRITE, used + 2, MEM_HALFWORD, &vio_used_idx, &interrupt);
	return 0;
}



uint32_t io_read(reg_type addr, reg_type *data)
{
	if (addr >= IO_PLIC_START && addr < IO_PLIC_END) {
		return plic_read(addr, data);
	}
	if (addr >= IO_VIRTIO_START && addr < IO_VIRTIO_END) {
		return vio_read(addr, data);
	}
	switch (addr) {
#ifdef CONFIG_RV64
		case IO_CLINT_TIMERL: *data = timer; break;
		case IO_CLINT_TIMERMATCHL: *data = timer_match; break;		// turn off timer interrupt
#else
	case IO_CLINT_TIMERL: *data = timer&0xffffffff; break;
	case IO_CLINT_TIMERH: *data = timer>>32; break;
#endif
		// emulate UART behavior: different between 32-bit non-MMU Linux and xv6
		case IO_UART_DATA: *data = *data = IsKBHit() ? ReadKBByte() : 0; break;
		case IO_UART_INTRENABLE: *data = uart_interrupt; break;  // should not be readable, but Linux seems to read it
		case IO_UART_INTRSTATUS: *data = 0; break;	// used by Linux driver?
		case IO_UART_LINECTRL: *data = 0; break;	// used by Linux driver?
		case IO_UART_MODEMCTRL: *data = 0; break;	// seems to be used by Linux driver
		case IO_UART_MODEMSTATUS: *data = 0; break;	// seems to be used by Linux driver

		case IO_UART_READY: *data = 0x60|IsKBHit(); break;
		default: assert(0);break;
	}
	return 0;
}

// TODO: special mode for UART baud latch?
uint32_t io_write(reg_type addr, reg_type* data)
{
	if (addr >= IO_PLIC_START && addr < IO_PLIC_END) {
		return plic_write(addr, data);
	}
	if (addr >= IO_VIRTIO_START && addr < IO_VIRTIO_END) {
		return vio_write(addr, data);
	}
	switch (addr) {
	//	case IO_CLINT_TIMERL: timer_l = *data; break;
	//	case IO_CLINT_TIMERH: timer_h = *data; break;
#ifdef CONFIG_RV64
	case IO_CLINT_TIMERMATCHL: timer_match = *data; timer_tick();/*printf("timer match=%llu, time=%llu\n", timer_match, timer); */break;
#else
	case IO_CLINT_TIMERMATCHL: timer_match = (timer_match&0xffffffff00000000)|(*data); break;
	case IO_CLINT_TIMERMATCHH: timer_match = (timer_match&0xffffffff)|(((uint64_t)(*data))<<32); break;
#endif
		// emulate UART behavior; LCR and FCR write should be ok
		case IO_UART_DATA: printf("%c",*data); fflush(stdout); break ;
		case IO_UART_INTRENABLE: uart_interrupt = *data; break;
		case IO_UART_LINECTRL: break;

		case IO_UART_FIFOCTRL: break;
		case IO_UART_MODEMCTRL: break;	// seems to be used by Linux driver
		case IO_DEBUG: debug_syscall(); break;	// debugging only
	
		default: assert(0);break;
	}
	return 0;
}

#if defined(WINDOWS) || defined(WIN32) || defined(_WIN32)

// get current microsecond
uint64_t get_microseconds()
{
	static LARGE_INTEGER lpf;
	LARGE_INTEGER li;

	if (!lpf.QuadPart)
		QueryPerformanceFrequency(&lpf);

	QueryPerformanceCounter(&li);
	return ((uint64_t)li.QuadPart * 1000000LL) / (uint64_t)lpf.QuadPart;
}


// Windows-specific
static int IsKBHit()
{
	return _kbhit();
}

// Windows-specific
static int ReadKBByte()
{
	// This code is kind of tricky, but used to convert windows arrow keys
	// to VT100 arrow keys.
	static int is_escape_sequence = 0;
	int r;
	if (is_escape_sequence == 1)
	{
		is_escape_sequence++;
		return '[';
	}

	r = _getch();

if (is_escape_sequence)
	{
		is_escape_sequence = 0;
		switch (r)
		{
		case 'H': return 'A'; // Up
		case 'P': return 'B'; // Down
		case 'K': return 'D'; // Left
		case 'M': return 'C'; // Right
		case 'G': return 'H'; // Home
		case 'O': return 'F'; // End
		default: return r; // Unknown code.
		}
	}
	else
	{
		switch (r)
		{
		case 13: return 10; //cr->lf
		case 224: is_escape_sequence = 1; return 27; // Escape arrow keys
		default: return r;
		}
	}
}



#else

uint64_t get_microseconds()
{
	struct timeval tv;
	gettimeofday(&tv, 0);
	return tv.tv_usec + ((uint64_t)(tv.tv_sec)) * 1000000LL;
}

static int is_eofd;

static int ReadKBByte()
{
	if (is_eofd) return 0xffffffff;
	char rxchar = 0;
	int rread = read(fileno(stdin), (char*)&rxchar, 1);

	if (rread > 0) // Tricky: getchar can't be used with arrow keys.
		return rxchar;
	else
		return -1;
}

static int IsKBHit()
{
	if (is_eofd) return -1;
	int byteswaiting;
	ioctl(0, FIONREAD, &byteswaiting);
	if (!byteswaiting && write(fileno(stdin), 0, 0) != 0) { is_eofd = 1; return -1; } // Is end-of-file for 
	return !!byteswaiting;
}

#endif


void virtio_tick()
{
}


int soc_tick()
{
	uart_tick();
	virtio_tick();
	plic_tick();
	timer_tick();

	return 0;
}

// initialize all SOC components
uint32_t init_soc()
{
	// TODO: put into init_clint()
	last_time = get_microseconds();

	// init_plic();
	init_vio();
	return 0;
}


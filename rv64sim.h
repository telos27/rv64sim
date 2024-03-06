
// log-level bits: any combination of bits, currently only 64 bits
#define LOG_INSTR 0		// log every instruction
#define LOG_A 1			// log every A instruction
#define LOG_M 2			// log every M instruction
#define LOG_INSTR_COUNT 3	// log # of each type of instructions (A, M etc.)

extern uint64_t log_level;		// TODO: signed vs. unsigned

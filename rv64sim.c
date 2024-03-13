#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>

#include "rv64sim.h"
#include "cpu.h"
#include "soc.h"


// machine code file name, if unspecificed
#define DEFAULT_FILE "asm.o"

static reg_type dtb_offset;    // offset to DTB

// file names
char* bin_file_name = 0;
char* dtb_file_name = 0;
char* fs_file_name = 0;
uint64_t log_level = 0;


void usage(char *prog)
{
    fprintf(stderr, "Usage: %s -b <bin file> -d <dtb file> -f <file system file> -l <log level>\n" , prog);
}


void init_args(int argc, char** argv)
{
    for (int i = 1; i < argc; i++) {
        char *arg = argv[i];
        if (arg[0] == '-' &&strlen(arg)==2) {
            switch (arg[1]) {
                case 'b': 
                    if (i + 1 < argc) {
                        bin_file_name = argv[i + 1]; 
                        i++;
                    }
                    break;
                case 'd': 
                    if (i + 1 < argc) {
                        dtb_file_name = argv[i + 1];
                        i++;
                    }
                    break;
                case 'l': 
                    if (i + 1 < argc) {
                        log_level = atol(argv[i + 1]);
                        i++;
                    }
                    break;
                case 'f': 
                    if (i + 1 < argc) {
                        fs_file_name = argv[i + 1];
                        i++;
                    }
                    break;
                default: fprintf("unrecognized argument: %s\n", arg); usage(argv[0]);exit(2);
            }
        } else {
            fprintf(stderr, "Unexpected argument: %s\n", arg);
            usage(argv[0]);
            exit(2);
        }
    }
}


// SIGINT handler
void sigint_handler(int sig)
{
    terminate_cpu();
    exit(0);
}


int main(int argc, char** argv)
{
    signal(SIGINT, sigint_handler);

    init_args(argc, argv);

    init_soc();
    init_cpu(INITIAL_PC);

    if (bin_file_name) load_code();
    if (dtb_file_name) load_dtb();
    if (fs_file_name) load_fs();

    if (dtb_offset > 0) {
        // a0 and a1 needed for Linux
        write_reg(10, 0x0); // hart ID
        write_reg(11, dtb_offset + INITIAL_PC); // DTB address in memory
    }

    execute_code();
}



// load machine code from file into memory starting at 0
int load_code()
{
    // read instructions and save into memory
    FILE* f = fopen(bin_file_name, "rb");
    int len;

    if (f == NULL) {
        printf("file open error: %s\n", bin_file_name);
        exit(1);
    }

    uint8_t* p = mem;

    fseek(f, 0, SEEK_END);
    len = ftell(f);   
    rewind(f);
    fread(p, len, 1, f);

    fclose(f);

    return 1;
}

// load DTB file
int load_dtb()
{
    FILE* f = fopen(dtb_file_name, "rb");
    if (!f || ferror(f))
    {
        fprintf(stderr, "Error: DTB file \"%s\" not found\n", dtb_file_name);
        return -5;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    unsigned char* dest = 0;
    fseek(f, 0, SEEK_SET);
    dtb_offset = MEMSIZE - len;  // do we need to store core structure in memory?
    dtb_offset &= ~(0x1fffffLL); // 2MB addr rounding, per QEMU/riscv_em
    // printf("dtb_addr=%llx\n", dtb_offset + INITIAL_PC);
    dest = mem + dtb_offset;    // dtb is near the end of the memory
    
    if (fread(dest, len, 1, f) != 1) {
        fprintf(stderr, "Error: Could not read dtb/ \"%s\"\n", dtb_file_name);
        return -9;
    }
    fclose(f);
    return 0;
}

// load fs file
int load_fs()
{
    FILE* f = fopen(fs_file_name, "rb");
    if (!f || ferror(f))
    {
        fprintf(stderr, "Error: FS file \"%s\" not found\n", fs_file_name);
        return -5;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    unsigned char* dest = 0;
    fseek(f, 0, SEEK_SET);
    dest = vio_disk;    // this is our emulated disk
    
    if (fread(dest, len, 1, f) != 1)
    {
        fprintf(stderr, "Error: Could not read fs \"%s\"\n", fs_file_name);
        return -9;
    }
    fclose(f);
    return 0;
}


#include <stdio.h>

#include "cpu.h"
#include "soc.h"


// machine code file name, if unspecificed
#define DEFAULT_FILE "asm.o"

static uint64_t dtb_offset;    // offset to DTB

// takes two optional argument: machine code file name & dtb file name
int main(int argc, char** argv)
{
    load_code((argc <= 1) ? DEFAULT_FILE : argv[1]);
    if (argc >= 2) load_dtb(argv[2]);

    init_soc();
    init_cpu();

    // a10 and a11 needed for Linux
    write_reg(10, 0x0); // hart ID
    write_reg(11, dtb_offset + INITIAL_PC); // DTB address in memory

    execute_code();
}



// load machine from file into memory starting at 0
int load_code(char* file_name)
{
    // read instructions and save into memory
    FILE* f = fopen(file_name, "rb");
    int len;

    if (f == NULL) {
        printf("file open error: %s\n", file_name);
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
int load_dtb(char* file_name)
{
    FILE* f = fopen(file_name, "rb");
    if (!f || ferror(f))
    {
        fprintf(stderr, "Error: DTB file \"%s\" not found\n", file_name);
        return -5;
    }
    fseek(f, 0, SEEK_END);
    long dtblen = ftell(f);
    fseek(f, 0, SEEK_SET);
    dtb_offset = MEMSIZE - dtblen;  // do we need to store core structure in memory?
    if (fread(mem + dtb_offset, dtblen, 1, f) != 1)
    {
        fprintf(stderr, "Error: Could not open dtb \"%s\"\n", file_name);
        return -9;
    }
    fclose(f);
    return 0;
}


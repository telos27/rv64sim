Cross-build assembly files on Ubuntu:
  * sudo apt install gcc-riscv64-unknown-elf
  * for any assembly file x.S, make x.bin will produce the corresponding binary file
  * the current Makefile is set up to build RV32 code, but it can be easily modified to build RV64 code
  * for details please see (admittedly simplistic) Makefile
    

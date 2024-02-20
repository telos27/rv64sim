RISC-V emulator, supports both 32-bit and 64-bit (select via #define CONFIG_RV64)

Currently work on Windows, Linux and MacOS.

32-bit: RISC-V testsuite, FreeRTOS, no-MMU Linux

64-bit: xv6


Building xv6 on Linux:

  git clone https://github.com/mit-pdos/xv6-riscv
  
  replace Makefile with xv6/Makefile
  
  make
  
  make fs.img
  

  This will produce kernel/xv6.img and fs.img, which can be used as input to the emulator.
  

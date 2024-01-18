RISC-V emulator, supports both 32-bit and 64-bit (select via #define CONFIG_RV64)

Currently only works on Windows, will support Linux/MacOS in the future.

Building xv6 on Linux:
  git clone 
  replace Makefile with xv6/Makefile
  make
  make fs.img

  This will produce kernel/xv6.img and fs.img, which can be used as input to the emulator.
  

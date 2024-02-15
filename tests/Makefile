t1.bin: 

%.bin: %.o
	riscv64-unknown-elf-objcopy --dump-section .text=$@ $<

%.o: %.S
	riscv64-unknown-elf-as -march=rv32ima $< -o $@

%.hex: %.bin
	od -An -t x1 $< > $@

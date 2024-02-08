	# RISC-V 32I
	# sum 1 to 10K: sum should be 0x2fb0408
	# NOTE: turn on LED first ; after it finishes, it will flash LED if correct, turn off LED if incorrect


	jal flip_led
	
	xor t3 , t3 , t3 	# sum
	li t4 , 0x1		# i
	li t5 , 10000		# loop limit


LOOP:	
	bgt t4 , t5 , DONE
	add t3 , t3 , t4	# sum += i
	addi t4 ,t4 , 1		# i++ 
	j LOOP

DONE:
	li t4 , 0x2fb0408	# expected sum
	bne t3, t4 , incorrect
L2:	jal flip_led		# blink LED
	j L2


incorrect:
	jal flip_led		# turn off LED
L3:	j L3			# loop forever

	
flip_led:	
	li t5 , 3000000		# about 0.5s
	li t4 , 0x1

	not x31 , x31		# flip LED

L1:	
	bgt t4 , t5, RET
	addi t4 , t4 , 1
	j L1

RET:	
	ret

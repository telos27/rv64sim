// copied and modified from picorv32
#ifndef _ENV_PICORV32_TEST_H
#define _ENV_PICORV32_TEST_H

#include "encoding.h"

#ifndef TEST_FUNC_NAME
#  define TEST_FUNC_NAME mytest
#  define TEST_FUNC_TXT "mytest"
#  define TEST_FUNC_RET mytest_ret
#endif

#define RVTEST_RV32U
#define RVTEST_RV32M
#define TESTNUM x28

#define RVTEST_CODE_BEGIN		\
	.text;				\
	.global TEST_FUNC_NAME;		\
	.global TEST_FUNC_RET;		\
TEST_FUNC_NAME:				\
	la sp, 0x8000ffff;		\
	sw ra, (sp);			\
	lui	a3,%hi(.test_name);	\
	addi a3,a3,%lo(.test_name);	\
.prname_next:				\
	lb	a0,0(a3);		\
	beq	a0,zero,.prname_done;	\
	addi	a3,a3,1;		\
	jal	zero,.prname_next;	\
.test_name:				\
	.ascii TEST_FUNC_TXT;		\
	.byte 0x00;			\
	.balign 4, 0;			\
.prname_done:				\
	addi	a0,zero,'.';		\
	lw ra, (sp);			\
	//sw	a1,0(a2);		\
	//sw	a1,0(a2);
	//sw	a0,0(a2);		\
	//lui	a2,0x10000000>>12;	\

#define RVTEST_PASS			\
	la sp, 0x8000ffff;		\
	sw ra, (sp);			\
	addi a0, zero, 'o';		\
	addi a0, zero, 'k';		\
	addi a0, zero, '\r';	\
	addi a0, zero, '\n';	\
	lw ra, (sp);			\
	jal zero, TEST_FUNC_RET;
	//lui	a0,0x10000000>>12;	\
	addi	a1,zero,'O';		\
	addi	a2,zero,'K';		\
	addi	a3,zero,'\n';		\
	sw	a1,0(a0);		\
	sw	a2,0(a0);		\
	sw	a3,0(a0);		\
	jal	zero,TEST_FUNC_RET;

#define RVTEST_FAIL			\
	la sp, 0x8000ffff;		\
	sw ra, (sp);			\
	li x30, 0xdeadbeef;		\
	addi a0, zero, 'e';		\
	addi a0, zero, 'r';		\
	addi a0, zero, 'r';		\
	addi a0, zero, 'o';		\
	addi a0, zero, 'r';		\
	addi a0, zero, '\r';	\
	addi a0, zero, '\n';	\
	lw ra, (sp);			\
	jal zero, TEST_FUNC_RET;
	//lui	a0,0x10000000>>12;	\
	addi	a1,zero,'E';		\
	addi	a2,zero,'R';		\
	addi	a3,zero,'O';		\
	addi	a4,zero,'\n';		\
	sw	a1,0(a0);		\
	sw	a2,0(a0);		\
	sw	a2,0(a0);		\
	sw	a3,0(a0);		\
	sw	a2,0(a0);		\
	sw	a4,0(a0);		\
	ebreak;

#define RVTEST_CODE_END
#define RVTEST_DATA_BEGIN .balign 4;
#define RVTEST_DATA_END

#endif

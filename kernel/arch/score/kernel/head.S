
#include <linux/init.h>
#include <linux/linkage.h>

#include <asm/asm-offsets.h>

	.extern start_kernel
	.global init_thread_union
	.global	kernelsp

__INIT
ENTRY(_stext)
	la	r30, __bss_start	/* initialize BSS segment. */
	la	r31, _end
	xor	r8, r8, r8

1:	cmp.c	r31, r30
	beq	2f

	sw	r8, [r30]		/* clean memory. */
	addi	r30, 4
	b	1b

2:	la	r28, init_thread_union	/* set kernel stack. */
	mv	r0, r28
	addi	r0, KERNEL_STACK_SIZE - 32
	la	r30, kernelsp
	sw	r0, [r30]
	subi	r0, 4*4
	xor	r30, r30, r30
	ori	r30, 0x02		/* enable MMU. */
	mtcr	r30, cr4
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	/* there is no parameter */
	xor	r4, r4, r4
	xor	r5, r5, r5
	xor	r6, r6, r6
	xor	r7, r7, r7
	la	r30, start_kernel	/* jump to init_arch */
	br	r30

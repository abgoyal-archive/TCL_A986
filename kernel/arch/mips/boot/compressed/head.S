

#include <asm/asm.h>
#include <asm/regdef.h>

	.set noreorder
	.cprestore
	LEAF(start)
start:
	/* Save boot rom start args */
	move	s0, a0
	move	s1, a1
	move	s2, a2
	move	s3, a3

	/* Clear BSS */
	PTR_LA	a0, _edata
	PTR_LA	a2, _end
1:	sw	zero, 0(a0)
	bne	a2, a0, 1b
	 addiu	a0, a0, 4

	PTR_LA	a0, (.heap)          /* heap address */
	PTR_LA  sp, (.stack + 8192)  /* stack address */

	PTR_LA	ra, 2f
	PTR_LA	k0, decompress_kernel
	jr	k0
	 nop
2:
	move	a0, s0
	move	a1, s1
	move	a2, s2
	move	a3, s3
	PTR_LI	k0, KERNEL_ENTRY
	jr	k0
	 nop
3:
	b	3b
	 nop
	END(start)

	.comm .heap,BOOT_HEAP_SIZE,4
	.comm .stack,4096*2,4


#ifndef __ASM_MACH_MIPS_KERNEL_ENTRY_INIT_H
#define __ASM_MACH_MIPS_KERNEL_ENTRY_INIT_H

	.macro	kernel_entry_setup
#ifdef CONFIG_MIPS_MT_SMTC
	mfc0	t0, CP0_CONFIG
	bgez	t0, 9f
	mfc0	t0, CP0_CONFIG, 1
	bgez	t0, 9f
	mfc0	t0, CP0_CONFIG, 2
	bgez	t0, 9f
	mfc0	t0, CP0_CONFIG, 3
	and	t0, 1<<2
	bnez	t0, 0f
9:
	/* Assume we came from YAMON... */
	PTR_LA	v0, 0x9fc00534	/* YAMON print */
	lw	v0, (v0)
	move	a0, zero
	PTR_LA	a1, nonmt_processor
	jal	v0

	PTR_LA	v0, 0x9fc00520	/* YAMON exit */
	lw	v0, (v0)
	li	a0, 1
	jal	v0

1:	b	1b

	__INITDATA
nonmt_processor:
	.asciz	"SMTC kernel requires the MT ASE to run\n"
	__FINIT
0:
#endif
	.endm

	.macro	smp_slave_setup
	.endm

#endif /* __ASM_MACH_MIPS_KERNEL_ENTRY_INIT_H */

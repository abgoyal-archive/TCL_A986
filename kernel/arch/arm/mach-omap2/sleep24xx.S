

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/io.h>

#include <plat/omap24xx.h>

#include "sdrc.h"

/* First address of reserved address space?  apparently valid for OMAP2 & 3 */
#define A_SDRC0_V		(0xC0000000)

	.text

ENTRY(omap24xx_idle_loop_suspend)
	stmfd	sp!, {r0, lr}		@ save registers on stack
	mov	r0, #0			@ clear for mcr setup
	mcr	p15, 0, r0, c7, c0, 4	@ wait for interrupt
	ldmfd	sp!, {r0, pc}		@ restore regs and return

ENTRY(omap24xx_idle_loop_suspend_sz)
	.word	. - omap24xx_idle_loop_suspend

ENTRY(omap24xx_cpu_suspend)
	stmfd	sp!, {r0 - r12, lr}	@ save registers on stack
	mov	r3, #0x0		@ clear for mcr call
	mcr	p15, 0, r3, c7, c10, 4	@ memory barrier, hope SDR/DDR finished
	nop
	nop
	ldr	r4, [r2]		@ read SDRC_POWER
	orr	r4, r4, #0x40		@ enable self refresh on idle req
	mov	r5, #0x2000		@ set delay (DPLL relock + DLL relock)
	str	r4, [r2]		@ make it so
	nop
	mcr	p15, 0, r3, c7, c0, 4	@ wait for interrupt
	nop
loop:
	subs	r5, r5, #0x1		@ awake, wait just a bit
	bne	loop

	/* The DPLL has to be on before we take the DDR out of self refresh */
	bic	r4, r4, #0x40		@ now clear self refresh bit.
	str	r4, [r2]		@ write to SDRC_POWER
	ldr	r4, A_SDRC0		@ make a clock happen
	ldr	r4, [r4]		@ read A_SDRC0
	nop				@ start auto refresh only after clk ok
	movs	r0, r0			@ see if DDR or SDR
	strne	r0, [r1]		@ rewrite DLLA to force DLL reload
	addne	r1, r1, #0x8		@ move to DLLB
	strne	r0, [r1]		@ rewrite DLLB to force DLL reload

	mov	r5, #0x1000
loop2:
	subs	r5, r5, #0x1
	bne	loop2
	/* resume*/
	ldmfd	sp!, {r0 - r12, pc}	@ restore regs and return

A_SDRC0:
	.word A_SDRC0_V

ENTRY(omap24xx_cpu_suspend_sz)
	.word	. - omap24xx_cpu_suspend

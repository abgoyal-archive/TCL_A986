

#include <linux/linkage.h>
#include <linux/init.h>

/* Physical address needed since MMU not enabled yet on secondary core */
#define OMAP4_AUX_CORE_BOOT1_PA			0x48281804

	__INIT

ENTRY(omap_secondary_startup)
hold:	ldr	r12,=0x103
	dsb
	smc	#0			@ read from AuxCoreBoot0
	mov	r0, r0, lsr #9
	mrc	p15, 0, r4, c0, c0, 5
	and	r4, r4, #0x0f
	cmp	r0, r4
	bne	hold

	/*
	 * we've been released from the wait loop,secondary_stack
	 * should now contain the SVC stack for this core
	 */
	b	secondary_startup
END(omap_secondary_startup)


ENTRY(omap_modify_auxcoreboot0)
	stmfd   sp!, {r1-r12, lr}
	ldr	r12, =0x104
	dsb
	smc	#0
	ldmfd   sp!, {r1-r12, pc}
END(omap_modify_auxcoreboot0)

ENTRY(omap_auxcoreboot_addr)
	stmfd   sp!, {r2-r12, lr}
	ldr	r12, =0x105
	dsb
	smc	#0
	ldmfd   sp!, {r2-r12, pc}
END(omap_auxcoreboot_addr)


#include <asm/processor.h>
#include <asm/cputable.h>
#include <asm/ppc_asm.h>
#include <asm/cache.h>
#include <asm/page.h>

_GLOBAL(_set_L2CR)
	/* Make sure this is a 750 or 7400 chip */
BEGIN_FTR_SECTION
	li	r3,-1
	blr
END_FTR_SECTION_IFCLR(CPU_FTR_L2CR)

	mflr	r9

	/* Stop DST streams */
BEGIN_FTR_SECTION
	DSSALL
	sync
END_FTR_SECTION_IFSET(CPU_FTR_ALTIVEC)

	/* Turn off interrupts and data relocation. */
	mfmsr	r7		/* Save MSR in r7 */
	rlwinm	r4,r7,0,17,15
	rlwinm	r4,r4,0,28,26	/* Turn off DR bit */
	sync
	mtmsr	r4
	isync

	/* Before we perform the global invalidation, we must disable dynamic
	 * power management via HID0[DPM] to work around a processor bug where
	 * DPM can possibly interfere with the state machine in the processor
	 * that invalidates the L2 cache tags.
	 */
	mfspr	r8,SPRN_HID0		/* Save HID0 in r8 */
	rlwinm	r4,r8,0,12,10		/* Turn off HID0[DPM] */
	sync
	mtspr	SPRN_HID0,r4		/* Disable DPM */
	sync

	/* Get the current enable bit of the L2CR into r4 */
	mfspr	r4,SPRN_L2CR

	/* Tweak some bits */
	rlwinm	r5,r3,0,0,0		/* r5 contains the new enable bit */
	rlwinm	r3,r3,0,11,9		/* Turn off the invalidate bit */
	rlwinm	r3,r3,0,1,31		/* Turn off the enable bit */

	/* Check to see if we need to flush */
	rlwinm.	r4,r4,0,0,0
	beq	2f

	/* Flush the cache. First, read the first 4MB of memory (physical) to
	 * put new data in the cache.  (Actually we only need
	 * the size of the L2 cache plus the size of the L1 cache, but 4MB will
	 * cover everything just to be safe).
	 */

	 /**** Might be a good idea to set L2DO here - to prevent instructions
	       from getting into the cache.  But since we invalidate
	       the next time we enable the cache it doesn't really matter.
	       Don't do this unless you accomodate all processor variations.
	       The bit moved on the 7450.....
	  ****/

BEGIN_FTR_SECTION
	/* Disable L2 prefetch on some 745x and try to ensure
	 * L2 prefetch engines are idle. As explained by errata
	 * text, we can't be sure they are, we just hope very hard
	 * that well be enough (sic !). At least I noticed Apple
	 * doesn't even bother doing the dcbf's here...
	 */
	mfspr	r4,SPRN_MSSCR0
	rlwinm	r4,r4,0,0,29
	sync
	mtspr	SPRN_MSSCR0,r4
	sync
	isync
	lis	r4,KERNELBASE@h
	dcbf	0,r4
	dcbf	0,r4
	dcbf	0,r4
	dcbf	0,r4
END_FTR_SECTION_IFSET(CPU_FTR_SPEC7450)

	/* TODO: use HW flush assist when available */

	lis	r4,0x0002
	mtctr	r4
	li	r4,0
1:
	lwzx	r0,r0,r4
	addi	r4,r4,32		/* Go to start of next cache line */
	bdnz	1b
	isync

	/* Now, flush the first 4MB of memory */
	lis	r4,0x0002
	mtctr	r4
	li	r4,0
	sync
1:
	dcbf	0,r4
	addi	r4,r4,32		/* Go to start of next cache line */
	bdnz	1b

2:
	/* Set up the L2CR configuration bits (and switch L2 off) */
	/* CPU errata: Make sure the mtspr below is already in the
	 * L1 icache
	 */
	b	20f
	.balign	L1_CACHE_BYTES
22:
	sync
	mtspr	SPRN_L2CR,r3
	sync
	b	23f
20:
	b	21f
21:	sync
	isync
	b	22b

23:
	/* Perform a global invalidation */
	oris	r3,r3,0x0020
	sync
	mtspr	SPRN_L2CR,r3
	sync
	isync				/* For errata */

BEGIN_FTR_SECTION
	/* On the 7450, we wait for the L2I bit to clear......
	*/
10:	mfspr	r3,SPRN_L2CR
	andis.	r4,r3,0x0020
	bne	10b
	b	11f
END_FTR_SECTION_IFSET(CPU_FTR_SPEC7450)

	/* Wait for the invalidation to complete */
3:	mfspr	r3,SPRN_L2CR
	rlwinm.	r4,r3,0,31,31
	bne	3b

11:	rlwinm	r3,r3,0,11,9		/* Turn off the L2I bit */
	sync
	mtspr	SPRN_L2CR,r3
	sync

	/* See if we need to enable the cache */
	cmplwi	r5,0
	beq	4f

	/* Enable the cache */
	oris	r3,r3,0x8000
	mtspr	SPRN_L2CR,r3
	sync
	
	/* Enable L2 HW prefetch on 744x/745x */
BEGIN_FTR_SECTION
	mfspr	r3,SPRN_MSSCR0
	ori	r3,r3,3
	sync
	mtspr	SPRN_MSSCR0,r3
	sync
	isync
END_FTR_SECTION_IFSET(CPU_FTR_SPEC7450)
4:

	/* Restore HID0[DPM] to whatever it was before */
	sync
	mtspr	1008,r8
	sync

	/* Restore MSR (restores EE and DR bits to original state) */
	SYNC
	mtmsr	r7
	isync

	mtlr	r9
	blr

_GLOBAL(_get_L2CR)
	/* Return the L2CR contents */
	li	r3,0
BEGIN_FTR_SECTION
	mfspr	r3,SPRN_L2CR
END_FTR_SECTION_IFSET(CPU_FTR_L2CR)
	blr



_GLOBAL(_set_L3CR)
	/* Make sure this is a 745x chip */
BEGIN_FTR_SECTION
	li	r3,-1
	blr
END_FTR_SECTION_IFCLR(CPU_FTR_L3CR)

	/* Turn off interrupts and data relocation. */
	mfmsr	r7		/* Save MSR in r7 */
	rlwinm	r4,r7,0,17,15
	rlwinm	r4,r4,0,28,26	/* Turn off DR bit */
	sync
	mtmsr	r4
	isync

	/* Stop DST streams */
	DSSALL
	sync

	/* Get the current enable bit of the L3CR into r4 */
	mfspr	r4,SPRN_L3CR

	/* Tweak some bits */
	rlwinm	r5,r3,0,0,0		/* r5 contains the new enable bit */
	rlwinm	r3,r3,0,22,20		/* Turn off the invalidate bit */
	rlwinm	r3,r3,0,2,31		/* Turn off the enable & PE bits */
	rlwinm	r3,r3,0,5,3		/* Turn off the clken bit */
	/* Check to see if we need to flush */
	rlwinm.	r4,r4,0,0,0
	beq	2f

	/* Flush the cache.
	 */

	/* TODO: use HW flush assist */

	lis	r4,0x0008
	mtctr	r4
	li	r4,0
1:
	lwzx	r0,r0,r4
	dcbf	0,r4
	addi	r4,r4,32		/* Go to start of next cache line */
	bdnz	1b

2:
	/* Set up the L3CR configuration bits (and switch L3 off) */
	sync
	mtspr	SPRN_L3CR,r3
	sync

	oris	r3,r3,L3CR_L3RES@h		/* Set reserved bit 5 */
	mtspr	SPRN_L3CR,r3
	sync
	oris	r3,r3,L3CR_L3CLKEN@h		/* Set clken */
	mtspr	SPRN_L3CR,r3
	sync

	/* Wait for stabilize */
	li	r0,256
	mtctr	r0
1:	bdnz	1b

	/* Perform a global invalidation */
	ori	r3,r3,0x0400
	sync
	mtspr	SPRN_L3CR,r3
	sync
	isync

	/* We wait for the L3I bit to clear...... */
10:	mfspr	r3,SPRN_L3CR
	andi.	r4,r3,0x0400
	bne	10b

	/* Clear CLKEN */
	rlwinm	r3,r3,0,5,3		/* Turn off the clken bit */
	mtspr	SPRN_L3CR,r3
	sync

	/* Wait for stabilize */
	li	r0,256
	mtctr	r0
1:	bdnz	1b

	/* See if we need to enable the cache */
	cmplwi	r5,0
	beq	4f

	/* Enable the cache */
	oris	r3,r3,(L3CR_L3E | L3CR_L3CLKEN)@h
	mtspr	SPRN_L3CR,r3
	sync

	/* Wait for stabilize */
	li	r0,256
	mtctr	r0
1:	bdnz	1b

	/* Restore MSR (restores EE and DR bits to original state) */
4:	SYNC
	mtmsr	r7
	isync
	blr

_GLOBAL(_get_L3CR)
	/* Return the L3CR contents */
	li	r3,0
BEGIN_FTR_SECTION
	mfspr	r3,SPRN_L3CR
END_FTR_SECTION_IFSET(CPU_FTR_L3CR)
	blr



_GLOBAL(__flush_disable_L1)
	/* Stop pending alitvec streams and memory accesses */
BEGIN_FTR_SECTION
	DSSALL
END_FTR_SECTION_IFSET(CPU_FTR_ALTIVEC)
 	sync

	/* Load counter to 0x4000 cache lines (512k) and
	 * load cache with datas
	 */
	li	r3,0x4000	/* 512kB / 32B */
	mtctr	r3
	lis	r3,KERNELBASE@h
1:
	lwz	r0,0(r3)
	addi	r3,r3,0x0020	/* Go to start of next cache line */
	bdnz	1b
	isync
	sync

	/* Now flush those cache lines */
	li	r3,0x4000	/* 512kB / 32B */
	mtctr	r3
	lis	r3,KERNELBASE@h
1:
	dcbf	0,r3
	addi	r3,r3,0x0020	/* Go to start of next cache line */
	bdnz	1b
	sync

	/* We can now disable the L1 cache (HID0:DCE, HID0:ICE) */
	mfspr	r3,SPRN_HID0
	rlwinm	r3,r3,0,18,15
	mtspr	SPRN_HID0,r3
	sync
	isync
 	blr

_GLOBAL(__inval_enable_L1)
	/* Enable and then Flash inval the instruction & data cache */
	mfspr	r3,SPRN_HID0
	ori	r3,r3, HID0_ICE|HID0_ICFI|HID0_DCE|HID0_DCI
	sync
	isync
	mtspr	SPRN_HID0,r3
	xori	r3,r3, HID0_ICFI|HID0_DCI
	mtspr	SPRN_HID0,r3
	sync

 	blr



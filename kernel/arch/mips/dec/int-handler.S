

#include <asm/addrspace.h>
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>

#include <asm/dec/interrupts.h>
#include <asm/dec/ioasic_addrs.h>
#include <asm/dec/ioasic_ints.h>
#include <asm/dec/kn01.h>
#include <asm/dec/kn02.h>
#include <asm/dec/kn02xa.h>
#include <asm/dec/kn03.h>

#define KN02_CSR_BASE		CKSEG1ADDR(KN02_SLOT_BASE + KN02_CSR)
#define KN02XA_IOASIC_BASE	CKSEG1ADDR(KN02XA_SLOT_BASE + IOASIC_IOCTL)
#define KN03_IOASIC_BASE	CKSEG1ADDR(KN03_SLOT_BASE + IOASIC_IOCTL)

		.text
		.set	noreorder
		.align	5
		NESTED(plat_irq_dispatch, PT_SIZE, ra)
		.set	noreorder

		/*
		 * Get pending Interrupts
		 */
		mfc0	t0,CP0_CAUSE		# get pending interrupts
		mfc0	t1,CP0_STATUS
#ifdef CONFIG_32BIT
		lw	t2,cpu_fpu_mask
#endif
		andi	t0,ST0_IM		# CAUSE.CE may be non-zero!
		and	t0,t1			# isolate allowed ones

		beqz	t0,spurious

#ifdef CONFIG_32BIT
		 and	t2,t0
		bnez	t2,fpu			# handle FPU immediately
#endif

		/*
		 * Find irq with highest priority
		 */
		 PTR_LA	t1,cpu_mask_nr_tbl
1:		lw	t2,(t1)
		nop
		and	t2,t0
		beqz	t2,1b
		 addu	t1,2*PTRSIZE		# delay slot

		/*
		 * Do the low-level stuff
		 */
		lw	a0,(-PTRSIZE)(t1)
		nop
		bgez	a0,handle_it		# irq_nr >= 0?
						# irq_nr < 0: it is an address
		 nop
		jr	a0
						# a trick to save a branch:
		 lui	t2,(KN03_IOASIC_BASE>>16)&0xffff
						# upper part of IOASIC Address

		FEXPORT(kn02_io_int)		# 3max
		lui	t0,(KN02_CSR_BASE>>16)&0xffff
						# get interrupt status and mask
		lw	t0,(t0)
		nop
		andi	t1,t0,KN02_IRQ_ALL
		b	1f
		 srl	t0,16			# shift interrupt mask

		FEXPORT(kn02xa_io_int)		# 3min/maxine
		lui	t2,(KN02XA_IOASIC_BASE>>16)&0xffff
						# upper part of IOASIC Address

		FEXPORT(kn03_io_int)		# 3max+ (t2 loaded earlier)
		lw	t0,IO_REG_SIR(t2)	# get status: IOASIC sir
		lw	t1,IO_REG_SIMR(t2)	# get mask:   IOASIC simr
		nop

1:		and	t0,t1			# mask out allowed ones

		beqz	t0,spurious

		/*
		 * Find irq with highest priority
		 */
		 PTR_LA	t1,asic_mask_nr_tbl
2:		lw	t2,(t1)
		nop
		and	t2,t0
		beq	zero,t2,2b
		 addu	t1,2*PTRSIZE		# delay slot

		/*
		 * Do the low-level stuff
		 */
		lw	a0,%lo(-PTRSIZE)(t1)
		nop
		bgez	a0,handle_it		# irq_nr >= 0?
						# irq_nr < 0: it is an address
		 nop
		jr	a0
		 nop				# delay slot

		FEXPORT(cpu_all_int)		# HALT, timers, software junk
		li	a0,DEC_CPU_IRQ_BASE
		srl	t0,CAUSEB_IP
		li	t1,CAUSEF_IP>>CAUSEB_IP	# mask
		b	1f
		 li	t2,4			# nr of bits / 2

		FEXPORT(kn02_all_int)		# impossible ?
		li	a0,KN02_IRQ_BASE
		li	t1,KN02_IRQ_ALL		# mask
		b	1f
		 li	t2,4			# nr of bits / 2

		FEXPORT(asic_all_int)		# various I/O ASIC junk
		li	a0,IO_IRQ_BASE
		li	t1,IO_IRQ_ALL		# mask
		b	1f
		 li	t2,8			# nr of bits / 2

		FEXPORT(asic_dma_int)		# I/O ASIC DMA events
		li	a0,IO_IRQ_BASE+IO_INR_DMA
		srl	t0,IO_INR_DMA
		li	t1,IO_IRQ_DMA>>IO_INR_DMA # mask
		li	t2,8			# nr of bits / 2

		/*
		 * Find irq with highest priority.
		 * Highest irq number takes precedence.
		 */
1:		srlv	t3,t1,t2
2:		xor	t1,t3
		and	t3,t0,t1
		beqz	t3,3f
		 nop
		move	t0,t3
		addu	a0,t2
3:		srl	t2,1
		bnez	t2,2b
		 srlv	t3,t1,t2

handle_it:
		j	dec_irq_dispatch
		 nop

#ifdef CONFIG_32BIT
fpu:
		j	handle_fpe_int
		 nop
#endif

spurious:
		j	spurious_interrupt
		 nop
		END(plat_irq_dispatch)

		FEXPORT(dec_intr_unimplemented)
		move	a1,t0			# cheats way of printing an arg!
		PANIC("Unimplemented cpu interrupt! CP0_CAUSE: 0x%08x");

		FEXPORT(asic_intr_unimplemented)
		move	a1,t0			# cheats way of printing an arg!
		PANIC("Unimplemented asic interrupt! ASIC ISR: 0x%08x");

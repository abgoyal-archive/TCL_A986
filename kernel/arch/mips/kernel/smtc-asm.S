


#include <asm/regdef.h>
#include <asm/asmmacro.h>
#include <asm/stackframe.h>
#include <asm/irqflags.h>

	.set noreorder



#define PT_PADSLOT4 (PT_R0-8)
#define PT_PADSLOT5 (PT_R0-4)

	.text
	.align 5
FEXPORT(__smtc_ipi_vector)
	.set	noat
	/* Disable thread scheduling to make Status update atomic */
	DMT	27					# dmt	k1
	_ehb
	/* Set EXL */
	mfc0	k0,CP0_STATUS
	ori	k0,k0,ST0_EXL
	mtc0	k0,CP0_STATUS
	_ehb
	/* Thread scheduling now inhibited by EXL. Restore TE state. */
	andi	k1,k1,VPECONTROL_TE
	beqz	k1,1f
	emt
1:
	/*
	 * The IPI sender has put some information on the anticipated
	 * kernel stack frame.  If we were in user mode, this will be
	 * built above the saved kernel SP.  If we were already in the
	 * kernel, it will be built above the current CPU SP.
	 *
	 * Were we in kernel mode, as indicated by CU0?
	 */
	sll	k1,k0,3
	.set noreorder
	bltz	k1,2f
	move	k1,sp
	.set reorder
	/*
	 * If previously in user mode, set CU0 and use kernel stack.
	 */
	li	k1,ST0_CU0
	or	k1,k1,k0
	mtc0	k1,CP0_STATUS
	_ehb
	get_saved_sp
	/* Interrupting TC will have pre-set values in slots in the new frame */
2:	subu	k1,k1,PT_SIZE
	/* Load TCStatus Value */
	lw	k0,PT_TCSTATUS(k1)
	/* Write it to TCStatus to restore CU/KSU/IXMT state */
	mtc0	k0,$2,1
	_ehb
	lw	k0,PT_EPC(k1)
	mtc0	k0,CP0_EPC
	/* Save all will redundantly recompute the SP, but use it for now */
	SAVE_ALL
	CLI
	TRACE_IRQS_OFF
	/* Function to be invoked passed stack pad slot 5 */
	lw	t0,PT_PADSLOT5(sp)
	/* Argument from sender passed in stack pad slot 4 */
	lw	a0,PT_PADSLOT4(sp)
	LONG_L	s0, TI_REGS($28)
	LONG_S	sp, TI_REGS($28)
	PTR_LA	ra, ret_from_irq
	jr	t0


LEAF(self_ipi)
	/* Before anything else, block interrupts */
	mfc0	t0,CP0_TCSTATUS
	ori	t1,t0,TCSTATUS_IXMT
	mtc0	t1,CP0_TCSTATUS
	_ehb
	/* We know we're in kernel mode, so prepare stack frame */
	subu	t1,sp,PT_SIZE
	sw	ra,PT_EPC(t1)
	sw	a0,PT_PADSLOT4(t1)
	la	t2,ipi_decode
	sw	t2,PT_PADSLOT5(t1)
	/* Save pre-disable value of TCStatus */
	sw	t0,PT_TCSTATUS(t1)
	j	__smtc_ipi_vector
	nop
END(self_ipi)

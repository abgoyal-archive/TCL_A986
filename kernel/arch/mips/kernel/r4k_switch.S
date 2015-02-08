
#include <asm/asm.h>
#include <asm/cachectl.h>
#include <asm/fpregdef.h>
#include <asm/mipsregs.h>
#include <asm/asm-offsets.h>
#include <asm/page.h>
#include <asm/pgtable-bits.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>
#include <asm/thread_info.h>

#include <asm/asmmacro.h>

#define ST_OFF (_THREAD_SIZE - 32 - PT_SIZE + PT_STATUS)


	.align	5
	LEAF(resume)
	mfc0	t1, CP0_STATUS
	LONG_S	t1, THREAD_STATUS(a0)
	cpu_save_nonscratch a0
	LONG_S	ra, THREAD_REG31(a0)

	/*
	 * check if we need to save FPU registers
	 */
	PTR_L	t3, TASK_THREAD_INFO(a0)
	LONG_L	t0, TI_FLAGS(t3)
	li	t1, _TIF_USEDFPU
	and	t2, t0, t1
	beqz	t2, 1f
	nor	t1, zero, t1

	and	t0, t0, t1
	LONG_S	t0, TI_FLAGS(t3)

	/*
	 * clear saved user stack CU1 bit
	 */
	LONG_L	t0, ST_OFF(t3)
	li	t1, ~ST0_CU1
	and	t0, t0, t1
	LONG_S	t0, ST_OFF(t3)

	fpu_save_double a0 t0 t1		# c0_status passed in t0
						# clobbers t1
1:

	/*
	 * The order of restoring the registers takes care of the race
	 * updating $28, $29 and kernelsp without disabling ints.
	 */
	move	$28, a2
	cpu_restore_nonscratch a1

	PTR_ADDU	t0, $28, _THREAD_SIZE - 32
	set_saved_sp	t0, t1, t2
#ifdef CONFIG_MIPS_MT_SMTC
	/* Read-modify-writes of Status must be atomic on a VPE */
	mfc0	t2, CP0_TCSTATUS
	ori	t1, t2, TCSTATUS_IXMT
	mtc0	t1, CP0_TCSTATUS
	andi	t2, t2, TCSTATUS_IXMT
	_ehb
	DMT	8				# dmt	t0
	move	t1,ra
	jal	mips_ihb
	move	ra,t1
#endif /* CONFIG_MIPS_MT_SMTC */
	mfc0	t1, CP0_STATUS		/* Do we really need this? */
	li	a3, 0xff01
	and	t1, a3
	LONG_L	a2, THREAD_STATUS(a1)
	nor	a3, $0, a3
	and	a2, a3
	or	a2, t1
	mtc0	a2, CP0_STATUS
#ifdef CONFIG_MIPS_MT_SMTC
	_ehb
	andi	t0, t0, VPECONTROL_TE
	beqz	t0, 1f
	emt
1:
	mfc0	t1, CP0_TCSTATUS
	xori	t1, t1, TCSTATUS_IXMT
	or	t1, t1, t2
	mtc0	t1, CP0_TCSTATUS
	_ehb
#endif /* CONFIG_MIPS_MT_SMTC */
	move	v0, a0
	jr	ra
	END(resume)

LEAF(_save_fp)
#ifdef CONFIG_64BIT
	mfc0	t0, CP0_STATUS
#endif
	fpu_save_double a0 t0 t1		# clobbers t1
	jr	ra
	END(_save_fp)

LEAF(_restore_fp)
#ifdef CONFIG_64BIT
	mfc0	t0, CP0_STATUS
#endif
	fpu_restore_double a0 t0 t1		# clobbers t1
	jr	ra
	END(_restore_fp)


#define FPU_DEFAULT  0x00000000

LEAF(_init_fpu)
#ifdef CONFIG_MIPS_MT_SMTC
	/* Rather than manipulate per-VPE Status, set per-TC bit in TCStatus */
	mfc0	t0, CP0_TCSTATUS
	/* Bit position is the same for Status, TCStatus */
	li	t1, ST0_CU1
	or	t0, t1
	mtc0	t0, CP0_TCSTATUS
#else /* Normal MIPS CU1 enable */
	mfc0	t0, CP0_STATUS
	li	t1, ST0_CU1
	or	t0, t1
	mtc0	t0, CP0_STATUS
#endif /* CONFIG_MIPS_MT_SMTC */
	enable_fpu_hazard

	li	t1, FPU_DEFAULT
	ctc1	t1, fcr31

	li	t1, -1				# SNaN

#ifdef CONFIG_64BIT
	sll	t0, t0, 5
	bgez	t0, 1f				# 16 / 32 register mode?

	dmtc1	t1, $f1
	dmtc1	t1, $f3
	dmtc1	t1, $f5
	dmtc1	t1, $f7
	dmtc1	t1, $f9
	dmtc1	t1, $f11
	dmtc1	t1, $f13
	dmtc1	t1, $f15
	dmtc1	t1, $f17
	dmtc1	t1, $f19
	dmtc1	t1, $f21
	dmtc1	t1, $f23
	dmtc1	t1, $f25
	dmtc1	t1, $f27
	dmtc1	t1, $f29
	dmtc1	t1, $f31
1:
#endif

#ifdef CONFIG_CPU_MIPS32
	mtc1	t1, $f0
	mtc1	t1, $f1
	mtc1	t1, $f2
	mtc1	t1, $f3
	mtc1	t1, $f4
	mtc1	t1, $f5
	mtc1	t1, $f6
	mtc1	t1, $f7
	mtc1	t1, $f8
	mtc1	t1, $f9
	mtc1	t1, $f10
	mtc1	t1, $f11
	mtc1	t1, $f12
	mtc1	t1, $f13
	mtc1	t1, $f14
	mtc1	t1, $f15
	mtc1	t1, $f16
	mtc1	t1, $f17
	mtc1	t1, $f18
	mtc1	t1, $f19
	mtc1	t1, $f20
	mtc1	t1, $f21
	mtc1	t1, $f22
	mtc1	t1, $f23
	mtc1	t1, $f24
	mtc1	t1, $f25
	mtc1	t1, $f26
	mtc1	t1, $f27
	mtc1	t1, $f28
	mtc1	t1, $f29
	mtc1	t1, $f30
	mtc1	t1, $f31
#else
	.set	mips3
	dmtc1	t1, $f0
	dmtc1	t1, $f2
	dmtc1	t1, $f4
	dmtc1	t1, $f6
	dmtc1	t1, $f8
	dmtc1	t1, $f10
	dmtc1	t1, $f12
	dmtc1	t1, $f14
	dmtc1	t1, $f16
	dmtc1	t1, $f18
	dmtc1	t1, $f20
	dmtc1	t1, $f22
	dmtc1	t1, $f24
	dmtc1	t1, $f26
	dmtc1	t1, $f28
	dmtc1	t1, $f30
#endif
	jr	ra
	END(_init_fpu)

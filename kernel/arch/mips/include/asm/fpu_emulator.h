
#ifndef _ASM_FPU_EMULATOR_H
#define _ASM_FPU_EMULATOR_H

#include <asm/break.h>
#include <asm/inst.h>
#include <asm/local.h>

#ifdef CONFIG_DEBUG_FS

struct mips_fpu_emulator_stats {
	local_t emulated;
	local_t loads;
	local_t stores;
	local_t cp1ops;
	local_t cp1xops;
	local_t errors;
};

DECLARE_PER_CPU(struct mips_fpu_emulator_stats, fpuemustats);

#define MIPS_FPU_EMU_INC_STATS(M)					\
do {									\
	preempt_disable();						\
	__local_inc(&__get_cpu_var(fpuemustats).M);			\
	preempt_enable();						\
} while (0)

#else
#define MIPS_FPU_EMU_INC_STATS(M) do { } while (0)
#endif /* CONFIG_DEBUG_FS */

extern int mips_dsemul(struct pt_regs *regs, mips_instruction ir,
	unsigned long cpc);
extern int do_dsemulret(struct pt_regs *xcp);

#define BD_COOKIE 0x0000bd36	/* tne $0, $0 with baggage */

#define BREAK_MATH (0x0000000d | (BRK_MEMU << 16))

#endif /* _ASM_FPU_EMULATOR_H */

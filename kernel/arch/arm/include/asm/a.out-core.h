

#ifndef _ASM_A_OUT_CORE_H
#define _ASM_A_OUT_CORE_H

#ifdef __KERNEL__

#include <linux/user.h>
#include <linux/elfcore.h>

static inline void aout_dump_thread(struct pt_regs *regs, struct user *dump)
{
	struct task_struct *tsk = current;

	dump->magic = CMAGIC;
	dump->start_code = tsk->mm->start_code;
	dump->start_stack = regs->ARM_sp & ~(PAGE_SIZE - 1);

	dump->u_tsize = (tsk->mm->end_code - tsk->mm->start_code) >> PAGE_SHIFT;
	dump->u_dsize = (tsk->mm->brk - tsk->mm->start_data + PAGE_SIZE - 1) >> PAGE_SHIFT;
	dump->u_ssize = 0;

	dump->u_debugreg[0] = tsk->thread.debug.bp[0].address;
	dump->u_debugreg[1] = tsk->thread.debug.bp[1].address;
	dump->u_debugreg[2] = tsk->thread.debug.bp[0].insn.arm;
	dump->u_debugreg[3] = tsk->thread.debug.bp[1].insn.arm;
	dump->u_debugreg[4] = tsk->thread.debug.nsaved;

	if (dump->start_stack < 0x04000000)
		dump->u_ssize = (0x04000000 - dump->start_stack) >> PAGE_SHIFT;

	dump->regs = *regs;
	dump->u_fpvalid = dump_fpu (regs, &dump->u_fp);
}

#endif /* __KERNEL__ */
#endif /* _ASM_A_OUT_CORE_H */

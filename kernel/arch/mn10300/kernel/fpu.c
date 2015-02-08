
#include <asm/uaccess.h>
#include <asm/fpu.h>
#include <asm/elf.h>
#include <asm/exceptions.h>

struct task_struct *fpu_state_owner;

asmlinkage void fpu_disabled(struct pt_regs *regs, enum exception_code code)
{
	struct task_struct *tsk = current;

	if (!user_mode(regs))
		die_if_no_fixup("An FPU Disabled exception happened in"
				" kernel space\n",
				regs, code);

#ifdef CONFIG_FPU
	preempt_disable();

	/* transfer the last process's FPU state to memory */
	if (fpu_state_owner) {
		fpu_save(&fpu_state_owner->thread.fpu_state);
		fpu_state_owner->thread.uregs->epsw &= ~EPSW_FE;
	}

	/* the current process now owns the FPU state */
	fpu_state_owner = tsk;
	regs->epsw |= EPSW_FE;

	/* load the FPU with the current process's FPU state or invent a new
	 * clean one if the process doesn't have one */
	if (is_using_fpu(tsk)) {
		fpu_restore(&tsk->thread.fpu_state);
	} else {
		fpu_init_state();
		set_using_fpu(tsk);
	}

	preempt_enable();
#else
	{
		siginfo_t info;

		info.si_signo = SIGFPE;
		info.si_errno = 0;
		info.si_addr = (void *) tsk->thread.uregs->pc;
		info.si_code = FPE_FLTINV;

		force_sig_info(SIGFPE, &info, tsk);
	}
#endif  /* CONFIG_FPU */
}

asmlinkage void fpu_exception(struct pt_regs *regs, enum exception_code code)
{
	struct task_struct *tsk = fpu_state_owner;
	siginfo_t info;

	if (!user_mode(regs))
		die_if_no_fixup("An FPU Operation exception happened in"
				" kernel space\n",
				regs, code);

	if (!tsk)
		die_if_no_fixup("An FPU Operation exception happened,"
				" but the FPU is not in use",
				regs, code);

	info.si_signo = SIGFPE;
	info.si_errno = 0;
	info.si_addr = (void *) tsk->thread.uregs->pc;
	info.si_code = FPE_FLTINV;

#ifdef CONFIG_FPU
	{
		u32 fpcr;

		/* get FPCR (we need to enable the FPU whilst we do this) */
		asm volatile("	or	%1,epsw		\n"
#ifdef CONFIG_MN10300_PROC_MN103E010
			     "	nop			\n"
			     "	nop			\n"
			     "	nop			\n"
#endif
			     "	fmov	fpcr,%0		\n"
#ifdef CONFIG_MN10300_PROC_MN103E010
			     "	nop			\n"
			     "	nop			\n"
			     "	nop			\n"
#endif
			     "	and	%2,epsw		\n"
			     : "=&d"(fpcr)
			     : "i"(EPSW_FE), "i"(~EPSW_FE)
			     );

		if (fpcr & FPCR_EC_Z)
			info.si_code = FPE_FLTDIV;
		else if	(fpcr & FPCR_EC_O)
			info.si_code = FPE_FLTOVF;
		else if	(fpcr & FPCR_EC_U)
			info.si_code = FPE_FLTUND;
		else if	(fpcr & FPCR_EC_I)
			info.si_code = FPE_FLTRES;
	}
#endif

	force_sig_info(SIGFPE, &info, tsk);
}

int fpu_setup_sigcontext(struct fpucontext *fpucontext)
{
#ifdef CONFIG_FPU
	struct task_struct *tsk = current;

	if (!is_using_fpu(tsk))
		return 0;

	/* transfer the current FPU state to memory and cause fpu_init() to be
	 * triggered by the next attempted FPU operation by the current
	 * process.
	 */
	preempt_disable();

	if (fpu_state_owner == tsk) {
		fpu_save(&tsk->thread.fpu_state);
		fpu_state_owner->thread.uregs->epsw &= ~EPSW_FE;
		fpu_state_owner = NULL;
	}

	preempt_enable();

	/* we no longer have a valid current FPU state */
	clear_using_fpu(tsk);

	/* transfer the saved FPU state onto the userspace stack */
	if (copy_to_user(fpucontext,
			 &tsk->thread.fpu_state,
			 min(sizeof(struct fpu_state_struct),
			     sizeof(struct fpucontext))))
		return -1;

	return 1;
#else
	return 0;
#endif
}

void fpu_kill_state(struct task_struct *tsk)
{
#ifdef CONFIG_FPU
	/* disown anything left in the FPU */
	preempt_disable();

	if (fpu_state_owner == tsk) {
		fpu_state_owner->thread.uregs->epsw &= ~EPSW_FE;
		fpu_state_owner = NULL;
	}

	preempt_enable();
#endif
	/* we no longer have a valid current FPU state */
	clear_using_fpu(tsk);
}

int fpu_restore_sigcontext(struct fpucontext *fpucontext)
{
	struct task_struct *tsk = current;
	int ret;

	/* load up the old FPU state */
	ret = copy_from_user(&tsk->thread.fpu_state,
			     fpucontext,
			     min(sizeof(struct fpu_state_struct),
				 sizeof(struct fpucontext)));
	if (!ret)
		set_using_fpu(tsk);

	return ret;
}

int dump_fpu(struct pt_regs *regs, elf_fpregset_t *fpreg)
{
	struct task_struct *tsk = current;
	int fpvalid;

	fpvalid = is_using_fpu(tsk);
	if (fpvalid) {
		unlazy_fpu(tsk);
		memcpy(fpreg, &tsk->thread.fpu_state, sizeof(*fpreg));
	}

	return fpvalid;
}

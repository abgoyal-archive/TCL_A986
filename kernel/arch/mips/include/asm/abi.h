
#ifndef _ASM_ABI_H
#define _ASM_ABI_H

#include <asm/signal.h>
#include <asm/siginfo.h>

struct mips_abi {
	int (* const setup_frame)(void *sig_return, struct k_sigaction *ka,
	                          struct pt_regs *regs, int signr,
	                          sigset_t *set);
	const unsigned long	signal_return_offset;
	int (* const setup_rt_frame)(void *sig_return, struct k_sigaction *ka,
	                       struct pt_regs *regs, int signr,
	                       sigset_t *set, siginfo_t *info);
	const unsigned long	rt_signal_return_offset;
	const unsigned long	restart;
};

#endif /* _ASM_ABI_H */

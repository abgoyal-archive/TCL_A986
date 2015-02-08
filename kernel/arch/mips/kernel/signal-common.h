

#ifndef __SIGNAL_COMMON_H
#define __SIGNAL_COMMON_H

/* #define DEBUG_SIG */

#ifdef DEBUG_SIG
#  define DEBUGP(fmt, args...) printk("%s: " fmt, __func__, ##args)
#else
#  define DEBUGP(fmt, args...)
#endif

#define _BLOCKABLE (~(sigmask(SIGKILL) | sigmask(SIGSTOP)))

extern void __user *get_sigframe(struct k_sigaction *ka, struct pt_regs *regs,
				 size_t frame_size);
/* Check and clear pending FPU exceptions in saved CSR */
extern int fpcsr_pending(unsigned int __user *fpcsr);

/* Make sure we will not lose FPU ownership */
#ifdef CONFIG_PREEMPT
#define lock_fpu_owner()	preempt_disable()
#define unlock_fpu_owner()	preempt_enable()
#else
#define lock_fpu_owner()	pagefault_disable()
#define unlock_fpu_owner()	pagefault_enable()
#endif

#endif	/* __SIGNAL_COMMON_H */

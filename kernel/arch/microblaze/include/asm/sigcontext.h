

#ifndef _ASM_MICROBLAZE_SIGCONTEXT_H
#define _ASM_MICROBLAZE_SIGCONTEXT_H

/* FIXME should be linux/ptrace.h */
#include <asm/ptrace.h>

struct sigcontext {
	struct pt_regs regs;
	unsigned long oldmask;
};

#endif /* _ASM_MICROBLAZE_SIGCONTEXT_H */


#ifndef _ASM_SIGCONTEXT_H
#define _ASM_SIGCONTEXT_H

#include <asm/registers.h>

struct sigcontext {
	struct user_context	sc_context;
	unsigned long		sc_oldmask; 	/* old sigmask */
} __attribute__((aligned(8)));

#endif

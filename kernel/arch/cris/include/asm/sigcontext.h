
/* $Id: sigcontext.h,v 1.1 2000/07/10 16:32:31 bjornw Exp $ */

#ifndef _ASM_CRIS_SIGCONTEXT_H
#define _ASM_CRIS_SIGCONTEXT_H

#include <asm/ptrace.h>


struct sigcontext {
	struct pt_regs regs;  /* needs to be first */
	unsigned long oldmask;
	unsigned long usp;    /* usp before stacking this gunk on it */
};

#endif




#ifndef _ASM_DELAY_H
#define _ASM_DELAY_H

#include <asm/param.h>
#include <asm/timer-regs.h>

extern unsigned long __delay_loops_MHz;

static inline void __delay(unsigned long loops)
{
	asm volatile("1:	subicc	%0,#1,%0,icc0	\n"
		     "		bnc	icc0,#2,1b	\n"
		     : "=r" (loops)
		     : "0" (loops)
		     : "icc0"
		     );
}


extern unsigned long loops_per_jiffy;

static inline void udelay(unsigned long usecs)
{
	__delay(usecs * __delay_loops_MHz);
}

#define ndelay(n)	udelay((n) * 5)

#endif /* _ASM_DELAY_H */

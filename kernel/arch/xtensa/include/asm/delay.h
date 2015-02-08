

#ifndef _XTENSA_DELAY_H
#define _XTENSA_DELAY_H

#include <asm/processor.h>
#include <asm/param.h>

extern unsigned long loops_per_jiffy;

static inline void __delay(unsigned long loops)
{
  /* 2 cycles per loop. */
  __asm__ __volatile__ ("1: addi %0, %0, -2; bgeui %0, 2, 1b"
			: "=r" (loops) : "0" (loops));
}

static __inline__ u32 xtensa_get_ccount(void)
{
	u32 ccount;
	asm volatile ("rsr %0, 234; # CCOUNT\n" : "=r" (ccount));
	return ccount;
}


static __inline__ void udelay (unsigned long usecs)
{
	unsigned long start = xtensa_get_ccount();
	unsigned long cycles = usecs * (loops_per_jiffy / (1000000UL / HZ));

	/* Note: all variables are unsigned (can wrap around)! */
	while (((unsigned long)xtensa_get_ccount()) - start < cycles)
		;
}

#endif


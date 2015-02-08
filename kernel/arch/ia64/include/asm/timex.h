
#ifndef _ASM_IA64_TIMEX_H
#define _ASM_IA64_TIMEX_H


#include <asm/intrinsics.h>
#include <asm/processor.h>

typedef unsigned long cycles_t;

extern void (*ia64_udelay)(unsigned long usecs);

#define CLOCK_TICK_RATE		(HZ * 100000UL)

static inline cycles_t
get_cycles (void)
{
	cycles_t ret;

	ret = ia64_getreg(_IA64_REG_AR_ITC);
	return ret;
}

extern void ia64_cpu_local_tick (void);
extern unsigned long long ia64_native_sched_clock (void);

#endif /* _ASM_IA64_TIMEX_H */

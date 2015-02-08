
#ifndef _ASM_TIMEX_H
#define _ASM_TIMEX_H

#include <asm/hardirq.h>
#include <unit/timex.h>

#define TICK_SIZE (tick_nsec / 1000)

#define CLOCK_TICK_RATE 1193180 /* Underlying HZ - this should probably be set
				 * to something appropriate, but what? */

extern cycles_t cacheflush_time;

#ifdef __KERNEL__

static inline cycles_t get_cycles(void)
{
	return read_timestamp_counter();
}

#endif /* __KERNEL__ */

#endif /* _ASM_TIMEX_H */

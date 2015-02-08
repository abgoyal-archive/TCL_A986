
#ifndef _ASM_TIMEX_H
#define _ASM_TIMEX_H

#ifdef __KERNEL__

#include <asm/mipsregs.h>

#define CLOCK_TICK_RATE 1193182

extern unsigned int mips_hpt_frequency;


typedef unsigned int cycles_t;

#ifdef CONFIG_HAVE_GET_CYCLES_32
static inline cycles_t get_cycles(void)
{
	return read_c0_count();
}

static inline void get_cycles_barrier(void)
{
}

static inline cycles_t get_cycles_rate(void)
{
	return mips_hpt_frequency;
}

extern int test_tsc_synchronization(void);
extern int _tsc_is_sync;
static inline int tsc_is_sync(void)
{
	return _tsc_is_sync;
}
#else
static inline cycles_t get_cycles(void)
{
	return 0;
}
static inline int test_tsc_synchronization(void)
{
	return 0;
}
static inline int tsc_is_sync(void)
{
	return 0;
}
#endif

#define DELAY_INTERRUPT 100
static inline void write_tsc(u32 val1, u32 val2)
{
	write_c0_count(val1);
	/* Arrange for an interrupt in a short while */
	write_c0_compare(read_c0_count() + DELAY_INTERRUPT);
}

static inline void mark_tsc_unstable(char *reason)
{
}

static inline int unsynchronized_tsc(void)
{
	return !tsc_is_sync();
}

#endif /* __KERNEL__ */

#endif /*  _ASM_TIMEX_H */

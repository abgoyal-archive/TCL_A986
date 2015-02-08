

#ifndef _ASM_SH_TRACE_CLOCK_H
#define _ASM_SH_TRACE_CLOCK_H

#include <linux/clocksource.h>
#include <asm/clock.h>

#define TC_HW_BITS			32

/* Expected maximum interrupt latency in ms : 15ms, *2 for security */
#define TC_EXPECTED_INTERRUPT_LATENCY	30

extern u64 trace_clock_read_synthetic_tsc(void);
extern u64 sh_get_clock_frequency(void);
extern u32 sh_read_timer_count(void);
extern void get_synthetic_tsc(void);
extern void put_synthetic_tsc(void);

static inline u32 trace_clock_read32(void)
{
	return sh_read_timer_count();
}

static inline u64 trace_clock_read64(void)
{
	return trace_clock_read_synthetic_tsc();
}

static inline u64 trace_clock_frequency(void)
{
	return sh_get_clock_frequency();
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}

static inline void get_trace_clock(void)
{
	get_synthetic_tsc();
}

static inline void put_trace_clock(void)
{
	put_synthetic_tsc();
}

static inline void set_trace_clock_is_sync(int state)
{
}
#endif /* _ASM_SH_TRACE_CLOCK_H */

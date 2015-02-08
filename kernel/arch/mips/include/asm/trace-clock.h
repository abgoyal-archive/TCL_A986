

#ifndef _ASM_MIPS_TRACE_CLOCK_H
#define _ASM_MIPS_TRACE_CLOCK_H

#include <linux/timex.h>
#include <asm/processor.h>

#define TRACE_CLOCK_MIN_PROBE_DURATION 200

#ifdef CONFIG_CPU_CAVIUM_OCTEON
# include <asm/octeon/trace-clock.h>
#else /* !CONFIG_CPU_CAVIUM_OCTEON */
#define TC_HW_BITS			32

/* Expected maximum interrupt latency in ms : 15ms, *2 for security */
#define TC_EXPECTED_INTERRUPT_LATENCY	30

extern u64 trace_clock_read_synthetic_tsc(void);


static inline u32 trace_clock_read32(void)
{
	return (u32)get_cycles(); /* only need the 32 LSB */
}

static inline u64 trace_clock_read64(void)
{
	return trace_clock_read_synthetic_tsc();
}

static inline u64 trace_clock_frequency(void)
{
	return get_cycles_rate();
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}

extern void get_synthetic_tsc(void);
extern void put_synthetic_tsc(void);

static inline void get_trace_clock(void)
{
	get_synthetic_tsc();
}

static inline void put_trace_clock(void)
{
	put_synthetic_tsc();
}
#endif /* CONFIG_CPU_CAVIUM_OCTEON */

static inline void set_trace_clock_is_sync(int state)
{
}
#endif /* _ASM_MIPS_TRACE_CLOCK_H */

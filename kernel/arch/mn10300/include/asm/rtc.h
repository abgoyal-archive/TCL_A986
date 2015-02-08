
#ifndef _ASM_RTC_H
#define _ASM_RTC_H

#ifdef CONFIG_MN10300_RTC

#include <linux/init.h>

extern void check_rtc_time(void);
extern void __init calibrate_clock(void);
extern unsigned long __init get_initial_rtc_time(void);

#else /* !CONFIG_MN10300_RTC */

static inline void check_rtc_time(void)
{
}

static inline void calibrate_clock(void)
{
}

static inline unsigned long get_initial_rtc_time(void)
{
	return 0;
}

#endif /* !CONFIG_MN10300_RTC */

#include <asm-generic/rtc.h>

#endif /* _ASM_RTC_H */

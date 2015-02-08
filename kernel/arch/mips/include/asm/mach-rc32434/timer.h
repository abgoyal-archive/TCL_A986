

#ifndef __ASM_RC32434_TIMER_H
#define __ASM_RC32434_TIMER_H

#include <asm/mach-rc32434/rb.h>

#define TIMER0_BASE_ADDR		0x18028000
#define TIMER_COUNT			3

struct timer_counter {
	u32 count;
	u32 compare;
	u32 ctc;		/*use CTC_ */
};

struct timer {
	struct timer_counter tim[TIMER_COUNT];
	u32 rcount;	/* use RCOUNT_ */
	u32 rcompare;	/* use RCOMPARE_ */
	u32 rtc;	/* use RTC_ */
};

#define RC32434_CTC_EN_BIT		0
#define RC32434_CTC_TO_BIT		1

/* Real time clock registers */
#define RC32434_RTC_MSK(x)              BIT_TO_MASK(x)
#define RC32434_RTC_CE_BIT              0
#define RC32434_RTC_TO_BIT              1
#define RC32434_RTC_RQE_BIT             2

/* Counter registers */
#define RC32434_RCOUNT_BIT              0
#define RC32434_RCOUNT_MSK              0x0000ffff
#define RC32434_RCOMP_BIT               0
#define RC32434_RCOMP_MSK               0x0000ffff

#endif  /* __ASM_RC32434_TIMER_H */

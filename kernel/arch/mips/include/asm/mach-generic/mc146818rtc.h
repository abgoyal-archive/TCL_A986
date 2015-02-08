
#ifndef __ASM_MACH_GENERIC_MC146818RTC_H
#define __ASM_MACH_GENERIC_MC146818RTC_H

#include <asm/io.h>

#define RTC_PORT(x)	(0x70 + (x))
#define RTC_IRQ		8

static inline unsigned char CMOS_READ(unsigned long addr)
{
	outb_p(addr, RTC_PORT(0));
	return inb_p(RTC_PORT(1));
}

static inline void CMOS_WRITE(unsigned char data, unsigned long addr)
{
	outb_p(addr, RTC_PORT(0));
	outb_p(data, RTC_PORT(1));
}

#define RTC_ALWAYS_BCD	1

#ifndef mc146818_decode_year
#define mc146818_decode_year(year) ((year) < 70 ? (year) + 2000 : (year) + 1900)
#endif

#endif /* __ASM_MACH_GENERIC_MC146818RTC_H */

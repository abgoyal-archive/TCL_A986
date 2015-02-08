
#ifndef __ASM_MACH_IP32_MC146818RTC_H
#define __ASM_MACH_IP32_MC146818RTC_H

#include <asm/ip32/mace.h>

#define RTC_PORT(x)	(0x70 + (x))

static unsigned char CMOS_READ(unsigned long addr)
{
	return mace->isa.rtc[addr << 8];
}

static inline void CMOS_WRITE(unsigned char data, unsigned long addr)
{
	mace->isa.rtc[addr << 8] = data;
}

#define mc146818_decode_year(year) ((year) + 2000)

#define RTC_ALWAYS_BCD	0

#endif /* __ASM_MACH_IP32_MC146818RTC_H */

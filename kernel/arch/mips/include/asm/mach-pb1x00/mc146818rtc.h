
#ifndef __ASM_MACH_AU1XX_MC146818RTC_H
#define __ASM_MACH_AU1XX_MC146818RTC_H

#include <asm/io.h>
#include <asm/mach-au1x00/au1000.h>

#define RTC_PORT(x)	(0x0c000000 + (x))
#define RTC_IRQ		8
#define PB1500_RTC_ADDR 0x0c000000

static inline unsigned char CMOS_READ(unsigned long offset)
{
	offset <<= 2;
	return (u8)(au_readl(offset + PB1500_RTC_ADDR) & 0xff);
}

static inline void CMOS_WRITE(unsigned char data, unsigned long offset)
{
	offset <<= 2;
	au_writel(data, offset + PB1500_RTC_ADDR);
}

#define RTC_ALWAYS_BCD	1

#endif /* __ASM_MACH_AU1XX_MC146818RTC_H */


#ifndef __ASM_MIPS_DEC_RTC_DEC_H
#define __ASM_MIPS_DEC_RTC_DEC_H

#include <linux/types.h>
#include <asm/addrspace.h>
#include <asm/dec/system.h>

extern volatile u8 *dec_rtc_base;

#define ARCH_RTC_LOCATION

#define RTC_PORT(x)	CPHYSADDR((long)dec_rtc_base)
#define RTC_IO_EXTENT	dec_kn_slot_size
#define RTC_IOMAPPED	0
#undef RTC_IRQ

#define RTC_DEC_YEAR	0x3f	/* Where we store the real year on DECs.  */

static inline unsigned char CMOS_READ(unsigned long addr)
{
	return dec_rtc_base[addr * 4];
}

static inline void CMOS_WRITE(unsigned char data, unsigned long addr)
{
	dec_rtc_base[addr * 4] = data;
}

#define RTC_ALWAYS_BCD	0

#endif /* __ASM_MIPS_DEC_RTC_DEC_H */

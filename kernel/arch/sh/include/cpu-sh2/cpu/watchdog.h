
#ifndef __ASM_CPU_SH2_WATCHDOG_H
#define __ASM_CPU_SH2_WATCHDOG_H

/* Register definitions */
#define WTCNT		0xfffffe80
#define WTCSR		0xfffffe80
#define RSTCSR		0xfffffe82

#define WTCNT_R		(WTCNT + 1)
#define RSTCSR_R	(RSTCSR + 1)

/* Bit definitions */
#define WTCSR_IOVF	0x80
#define WTCSR_WT	0x40
#define WTCSR_TME	0x20
#define WTCSR_RSTS	0x00

#define RSTCSR_RSTS	0x20

static inline __u8 sh_wdt_read_rstcsr(void)
{
	/*
	 * Same read/write brain-damage as for WTCNT here..
	 */
	return __raw_readb(RSTCSR_R);
}

static inline void sh_wdt_write_rstcsr(__u8 val)
{
	/*
	 * Note: Due to the brain-damaged nature of this register,
	 * we can't presently touch the WOVF bit, since the upper byte
	 * has to be swapped for this. So just leave it alone..
	 */
	__raw_writeb((WTCNT_HIGH << 8) | (__u16)val, RSTCSR);
}

#endif /* __ASM_CPU_SH2_WATCHDOG_H */


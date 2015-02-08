

#ifndef __ASM_ARCH_TICK_H
#define __ASM_ARCH_TICK_H __FILE__

static inline u32 s3c24xx_ostimer_pending(void)
{
	u32 pend = __raw_readl(VA_VIC0 + VIC_RAW_STATUS);
	return pend & 1 << (IRQ_TIMER4_VIC - S3C64XX_IRQ_VIC0(0));
}

#define TICK_MAX	(0xffffffff)

#endif /* __ASM_ARCH_6400_TICK_H */

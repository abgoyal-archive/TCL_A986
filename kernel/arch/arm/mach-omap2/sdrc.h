
#ifndef __ARCH_ARM_MACH_OMAP2_SDRC_H
#define __ARCH_ARM_MACH_OMAP2_SDRC_H

#undef DEBUG

#include <plat/sdrc.h>

#ifndef __ASSEMBLER__

#include <linux/io.h>

extern void __iomem *omap2_sdrc_base;
extern void __iomem *omap2_sms_base;

#define OMAP_SDRC_REGADDR(reg)			(omap2_sdrc_base + (reg))
#define OMAP_SMS_REGADDR(reg)			(omap2_sms_base + (reg))

/* SDRC global register get/set */

static inline void sdrc_write_reg(u32 val, u16 reg)
{
	__raw_writel(val, OMAP_SDRC_REGADDR(reg));
}

static inline u32 sdrc_read_reg(u16 reg)
{
	return __raw_readl(OMAP_SDRC_REGADDR(reg));
}

/* SMS global register get/set */

static inline void sms_write_reg(u32 val, u16 reg)
{
	__raw_writel(val, OMAP_SMS_REGADDR(reg));
}

static inline u32 sms_read_reg(u16 reg)
{
	return __raw_readl(OMAP_SMS_REGADDR(reg));
}
#else
#define OMAP242X_SDRC_REGADDR(reg)					\
			OMAP2_L3_IO_ADDRESS(OMAP2420_SDRC_BASE + (reg))
#define OMAP243X_SDRC_REGADDR(reg)					\
			OMAP2_L3_IO_ADDRESS(OMAP243X_SDRC_BASE + (reg))
#define OMAP34XX_SDRC_REGADDR(reg)					\
			OMAP2_L3_IO_ADDRESS(OMAP343X_SDRC_BASE + (reg))
#endif	/* __ASSEMBLER__ */

/* Minimum frequency that the SDRC DLL can lock at */
#define MIN_SDRC_DLL_LOCK_FREQ		83000000

/* Scale factor for fixed-point arith in omap3_core_dpll_m2_set_rate() */
#define SDRC_MPURATE_SCALE		8

/* 2^SDRC_MPURATE_BASE_SHIFT: MPU MHz that SDRC_MPURATE_LOOPS is defined for */
#define SDRC_MPURATE_BASE_SHIFT		9

#define SDRC_MPURATE_LOOPS		96


#endif

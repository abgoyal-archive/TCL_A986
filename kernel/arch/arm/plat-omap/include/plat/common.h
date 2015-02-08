

#ifndef __ARCH_ARM_MACH_OMAP_COMMON_H
#define __ARCH_ARM_MACH_OMAP_COMMON_H

#include <plat/i2c.h>

struct sys_timer;

extern void omap_map_common_io(void);
extern struct sys_timer omap_timer;

struct omap_globals {
	u32		class;		/* OMAP class to detect */
	void __iomem	*tap;		/* Control module ID code */
	unsigned long   sdrc;           /* SDRAM Controller */
	unsigned long   sms;            /* SDRAM Memory Scheduler */
	unsigned long   ctrl;           /* System Control Module */
	unsigned long   prm;            /* Power and Reset Management */
	unsigned long   cm;             /* Clock Management */
	unsigned long   cm2;
	unsigned long	uart1_phys;
	unsigned long	uart2_phys;
	unsigned long	uart3_phys;
	unsigned long	uart4_phys;
};

void omap2_set_globals_242x(void);
void omap2_set_globals_243x(void);
void omap2_set_globals_343x(void);
void omap2_set_globals_36xx(void);
void omap2_set_globals_443x(void);

/* These get called from omap2_set_globals_xxxx(), do not call these */
void omap2_set_globals_tap(struct omap_globals *);
void omap2_set_globals_sdrc(struct omap_globals *);
void omap2_set_globals_control(struct omap_globals *);
void omap2_set_globals_prcm(struct omap_globals *);
void omap2_set_globals_uart(struct omap_globals *);

#define omap_test_timeout(cond, timeout, index)			\
({								\
	for (index = 0; index < timeout; index++) {		\
		if (cond)					\
			break;					\
		udelay(1);					\
	}							\
})

#endif /* __ARCH_ARM_MACH_OMAP_COMMON_H */

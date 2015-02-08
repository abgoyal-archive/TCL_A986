
#ifndef __ASM_ARCH_SETUP_H
#define __ASM_ARCH_SETUP_H

#include <asm/mach/time.h>
#include <linux/init.h>

extern void __init ux500_map_io(void);
extern void __init u5500_map_io(void);
extern void __init u8500_map_io(void);

extern void __init ux500_init_devices(void);
extern void __init u5500_init_devices(void);
extern void __init u8500_init_devices(void);

extern void __init ux500_init_irq(void);
/* We re-use nomadik_timer for this platform */
extern void nmdk_timer_init(void);

extern void __init amba_add_devices(struct amba_device *devs[], int num);

struct sys_timer;
extern struct sys_timer ux500_timer;

#define __IO_DEV_DESC(x, sz)	{		\
	.virtual	= IO_ADDRESS(x),	\
	.pfn		= __phys_to_pfn(x),	\
	.length		= sz,			\
	.type		= MT_DEVICE,		\
}

#endif /*  __ASM_ARCH_SETUP_H */

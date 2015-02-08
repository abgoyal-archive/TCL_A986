

#ifndef __ASM_ARCH_MACH_REGS_FB_H
#define __ASM_ARCH_MACH_REGS_FB_H __FILE__

#include <plat/regs-fb-v4.h>

static inline unsigned int s3c_fb_pal_reg(unsigned int window, int reg)
{
	return 0x2400 + (window * 256 *4 ) + reg;
}

#endif /* __ASM_ARCH_MACH_REGS_FB_H */

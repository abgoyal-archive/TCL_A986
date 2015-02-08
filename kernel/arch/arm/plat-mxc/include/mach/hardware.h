

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#define __ASM_ARCH_MXC_HARDWARE_H__

#include <asm/sizes.h>

#define IMX_IO_ADDRESS(addr, module)					\
	((void __force __iomem *)					\
	 (((unsigned long)((addr) - (module ## _BASE_ADDR)) < module ## _SIZE) ?\
	 (addr) - (module ## _BASE_ADDR) + (module ## _BASE_ADDR_VIRT) : 0))

#ifdef CONFIG_ARCH_MX5
#include <mach/mx51.h>
#endif

#ifdef CONFIG_ARCH_MX3
#include <mach/mx3x.h>
#include <mach/mx31.h>
#include <mach/mx35.h>
#endif

#ifdef CONFIG_ARCH_MX2
# include <mach/mx2x.h>
# ifdef CONFIG_MACH_MX21
#  include <mach/mx21.h>
# endif
# ifdef CONFIG_MACH_MX27
#  include <mach/mx27.h>
# endif
#endif

#ifdef CONFIG_ARCH_MX1
# include <mach/mx1.h>
#endif

#ifdef CONFIG_ARCH_MX25
# include <mach/mx25.h>
#endif

#ifdef CONFIG_ARCH_MXC91231
# include <mach/mxc91231.h>
#endif

#include <mach/mxc.h>

#endif /* __ASM_ARCH_MXC_HARDWARE_H__ */

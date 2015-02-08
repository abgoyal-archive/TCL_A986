

#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

#include "orion5x.h"

#define IO_SPACE_LIMIT		0xffffffff

static inline void __iomem *
__arch_ioremap(unsigned long paddr, size_t size, unsigned int mtype)
{
	void __iomem *retval;
	unsigned long offs = paddr - ORION5X_REGS_PHYS_BASE;
	if (mtype == MT_DEVICE && size && offs < ORION5X_REGS_SIZE &&
	    size <= ORION5X_REGS_SIZE && offs + size <= ORION5X_REGS_SIZE) {
		retval = (void __iomem *)ORION5X_REGS_VIRT_BASE + offs;
	} else {
		retval = __arm_ioremap(paddr, size, mtype);
	}

	return retval;
}

static inline void
__arch_iounmap(void __iomem *addr)
{
	if (addr < (void __iomem *)ORION5X_REGS_VIRT_BASE ||
	    addr >= (void __iomem *)(ORION5X_REGS_VIRT_BASE + ORION5X_REGS_SIZE))
		__iounmap(addr);
}

#define __arch_ioremap(p, s, m)	__arch_ioremap(p, s, m)
#define __arch_iounmap(a)	__arch_iounmap(a)
#define __io(a)			__typesafe_io(a)
#define __mem_pci(a)		(a)


#define orion5x_setbits(r, mask)	writel(readl(r) | (mask), (r))
#define orion5x_clrbits(r, mask)	writel(readl(r) & ~(mask), (r))


#endif



#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

#define IO_SPACE_LIMIT		0xFFFFFFFF

#define __io(a)		__typesafe_io(a)
#define __mem_pci(a)	(a)


#ifndef __ASSEMBLY__

static inline unsigned int at91_sys_read(unsigned int reg_offset)
{
	void __iomem *addr = (void __iomem *)AT91_VA_BASE_SYS;

	return __raw_readl(addr + reg_offset);
}

static inline void at91_sys_write(unsigned int reg_offset, unsigned long value)
{
	void __iomem *addr = (void __iomem *)AT91_VA_BASE_SYS;

	__raw_writel(value, addr + reg_offset);
}

#endif

#endif

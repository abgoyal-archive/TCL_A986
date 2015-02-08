
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#define PCIO_SIZE		0x00100000
#define PCIO_BASE		0xff000000

#define IO_SPACE_LIMIT 0xffff

#define __io(a)			((void __iomem *)(PCIO_BASE + (a)))
#if 1
#define __mem_pci(a)		(a)
#else

static inline void __iomem *___mem_pci(void __iomem *p)
{
	unsigned long a = (unsigned long)p;
	BUG_ON(a <= 0xc0000000 || a >= 0xe0000000);
	return p;
}

#define __mem_pci(a)		___mem_pci(a)
#endif

#endif

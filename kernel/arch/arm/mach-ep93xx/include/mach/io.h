

#ifndef __ASM_MACH_IO_H
#define __ASM_MACH_IO_H

#define IO_SPACE_LIMIT		0xffffffff

#define __io(p)			__typesafe_io(p)
#define __mem_pci(p)		(p)

#ifdef __ASSEMBLER__
#define IOMEM(p)		p
#else
#define IOMEM(p)		((void __iomem __force *)(p))
#endif

#endif /* __ASM_MACH_IO_H */

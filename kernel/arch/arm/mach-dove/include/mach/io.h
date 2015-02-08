

#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

#include "dove.h"

#define IO_SPACE_LIMIT		0xffffffff

#define __io(a)  ((void __iomem *)(((a) - DOVE_PCIE0_IO_PHYS_BASE) +\
				   DOVE_PCIE0_IO_VIRT_BASE))
#define __mem_pci(a)		(a)

#endif

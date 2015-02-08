
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>

#define VERSATILE_PCI_VIRT_BASE		(void __iomem *)0xe8000000ul
#define VERSATILE_PCI_CFG_VIRT_BASE	(void __iomem *)0xe9000000ul

#if 0
#define VERSATILE_PCI_VIRT_MEM_BASE0	0xf4000000
#define VERSATILE_PCI_VIRT_MEM_BASE1	0xf5000000
#define VERSATILE_PCI_VIRT_MEM_BASE2	0xf6000000

#define PCIO_BASE			VERSATILE_PCI_VIRT_MEM_BASE0
#define PCIMEM_BASE			VERSATILE_PCI_VIRT_MEM_BASE1
#endif

/* CIK guesswork */
#define PCIBIOS_MIN_IO			0x44000000
#define PCIBIOS_MIN_MEM			0x50000000

#define pcibios_assign_all_busses()     1

/* macro to get at IO space when running virtually */
#define IO_ADDRESS(x)		(((x) & 0x0fffffff) + (((x) >> 4) & 0x0f000000) + 0xf0000000)

#define __io_address(n)		__io(IO_ADDRESS(n))

#endif

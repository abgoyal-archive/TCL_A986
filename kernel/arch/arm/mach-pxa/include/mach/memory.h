

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#define PHYS_OFFSET	UL(0xa0000000)

#define NODE_MEM_SIZE_BITS	26

#if !defined(__ASSEMBLY__) && defined(CONFIG_MACH_ARMCORE) && defined(CONFIG_PCI)
void cmx2xx_pci_adjust_zones(int node, unsigned long *size,
			     unsigned long *holes);

#define arch_adjust_zones(node, size, holes) \
	cmx2xx_pci_adjust_zones(node, size, holes)

#define ISA_DMA_THRESHOLD	(PHYS_OFFSET + SZ_64M - 1)
#define MAX_DMA_ADDRESS		(PAGE_OFFSET + SZ_64M)
#endif

#endif

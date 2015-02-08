

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#include <asm/sizes.h>

#define PHYS_OFFSET	UL(0xc0000000)

#ifndef __ASSEMBLY__

#ifdef CONFIG_SA1111
void sa1111_adjust_zones(int node, unsigned long *size, unsigned long *holes);

#define arch_adjust_zones(node, size, holes) \
	sa1111_adjust_zones(node, size, holes)

#define ISA_DMA_THRESHOLD	(PHYS_OFFSET + SZ_1M - 1)
#define MAX_DMA_ADDRESS		(PAGE_OFFSET + SZ_1M)

#endif
#endif

#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	27

#define FLUSH_BASE_PHYS		0xe0000000
#define FLUSH_BASE		0xf5000000
#define FLUSH_BASE_MINICACHE	0xf5100000

#endif

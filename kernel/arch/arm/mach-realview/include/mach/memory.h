
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#ifdef CONFIG_REALVIEW_HIGH_PHYS_OFFSET
#define PHYS_OFFSET		UL(0x70000000)
#else
#define PHYS_OFFSET		UL(0x00000000)
#endif

#if !defined(__ASSEMBLY__) && defined(CONFIG_ZONE_DMA)
extern void realview_adjust_zones(int node, unsigned long *size,
				  unsigned long *hole);
#define arch_adjust_zones(node, size, hole) \
	realview_adjust_zones(node, size, hole)

#define ISA_DMA_THRESHOLD	(PHYS_OFFSET + SZ_256M - 1)
#define MAX_DMA_ADDRESS		(PAGE_OFFSET + SZ_256M)
#endif

#ifdef CONFIG_SPARSEMEM

#ifdef CONFIG_REALVIEW_HIGH_PHYS_OFFSET
#error "SPARSEMEM not available with REALVIEW_HIGH_PHYS_OFFSET"
#endif

#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	28

/* bank page offsets */
#define PAGE_OFFSET1	(PAGE_OFFSET + 0x10000000)
#define PAGE_OFFSET2	(PAGE_OFFSET + 0x30000000)

#define __phys_to_virt(phys)						\
	((phys) >= 0x80000000 ?	(phys) - 0x80000000 + PAGE_OFFSET2 :	\
	 (phys) >= 0x20000000 ?	(phys) - 0x20000000 + PAGE_OFFSET1 :	\
	 (phys) + PAGE_OFFSET)

#define __virt_to_phys(virt)						\
	 ((virt) >= PAGE_OFFSET2 ? (virt) - PAGE_OFFSET2 + 0x80000000 :	\
	  (virt) >= PAGE_OFFSET1 ? (virt) - PAGE_OFFSET1 + 0x20000000 :	\
	  (virt) - PAGE_OFFSET)

#endif	/* CONFIG_SPARSEMEM */

#endif

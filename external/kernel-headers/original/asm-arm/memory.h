#ifndef __ASM_ARM_MEMORY_H
#define __ASM_ARM_MEMORY_H

#ifndef __ASSEMBLY__
#define UL(x) (x##UL)
#else
#define UL(x) (x)
#endif

#include <linux/compiler.h>
#include <asm/arch/memory.h>
#include <asm/sizes.h>

#ifdef CONFIG_MMU

#ifndef TASK_SIZE
#define TASK_SIZE		UL(0xbf000000)
#define TASK_UNMAPPED_BASE	UL(0x40000000)
#endif

#define TASK_SIZE_26		UL(0x04000000)

#ifndef PAGE_OFFSET
#define PAGE_OFFSET		UL(0xc0000000)
#endif

#define MODULE_END		(PAGE_OFFSET)
#define MODULE_START		(MODULE_END - 16*1048576)

#if TASK_SIZE > MODULE_START
#error Top of user space clashes with start of module space
#endif

#define XIP_VIRT_ADDR(physaddr)  (MODULE_START + ((physaddr) & 0x000fffff))

#define IOREMAP_MAX_ORDER	24

#else /* CONFIG_MMU */

#ifndef TASK_SIZE
#define TASK_SIZE		(CONFIG_DRAM_SIZE)
#endif

#ifndef TASK_UNMAPPED_BASE
#define TASK_UNMAPPED_BASE	UL(0x00000000)
#endif

#ifndef PHYS_OFFSET
#define PHYS_OFFSET 		(CONFIG_DRAM_BASE)
#endif

#ifndef END_MEM
#define END_MEM     		(CONFIG_DRAM_BASE + CONFIG_DRAM_SIZE)
#endif

#ifndef PAGE_OFFSET
#define PAGE_OFFSET		(PHYS_OFFSET)
#endif

#define MODULE_END		(END_MEM)
#define MODULE_START		(PHYS_OFFSET)

#endif /* !CONFIG_MMU */

#ifndef CONSISTENT_DMA_SIZE
#define CONSISTENT_DMA_SIZE SZ_2M
#endif

#ifndef __virt_to_phys
#define __virt_to_phys(x)	((x) - PAGE_OFFSET + PHYS_OFFSET)
#define __phys_to_virt(x)	((x) - PHYS_OFFSET + PAGE_OFFSET)
#endif

#define	__phys_to_pfn(paddr)	((paddr) >> PAGE_SHIFT)
#define	__pfn_to_phys(pfn)	((pfn) << PAGE_SHIFT)

#ifndef __ASSEMBLY__

#ifndef ISA_DMA_THRESHOLD
#define ISA_DMA_THRESHOLD	(0xffffffffULL)
#endif

#ifndef arch_adjust_zones
#define arch_adjust_zones(node,size,holes) do { } while (0)
#endif

#define PHYS_PFN_OFFSET	(PHYS_OFFSET >> PAGE_SHIFT)

static inline unsigned long virt_to_phys(void *x)
{
	return __virt_to_phys((unsigned long)(x));
}

static inline void *phys_to_virt(unsigned long x)
{
	return (void *)(__phys_to_virt((unsigned long)(x)));
}

#define __pa(x)			__virt_to_phys((unsigned long)(x))
#define __va(x)			((void *)__phys_to_virt((unsigned long)(x)))
#define pfn_to_kaddr(pfn)	__va((pfn) << PAGE_SHIFT)

static inline __deprecated unsigned long virt_to_bus(void *x)
{
	return __virt_to_bus((unsigned long)x);
}

static inline __deprecated void *bus_to_virt(unsigned long x)
{
	return (void *)__bus_to_virt(x);
}

#ifndef CONFIG_DISCONTIGMEM
#define ARCH_PFN_OFFSET		PHYS_PFN_OFFSET
#define pfn_valid(pfn)		((pfn) >= PHYS_PFN_OFFSET && (pfn) < (PHYS_PFN_OFFSET + max_mapnr))

#define virt_to_page(kaddr)	pfn_to_page(__pa(kaddr) >> PAGE_SHIFT)
#define virt_addr_valid(kaddr)	((unsigned long)(kaddr) >= PAGE_OFFSET && (unsigned long)(kaddr) < (unsigned long)high_memory)

#define PHYS_TO_NID(addr)	(0)

#else /* CONFIG_DISCONTIGMEM */

#include <linux/numa.h>
#define arch_pfn_to_nid(pfn)	PFN_TO_NID(pfn)
#define arch_local_page_offset(pfn, nid) LOCAL_MAP_NR((pfn) << PAGE_SHIFT)

#define pfn_valid(pfn)						\
	({							\
		unsigned int nid = PFN_TO_NID(pfn);		\
		int valid = nid < MAX_NUMNODES;			\
		if (valid) {					\
			pg_data_t *node = NODE_DATA(nid);	\
			valid = (pfn - node->node_start_pfn) <	\
				node->node_spanned_pages;	\
		}						\
		valid;						\
	})

#define virt_to_page(kaddr)					\
	(ADDR_TO_MAPBASE(kaddr) + LOCAL_MAP_NR(kaddr))

#define virt_addr_valid(kaddr)	(KVADDR_TO_NID(kaddr) < MAX_NUMNODES)

#define PHYS_TO_NID(addr)	PFN_TO_NID((addr) >> PAGE_SHIFT)

#endif /* !CONFIG_DISCONTIGMEM */

#define page_to_phys(page)	(page_to_pfn(page) << PAGE_SHIFT)

#ifndef __arch_page_to_dma
#define page_to_dma(dev, page)		((dma_addr_t)__virt_to_bus((unsigned long)page_address(page)))
#define dma_to_virt(dev, addr)		((void *)__bus_to_virt(addr))
#define virt_to_dma(dev, addr)		((dma_addr_t)__virt_to_bus((unsigned long)(addr)))
#else
#define page_to_dma(dev, page)		(__arch_page_to_dma(dev, page))
#define dma_to_virt(dev, addr)		(__arch_dma_to_virt(dev, addr))
#define virt_to_dma(dev, addr)		(__arch_virt_to_dma(dev, addr))
#endif

#ifndef arch_is_coherent
#define arch_is_coherent()		0
#endif

#endif

#include <asm-generic/memory_model.h>

#endif

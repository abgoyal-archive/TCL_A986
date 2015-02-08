
#ifndef __ASM_ARM_MEMORY_H
#define __ASM_ARM_MEMORY_H

#include <linux/compiler.h>
#include <linux/const.h>
#include <mach/memory.h>
#include <asm/sizes.h>

#define UL(x) _AC(x, UL)

#ifdef CONFIG_MMU

#define PAGE_OFFSET		UL(CONFIG_PAGE_OFFSET)
#define TASK_SIZE		(UL(CONFIG_PAGE_OFFSET) - UL(0x01000000))
#define TASK_UNMAPPED_BASE	(UL(CONFIG_PAGE_OFFSET) / 3)

#define TASK_SIZE_26		UL(0x04000000)

#ifndef CONFIG_THUMB2_KERNEL
#define MODULES_VADDR		(PAGE_OFFSET - 16*1024*1024)
#else
/* smaller range for Thumb-2 symbols relocation (2^24)*/
#define MODULES_VADDR		(PAGE_OFFSET - 8*1024*1024)
#endif

#if TASK_SIZE > MODULES_VADDR
#error Top of user space clashes with start of module space
#endif

#ifdef CONFIG_HIGHMEM
#define MODULES_END		(PAGE_OFFSET - PMD_SIZE)
#else
#define MODULES_END		(PAGE_OFFSET)
#endif

#define XIP_VIRT_ADDR(physaddr)  (MODULES_VADDR + ((physaddr) & 0x000fffff))

#define IOREMAP_MAX_ORDER	24

#ifndef CONSISTENT_DMA_SIZE
#define CONSISTENT_DMA_SIZE 	SZ_2M
#endif

#define CONSISTENT_END		(0xffe00000UL)
#define CONSISTENT_BASE		(CONSISTENT_END - CONSISTENT_DMA_SIZE)

#else /* CONFIG_MMU */

#ifndef TASK_SIZE
#define TASK_SIZE		(CONFIG_DRAM_SIZE)
#endif

#ifndef TASK_UNMAPPED_BASE
#define TASK_UNMAPPED_BASE	UL(0x00000000)
#endif

#ifndef PHYS_OFFSET
#define PHYS_OFFSET 		UL(CONFIG_DRAM_BASE)
#endif

#ifndef END_MEM
#define END_MEM     		(UL(CONFIG_DRAM_BASE) + CONFIG_DRAM_SIZE)
#endif

#ifndef PAGE_OFFSET
#define PAGE_OFFSET		(PHYS_OFFSET)
#endif

#define MODULES_END		(END_MEM)
#define MODULES_VADDR		(PHYS_OFFSET)

#endif /* !CONFIG_MMU */

#ifndef __virt_to_phys
#define __virt_to_phys(x)	((x) - PAGE_OFFSET + PHYS_OFFSET)
#define __phys_to_virt(x)	((x) - PHYS_OFFSET + PAGE_OFFSET)
#endif

#define	__phys_to_pfn(paddr)	((paddr) >> PAGE_SHIFT)
#define	__pfn_to_phys(pfn)	((pfn) << PAGE_SHIFT)

#define page_to_phys(page)	(__pfn_to_phys(page_to_pfn(page)))
#define phys_to_page(phys)	(pfn_to_page(__phys_to_pfn(phys)))

#ifndef __ASSEMBLY__

#ifndef ISA_DMA_THRESHOLD
#define ISA_DMA_THRESHOLD	(0xffffffffULL)
#endif

#ifndef arch_adjust_zones
#define arch_adjust_zones(node,size,holes) do { } while (0)
#elif !defined(CONFIG_ZONE_DMA)
#error "custom arch_adjust_zones() requires CONFIG_ZONE_DMA"
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

#ifndef __virt_to_bus
#define __virt_to_bus	__virt_to_phys
#define __bus_to_virt	__phys_to_virt
#define __pfn_to_bus(x)	__pfn_to_phys(x)
#define __bus_to_pfn(x)	__phys_to_pfn(x)
#endif

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

#define virt_to_page(kaddr)	pfn_to_page(__pa(kaddr) >> PAGE_SHIFT)
#define virt_addr_valid(kaddr)	((unsigned long)(kaddr) >= PAGE_OFFSET && (unsigned long)(kaddr) < (unsigned long)high_memory)

#define PHYS_TO_NID(addr)	(0)

#else /* CONFIG_DISCONTIGMEM */

#include <linux/numa.h>

#define arch_pfn_to_nid(pfn)	PFN_TO_NID(pfn)
#define arch_local_page_offset(pfn, nid) LOCAL_MAP_NR((pfn) << PAGE_SHIFT)

#define virt_to_page(kaddr)					\
	(ADDR_TO_MAPBASE(kaddr) + LOCAL_MAP_NR(kaddr))

#define virt_addr_valid(kaddr)	(KVADDR_TO_NID(kaddr) < MAX_NUMNODES)

#define PHYS_TO_NID(addr)	PFN_TO_NID((addr) >> PAGE_SHIFT)

#define ADDR_TO_MAPBASE(kaddr)	NODE_MEM_MAP(KVADDR_TO_NID(kaddr))

#define PFN_TO_MAPBASE(pfn)	NODE_MEM_MAP(PFN_TO_NID(pfn))

#ifdef NODE_MEM_SIZE_BITS
#define NODE_MEM_SIZE_MASK	((1 << NODE_MEM_SIZE_BITS) - 1)

#define KVADDR_TO_NID(addr) \
	(((unsigned long)(addr) - PAGE_OFFSET) >> NODE_MEM_SIZE_BITS)

#define PFN_TO_NID(pfn) \
	(((pfn) - PHYS_PFN_OFFSET) >> (NODE_MEM_SIZE_BITS - PAGE_SHIFT))

#define LOCAL_MAP_NR(addr) \
	(((unsigned long)(addr) & NODE_MEM_SIZE_MASK) >> PAGE_SHIFT)

#endif /* NODE_MEM_SIZE_BITS */

#endif /* !CONFIG_DISCONTIGMEM */

#ifndef arch_is_coherent
#define arch_is_coherent()		0
#endif

#endif

#include <asm-generic/memory_model.h>

#endif

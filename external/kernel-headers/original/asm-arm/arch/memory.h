
#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#if defined(CONFIG_ARCH_OMAP1)
#define PHYS_OFFSET		UL(0x10000000)
#elif defined(CONFIG_ARCH_OMAP2)
#define PHYS_OFFSET		UL(0x80000000)
#endif


#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt(x)	__phys_to_virt(x)

#ifdef CONFIG_ARCH_OMAP15XX

#define OMAP1510_LB_OFFSET	UL(0x30000000)

#define virt_to_lbus(x)		((x) - PAGE_OFFSET + OMAP1510_LB_OFFSET)
#define lbus_to_virt(x)		((x) - OMAP1510_LB_OFFSET + PAGE_OFFSET)
#define is_lbus_device(dev)	(cpu_is_omap1510() && dev && (strncmp(dev->bus_id, "ohci", 4) == 0))

#define __arch_page_to_dma(dev, page)	({is_lbus_device(dev) ? \
					(dma_addr_t)virt_to_lbus(page_address(page)) : \
					(dma_addr_t)__virt_to_bus(page_address(page));})

#define __arch_dma_to_virt(dev, addr)	({is_lbus_device(dev) ? \
					lbus_to_virt(addr) : \
					__bus_to_virt(addr);})

#define __arch_virt_to_dma(dev, addr)	({is_lbus_device(dev) ? \
					virt_to_lbus(addr) : \
					__virt_to_bus(addr);})

#endif	/* CONFIG_ARCH_OMAP15XX */

/* Override the ARM default */
#ifdef CONFIG_FB_OMAP_CONSISTENT_DMA_SIZE

#if (CONFIG_FB_OMAP_CONSISTENT_DMA_SIZE == 0)
#undef CONFIG_FB_OMAP_CONSISTENT_DMA_SIZE
#define CONFIG_FB_OMAP_CONSISTENT_DMA_SIZE 2
#endif

#define CONSISTENT_DMA_SIZE \
	(((CONFIG_FB_OMAP_CONSISTENT_DMA_SIZE + 1) & ~1) * 1024 * 1024)

#endif

#endif


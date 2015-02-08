

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#if defined(CONFIG_ARCH_OMAP1)
#define PHYS_OFFSET		UL(0x10000000)
#else
#define PHYS_OFFSET		UL(0x80000000)
#endif

#ifdef CONFIG_ARCH_OMAP15XX

#define OMAP1510_LB_OFFSET	UL(0x30000000)

#define virt_to_lbus(x)		((x) - PAGE_OFFSET + OMAP1510_LB_OFFSET)
#define lbus_to_virt(x)		((x) - OMAP1510_LB_OFFSET + PAGE_OFFSET)
#define is_lbus_device(dev)	(cpu_is_omap15xx() && dev && (strncmp(dev_name(dev), "ohci", 4) == 0))

#define __arch_page_to_dma(dev, page)	\
	({ dma_addr_t __dma = page_to_phys(page); \
	   if (is_lbus_device(dev)) \
		__dma = __dma - PHYS_OFFSET + OMAP1510_LB_OFFSET; \
	   __dma; })

#define __arch_dma_to_page(dev, addr)	\
	({ dma_addr_t __dma = addr;				\
	   if (is_lbus_device(dev))				\
		__dma += PHYS_OFFSET - OMAP1510_LB_OFFSET;	\
	   phys_to_page(__dma);					\
	})

#define __arch_dma_to_virt(dev, addr)	({ (void *) (is_lbus_device(dev) ? \
						lbus_to_virt(addr) : \
						__phys_to_virt(addr)); })

#define __arch_virt_to_dma(dev, addr)	({ unsigned long __addr = (unsigned long)(addr); \
					   (dma_addr_t) (is_lbus_device(dev) ? \
						virt_to_lbus(__addr) : \
						__virt_to_phys(__addr)); })

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


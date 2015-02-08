

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#define PHYS_OFFSET		UL(0x20000000)
#define CONSISTENT_DMA_SIZE	(SZ_8M + SZ_4M + SZ_2M)

/* Maximum of 256MiB in one bank */
#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	28

#endif /* __ASM_ARCH_MEMORY_H */

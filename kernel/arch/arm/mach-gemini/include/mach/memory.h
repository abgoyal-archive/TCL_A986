
#ifndef __MACH_MEMORY_H
#define __MACH_MEMORY_H

#ifdef CONFIG_GEMINI_MEM_SWAP
# define PHYS_OFFSET	UL(0x00000000)
#else
# define PHYS_OFFSET	UL(0x10000000)
#endif

#endif /* __MACH_MEMORY_H */

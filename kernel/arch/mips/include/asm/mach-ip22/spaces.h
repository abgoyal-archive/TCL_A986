
#ifndef _ASM_MACH_IP22_SPACES_H
#define _ASM_MACH_IP22_SPACES_H


#ifdef CONFIG_64BIT

#define PAGE_OFFSET		0xffffffff80000000UL

#define CAC_BASE		0xffffffff80000000
#define IO_BASE			0xffffffffa0000000
#define UNCAC_BASE		0xffffffffa0000000
#define MAP_BASE		0xc000000000000000

#endif /* CONFIG_64BIT */

#include <asm/mach-generic/spaces.h>

#endif /* __ASM_MACH_IP22_SPACES_H */

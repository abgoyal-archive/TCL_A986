
#ifndef _ASM_VIRTCONVERT_H
#define _ASM_VIRTCONVERT_H


#ifdef __KERNEL__

#include <asm/setup.h>

#ifdef CONFIG_MMU

#define phys_to_virt(vaddr)	((void *) ((unsigned long)(vaddr) + PAGE_OFFSET))
#define virt_to_phys(vaddr)	((unsigned long) (vaddr) - PAGE_OFFSET)

#else

#define phys_to_virt(vaddr)	((void *) (vaddr))
#define virt_to_phys(vaddr)	((unsigned long) (vaddr))

#endif

#define virt_to_bus virt_to_phys
#define bus_to_virt phys_to_virt

#define __page_address(page)	(PAGE_OFFSET + (((page) - mem_map) << PAGE_SHIFT))
#define page_to_phys(page)	virt_to_phys((void *)__page_address(page))

#endif
#endif

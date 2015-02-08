

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#define PHYS_OFFSET	UL(0xc0000000)

#ifdef CONFIG_DISCONTIGMEM


# ifdef CONFIG_LH7A40X_ONE_BANK_PER_NODE
#  define KVADDR_TO_NID(addr) \
  (  ((((unsigned long) (addr) - PAGE_OFFSET) >> 24) &  1)\
   | ((((unsigned long) (addr) - PAGE_OFFSET) >> 25) & ~1))
# else  /* 2 banks per node */
#  define KVADDR_TO_NID(addr) \
      (((unsigned long) (addr) - PAGE_OFFSET) >> 26)
# endif


# ifdef CONFIG_LH7A40X_ONE_BANK_PER_NODE
#  define PFN_TO_NID(pfn) \
  (((((pfn) - PHYS_PFN_OFFSET) >> (24 - PAGE_SHIFT)) &  1)\
 | ((((pfn) - PHYS_PFN_OFFSET) >> (25 - PAGE_SHIFT)) & ~1))
# else  /* 2 banks per node */
#  define PFN_TO_NID(pfn) \
    (((pfn) - PHYS_PFN_OFFSET) >> (26 - PAGE_SHIFT))
#endif


# ifdef CONFIG_LH7A40X_ONE_BANK_PER_NODE
#  define LOCAL_MAP_NR(addr) \
       (((unsigned long)(addr) & 0x003fffff) >> PAGE_SHIFT)
# else  /* 2 banks per node */
#  define LOCAL_MAP_NR(addr) \
       (((unsigned long)(addr) & 0x01ffffff) >> PAGE_SHIFT)
# endif

#endif

#define MAX_PHYSMEM_BITS	32
#define SECTION_SIZE_BITS	24

#endif

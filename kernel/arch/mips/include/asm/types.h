
#ifndef _ASM_TYPES_H
#define _ASM_TYPES_H

#if (_MIPS_SZLONG == 64) && !defined(__KERNEL__)
# include <asm-generic/int-l64.h>
#else
# include <asm-generic/int-ll64.h>
#endif

#ifndef __ASSEMBLY__

typedef unsigned short umode_t;

#endif /* __ASSEMBLY__ */

#ifdef __KERNEL__
#ifndef __ASSEMBLY__

#if (defined(CONFIG_HIGHMEM) && defined(CONFIG_64BIT_PHYS_ADDR)) \
    || defined(CONFIG_64BIT)
typedef u64 dma_addr_t;
#else
typedef u32 dma_addr_t;
#endif
typedef u64 dma64_addr_t;

#ifdef CONFIG_64BIT_PHYS_ADDR
typedef unsigned long long phys_t;
#else
typedef unsigned long phys_t;
#endif

#endif /* __ASSEMBLY__ */

#endif /* __KERNEL__ */

#endif /* _ASM_TYPES_H */

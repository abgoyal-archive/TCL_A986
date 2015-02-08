
#ifndef _ASM_POWERPC_TYPES_H
#define _ASM_POWERPC_TYPES_H

#if defined(__powerpc64__) && !defined(__KERNEL__)
# include <asm-generic/int-l64.h>
#else
# include <asm-generic/int-ll64.h>
#endif

#ifndef __ASSEMBLY__


#ifdef __powerpc64__
typedef unsigned int umode_t;
#else
typedef unsigned short umode_t;
#endif

typedef struct {
	__u32 u[4];
} __attribute__((aligned(16))) __vector128;

#endif /* __ASSEMBLY__ */

#ifdef __KERNEL__
#ifndef __ASSEMBLY__

typedef __vector128 vector128;

#if defined(__powerpc64__) || defined(CONFIG_PHYS_64BIT)
typedef u64 dma_addr_t;
#else
typedef u32 dma_addr_t;
#endif
typedef u64 dma64_addr_t;

typedef struct {
	unsigned long entry;
	unsigned long toc;
	unsigned long env;
} func_descr_t;

#endif /* __ASSEMBLY__ */

#endif /* __KERNEL__ */

#endif /* _ASM_POWERPC_TYPES_H */


#ifndef _ASM_IA64_TYPES_H
#define _ASM_IA64_TYPES_H


#ifdef __KERNEL__
#include <asm-generic/int-ll64.h>
#else
#include <asm-generic/int-l64.h>
#endif

#ifdef __ASSEMBLY__
# define __IA64_UL(x)		(x)
# define __IA64_UL_CONST(x)	x

#else
# define __IA64_UL(x)		((unsigned long)(x))
# define __IA64_UL_CONST(x)	x##UL

typedef unsigned int umode_t;

# ifdef __KERNEL__

struct fnptr {
	unsigned long ip;
	unsigned long gp;
};

/* DMA addresses are 64-bits wide, in general.  */
typedef u64 dma_addr_t;

# endif /* __KERNEL__ */
#endif /* !__ASSEMBLY__ */

#endif /* _ASM_IA64_TYPES_H */

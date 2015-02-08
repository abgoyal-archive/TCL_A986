

#ifndef _XTENSA_TYPES_H
#define _XTENSA_TYPES_H

#include <asm-generic/int-ll64.h>

#ifdef __ASSEMBLY__
# define __XTENSA_UL(x)		(x)
# define __XTENSA_UL_CONST(x)	x
#else
# define __XTENSA_UL(x)		((unsigned long)(x))
# define __XTENSA_UL_CONST(x)	x##UL
#endif

#ifndef __ASSEMBLY__

typedef unsigned short umode_t;

#ifdef __KERNEL__

#define BITS_PER_LONG 32

/* Dma addresses are 32-bits wide.  */

typedef u32 dma_addr_t;

#endif	/* __KERNEL__ */
#endif

#endif	/* _XTENSA_TYPES_H */

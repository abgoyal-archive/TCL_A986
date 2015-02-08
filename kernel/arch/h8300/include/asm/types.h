
#ifndef _H8300_TYPES_H
#define _H8300_TYPES_H

#include <asm-generic/int-ll64.h>

#if !defined(__ASSEMBLY__)


typedef unsigned short umode_t;

#ifdef __KERNEL__

#define BITS_PER_LONG 32

/* Dma addresses are 32-bits wide.  */

typedef u32 dma_addr_t;

#endif /* __KERNEL__ */

#endif /* __ASSEMBLY__ */

#endif /* _H8300_TYPES_H */

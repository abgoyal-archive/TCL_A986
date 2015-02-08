
#ifndef _M68K_TYPES_H
#define _M68K_TYPES_H

#include <asm-generic/int-ll64.h>

#ifndef __ASSEMBLY__

typedef unsigned short umode_t;

#endif /* __ASSEMBLY__ */

#ifdef __KERNEL__

#define BITS_PER_LONG 32

#ifndef __ASSEMBLY__

/* DMA addresses are always 32-bits wide */

typedef u32 dma_addr_t;
typedef u32 dma64_addr_t;

#endif /* __ASSEMBLY__ */

#endif /* __KERNEL__ */

#endif /* _M68K_TYPES_H */


#ifndef _ALPHA_TYPES_H
#define _ALPHA_TYPES_H


#ifdef __KERNEL__
#include <asm-generic/int-ll64.h>
#else
#include <asm-generic/int-l64.h>
#endif

#ifndef __ASSEMBLY__

typedef unsigned int umode_t;

#endif /* __ASSEMBLY__ */

#ifdef __KERNEL__
#ifndef __ASSEMBLY__

typedef u64 dma_addr_t;
typedef u64 dma64_addr_t;

#endif /* __ASSEMBLY__ */
#endif /* __KERNEL__ */
#endif /* _ALPHA_TYPES_H */

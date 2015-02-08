
#ifndef __ASM_SH_ADDRSPACE_H
#define __ASM_SH_ADDRSPACE_H

#ifdef __KERNEL__

#include <cpu/addrspace.h>

/* If this CPU supports segmentation, hook up the helpers */
#ifdef P1SEG


/* Returns the privileged segment base of a given address  */
#define PXSEG(a)	(((unsigned long)(a)) & 0xe0000000)

#ifdef CONFIG_29BIT
#define P1SEGADDR(a)	\
	((__typeof__(a))(((unsigned long)(a) & 0x1fffffff) | P1SEG))
#define P2SEGADDR(a)	\
	((__typeof__(a))(((unsigned long)(a) & 0x1fffffff) | P2SEG))
#define P3SEGADDR(a)	\
	((__typeof__(a))(((unsigned long)(a) & 0x1fffffff) | P3SEG))
#define P4SEGADDR(a)	\
	((__typeof__(a))(((unsigned long)(a) & 0x1fffffff) | P4SEG))
#else
#define P1SEGADDR(a)	__futile_remapping_attempt
#define P2SEGADDR(a)	__futile_remapping_attempt
#define P3SEGADDR(a)	__futile_remapping_attempt
#define P4SEGADDR(a)	__futile_remapping_attempt
#endif
#endif /* P1SEG */

/* Check if an address can be reached in 29 bits */
#define IS_29BIT(a)	(((unsigned long)(a)) < 0x20000000)

#ifdef CONFIG_SH_STORE_QUEUES
#define P3_ADDR_MAX		(P4SEG_STORE_QUE + 0x04000000)
#else
#define P3_ADDR_MAX		P4SEG
#endif

#endif /* __KERNEL__ */
#endif /* __ASM_SH_ADDRSPACE_H */

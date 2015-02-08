
#ifndef _PPC64_KDUMP_H
#define _PPC64_KDUMP_H

#include <asm/page.h>

#if defined(CONFIG_RELOCATABLE) && defined(CONFIG_PPC64)
#define KDUMP_KERNELBASE	0x4000000
#else
#define KDUMP_KERNELBASE	0x2000000
#endif

#define KDUMP_RESERVE_LIMIT	0x10000 /* 64K */

#ifdef CONFIG_CRASH_DUMP

#ifdef __powerpc64__
#define KDUMP_TRAMPOLINE_START	0x0100
#define KDUMP_TRAMPOLINE_END	0x3000
#else
#define KDUMP_TRAMPOLINE_START	(0x0100 + PAGE_OFFSET)
#define KDUMP_TRAMPOLINE_END	(0x3000 + PAGE_OFFSET)
#endif /* __powerpc64__ */

#define KDUMP_MIN_TCE_ENTRIES	2048

#endif /* CONFIG_CRASH_DUMP */

#ifndef __ASSEMBLY__

#if defined(CONFIG_CRASH_DUMP) && !defined(CONFIG_RELOCATABLE)
extern void reserve_kdump_trampoline(void);
extern void setup_kdump_trampoline(void);
#else
/* !CRASH_DUMP || RELOCATABLE */
static inline void reserve_kdump_trampoline(void) { ; }
static inline void setup_kdump_trampoline(void) { ; }
#endif

#endif /* __ASSEMBLY__ */

#endif /* __PPC64_KDUMP_H */

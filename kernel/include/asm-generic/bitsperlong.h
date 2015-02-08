
#ifndef __ASM_GENERIC_BITS_PER_LONG
#define __ASM_GENERIC_BITS_PER_LONG

#ifndef __BITS_PER_LONG
#define __BITS_PER_LONG 32
#endif

#ifdef __KERNEL__

#ifdef CONFIG_64BIT
#define BITS_PER_LONG 64
#else
#define BITS_PER_LONG 32
#endif /* CONFIG_64BIT */

#if 0 && BITS_PER_LONG != __BITS_PER_LONG
#error Inconsistent word size. Check asm/bitsperlong.h
#endif

#endif /* __KERNEL__ */
#endif /* __ASM_GENERIC_BITS_PER_LONG */

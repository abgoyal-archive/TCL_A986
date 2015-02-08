
#ifndef _ASM_IA64_STRING_H
#define _ASM_IA64_STRING_H



#define __HAVE_ARCH_STRLEN	1 /* see arch/ia64/lib/strlen.S */
#define __HAVE_ARCH_MEMSET	1 /* see arch/ia64/lib/memset.S */
#define __HAVE_ARCH_MEMCPY	1 /* see arch/ia64/lib/memcpy.S */

extern __kernel_size_t strlen (const char *);
extern void *memcpy (void *, const void *, __kernel_size_t);
extern void *memset (void *, int, __kernel_size_t);

#endif /* _ASM_IA64_STRING_H */

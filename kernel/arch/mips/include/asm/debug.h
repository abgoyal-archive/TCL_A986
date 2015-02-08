

#ifndef _ASM_DEBUG_H
#define _ASM_DEBUG_H



#ifdef CONFIG_RUNTIME_DEBUG

#include <linux/kernel.h>

#define db_assert(x)  if (!(x)) { \
	panic("assertion failed at %s:%d: %s", __FILE__, __LINE__, #x); }
#define db_warn(x)  if (!(x)) { \
	printk(KERN_WARNING "warning at %s:%d: %s", __FILE__, __LINE__, #x); }
#define db_verify(x, y) db_assert(x y)
#define db_verify_warn(x, y) db_warn(x y)
#define db_run(x)  do { x; } while (0)

#else

#define db_assert(x)
#define db_warn(x)
#define db_verify(x, y) x
#define db_verify_warn(x, y) x
#define db_run(x)

#endif

#endif /* _ASM_DEBUG_H */

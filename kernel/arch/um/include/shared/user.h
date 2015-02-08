

#ifndef __USER_H__
#define __USER_H__

#include "kern_constants.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/* This is to get size_t */
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stddef.h>
#endif

extern void panic(const char *fmt, ...)
	__attribute__ ((format (printf, 1, 2)));

#ifdef UML_CONFIG_PRINTK
extern int printk(const char *fmt, ...)
	__attribute__ ((format (printf, 1, 2)));
#else
static inline int printk(const char *fmt, ...)
{
	return 0;
}
#endif

extern void schedule(void);
extern int in_aton(char *str);
extern int open_gdb_chan(void);
extern size_t strlcpy(char *, const char *, size_t);
extern size_t strlcat(char *, const char *, size_t);

#endif

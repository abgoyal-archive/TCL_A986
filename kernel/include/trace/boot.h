
#ifndef _LINUX_TRACE_BOOT_H
#define _LINUX_TRACE_BOOT_H

#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/init.h>

struct boot_trace_call {
	pid_t			caller;
	char			func[KSYM_SYMBOL_LEN];
};

struct boot_trace_ret {
	char			func[KSYM_SYMBOL_LEN];
	int				result;
	unsigned long long	duration;		/* nsecs */
};

#ifdef CONFIG_BOOT_TRACER
/* Append the traces on the ring-buffer */
extern void trace_boot_call(struct boot_trace_call *bt, initcall_t fn);
extern void trace_boot_ret(struct boot_trace_ret *bt, initcall_t fn);

extern void start_boot_trace(void);

extern void enable_boot_trace(void);

extern void disable_boot_trace(void);
#else
static inline
void trace_boot_call(struct boot_trace_call *bt, initcall_t fn) { }

static inline
void trace_boot_ret(struct boot_trace_ret *bt, initcall_t fn) { }

static inline void start_boot_trace(void) { }
static inline void enable_boot_trace(void) { }
static inline void disable_boot_trace(void) { }
#endif /* CONFIG_BOOT_TRACER */

#endif /* __LINUX_TRACE_BOOT_H */

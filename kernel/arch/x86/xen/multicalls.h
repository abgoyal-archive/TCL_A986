
#ifndef _XEN_MULTICALLS_H
#define _XEN_MULTICALLS_H

#include "xen-ops.h"

/* Multicalls */
struct multicall_space
{
	struct multicall_entry *mc;
	void *args;
};

/* Allocate room for a multicall and its args */
struct multicall_space __xen_mc_entry(size_t args);

DECLARE_PER_CPU(unsigned long, xen_mc_irq_flags);

static inline void xen_mc_batch(void)
{
	unsigned long flags;
	/* need to disable interrupts until this entry is complete */
	local_irq_save(flags);
	__get_cpu_var(xen_mc_irq_flags) = flags;
}

static inline struct multicall_space xen_mc_entry(size_t args)
{
	xen_mc_batch();
	return __xen_mc_entry(args);
}

/* Flush all pending multicalls */
void xen_mc_flush(void);

/* Issue a multicall if we're not in a lazy mode */
static inline void xen_mc_issue(unsigned mode)
{
	if ((paravirt_get_lazy_mode() & mode) == 0)
		xen_mc_flush();

	/* restore flags saved in xen_mc_batch */
	local_irq_restore(percpu_read(xen_mc_irq_flags));
}

/* Set up a callback to be called when the current batch is flushed */
void xen_mc_callback(void (*fn)(void *), void *data);

struct multicall_space xen_mc_extend_args(unsigned long op, size_t arg_size);

#endif /* _XEN_MULTICALLS_H */

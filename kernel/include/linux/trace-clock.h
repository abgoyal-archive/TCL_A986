
#ifndef _LINUX_TRACE_CLOCK_H
#define _LINUX_TRACE_CLOCK_H


#ifdef CONFIG_HAVE_TRACE_CLOCK
#include <asm/trace-clock.h>
#else
#include <asm-generic/trace-clock.h>
#endif /* CONFIG_HAVE_TRACE_CLOCK */
#endif /* _LINUX_TRACE_CLOCK_H */

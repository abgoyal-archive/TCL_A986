
#ifndef _ASMARM_TIMEX_H
#define _ASMARM_TIMEX_H

#include <mach/timex.h>

typedef unsigned long cycles_t;

static inline cycles_t get_cycles (void)
{
	return 0;
}

#endif

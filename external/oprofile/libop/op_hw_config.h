
#ifndef OP_HW_CONFIG_H
#define OP_HW_CONFIG_H

#ifdef __alpha__
#define OP_MAX_COUNTERS	20
#else
#define OP_MAX_COUNTERS	8
#endif

#define OP_MAX_PERF_COUNT	2147483647UL

#endif /* OP_HW_CONFIG_H */

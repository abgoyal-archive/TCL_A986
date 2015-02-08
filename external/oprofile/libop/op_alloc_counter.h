
#ifndef OP_ALLOC_COUNTER_H
#define OP_ALLOC_COUNTER_H

#include <stddef.h>

#include "op_cpu_type.h"

struct op_event;

#ifdef __cplusplus
extern "C" {
#endif

size_t * map_event_to_counter(struct op_event const * pev[], int nr_events,
                              op_cpu cpu_type);

#ifdef __cplusplus
}
#endif

#endif /* !OP_ALLOC_COUNTER_H */

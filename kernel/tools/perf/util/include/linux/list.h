
#include "../../../../include/linux/list.h"

#ifndef PERF_LIST_H
#define PERF_LIST_H
static inline void list_del_range(struct list_head *begin,
				  struct list_head *end)
{
	begin->prev->next = end->next;
	end->next->prev = begin->prev;
}
#endif

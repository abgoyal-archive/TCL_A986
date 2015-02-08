

#include <linux/module.h>
#include <linux/list.h>


void __list_add(struct list_head *new,
			      struct list_head *prev,
			      struct list_head *next)
{
	WARN(next->prev != prev,
		"list_add corruption. next->prev should be "
		"prev (%p), but was %p. (next=%p).\n",
		prev, next->prev, next);
	WARN(prev->next != next,
		"list_add corruption. prev->next should be "
		"next (%p), but was %p. (prev=%p).\n",
		next, prev->next, prev);
	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}
EXPORT_SYMBOL(__list_add);

void list_del(struct list_head *entry)
{
	WARN(entry->prev->next != entry,
		"list_del corruption. prev->next should be %p, "
		"but was %p\n", entry, entry->prev->next);
	WARN(entry->next->prev != entry,
		"list_del corruption. next->prev should be %p, "
		"but was %p\n", entry, entry->next->prev);
	__list_del(entry->prev, entry->next);
	entry->next = LIST_POISON1;
	entry->prev = LIST_POISON2;
}
EXPORT_SYMBOL(list_del);

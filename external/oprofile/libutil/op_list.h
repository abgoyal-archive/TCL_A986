
#ifndef OP_LIST_H
#define OP_LIST_H


struct list_head {
	struct list_head * next, * prev;
};

static __inline__ void list_init(struct list_head * ptr)
{
	ptr->next = ptr;
	ptr->prev = ptr;
}

static __inline__ void __list_add(struct list_head * new_entry,
	struct list_head * prev,
	struct list_head * next)
{
	next->prev = new_entry;
	new_entry->next = next;
	new_entry->prev = prev;
	prev->next = new_entry;
}

static __inline__ void list_add(struct list_head * new_entry, struct list_head * head)
{
	__list_add(new_entry, head, head->next);
}

static __inline__ void list_add_tail(struct list_head * new_entry, struct list_head * head)
{
	__list_add(new_entry, head->prev, head);
}

static __inline__ void __list_del(struct list_head * prev,
				  struct list_head * next)
{
	next->prev = prev;
	prev->next = next;
}

static __inline__ void list_del(struct list_head * entry)
{
	__list_del(entry->prev, entry->next);
}

static __inline__ void list_del_init(struct list_head * entry)
{
	__list_del(entry->prev, entry->next);
	list_init(entry);
}

static __inline__ int list_empty(struct list_head const * head)
{
	return head->next == head;
}

static __inline__ void list_splice(struct list_head * list, struct list_head * head)
{
	struct list_head * first = list->next;

	if (first != list) {
		struct list_head * last = list->prev;
		struct list_head * at = head->next;

		first->prev = head;
		head->next = first;

		last->next = at;
		at->prev = last;
	}
}

#define list_entry(ptr, type, member) \
	((type *)((char *)(ptr)-(unsigned long)(&((type *)0)->member)))

#define list_for_each(pos, head) \
	for (pos = (head)->next; pos != (head); pos = pos->next)

#define list_for_each_safe(pos, n, head) \
	for (pos = (head)->next, n = pos->next; pos != (head); \
		pos = n, n = pos->next)

#define LIST_HEAD_INIT(name) { &(name), &(name) }

#define LIST_HEAD(name) struct list_head name = LIST_HEAD_INIT(name)

#endif /* OP_LIST_H */

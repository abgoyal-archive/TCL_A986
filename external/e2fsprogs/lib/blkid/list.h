#if !defined(_BLKID_LIST_H) && !defined(LIST_HEAD)
#define _BLKID_LIST_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __GNUC__
#define _INLINE_ static __inline__
#else                         /* For Watcom C */
#define _INLINE_ static inline
#endif


struct list_head {
	struct list_head *next, *prev;
};

#define LIST_HEAD_INIT(name) { &(name), &(name) }

#define LIST_HEAD(name) \
	struct list_head name = LIST_HEAD_INIT(name)

#define INIT_LIST_HEAD(ptr) do { \
	(ptr)->next = (ptr); (ptr)->prev = (ptr); \
} while (0)

_INLINE_ void __list_add(struct list_head * add,
	struct list_head * prev,
	struct list_head * next)
{
	next->prev = add;
	add->next = next;
	add->prev = prev;
	prev->next = add;
}

_INLINE_ void list_add(struct list_head *add, struct list_head *head)
{
	__list_add(add, head, head->next);
}

_INLINE_ void list_add_tail(struct list_head *add, struct list_head *head)
{
	__list_add(add, head->prev, head);
}

_INLINE_ void __list_del(struct list_head * prev,
				  struct list_head * next)
{
	next->prev = prev;
	prev->next = next;
}

_INLINE_ void list_del(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
}

_INLINE_ void list_del_init(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
	INIT_LIST_HEAD(entry);
}

_INLINE_ int list_empty(struct list_head *head)
{
	return head->next == head;
}

_INLINE_ void list_splice(struct list_head *list, struct list_head *head)
{
	struct list_head *first = list->next;

	if (first != list) {
		struct list_head *last = list->prev;
		struct list_head *at = head->next;

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

#define list_for_each_safe(pos, pnext, head) \
	for (pos = (head)->next, pnext = pos->next; pos != (head); \
	     pos = pnext, pnext = pos->next)

#undef _INLINE_

#ifdef __cplusplus
}
#endif

#endif /* _BLKID_LIST_H */

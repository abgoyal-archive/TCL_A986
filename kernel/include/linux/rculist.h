
#ifndef _LINUX_RCULIST_H
#define _LINUX_RCULIST_H

#ifdef __KERNEL__

#include <linux/list.h>
#include <linux/rcupdate.h>

static inline void __list_add_rcu(struct list_head *new,
		struct list_head *prev, struct list_head *next)
{
	new->next = next;
	new->prev = prev;
	rcu_assign_pointer(prev->next, new);
	next->prev = new;
}

static inline void list_add_rcu(struct list_head *new, struct list_head *head)
{
	__list_add_rcu(new, head, head->next);
}

static inline void list_add_tail_rcu(struct list_head *new,
					struct list_head *head)
{
	__list_add_rcu(new, head->prev, head);
}

static inline void list_del_rcu(struct list_head *entry)
{
	__list_del(entry->prev, entry->next);
	entry->prev = LIST_POISON2;
}

static inline void hlist_del_init_rcu(struct hlist_node *n)
{
	if (!hlist_unhashed(n)) {
		__hlist_del(n);
		n->pprev = NULL;
	}
}

static inline void list_replace_rcu(struct list_head *old,
				struct list_head *new)
{
	new->next = old->next;
	new->prev = old->prev;
	rcu_assign_pointer(new->prev->next, new);
	new->next->prev = new;
	old->prev = LIST_POISON2;
}

static inline void list_splice_init_rcu(struct list_head *list,
					struct list_head *head,
					void (*sync)(void))
{
	struct list_head *first = list->next;
	struct list_head *last = list->prev;
	struct list_head *at = head->next;

	if (list_empty(head))
		return;

	/* "first" and "last" tracking list, so initialize it. */

	INIT_LIST_HEAD(list);

	/*
	 * At this point, the list body still points to the source list.
	 * Wait for any readers to finish using the list before splicing
	 * the list body into the new list.  Any new readers will see
	 * an empty list.
	 */

	sync();

	/*
	 * Readers are finished with the source list, so perform splice.
	 * The order is important if the new list is global and accessible
	 * to concurrent RCU readers.  Note that RCU readers are not
	 * permitted to traverse the prev pointers without excluding
	 * this function.
	 */

	last->next = at;
	rcu_assign_pointer(head->next, first);
	first->prev = head;
	at->prev = last;
}

#define list_entry_rcu(ptr, type, member) \
	container_of(rcu_dereference_raw(ptr), type, member)

#define list_first_entry_rcu(ptr, type, member) \
	list_entry_rcu((ptr)->next, type, member)

#define __list_for_each_rcu(pos, head) \
	for (pos = rcu_dereference_raw((head)->next); \
		pos != (head); \
		pos = rcu_dereference_raw(pos->next))

#define __list_for_each_entry_rcu(pos, head, member) \
	for (pos = list_entry(rcu_dereference((head)->next), typeof(*pos), member); \
		&pos->member != (head); \
		pos = list_entry(rcu_dereference(pos->member.next), typeof(*pos), member))

#define list_for_each_entry_rcu(pos, head, member) \
	for (pos = list_entry_rcu((head)->next, typeof(*pos), member); \
		prefetch(pos->member.next), &pos->member != (head); \
		pos = list_entry_rcu(pos->member.next, typeof(*pos), member))


#define list_for_each_continue_rcu(pos, head) \
	for ((pos) = rcu_dereference_raw((pos)->next); \
		prefetch((pos)->next), (pos) != (head); \
		(pos) = rcu_dereference_raw((pos)->next))

#define list_for_each_entry_continue_rcu(pos, head, member) 		\
	for (pos = list_entry_rcu(pos->member.next, typeof(*pos), member); \
	     prefetch(pos->member.next), &pos->member != (head);	\
	     pos = list_entry_rcu(pos->member.next, typeof(*pos), member))

#define list_for_each_entry_continue_rcu(pos, head, member) \
	for (pos = list_entry_rcu(pos->member.next, typeof(*pos), member); \
		prefetch(pos->member.next), &pos->member != (head); \
		pos = list_entry_rcu(pos->member.next, typeof(*pos), member))


static inline void hlist_del_rcu(struct hlist_node *n)
{
	__hlist_del(n);
	n->pprev = LIST_POISON2;
}

static inline void hlist_replace_rcu(struct hlist_node *old,
					struct hlist_node *new)
{
	struct hlist_node *next = old->next;

	new->next = next;
	new->pprev = old->pprev;
	rcu_assign_pointer(*new->pprev, new);
	if (next)
		new->next->pprev = &new->next;
	old->pprev = LIST_POISON2;
}

static inline void hlist_add_head_rcu(struct hlist_node *n,
					struct hlist_head *h)
{
	struct hlist_node *first = h->first;

	n->next = first;
	n->pprev = &h->first;
	rcu_assign_pointer(h->first, n);
	if (first)
		first->pprev = &n->next;
}

static inline void hlist_add_before_rcu(struct hlist_node *n,
					struct hlist_node *next)
{
	n->pprev = next->pprev;
	n->next = next;
	rcu_assign_pointer(*(n->pprev), n);
	next->pprev = &n->next;
}

static inline void hlist_add_after_rcu(struct hlist_node *prev,
				       struct hlist_node *n)
{
	n->next = prev->next;
	n->pprev = &prev->next;
	rcu_assign_pointer(prev->next, n);
	if (n->next)
		n->next->pprev = &n->next;
}

#define __hlist_for_each_rcu(pos, head)			\
	for (pos = rcu_dereference((head)->first);	\
	     pos && ({ prefetch(pos->next); 1; });	\
	     pos = rcu_dereference(pos->next))

#define hlist_for_each_entry_rcu(tpos, pos, head, member)		 \
	for (pos = rcu_dereference_raw((head)->first);			 \
		pos && ({ prefetch(pos->next); 1; }) &&			 \
		({ tpos = hlist_entry(pos, typeof(*tpos), member); 1; }); \
		pos = rcu_dereference_raw(pos->next))

#define hlist_for_each_entry_rcu_bh(tpos, pos, head, member)		 \
	for (pos = rcu_dereference_bh((head)->first);			 \
		pos && ({ prefetch(pos->next); 1; }) &&			 \
		({ tpos = hlist_entry(pos, typeof(*tpos), member); 1; }); \
		pos = rcu_dereference_bh(pos->next))

#define hlist_for_each_entry_continue_rcu(tpos, pos, member)		\
	for (pos = rcu_dereference((pos)->next);			\
	     pos && ({ prefetch(pos->next); 1; }) &&			\
	     ({ tpos = hlist_entry(pos, typeof(*tpos), member); 1; });  \
	     pos = rcu_dereference(pos->next))

#define hlist_for_each_entry_continue_rcu_bh(tpos, pos, member)		\
	for (pos = rcu_dereference_bh((pos)->next);			\
	     pos && ({ prefetch(pos->next); 1; }) &&			\
	     ({ tpos = hlist_entry(pos, typeof(*tpos), member); 1; });  \
	     pos = rcu_dereference_bh(pos->next))


#endif	/* __KERNEL__ */
#endif

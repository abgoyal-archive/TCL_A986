
#ifndef _LINUX_RCULIST_NULLS_H
#define _LINUX_RCULIST_NULLS_H

#ifdef __KERNEL__

#include <linux/list_nulls.h>
#include <linux/rcupdate.h>

static inline void hlist_nulls_del_init_rcu(struct hlist_nulls_node *n)
{
	if (!hlist_nulls_unhashed(n)) {
		__hlist_nulls_del(n);
		n->pprev = NULL;
	}
}

static inline void hlist_nulls_del_rcu(struct hlist_nulls_node *n)
{
	__hlist_nulls_del(n);
	n->pprev = LIST_POISON2;
}

static inline void hlist_nulls_add_head_rcu(struct hlist_nulls_node *n,
					struct hlist_nulls_head *h)
{
	struct hlist_nulls_node *first = h->first;

	n->next = first;
	n->pprev = &h->first;
	rcu_assign_pointer(h->first, n);
	if (!is_a_nulls(first))
		first->pprev = &n->next;
}
#define hlist_nulls_for_each_entry_rcu(tpos, pos, head, member) \
	for (pos = rcu_dereference_raw((head)->first);			 \
		(!is_a_nulls(pos)) &&			\
		({ tpos = hlist_nulls_entry(pos, typeof(*tpos), member); 1; }); \
		pos = rcu_dereference_raw(pos->next))

#endif
#endif

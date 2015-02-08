

#include <linux/plist.h>
#include <linux/spinlock.h>

#ifdef CONFIG_DEBUG_PI_LIST

static void plist_check_prev_next(struct list_head *t, struct list_head *p,
				  struct list_head *n)
{
	WARN(n->prev != p || p->next != n,
			"top: %p, n: %p, p: %p\n"
			"prev: %p, n: %p, p: %p\n"
			"next: %p, n: %p, p: %p\n",
			 t, t->next, t->prev,
			p, p->next, p->prev,
			n, n->next, n->prev);
}

static void plist_check_list(struct list_head *top)
{
	struct list_head *prev = top, *next = top->next;

	plist_check_prev_next(top, prev, next);
	while (next != top) {
		prev = next;
		next = prev->next;
		plist_check_prev_next(top, prev, next);
	}
}

static void plist_check_head(struct plist_head *head)
{
	WARN_ON(!head->rawlock && !head->spinlock);
	if (head->rawlock)
		WARN_ON_SMP(!raw_spin_is_locked(head->rawlock));
	if (head->spinlock)
		WARN_ON_SMP(!spin_is_locked(head->spinlock));
	plist_check_list(&head->prio_list);
	plist_check_list(&head->node_list);
}

#else
# define plist_check_head(h)	do { } while (0)
#endif

void plist_add(struct plist_node *node, struct plist_head *head)
{
	struct plist_node *iter;

	plist_check_head(head);
	WARN_ON(!plist_node_empty(node));

	list_for_each_entry(iter, &head->prio_list, plist.prio_list) {
		if (node->prio < iter->prio)
			goto lt_prio;
		else if (node->prio == iter->prio) {
			iter = list_entry(iter->plist.prio_list.next,
					struct plist_node, plist.prio_list);
			goto eq_prio;
		}
	}

lt_prio:
	list_add_tail(&node->plist.prio_list, &iter->plist.prio_list);
eq_prio:
	list_add_tail(&node->plist.node_list, &iter->plist.node_list);

	plist_check_head(head);
}

void plist_del(struct plist_node *node, struct plist_head *head)
{
	plist_check_head(head);

	if (!list_empty(&node->plist.prio_list)) {
		struct plist_node *next = plist_first(&node->plist);

		list_move_tail(&next->plist.prio_list, &node->plist.prio_list);
		list_del_init(&node->plist.prio_list);
	}

	list_del_init(&node->plist.node_list);

	plist_check_head(head);
}

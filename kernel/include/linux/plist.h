
#ifndef _LINUX_PLIST_H_
#define _LINUX_PLIST_H_

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock_types.h>

struct plist_head {
	struct list_head prio_list;
	struct list_head node_list;
#ifdef CONFIG_DEBUG_PI_LIST
	raw_spinlock_t *rawlock;
	spinlock_t *spinlock;
#endif
};

struct plist_node {
	int			prio;
	struct plist_head	plist;
};

#ifdef CONFIG_DEBUG_PI_LIST
# define PLIST_HEAD_LOCK_INIT(_lock)		.spinlock = _lock
# define PLIST_HEAD_LOCK_INIT_RAW(_lock)	.rawlock = _lock
#else
# define PLIST_HEAD_LOCK_INIT(_lock)
# define PLIST_HEAD_LOCK_INIT_RAW(_lock)
#endif

#define _PLIST_HEAD_INIT(head)				\
	.prio_list = LIST_HEAD_INIT((head).prio_list),	\
	.node_list = LIST_HEAD_INIT((head).node_list)

#define PLIST_HEAD_INIT(head, _lock)			\
{							\
	_PLIST_HEAD_INIT(head),				\
	PLIST_HEAD_LOCK_INIT(&(_lock))			\
}

#define PLIST_HEAD_INIT_RAW(head, _lock)		\
{							\
	_PLIST_HEAD_INIT(head),				\
	PLIST_HEAD_LOCK_INIT_RAW(&(_lock))		\
}

#define PLIST_NODE_INIT(node, __prio)			\
{							\
	.prio  = (__prio),				\
	.plist = { _PLIST_HEAD_INIT((node).plist) },	\
}

static inline void
plist_head_init(struct plist_head *head, spinlock_t *lock)
{
	INIT_LIST_HEAD(&head->prio_list);
	INIT_LIST_HEAD(&head->node_list);
#ifdef CONFIG_DEBUG_PI_LIST
	head->spinlock = lock;
	head->rawlock = NULL;
#endif
}

static inline void
plist_head_init_raw(struct plist_head *head, raw_spinlock_t *lock)
{
	INIT_LIST_HEAD(&head->prio_list);
	INIT_LIST_HEAD(&head->node_list);
#ifdef CONFIG_DEBUG_PI_LIST
	head->rawlock = lock;
	head->spinlock = NULL;
#endif
}

static inline void plist_node_init(struct plist_node *node, int prio)
{
	node->prio = prio;
	plist_head_init(&node->plist, NULL);
}

extern void plist_add(struct plist_node *node, struct plist_head *head);
extern void plist_del(struct plist_node *node, struct plist_head *head);

#define plist_for_each(pos, head)	\
	 list_for_each_entry(pos, &(head)->node_list, plist.node_list)

#define plist_for_each_safe(pos, n, head)	\
	 list_for_each_entry_safe(pos, n, &(head)->node_list, plist.node_list)

#define plist_for_each_entry(pos, head, mem)	\
	 list_for_each_entry(pos, &(head)->node_list, mem.plist.node_list)

#define plist_for_each_entry_safe(pos, n, head, m)	\
	list_for_each_entry_safe(pos, n, &(head)->node_list, m.plist.node_list)

static inline int plist_head_empty(const struct plist_head *head)
{
	return list_empty(&head->node_list);
}

static inline int plist_node_empty(const struct plist_node *node)
{
	return plist_head_empty(&node->plist);
}

/* All functions below assume the plist_head is not empty. */

#ifdef CONFIG_DEBUG_PI_LIST
# define plist_first_entry(head, type, member)	\
({ \
	WARN_ON(plist_head_empty(head)); \
	container_of(plist_first(head), type, member); \
})
#else
# define plist_first_entry(head, type, member)	\
	container_of(plist_first(head), type, member)
#endif

static inline struct plist_node *plist_first(const struct plist_head *head)
{
	return list_entry(head->node_list.next,
			  struct plist_node, plist.node_list);
}

#endif

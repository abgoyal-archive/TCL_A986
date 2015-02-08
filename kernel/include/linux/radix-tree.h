
#ifndef _LINUX_RADIX_TREE_H
#define _LINUX_RADIX_TREE_H

#include <linux/preempt.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/rcupdate.h>

#define RADIX_TREE_INDIRECT_PTR	1
#define RADIX_TREE_RETRY ((void *)-1UL)

static inline void *radix_tree_ptr_to_indirect(void *ptr)
{
	return (void *)((unsigned long)ptr | RADIX_TREE_INDIRECT_PTR);
}

static inline void *radix_tree_indirect_to_ptr(void *ptr)
{
	return (void *)((unsigned long)ptr & ~RADIX_TREE_INDIRECT_PTR);
}

static inline int radix_tree_is_indirect_ptr(void *ptr)
{
	return (int)((unsigned long)ptr & RADIX_TREE_INDIRECT_PTR);
}

/*** radix-tree API starts here ***/

#define RADIX_TREE_MAX_TAGS 2

/* root tags are stored in gfp_mask, shifted by __GFP_BITS_SHIFT */
struct radix_tree_root {
	unsigned int		height;
	gfp_t			gfp_mask;
	struct radix_tree_node	*rnode;
};

#define RADIX_TREE_INIT(mask)	{					\
	.height = 0,							\
	.gfp_mask = (mask),						\
	.rnode = NULL,							\
}

#define RADIX_TREE(name, mask) \
	struct radix_tree_root name = RADIX_TREE_INIT(mask)

#define INIT_RADIX_TREE(root, mask)					\
do {									\
	(root)->height = 0;						\
	(root)->gfp_mask = (mask);					\
	(root)->rnode = NULL;						\
} while (0)


static inline void *radix_tree_deref_slot(void **pslot)
{
	void *ret = rcu_dereference(*pslot);
	if (unlikely(radix_tree_is_indirect_ptr(ret)))
		ret = RADIX_TREE_RETRY;
	return ret;
}
static inline void radix_tree_replace_slot(void **pslot, void *item)
{
	BUG_ON(radix_tree_is_indirect_ptr(item));
	rcu_assign_pointer(*pslot, item);
}

int radix_tree_insert(struct radix_tree_root *, unsigned long, void *);
void *radix_tree_lookup(struct radix_tree_root *, unsigned long);
void **radix_tree_lookup_slot(struct radix_tree_root *, unsigned long);
void *radix_tree_delete(struct radix_tree_root *, unsigned long);
unsigned int
radix_tree_gang_lookup(struct radix_tree_root *root, void **results,
			unsigned long first_index, unsigned int max_items);
unsigned int
radix_tree_gang_lookup_slot(struct radix_tree_root *root, void ***results,
			unsigned long first_index, unsigned int max_items);
unsigned long radix_tree_next_hole(struct radix_tree_root *root,
				unsigned long index, unsigned long max_scan);
unsigned long radix_tree_prev_hole(struct radix_tree_root *root,
				unsigned long index, unsigned long max_scan);
int radix_tree_preload(gfp_t gfp_mask);
void radix_tree_init(void);
void *radix_tree_tag_set(struct radix_tree_root *root,
			unsigned long index, unsigned int tag);
void *radix_tree_tag_clear(struct radix_tree_root *root,
			unsigned long index, unsigned int tag);
int radix_tree_tag_get(struct radix_tree_root *root,
			unsigned long index, unsigned int tag);
unsigned int
radix_tree_gang_lookup_tag(struct radix_tree_root *root, void **results,
		unsigned long first_index, unsigned int max_items,
		unsigned int tag);
unsigned int
radix_tree_gang_lookup_tag_slot(struct radix_tree_root *root, void ***results,
		unsigned long first_index, unsigned int max_items,
		unsigned int tag);
int radix_tree_tagged(struct radix_tree_root *root, unsigned int tag);

static inline void radix_tree_preload_end(void)
{
	preempt_enable();
}

#endif /* _LINUX_RADIX_TREE_H */

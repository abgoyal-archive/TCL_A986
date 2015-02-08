

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/radix-tree.h>
#include <linux/percpu.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/cpu.h>
#include <linux/string.h>
#include <linux/bitops.h>
#include <linux/rcupdate.h>


#ifdef __KERNEL__
#define RADIX_TREE_MAP_SHIFT	(CONFIG_BASE_SMALL ? 4 : 6)
#else
#define RADIX_TREE_MAP_SHIFT	3	/* For more stressful testing */
#endif

#define RADIX_TREE_MAP_SIZE	(1UL << RADIX_TREE_MAP_SHIFT)
#define RADIX_TREE_MAP_MASK	(RADIX_TREE_MAP_SIZE-1)

#define RADIX_TREE_TAG_LONGS	\
	((RADIX_TREE_MAP_SIZE + BITS_PER_LONG - 1) / BITS_PER_LONG)

struct radix_tree_node {
	unsigned int	height;		/* Height from the bottom */
	unsigned int	count;
	struct rcu_head	rcu_head;
	void		*slots[RADIX_TREE_MAP_SIZE];
	unsigned long	tags[RADIX_TREE_MAX_TAGS][RADIX_TREE_TAG_LONGS];
};

struct radix_tree_path {
	struct radix_tree_node *node;
	int offset;
};

#define RADIX_TREE_INDEX_BITS  (8 /* CHAR_BIT */ * sizeof(unsigned long))
#define RADIX_TREE_MAX_PATH (DIV_ROUND_UP(RADIX_TREE_INDEX_BITS, \
					  RADIX_TREE_MAP_SHIFT))

static unsigned long height_to_maxindex[RADIX_TREE_MAX_PATH + 1] __read_mostly;

static struct kmem_cache *radix_tree_node_cachep;

struct radix_tree_preload {
	int nr;
	struct radix_tree_node *nodes[RADIX_TREE_MAX_PATH];
};
static DEFINE_PER_CPU(struct radix_tree_preload, radix_tree_preloads) = { 0, };

static inline gfp_t root_gfp_mask(struct radix_tree_root *root)
{
	return root->gfp_mask & __GFP_BITS_MASK;
}

static inline void tag_set(struct radix_tree_node *node, unsigned int tag,
		int offset)
{
	__set_bit(offset, node->tags[tag]);
}

static inline void tag_clear(struct radix_tree_node *node, unsigned int tag,
		int offset)
{
	__clear_bit(offset, node->tags[tag]);
}

static inline int tag_get(struct radix_tree_node *node, unsigned int tag,
		int offset)
{
	return test_bit(offset, node->tags[tag]);
}

static inline void root_tag_set(struct radix_tree_root *root, unsigned int tag)
{
	root->gfp_mask |= (__force gfp_t)(1 << (tag + __GFP_BITS_SHIFT));
}

static inline void root_tag_clear(struct radix_tree_root *root, unsigned int tag)
{
	root->gfp_mask &= (__force gfp_t)~(1 << (tag + __GFP_BITS_SHIFT));
}

static inline void root_tag_clear_all(struct radix_tree_root *root)
{
	root->gfp_mask &= __GFP_BITS_MASK;
}

static inline int root_tag_get(struct radix_tree_root *root, unsigned int tag)
{
	return (__force unsigned)root->gfp_mask & (1 << (tag + __GFP_BITS_SHIFT));
}

static inline int any_tag_set(struct radix_tree_node *node, unsigned int tag)
{
	int idx;
	for (idx = 0; idx < RADIX_TREE_TAG_LONGS; idx++) {
		if (node->tags[tag][idx])
			return 1;
	}
	return 0;
}
static struct radix_tree_node *
radix_tree_node_alloc(struct radix_tree_root *root)
{
	struct radix_tree_node *ret = NULL;
	gfp_t gfp_mask = root_gfp_mask(root);

	if (!(gfp_mask & __GFP_WAIT)) {
		struct radix_tree_preload *rtp;

		/*
		 * Provided the caller has preloaded here, we will always
		 * succeed in getting a node here (and never reach
		 * kmem_cache_alloc)
		 */
		rtp = &__get_cpu_var(radix_tree_preloads);
		if (rtp->nr) {
			ret = rtp->nodes[rtp->nr - 1];
			rtp->nodes[rtp->nr - 1] = NULL;
			rtp->nr--;
		}
	}
	if (ret == NULL)
		ret = kmem_cache_alloc(radix_tree_node_cachep, gfp_mask);

	BUG_ON(radix_tree_is_indirect_ptr(ret));
	return ret;
}

static void radix_tree_node_rcu_free(struct rcu_head *head)
{
	struct radix_tree_node *node =
			container_of(head, struct radix_tree_node, rcu_head);

	/*
	 * must only free zeroed nodes into the slab. radix_tree_shrink
	 * can leave us with a non-NULL entry in the first slot, so clear
	 * that here to make sure.
	 */
	tag_clear(node, 0, 0);
	tag_clear(node, 1, 0);
	node->slots[0] = NULL;
	node->count = 0;

	kmem_cache_free(radix_tree_node_cachep, node);
}

static inline void
radix_tree_node_free(struct radix_tree_node *node)
{
	call_rcu(&node->rcu_head, radix_tree_node_rcu_free);
}

int radix_tree_preload(gfp_t gfp_mask)
{
	struct radix_tree_preload *rtp;
	struct radix_tree_node *node;
	int ret = -ENOMEM;

	preempt_disable();
	rtp = &__get_cpu_var(radix_tree_preloads);
	while (rtp->nr < ARRAY_SIZE(rtp->nodes)) {
		preempt_enable();
		node = kmem_cache_alloc(radix_tree_node_cachep, gfp_mask);
		if (node == NULL)
			goto out;
		preempt_disable();
		rtp = &__get_cpu_var(radix_tree_preloads);
		if (rtp->nr < ARRAY_SIZE(rtp->nodes))
			rtp->nodes[rtp->nr++] = node;
		else
			kmem_cache_free(radix_tree_node_cachep, node);
	}
	ret = 0;
out:
	return ret;
}
EXPORT_SYMBOL(radix_tree_preload);

static inline unsigned long radix_tree_maxindex(unsigned int height)
{
	return height_to_maxindex[height];
}

static int radix_tree_extend(struct radix_tree_root *root, unsigned long index)
{
	struct radix_tree_node *node;
	unsigned int height;
	int tag;

	/* Figure out what the height should be.  */
	height = root->height + 1;
	while (index > radix_tree_maxindex(height))
		height++;

	if (root->rnode == NULL) {
		root->height = height;
		goto out;
	}

	do {
		unsigned int newheight;
		if (!(node = radix_tree_node_alloc(root)))
			return -ENOMEM;

		/* Increase the height.  */
		node->slots[0] = radix_tree_indirect_to_ptr(root->rnode);

		/* Propagate the aggregated tag info into the new root */
		for (tag = 0; tag < RADIX_TREE_MAX_TAGS; tag++) {
			if (root_tag_get(root, tag))
				tag_set(node, tag, 0);
		}

		newheight = root->height+1;
		node->height = newheight;
		node->count = 1;
		node = radix_tree_ptr_to_indirect(node);
		rcu_assign_pointer(root->rnode, node);
		root->height = newheight;
	} while (height > root->height);
out:
	return 0;
}

int radix_tree_insert(struct radix_tree_root *root,
			unsigned long index, void *item)
{
	struct radix_tree_node *node = NULL, *slot;
	unsigned int height, shift;
	int offset;
	int error;

	BUG_ON(radix_tree_is_indirect_ptr(item));

	/* Make sure the tree is high enough.  */
	if (index > radix_tree_maxindex(root->height)) {
		error = radix_tree_extend(root, index);
		if (error)
			return error;
	}

	slot = radix_tree_indirect_to_ptr(root->rnode);

	height = root->height;
	shift = (height-1) * RADIX_TREE_MAP_SHIFT;

	offset = 0;			/* uninitialised var warning */
	while (height > 0) {
		if (slot == NULL) {
			/* Have to add a child node.  */
			if (!(slot = radix_tree_node_alloc(root)))
				return -ENOMEM;
			slot->height = height;
			if (node) {
				rcu_assign_pointer(node->slots[offset], slot);
				node->count++;
			} else
				rcu_assign_pointer(root->rnode,
					radix_tree_ptr_to_indirect(slot));
		}

		/* Go a level down */
		offset = (index >> shift) & RADIX_TREE_MAP_MASK;
		node = slot;
		slot = node->slots[offset];
		shift -= RADIX_TREE_MAP_SHIFT;
		height--;
	}

	if (slot != NULL)
		return -EEXIST;

	if (node) {
		node->count++;
		rcu_assign_pointer(node->slots[offset], item);
		BUG_ON(tag_get(node, 0, offset));
		BUG_ON(tag_get(node, 1, offset));
	} else {
		rcu_assign_pointer(root->rnode, item);
		BUG_ON(root_tag_get(root, 0));
		BUG_ON(root_tag_get(root, 1));
	}

	return 0;
}
EXPORT_SYMBOL(radix_tree_insert);

static void *radix_tree_lookup_element(struct radix_tree_root *root,
				unsigned long index, int is_slot)
{
	unsigned int height, shift;
	struct radix_tree_node *node, **slot;

	node = rcu_dereference_raw(root->rnode);
	if (node == NULL)
		return NULL;

	if (!radix_tree_is_indirect_ptr(node)) {
		if (index > 0)
			return NULL;
		return is_slot ? (void *)&root->rnode : node;
	}
	node = radix_tree_indirect_to_ptr(node);

	height = node->height;
	if (index > radix_tree_maxindex(height))
		return NULL;

	shift = (height-1) * RADIX_TREE_MAP_SHIFT;

	do {
		slot = (struct radix_tree_node **)
			(node->slots + ((index>>shift) & RADIX_TREE_MAP_MASK));
		node = rcu_dereference_raw(*slot);
		if (node == NULL)
			return NULL;

		shift -= RADIX_TREE_MAP_SHIFT;
		height--;
	} while (height > 0);

	return is_slot ? (void *)slot:node;
}

void **radix_tree_lookup_slot(struct radix_tree_root *root, unsigned long index)
{
	return (void **)radix_tree_lookup_element(root, index, 1);
}
EXPORT_SYMBOL(radix_tree_lookup_slot);

void *radix_tree_lookup(struct radix_tree_root *root, unsigned long index)
{
	return radix_tree_lookup_element(root, index, 0);
}
EXPORT_SYMBOL(radix_tree_lookup);

void *radix_tree_tag_set(struct radix_tree_root *root,
			unsigned long index, unsigned int tag)
{
	unsigned int height, shift;
	struct radix_tree_node *slot;

	height = root->height;
	BUG_ON(index > radix_tree_maxindex(height));

	slot = radix_tree_indirect_to_ptr(root->rnode);
	shift = (height - 1) * RADIX_TREE_MAP_SHIFT;

	while (height > 0) {
		int offset;

		offset = (index >> shift) & RADIX_TREE_MAP_MASK;
		if (!tag_get(slot, tag, offset))
			tag_set(slot, tag, offset);
		slot = slot->slots[offset];
		BUG_ON(slot == NULL);
		shift -= RADIX_TREE_MAP_SHIFT;
		height--;
	}

	/* set the root's tag bit */
	if (slot && !root_tag_get(root, tag))
		root_tag_set(root, tag);

	return slot;
}
EXPORT_SYMBOL(radix_tree_tag_set);

void *radix_tree_tag_clear(struct radix_tree_root *root,
			unsigned long index, unsigned int tag)
{
	/*
	 * The radix tree path needs to be one longer than the maximum path
	 * since the "list" is null terminated.
	 */
	struct radix_tree_path path[RADIX_TREE_MAX_PATH + 1], *pathp = path;
	struct radix_tree_node *slot = NULL;
	unsigned int height, shift;

	height = root->height;
	if (index > radix_tree_maxindex(height))
		goto out;

	shift = (height - 1) * RADIX_TREE_MAP_SHIFT;
	pathp->node = NULL;
	slot = radix_tree_indirect_to_ptr(root->rnode);

	while (height > 0) {
		int offset;

		if (slot == NULL)
			goto out;

		offset = (index >> shift) & RADIX_TREE_MAP_MASK;
		pathp[1].offset = offset;
		pathp[1].node = slot;
		slot = slot->slots[offset];
		pathp++;
		shift -= RADIX_TREE_MAP_SHIFT;
		height--;
	}

	if (slot == NULL)
		goto out;

	while (pathp->node) {
		if (!tag_get(pathp->node, tag, pathp->offset))
			goto out;
		tag_clear(pathp->node, tag, pathp->offset);
		if (any_tag_set(pathp->node, tag))
			goto out;
		pathp--;
	}

	/* clear the root's tag bit */
	if (root_tag_get(root, tag))
		root_tag_clear(root, tag);

out:
	return slot;
}
EXPORT_SYMBOL(radix_tree_tag_clear);

int radix_tree_tag_get(struct radix_tree_root *root,
			unsigned long index, unsigned int tag)
{
	unsigned int height, shift;
	struct radix_tree_node *node;
	int saw_unset_tag = 0;

	/* check the root's tag bit */
	if (!root_tag_get(root, tag))
		return 0;

	node = rcu_dereference_raw(root->rnode);
	if (node == NULL)
		return 0;

	if (!radix_tree_is_indirect_ptr(node))
		return (index == 0);
	node = radix_tree_indirect_to_ptr(node);

	height = node->height;
	if (index > radix_tree_maxindex(height))
		return 0;

	shift = (height - 1) * RADIX_TREE_MAP_SHIFT;

	for ( ; ; ) {
		int offset;

		if (node == NULL)
			return 0;

		offset = (index >> shift) & RADIX_TREE_MAP_MASK;

		/*
		 * This is just a debug check.  Later, we can bale as soon as
		 * we see an unset tag.
		 */
		if (!tag_get(node, tag, offset))
			saw_unset_tag = 1;
		if (height == 1)
			return !!tag_get(node, tag, offset);
		node = rcu_dereference_raw(node->slots[offset]);
		shift -= RADIX_TREE_MAP_SHIFT;
		height--;
	}
}
EXPORT_SYMBOL(radix_tree_tag_get);

unsigned long radix_tree_next_hole(struct radix_tree_root *root,
				unsigned long index, unsigned long max_scan)
{
	unsigned long i;

	for (i = 0; i < max_scan; i++) {
		if (!radix_tree_lookup(root, index))
			break;
		index++;
		if (index == 0)
			break;
	}

	return index;
}
EXPORT_SYMBOL(radix_tree_next_hole);

unsigned long radix_tree_prev_hole(struct radix_tree_root *root,
				   unsigned long index, unsigned long max_scan)
{
	unsigned long i;

	for (i = 0; i < max_scan; i++) {
		if (!radix_tree_lookup(root, index))
			break;
		index--;
		if (index == ULONG_MAX)
			break;
	}

	return index;
}
EXPORT_SYMBOL(radix_tree_prev_hole);

static unsigned int
__lookup(struct radix_tree_node *slot, void ***results, unsigned long index,
	unsigned int max_items, unsigned long *next_index)
{
	unsigned int nr_found = 0;
	unsigned int shift, height;
	unsigned long i;

	height = slot->height;
	if (height == 0)
		goto out;
	shift = (height-1) * RADIX_TREE_MAP_SHIFT;

	for ( ; height > 1; height--) {
		i = (index >> shift) & RADIX_TREE_MAP_MASK;
		for (;;) {
			if (slot->slots[i] != NULL)
				break;
			index &= ~((1UL << shift) - 1);
			index += 1UL << shift;
			if (index == 0)
				goto out;	/* 32-bit wraparound */
			i++;
			if (i == RADIX_TREE_MAP_SIZE)
				goto out;
		}

		shift -= RADIX_TREE_MAP_SHIFT;
		slot = rcu_dereference_raw(slot->slots[i]);
		if (slot == NULL)
			goto out;
	}

	/* Bottom level: grab some items */
	for (i = index & RADIX_TREE_MAP_MASK; i < RADIX_TREE_MAP_SIZE; i++) {
		index++;
		if (slot->slots[i]) {
			results[nr_found++] = &(slot->slots[i]);
			if (nr_found == max_items)
				goto out;
		}
	}
out:
	*next_index = index;
	return nr_found;
}

unsigned int
radix_tree_gang_lookup(struct radix_tree_root *root, void **results,
			unsigned long first_index, unsigned int max_items)
{
	unsigned long max_index;
	struct radix_tree_node *node;
	unsigned long cur_index = first_index;
	unsigned int ret;

	node = rcu_dereference_raw(root->rnode);
	if (!node)
		return 0;

	if (!radix_tree_is_indirect_ptr(node)) {
		if (first_index > 0)
			return 0;
		results[0] = node;
		return 1;
	}
	node = radix_tree_indirect_to_ptr(node);

	max_index = radix_tree_maxindex(node->height);

	ret = 0;
	while (ret < max_items) {
		unsigned int nr_found, slots_found, i;
		unsigned long next_index;	/* Index of next search */

		if (cur_index > max_index)
			break;
		slots_found = __lookup(node, (void ***)results + ret, cur_index,
					max_items - ret, &next_index);
		nr_found = 0;
		for (i = 0; i < slots_found; i++) {
			struct radix_tree_node *slot;
			slot = *(((void ***)results)[ret + i]);
			if (!slot)
				continue;
			results[ret + nr_found] = rcu_dereference_raw(slot);
			nr_found++;
		}
		ret += nr_found;
		if (next_index == 0)
			break;
		cur_index = next_index;
	}

	return ret;
}
EXPORT_SYMBOL(radix_tree_gang_lookup);

unsigned int
radix_tree_gang_lookup_slot(struct radix_tree_root *root, void ***results,
			unsigned long first_index, unsigned int max_items)
{
	unsigned long max_index;
	struct radix_tree_node *node;
	unsigned long cur_index = first_index;
	unsigned int ret;

	node = rcu_dereference_raw(root->rnode);
	if (!node)
		return 0;

	if (!radix_tree_is_indirect_ptr(node)) {
		if (first_index > 0)
			return 0;
		results[0] = (void **)&root->rnode;
		return 1;
	}
	node = radix_tree_indirect_to_ptr(node);

	max_index = radix_tree_maxindex(node->height);

	ret = 0;
	while (ret < max_items) {
		unsigned int slots_found;
		unsigned long next_index;	/* Index of next search */

		if (cur_index > max_index)
			break;
		slots_found = __lookup(node, results + ret, cur_index,
					max_items - ret, &next_index);
		ret += slots_found;
		if (next_index == 0)
			break;
		cur_index = next_index;
	}

	return ret;
}
EXPORT_SYMBOL(radix_tree_gang_lookup_slot);

static unsigned int
__lookup_tag(struct radix_tree_node *slot, void ***results, unsigned long index,
	unsigned int max_items, unsigned long *next_index, unsigned int tag)
{
	unsigned int nr_found = 0;
	unsigned int shift, height;

	height = slot->height;
	if (height == 0)
		goto out;
	shift = (height-1) * RADIX_TREE_MAP_SHIFT;

	while (height > 0) {
		unsigned long i = (index >> shift) & RADIX_TREE_MAP_MASK ;

		for (;;) {
			if (tag_get(slot, tag, i))
				break;
			index &= ~((1UL << shift) - 1);
			index += 1UL << shift;
			if (index == 0)
				goto out;	/* 32-bit wraparound */
			i++;
			if (i == RADIX_TREE_MAP_SIZE)
				goto out;
		}
		height--;
		if (height == 0) {	/* Bottom level: grab some items */
			unsigned long j = index & RADIX_TREE_MAP_MASK;

			for ( ; j < RADIX_TREE_MAP_SIZE; j++) {
				index++;
				if (!tag_get(slot, tag, j))
					continue;
				/*
				 * Even though the tag was found set, we need to
				 * recheck that we have a non-NULL node, because
				 * if this lookup is lockless, it may have been
				 * subsequently deleted.
				 *
				 * Similar care must be taken in any place that
				 * lookup ->slots[x] without a lock (ie. can't
				 * rely on its value remaining the same).
				 */
				if (slot->slots[j]) {
					results[nr_found++] = &(slot->slots[j]);
					if (nr_found == max_items)
						goto out;
				}
			}
		}
		shift -= RADIX_TREE_MAP_SHIFT;
		slot = rcu_dereference_raw(slot->slots[i]);
		if (slot == NULL)
			break;
	}
out:
	*next_index = index;
	return nr_found;
}

unsigned int
radix_tree_gang_lookup_tag(struct radix_tree_root *root, void **results,
		unsigned long first_index, unsigned int max_items,
		unsigned int tag)
{
	struct radix_tree_node *node;
	unsigned long max_index;
	unsigned long cur_index = first_index;
	unsigned int ret;

	/* check the root's tag bit */
	if (!root_tag_get(root, tag))
		return 0;

	node = rcu_dereference_raw(root->rnode);
	if (!node)
		return 0;

	if (!radix_tree_is_indirect_ptr(node)) {
		if (first_index > 0)
			return 0;
		results[0] = node;
		return 1;
	}
	node = radix_tree_indirect_to_ptr(node);

	max_index = radix_tree_maxindex(node->height);

	ret = 0;
	while (ret < max_items) {
		unsigned int nr_found, slots_found, i;
		unsigned long next_index;	/* Index of next search */

		if (cur_index > max_index)
			break;
		slots_found = __lookup_tag(node, (void ***)results + ret,
				cur_index, max_items - ret, &next_index, tag);
		nr_found = 0;
		for (i = 0; i < slots_found; i++) {
			struct radix_tree_node *slot;
			slot = *(((void ***)results)[ret + i]);
			if (!slot)
				continue;
			results[ret + nr_found] = rcu_dereference_raw(slot);
			nr_found++;
		}
		ret += nr_found;
		if (next_index == 0)
			break;
		cur_index = next_index;
	}

	return ret;
}
EXPORT_SYMBOL(radix_tree_gang_lookup_tag);

unsigned int
radix_tree_gang_lookup_tag_slot(struct radix_tree_root *root, void ***results,
		unsigned long first_index, unsigned int max_items,
		unsigned int tag)
{
	struct radix_tree_node *node;
	unsigned long max_index;
	unsigned long cur_index = first_index;
	unsigned int ret;

	/* check the root's tag bit */
	if (!root_tag_get(root, tag))
		return 0;

	node = rcu_dereference_raw(root->rnode);
	if (!node)
		return 0;

	if (!radix_tree_is_indirect_ptr(node)) {
		if (first_index > 0)
			return 0;
		results[0] = (void **)&root->rnode;
		return 1;
	}
	node = radix_tree_indirect_to_ptr(node);

	max_index = radix_tree_maxindex(node->height);

	ret = 0;
	while (ret < max_items) {
		unsigned int slots_found;
		unsigned long next_index;	/* Index of next search */

		if (cur_index > max_index)
			break;
		slots_found = __lookup_tag(node, results + ret,
				cur_index, max_items - ret, &next_index, tag);
		ret += slots_found;
		if (next_index == 0)
			break;
		cur_index = next_index;
	}

	return ret;
}
EXPORT_SYMBOL(radix_tree_gang_lookup_tag_slot);


static inline void radix_tree_shrink(struct radix_tree_root *root)
{
	/* try to shrink tree height */
	while (root->height > 0) {
		struct radix_tree_node *to_free = root->rnode;
		void *newptr;

		BUG_ON(!radix_tree_is_indirect_ptr(to_free));
		to_free = radix_tree_indirect_to_ptr(to_free);

		/*
		 * The candidate node has more than one child, or its child
		 * is not at the leftmost slot, we cannot shrink.
		 */
		if (to_free->count != 1)
			break;
		if (!to_free->slots[0])
			break;

		/*
		 * We don't need rcu_assign_pointer(), since we are simply
		 * moving the node from one part of the tree to another. If
		 * it was safe to dereference the old pointer to it
		 * (to_free->slots[0]), it will be safe to dereference the new
		 * one (root->rnode).
		 */
		newptr = to_free->slots[0];
		if (root->height > 1)
			newptr = radix_tree_ptr_to_indirect(newptr);
		root->rnode = newptr;
		root->height--;
		radix_tree_node_free(to_free);
	}
}

void *radix_tree_delete(struct radix_tree_root *root, unsigned long index)
{
	/*
	 * The radix tree path needs to be one longer than the maximum path
	 * since the "list" is null terminated.
	 */
	struct radix_tree_path path[RADIX_TREE_MAX_PATH + 1], *pathp = path;
	struct radix_tree_node *slot = NULL;
	struct radix_tree_node *to_free;
	unsigned int height, shift;
	int tag;
	int offset;

	height = root->height;
	if (index > radix_tree_maxindex(height))
		goto out;

	slot = root->rnode;
	if (height == 0) {
		root_tag_clear_all(root);
		root->rnode = NULL;
		goto out;
	}
	slot = radix_tree_indirect_to_ptr(slot);

	shift = (height - 1) * RADIX_TREE_MAP_SHIFT;
	pathp->node = NULL;

	do {
		if (slot == NULL)
			goto out;

		pathp++;
		offset = (index >> shift) & RADIX_TREE_MAP_MASK;
		pathp->offset = offset;
		pathp->node = slot;
		slot = slot->slots[offset];
		shift -= RADIX_TREE_MAP_SHIFT;
		height--;
	} while (height > 0);

	if (slot == NULL)
		goto out;

	/*
	 * Clear all tags associated with the just-deleted item
	 */
	for (tag = 0; tag < RADIX_TREE_MAX_TAGS; tag++) {
		if (tag_get(pathp->node, tag, pathp->offset))
			radix_tree_tag_clear(root, index, tag);
	}

	to_free = NULL;
	/* Now free the nodes we do not need anymore */
	while (pathp->node) {
		pathp->node->slots[pathp->offset] = NULL;
		pathp->node->count--;
		/*
		 * Queue the node for deferred freeing after the
		 * last reference to it disappears (set NULL, above).
		 */
		if (to_free)
			radix_tree_node_free(to_free);

		if (pathp->node->count) {
			if (pathp->node ==
					radix_tree_indirect_to_ptr(root->rnode))
				radix_tree_shrink(root);
			goto out;
		}

		/* Node with zero slots in use so free it */
		to_free = pathp->node;
		pathp--;

	}
	root_tag_clear_all(root);
	root->height = 0;
	root->rnode = NULL;
	if (to_free)
		radix_tree_node_free(to_free);

out:
	return slot;
}
EXPORT_SYMBOL(radix_tree_delete);

int radix_tree_tagged(struct radix_tree_root *root, unsigned int tag)
{
	return root_tag_get(root, tag);
}
EXPORT_SYMBOL(radix_tree_tagged);

static void
radix_tree_node_ctor(void *node)
{
	memset(node, 0, sizeof(struct radix_tree_node));
}

static __init unsigned long __maxindex(unsigned int height)
{
	unsigned int width = height * RADIX_TREE_MAP_SHIFT;
	int shift = RADIX_TREE_INDEX_BITS - width;

	if (shift < 0)
		return ~0UL;
	if (shift >= BITS_PER_LONG)
		return 0UL;
	return ~0UL >> shift;
}

static __init void radix_tree_init_maxindex(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(height_to_maxindex); i++)
		height_to_maxindex[i] = __maxindex(i);
}

static int radix_tree_callback(struct notifier_block *nfb,
                            unsigned long action,
                            void *hcpu)
{
       int cpu = (long)hcpu;
       struct radix_tree_preload *rtp;

       /* Free per-cpu pool of perloaded nodes */
       if (action == CPU_DEAD || action == CPU_DEAD_FROZEN) {
               rtp = &per_cpu(radix_tree_preloads, cpu);
               while (rtp->nr) {
                       kmem_cache_free(radix_tree_node_cachep,
                                       rtp->nodes[rtp->nr-1]);
                       rtp->nodes[rtp->nr-1] = NULL;
                       rtp->nr--;
               }
       }
       return NOTIFY_OK;
}

void __init radix_tree_init(void)
{
	radix_tree_node_cachep = kmem_cache_create("radix_tree_node",
			sizeof(struct radix_tree_node), 0,
			SLAB_PANIC | SLAB_RECLAIM_ACCOUNT,
			radix_tree_node_ctor);
	radix_tree_init_maxindex();
	hotcpu_notifier(radix_tree_callback, 0);
}

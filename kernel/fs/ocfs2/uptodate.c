

#include <linux/fs.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/buffer_head.h>
#include <linux/rbtree.h>

#define MLOG_MASK_PREFIX ML_UPTODATE

#include <cluster/masklog.h>

#include "ocfs2.h"

#include "inode.h"
#include "uptodate.h"

struct ocfs2_meta_cache_item {
	struct rb_node	c_node;
	sector_t	c_block;
};

static struct kmem_cache *ocfs2_uptodate_cachep = NULL;

u64 ocfs2_metadata_cache_owner(struct ocfs2_caching_info *ci)
{
	BUG_ON(!ci || !ci->ci_ops);

	return ci->ci_ops->co_owner(ci);
}

struct super_block *ocfs2_metadata_cache_get_super(struct ocfs2_caching_info *ci)
{
	BUG_ON(!ci || !ci->ci_ops);

	return ci->ci_ops->co_get_super(ci);
}

static void ocfs2_metadata_cache_lock(struct ocfs2_caching_info *ci)
{
	BUG_ON(!ci || !ci->ci_ops);

	ci->ci_ops->co_cache_lock(ci);
}

static void ocfs2_metadata_cache_unlock(struct ocfs2_caching_info *ci)
{
	BUG_ON(!ci || !ci->ci_ops);

	ci->ci_ops->co_cache_unlock(ci);
}

void ocfs2_metadata_cache_io_lock(struct ocfs2_caching_info *ci)
{
	BUG_ON(!ci || !ci->ci_ops);

	ci->ci_ops->co_io_lock(ci);
}

void ocfs2_metadata_cache_io_unlock(struct ocfs2_caching_info *ci)
{
	BUG_ON(!ci || !ci->ci_ops);

	ci->ci_ops->co_io_unlock(ci);
}


static void ocfs2_metadata_cache_reset(struct ocfs2_caching_info *ci,
				       int clear)
{
	ci->ci_flags |= OCFS2_CACHE_FL_INLINE;
	ci->ci_num_cached = 0;

	if (clear) {
		ci->ci_created_trans = 0;
		ci->ci_last_trans = 0;
	}
}

void ocfs2_metadata_cache_init(struct ocfs2_caching_info *ci,
			       const struct ocfs2_caching_operations *ops)
{
	BUG_ON(!ops);

	ci->ci_ops = ops;
	ocfs2_metadata_cache_reset(ci, 1);
}

void ocfs2_metadata_cache_exit(struct ocfs2_caching_info *ci)
{
	ocfs2_metadata_cache_purge(ci);
	ocfs2_metadata_cache_reset(ci, 1);
}


static unsigned int ocfs2_purge_copied_metadata_tree(struct rb_root *root)
{
	unsigned int purged = 0;
	struct rb_node *node;
	struct ocfs2_meta_cache_item *item;

	while ((node = rb_last(root)) != NULL) {
		item = rb_entry(node, struct ocfs2_meta_cache_item, c_node);

		mlog(0, "Purge item %llu\n",
		     (unsigned long long) item->c_block);

		rb_erase(&item->c_node, root);
		kmem_cache_free(ocfs2_uptodate_cachep, item);

		purged++;
	}
	return purged;
}

void ocfs2_metadata_cache_purge(struct ocfs2_caching_info *ci)
{
	unsigned int tree, to_purge, purged;
	struct rb_root root = RB_ROOT;

	BUG_ON(!ci || !ci->ci_ops);

	ocfs2_metadata_cache_lock(ci);
	tree = !(ci->ci_flags & OCFS2_CACHE_FL_INLINE);
	to_purge = ci->ci_num_cached;

	mlog(0, "Purge %u %s items from Owner %llu\n", to_purge,
	     tree ? "array" : "tree",
	     (unsigned long long)ocfs2_metadata_cache_owner(ci));

	/* If we're a tree, save off the root so that we can safely
	 * initialize the cache. We do the work to free tree members
	 * without the spinlock. */
	if (tree)
		root = ci->ci_cache.ci_tree;

	ocfs2_metadata_cache_reset(ci, 0);
	ocfs2_metadata_cache_unlock(ci);

	purged = ocfs2_purge_copied_metadata_tree(&root);
	/* If possible, track the number wiped so that we can more
	 * easily detect counting errors. Unfortunately, this is only
	 * meaningful for trees. */
	if (tree && purged != to_purge)
		mlog(ML_ERROR, "Owner %llu, count = %u, purged = %u\n",
		     (unsigned long long)ocfs2_metadata_cache_owner(ci),
		     to_purge, purged);
}

static int ocfs2_search_cache_array(struct ocfs2_caching_info *ci,
				    sector_t item)
{
	int i;

	for (i = 0; i < ci->ci_num_cached; i++) {
		if (item == ci->ci_cache.ci_array[i])
			return i;
	}

	return -1;
}

static struct ocfs2_meta_cache_item *
ocfs2_search_cache_tree(struct ocfs2_caching_info *ci,
			sector_t block)
{
	struct rb_node * n = ci->ci_cache.ci_tree.rb_node;
	struct ocfs2_meta_cache_item *item = NULL;

	while (n) {
		item = rb_entry(n, struct ocfs2_meta_cache_item, c_node);

		if (block < item->c_block)
			n = n->rb_left;
		else if (block > item->c_block)
			n = n->rb_right;
		else
			return item;
	}

	return NULL;
}

static int ocfs2_buffer_cached(struct ocfs2_caching_info *ci,
			       struct buffer_head *bh)
{
	int index = -1;
	struct ocfs2_meta_cache_item *item = NULL;

	ocfs2_metadata_cache_lock(ci);

	mlog(0, "Owner %llu, query block %llu (inline = %u)\n",
	     (unsigned long long)ocfs2_metadata_cache_owner(ci),
	     (unsigned long long) bh->b_blocknr,
	     !!(ci->ci_flags & OCFS2_CACHE_FL_INLINE));

	if (ci->ci_flags & OCFS2_CACHE_FL_INLINE)
		index = ocfs2_search_cache_array(ci, bh->b_blocknr);
	else
		item = ocfs2_search_cache_tree(ci, bh->b_blocknr);

	ocfs2_metadata_cache_unlock(ci);

	mlog(0, "index = %d, item = %p\n", index, item);

	return (index != -1) || (item != NULL);
}

int ocfs2_buffer_uptodate(struct ocfs2_caching_info *ci,
			  struct buffer_head *bh)
{
	/* Doesn't matter if the bh is in our cache or not -- if it's
	 * not marked uptodate then we know it can't have correct
	 * data. */
	if (!buffer_uptodate(bh))
		return 0;

	/* OCFS2 does not allow multiple nodes to be changing the same
	 * block at the same time. */
	if (buffer_jbd(bh))
		return 1;

	/* Ok, locally the buffer is marked as up to date, now search
	 * our cache to see if we can trust that. */
	return ocfs2_buffer_cached(ci, bh);
}

int ocfs2_buffer_read_ahead(struct ocfs2_caching_info *ci,
			    struct buffer_head *bh)
{
	return buffer_locked(bh) && ocfs2_buffer_cached(ci, bh);
}

/* Requires ip_lock */
static void ocfs2_append_cache_array(struct ocfs2_caching_info *ci,
				     sector_t block)
{
	BUG_ON(ci->ci_num_cached >= OCFS2_CACHE_INFO_MAX_ARRAY);

	mlog(0, "block %llu takes position %u\n", (unsigned long long) block,
	     ci->ci_num_cached);

	ci->ci_cache.ci_array[ci->ci_num_cached] = block;
	ci->ci_num_cached++;
}

static void __ocfs2_insert_cache_tree(struct ocfs2_caching_info *ci,
				      struct ocfs2_meta_cache_item *new)
{
	sector_t block = new->c_block;
	struct rb_node *parent = NULL;
	struct rb_node **p = &ci->ci_cache.ci_tree.rb_node;
	struct ocfs2_meta_cache_item *tmp;

	mlog(0, "Insert block %llu num = %u\n", (unsigned long long) block,
	     ci->ci_num_cached);

	while(*p) {
		parent = *p;

		tmp = rb_entry(parent, struct ocfs2_meta_cache_item, c_node);

		if (block < tmp->c_block)
			p = &(*p)->rb_left;
		else if (block > tmp->c_block)
			p = &(*p)->rb_right;
		else {
			/* This should never happen! */
			mlog(ML_ERROR, "Duplicate block %llu cached!\n",
			     (unsigned long long) block);
			BUG();
		}
	}

	rb_link_node(&new->c_node, parent, p);
	rb_insert_color(&new->c_node, &ci->ci_cache.ci_tree);
	ci->ci_num_cached++;
}

/* co_cache_lock() must be held */
static inline int ocfs2_insert_can_use_array(struct ocfs2_caching_info *ci)
{
	return (ci->ci_flags & OCFS2_CACHE_FL_INLINE) &&
		(ci->ci_num_cached < OCFS2_CACHE_INFO_MAX_ARRAY);
}

static void ocfs2_expand_cache(struct ocfs2_caching_info *ci,
			       struct ocfs2_meta_cache_item **tree)
{
	int i;

	mlog_bug_on_msg(ci->ci_num_cached != OCFS2_CACHE_INFO_MAX_ARRAY,
			"Owner %llu, num cached = %u, should be %u\n",
			(unsigned long long)ocfs2_metadata_cache_owner(ci),
			ci->ci_num_cached, OCFS2_CACHE_INFO_MAX_ARRAY);
	mlog_bug_on_msg(!(ci->ci_flags & OCFS2_CACHE_FL_INLINE),
			"Owner %llu not marked as inline anymore!\n",
			(unsigned long long)ocfs2_metadata_cache_owner(ci));

	/* Be careful to initialize the tree members *first* because
	 * once the ci_tree is used, the array is junk... */
	for (i = 0; i < OCFS2_CACHE_INFO_MAX_ARRAY; i++)
		tree[i]->c_block = ci->ci_cache.ci_array[i];

	ci->ci_flags &= ~OCFS2_CACHE_FL_INLINE;
	ci->ci_cache.ci_tree = RB_ROOT;
	/* this will be set again by __ocfs2_insert_cache_tree */
	ci->ci_num_cached = 0;

	for (i = 0; i < OCFS2_CACHE_INFO_MAX_ARRAY; i++) {
		__ocfs2_insert_cache_tree(ci, tree[i]);
		tree[i] = NULL;
	}

	mlog(0, "Expanded %llu to a tree cache: flags 0x%x, num = %u\n",
	     (unsigned long long)ocfs2_metadata_cache_owner(ci),
	     ci->ci_flags, ci->ci_num_cached);
}

static void __ocfs2_set_buffer_uptodate(struct ocfs2_caching_info *ci,
					sector_t block,
					int expand_tree)
{
	int i;
	struct ocfs2_meta_cache_item *new = NULL;
	struct ocfs2_meta_cache_item *tree[OCFS2_CACHE_INFO_MAX_ARRAY] =
		{ NULL, };

	mlog(0, "Owner %llu, block %llu, expand = %d\n",
	     (unsigned long long)ocfs2_metadata_cache_owner(ci),
	     (unsigned long long)block, expand_tree);

	new = kmem_cache_alloc(ocfs2_uptodate_cachep, GFP_NOFS);
	if (!new) {
		mlog_errno(-ENOMEM);
		return;
	}
	new->c_block = block;

	if (expand_tree) {
		/* Do *not* allocate an array here - the removal code
		 * has no way of tracking that. */
		for (i = 0; i < OCFS2_CACHE_INFO_MAX_ARRAY; i++) {
			tree[i] = kmem_cache_alloc(ocfs2_uptodate_cachep,
						   GFP_NOFS);
			if (!tree[i]) {
				mlog_errno(-ENOMEM);
				goto out_free;
			}

			/* These are initialized in ocfs2_expand_cache! */
		}
	}

	ocfs2_metadata_cache_lock(ci);
	if (ocfs2_insert_can_use_array(ci)) {
		mlog(0, "Someone cleared the tree underneath us\n");
		/* Ok, items were removed from the cache in between
		 * locks. Detect this and revert back to the fast path */
		ocfs2_append_cache_array(ci, block);
		ocfs2_metadata_cache_unlock(ci);
		goto out_free;
	}

	if (expand_tree)
		ocfs2_expand_cache(ci, tree);

	__ocfs2_insert_cache_tree(ci, new);
	ocfs2_metadata_cache_unlock(ci);

	new = NULL;
out_free:
	if (new)
		kmem_cache_free(ocfs2_uptodate_cachep, new);

	/* If these were used, then ocfs2_expand_cache re-set them to
	 * NULL for us. */
	if (tree[0]) {
		for (i = 0; i < OCFS2_CACHE_INFO_MAX_ARRAY; i++)
			if (tree[i])
				kmem_cache_free(ocfs2_uptodate_cachep,
						tree[i]);
	}
}

void ocfs2_set_buffer_uptodate(struct ocfs2_caching_info *ci,
			       struct buffer_head *bh)
{
	int expand;

	/* The block may very well exist in our cache already, so avoid
	 * doing any more work in that case. */
	if (ocfs2_buffer_cached(ci, bh))
		return;

	mlog(0, "Owner %llu, inserting block %llu\n",
	     (unsigned long long)ocfs2_metadata_cache_owner(ci),
	     (unsigned long long)bh->b_blocknr);

	/* No need to recheck under spinlock - insertion is guarded by
	 * co_io_lock() */
	ocfs2_metadata_cache_lock(ci);
	if (ocfs2_insert_can_use_array(ci)) {
		/* Fast case - it's an array and there's a free
		 * spot. */
		ocfs2_append_cache_array(ci, bh->b_blocknr);
		ocfs2_metadata_cache_unlock(ci);
		return;
	}

	expand = 0;
	if (ci->ci_flags & OCFS2_CACHE_FL_INLINE) {
		/* We need to bump things up to a tree. */
		expand = 1;
	}
	ocfs2_metadata_cache_unlock(ci);

	__ocfs2_set_buffer_uptodate(ci, bh->b_blocknr, expand);
}

void ocfs2_set_new_buffer_uptodate(struct ocfs2_caching_info *ci,
				   struct buffer_head *bh)
{
	/* This should definitely *not* exist in our cache */
	BUG_ON(ocfs2_buffer_cached(ci, bh));

	set_buffer_uptodate(bh);

	ocfs2_metadata_cache_io_lock(ci);
	ocfs2_set_buffer_uptodate(ci, bh);
	ocfs2_metadata_cache_io_unlock(ci);
}

/* Requires ip_lock. */
static void ocfs2_remove_metadata_array(struct ocfs2_caching_info *ci,
					int index)
{
	sector_t *array = ci->ci_cache.ci_array;
	int bytes;

	BUG_ON(index < 0 || index >= OCFS2_CACHE_INFO_MAX_ARRAY);
	BUG_ON(index >= ci->ci_num_cached);
	BUG_ON(!ci->ci_num_cached);

	mlog(0, "remove index %d (num_cached = %u\n", index,
	     ci->ci_num_cached);

	ci->ci_num_cached--;

	/* don't need to copy if the array is now empty, or if we
	 * removed at the tail */
	if (ci->ci_num_cached && index < ci->ci_num_cached) {
		bytes = sizeof(sector_t) * (ci->ci_num_cached - index);
		memmove(&array[index], &array[index + 1], bytes);
	}
}

/* Requires ip_lock. */
static void ocfs2_remove_metadata_tree(struct ocfs2_caching_info *ci,
				       struct ocfs2_meta_cache_item *item)
{
	mlog(0, "remove block %llu from tree\n",
	     (unsigned long long) item->c_block);

	rb_erase(&item->c_node, &ci->ci_cache.ci_tree);
	ci->ci_num_cached--;
}

static void ocfs2_remove_block_from_cache(struct ocfs2_caching_info *ci,
					  sector_t block)
{
	int index;
	struct ocfs2_meta_cache_item *item = NULL;

	ocfs2_metadata_cache_lock(ci);
	mlog(0, "Owner %llu, remove %llu, items = %u, array = %u\n",
	     (unsigned long long)ocfs2_metadata_cache_owner(ci),
	     (unsigned long long) block, ci->ci_num_cached,
	     ci->ci_flags & OCFS2_CACHE_FL_INLINE);

	if (ci->ci_flags & OCFS2_CACHE_FL_INLINE) {
		index = ocfs2_search_cache_array(ci, block);
		if (index != -1)
			ocfs2_remove_metadata_array(ci, index);
	} else {
		item = ocfs2_search_cache_tree(ci, block);
		if (item)
			ocfs2_remove_metadata_tree(ci, item);
	}
	ocfs2_metadata_cache_unlock(ci);

	if (item)
		kmem_cache_free(ocfs2_uptodate_cachep, item);
}

void ocfs2_remove_from_cache(struct ocfs2_caching_info *ci,
			     struct buffer_head *bh)
{
	sector_t block = bh->b_blocknr;

	ocfs2_remove_block_from_cache(ci, block);
}

/* Called when we remove xattr clusters from an inode. */
void ocfs2_remove_xattr_clusters_from_cache(struct ocfs2_caching_info *ci,
					    sector_t block,
					    u32 c_len)
{
	struct super_block *sb = ocfs2_metadata_cache_get_super(ci);
	unsigned int i, b_len = ocfs2_clusters_to_blocks(sb, 1) * c_len;

	for (i = 0; i < b_len; i++, block++)
		ocfs2_remove_block_from_cache(ci, block);
}

int __init init_ocfs2_uptodate_cache(void)
{
	ocfs2_uptodate_cachep = kmem_cache_create("ocfs2_uptodate",
				  sizeof(struct ocfs2_meta_cache_item),
				  0, SLAB_HWCACHE_ALIGN, NULL);
	if (!ocfs2_uptodate_cachep)
		return -ENOMEM;

	mlog(0, "%u inlined cache items per inode.\n",
	     OCFS2_CACHE_INFO_MAX_ARRAY);

	return 0;
}

void exit_ocfs2_uptodate_cache(void)
{
	if (ocfs2_uptodate_cachep)
		kmem_cache_destroy(ocfs2_uptodate_cachep);
}


#ifndef __REFCACHE__
#define __REFCACHE__

struct btrfs_extent_info {
	/* bytenr and num_bytes find the extent in the extent allocation tree */
	u64 bytenr;
	u64 num_bytes;

	/* objectid and offset find the back reference for the file */
	u64 objectid;
	u64 offset;
};

struct btrfs_leaf_ref {
	struct rb_node rb_node;
	struct btrfs_leaf_ref_tree *tree;
	int in_tree;
	atomic_t usage;

	u64 root_gen;
	u64 bytenr;
	u64 owner;
	u64 generation;
	int nritems;

	struct list_head list;
	struct btrfs_extent_info extents[];
};

static inline size_t btrfs_leaf_ref_size(int nr_extents)
{
	return sizeof(struct btrfs_leaf_ref) +
	       sizeof(struct btrfs_extent_info) * nr_extents;
}

static inline void btrfs_leaf_ref_tree_init(struct btrfs_leaf_ref_tree *tree)
{
	tree->root = RB_ROOT;
	INIT_LIST_HEAD(&tree->list);
	spin_lock_init(&tree->lock);
}

static inline int btrfs_leaf_ref_tree_empty(struct btrfs_leaf_ref_tree *tree)
{
	return RB_EMPTY_ROOT(&tree->root);
}

void btrfs_leaf_ref_tree_init(struct btrfs_leaf_ref_tree *tree);
struct btrfs_leaf_ref *btrfs_alloc_leaf_ref(struct btrfs_root *root,
					    int nr_extents);
void btrfs_free_leaf_ref(struct btrfs_root *root, struct btrfs_leaf_ref *ref);
struct btrfs_leaf_ref *btrfs_lookup_leaf_ref(struct btrfs_root *root,
					     u64 bytenr);
int btrfs_add_leaf_ref(struct btrfs_root *root, struct btrfs_leaf_ref *ref,
		       int shared);
int btrfs_remove_leaf_refs(struct btrfs_root *root, u64 max_root_gen,
			   int shared);
int btrfs_remove_leaf_ref(struct btrfs_root *root, struct btrfs_leaf_ref *ref);
#endif

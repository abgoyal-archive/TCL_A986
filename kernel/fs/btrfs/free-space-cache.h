

#ifndef __BTRFS_FREE_SPACE_CACHE
#define __BTRFS_FREE_SPACE_CACHE

struct btrfs_free_space {
	struct rb_node offset_index;
	u64 offset;
	u64 bytes;
	unsigned long *bitmap;
	struct list_head list;
};

int btrfs_add_free_space(struct btrfs_block_group_cache *block_group,
			 u64 bytenr, u64 size);
int btrfs_remove_free_space(struct btrfs_block_group_cache *block_group,
			    u64 bytenr, u64 size);
void btrfs_remove_free_space_cache(struct btrfs_block_group_cache
				   *block_group);
u64 btrfs_find_space_for_alloc(struct btrfs_block_group_cache *block_group,
			       u64 offset, u64 bytes, u64 empty_size);
void btrfs_dump_free_space(struct btrfs_block_group_cache *block_group,
			   u64 bytes);
u64 btrfs_block_group_free_space(struct btrfs_block_group_cache *block_group);
int btrfs_find_space_cluster(struct btrfs_trans_handle *trans,
			     struct btrfs_root *root,
			     struct btrfs_block_group_cache *block_group,
			     struct btrfs_free_cluster *cluster,
			     u64 offset, u64 bytes, u64 empty_size);
void btrfs_init_free_cluster(struct btrfs_free_cluster *cluster);
u64 btrfs_alloc_from_cluster(struct btrfs_block_group_cache *block_group,
			     struct btrfs_free_cluster *cluster, u64 bytes,
			     u64 min_start);
int btrfs_return_cluster_to_free_space(
			       struct btrfs_block_group_cache *block_group,
			       struct btrfs_free_cluster *cluster);
#endif

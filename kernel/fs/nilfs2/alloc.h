

#ifndef _NILFS_ALLOC_H
#define _NILFS_ALLOC_H

#include <linux/types.h>
#include <linux/buffer_head.h>
#include <linux/fs.h>

static inline unsigned long
nilfs_palloc_entries_per_group(const struct inode *inode)
{
	return 1UL << (inode->i_blkbits + 3 /* log2(8 = CHAR_BITS) */);
}

int nilfs_palloc_init_blockgroup(struct inode *, unsigned);
int nilfs_palloc_get_entry_block(struct inode *, __u64, int,
				 struct buffer_head **);
void *nilfs_palloc_block_get_entry(const struct inode *, __u64,
				   const struct buffer_head *, void *);

struct nilfs_palloc_req {
	__u64 pr_entry_nr;
	struct buffer_head *pr_desc_bh;
	struct buffer_head *pr_bitmap_bh;
	struct buffer_head *pr_entry_bh;
};

int nilfs_palloc_prepare_alloc_entry(struct inode *,
				     struct nilfs_palloc_req *);
void nilfs_palloc_commit_alloc_entry(struct inode *,
				     struct nilfs_palloc_req *);
void nilfs_palloc_abort_alloc_entry(struct inode *, struct nilfs_palloc_req *);
void nilfs_palloc_commit_free_entry(struct inode *, struct nilfs_palloc_req *);
int nilfs_palloc_prepare_free_entry(struct inode *, struct nilfs_palloc_req *);
void nilfs_palloc_abort_free_entry(struct inode *, struct nilfs_palloc_req *);
int nilfs_palloc_freev(struct inode *, __u64 *, size_t);

#define nilfs_set_bit_atomic		ext2_set_bit_atomic
#define nilfs_clear_bit_atomic		ext2_clear_bit_atomic
#define nilfs_find_next_zero_bit	ext2_find_next_zero_bit


struct nilfs_bh_assoc {
	unsigned long blkoff;
	struct buffer_head *bh;
};

struct nilfs_palloc_cache {
	spinlock_t lock;
	struct nilfs_bh_assoc prev_desc;
	struct nilfs_bh_assoc prev_bitmap;
	struct nilfs_bh_assoc prev_entry;
};

void nilfs_palloc_setup_cache(struct inode *inode,
			      struct nilfs_palloc_cache *cache);
void nilfs_palloc_clear_cache(struct inode *inode);
void nilfs_palloc_destroy_cache(struct inode *inode);

#endif	/* _NILFS_ALLOC_H */

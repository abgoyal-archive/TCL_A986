

#ifndef _NILFS_BMAP_H
#define _NILFS_BMAP_H

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/nilfs2_fs.h>
#include "alloc.h"
#include "dat.h"

#define NILFS_BMAP_INVALID_PTR	0

#define nilfs_bmap_dkey_to_key(dkey)	le64_to_cpu(dkey)
#define nilfs_bmap_key_to_dkey(key)	cpu_to_le64(key)
#define nilfs_bmap_dptr_to_ptr(dptr)	le64_to_cpu(dptr)
#define nilfs_bmap_ptr_to_dptr(ptr)	cpu_to_le64(ptr)

#define nilfs_bmap_keydiff_abs(diff)	((diff) < 0 ? -(diff) : (diff))


struct nilfs_bmap;

union nilfs_bmap_ptr_req {
	__u64 bpr_ptr;
	struct nilfs_palloc_req bpr_req;
};

struct nilfs_bmap_stats {
	unsigned int bs_nblocks;
};

struct nilfs_bmap_operations {
	int (*bop_lookup)(const struct nilfs_bmap *, __u64, int, __u64 *);
	int (*bop_lookup_contig)(const struct nilfs_bmap *, __u64, __u64 *,
				 unsigned);
	int (*bop_insert)(struct nilfs_bmap *, __u64, __u64);
	int (*bop_delete)(struct nilfs_bmap *, __u64);
	void (*bop_clear)(struct nilfs_bmap *);

	int (*bop_propagate)(const struct nilfs_bmap *, struct buffer_head *);
	void (*bop_lookup_dirty_buffers)(struct nilfs_bmap *,
					 struct list_head *);

	int (*bop_assign)(struct nilfs_bmap *,
			  struct buffer_head **,
			  sector_t,
			  union nilfs_binfo *);
	int (*bop_mark)(struct nilfs_bmap *, __u64, int);

	/* The following functions are internal use only. */
	int (*bop_last_key)(const struct nilfs_bmap *, __u64 *);
	int (*bop_check_insert)(const struct nilfs_bmap *, __u64);
	int (*bop_check_delete)(struct nilfs_bmap *, __u64);
	int (*bop_gather_data)(struct nilfs_bmap *, __u64 *, __u64 *, int);
};


#define NILFS_BMAP_SIZE		(NILFS_INODE_BMAP_SIZE * sizeof(__le64))
#define NILFS_BMAP_KEY_BIT	(sizeof(unsigned long) * 8 /* CHAR_BIT */)
#define NILFS_BMAP_NEW_PTR_INIT	\
	(1UL << (sizeof(unsigned long) * 8 /* CHAR_BIT */ - 1))

static inline int nilfs_bmap_is_new_ptr(unsigned long ptr)
{
	return !!(ptr & NILFS_BMAP_NEW_PTR_INIT);
}


struct nilfs_bmap {
	union {
		__u8 u_flags;
		__le64 u_data[NILFS_BMAP_SIZE / sizeof(__le64)];
	} b_u;
	struct rw_semaphore b_sem;
	struct inode *b_inode;
	const struct nilfs_bmap_operations *b_ops;
	__u64 b_last_allocated_key;
	__u64 b_last_allocated_ptr;
	int b_ptr_type;
	int b_state;
};

/* pointer type */
#define NILFS_BMAP_PTR_P	0	/* physical block number (i.e. LBN) */
#define NILFS_BMAP_PTR_VS	1	/* virtual block number (single
					   version) */
#define NILFS_BMAP_PTR_VM	2	/* virtual block number (has multiple
					   versions) */
#define NILFS_BMAP_PTR_U	(-1)	/* never perform pointer operations */

#define NILFS_BMAP_USE_VBN(bmap)	((bmap)->b_ptr_type > 0)

/* state */
#define NILFS_BMAP_DIRTY	0x00000001


int nilfs_bmap_test_and_clear_dirty(struct nilfs_bmap *);
int nilfs_bmap_read(struct nilfs_bmap *, struct nilfs_inode *);
void nilfs_bmap_write(struct nilfs_bmap *, struct nilfs_inode *);
int nilfs_bmap_lookup_contig(struct nilfs_bmap *, __u64, __u64 *, unsigned);
int nilfs_bmap_insert(struct nilfs_bmap *, unsigned long, unsigned long);
int nilfs_bmap_delete(struct nilfs_bmap *, unsigned long);
int nilfs_bmap_last_key(struct nilfs_bmap *, unsigned long *);
int nilfs_bmap_truncate(struct nilfs_bmap *, unsigned long);
void nilfs_bmap_clear(struct nilfs_bmap *);
int nilfs_bmap_propagate(struct nilfs_bmap *, struct buffer_head *);
void nilfs_bmap_lookup_dirty_buffers(struct nilfs_bmap *, struct list_head *);
int nilfs_bmap_assign(struct nilfs_bmap *, struct buffer_head **,
		      unsigned long, union nilfs_binfo *);
int nilfs_bmap_lookup_at_level(struct nilfs_bmap *, __u64, int, __u64 *);
int nilfs_bmap_mark(struct nilfs_bmap *, __u64, int);

void nilfs_bmap_init_gc(struct nilfs_bmap *);
void nilfs_bmap_init_gcdat(struct nilfs_bmap *, struct nilfs_bmap *);
void nilfs_bmap_commit_gcdat(struct nilfs_bmap *, struct nilfs_bmap *);


static inline int nilfs_bmap_lookup(struct nilfs_bmap *bmap, __u64 key,
				    __u64 *ptr)
{
	return nilfs_bmap_lookup_at_level(bmap, key, 1, ptr);
}

struct inode *nilfs_bmap_get_dat(const struct nilfs_bmap *);

static inline int nilfs_bmap_prepare_alloc_ptr(struct nilfs_bmap *bmap,
					       union nilfs_bmap_ptr_req *req,
					       struct inode *dat)
{
	if (dat)
		return nilfs_dat_prepare_alloc(dat, &req->bpr_req);
	/* ignore target ptr */
	req->bpr_ptr = bmap->b_last_allocated_ptr++;
	return 0;
}

static inline void nilfs_bmap_commit_alloc_ptr(struct nilfs_bmap *bmap,
					       union nilfs_bmap_ptr_req *req,
					       struct inode *dat)
{
	if (dat)
		nilfs_dat_commit_alloc(dat, &req->bpr_req);
}

static inline void nilfs_bmap_abort_alloc_ptr(struct nilfs_bmap *bmap,
					      union nilfs_bmap_ptr_req *req,
					      struct inode *dat)
{
	if (dat)
		nilfs_dat_abort_alloc(dat, &req->bpr_req);
	else
		bmap->b_last_allocated_ptr--;
}

static inline int nilfs_bmap_prepare_end_ptr(struct nilfs_bmap *bmap,
					     union nilfs_bmap_ptr_req *req,
					     struct inode *dat)
{
	return dat ? nilfs_dat_prepare_end(dat, &req->bpr_req) : 0;
}

static inline void nilfs_bmap_commit_end_ptr(struct nilfs_bmap *bmap,
					     union nilfs_bmap_ptr_req *req,
					     struct inode *dat)
{
	if (dat)
		nilfs_dat_commit_end(dat, &req->bpr_req,
				     bmap->b_ptr_type == NILFS_BMAP_PTR_VS);
}

static inline void nilfs_bmap_abort_end_ptr(struct nilfs_bmap *bmap,
					    union nilfs_bmap_ptr_req *req,
					    struct inode *dat)
{
	if (dat)
		nilfs_dat_abort_end(dat, &req->bpr_req);
}

__u64 nilfs_bmap_data_get_key(const struct nilfs_bmap *,
			      const struct buffer_head *);

__u64 nilfs_bmap_find_target_seq(const struct nilfs_bmap *, __u64);
__u64 nilfs_bmap_find_target_in_group(const struct nilfs_bmap *);

void nilfs_bmap_add_blocks(const struct nilfs_bmap *, int);
void nilfs_bmap_sub_blocks(const struct nilfs_bmap *, int);


/* Assume that bmap semaphore is locked. */
static inline int nilfs_bmap_dirty(const struct nilfs_bmap *bmap)
{
	return !!(bmap->b_state & NILFS_BMAP_DIRTY);
}

/* Assume that bmap semaphore is locked. */
static inline void nilfs_bmap_set_dirty(struct nilfs_bmap *bmap)
{
	bmap->b_state |= NILFS_BMAP_DIRTY;
}

/* Assume that bmap semaphore is locked. */
static inline void nilfs_bmap_clear_dirty(struct nilfs_bmap *bmap)
{
	bmap->b_state &= ~NILFS_BMAP_DIRTY;
}


#define NILFS_BMAP_LARGE	0x1

#define NILFS_BMAP_SMALL_LOW	NILFS_DIRECT_KEY_MIN
#define NILFS_BMAP_SMALL_HIGH	NILFS_DIRECT_KEY_MAX
#define NILFS_BMAP_LARGE_LOW	NILFS_BTREE_ROOT_NCHILDREN_MAX
#define NILFS_BMAP_LARGE_HIGH	NILFS_BTREE_KEY_MAX

#endif	/* _NILFS_BMAP_H */

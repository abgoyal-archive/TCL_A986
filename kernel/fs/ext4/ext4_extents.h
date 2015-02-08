

#ifndef _EXT4_EXTENTS
#define _EXT4_EXTENTS

#include "ext4.h"

#define AGGRESSIVE_TEST_

#define EXTENTS_STATS__

#define CHECK_BINSEARCH__

#define EXT_DEBUG__
#ifdef EXT_DEBUG
#define ext_debug(a...)		printk(a)
#else
#define ext_debug(a...)
#endif

#define EXT_STATS_



struct ext4_extent {
	__le32	ee_block;	/* first logical block extent covers */
	__le16	ee_len;		/* number of blocks covered by extent */
	__le16	ee_start_hi;	/* high 16 bits of physical block */
	__le32	ee_start_lo;	/* low 32 bits of physical block */
};

struct ext4_extent_idx {
	__le32	ei_block;	/* index covers logical blocks from 'block' */
	__le32	ei_leaf_lo;	/* pointer to the physical block of the next *
				 * level. leaf or next index could be there */
	__le16	ei_leaf_hi;	/* high 16 bits of physical block */
	__u16	ei_unused;
};

struct ext4_extent_header {
	__le16	eh_magic;	/* probably will support different formats */
	__le16	eh_entries;	/* number of valid entries */
	__le16	eh_max;		/* capacity of store in entries */
	__le16	eh_depth;	/* has tree real underlying blocks? */
	__le32	eh_generation;	/* generation of the tree */
};

#define EXT4_EXT_MAGIC		cpu_to_le16(0xf30a)

struct ext4_ext_path {
	ext4_fsblk_t			p_block;
	__u16				p_depth;
	struct ext4_extent		*p_ext;
	struct ext4_extent_idx		*p_idx;
	struct ext4_extent_header	*p_hdr;
	struct buffer_head		*p_bh;
};


#define EXT4_EXT_CACHE_NO	0
#define EXT4_EXT_CACHE_GAP	1
#define EXT4_EXT_CACHE_EXTENT	2

typedef int (*ext_prepare_callback)(struct inode *, struct ext4_ext_path *,
					struct ext4_ext_cache *,
					struct ext4_extent *, void *);

#define EXT_CONTINUE   0
#define EXT_BREAK      1
#define EXT_REPEAT     2

/* Maximum logical block in a file; ext4_extent's ee_block is __le32 */
#define EXT_MAX_BLOCK	0xffffffff

#define EXT_INIT_MAX_LEN	(1UL << 15)
#define EXT_UNINIT_MAX_LEN	(EXT_INIT_MAX_LEN - 1)


#define EXT_FIRST_EXTENT(__hdr__) \
	((struct ext4_extent *) (((char *) (__hdr__)) +		\
				 sizeof(struct ext4_extent_header)))
#define EXT_FIRST_INDEX(__hdr__) \
	((struct ext4_extent_idx *) (((char *) (__hdr__)) +	\
				     sizeof(struct ext4_extent_header)))
#define EXT_HAS_FREE_INDEX(__path__) \
	(le16_to_cpu((__path__)->p_hdr->eh_entries) \
				     < le16_to_cpu((__path__)->p_hdr->eh_max))
#define EXT_LAST_EXTENT(__hdr__) \
	(EXT_FIRST_EXTENT((__hdr__)) + le16_to_cpu((__hdr__)->eh_entries) - 1)
#define EXT_LAST_INDEX(__hdr__) \
	(EXT_FIRST_INDEX((__hdr__)) + le16_to_cpu((__hdr__)->eh_entries) - 1)
#define EXT_MAX_EXTENT(__hdr__) \
	(EXT_FIRST_EXTENT((__hdr__)) + le16_to_cpu((__hdr__)->eh_max) - 1)
#define EXT_MAX_INDEX(__hdr__) \
	(EXT_FIRST_INDEX((__hdr__)) + le16_to_cpu((__hdr__)->eh_max) - 1)

static inline struct ext4_extent_header *ext_inode_hdr(struct inode *inode)
{
	return (struct ext4_extent_header *) EXT4_I(inode)->i_data;
}

static inline struct ext4_extent_header *ext_block_hdr(struct buffer_head *bh)
{
	return (struct ext4_extent_header *) bh->b_data;
}

static inline unsigned short ext_depth(struct inode *inode)
{
	return le16_to_cpu(ext_inode_hdr(inode)->eh_depth);
}

static inline void
ext4_ext_invalidate_cache(struct inode *inode)
{
	EXT4_I(inode)->i_cached_extent.ec_type = EXT4_EXT_CACHE_NO;
}

static inline void ext4_ext_mark_uninitialized(struct ext4_extent *ext)
{
	/* We can not have an uninitialized extent of zero length! */
	BUG_ON((le16_to_cpu(ext->ee_len) & ~EXT_INIT_MAX_LEN) == 0);
	ext->ee_len |= cpu_to_le16(EXT_INIT_MAX_LEN);
}

static inline int ext4_ext_is_uninitialized(struct ext4_extent *ext)
{
	/* Extent with ee_len of 0x8000 is treated as an initialized extent */
	return (le16_to_cpu(ext->ee_len) > EXT_INIT_MAX_LEN);
}

static inline int ext4_ext_get_actual_len(struct ext4_extent *ext)
{
	return (le16_to_cpu(ext->ee_len) <= EXT_INIT_MAX_LEN ?
		le16_to_cpu(ext->ee_len) :
		(le16_to_cpu(ext->ee_len) - EXT_INIT_MAX_LEN));
}

static inline void ext4_ext_mark_initialized(struct ext4_extent *ext)
{
	ext->ee_len = cpu_to_le16(ext4_ext_get_actual_len(ext));
}

extern int ext4_ext_calc_metadata_amount(struct inode *inode,
					 sector_t lblocks);
extern ext4_fsblk_t ext_pblock(struct ext4_extent *ex);
extern ext4_fsblk_t idx_pblock(struct ext4_extent_idx *);
extern void ext4_ext_store_pblock(struct ext4_extent *, ext4_fsblk_t);
extern int ext4_extent_tree_init(handle_t *, struct inode *);
extern int ext4_ext_calc_credits_for_single_extent(struct inode *inode,
						   int num,
						   struct ext4_ext_path *path);
extern int ext4_can_extents_be_merged(struct inode *inode,
				      struct ext4_extent *ex1,
				      struct ext4_extent *ex2);
extern int ext4_ext_try_to_merge(struct inode *inode,
				 struct ext4_ext_path *path,
				 struct ext4_extent *);
extern unsigned int ext4_ext_check_overlap(struct inode *, struct ext4_extent *, struct ext4_ext_path *);
extern int ext4_ext_insert_extent(handle_t *, struct inode *, struct ext4_ext_path *, struct ext4_extent *, int);
extern int ext4_ext_walk_space(struct inode *, ext4_lblk_t, ext4_lblk_t,
							ext_prepare_callback, void *);
extern struct ext4_ext_path *ext4_ext_find_extent(struct inode *, ext4_lblk_t,
							struct ext4_ext_path *);
extern int ext4_ext_search_left(struct inode *, struct ext4_ext_path *,
						ext4_lblk_t *, ext4_fsblk_t *);
extern int ext4_ext_search_right(struct inode *, struct ext4_ext_path *,
						ext4_lblk_t *, ext4_fsblk_t *);
extern void ext4_ext_drop_refs(struct ext4_ext_path *);
extern int ext4_ext_check_inode(struct inode *inode);
#endif /* _EXT4_EXTENTS */


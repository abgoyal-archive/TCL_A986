
#ifndef __XFS_DIR2_BLOCK_H__
#define	__XFS_DIR2_BLOCK_H__


struct uio;
struct xfs_dabuf;
struct xfs_da_args;
struct xfs_dir2_data_hdr;
struct xfs_dir2_leaf_entry;
struct xfs_inode;
struct xfs_mount;
struct xfs_trans;


#define	XFS_DIR2_BLOCK_MAGIC	0x58443242	/* XD2B: for one block dirs */

typedef struct xfs_dir2_block_tail {
	__be32		count;			/* count of leaf entries */
	__be32		stale;			/* count of stale lf entries */
} xfs_dir2_block_tail_t;

typedef struct xfs_dir2_block {
	xfs_dir2_data_hdr_t	hdr;		/* magic XFS_DIR2_BLOCK_MAGIC */
	xfs_dir2_data_union_t	u[1];
	xfs_dir2_leaf_entry_t	leaf[1];
	xfs_dir2_block_tail_t	tail;
} xfs_dir2_block_t;

static inline xfs_dir2_block_tail_t *
xfs_dir2_block_tail_p(struct xfs_mount *mp, xfs_dir2_block_t *block)
{
	return (((xfs_dir2_block_tail_t *)
		((char *)(block) + (mp)->m_dirblksize)) - 1);
}

static inline struct xfs_dir2_leaf_entry *
xfs_dir2_block_leaf_p(xfs_dir2_block_tail_t *btp)
{
	return ((struct xfs_dir2_leaf_entry *)btp) - be32_to_cpu(btp->count);
}

extern int xfs_dir2_block_addname(struct xfs_da_args *args);
extern int xfs_dir2_block_getdents(struct xfs_inode *dp, void *dirent,
				   xfs_off_t *offset, filldir_t filldir);
extern int xfs_dir2_block_lookup(struct xfs_da_args *args);
extern int xfs_dir2_block_removename(struct xfs_da_args *args);
extern int xfs_dir2_block_replace(struct xfs_da_args *args);
extern int xfs_dir2_leaf_to_block(struct xfs_da_args *args,
				  struct xfs_dabuf *lbp, struct xfs_dabuf *dbp);
extern int xfs_dir2_sf_to_block(struct xfs_da_args *args);

#endif	/* __XFS_DIR2_BLOCK_H__ */

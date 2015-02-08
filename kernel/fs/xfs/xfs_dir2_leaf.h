
#ifndef __XFS_DIR2_LEAF_H__
#define	__XFS_DIR2_LEAF_H__

struct uio;
struct xfs_dabuf;
struct xfs_da_args;
struct xfs_inode;
struct xfs_mount;
struct xfs_trans;

#define	XFS_DIR2_LEAF_SPACE	1
#define	XFS_DIR2_LEAF_OFFSET	(XFS_DIR2_LEAF_SPACE * XFS_DIR2_SPACE_SIZE)
#define	XFS_DIR2_LEAF_FIRSTDB(mp)	\
	xfs_dir2_byte_to_db(mp, XFS_DIR2_LEAF_OFFSET)

typedef	__uint32_t	xfs_dir2_dataptr_t;
#define	XFS_DIR2_MAX_DATAPTR	((xfs_dir2_dataptr_t)0xffffffff)
#define	XFS_DIR2_NULL_DATAPTR	((xfs_dir2_dataptr_t)0)

typedef struct xfs_dir2_leaf_hdr {
	xfs_da_blkinfo_t	info;		/* header for da routines */
	__be16			count;		/* count of entries */
	__be16			stale;		/* count of stale entries */
} xfs_dir2_leaf_hdr_t;

typedef struct xfs_dir2_leaf_entry {
	__be32			hashval;	/* hash value of name */
	__be32			address;	/* address of data entry */
} xfs_dir2_leaf_entry_t;

typedef struct xfs_dir2_leaf_tail {
	__be32			bestcount;
} xfs_dir2_leaf_tail_t;

typedef struct xfs_dir2_leaf {
	xfs_dir2_leaf_hdr_t	hdr;		/* leaf header */
	xfs_dir2_leaf_entry_t	ents[1];	/* entries */
						/* ... */
	xfs_dir2_data_off_t	bests[1];	/* best free counts */
	xfs_dir2_leaf_tail_t	tail;		/* leaf tail */
} xfs_dir2_leaf_t;


static inline int xfs_dir2_max_leaf_ents(struct xfs_mount *mp)
{
	return (int)(((mp)->m_dirblksize - (uint)sizeof(xfs_dir2_leaf_hdr_t)) /
	       (uint)sizeof(xfs_dir2_leaf_entry_t));
}

static inline xfs_dir2_leaf_tail_t *
xfs_dir2_leaf_tail_p(struct xfs_mount *mp, xfs_dir2_leaf_t *lp)
{
	return (xfs_dir2_leaf_tail_t *)
		((char *)(lp) + (mp)->m_dirblksize - 
		  (uint)sizeof(xfs_dir2_leaf_tail_t));
}

static inline __be16 *
xfs_dir2_leaf_bests_p(xfs_dir2_leaf_tail_t *ltp)
{
	return (__be16 *)ltp - be32_to_cpu(ltp->bestcount);
}

static inline xfs_dir2_off_t
xfs_dir2_dataptr_to_byte(struct xfs_mount *mp, xfs_dir2_dataptr_t dp)
{
	return (xfs_dir2_off_t)(dp) << XFS_DIR2_DATA_ALIGN_LOG;
}

static inline xfs_dir2_dataptr_t
xfs_dir2_byte_to_dataptr(struct xfs_mount *mp, xfs_dir2_off_t by)
{
	return (xfs_dir2_dataptr_t)((by) >> XFS_DIR2_DATA_ALIGN_LOG);
}

static inline xfs_dir2_db_t
xfs_dir2_byte_to_db(struct xfs_mount *mp, xfs_dir2_off_t by)
{
	return (xfs_dir2_db_t)((by) >> \
		 ((mp)->m_sb.sb_blocklog + (mp)->m_sb.sb_dirblklog));
}

static inline xfs_dir2_db_t
xfs_dir2_dataptr_to_db(struct xfs_mount *mp, xfs_dir2_dataptr_t dp)
{
	return xfs_dir2_byte_to_db(mp, xfs_dir2_dataptr_to_byte(mp, dp));
}

static inline xfs_dir2_data_aoff_t
xfs_dir2_byte_to_off(struct xfs_mount *mp, xfs_dir2_off_t by)
{
	return (xfs_dir2_data_aoff_t)((by) & \
		((1 << ((mp)->m_sb.sb_blocklog + (mp)->m_sb.sb_dirblklog)) - 1));
}

static inline xfs_dir2_data_aoff_t
xfs_dir2_dataptr_to_off(struct xfs_mount *mp, xfs_dir2_dataptr_t dp)
{
	return xfs_dir2_byte_to_off(mp, xfs_dir2_dataptr_to_byte(mp, dp));
}

static inline xfs_dir2_off_t
xfs_dir2_db_off_to_byte(struct xfs_mount *mp, xfs_dir2_db_t db,
			xfs_dir2_data_aoff_t o)
{
	return ((xfs_dir2_off_t)(db) << \
		((mp)->m_sb.sb_blocklog + (mp)->m_sb.sb_dirblklog)) + (o);
}

static inline xfs_dablk_t
xfs_dir2_db_to_da(struct xfs_mount *mp, xfs_dir2_db_t db)
{
	return (xfs_dablk_t)((db) << (mp)->m_sb.sb_dirblklog);
}

static inline xfs_dablk_t
xfs_dir2_byte_to_da(struct xfs_mount *mp, xfs_dir2_off_t by)
{
	return xfs_dir2_db_to_da(mp, xfs_dir2_byte_to_db(mp, by));
}

static inline xfs_dir2_dataptr_t
xfs_dir2_db_off_to_dataptr(struct xfs_mount *mp, xfs_dir2_db_t db,
			   xfs_dir2_data_aoff_t o)
{
	return xfs_dir2_byte_to_dataptr(mp, xfs_dir2_db_off_to_byte(mp, db, o));
}

static inline xfs_dir2_db_t
xfs_dir2_da_to_db(struct xfs_mount *mp, xfs_dablk_t da)
{
	return (xfs_dir2_db_t)((da) >> (mp)->m_sb.sb_dirblklog);
}

static inline xfs_dir2_off_t
xfs_dir2_da_to_byte(struct xfs_mount *mp, xfs_dablk_t da)
{
	return xfs_dir2_db_off_to_byte(mp, xfs_dir2_da_to_db(mp, da), 0);
}

extern int xfs_dir2_block_to_leaf(struct xfs_da_args *args,
				  struct xfs_dabuf *dbp);
extern int xfs_dir2_leaf_addname(struct xfs_da_args *args);
extern void xfs_dir2_leaf_compact(struct xfs_da_args *args,
				  struct xfs_dabuf *bp);
extern void xfs_dir2_leaf_compact_x1(struct xfs_dabuf *bp, int *indexp,
				     int *lowstalep, int *highstalep,
				     int *lowlogp, int *highlogp);
extern int xfs_dir2_leaf_getdents(struct xfs_inode *dp, void *dirent,
				  size_t bufsize, xfs_off_t *offset,
				  filldir_t filldir);
extern int xfs_dir2_leaf_init(struct xfs_da_args *args, xfs_dir2_db_t bno,
			      struct xfs_dabuf **bpp, int magic);
extern void xfs_dir2_leaf_log_ents(struct xfs_trans *tp, struct xfs_dabuf *bp,
				   int first, int last);
extern void xfs_dir2_leaf_log_header(struct xfs_trans *tp,
				     struct xfs_dabuf *bp);
extern int xfs_dir2_leaf_lookup(struct xfs_da_args *args);
extern int xfs_dir2_leaf_removename(struct xfs_da_args *args);
extern int xfs_dir2_leaf_replace(struct xfs_da_args *args);
extern int xfs_dir2_leaf_search_hash(struct xfs_da_args *args,
				     struct xfs_dabuf *lbp);
extern int xfs_dir2_leaf_trim_data(struct xfs_da_args *args,
				   struct xfs_dabuf *lbp, xfs_dir2_db_t db);
extern int xfs_dir2_node_to_leaf(struct xfs_da_state *state);

#endif	/* __XFS_DIR2_LEAF_H__ */

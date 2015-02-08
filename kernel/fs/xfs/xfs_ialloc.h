
#ifndef __XFS_IALLOC_H__
#define	__XFS_IALLOC_H__

struct xfs_buf;
struct xfs_dinode;
struct xfs_imap;
struct xfs_mount;
struct xfs_trans;

#define	XFS_IALLOC_INODES(mp)	(mp)->m_ialloc_inos
#define	XFS_IALLOC_BLOCKS(mp)	(mp)->m_ialloc_blks

#define	XFS_INODE_BIG_CLUSTER_SIZE	8192
#define	XFS_INODE_CLUSTER_SIZE(mp)	(mp)->m_inode_cluster_size

static inline struct xfs_dinode *
xfs_make_iptr(struct xfs_mount *mp, struct xfs_buf *b, int o)
{
	return (xfs_dinode_t *)
		(xfs_buf_offset(b, o << (mp)->m_sb.sb_inodelog));
}

static inline int xfs_ialloc_find_free(xfs_inofree_t *fp)
{
	return xfs_lowbit64(*fp);
}


int					/* error */
xfs_dialloc(
	struct xfs_trans *tp,		/* transaction pointer */
	xfs_ino_t	parent,		/* parent inode (directory) */
	mode_t		mode,		/* mode bits for new inode */
	int		okalloc,	/* ok to allocate more space */
	struct xfs_buf	**agbp,		/* buf for a.g. inode header */
	boolean_t	*alloc_done,	/* an allocation was done to replenish
					   the free inodes */
	xfs_ino_t	*inop);		/* inode number allocated */

int					/* error */
xfs_difree(
	struct xfs_trans *tp,		/* transaction pointer */
	xfs_ino_t	inode,		/* inode to be freed */
	struct xfs_bmap_free *flist,	/* extents to free */
	int		*delete,	/* set if inode cluster was deleted */
	xfs_ino_t	*first_ino);	/* first inode in deleted cluster */

int
xfs_imap(
	struct xfs_mount *mp,		/* file system mount structure */
	struct xfs_trans *tp,		/* transaction pointer */
	xfs_ino_t	ino,		/* inode to locate */
	struct xfs_imap	*imap,		/* location map structure */
	uint		flags);		/* flags for inode btree lookup */

void
xfs_ialloc_compute_maxlevels(
	struct xfs_mount *mp);		/* file system mount structure */

void
xfs_ialloc_log_agi(
	struct xfs_trans *tp,		/* transaction pointer */
	struct xfs_buf	*bp,		/* allocation group header buffer */
	int		fields);	/* bitmask of fields to log */

int					/* error */
xfs_ialloc_read_agi(
	struct xfs_mount *mp,		/* file system mount structure */
	struct xfs_trans *tp,		/* transaction pointer */
	xfs_agnumber_t	agno,		/* allocation group number */
	struct xfs_buf	**bpp);		/* allocation group hdr buf */

int
xfs_ialloc_pagi_init(
	struct xfs_mount *mp,		/* file system mount structure */
	struct xfs_trans *tp,		/* transaction pointer */
        xfs_agnumber_t  agno);		/* allocation group number */

int xfs_inobt_lookup(struct xfs_btree_cur *cur, xfs_agino_t ino,
		xfs_lookup_t dir, int *stat);

extern int xfs_inobt_get_rec(struct xfs_btree_cur *cur,
		xfs_inobt_rec_incore_t *rec, int *stat);

#endif	/* __XFS_IALLOC_H__ */


#ifndef __XFS_ALLOC_H__
#define	__XFS_ALLOC_H__

struct xfs_buf;
struct xfs_mount;
struct xfs_perag;
struct xfs_trans;
struct xfs_busy_extent;

typedef enum xfs_alloctype
{
	XFS_ALLOCTYPE_ANY_AG,		/* allocate anywhere, use rotor */
	XFS_ALLOCTYPE_FIRST_AG,		/* ... start at ag 0 */
	XFS_ALLOCTYPE_START_AG,		/* anywhere, start in this a.g. */
	XFS_ALLOCTYPE_THIS_AG,		/* anywhere in this a.g. */
	XFS_ALLOCTYPE_START_BNO,	/* near this block else anywhere */
	XFS_ALLOCTYPE_NEAR_BNO,		/* in this a.g. and near this block */
	XFS_ALLOCTYPE_THIS_BNO		/* at exactly this block */
} xfs_alloctype_t;

#define XFS_ALLOC_TYPES \
	{ XFS_ALLOCTYPE_ANY_AG,		"ANY_AG" }, \
	{ XFS_ALLOCTYPE_FIRST_AG,	"FIRST_AG" }, \
	{ XFS_ALLOCTYPE_START_AG,	"START_AG" }, \
	{ XFS_ALLOCTYPE_THIS_AG,	"THIS_AG" }, \
	{ XFS_ALLOCTYPE_START_BNO,	"START_BNO" }, \
	{ XFS_ALLOCTYPE_NEAR_BNO,	"NEAR_BNO" }, \
	{ XFS_ALLOCTYPE_THIS_BNO,	"THIS_BNO" }

#define	XFS_ALLOC_FLAG_TRYLOCK	0x00000001  /* use trylock for buffer locking */
#define	XFS_ALLOC_FLAG_FREEING	0x00000002  /* indicate caller is freeing extents*/

#define XFS_ALLOC_SET_ASIDE(mp)  (4 + ((mp)->m_sb.sb_agcount * 4))

typedef struct xfs_alloc_arg {
	struct xfs_trans *tp;		/* transaction pointer */
	struct xfs_mount *mp;		/* file system mount point */
	struct xfs_buf	*agbp;		/* buffer for a.g. freelist header */
	struct xfs_perag *pag;		/* per-ag struct for this agno */
	xfs_fsblock_t	fsbno;		/* file system block number */
	xfs_agnumber_t	agno;		/* allocation group number */
	xfs_agblock_t	agbno;		/* allocation group-relative block # */
	xfs_extlen_t	minlen;		/* minimum size of extent */
	xfs_extlen_t	maxlen;		/* maximum size of extent */
	xfs_extlen_t	mod;		/* mod value for extent size */
	xfs_extlen_t	prod;		/* prod value for extent size */
	xfs_extlen_t	minleft;	/* min blocks must be left after us */
	xfs_extlen_t	total;		/* total blocks needed in xaction */
	xfs_extlen_t	alignment;	/* align answer to multiple of this */
	xfs_extlen_t	minalignslop;	/* slop for minlen+alignment calcs */
	xfs_extlen_t	len;		/* output: actual size of extent */
	xfs_alloctype_t	type;		/* allocation type XFS_ALLOCTYPE_... */
	xfs_alloctype_t	otype;		/* original allocation type */
	char		wasdel;		/* set if allocation was prev delayed */
	char		wasfromfl;	/* set if allocation is from freelist */
	char		isfl;		/* set if is freelist blocks - !acctg */
	char		userdata;	/* set if this is user data */
	xfs_fsblock_t	firstblock;	/* io first block allocated */
} xfs_alloc_arg_t;

#define XFS_ALLOC_USERDATA		1	/* allocation is for user data*/
#define XFS_ALLOC_INITIAL_USER_DATA	2	/* special case start of file */

xfs_extlen_t
xfs_alloc_longest_free_extent(struct xfs_mount *mp,
		struct xfs_perag *pag);

#ifdef __KERNEL__

void
xfs_alloc_busy_insert(xfs_trans_t *tp,
		xfs_agnumber_t agno,
		xfs_agblock_t bno,
		xfs_extlen_t len);

void
xfs_alloc_busy_clear(struct xfs_mount *mp, struct xfs_busy_extent *busyp);

#endif	/* __KERNEL__ */

void
xfs_alloc_compute_maxlevels(
	struct xfs_mount	*mp);	/* file system mount structure */

int				/* error */
xfs_alloc_get_freelist(
	struct xfs_trans *tp,	/* transaction pointer */
	struct xfs_buf	*agbp,	/* buffer containing the agf structure */
	xfs_agblock_t	*bnop,	/* block address retrieved from freelist */
	int		btreeblk); /* destination is a AGF btree */

void
xfs_alloc_log_agf(
	struct xfs_trans *tp,	/* transaction pointer */
	struct xfs_buf	*bp,	/* buffer for a.g. freelist header */
	int		fields);/* mask of fields to be logged (XFS_AGF_...) */

int				/* error */
xfs_alloc_pagf_init(
	struct xfs_mount *mp,	/* file system mount structure */
	struct xfs_trans *tp,	/* transaction pointer */
	xfs_agnumber_t	agno,	/* allocation group number */
	int		flags);	/* XFS_ALLOC_FLAGS_... */

int				/* error */
xfs_alloc_put_freelist(
	struct xfs_trans *tp,	/* transaction pointer */
	struct xfs_buf	*agbp,	/* buffer for a.g. freelist header */
	struct xfs_buf	*agflbp,/* buffer for a.g. free block array */
	xfs_agblock_t	bno,	/* block being freed */
	int		btreeblk); /* owner was a AGF btree */

int					/* error  */
xfs_alloc_read_agf(
	struct xfs_mount *mp,		/* mount point structure */
	struct xfs_trans *tp,		/* transaction pointer */
	xfs_agnumber_t	agno,		/* allocation group number */
	int		flags,		/* XFS_ALLOC_FLAG_... */
	struct xfs_buf	**bpp);		/* buffer for the ag freelist header */

int				/* error */
xfs_alloc_vextent(
	xfs_alloc_arg_t	*args);	/* allocation argument structure */

int				/* error */
xfs_free_extent(
	struct xfs_trans *tp,	/* transaction pointer */
	xfs_fsblock_t	bno,	/* starting block number of extent */
	xfs_extlen_t	len);	/* length of extent */

#endif	/* __XFS_ALLOC_H__ */

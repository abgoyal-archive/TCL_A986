
#ifndef __XFS_BMAP_H__
#define	__XFS_BMAP_H__

struct getbmap;
struct xfs_bmbt_irec;
struct xfs_ifork;
struct xfs_inode;
struct xfs_mount;
struct xfs_trans;

extern kmem_zone_t	*xfs_bmap_free_item_zone;

typedef struct xfs_extdelta
{
	xfs_fileoff_t		xed_startoff;	/* offset of range */
	xfs_filblks_t		xed_blockcount;	/* blocks in range */
} xfs_extdelta_t;

typedef struct xfs_bmap_free_item
{
	xfs_fsblock_t		xbfi_startblock;/* starting fs block number */
	xfs_extlen_t		xbfi_blockcount;/* number of blocks in extent */
	struct xfs_bmap_free_item *xbfi_next;	/* link to next entry */
} xfs_bmap_free_item_t;

typedef	struct xfs_bmap_free
{
	xfs_bmap_free_item_t	*xbf_first;	/* list of to-be-free extents */
	int			xbf_count;	/* count of items on list */
	int			xbf_low;	/* alloc in low mode */
} xfs_bmap_free_t;

#define	XFS_BMAP_MAX_NMAP	4

#define	XFS_BMAPI_WRITE		0x001	/* write operation: allocate space */
#define XFS_BMAPI_DELAY		0x002	/* delayed write operation */
#define XFS_BMAPI_ENTIRE	0x004	/* return entire extent, not trimmed */
#define XFS_BMAPI_METADATA	0x008	/* mapping metadata not user data */
#define XFS_BMAPI_EXACT		0x010	/* allocate only to spec'd bounds */
#define XFS_BMAPI_ATTRFORK	0x020	/* use attribute fork not data */
#define XFS_BMAPI_ASYNC		0x040	/* bunmapi xactions can be async */
#define XFS_BMAPI_RSVBLOCKS	0x080	/* OK to alloc. reserved data blocks */
#define	XFS_BMAPI_PREALLOC	0x100	/* preallocation op: unwritten space */
#define	XFS_BMAPI_IGSTATE	0x200	/* Ignore state - */
					/* combine contig. space */
#define	XFS_BMAPI_CONTIG	0x400	/* must allocate only one extent */
/*	XFS_BMAPI_DIRECT_IO	0x800	*/
#define XFS_BMAPI_CONVERT	0x1000	/* unwritten extent conversion - */
					/* need write cache flushing and no */
					/* additional allocation alignments */

#define XFS_BMAPI_FLAGS \
	{ XFS_BMAPI_WRITE,	"WRITE" }, \
	{ XFS_BMAPI_DELAY,	"DELAY" }, \
	{ XFS_BMAPI_ENTIRE,	"ENTIRE" }, \
	{ XFS_BMAPI_METADATA,	"METADATA" }, \
	{ XFS_BMAPI_EXACT,	"EXACT" }, \
	{ XFS_BMAPI_ATTRFORK,	"ATTRFORK" }, \
	{ XFS_BMAPI_ASYNC,	"ASYNC" }, \
	{ XFS_BMAPI_RSVBLOCKS,	"RSVBLOCKS" }, \
	{ XFS_BMAPI_PREALLOC,	"PREALLOC" }, \
	{ XFS_BMAPI_IGSTATE,	"IGSTATE" }, \
	{ XFS_BMAPI_CONTIG,	"CONTIG" }, \
	{ XFS_BMAPI_CONVERT,	"CONVERT" }


static inline int xfs_bmapi_aflag(int w)
{
	return (w == XFS_ATTR_FORK ? XFS_BMAPI_ATTRFORK : 0);
}

#define	DELAYSTARTBLOCK		((xfs_fsblock_t)-1LL)
#define	HOLESTARTBLOCK		((xfs_fsblock_t)-2LL)

static inline void xfs_bmap_init(xfs_bmap_free_t *flp, xfs_fsblock_t *fbp)
{
	((flp)->xbf_first = NULL, (flp)->xbf_count = 0, \
		(flp)->xbf_low = 0, *(fbp) = NULLFSBLOCK);
}

typedef struct xfs_bmalloca {
	xfs_fsblock_t		firstblock; /* i/o first block allocated */
	xfs_fsblock_t		rval;	/* starting block of new extent */
	xfs_fileoff_t		off;	/* offset in file filling in */
	struct xfs_trans	*tp;	/* transaction pointer */
	struct xfs_inode	*ip;	/* incore inode pointer */
	struct xfs_bmbt_irec	*prevp;	/* extent before the new one */
	struct xfs_bmbt_irec	*gotp;	/* extent after, or delayed */
	xfs_extlen_t		alen;	/* i/o length asked/allocated */
	xfs_extlen_t		total;	/* total blocks needed for xaction */
	xfs_extlen_t		minlen;	/* minimum allocation size (blocks) */
	xfs_extlen_t		minleft; /* amount must be left after alloc */
	char			eof;	/* set if allocating past last extent */
	char			wasdel;	/* replacing a delayed allocation */
	char			userdata;/* set if is user data */
	char			low;	/* low on space, using seq'l ags */
	char			aeof;	/* allocated space at eof */
	char			conv;	/* overwriting unwritten extents */
} xfs_bmalloca_t;

#define BMAP_LEFT_CONTIG	(1 << 0)
#define BMAP_RIGHT_CONTIG	(1 << 1)
#define BMAP_LEFT_FILLING	(1 << 2)
#define BMAP_RIGHT_FILLING	(1 << 3)
#define BMAP_LEFT_DELAY		(1 << 4)
#define BMAP_RIGHT_DELAY	(1 << 5)
#define BMAP_LEFT_VALID		(1 << 6)
#define BMAP_RIGHT_VALID	(1 << 7)
#define BMAP_ATTRFORK		(1 << 8)

#define XFS_BMAP_EXT_FLAGS \
	{ BMAP_LEFT_CONTIG,	"LC" }, \
	{ BMAP_RIGHT_CONTIG,	"RC" }, \
	{ BMAP_LEFT_FILLING,	"LF" }, \
	{ BMAP_RIGHT_FILLING,	"RF" }, \
	{ BMAP_ATTRFORK,	"ATTR" }

#if defined(__KERNEL) && defined(DEBUG)
void
xfs_bmap_trace_exlist(
	struct xfs_inode	*ip,		/* incore inode pointer */
	xfs_extnum_t		cnt,		/* count of entries in list */
	int			whichfork,
	unsigned long		caller_ip);	/* data or attr fork */
#define	XFS_BMAP_TRACE_EXLIST(ip,c,w)	\
	xfs_bmap_trace_exlist(ip,c,w, _THIS_IP_)
#else
#define	XFS_BMAP_TRACE_EXLIST(ip,c,w)
#endif

int					/* error code */
xfs_bmap_add_attrfork(
	struct xfs_inode	*ip,	/* incore inode pointer */
	int			size,	/* space needed for new attribute */
	int			rsvd);	/* flag for reserved block allocation */

void
xfs_bmap_add_free(
	xfs_fsblock_t		bno,		/* fs block number of extent */
	xfs_filblks_t		len,		/* length of extent */
	xfs_bmap_free_t		*flist,		/* list of extents */
	struct xfs_mount	*mp);		/* mount point structure */

void
xfs_bmap_cancel(
	xfs_bmap_free_t		*flist);	/* free list to clean up */

void
xfs_bmap_compute_maxlevels(
	struct xfs_mount	*mp,	/* file system mount structure */
	int			whichfork);	/* data or attr fork */

int						/* error */
xfs_bmap_first_unused(
	struct xfs_trans	*tp,		/* transaction pointer */
	struct xfs_inode	*ip,		/* incore inode */
	xfs_extlen_t		len,		/* size of hole to find */
	xfs_fileoff_t		*unused,	/* unused block num */
	int			whichfork);	/* data or attr fork */

int						/* error */
xfs_bmap_last_before(
	struct xfs_trans	*tp,		/* transaction pointer */
	struct xfs_inode	*ip,		/* incore inode */
	xfs_fileoff_t		*last_block,	/* last block */
	int			whichfork);	/* data or attr fork */

int						/* error */
xfs_bmap_last_offset(
	struct xfs_trans	*tp,		/* transaction pointer */
	struct xfs_inode	*ip,		/* incore inode */
	xfs_fileoff_t		*unused,	/* last block num */
	int			whichfork);	/* data or attr fork */

int
xfs_bmap_one_block(
	struct xfs_inode	*ip,		/* incore inode */
	int			whichfork);	/* data or attr fork */

int						/* error */
xfs_bmap_read_extents(
	struct xfs_trans	*tp,		/* transaction pointer */
	struct xfs_inode	*ip,		/* incore inode */
	int			whichfork);	/* data or attr fork */

int						/* error */
xfs_bmapi(
	struct xfs_trans	*tp,		/* transaction pointer */
	struct xfs_inode	*ip,		/* incore inode */
	xfs_fileoff_t		bno,		/* starting file offs. mapped */
	xfs_filblks_t		len,		/* length to map in file */
	int			flags,		/* XFS_BMAPI_... */
	xfs_fsblock_t		*firstblock,	/* first allocated block
						   controls a.g. for allocs */
	xfs_extlen_t		total,		/* total blocks needed */
	struct xfs_bmbt_irec	*mval,		/* output: map values */
	int			*nmap,		/* i/o: mval size/count */
	xfs_bmap_free_t		*flist,		/* i/o: list extents to free */
	xfs_extdelta_t		*delta);	/* o: change made to incore
						   extents */

int						/* error */
xfs_bmapi_single(
	struct xfs_trans	*tp,		/* transaction pointer */
	struct xfs_inode	*ip,		/* incore inode */
	int			whichfork,	/* data or attr fork */
	xfs_fsblock_t		*fsb,		/* output: mapped block */
	xfs_fileoff_t		bno);		/* starting file offs. mapped */

int						/* error */
xfs_bunmapi(
	struct xfs_trans	*tp,		/* transaction pointer */
	struct xfs_inode	*ip,		/* incore inode */
	xfs_fileoff_t		bno,		/* starting offset to unmap */
	xfs_filblks_t		len,		/* length to unmap in file */
	int			flags,		/* XFS_BMAPI_... */
	xfs_extnum_t		nexts,		/* number of extents max */
	xfs_fsblock_t		*firstblock,	/* first allocated block
						   controls a.g. for allocs */
	xfs_bmap_free_t		*flist,		/* i/o: list extents to free */
	xfs_extdelta_t		*delta,		/* o: change made to incore
						   extents */
	int			*done);		/* set if not done yet */

int
xfs_check_nostate_extents(
	struct xfs_ifork	*ifp,
	xfs_extnum_t		idx,
	xfs_extnum_t		num);

uint
xfs_default_attroffset(
	struct xfs_inode	*ip);

#ifdef __KERNEL__

int						/* error */
xfs_bmap_finish(
	struct xfs_trans	**tp,		/* transaction pointer addr */
	xfs_bmap_free_t		*flist,		/* i/o: list extents to free */
	int			*committed);	/* xact committed or not */

/* bmap to userspace formatter - copy to user & advance pointer */
typedef int (*xfs_bmap_format_t)(void **, struct getbmapx *, int *);

int						/* error code */
xfs_getbmap(
	xfs_inode_t		*ip,
	struct getbmapx		*bmv,		/* user bmap structure */
	xfs_bmap_format_t	formatter,	/* format to user */
	void			*arg);		/* formatter arg */

int
xfs_bmap_eof(
	struct xfs_inode        *ip,
	xfs_fileoff_t           endoff,
	int                     whichfork,
	int                     *eof);

int
xfs_bmap_count_blocks(
	xfs_trans_t		*tp,
	struct xfs_inode	*ip,
	int			whichfork,
	int			*count);

#endif	/* __KERNEL__ */

#endif	/* __XFS_BMAP_H__ */

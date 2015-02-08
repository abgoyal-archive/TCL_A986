
#ifndef __XFS_RTALLOC_H__
#define	__XFS_RTALLOC_H__

struct xfs_mount;
struct xfs_trans;

/* Min and max rt extent sizes, specified in bytes */
#define	XFS_MAX_RTEXTSIZE	(1024 * 1024 * 1024)	/* 1GB */
#define	XFS_DFL_RTEXTSIZE	(64 * 1024)	        /* 64kB */
#define	XFS_MIN_RTEXTSIZE	(4 * 1024)		/* 4kB */

#define	XFS_NBBYLOG	3		/* log2(NBBY) */
#define	XFS_WORDLOG	2		/* log2(sizeof(xfs_rtword_t)) */
#define	XFS_NBWORDLOG	(XFS_NBBYLOG + XFS_WORDLOG)
#define	XFS_NBWORD	(1 << XFS_NBWORDLOG)
#define	XFS_WORDMASK	((1 << XFS_WORDLOG) - 1)

#define	XFS_BLOCKSIZE(mp)	((mp)->m_sb.sb_blocksize)
#define	XFS_BLOCKMASK(mp)	((mp)->m_blockmask)
#define	XFS_BLOCKWSIZE(mp)	((mp)->m_blockwsize)
#define	XFS_BLOCKWMASK(mp)	((mp)->m_blockwmask)

#define	XFS_SUMOFFS(mp,ls,bb)	((int)((ls) * (mp)->m_sb.sb_rbmblocks + (bb)))
#define	XFS_SUMOFFSTOBLOCK(mp,s)	\
	(((s) * (uint)sizeof(xfs_suminfo_t)) >> (mp)->m_sb.sb_blocklog)
#define	XFS_SUMPTR(mp,bp,so)	\
	((xfs_suminfo_t *)((char *)XFS_BUF_PTR(bp) + \
		(((so) * (uint)sizeof(xfs_suminfo_t)) & XFS_BLOCKMASK(mp))))

#define	XFS_BITTOBLOCK(mp,bi)	((bi) >> (mp)->m_blkbit_log)
#define	XFS_BLOCKTOBIT(mp,bb)	((bb) << (mp)->m_blkbit_log)
#define	XFS_BITTOWORD(mp,bi)	\
	((int)(((bi) >> XFS_NBWORDLOG) & XFS_BLOCKWMASK(mp)))

#define	XFS_RTMIN(a,b)	((a) < (b) ? (a) : (b))
#define	XFS_RTMAX(a,b)	((a) > (b) ? (a) : (b))

#define	XFS_RTLOBIT(w)	xfs_lowbit32(w)
#define	XFS_RTHIBIT(w)	xfs_highbit32(w)

#if XFS_BIG_BLKNOS
#define	XFS_RTBLOCKLOG(b)	xfs_highbit64(b)
#else
#define	XFS_RTBLOCKLOG(b)	xfs_highbit32(b)
#endif


#ifdef __KERNEL__

#ifdef CONFIG_XFS_RT

int					/* error */
xfs_rtallocate_extent(
	struct xfs_trans	*tp,	/* transaction pointer */
	xfs_rtblock_t		bno,	/* starting block number to allocate */
	xfs_extlen_t		minlen,	/* minimum length to allocate */
	xfs_extlen_t		maxlen,	/* maximum length to allocate */
	xfs_extlen_t		*len,	/* out: actual length allocated */
	xfs_alloctype_t		type,	/* allocation type XFS_ALLOCTYPE... */
	int			wasdel,	/* was a delayed allocation extent */
	xfs_extlen_t		prod,	/* extent product factor */
	xfs_rtblock_t		*rtblock); /* out: start block allocated */

int					/* error */
xfs_rtfree_extent(
	struct xfs_trans	*tp,	/* transaction pointer */
	xfs_rtblock_t		bno,	/* starting block number to free */
	xfs_extlen_t		len);	/* length of extent freed */

int					/* error */
xfs_rtmount_init(
	struct xfs_mount	*mp);	/* file system mount structure */
void
xfs_rtunmount_inodes(
	struct xfs_mount	*mp);

int					/* error */
xfs_rtmount_inodes(
	struct xfs_mount	*mp);	/* file system mount structure */

int					/* error */
xfs_rtpick_extent(
	struct xfs_mount	*mp,	/* file system mount point */
	struct xfs_trans	*tp,	/* transaction pointer */
	xfs_extlen_t		len,	/* allocation length (rtextents) */
	xfs_rtblock_t		*pick);	/* result rt extent */

int
xfs_growfs_rt(
	struct xfs_mount	*mp,	/* file system mount structure */
	xfs_growfs_rt_t		*in);	/* user supplied growfs struct */

#else
# define xfs_rtallocate_extent(t,b,min,max,l,a,f,p,rb)  (ENOSYS)
# define xfs_rtfree_extent(t,b,l)                       (ENOSYS)
# define xfs_rtpick_extent(m,t,l,rb)                    (ENOSYS)
# define xfs_growfs_rt(mp,in)                           (ENOSYS)
static inline int		/* error */
xfs_rtmount_init(
	xfs_mount_t	*mp)	/* file system mount structure */
{
	if (mp->m_sb.sb_rblocks == 0)
		return 0;

	cmn_err(CE_WARN, "XFS: Not built with CONFIG_XFS_RT");
	return ENOSYS;
}
# define xfs_rtmount_inodes(m)  (((mp)->m_sb.sb_rblocks == 0)? 0 : (ENOSYS))
# define xfs_rtunmount_inodes(m)
#endif	/* CONFIG_XFS_RT */

#endif	/* __KERNEL__ */

#endif	/* __XFS_RTALLOC_H__ */

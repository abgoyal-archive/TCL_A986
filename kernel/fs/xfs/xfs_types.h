
#ifndef __XFS_TYPES_H__
#define	__XFS_TYPES_H__

#ifdef __KERNEL__

typedef signed char		__int8_t;
typedef unsigned char		__uint8_t;
typedef signed short int	__int16_t;
typedef unsigned short int	__uint16_t;
typedef signed int		__int32_t;
typedef unsigned int		__uint32_t;
typedef signed long long int	__int64_t;
typedef unsigned long long int	__uint64_t;

typedef enum { B_FALSE,B_TRUE }	boolean_t;
typedef __uint32_t		prid_t;		/* project ID */
typedef __uint32_t		inst_t;		/* an instruction */

typedef __s64			xfs_off_t;	/* <file offset> type */
typedef unsigned long long	xfs_ino_t;	/* <inode> type */
typedef __s64			xfs_daddr_t;	/* <disk address> type */
typedef char *			xfs_caddr_t;	/* <core address> type */
typedef __u32			xfs_dev_t;
typedef __u32			xfs_nlink_t;

/* __psint_t is the same size as a pointer */
#if (BITS_PER_LONG == 32)
typedef __int32_t __psint_t;
typedef __uint32_t __psunsigned_t;
#elif (BITS_PER_LONG == 64)
typedef __int64_t __psint_t;
typedef __uint64_t __psunsigned_t;
#else
#error BITS_PER_LONG must be 32 or 64
#endif

#endif	/* __KERNEL__ */

typedef __uint32_t	xfs_agblock_t;	/* blockno in alloc. group */
typedef	__uint32_t	xfs_extlen_t;	/* extent length in blocks */
typedef	__uint32_t	xfs_agnumber_t;	/* allocation group number */
typedef __int32_t	xfs_extnum_t;	/* # of extents in a file */
typedef __int16_t	xfs_aextnum_t;	/* # extents in an attribute fork */
typedef	__int64_t	xfs_fsize_t;	/* bytes in a file */
typedef __uint64_t	xfs_ufsize_t;	/* unsigned bytes in a file */

typedef	__int32_t	xfs_suminfo_t;	/* type of bitmap summary info */
typedef	__int32_t	xfs_rtword_t;	/* word type for bitmap manipulations */

typedef	__int64_t	xfs_lsn_t;	/* log sequence number */
typedef	__int32_t	xfs_tid_t;	/* transaction identifier */

typedef	__uint32_t	xfs_dablk_t;	/* dir/attr block number (in file) */
typedef	__uint32_t	xfs_dahash_t;	/* dir/attr hash value */

typedef __uint16_t	xfs_prid_t;	/* prid_t truncated to 16bits in XFS */

typedef __uint32_t	xlog_tid_t;	/* transaction ID type */

typedef __uint64_t	xfs_dfsbno_t;	/* blockno in filesystem (agno|agbno) */
typedef __uint64_t	xfs_drfsbno_t;	/* blockno in filesystem (raw) */
typedef	__uint64_t	xfs_drtbno_t;	/* extent (block) in realtime area */
typedef	__uint64_t	xfs_dfiloff_t;	/* block number in a file */
typedef	__uint64_t	xfs_dfilblks_t;	/* number of blocks in a file */

#if XFS_BIG_BLKNOS
typedef	__uint64_t	xfs_fsblock_t;	/* blockno in filesystem (agno|agbno) */
typedef __uint64_t	xfs_rfsblock_t;	/* blockno in filesystem (raw) */
typedef __uint64_t	xfs_rtblock_t;	/* extent (block) in realtime area */
typedef	__int64_t	xfs_srtblock_t;	/* signed version of xfs_rtblock_t */
#else
typedef	__uint32_t	xfs_fsblock_t;	/* blockno in filesystem (agno|agbno) */
typedef __uint32_t	xfs_rfsblock_t;	/* blockno in filesystem (raw) */
typedef __uint32_t	xfs_rtblock_t;	/* extent (block) in realtime area */
typedef	__int32_t	xfs_srtblock_t;	/* signed version of xfs_rtblock_t */
#endif
typedef __uint64_t	xfs_fileoff_t;	/* block number in a file */
typedef __int64_t	xfs_sfiloff_t;	/* signed block number in a file */
typedef __uint64_t	xfs_filblks_t;	/* number of blocks in a file */

#define	NULLDFSBNO	((xfs_dfsbno_t)-1)
#define	NULLDRFSBNO	((xfs_drfsbno_t)-1)
#define	NULLDRTBNO	((xfs_drtbno_t)-1)
#define	NULLDFILOFF	((xfs_dfiloff_t)-1)

#define	NULLFSBLOCK	((xfs_fsblock_t)-1)
#define	NULLRFSBLOCK	((xfs_rfsblock_t)-1)
#define	NULLRTBLOCK	((xfs_rtblock_t)-1)
#define	NULLFILEOFF	((xfs_fileoff_t)-1)

#define	NULLAGBLOCK	((xfs_agblock_t)-1)
#define	NULLAGNUMBER	((xfs_agnumber_t)-1)
#define	NULLEXTNUM	((xfs_extnum_t)-1)

#define NULLCOMMITLSN	((xfs_lsn_t)-1)

#define	MAXEXTLEN	((xfs_extlen_t)0x001fffff)	/* 21 bits */
#define	MAXEXTNUM	((xfs_extnum_t)0x7fffffff)	/* signed int */
#define	MAXAEXTNUM	((xfs_aextnum_t)0x7fff)		/* signed short */

#define MINDBTPTRS	3
#define MINABTPTRS	2

#define MAXNAMELEN	256

typedef enum {
	XFS_LOOKUP_EQi, XFS_LOOKUP_LEi, XFS_LOOKUP_GEi
} xfs_lookup_t;

typedef enum {
	XFS_BTNUM_BNOi, XFS_BTNUM_CNTi, XFS_BTNUM_BMAPi, XFS_BTNUM_INOi,
	XFS_BTNUM_MAX
} xfs_btnum_t;

struct xfs_name {
	const unsigned char	*name;
	int			len;
};

#endif	/* __XFS_TYPES_H__ */

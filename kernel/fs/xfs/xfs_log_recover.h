
#ifndef	__XFS_LOG_RECOVER_H__
#define __XFS_LOG_RECOVER_H__


#define XLOG_RHASH_BITS  4
#define XLOG_RHASH_SIZE	16
#define XLOG_RHASH_SHIFT 2
#define XLOG_RHASH(tid)	\
	((((__uint32_t)tid)>>XLOG_RHASH_SHIFT) & (XLOG_RHASH_SIZE-1))

#define XLOG_MAX_REGIONS_IN_ITEM   (XFS_MAX_BLOCKSIZE / XFS_BLF_CHUNK / 2 + 1)


typedef struct xlog_recover_item {
	struct list_head	ri_list;
	int			ri_type;
	int			ri_cnt;	/* count of regions found */
	int			ri_total;	/* total regions */
	xfs_log_iovec_t		*ri_buf;	/* ptr to regions buffer */
} xlog_recover_item_t;

struct xlog_tid;
typedef struct xlog_recover {
	struct hlist_node	r_list;
	xlog_tid_t		r_log_tid;	/* log's transaction id */
	xfs_trans_header_t	r_theader;	/* trans header for partial */
	int			r_state;	/* not needed */
	xfs_lsn_t		r_lsn;		/* xact lsn */
	struct list_head	r_itemq;	/* q for items */
} xlog_recover_t;

#define ITEM_TYPE(i)	(*(ushort *)(i)->ri_buf[0].i_addr)

#define	XLOG_BC_TABLE_SIZE	64

#define	XLOG_RECOVER_PASS1	1
#define	XLOG_RECOVER_PASS2	2

#endif	/* __XFS_LOG_RECOVER_H__ */

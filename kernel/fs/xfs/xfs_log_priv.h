
#ifndef	__XFS_LOG_PRIV_H__
#define __XFS_LOG_PRIV_H__

struct xfs_buf;
struct log;
struct xlog_ticket;
struct xfs_buf_cancel;
struct xfs_mount;


#define XLOG_MIN_ICLOGS		2
#define XLOG_MAX_ICLOGS		8
#define XLOG_HEADER_MAGIC_NUM	0xFEEDbabe	/* Invalid cycle number */
#define XLOG_VERSION_1		1
#define XLOG_VERSION_2		2		/* Large IClogs, Log sunit */
#define XLOG_VERSION_OKBITS	(XLOG_VERSION_1 | XLOG_VERSION_2)
#define XLOG_MIN_RECORD_BSIZE	(16*1024)	/* eventually 32k */
#define XLOG_BIG_RECORD_BSIZE	(32*1024)	/* 32k buffers */
#define XLOG_MAX_RECORD_BSIZE	(256*1024)
#define XLOG_HEADER_CYCLE_SIZE	(32*1024)	/* cycle data in header */
#define XLOG_MIN_RECORD_BSHIFT	14		/* 16384 == 1 << 14 */
#define XLOG_BIG_RECORD_BSHIFT	15		/* 32k == 1 << 15 */
#define XLOG_MAX_RECORD_BSHIFT	18		/* 256k == 1 << 18 */
#define XLOG_BTOLSUNIT(log, b)  (((b)+(log)->l_mp->m_sb.sb_logsunit-1) / \
                                 (log)->l_mp->m_sb.sb_logsunit)
#define XLOG_LSUNITTOB(log, su) ((su) * (log)->l_mp->m_sb.sb_logsunit)

#define XLOG_HEADER_SIZE	512

#define XLOG_REC_SHIFT(log) \
	BTOBB(1 << (xfs_sb_version_haslogv2(&log->l_mp->m_sb) ? \
	 XLOG_MAX_RECORD_BSHIFT : XLOG_BIG_RECORD_BSHIFT))
#define XLOG_TOTAL_REC_SHIFT(log) \
	BTOBB(XLOG_MAX_ICLOGS << (xfs_sb_version_haslogv2(&log->l_mp->m_sb) ? \
	 XLOG_MAX_RECORD_BSHIFT : XLOG_BIG_RECORD_BSHIFT))


static inline xfs_lsn_t xlog_assign_lsn(uint cycle, uint block)
{
	return ((xfs_lsn_t)cycle << 32) | block;
}

static inline uint xlog_get_cycle(char *ptr)
{
	if (be32_to_cpu(*(__be32 *)ptr) == XLOG_HEADER_MAGIC_NUM)
		return be32_to_cpu(*((__be32 *)ptr + 1));
	else
		return be32_to_cpu(*(__be32 *)ptr);
}

#define BLK_AVG(blk1, blk2)	((blk1+blk2) >> 1)

#ifdef __KERNEL__

static inline uint xlog_get_client_id(__be32 i)
{
	return be32_to_cpu(i) >> 24;
}

#define xlog_panic(args...)	cmn_err(CE_PANIC, ## args)
#define xlog_exit(args...)	cmn_err(CE_PANIC, ## args)
#define xlog_warn(args...)	cmn_err(CE_WARN, ## args)

#define XLOG_STATE_ACTIVE    0x0001 /* Current IC log being written to */
#define XLOG_STATE_WANT_SYNC 0x0002 /* Want to sync this iclog; no more writes */
#define XLOG_STATE_SYNCING   0x0004 /* This IC log is syncing */
#define XLOG_STATE_DONE_SYNC 0x0008 /* Done syncing to disk */
#define XLOG_STATE_DO_CALLBACK \
			     0x0010 /* Process callback functions */
#define XLOG_STATE_CALLBACK  0x0020 /* Callback functions now */
#define XLOG_STATE_DIRTY     0x0040 /* Dirty IC log, not ready for ACTIVE status*/
#define XLOG_STATE_IOERROR   0x0080 /* IO error happened in sync'ing log */
#define XLOG_STATE_ALL	     0x7FFF /* All possible valid flags */
#define XLOG_STATE_NOTUSED   0x8000 /* This IC log not being used */
#endif	/* __KERNEL__ */

#define XLOG_START_TRANS	0x01	/* Start a new transaction */
#define XLOG_COMMIT_TRANS	0x02	/* Commit this transaction */
#define XLOG_CONTINUE_TRANS	0x04	/* Cont this trans into new region */
#define XLOG_WAS_CONT_TRANS	0x08	/* Cont this trans into new region */
#define XLOG_END_TRANS		0x10	/* End a continued transaction */
#define XLOG_UNMOUNT_TRANS	0x20	/* Unmount a filesystem transaction */

#ifdef __KERNEL__
#define XLOG_TIC_INITED		0x1	/* has been initialized */
#define XLOG_TIC_PERM_RESERV	0x2	/* permanent reservation */
#define XLOG_TIC_IN_Q		0x4

#define XLOG_TIC_FLAGS \
	{ XLOG_TIC_INITED,	"XLOG_TIC_INITED" }, \
	{ XLOG_TIC_PERM_RESERV,	"XLOG_TIC_PERM_RESERV" }, \
	{ XLOG_TIC_IN_Q,	"XLOG_TIC_IN_Q" }

#endif	/* __KERNEL__ */

#define XLOG_UNMOUNT_TYPE	0x556e	/* Un for Unmount */

#define XLOG_CHKSUM_MISMATCH	0x1	/* used only during recovery */
#define XLOG_ACTIVE_RECOVERY	0x2	/* in the middle of recovery */
#define	XLOG_RECOVERY_NEEDED	0x4	/* log was recovered */
#define XLOG_IO_ERROR		0x8	/* log hit an I/O error, and being
					   shutdown */

#ifdef __KERNEL__

#define XLOG_STATE_COVER_IDLE	0
#define XLOG_STATE_COVER_NEED	1
#define XLOG_STATE_COVER_DONE	2
#define XLOG_STATE_COVER_NEED2	3
#define XLOG_STATE_COVER_DONE2	4

#define XLOG_COVER_OPS		5


/* Ticket reservation region accounting */ 
#define XLOG_TIC_LEN_MAX	15

typedef struct xlog_res {
	uint	r_len;	/* region length		:4 */
	uint	r_type;	/* region's transaction type	:4 */
} xlog_res_t;

typedef struct xlog_ticket {
	sv_t		   t_wait;	 /* ticket wait queue            : 20 */
	struct xlog_ticket *t_next;	 /*			         :4|8 */
	struct xlog_ticket *t_prev;	 /*				 :4|8 */
	xlog_tid_t	   t_tid;	 /* transaction identifier	 : 4  */
	atomic_t	   t_ref;	 /* ticket reference count       : 4  */
	int		   t_curr_res;	 /* current reservation in bytes : 4  */
	int		   t_unit_res;	 /* unit reservation in bytes    : 4  */
	char		   t_ocnt;	 /* original count		 : 1  */
	char		   t_cnt;	 /* current count		 : 1  */
	char		   t_clientid;	 /* who does this belong to;	 : 1  */
	char		   t_flags;	 /* properties of reservation	 : 1  */
	uint		   t_trans_type; /* transaction type             : 4  */

        /* reservation array fields */
	uint		   t_res_num;                    /* num in array : 4 */
	uint		   t_res_num_ophdrs;		 /* num op hdrs  : 4 */
	uint		   t_res_arr_sum;		 /* array sum    : 4 */
	uint		   t_res_o_flow;		 /* sum overflow : 4 */
	xlog_res_t	   t_res_arr[XLOG_TIC_LEN_MAX];  /* array of res : 8 * 15 */ 
} xlog_ticket_t;

#endif


typedef struct xlog_op_header {
	__be32	   oh_tid;	/* transaction id of operation	:  4 b */
	__be32	   oh_len;	/* bytes in data region		:  4 b */
	__u8	   oh_clientid;	/* who sent me this		:  1 b */
	__u8	   oh_flags;	/*				:  1 b */
	__u16	   oh_res2;	/* 32 bit align			:  2 b */
} xlog_op_header_t;


/* valid values for h_fmt */
#define XLOG_FMT_UNKNOWN  0
#define XLOG_FMT_LINUX_LE 1
#define XLOG_FMT_LINUX_BE 2
#define XLOG_FMT_IRIX_BE  3

/* our fmt */
#ifdef XFS_NATIVE_HOST
#define XLOG_FMT XLOG_FMT_LINUX_BE
#else
#define XLOG_FMT XLOG_FMT_LINUX_LE
#endif

typedef struct xlog_rec_header {
	__be32	  h_magicno;	/* log record (LR) identifier		:  4 */
	__be32	  h_cycle;	/* write cycle of log			:  4 */
	__be32	  h_version;	/* LR version				:  4 */
	__be32	  h_len;	/* len in bytes; should be 64-bit aligned: 4 */
	__be64	  h_lsn;	/* lsn of this LR			:  8 */
	__be64	  h_tail_lsn;	/* lsn of 1st LR w/ buffers not committed: 8 */
	__be32	  h_chksum;	/* may not be used; non-zero if used	:  4 */
	__be32	  h_prev_block; /* block number to previous LR		:  4 */
	__be32	  h_num_logops;	/* number of log operations in this LR	:  4 */
	__be32	  h_cycle_data[XLOG_HEADER_CYCLE_SIZE / BBSIZE];
	/* new fields */
	__be32    h_fmt;        /* format of log record                 :  4 */
	uuid_t	  h_fs_uuid;    /* uuid of FS                           : 16 */
	__be32	  h_size;	/* iclog size				:  4 */
} xlog_rec_header_t;

typedef struct xlog_rec_ext_header {
	__be32	  xh_cycle;	/* write cycle of log			: 4 */
	__be32	  xh_cycle_data[XLOG_HEADER_CYCLE_SIZE / BBSIZE]; /*	: 256 */
} xlog_rec_ext_header_t;

#ifdef __KERNEL__

typedef union xlog_in_core2 {
	xlog_rec_header_t	hic_header;
	xlog_rec_ext_header_t	hic_xheader;
	char			hic_sector[XLOG_HEADER_SIZE];
} xlog_in_core_2_t;

typedef struct xlog_in_core {
	sv_t			ic_force_wait;
	sv_t			ic_write_wait;
	struct xlog_in_core	*ic_next;
	struct xlog_in_core	*ic_prev;
	struct xfs_buf		*ic_bp;
	struct log		*ic_log;
	int			ic_size;
	int			ic_offset;
	int			ic_bwritecnt;
	unsigned short		ic_state;
	char			*ic_datap;	/* pointer to iclog data */

	/* Callback structures need their own cacheline */
	spinlock_t		ic_callback_lock ____cacheline_aligned_in_smp;
	xfs_log_callback_t	*ic_callback;
	xfs_log_callback_t	**ic_callback_tail;

	/* reference counts need their own cacheline */
	atomic_t		ic_refcnt ____cacheline_aligned_in_smp;
	xlog_in_core_2_t	*ic_data;
#define ic_header	ic_data->hic_header
} xlog_in_core_t;

struct xfs_cil;

struct xfs_cil_ctx {
	struct xfs_cil		*cil;
	xfs_lsn_t		sequence;	/* chkpt sequence # */
	xfs_lsn_t		start_lsn;	/* first LSN of chkpt commit */
	xfs_lsn_t		commit_lsn;	/* chkpt commit record lsn */
	struct xlog_ticket	*ticket;	/* chkpt ticket */
	int			nvecs;		/* number of regions */
	int			space_used;	/* aggregate size of regions */
	struct list_head	busy_extents;	/* busy extents in chkpt */
	struct xfs_log_vec	*lv_chain;	/* logvecs being pushed */
	xfs_log_callback_t	log_cb;		/* completion callback hook. */
	struct list_head	committing;	/* ctx committing list */
};

struct xfs_cil {
	struct log		*xc_log;
	struct list_head	xc_cil;
	spinlock_t		xc_cil_lock;
	struct xfs_cil_ctx	*xc_ctx;
	struct rw_semaphore	xc_ctx_lock;
	struct list_head	xc_committing;
	sv_t			xc_commit_wait;
};


#define XLOG_CIL_SPACE_LIMIT(log)	\
	(min((log->l_logsize >> 2), (8 * 1024 * 1024)))

typedef struct log {
	/* The following fields don't need locking */
	struct xfs_mount	*l_mp;	        /* mount point */
	struct xfs_ail		*l_ailp;	/* AIL log is working with */
	struct xfs_cil		*l_cilp;	/* CIL log is working with */
	struct xfs_buf		*l_xbuf;        /* extra buffer for log
						 * wrapping */
	struct xfs_buftarg	*l_targ;        /* buftarg of log */
	uint			l_flags;
	uint			l_quotaoffs_flag; /* XFS_DQ_*, for QUOTAOFFs */
	struct xfs_buf_cancel	**l_buf_cancel_table;
	int			l_iclog_hsize;  /* size of iclog header */
	int			l_iclog_heads;  /* # of iclog header sectors */
	uint			l_sectBBsize;   /* sector size in BBs (2^n) */
	int			l_iclog_size;	/* size of log in bytes */
	int			l_iclog_size_log; /* log power size of log */
	int			l_iclog_bufs;	/* number of iclog buffers */
	xfs_daddr_t		l_logBBstart;   /* start block of log */
	int			l_logsize;      /* size of log in bytes */
	int			l_logBBsize;    /* size of log in BB chunks */

	/* The following block of fields are changed while holding icloglock */
	sv_t			l_flush_wait ____cacheline_aligned_in_smp;
						/* waiting for iclog flush */
	int			l_covered_state;/* state of "covering disk
						 * log entries" */
	xlog_in_core_t		*l_iclog;       /* head log queue	*/
	spinlock_t		l_icloglock;    /* grab to change iclog state */
	xfs_lsn_t		l_tail_lsn;     /* lsn of 1st LR with unflushed
						 * buffers */
	xfs_lsn_t		l_last_sync_lsn;/* lsn of last LR on disk */
	int			l_curr_cycle;   /* Cycle number of log writes */
	int			l_prev_cycle;   /* Cycle number before last
						 * block increment */
	int			l_curr_block;   /* current logical log block */
	int			l_prev_block;   /* previous logical log block */

	/* The following block of fields are changed while holding grant_lock */
	spinlock_t		l_grant_lock ____cacheline_aligned_in_smp;
	xlog_ticket_t		*l_reserve_headq;
	xlog_ticket_t		*l_write_headq;
	int			l_grant_reserve_cycle;
	int			l_grant_reserve_bytes;
	int			l_grant_write_cycle;
	int			l_grant_write_bytes;

	/* The following field are used for debugging; need to hold icloglock */
#ifdef DEBUG
	char			*l_iclog_bak[XLOG_MAX_ICLOGS];
#endif

} xlog_t;

#define XLOG_FORCED_SHUTDOWN(log)	((log)->l_flags & XLOG_IO_ERROR)

/* common routines */
extern xfs_lsn_t xlog_assign_tail_lsn(struct xfs_mount *mp);
extern int	 xlog_recover(xlog_t *log);
extern int	 xlog_recover_finish(xlog_t *log);
extern void	 xlog_pack_data(xlog_t *log, xlog_in_core_t *iclog, int);

extern kmem_zone_t *xfs_log_ticket_zone;
struct xlog_ticket *xlog_ticket_alloc(struct log *log, int unit_bytes,
				int count, char client, uint xflags,
				int alloc_flags);


static inline void
xlog_write_adv_cnt(void **ptr, int *len, int *off, size_t bytes)
{
	*ptr += bytes;
	*len -= bytes;
	*off += bytes;
}

void	xlog_print_tic_res(struct xfs_mount *mp, struct xlog_ticket *ticket);
int	xlog_write(struct log *log, struct xfs_log_vec *log_vector,
				struct xlog_ticket *tic, xfs_lsn_t *start_lsn,
				xlog_in_core_t **commit_iclog, uint flags);

int	xlog_cil_init(struct log *log);
void	xlog_cil_init_post_recovery(struct log *log);
void	xlog_cil_destroy(struct log *log);

int	xlog_cil_push(struct log *log, int push_now);
xfs_lsn_t xlog_cil_push_lsn(struct log *log, xfs_lsn_t push_sequence);

#define XLOG_UNMOUNT_REC_TYPE	(-1U)

#endif	/* __KERNEL__ */

#endif	/* __XFS_LOG_PRIV_H__ */

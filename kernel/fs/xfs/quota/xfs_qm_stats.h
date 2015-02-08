
#ifndef __XFS_QM_STATS_H__
#define __XFS_QM_STATS_H__

#if defined(CONFIG_PROC_FS) && !defined(XFS_STATS_OFF)

struct xqmstats {
	__uint32_t		xs_qm_dqreclaims;
	__uint32_t		xs_qm_dqreclaim_misses;
	__uint32_t		xs_qm_dquot_dups;
	__uint32_t		xs_qm_dqcachemisses;
	__uint32_t		xs_qm_dqcachehits;
	__uint32_t		xs_qm_dqwants;
	__uint32_t		xs_qm_dqshake_reclaims;
	__uint32_t		xs_qm_dqinact_reclaims;
};

extern struct xqmstats xqmstats;

# define XQM_STATS_INC(count)	( (count)++ )

extern void xfs_qm_init_procfs(void);
extern void xfs_qm_cleanup_procfs(void);

#else

# define XQM_STATS_INC(count)	do { } while (0)

static inline void xfs_qm_init_procfs(void) { };
static inline void xfs_qm_cleanup_procfs(void) { };

#endif

#endif	/* __XFS_QM_STATS_H__ */

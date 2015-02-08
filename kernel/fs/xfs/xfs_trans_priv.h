
#ifndef __XFS_TRANS_PRIV_H__
#define	__XFS_TRANS_PRIV_H__

struct xfs_log_item;
struct xfs_log_item_desc;
struct xfs_mount;
struct xfs_trans;

struct xfs_log_item_desc	*xfs_trans_add_item(struct xfs_trans *,
					    struct xfs_log_item *);
void				xfs_trans_free_item(struct xfs_trans *,
					    struct xfs_log_item_desc *);
struct xfs_log_item_desc	*xfs_trans_find_item(struct xfs_trans *,
					     struct xfs_log_item *);
struct xfs_log_item_desc	*xfs_trans_first_item(struct xfs_trans *);
struct xfs_log_item_desc	*xfs_trans_next_item(struct xfs_trans *,
					     struct xfs_log_item_desc *);

void	xfs_trans_unlock_items(struct xfs_trans *tp, xfs_lsn_t commit_lsn);
void	xfs_trans_free_items(struct xfs_trans *tp, xfs_lsn_t commit_lsn,
				int flags);

void	xfs_trans_item_committed(struct xfs_log_item *lip,
				xfs_lsn_t commit_lsn, int aborted);
void	xfs_trans_unreserve_and_mod_sb(struct xfs_trans *tp);

struct xfs_ail_cursor {
	struct xfs_ail_cursor	*next;
	struct xfs_log_item	*item;
};

struct xfs_ail {
	struct xfs_mount	*xa_mount;
	struct list_head	xa_ail;
	uint			xa_gen;
	struct task_struct	*xa_task;
	xfs_lsn_t		xa_target;
	struct xfs_ail_cursor	xa_cursors;
	spinlock_t		xa_lock;
};

void			xfs_trans_ail_update(struct xfs_ail *ailp,
					struct xfs_log_item *lip, xfs_lsn_t lsn)
					__releases(ailp->xa_lock);
void			xfs_trans_ail_delete(struct xfs_ail *ailp,
					struct xfs_log_item *lip)
					__releases(ailp->xa_lock);
void			xfs_trans_ail_push(struct xfs_ail *, xfs_lsn_t);
void			xfs_trans_unlocked_item(struct xfs_ail *,
					xfs_log_item_t *);

xfs_lsn_t		xfs_trans_ail_tail(struct xfs_ail *ailp);

struct xfs_log_item	*xfs_trans_ail_cursor_first(struct xfs_ail *ailp,
					struct xfs_ail_cursor *cur,
					xfs_lsn_t lsn);
struct xfs_log_item	*xfs_trans_ail_cursor_next(struct xfs_ail *ailp,
					struct xfs_ail_cursor *cur);
void			xfs_trans_ail_cursor_done(struct xfs_ail *ailp,
					struct xfs_ail_cursor *cur);

long	xfsaild_push(struct xfs_ail *, xfs_lsn_t *);
void	xfsaild_wakeup(struct xfs_ail *, xfs_lsn_t);
int	xfsaild_start(struct xfs_ail *);
void	xfsaild_stop(struct xfs_ail *);

#if BITS_PER_LONG != 64
static inline void
xfs_trans_ail_copy_lsn(
	struct xfs_ail	*ailp,
	xfs_lsn_t	*dst,
	xfs_lsn_t	*src)
{
	ASSERT(sizeof(xfs_lsn_t) == 8);	/* don't lock if it shrinks */
	spin_lock(&ailp->xa_lock);
	*dst = *src;
	spin_unlock(&ailp->xa_lock);
}
#else
static inline void
xfs_trans_ail_copy_lsn(
	struct xfs_ail	*ailp,
	xfs_lsn_t	*dst,
	xfs_lsn_t	*src)
{
	ASSERT(sizeof(xfs_lsn_t) == 8);
	*dst = *src;
}
#endif
#endif	/* __XFS_TRANS_PRIV_H__ */

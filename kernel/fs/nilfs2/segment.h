
#ifndef _NILFS_SEGMENT_H
#define _NILFS_SEGMENT_H

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/nilfs2_fs.h>
#include "sb.h"

struct nilfs_recovery_info {
	int			ri_need_recovery;
	sector_t		ri_super_root;
	__u64			ri_cno;

	sector_t		ri_lsegs_start;
	sector_t		ri_lsegs_end;
	u64			ri_lsegs_start_seq;
	struct list_head	ri_used_segments;
	sector_t		ri_pseg_start;
	u64			ri_seq;
	__u64			ri_segnum;
	__u64			ri_nextnum;
};

/* ri_need_recovery */
#define NILFS_RECOVERY_SR_UPDATED	 1  /* The super root was updated */
#define NILFS_RECOVERY_ROLLFORWARD_DONE	 2  /* Rollforward was carried out */

struct nilfs_cstage {
	int			scnt;
	unsigned		flags;
	struct nilfs_inode_info *dirty_file_ptr;
	struct nilfs_inode_info *gc_inode_ptr;
};

struct nilfs_segment_buffer;

struct nilfs_segsum_pointer {
	struct buffer_head     *bh;
	unsigned		offset; /* offset in bytes */
};

struct nilfs_sc_info {
	struct super_block     *sc_super;
	struct nilfs_sb_info   *sc_sbi;

	unsigned long		sc_nblk_inc;

	struct list_head	sc_dirty_files;
	struct list_head	sc_gc_inodes;
	struct list_head	sc_copied_buffers;

	__u64		       *sc_freesegs;
	size_t			sc_nfreesegs;

	struct nilfs_inode_info *sc_dsync_inode;
	loff_t			sc_dsync_start;
	loff_t			sc_dsync_end;

	/* Segment buffers */
	struct list_head	sc_segbufs;
	struct list_head	sc_write_logs;
	unsigned long		sc_segbuf_nblocks;
	struct nilfs_segment_buffer *sc_curseg;

	struct nilfs_cstage	sc_stage;

	struct nilfs_segsum_pointer sc_finfo_ptr;
	struct nilfs_segsum_pointer sc_binfo_ptr;
	unsigned long		sc_blk_cnt;
	unsigned long		sc_datablk_cnt;
	unsigned long		sc_nblk_this_inc;
	time_t			sc_seg_ctime;

	unsigned long		sc_flags;

	spinlock_t		sc_state_lock;
	unsigned long		sc_state;
	unsigned long		sc_flush_request;

	wait_queue_head_t	sc_wait_request;
	wait_queue_head_t	sc_wait_daemon;
	wait_queue_head_t	sc_wait_task;

	__u32			sc_seq_request;
	__u32			sc_seq_accepted;
	__u32			sc_seq_done;

	int			sc_sync;
	unsigned long		sc_interval;
	unsigned long		sc_mjcp_freq;
	unsigned long		sc_lseg_stime;	/* in 1/HZ seconds */
	unsigned long		sc_watermark;

	struct timer_list	sc_timer;
	struct task_struct     *sc_task;
};

/* sc_flags */
enum {
	NILFS_SC_DIRTY,		/* One or more dirty meta-data blocks exist */
	NILFS_SC_UNCLOSED,	/* Logical segment is not closed */
	NILFS_SC_SUPER_ROOT,	/* The latest segment has a super root */
	NILFS_SC_PRIOR_FLUSH,	/* Requesting immediate flush without making a
				   checkpoint */
	NILFS_SC_HAVE_DELTA,	/* Next checkpoint will have update of files
				   other than DAT, cpfile, sufile, or files
				   moved by GC */
};

/* sc_state */
#define NILFS_SEGCTOR_QUIT	    0x0001  /* segctord is being destroyed */
#define NILFS_SEGCTOR_COMMIT	    0x0004  /* committed transaction exists */

#define NILFS_SC_CLEANUP_RETRY	    3  /* Retry count of construction when
					  destroying segctord */

#define NILFS_SC_DEFAULT_TIMEOUT    5   /* Timeout value of dirty blocks.
					   It triggers construction of a
					   logical segment with a super root */
#define NILFS_SC_DEFAULT_SR_FREQ    30  /* Maximum frequency of super root
					   creation */

#define NILFS_SC_DEFAULT_WATERMARK  3600

/* super.c */
extern struct kmem_cache *nilfs_transaction_cachep;

/* segment.c */
extern void nilfs_relax_pressure_in_lock(struct super_block *);

extern int nilfs_construct_segment(struct super_block *);
extern int nilfs_construct_dsync_segment(struct super_block *, struct inode *,
					 loff_t, loff_t);
extern void nilfs_flush_segment(struct super_block *, ino_t);
extern int nilfs_clean_segments(struct super_block *, struct nilfs_argv *,
				void **);

extern int nilfs_attach_segment_constructor(struct nilfs_sb_info *);
extern void nilfs_detach_segment_constructor(struct nilfs_sb_info *);

/* recovery.c */
extern int nilfs_read_super_root_block(struct super_block *, sector_t,
				       struct buffer_head **, int);
extern int nilfs_search_super_root(struct the_nilfs *, struct nilfs_sb_info *,
				   struct nilfs_recovery_info *);
extern int nilfs_recover_logical_segments(struct the_nilfs *,
					  struct nilfs_sb_info *,
					  struct nilfs_recovery_info *);
extern void nilfs_dispose_segment_list(struct list_head *);

#endif /* _NILFS_SEGMENT_H */

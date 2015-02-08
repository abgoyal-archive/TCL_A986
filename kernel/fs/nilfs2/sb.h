

#ifndef _NILFS_SB
#define _NILFS_SB

#include <linux/types.h>
#include <linux/fs.h>

struct nilfs_mount_options {
	unsigned long mount_opt;
	__u64 snapshot_cno;
};

struct the_nilfs;
struct nilfs_sc_info;

struct nilfs_sb_info {
	/* Snapshot status */
	__u64 s_snapshot_cno;		/* Checkpoint number */
	atomic_t s_inodes_count;
	atomic_t s_blocks_count;	/* Reserved (might be deleted) */

	/* Mount options */
	unsigned long s_mount_opt;
	uid_t s_resuid;
	gid_t s_resgid;

	unsigned long s_interval;	/* construction interval */
	unsigned long s_watermark;	/* threshold of data amount
					   for the segment construction */

	/* Fundamental members */
	struct super_block *s_super;	/* reverse pointer to super_block */
	struct the_nilfs *s_nilfs;
	struct list_head s_list;	/* list head for nilfs->ns_supers */
	atomic_t s_count;		/* reference count */

	/* Segment constructor */
	struct list_head s_dirty_files;	/* dirty files list */
	struct nilfs_sc_info *s_sc_info; /* segment constructor info */
	spinlock_t s_inode_lock;	/* Lock for the nilfs inode.
					   It covers s_dirty_files list */

	/* Metadata files */
	struct inode *s_ifile;		/* index file inode */

	/* Inode allocator */
	spinlock_t s_next_gen_lock;
	u32 s_next_generation;
};

static inline struct nilfs_sb_info *NILFS_SB(struct super_block *sb)
{
	return sb->s_fs_info;
}

static inline struct nilfs_sc_info *NILFS_SC(struct nilfs_sb_info *sbi)
{
	return sbi->s_sc_info;
}

#define nilfs_clear_opt(sbi, opt)  \
	do { (sbi)->s_mount_opt &= ~NILFS_MOUNT_##opt; } while (0)
#define nilfs_set_opt(sbi, opt)  \
	do { (sbi)->s_mount_opt |= NILFS_MOUNT_##opt; } while (0)
#define nilfs_test_opt(sbi, opt)   ((sbi)->s_mount_opt & NILFS_MOUNT_##opt)
#define nilfs_write_opt(sbi, mask, opt)					\
	do { (sbi)->s_mount_opt =					\
		(((sbi)->s_mount_opt & ~NILFS_MOUNT_##mask) |		\
		 NILFS_MOUNT_##opt);					\
	} while (0)

#endif /* _NILFS_SB */

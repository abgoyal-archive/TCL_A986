

#ifndef _LINUX_EXT3_JBD_H
#define _LINUX_EXT3_JBD_H

#include <linux/fs.h>
#include <linux/jbd.h>
#include <linux/ext3_fs.h>

#define EXT3_JOURNAL(inode)	(EXT3_SB((inode)->i_sb)->s_journal)


#define EXT3_SINGLEDATA_TRANS_BLOCKS	8U


#define EXT3_XATTR_TRANS_BLOCKS		6U


#define EXT3_DATA_TRANS_BLOCKS(sb)	(EXT3_SINGLEDATA_TRANS_BLOCKS + \
					 EXT3_XATTR_TRANS_BLOCKS - 2 + \
					 EXT3_MAXQUOTAS_TRANS_BLOCKS(sb))


#define EXT3_DELETE_TRANS_BLOCKS(sb)   (EXT3_MAXQUOTAS_TRANS_BLOCKS(sb) + 64)


#define EXT3_MAX_TRANS_DATA		64U


#define EXT3_RESERVE_TRANS_BLOCKS	12U

#define EXT3_INDEX_EXTRA_TRANS_BLOCKS	8

#ifdef CONFIG_QUOTA
#define EXT3_QUOTA_TRANS_BLOCKS(sb) (test_opt(sb, QUOTA) ? 2 : 0)
#define EXT3_QUOTA_INIT_BLOCKS(sb) (test_opt(sb, QUOTA) ? (DQUOT_INIT_ALLOC*\
		(EXT3_SINGLEDATA_TRANS_BLOCKS-3)+3+DQUOT_INIT_REWRITE) : 0)
#define EXT3_QUOTA_DEL_BLOCKS(sb) (test_opt(sb, QUOTA) ? (DQUOT_DEL_ALLOC*\
		(EXT3_SINGLEDATA_TRANS_BLOCKS-3)+3+DQUOT_DEL_REWRITE) : 0)
#else
#define EXT3_QUOTA_TRANS_BLOCKS(sb) 0
#define EXT3_QUOTA_INIT_BLOCKS(sb) 0
#define EXT3_QUOTA_DEL_BLOCKS(sb) 0
#endif
#define EXT3_MAXQUOTAS_TRANS_BLOCKS(sb) (MAXQUOTAS*EXT3_QUOTA_TRANS_BLOCKS(sb))
#define EXT3_MAXQUOTAS_INIT_BLOCKS(sb) (MAXQUOTAS*EXT3_QUOTA_INIT_BLOCKS(sb))
#define EXT3_MAXQUOTAS_DEL_BLOCKS(sb) (MAXQUOTAS*EXT3_QUOTA_DEL_BLOCKS(sb))

int
ext3_mark_iloc_dirty(handle_t *handle,
		     struct inode *inode,
		     struct ext3_iloc *iloc);


int ext3_reserve_inode_write(handle_t *handle, struct inode *inode,
			struct ext3_iloc *iloc);

int ext3_mark_inode_dirty(handle_t *handle, struct inode *inode);


static inline void ext3_journal_release_buffer(handle_t *handle,
						struct buffer_head *bh)
{
	journal_release_buffer(handle, bh);
}

void ext3_journal_abort_handle(const char *caller, const char *err_fn,
		struct buffer_head *bh, handle_t *handle, int err);

int __ext3_journal_get_undo_access(const char *where, handle_t *handle,
				struct buffer_head *bh);

int __ext3_journal_get_write_access(const char *where, handle_t *handle,
				struct buffer_head *bh);

int __ext3_journal_forget(const char *where, handle_t *handle,
				struct buffer_head *bh);

int __ext3_journal_revoke(const char *where, handle_t *handle,
				unsigned long blocknr, struct buffer_head *bh);

int __ext3_journal_get_create_access(const char *where,
				handle_t *handle, struct buffer_head *bh);

int __ext3_journal_dirty_metadata(const char *where,
				handle_t *handle, struct buffer_head *bh);

#define ext3_journal_get_undo_access(handle, bh) \
	__ext3_journal_get_undo_access(__func__, (handle), (bh))
#define ext3_journal_get_write_access(handle, bh) \
	__ext3_journal_get_write_access(__func__, (handle), (bh))
#define ext3_journal_revoke(handle, blocknr, bh) \
	__ext3_journal_revoke(__func__, (handle), (blocknr), (bh))
#define ext3_journal_get_create_access(handle, bh) \
	__ext3_journal_get_create_access(__func__, (handle), (bh))
#define ext3_journal_dirty_metadata(handle, bh) \
	__ext3_journal_dirty_metadata(__func__, (handle), (bh))
#define ext3_journal_forget(handle, bh) \
	__ext3_journal_forget(__func__, (handle), (bh))

int ext3_journal_dirty_data(handle_t *handle, struct buffer_head *bh);

handle_t *ext3_journal_start_sb(struct super_block *sb, int nblocks);
int __ext3_journal_stop(const char *where, handle_t *handle);

static inline handle_t *ext3_journal_start(struct inode *inode, int nblocks)
{
	return ext3_journal_start_sb(inode->i_sb, nblocks);
}

#define ext3_journal_stop(handle) \
	__ext3_journal_stop(__func__, (handle))

static inline handle_t *ext3_journal_current_handle(void)
{
	return journal_current_handle();
}

static inline int ext3_journal_extend(handle_t *handle, int nblocks)
{
	return journal_extend(handle, nblocks);
}

static inline int ext3_journal_restart(handle_t *handle, int nblocks)
{
	return journal_restart(handle, nblocks);
}

static inline int ext3_journal_blocks_per_page(struct inode *inode)
{
	return journal_blocks_per_page(inode);
}

static inline int ext3_journal_force_commit(journal_t *journal)
{
	return journal_force_commit(journal);
}

/* super.c */
int ext3_force_commit(struct super_block *sb);

static inline int ext3_should_journal_data(struct inode *inode)
{
	if (!S_ISREG(inode->i_mode))
		return 1;
	if (test_opt(inode->i_sb, DATA_FLAGS) == EXT3_MOUNT_JOURNAL_DATA)
		return 1;
	if (EXT3_I(inode)->i_flags & EXT3_JOURNAL_DATA_FL)
		return 1;
	return 0;
}

static inline int ext3_should_order_data(struct inode *inode)
{
	if (!S_ISREG(inode->i_mode))
		return 0;
	if (EXT3_I(inode)->i_flags & EXT3_JOURNAL_DATA_FL)
		return 0;
	if (test_opt(inode->i_sb, DATA_FLAGS) == EXT3_MOUNT_ORDERED_DATA)
		return 1;
	return 0;
}

static inline int ext3_should_writeback_data(struct inode *inode)
{
	if (!S_ISREG(inode->i_mode))
		return 0;
	if (EXT3_I(inode)->i_flags & EXT3_JOURNAL_DATA_FL)
		return 0;
	if (test_opt(inode->i_sb, DATA_FLAGS) == EXT3_MOUNT_WRITEBACK_DATA)
		return 1;
	return 0;
}

#endif	/* _LINUX_EXT3_JBD_H */

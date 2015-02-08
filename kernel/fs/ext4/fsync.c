

#include <linux/time.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/jbd2.h>
#include <linux/blkdev.h>

#include "ext4.h"
#include "ext4_jbd2.h"

#include <trace/events/ext4.h>

static void ext4_sync_parent(struct inode *inode)
{
	struct dentry *dentry = NULL;

	while (inode && ext4_test_inode_state(inode, EXT4_STATE_NEWENTRY)) {
		ext4_clear_inode_state(inode, EXT4_STATE_NEWENTRY);
		dentry = list_entry(inode->i_dentry.next,
				    struct dentry, d_alias);
		if (!dentry || !dentry->d_parent || !dentry->d_parent->d_inode)
			break;
		inode = dentry->d_parent->d_inode;
		sync_mapping_buffers(inode->i_mapping);
	}
}


int ext4_sync_file(struct file *file, int datasync)
{
	struct inode *inode = file->f_mapping->host;
	struct ext4_inode_info *ei = EXT4_I(inode);
	journal_t *journal = EXT4_SB(inode->i_sb)->s_journal;
	int ret;
	tid_t commit_tid;

	J_ASSERT(ext4_journal_current_handle() == NULL);

	trace_ext4_sync_file(file, datasync);

	if (inode->i_sb->s_flags & MS_RDONLY)
		return 0;

	ret = flush_completed_IO(inode);
	if (ret < 0)
		return ret;

	if (!journal) {
		ret = generic_file_fsync(file, datasync);
		if (!ret && !list_empty(&inode->i_dentry))
			ext4_sync_parent(inode);
		return ret;
	}

	/*
	 * data=writeback,ordered:
	 *  The caller's filemap_fdatawrite()/wait will sync the data.
	 *  Metadata is in the journal, we wait for proper transaction to
	 *  commit here.
	 *
	 * data=journal:
	 *  filemap_fdatawrite won't do anything (the buffers are clean).
	 *  ext4_force_commit will write the file data into the journal and
	 *  will wait on that.
	 *  filemap_fdatawait() will encounter a ton of newly-dirtied pages
	 *  (they were dirtied by commit).  But that's OK - the blocks are
	 *  safe in-journal, which is all fsync() needs to ensure.
	 */
	if (ext4_should_journal_data(inode))
		return ext4_force_commit(inode->i_sb);

	commit_tid = datasync ? ei->i_datasync_tid : ei->i_sync_tid;
	if (jbd2_log_start_commit(journal, commit_tid)) {
		/*
		 * When the journal is on a different device than the
		 * fs data disk, we need to issue the barrier in
		 * writeback mode.  (In ordered mode, the jbd2 layer
		 * will take care of issuing the barrier.  In
		 * data=journal, all of the data blocks are written to
		 * the journal device.)
		 */
		if (ext4_should_writeback_data(inode) &&
		    (journal->j_fs_dev != journal->j_dev) &&
		    (journal->j_flags & JBD2_BARRIER))
			blkdev_issue_flush(inode->i_sb->s_bdev, GFP_KERNEL,
					NULL, BLKDEV_IFL_WAIT);
		ret = jbd2_log_wait_commit(journal, commit_tid);
	} else if (journal->j_flags & JBD2_BARRIER)
		blkdev_issue_flush(inode->i_sb->s_bdev, GFP_KERNEL, NULL,
			BLKDEV_IFL_WAIT);
	return ret;
}

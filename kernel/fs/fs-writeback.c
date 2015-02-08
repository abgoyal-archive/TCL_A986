

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/writeback.h>
#include <linux/blkdev.h>
#include <linux/backing-dev.h>
#include <linux/buffer_head.h>
#include "internal.h"

#define inode_to_bdi(inode)	((inode)->i_mapping->backing_dev_info)

int nr_pdflush_threads;

struct wb_writeback_work {
	long nr_pages;
	struct super_block *sb;
	enum writeback_sync_modes sync_mode;
	unsigned int for_kupdate:1;
	unsigned int range_cyclic:1;
	unsigned int for_background:1;

	struct list_head list;		/* pending work list */
	struct completion *done;	/* set if the caller waits */
};

int writeback_in_progress(struct backing_dev_info *bdi)
{
	return !list_empty(&bdi->work_list);
}

static void bdi_queue_work(struct backing_dev_info *bdi,
		struct wb_writeback_work *work)
{
	spin_lock(&bdi->wb_lock);
	list_add_tail(&work->list, &bdi->work_list);
	spin_unlock(&bdi->wb_lock);

	/*
	 * If the default thread isn't there, make sure we add it. When
	 * it gets created and wakes up, we'll run this work.
	 */
	if (unlikely(list_empty_careful(&bdi->wb_list)))
		wake_up_process(default_backing_dev_info.wb.task);
	else {
		struct bdi_writeback *wb = &bdi->wb;

		if (wb->task)
			wake_up_process(wb->task);
	}
}

static void
__bdi_start_writeback(struct backing_dev_info *bdi, long nr_pages,
		bool range_cyclic, bool for_background)
{
	struct wb_writeback_work *work;

	/*
	 * This is WB_SYNC_NONE writeback, so if allocation fails just
	 * wakeup the thread for old dirty data writeback
	 */
	work = kzalloc(sizeof(*work), GFP_ATOMIC);
	if (!work) {
		if (bdi->wb.task)
			wake_up_process(bdi->wb.task);
		return;
	}

	work->sync_mode	= WB_SYNC_NONE;
	work->nr_pages	= nr_pages;
	work->range_cyclic = range_cyclic;
	work->for_background = for_background;

	bdi_queue_work(bdi, work);
}

void bdi_start_writeback(struct backing_dev_info *bdi, long nr_pages)
{
	__bdi_start_writeback(bdi, nr_pages, true, false);
}

void bdi_start_background_writeback(struct backing_dev_info *bdi)
{
	__bdi_start_writeback(bdi, LONG_MAX, true, true);
}

static void redirty_tail(struct inode *inode)
{
	struct bdi_writeback *wb = &inode_to_bdi(inode)->wb;

	if (!list_empty(&wb->b_dirty)) {
		struct inode *tail;

		tail = list_entry(wb->b_dirty.next, struct inode, i_list);
		if (time_before(inode->dirtied_when, tail->dirtied_when))
			inode->dirtied_when = jiffies;
	}
	list_move(&inode->i_list, &wb->b_dirty);
}

static void requeue_io(struct inode *inode)
{
	struct bdi_writeback *wb = &inode_to_bdi(inode)->wb;

	list_move(&inode->i_list, &wb->b_more_io);
}

static void inode_sync_complete(struct inode *inode)
{
	/*
	 * Prevent speculative execution through spin_unlock(&inode_lock);
	 */
	smp_mb();
	wake_up_bit(&inode->i_state, __I_SYNC);
}

static bool inode_dirtied_after(struct inode *inode, unsigned long t)
{
	bool ret = time_after(inode->dirtied_when, t);
#ifndef CONFIG_64BIT
	/*
	 * For inodes being constantly redirtied, dirtied_when can get stuck.
	 * It _appears_ to be in the future, but is actually in distant past.
	 * This test is necessary to prevent such wrapped-around relative times
	 * from permanently stopping the whole bdi writeback.
	 */
	ret = ret && time_before_eq(inode->dirtied_when, jiffies);
#endif
	return ret;
}

static void move_expired_inodes(struct list_head *delaying_queue,
			       struct list_head *dispatch_queue,
				unsigned long *older_than_this)
{
	LIST_HEAD(tmp);
	struct list_head *pos, *node;
	struct super_block *sb = NULL;
	struct inode *inode;
	int do_sb_sort = 0;

	while (!list_empty(delaying_queue)) {
		inode = list_entry(delaying_queue->prev, struct inode, i_list);
		if (older_than_this &&
		    inode_dirtied_after(inode, *older_than_this))
			break;
		if (sb && sb != inode->i_sb)
			do_sb_sort = 1;
		sb = inode->i_sb;
		list_move(&inode->i_list, &tmp);
	}

	/* just one sb in list, splice to dispatch_queue and we're done */
	if (!do_sb_sort) {
		list_splice(&tmp, dispatch_queue);
		return;
	}

	/* Move inodes from one superblock together */
	while (!list_empty(&tmp)) {
		inode = list_entry(tmp.prev, struct inode, i_list);
		sb = inode->i_sb;
		list_for_each_prev_safe(pos, node, &tmp) {
			inode = list_entry(pos, struct inode, i_list);
			if (inode->i_sb == sb)
				list_move(&inode->i_list, dispatch_queue);
		}
	}
}

static void queue_io(struct bdi_writeback *wb, unsigned long *older_than_this)
{
	list_splice_init(&wb->b_more_io, wb->b_io.prev);
	move_expired_inodes(&wb->b_dirty, &wb->b_io, older_than_this);
}

static int write_inode(struct inode *inode, struct writeback_control *wbc)
{
	if (inode->i_sb->s_op->write_inode && !is_bad_inode(inode))
		return inode->i_sb->s_op->write_inode(inode, wbc);
	return 0;
}

static void inode_wait_for_writeback(struct inode *inode)
{
	DEFINE_WAIT_BIT(wq, &inode->i_state, __I_SYNC);
	wait_queue_head_t *wqh;

	wqh = bit_waitqueue(&inode->i_state, __I_SYNC);
	 while (inode->i_state & I_SYNC) {
		spin_unlock(&inode_lock);
		__wait_on_bit(wqh, &wq, inode_wait, TASK_UNINTERRUPTIBLE);
		spin_lock(&inode_lock);
	}
}

static int
writeback_single_inode(struct inode *inode, struct writeback_control *wbc)
{
	struct address_space *mapping = inode->i_mapping;
	unsigned dirty;
	int ret;

	if (!atomic_read(&inode->i_count))
		WARN_ON(!(inode->i_state & (I_WILL_FREE|I_FREEING)));
	else
		WARN_ON(inode->i_state & I_WILL_FREE);

	if (inode->i_state & I_SYNC) {
		/*
		 * If this inode is locked for writeback and we are not doing
		 * writeback-for-data-integrity, move it to b_more_io so that
		 * writeback can proceed with the other inodes on s_io.
		 *
		 * We'll have another go at writing back this inode when we
		 * completed a full scan of b_io.
		 */
		if (wbc->sync_mode != WB_SYNC_ALL) {
			requeue_io(inode);
			return 0;
		}

		/*
		 * It's a data-integrity sync.  We must wait.
		 */
		inode_wait_for_writeback(inode);
	}

	BUG_ON(inode->i_state & I_SYNC);

	/* Set I_SYNC, reset I_DIRTY_PAGES */
	inode->i_state |= I_SYNC;
	inode->i_state &= ~I_DIRTY_PAGES;
	spin_unlock(&inode_lock);

	ret = do_writepages(mapping, wbc);

	/*
	 * Make sure to wait on the data before writing out the metadata.
	 * This is important for filesystems that modify metadata on data
	 * I/O completion.
	 */
	if (wbc->sync_mode == WB_SYNC_ALL) {
		int err = filemap_fdatawait(mapping);
		if (ret == 0)
			ret = err;
	}

	/*
	 * Some filesystems may redirty the inode during the writeback
	 * due to delalloc, clear dirty metadata flags right before
	 * write_inode()
	 */
	spin_lock(&inode_lock);
	dirty = inode->i_state & I_DIRTY;
	inode->i_state &= ~(I_DIRTY_SYNC | I_DIRTY_DATASYNC);
	spin_unlock(&inode_lock);
	/* Don't write the inode if only I_DIRTY_PAGES was set */
	if (dirty & (I_DIRTY_SYNC | I_DIRTY_DATASYNC)) {
		int err = write_inode(inode, wbc);
		if (ret == 0)
			ret = err;
	}

	spin_lock(&inode_lock);
	inode->i_state &= ~I_SYNC;
	if (!(inode->i_state & (I_FREEING | I_CLEAR))) {
		if ((inode->i_state & I_DIRTY_PAGES) && wbc->for_kupdate) {
			/*
			 * More pages get dirtied by a fast dirtier.
			 */
			goto select_queue;
		} else if (inode->i_state & I_DIRTY) {
			/*
			 * At least XFS will redirty the inode during the
			 * writeback (delalloc) and on io completion (isize).
			 */
			redirty_tail(inode);
		} else if (mapping_tagged(mapping, PAGECACHE_TAG_DIRTY)) {
			/*
			 * We didn't write back all the pages.  nfs_writepages()
			 * sometimes bales out without doing anything. Redirty
			 * the inode; Move it from b_io onto b_more_io/b_dirty.
			 */
			/*
			 * akpm: if the caller was the kupdate function we put
			 * this inode at the head of b_dirty so it gets first
			 * consideration.  Otherwise, move it to the tail, for
			 * the reasons described there.  I'm not really sure
			 * how much sense this makes.  Presumably I had a good
			 * reasons for doing it this way, and I'd rather not
			 * muck with it at present.
			 */
			if (wbc->for_kupdate) {
				/*
				 * For the kupdate function we move the inode
				 * to b_more_io so it will get more writeout as
				 * soon as the queue becomes uncongested.
				 */
				inode->i_state |= I_DIRTY_PAGES;
select_queue:
				if (wbc->nr_to_write <= 0) {
					/*
					 * slice used up: queue for next turn
					 */
					requeue_io(inode);
				} else {
					/*
					 * somehow blocked: retry later
					 */
					redirty_tail(inode);
				}
			} else {
				/*
				 * Otherwise fully redirty the inode so that
				 * other inodes on this superblock will get some
				 * writeout.  Otherwise heavy writing to one
				 * file would indefinitely suspend writeout of
				 * all the other files.
				 */
				inode->i_state |= I_DIRTY_PAGES;
				redirty_tail(inode);
			}
		} else if (atomic_read(&inode->i_count)) {
			/*
			 * The inode is clean, inuse
			 */
			list_move(&inode->i_list, &inode_in_use);
		} else {
			/*
			 * The inode is clean, unused
			 */
			list_move(&inode->i_list, &inode_unused);
		}
	}
	inode_sync_complete(inode);
	return ret;
}

static bool pin_sb_for_writeback(struct super_block *sb)
{
	spin_lock(&sb_lock);
	if (list_empty(&sb->s_instances)) {
		spin_unlock(&sb_lock);
		return false;
	}

	sb->s_count++;
	spin_unlock(&sb_lock);

	if (down_read_trylock(&sb->s_umount)) {
		if (sb->s_root)
			return true;
		up_read(&sb->s_umount);
	}

	put_super(sb);
	return false;
}

static int writeback_sb_inodes(struct super_block *sb, struct bdi_writeback *wb,
		struct writeback_control *wbc, bool only_this_sb)
{
	while (!list_empty(&wb->b_io)) {
		long pages_skipped;
		struct inode *inode = list_entry(wb->b_io.prev,
						 struct inode, i_list);

		if (inode->i_sb != sb) {
			if (only_this_sb) {
				/*
				 * We only want to write back data for this
				 * superblock, move all inodes not belonging
				 * to it back onto the dirty list.
				 */
				redirty_tail(inode);
				continue;
			}

			/*
			 * The inode belongs to a different superblock.
			 * Bounce back to the caller to unpin this and
			 * pin the next superblock.
			 */
			return 0;
		}

		if (inode->i_state & (I_NEW | I_WILL_FREE)) {
			requeue_io(inode);
			continue;
		}
		/*
		 * Was this inode dirtied after sync_sb_inodes was called?
		 * This keeps sync from extra jobs and livelock.
		 */
		if (inode_dirtied_after(inode, wbc->wb_start))
			return 1;

		BUG_ON(inode->i_state & (I_FREEING | I_CLEAR));
		__iget(inode);
		pages_skipped = wbc->pages_skipped;
		writeback_single_inode(inode, wbc);
		if (wbc->pages_skipped != pages_skipped) {
			/*
			 * writeback is not making progress due to locked
			 * buffers.  Skip this inode for now.
			 */
			redirty_tail(inode);
		}
		spin_unlock(&inode_lock);
		iput(inode);
		cond_resched();
		spin_lock(&inode_lock);
		if (wbc->nr_to_write <= 0) {
			wbc->more_io = 1;
			return 1;
		}
		if (!list_empty(&wb->b_more_io))
			wbc->more_io = 1;
	}
	/* b_io is empty */
	return 1;
}

void writeback_inodes_wb(struct bdi_writeback *wb,
		struct writeback_control *wbc)
{
	int ret = 0;

	wbc->wb_start = jiffies; /* livelock avoidance */
	spin_lock(&inode_lock);
	if (!wbc->for_kupdate || list_empty(&wb->b_io))
		queue_io(wb, wbc->older_than_this);

	while (!list_empty(&wb->b_io)) {
		struct inode *inode = list_entry(wb->b_io.prev,
						 struct inode, i_list);
		struct super_block *sb = inode->i_sb;

		if (!pin_sb_for_writeback(sb)) {
			requeue_io(inode);
			continue;
		}
		ret = writeback_sb_inodes(sb, wb, wbc, false);
		drop_super(sb);

		if (ret)
			break;
	}
	spin_unlock(&inode_lock);
	/* Leave any unwritten inodes on b_io */
}

static void __writeback_inodes_sb(struct super_block *sb,
		struct bdi_writeback *wb, struct writeback_control *wbc)
{
	WARN_ON(!rwsem_is_locked(&sb->s_umount));

	wbc->wb_start = jiffies; /* livelock avoidance */
	spin_lock(&inode_lock);
	if (!wbc->for_kupdate || list_empty(&wb->b_io))
		queue_io(wb, wbc->older_than_this);
	writeback_sb_inodes(sb, wb, wbc, true);
	spin_unlock(&inode_lock);
}

#define MAX_WRITEBACK_PAGES     1024

static inline bool over_bground_thresh(void)
{
	unsigned long background_thresh, dirty_thresh;

	get_dirty_limits(&background_thresh, &dirty_thresh, NULL, NULL);

	return (global_page_state(NR_FILE_DIRTY) +
		global_page_state(NR_UNSTABLE_NFS) >= background_thresh);
}

static long wb_writeback(struct bdi_writeback *wb,
			 struct wb_writeback_work *work)
{
	struct writeback_control wbc = {
		.sync_mode		= work->sync_mode,
		.older_than_this	= NULL,
		.for_kupdate		= work->for_kupdate,
		.for_background		= work->for_background,
		.range_cyclic		= work->range_cyclic,
	};
	unsigned long oldest_jif;
	long wrote = 0;
	struct inode *inode;

	if (wbc.for_kupdate) {
		wbc.older_than_this = &oldest_jif;
		oldest_jif = jiffies -
				msecs_to_jiffies(dirty_expire_interval * 10);
	}
	if (!wbc.range_cyclic) {
		wbc.range_start = 0;
		wbc.range_end = LLONG_MAX;
	}

	for (;;) {
		/*
		 * Stop writeback when nr_pages has been consumed
		 */
		if (work->nr_pages <= 0)
			break;

		/*
		 * For background writeout, stop when we are below the
		 * background dirty threshold
		 */
		if (work->for_background && !over_bground_thresh())
			break;

		wbc.more_io = 0;
		wbc.nr_to_write = MAX_WRITEBACK_PAGES;
		wbc.pages_skipped = 0;
		if (work->sb)
			__writeback_inodes_sb(work->sb, wb, &wbc);
		else
			writeback_inodes_wb(wb, &wbc);
		work->nr_pages -= MAX_WRITEBACK_PAGES - wbc.nr_to_write;
		wrote += MAX_WRITEBACK_PAGES - wbc.nr_to_write;

		/*
		 * If we consumed everything, see if we have more
		 */
		if (wbc.nr_to_write <= 0)
			continue;
		/*
		 * Didn't write everything and we don't have more IO, bail
		 */
		if (!wbc.more_io)
			break;
		/*
		 * Did we write something? Try for more
		 */
		if (wbc.nr_to_write < MAX_WRITEBACK_PAGES)
			continue;
		/*
		 * Nothing written. Wait for some inode to
		 * become available for writeback. Otherwise
		 * we'll just busyloop.
		 */
		spin_lock(&inode_lock);
		if (!list_empty(&wb->b_more_io))  {
			inode = list_entry(wb->b_more_io.prev,
						struct inode, i_list);
			inode_wait_for_writeback(inode);
		}
		spin_unlock(&inode_lock);
	}

	return wrote;
}

static struct wb_writeback_work *
get_next_work_item(struct backing_dev_info *bdi, struct bdi_writeback *wb)
{
	struct wb_writeback_work *work = NULL;

	spin_lock(&bdi->wb_lock);
	if (!list_empty(&bdi->work_list)) {
		work = list_entry(bdi->work_list.next,
				  struct wb_writeback_work, list);
		list_del_init(&work->list);
	}
	spin_unlock(&bdi->wb_lock);
	return work;
}

static long wb_check_old_data_flush(struct bdi_writeback *wb)
{
	unsigned long expired;
	long nr_pages;

	/*
	 * When set to zero, disable periodic writeback
	 */
	if (!dirty_writeback_interval)
		return 0;

	expired = wb->last_old_flush +
			msecs_to_jiffies(dirty_writeback_interval * 10);
	if (time_before(jiffies, expired))
		return 0;

	wb->last_old_flush = jiffies;
	nr_pages = global_page_state(NR_FILE_DIRTY) +
			global_page_state(NR_UNSTABLE_NFS) +
			(inodes_stat.nr_inodes - inodes_stat.nr_unused);

	if (nr_pages) {
		struct wb_writeback_work work = {
			.nr_pages	= nr_pages,
			.sync_mode	= WB_SYNC_NONE,
			.for_kupdate	= 1,
			.range_cyclic	= 1,
		};

		return wb_writeback(wb, &work);
	}

	return 0;
}

long wb_do_writeback(struct bdi_writeback *wb, int force_wait)
{
	struct backing_dev_info *bdi = wb->bdi;
	struct wb_writeback_work *work;
	long wrote = 0;

	while ((work = get_next_work_item(bdi, wb)) != NULL) {
		/*
		 * Override sync mode, in case we must wait for completion
		 * because this thread is exiting now.
		 */
		if (force_wait)
			work->sync_mode = WB_SYNC_ALL;

		wrote += wb_writeback(wb, work);

		/*
		 * Notify the caller of completion if this is a synchronous
		 * work item, otherwise just free it.
		 */
		if (work->done)
			complete(work->done);
		else
			kfree(work);
	}

	/*
	 * Check for periodic writeback, kupdated() style
	 */
	wrote += wb_check_old_data_flush(wb);

	return wrote;
}

int bdi_writeback_task(struct bdi_writeback *wb)
{
	unsigned long last_active = jiffies;
	unsigned long wait_jiffies = -1UL;
	long pages_written;

	while (!kthread_should_stop()) {
		pages_written = wb_do_writeback(wb, 0);

		if (pages_written)
			last_active = jiffies;
		else if (wait_jiffies != -1UL) {
			unsigned long max_idle;

			/*
			 * Longest period of inactivity that we tolerate. If we
			 * see dirty data again later, the task will get
			 * recreated automatically.
			 */
			max_idle = max(5UL * 60 * HZ, wait_jiffies);
			if (time_after(jiffies, max_idle + last_active))
				break;
		}

		if (dirty_writeback_interval) {
			wait_jiffies = msecs_to_jiffies(dirty_writeback_interval * 10);
			schedule_timeout_interruptible(wait_jiffies);
		} else {
			set_current_state(TASK_INTERRUPTIBLE);
			if (list_empty_careful(&wb->bdi->work_list) &&
			    !kthread_should_stop())
				schedule();
			__set_current_state(TASK_RUNNING);
		}

		try_to_freeze();
	}

	return 0;
}

void wakeup_flusher_threads(long nr_pages)
{
	struct backing_dev_info *bdi;

	if (!nr_pages) {
		nr_pages = global_page_state(NR_FILE_DIRTY) +
				global_page_state(NR_UNSTABLE_NFS);
	}

	rcu_read_lock();
	list_for_each_entry_rcu(bdi, &bdi_list, bdi_list) {
		if (!bdi_has_dirty_io(bdi))
			continue;
		__bdi_start_writeback(bdi, nr_pages, false, false);
	}
	rcu_read_unlock();
}

static noinline void block_dump___mark_inode_dirty(struct inode *inode)
{
	if (inode->i_ino || strcmp(inode->i_sb->s_id, "bdev")) {
		struct dentry *dentry;
		const char *name = "?";

		dentry = d_find_alias(inode);
		if (dentry) {
			spin_lock(&dentry->d_lock);
			name = (const char *) dentry->d_name.name;
		}
		printk(KERN_DEBUG
		       "%s(%d): dirtied inode %lu (%s) on %s\n",
		       current->comm, task_pid_nr(current), inode->i_ino,
		       name, inode->i_sb->s_id);
		if (dentry) {
			spin_unlock(&dentry->d_lock);
			dput(dentry);
		}
	}
}

void __mark_inode_dirty(struct inode *inode, int flags)
{
	struct super_block *sb = inode->i_sb;

	/*
	 * Don't do this for I_DIRTY_PAGES - that doesn't actually
	 * dirty the inode itself
	 */
	if (flags & (I_DIRTY_SYNC | I_DIRTY_DATASYNC)) {
		if (sb->s_op->dirty_inode)
			sb->s_op->dirty_inode(inode);
	}

	/*
	 * make sure that changes are seen by all cpus before we test i_state
	 * -- mikulas
	 */
	smp_mb();

	/* avoid the locking if we can */
	if ((inode->i_state & flags) == flags)
		return;

	if (unlikely(block_dump > 1))
		block_dump___mark_inode_dirty(inode);

	spin_lock(&inode_lock);
	if ((inode->i_state & flags) != flags) {
		const int was_dirty = inode->i_state & I_DIRTY;

		inode->i_state |= flags;

		/*
		 * If the inode is being synced, just update its dirty state.
		 * The unlocker will place the inode on the appropriate
		 * superblock list, based upon its state.
		 */
		if (inode->i_state & I_SYNC)
			goto out;

		/*
		 * Only add valid (hashed) inodes to the superblock's
		 * dirty list.  Add blockdev inodes as well.
		 */
		if (!S_ISBLK(inode->i_mode)) {
			if (hlist_unhashed(&inode->i_hash))
				goto out;
		}
		if (inode->i_state & (I_FREEING|I_CLEAR))
			goto out;

		/*
		 * If the inode was already on b_dirty/b_io/b_more_io, don't
		 * reposition it (that would break b_dirty time-ordering).
		 */
		if (!was_dirty) {
			struct bdi_writeback *wb = &inode_to_bdi(inode)->wb;
			struct backing_dev_info *bdi = wb->bdi;

			if (bdi_cap_writeback_dirty(bdi) &&
			    !test_bit(BDI_registered, &bdi->state)) {
				WARN_ON(1);
				printk(KERN_ERR "bdi-%s not registered\n",
								bdi->name);
			}

			inode->dirtied_when = jiffies;
			list_move(&inode->i_list, &wb->b_dirty);
		}
	}
out:
	spin_unlock(&inode_lock);
}
EXPORT_SYMBOL(__mark_inode_dirty);

static void wait_sb_inodes(struct super_block *sb)
{
	struct inode *inode, *old_inode = NULL;

	/*
	 * We need to be protected against the filesystem going from
	 * r/o to r/w or vice versa.
	 */
	WARN_ON(!rwsem_is_locked(&sb->s_umount));

	spin_lock(&inode_lock);

	/*
	 * Data integrity sync. Must wait for all pages under writeback,
	 * because there may have been pages dirtied before our sync
	 * call, but which had writeout started before we write it out.
	 * In which case, the inode may not be on the dirty list, but
	 * we still have to wait for that writeout.
	 */
	list_for_each_entry(inode, &sb->s_inodes, i_sb_list) {
		struct address_space *mapping;

		if (inode->i_state & (I_FREEING|I_CLEAR|I_WILL_FREE|I_NEW))
			continue;
		mapping = inode->i_mapping;
		if (mapping->nrpages == 0)
			continue;
		__iget(inode);
		spin_unlock(&inode_lock);
		/*
		 * We hold a reference to 'inode' so it couldn't have
		 * been removed from s_inodes list while we dropped the
		 * inode_lock.  We cannot iput the inode now as we can
		 * be holding the last reference and we cannot iput it
		 * under inode_lock. So we keep the reference and iput
		 * it later.
		 */
		iput(old_inode);
		old_inode = inode;

		filemap_fdatawait(mapping);

		cond_resched();

		spin_lock(&inode_lock);
	}
	spin_unlock(&inode_lock);
	iput(old_inode);
}

void writeback_inodes_sb(struct super_block *sb)
{
	unsigned long nr_dirty = global_page_state(NR_FILE_DIRTY);
	unsigned long nr_unstable = global_page_state(NR_UNSTABLE_NFS);
	DECLARE_COMPLETION_ONSTACK(done);
	struct wb_writeback_work work = {
		.sb		= sb,
		.sync_mode	= WB_SYNC_NONE,
		.done		= &done,
	};

	WARN_ON(!rwsem_is_locked(&sb->s_umount));

	work.nr_pages = nr_dirty + nr_unstable +
			(inodes_stat.nr_inodes - inodes_stat.nr_unused);

	bdi_queue_work(sb->s_bdi, &work);
	wait_for_completion(&done);
}
EXPORT_SYMBOL(writeback_inodes_sb);

int writeback_inodes_sb_if_idle(struct super_block *sb)
{
	if (!writeback_in_progress(sb->s_bdi)) {
		down_read(&sb->s_umount);
		writeback_inodes_sb(sb);
		up_read(&sb->s_umount);
		return 1;
	} else
		return 0;
}
EXPORT_SYMBOL(writeback_inodes_sb_if_idle);

void sync_inodes_sb(struct super_block *sb)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct wb_writeback_work work = {
		.sb		= sb,
		.sync_mode	= WB_SYNC_ALL,
		.nr_pages	= LONG_MAX,
		.range_cyclic	= 0,
		.done		= &done,
	};

	WARN_ON(!rwsem_is_locked(&sb->s_umount));

	bdi_queue_work(sb->s_bdi, &work);
	wait_for_completion(&done);

	wait_sb_inodes(sb);
}
EXPORT_SYMBOL(sync_inodes_sb);

int write_inode_now(struct inode *inode, int sync)
{
	int ret;
	struct writeback_control wbc = {
		.nr_to_write = LONG_MAX,
		.sync_mode = sync ? WB_SYNC_ALL : WB_SYNC_NONE,
		.range_start = 0,
		.range_end = LLONG_MAX,
	};

	if (!mapping_cap_writeback_dirty(inode->i_mapping))
		wbc.nr_to_write = 0;

	might_sleep();
	spin_lock(&inode_lock);
	ret = writeback_single_inode(inode, &wbc);
	spin_unlock(&inode_lock);
	if (sync)
		inode_sync_wait(inode);
	return ret;
}
EXPORT_SYMBOL(write_inode_now);

int sync_inode(struct inode *inode, struct writeback_control *wbc)
{
	int ret;

	spin_lock(&inode_lock);
	ret = writeback_single_inode(inode, wbc);
	spin_unlock(&inode_lock);
	return ret;
}
EXPORT_SYMBOL(sync_inode);



#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/writeback.h>
#include <linux/init.h>
#include <linux/backing-dev.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/blkdev.h>
#include <linux/mpage.h>
#include <linux/rmap.h>
#include <linux/percpu.h>
#include <linux/notifier.h>
#include <linux/smp.h>
#include <linux/sysctl.h>
#include <linux/cpu.h>
#include <linux/syscalls.h>
#include <linux/buffer_head.h>
#include <linux/pagevec.h>

static long ratelimit_pages = 32;

static inline long sync_writeback_pages(unsigned long dirtied)
{
	if (dirtied < ratelimit_pages)
		dirtied = ratelimit_pages;

	return dirtied + dirtied / 2;
}

/* The following parameters are exported via /proc/sys/vm */

int dirty_background_ratio = 10;

unsigned long dirty_background_bytes;

int vm_highmem_is_dirtyable;

int vm_dirty_ratio = 20;

unsigned long vm_dirty_bytes;

unsigned int dirty_writeback_interval = 5 * 100; /* centiseconds */
EXPORT_SYMBOL(dirty_writeback_interval);

unsigned int dirty_expire_interval = 30 * 100; /* centiseconds */
EXPORT_SYMBOL(dirty_expire_interval);

int block_dump;

int laptop_mode;

EXPORT_SYMBOL(laptop_mode);

/* End of sysctl-exported parameters */


static struct prop_descriptor vm_completions;
static struct prop_descriptor vm_dirties;

static int calc_period_shift(void)
{
	unsigned long dirty_total;

	if (vm_dirty_bytes)
		dirty_total = vm_dirty_bytes / PAGE_SIZE;
	else
		dirty_total = (vm_dirty_ratio * determine_dirtyable_memory()) /
				100;
	return 2 + ilog2(dirty_total - 1);
}

static void update_completion_period(void)
{
	int shift = calc_period_shift();
	prop_change_shift(&vm_completions, shift);
	prop_change_shift(&vm_dirties, shift);
}

int dirty_background_ratio_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret;

	ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (ret == 0 && write)
		dirty_background_bytes = 0;
	return ret;
}

int dirty_background_bytes_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int ret;

	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);
	if (ret == 0 && write)
		dirty_background_ratio = 0;
	return ret;
}

int dirty_ratio_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	int old_ratio = vm_dirty_ratio;
	int ret;

	ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	if (ret == 0 && write && vm_dirty_ratio != old_ratio) {
		update_completion_period();
		vm_dirty_bytes = 0;
	}
	return ret;
}


int dirty_bytes_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos)
{
	unsigned long old_bytes = vm_dirty_bytes;
	int ret;

	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);
	if (ret == 0 && write && vm_dirty_bytes != old_bytes) {
		update_completion_period();
		vm_dirty_ratio = 0;
	}
	return ret;
}

static inline void __bdi_writeout_inc(struct backing_dev_info *bdi)
{
	__prop_inc_percpu_max(&vm_completions, &bdi->completions,
			      bdi->max_prop_frac);
}

void bdi_writeout_inc(struct backing_dev_info *bdi)
{
	unsigned long flags;

	local_irq_save(flags);
	__bdi_writeout_inc(bdi);
	local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(bdi_writeout_inc);

void task_dirty_inc(struct task_struct *tsk)
{
	prop_inc_single(&vm_dirties, &tsk->dirties);
}

static void bdi_writeout_fraction(struct backing_dev_info *bdi,
		long *numerator, long *denominator)
{
	if (bdi_cap_writeback_dirty(bdi)) {
		prop_fraction_percpu(&vm_completions, &bdi->completions,
				numerator, denominator);
	} else {
		*numerator = 0;
		*denominator = 1;
	}
}

static void clip_bdi_dirty_limit(struct backing_dev_info *bdi,
		unsigned long dirty, unsigned long *pbdi_dirty)
{
	unsigned long avail_dirty;

	avail_dirty = global_page_state(NR_FILE_DIRTY) +
		 global_page_state(NR_WRITEBACK) +
		 global_page_state(NR_UNSTABLE_NFS) +
		 global_page_state(NR_WRITEBACK_TEMP);

	if (avail_dirty < dirty)
		avail_dirty = dirty - avail_dirty;
	else
		avail_dirty = 0;

	avail_dirty += bdi_stat(bdi, BDI_RECLAIMABLE) +
		bdi_stat(bdi, BDI_WRITEBACK);

	*pbdi_dirty = min(*pbdi_dirty, avail_dirty);
}

static inline void task_dirties_fraction(struct task_struct *tsk,
		long *numerator, long *denominator)
{
	prop_fraction_single(&vm_dirties, &tsk->dirties,
				numerator, denominator);
}

static void task_dirty_limit(struct task_struct *tsk, unsigned long *pdirty)
{
	long numerator, denominator;
	unsigned long dirty = *pdirty;
	u64 inv = dirty >> 3;

	task_dirties_fraction(tsk, &numerator, &denominator);
	inv *= numerator;
	do_div(inv, denominator);

	dirty -= inv;
	if (dirty < *pdirty/2)
		dirty = *pdirty/2;

	*pdirty = dirty;
}

static unsigned int bdi_min_ratio;

int bdi_set_min_ratio(struct backing_dev_info *bdi, unsigned int min_ratio)
{
	int ret = 0;

	spin_lock_bh(&bdi_lock);
	if (min_ratio > bdi->max_ratio) {
		ret = -EINVAL;
	} else {
		min_ratio -= bdi->min_ratio;
		if (bdi_min_ratio + min_ratio < 100) {
			bdi_min_ratio += min_ratio;
			bdi->min_ratio += min_ratio;
		} else {
			ret = -EINVAL;
		}
	}
	spin_unlock_bh(&bdi_lock);

	return ret;
}

int bdi_set_max_ratio(struct backing_dev_info *bdi, unsigned max_ratio)
{
	int ret = 0;

	if (max_ratio > 100)
		return -EINVAL;

	spin_lock_bh(&bdi_lock);
	if (bdi->min_ratio > max_ratio) {
		ret = -EINVAL;
	} else {
		bdi->max_ratio = max_ratio;
		bdi->max_prop_frac = (PROP_FRAC_BASE * max_ratio) / 100;
	}
	spin_unlock_bh(&bdi_lock);

	return ret;
}
EXPORT_SYMBOL(bdi_set_max_ratio);


static unsigned long highmem_dirtyable_memory(unsigned long total)
{
#ifdef CONFIG_HIGHMEM
	int node;
	unsigned long x = 0;

	for_each_node_state(node, N_HIGH_MEMORY) {
		struct zone *z =
			&NODE_DATA(node)->node_zones[ZONE_HIGHMEM];

		x += zone_page_state(z, NR_FREE_PAGES) +
		     zone_reclaimable_pages(z);
	}
	/*
	 * Make sure that the number of highmem pages is never larger
	 * than the number of the total dirtyable memory. This can only
	 * occur in very strange VM situations but we want to make sure
	 * that this does not occur.
	 */
	return min(x, total);
#else
	return 0;
#endif
}

unsigned long determine_dirtyable_memory(void)
{
	unsigned long x;

	x = global_page_state(NR_FREE_PAGES) + global_reclaimable_pages();

	if (!vm_highmem_is_dirtyable)
		x -= highmem_dirtyable_memory(x);

	return x + 1;	/* Ensure that we never return 0 */
}

void
get_dirty_limits(unsigned long *pbackground, unsigned long *pdirty,
		 unsigned long *pbdi_dirty, struct backing_dev_info *bdi)
{
	unsigned long background;
	unsigned long dirty;
	unsigned long available_memory = determine_dirtyable_memory();
	struct task_struct *tsk;

	if (vm_dirty_bytes)
		dirty = DIV_ROUND_UP(vm_dirty_bytes, PAGE_SIZE);
	else {
		int dirty_ratio;

		dirty_ratio = vm_dirty_ratio;
		if (dirty_ratio < 5)
			dirty_ratio = 5;
		dirty = (dirty_ratio * available_memory) / 100;
	}

	if (dirty_background_bytes)
		background = DIV_ROUND_UP(dirty_background_bytes, PAGE_SIZE);
	else
		background = (dirty_background_ratio * available_memory) / 100;

	if (background >= dirty)
		background = dirty / 2;
	tsk = current;
	if (tsk->flags & PF_LESS_THROTTLE || rt_task(tsk)) {
		background += background / 4;
		dirty += dirty / 4;
	}
	*pbackground = background;
	*pdirty = dirty;

	if (bdi) {
		u64 bdi_dirty;
		long numerator, denominator;

		/*
		 * Calculate this BDI's share of the dirty ratio.
		 */
		bdi_writeout_fraction(bdi, &numerator, &denominator);

		bdi_dirty = (dirty * (100 - bdi_min_ratio)) / 100;
		bdi_dirty *= numerator;
		do_div(bdi_dirty, denominator);
		bdi_dirty += (dirty * bdi->min_ratio) / 100;
		if (bdi_dirty > (dirty * bdi->max_ratio) / 100)
			bdi_dirty = dirty * bdi->max_ratio / 100;

		*pbdi_dirty = bdi_dirty;
		clip_bdi_dirty_limit(bdi, dirty, pbdi_dirty);
		task_dirty_limit(current, pbdi_dirty);
	}
}

static void balance_dirty_pages(struct address_space *mapping,
				unsigned long write_chunk)
{
	long nr_reclaimable, bdi_nr_reclaimable;
	long nr_writeback, bdi_nr_writeback;
	unsigned long background_thresh;
	unsigned long dirty_thresh;
	unsigned long bdi_thresh;
	unsigned long pages_written = 0;
	unsigned long pause = 1;

	struct backing_dev_info *bdi = mapping->backing_dev_info;

	for (;;) {
		struct writeback_control wbc = {
			.sync_mode	= WB_SYNC_NONE,
			.older_than_this = NULL,
			.nr_to_write	= write_chunk,
			.range_cyclic	= 1,
		};

		get_dirty_limits(&background_thresh, &dirty_thresh,
				&bdi_thresh, bdi);

		nr_reclaimable = global_page_state(NR_FILE_DIRTY) +
					global_page_state(NR_UNSTABLE_NFS);
		nr_writeback = global_page_state(NR_WRITEBACK);

		bdi_nr_reclaimable = bdi_stat(bdi, BDI_RECLAIMABLE);
		bdi_nr_writeback = bdi_stat(bdi, BDI_WRITEBACK);

		if (bdi_nr_reclaimable + bdi_nr_writeback <= bdi_thresh)
			break;

		/*
		 * Throttle it only when the background writeback cannot
		 * catch-up. This avoids (excessively) small writeouts
		 * when the bdi limits are ramping up.
		 */
		if (nr_reclaimable + nr_writeback <
				(background_thresh + dirty_thresh) / 2)
			break;

		if (!bdi->dirty_exceeded)
			bdi->dirty_exceeded = 1;

		/* Note: nr_reclaimable denotes nr_dirty + nr_unstable.
		 * Unstable writes are a feature of certain networked
		 * filesystems (i.e. NFS) in which data may have been
		 * written to the server's write cache, but has not yet
		 * been flushed to permanent storage.
		 * Only move pages to writeback if this bdi is over its
		 * threshold otherwise wait until the disk writes catch
		 * up.
		 */
		if (bdi_nr_reclaimable > bdi_thresh) {
			writeback_inodes_wb(&bdi->wb, &wbc);
			pages_written += write_chunk - wbc.nr_to_write;
			get_dirty_limits(&background_thresh, &dirty_thresh,
				       &bdi_thresh, bdi);
		}

		/*
		 * In order to avoid the stacked BDI deadlock we need
		 * to ensure we accurately count the 'dirty' pages when
		 * the threshold is low.
		 *
		 * Otherwise it would be possible to get thresh+n pages
		 * reported dirty, even though there are thresh-m pages
		 * actually dirty; with m+n sitting in the percpu
		 * deltas.
		 */
		if (bdi_thresh < 2*bdi_stat_error(bdi)) {
			bdi_nr_reclaimable = bdi_stat_sum(bdi, BDI_RECLAIMABLE);
			bdi_nr_writeback = bdi_stat_sum(bdi, BDI_WRITEBACK);
		} else if (bdi_nr_reclaimable) {
			bdi_nr_reclaimable = bdi_stat(bdi, BDI_RECLAIMABLE);
			bdi_nr_writeback = bdi_stat(bdi, BDI_WRITEBACK);
		}

		if (bdi_nr_reclaimable + bdi_nr_writeback <= bdi_thresh)
			break;
		if (pages_written >= write_chunk)
			break;		/* We've done our duty */

		__set_current_state(TASK_INTERRUPTIBLE);
		io_schedule_timeout(pause);

		/*
		 * Increase the delay for each loop, up to our previous
		 * default of taking a 100ms nap.
		 */
		pause <<= 1;
		if (pause > HZ / 10)
			pause = HZ / 10;
	}

	if (bdi_nr_reclaimable + bdi_nr_writeback < bdi_thresh &&
			bdi->dirty_exceeded)
		bdi->dirty_exceeded = 0;

	if (writeback_in_progress(bdi))
		return;

	/*
	 * In laptop mode, we wait until hitting the higher threshold before
	 * starting background writeout, and then write out all the way down
	 * to the lower threshold.  So slow writers cause minimal disk activity.
	 *
	 * In normal mode, we start background writeout at the lower
	 * background_thresh, to keep the amount of dirty memory low.
	 */
	if ((laptop_mode && pages_written) ||
	    (!laptop_mode && ((global_page_state(NR_FILE_DIRTY)
			       + global_page_state(NR_UNSTABLE_NFS))
					  > background_thresh)))
		bdi_start_background_writeback(bdi);
}

void set_page_dirty_balance(struct page *page, int page_mkwrite)
{
	if (set_page_dirty(page) || page_mkwrite) {
		struct address_space *mapping = page_mapping(page);

		if (mapping)
			balance_dirty_pages_ratelimited(mapping);
	}
}

static DEFINE_PER_CPU(unsigned long, bdp_ratelimits) = 0;

void balance_dirty_pages_ratelimited_nr(struct address_space *mapping,
					unsigned long nr_pages_dirtied)
{
	unsigned long ratelimit;
	unsigned long *p;

	ratelimit = ratelimit_pages;
	if (mapping->backing_dev_info->dirty_exceeded)
		ratelimit = 8;

	/*
	 * Check the rate limiting. Also, we do not want to throttle real-time
	 * tasks in balance_dirty_pages(). Period.
	 */
	preempt_disable();
	p =  &__get_cpu_var(bdp_ratelimits);
	*p += nr_pages_dirtied;
	if (unlikely(*p >= ratelimit)) {
		ratelimit = sync_writeback_pages(*p);
		*p = 0;
		preempt_enable();
		balance_dirty_pages(mapping, ratelimit);
		return;
	}
	preempt_enable();
}
EXPORT_SYMBOL(balance_dirty_pages_ratelimited_nr);

void throttle_vm_writeout(gfp_t gfp_mask)
{
	unsigned long background_thresh;
	unsigned long dirty_thresh;

        for ( ; ; ) {
		get_dirty_limits(&background_thresh, &dirty_thresh, NULL, NULL);

                /*
                 * Boost the allowable dirty threshold a bit for page
                 * allocators so they don't get DoS'ed by heavy writers
                 */
                dirty_thresh += dirty_thresh / 10;      /* wheeee... */

                if (global_page_state(NR_UNSTABLE_NFS) +
			global_page_state(NR_WRITEBACK) <= dirty_thresh)
                        	break;
                congestion_wait(BLK_RW_ASYNC, HZ/10);

		/*
		 * The caller might hold locks which can prevent IO completion
		 * or progress in the filesystem.  So we cannot just sit here
		 * waiting for IO to complete.
		 */
		if ((gfp_mask & (__GFP_FS|__GFP_IO)) != (__GFP_FS|__GFP_IO))
			break;
        }
}

int dirty_writeback_centisecs_handler(ctl_table *table, int write,
	void __user *buffer, size_t *length, loff_t *ppos)
{
	proc_dointvec(table, write, buffer, length, ppos);
	bdi_arm_supers_timer();
	return 0;
}

#ifdef CONFIG_BLOCK
void laptop_mode_timer_fn(unsigned long data)
{
	struct request_queue *q = (struct request_queue *)data;
	int nr_pages = global_page_state(NR_FILE_DIRTY) +
		global_page_state(NR_UNSTABLE_NFS);

	/*
	 * We want to write everything out, not just down to the dirty
	 * threshold
	 */
	if (bdi_has_dirty_io(&q->backing_dev_info))
		bdi_start_writeback(&q->backing_dev_info, nr_pages);
}

void laptop_io_completion(struct backing_dev_info *info)
{
	mod_timer(&info->laptop_mode_wb_timer, jiffies + laptop_mode);
}

void laptop_sync_completion(void)
{
	struct backing_dev_info *bdi;

	rcu_read_lock();

	list_for_each_entry_rcu(bdi, &bdi_list, bdi_list)
		del_timer(&bdi->laptop_mode_wb_timer);

	rcu_read_unlock();
}
#endif


void writeback_set_ratelimit(void)
{
	ratelimit_pages = vm_total_pages / (num_online_cpus() * 32);
	if (ratelimit_pages < 16)
		ratelimit_pages = 16;
	if (ratelimit_pages * PAGE_CACHE_SIZE > 4096 * 1024)
		ratelimit_pages = (4096 * 1024) / PAGE_CACHE_SIZE;
}

static int __cpuinit
ratelimit_handler(struct notifier_block *self, unsigned long u, void *v)
{
	writeback_set_ratelimit();
	return NOTIFY_DONE;
}

static struct notifier_block __cpuinitdata ratelimit_nb = {
	.notifier_call	= ratelimit_handler,
	.next		= NULL,
};

void __init page_writeback_init(void)
{
	int shift;

	writeback_set_ratelimit();
	register_cpu_notifier(&ratelimit_nb);

	shift = calc_period_shift();
	prop_descriptor_init(&vm_completions, shift);
	prop_descriptor_init(&vm_dirties, shift);
}

int write_cache_pages(struct address_space *mapping,
		      struct writeback_control *wbc, writepage_t writepage,
		      void *data)
{
	int ret = 0;
	int done = 0;
	struct pagevec pvec;
	int nr_pages;
	pgoff_t uninitialized_var(writeback_index);
	pgoff_t index;
	pgoff_t end;		/* Inclusive */
	pgoff_t done_index;
	int cycled;
	int range_whole = 0;

	pagevec_init(&pvec, 0);
	if (wbc->range_cyclic) {
		writeback_index = mapping->writeback_index; /* prev offset */
		index = writeback_index;
		if (index == 0)
			cycled = 1;
		else
			cycled = 0;
		end = -1;
	} else {
		index = wbc->range_start >> PAGE_CACHE_SHIFT;
		end = wbc->range_end >> PAGE_CACHE_SHIFT;
		if (wbc->range_start == 0 && wbc->range_end == LLONG_MAX)
			range_whole = 1;
		cycled = 1; /* ignore range_cyclic tests */

		/*
		 * If this is a data integrity sync, cap the writeback to the
		 * current end of file. Any extension to the file that occurs
		 * after this is a new write and we don't need to write those
		 * pages out to fulfil our data integrity requirements. If we
		 * try to write them out, we can get stuck in this scan until
		 * the concurrent writer stops adding dirty pages and extending
		 * EOF.
		 */
		if (wbc->sync_mode == WB_SYNC_ALL &&
		    wbc->range_end == LLONG_MAX) {
			end = i_size_read(mapping->host) >> PAGE_CACHE_SHIFT;
		}
	}

retry:
	done_index = index;
	while (!done && (index <= end)) {
		int i;

		nr_pages = pagevec_lookup_tag(&pvec, mapping, &index,
			      PAGECACHE_TAG_DIRTY,
			      min(end - index, (pgoff_t)PAGEVEC_SIZE-1) + 1);
		if (nr_pages == 0)
			break;

		for (i = 0; i < nr_pages; i++) {
			struct page *page = pvec.pages[i];

			/*
			 * At this point, the page may be truncated or
			 * invalidated (changing page->mapping to NULL), or
			 * even swizzled back from swapper_space to tmpfs file
			 * mapping. However, page->index will not change
			 * because we have a reference on the page.
			 */
			if (page->index > end) {
				/*
				 * can't be range_cyclic (1st pass) because
				 * end == -1 in that case.
				 */
				done = 1;
				break;
			}

			done_index = page->index + 1;

			lock_page(page);

			/*
			 * Page truncated or invalidated. We can freely skip it
			 * then, even for data integrity operations: the page
			 * has disappeared concurrently, so there could be no
			 * real expectation of this data interity operation
			 * even if there is now a new, dirty page at the same
			 * pagecache address.
			 */
			if (unlikely(page->mapping != mapping)) {
continue_unlock:
				unlock_page(page);
				continue;
			}

			if (!PageDirty(page)) {
				/* someone wrote it for us */
				goto continue_unlock;
			}

			if (PageWriteback(page)) {
				if (wbc->sync_mode != WB_SYNC_NONE)
					wait_on_page_writeback(page);
				else
					goto continue_unlock;
			}

			BUG_ON(PageWriteback(page));
			if (!clear_page_dirty_for_io(page))
				goto continue_unlock;

			ret = (*writepage)(page, wbc, data);
			if (unlikely(ret)) {
				if (ret == AOP_WRITEPAGE_ACTIVATE) {
					unlock_page(page);
					ret = 0;
				} else {
					/*
					 * done_index is set past this page,
					 * so media errors will not choke
					 * background writeout for the entire
					 * file. This has consequences for
					 * range_cyclic semantics (ie. it may
					 * not be suitable for data integrity
					 * writeout).
					 */
					done = 1;
					break;
				}
			}

			/*
			 * We stop writing back only if we are not doing
			 * integrity sync. In case of integrity sync we have to
			 * keep going until we have written all the pages
			 * we tagged for writeback prior to entering this loop.
			 */
			if (--wbc->nr_to_write <= 0 &&
			    wbc->sync_mode == WB_SYNC_NONE) {
				done = 1;
				break;
			}
		}
		pagevec_release(&pvec);
		cond_resched();
	}
	if (!cycled && !done) {
		/*
		 * range_cyclic:
		 * We hit the last page and there is more work to be done: wrap
		 * back to the start of the file
		 */
		cycled = 1;
		index = 0;
		end = writeback_index - 1;
		goto retry;
	}
	if (wbc->range_cyclic || (range_whole && wbc->nr_to_write > 0))
		mapping->writeback_index = done_index;

	return ret;
}
EXPORT_SYMBOL(write_cache_pages);

static int __writepage(struct page *page, struct writeback_control *wbc,
		       void *data)
{
	struct address_space *mapping = data;
	int ret = mapping->a_ops->writepage(page, wbc);
	mapping_set_error(mapping, ret);
	return ret;
}

int generic_writepages(struct address_space *mapping,
		       struct writeback_control *wbc)
{
	/* deal with chardevs and other special file */
	if (!mapping->a_ops->writepage)
		return 0;

	return write_cache_pages(mapping, wbc, __writepage, mapping);
}

EXPORT_SYMBOL(generic_writepages);

int do_writepages(struct address_space *mapping, struct writeback_control *wbc)
{
	int ret;

	if (wbc->nr_to_write <= 0)
		return 0;
	if (mapping->a_ops->writepages)
		ret = mapping->a_ops->writepages(mapping, wbc);
	else
		ret = generic_writepages(mapping, wbc);
	return ret;
}

int write_one_page(struct page *page, int wait)
{
	struct address_space *mapping = page->mapping;
	int ret = 0;
	struct writeback_control wbc = {
		.sync_mode = WB_SYNC_ALL,
		.nr_to_write = 1,
	};

	BUG_ON(!PageLocked(page));

	if (wait)
		wait_on_page_writeback(page);

	if (clear_page_dirty_for_io(page)) {
		page_cache_get(page);
		ret = mapping->a_ops->writepage(page, &wbc);
		if (ret == 0 && wait) {
			wait_on_page_writeback(page);
			if (PageError(page))
				ret = -EIO;
		}
		page_cache_release(page);
	} else {
		unlock_page(page);
	}
	return ret;
}
EXPORT_SYMBOL(write_one_page);

int __set_page_dirty_no_writeback(struct page *page)
{
	if (!PageDirty(page))
		SetPageDirty(page);
	return 0;
}

void account_page_dirtied(struct page *page, struct address_space *mapping)
{
	if (mapping_cap_account_dirty(mapping)) {
		__inc_zone_page_state(page, NR_FILE_DIRTY);
		__inc_bdi_stat(mapping->backing_dev_info, BDI_RECLAIMABLE);
		task_dirty_inc(current);
		task_io_account_write(PAGE_CACHE_SIZE);
	}
}

int __set_page_dirty_nobuffers(struct page *page)
{
	if (!TestSetPageDirty(page)) {
		struct address_space *mapping = page_mapping(page);
		struct address_space *mapping2;

		if (!mapping)
			return 1;

		spin_lock_irq(&mapping->tree_lock);
		mapping2 = page_mapping(page);
		if (mapping2) { /* Race with truncate? */
			BUG_ON(mapping2 != mapping);
			WARN_ON_ONCE(!PagePrivate(page) && !PageUptodate(page));
			account_page_dirtied(page, mapping);
			radix_tree_tag_set(&mapping->page_tree,
				page_index(page), PAGECACHE_TAG_DIRTY);
		}
		spin_unlock_irq(&mapping->tree_lock);
		if (mapping->host) {
			/* !PageAnon && !swapper_space */
			__mark_inode_dirty(mapping->host, I_DIRTY_PAGES);
		}
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(__set_page_dirty_nobuffers);

int redirty_page_for_writepage(struct writeback_control *wbc, struct page *page)
{
	wbc->pages_skipped++;
	return __set_page_dirty_nobuffers(page);
}
EXPORT_SYMBOL(redirty_page_for_writepage);

int set_page_dirty(struct page *page)
{
	struct address_space *mapping = page_mapping(page);

	if (likely(mapping)) {
		int (*spd)(struct page *) = mapping->a_ops->set_page_dirty;
#ifdef CONFIG_BLOCK
		if (!spd)
			spd = __set_page_dirty_buffers;
#endif
		return (*spd)(page);
	}
	if (!PageDirty(page)) {
		if (!TestSetPageDirty(page))
			return 1;
	}
	return 0;
}
EXPORT_SYMBOL(set_page_dirty);

int set_page_dirty_lock(struct page *page)
{
	int ret;

	lock_page_nosync(page);
	ret = set_page_dirty(page);
	unlock_page(page);
	return ret;
}
EXPORT_SYMBOL(set_page_dirty_lock);

int clear_page_dirty_for_io(struct page *page)
{
	struct address_space *mapping = page_mapping(page);

	BUG_ON(!PageLocked(page));

	ClearPageReclaim(page);
	if (mapping && mapping_cap_account_dirty(mapping)) {
		/*
		 * Yes, Virginia, this is indeed insane.
		 *
		 * We use this sequence to make sure that
		 *  (a) we account for dirty stats properly
		 *  (b) we tell the low-level filesystem to
		 *      mark the whole page dirty if it was
		 *      dirty in a pagetable. Only to then
		 *  (c) clean the page again and return 1 to
		 *      cause the writeback.
		 *
		 * This way we avoid all nasty races with the
		 * dirty bit in multiple places and clearing
		 * them concurrently from different threads.
		 *
		 * Note! Normally the "set_page_dirty(page)"
		 * has no effect on the actual dirty bit - since
		 * that will already usually be set. But we
		 * need the side effects, and it can help us
		 * avoid races.
		 *
		 * We basically use the page "master dirty bit"
		 * as a serialization point for all the different
		 * threads doing their things.
		 */
		if (page_mkclean(page))
			set_page_dirty(page);
		/*
		 * We carefully synchronise fault handlers against
		 * installing a dirty pte and marking the page dirty
		 * at this point. We do this by having them hold the
		 * page lock at some point after installing their
		 * pte, but before marking the page dirty.
		 * Pages are always locked coming in here, so we get
		 * the desired exclusion. See mm/memory.c:do_wp_page()
		 * for more comments.
		 */
		if (TestClearPageDirty(page)) {
			dec_zone_page_state(page, NR_FILE_DIRTY);
			dec_bdi_stat(mapping->backing_dev_info,
					BDI_RECLAIMABLE);
			return 1;
		}
		return 0;
	}
	return TestClearPageDirty(page);
}
EXPORT_SYMBOL(clear_page_dirty_for_io);

int test_clear_page_writeback(struct page *page)
{
	struct address_space *mapping = page_mapping(page);
	int ret;

	if (mapping) {
		struct backing_dev_info *bdi = mapping->backing_dev_info;
		unsigned long flags;

		spin_lock_irqsave(&mapping->tree_lock, flags);
		ret = TestClearPageWriteback(page);
		if (ret) {
			radix_tree_tag_clear(&mapping->page_tree,
						page_index(page),
						PAGECACHE_TAG_WRITEBACK);
			if (bdi_cap_account_writeback(bdi)) {
				__dec_bdi_stat(bdi, BDI_WRITEBACK);
				__bdi_writeout_inc(bdi);
			}
		}
		spin_unlock_irqrestore(&mapping->tree_lock, flags);
	} else {
		ret = TestClearPageWriteback(page);
	}
	if (ret)
		dec_zone_page_state(page, NR_WRITEBACK);
	return ret;
}

int test_set_page_writeback(struct page *page)
{
	struct address_space *mapping = page_mapping(page);
	int ret;

	if (mapping) {
		struct backing_dev_info *bdi = mapping->backing_dev_info;
		unsigned long flags;

		spin_lock_irqsave(&mapping->tree_lock, flags);
		ret = TestSetPageWriteback(page);
		if (!ret) {
			radix_tree_tag_set(&mapping->page_tree,
						page_index(page),
						PAGECACHE_TAG_WRITEBACK);
			if (bdi_cap_account_writeback(bdi))
				__inc_bdi_stat(bdi, BDI_WRITEBACK);
		}
		if (!PageDirty(page))
			radix_tree_tag_clear(&mapping->page_tree,
						page_index(page),
						PAGECACHE_TAG_DIRTY);
		spin_unlock_irqrestore(&mapping->tree_lock, flags);
	} else {
		ret = TestSetPageWriteback(page);
	}
	if (!ret)
		inc_zone_page_state(page, NR_WRITEBACK);
	return ret;

}
EXPORT_SYMBOL(test_set_page_writeback);

int mapping_tagged(struct address_space *mapping, int tag)
{
	int ret;
	rcu_read_lock();
	ret = radix_tree_tagged(&mapping->page_tree, tag);
	rcu_read_unlock();
	return ret;
}
EXPORT_SYMBOL(mapping_tagged);


#include "xfs.h"
#include "xfs_bit.h"
#include "xfs_log.h"
#include "xfs_inum.h"
#include "xfs_sb.h"
#include "xfs_ag.h"
#include "xfs_dir2.h"
#include "xfs_trans.h"
#include "xfs_dmapi.h"
#include "xfs_mount.h"
#include "xfs_bmap_btree.h"
#include "xfs_alloc_btree.h"
#include "xfs_ialloc_btree.h"
#include "xfs_dir2_sf.h"
#include "xfs_attr_sf.h"
#include "xfs_dinode.h"
#include "xfs_inode.h"
#include "xfs_alloc.h"
#include "xfs_btree.h"
#include "xfs_error.h"
#include "xfs_rw.h"
#include "xfs_iomap.h"
#include "xfs_vnodeops.h"
#include "xfs_trace.h"
#include "xfs_bmap.h"
#include <linux/gfp.h>
#include <linux/mpage.h>
#include <linux/pagevec.h>
#include <linux/writeback.h>

enum {
	IO_READ,	/* mapping for a read */
	IO_DELAY,	/* mapping covers delalloc region */
	IO_UNWRITTEN,	/* mapping covers allocated but uninitialized data */
	IO_NEW		/* just allocated */
};

#define NVSYNC		37
#define to_ioend_wq(v)	(&xfs_ioend_wq[((unsigned long)v) % NVSYNC])
static wait_queue_head_t xfs_ioend_wq[NVSYNC];

void __init
xfs_ioend_init(void)
{
	int i;

	for (i = 0; i < NVSYNC; i++)
		init_waitqueue_head(&xfs_ioend_wq[i]);
}

void
xfs_ioend_wait(
	xfs_inode_t	*ip)
{
	wait_queue_head_t *wq = to_ioend_wq(ip);

	wait_event(*wq, (atomic_read(&ip->i_iocount) == 0));
}

STATIC void
xfs_ioend_wake(
	xfs_inode_t	*ip)
{
	if (atomic_dec_and_test(&ip->i_iocount))
		wake_up(to_ioend_wq(ip));
}

void
xfs_count_page_state(
	struct page		*page,
	int			*delalloc,
	int			*unmapped,
	int			*unwritten)
{
	struct buffer_head	*bh, *head;

	*delalloc = *unmapped = *unwritten = 0;

	bh = head = page_buffers(page);
	do {
		if (buffer_uptodate(bh) && !buffer_mapped(bh))
			(*unmapped) = 1;
		else if (buffer_unwritten(bh))
			(*unwritten) = 1;
		else if (buffer_delay(bh))
			(*delalloc) = 1;
	} while ((bh = bh->b_this_page) != head);
}

STATIC struct block_device *
xfs_find_bdev_for_inode(
	struct inode		*inode)
{
	struct xfs_inode	*ip = XFS_I(inode);
	struct xfs_mount	*mp = ip->i_mount;

	if (XFS_IS_REALTIME_INODE(ip))
		return mp->m_rtdev_targp->bt_bdev;
	else
		return mp->m_ddev_targp->bt_bdev;
}

STATIC void
xfs_destroy_ioend(
	xfs_ioend_t		*ioend)
{
	struct buffer_head	*bh, *next;
	struct xfs_inode	*ip = XFS_I(ioend->io_inode);

	for (bh = ioend->io_buffer_head; bh; bh = next) {
		next = bh->b_private;
		bh->b_end_io(bh, !ioend->io_error);
	}

	/*
	 * Volume managers supporting multiple paths can send back ENODEV
	 * when the final path disappears.  In this case continuing to fill
	 * the page cache with dirty data which cannot be written out is
	 * evil, so prevent that.
	 */
	if (unlikely(ioend->io_error == -ENODEV)) {
		xfs_do_force_shutdown(ip->i_mount, SHUTDOWN_DEVICE_REQ,
				      __FILE__, __LINE__);
	}

	xfs_ioend_wake(ip);
	mempool_free(ioend, xfs_ioend_pool);
}

STATIC xfs_fsize_t
xfs_ioend_new_eof(
	xfs_ioend_t		*ioend)
{
	xfs_inode_t		*ip = XFS_I(ioend->io_inode);
	xfs_fsize_t		isize;
	xfs_fsize_t		bsize;

	bsize = ioend->io_offset + ioend->io_size;
	isize = MAX(ip->i_size, ip->i_new_size);
	isize = MIN(isize, bsize);
	return isize > ip->i_d.di_size ? isize : 0;
}

STATIC int
xfs_setfilesize(
	xfs_ioend_t		*ioend)
{
	xfs_inode_t		*ip = XFS_I(ioend->io_inode);
	xfs_fsize_t		isize;

	ASSERT((ip->i_d.di_mode & S_IFMT) == S_IFREG);
	ASSERT(ioend->io_type != IO_READ);

	if (unlikely(ioend->io_error))
		return 0;

	if (!xfs_ilock_nowait(ip, XFS_ILOCK_EXCL))
		return EAGAIN;

	isize = xfs_ioend_new_eof(ioend);
	if (isize) {
		ip->i_d.di_size = isize;
		xfs_mark_inode_dirty(ip);
	}

	xfs_iunlock(ip, XFS_ILOCK_EXCL);
	return 0;
}

STATIC void
xfs_finish_ioend(
	xfs_ioend_t	*ioend,
	int		wait)
{
	if (atomic_dec_and_test(&ioend->io_remaining)) {
		struct workqueue_struct *wq;

		wq = (ioend->io_type == IO_UNWRITTEN) ?
			xfsconvertd_workqueue : xfsdatad_workqueue;
		queue_work(wq, &ioend->io_work);
		if (wait)
			flush_workqueue(wq);
	}
}

STATIC void
xfs_end_io(
	struct work_struct *work)
{
	xfs_ioend_t	*ioend = container_of(work, xfs_ioend_t, io_work);
	struct xfs_inode *ip = XFS_I(ioend->io_inode);
	int		error = 0;

	/*
	 * For unwritten extents we need to issue transactions to convert a
	 * range to normal written extens after the data I/O has finished.
	 */
	if (ioend->io_type == IO_UNWRITTEN &&
	    likely(!ioend->io_error && !XFS_FORCED_SHUTDOWN(ip->i_mount))) {

		error = xfs_iomap_write_unwritten(ip, ioend->io_offset,
						 ioend->io_size);
		if (error)
			ioend->io_error = error;
	}

	/*
	 * We might have to update the on-disk file size after extending
	 * writes.
	 */
	if (ioend->io_type != IO_READ) {
		error = xfs_setfilesize(ioend);
		ASSERT(!error || error == EAGAIN);
	}

	/*
	 * If we didn't complete processing of the ioend, requeue it to the
	 * tail of the workqueue for another attempt later. Otherwise destroy
	 * it.
	 */
	if (error == EAGAIN) {
		atomic_inc(&ioend->io_remaining);
		xfs_finish_ioend(ioend, 0);
		/* ensure we don't spin on blocked ioends */
		delay(1);
	} else {
		if (ioend->io_iocb)
			aio_complete(ioend->io_iocb, ioend->io_result, 0);
		xfs_destroy_ioend(ioend);
	}
}

STATIC xfs_ioend_t *
xfs_alloc_ioend(
	struct inode		*inode,
	unsigned int		type)
{
	xfs_ioend_t		*ioend;

	ioend = mempool_alloc(xfs_ioend_pool, GFP_NOFS);

	/*
	 * Set the count to 1 initially, which will prevent an I/O
	 * completion callback from happening before we have started
	 * all the I/O from calling the completion routine too early.
	 */
	atomic_set(&ioend->io_remaining, 1);
	ioend->io_error = 0;
	ioend->io_list = NULL;
	ioend->io_type = type;
	ioend->io_inode = inode;
	ioend->io_buffer_head = NULL;
	ioend->io_buffer_tail = NULL;
	atomic_inc(&XFS_I(ioend->io_inode)->i_iocount);
	ioend->io_offset = 0;
	ioend->io_size = 0;
	ioend->io_iocb = NULL;
	ioend->io_result = 0;

	INIT_WORK(&ioend->io_work, xfs_end_io);
	return ioend;
}

STATIC int
xfs_map_blocks(
	struct inode		*inode,
	loff_t			offset,
	ssize_t			count,
	struct xfs_bmbt_irec	*imap,
	int			flags)
{
	int			nmaps = 1;
	int			new = 0;

	return -xfs_iomap(XFS_I(inode), offset, count, flags, imap, &nmaps, &new);
}

STATIC int
xfs_imap_valid(
	struct inode		*inode,
	struct xfs_bmbt_irec	*imap,
	xfs_off_t		offset)
{
	offset >>= inode->i_blkbits;

	return offset >= imap->br_startoff &&
		offset < imap->br_startoff + imap->br_blockcount;
}

STATIC void
xfs_end_bio(
	struct bio		*bio,
	int			error)
{
	xfs_ioend_t		*ioend = bio->bi_private;

	ASSERT(atomic_read(&bio->bi_cnt) >= 1);
	ioend->io_error = test_bit(BIO_UPTODATE, &bio->bi_flags) ? 0 : error;

	/* Toss bio and pass work off to an xfsdatad thread */
	bio->bi_private = NULL;
	bio->bi_end_io = NULL;
	bio_put(bio);

	xfs_finish_ioend(ioend, 0);
}

STATIC void
xfs_submit_ioend_bio(
	struct writeback_control *wbc,
	xfs_ioend_t		*ioend,
	struct bio		*bio)
{
	atomic_inc(&ioend->io_remaining);
	bio->bi_private = ioend;
	bio->bi_end_io = xfs_end_bio;

	/*
	 * If the I/O is beyond EOF we mark the inode dirty immediately
	 * but don't update the inode size until I/O completion.
	 */
	if (xfs_ioend_new_eof(ioend))
		xfs_mark_inode_dirty(XFS_I(ioend->io_inode));

	submit_bio(wbc->sync_mode == WB_SYNC_ALL ?
		   WRITE_SYNC_PLUG : WRITE, bio);
	ASSERT(!bio_flagged(bio, BIO_EOPNOTSUPP));
	bio_put(bio);
}

STATIC struct bio *
xfs_alloc_ioend_bio(
	struct buffer_head	*bh)
{
	struct bio		*bio;
	int			nvecs = bio_get_nr_vecs(bh->b_bdev);

	do {
		bio = bio_alloc(GFP_NOIO, nvecs);
		nvecs >>= 1;
	} while (!bio);

	ASSERT(bio->bi_private == NULL);
	bio->bi_sector = bh->b_blocknr * (bh->b_size >> 9);
	bio->bi_bdev = bh->b_bdev;
	bio_get(bio);
	return bio;
}

STATIC void
xfs_start_buffer_writeback(
	struct buffer_head	*bh)
{
	ASSERT(buffer_mapped(bh));
	ASSERT(buffer_locked(bh));
	ASSERT(!buffer_delay(bh));
	ASSERT(!buffer_unwritten(bh));

	mark_buffer_async_write(bh);
	set_buffer_uptodate(bh);
	clear_buffer_dirty(bh);
}

STATIC void
xfs_start_page_writeback(
	struct page		*page,
	int			clear_dirty,
	int			buffers)
{
	ASSERT(PageLocked(page));
	ASSERT(!PageWriteback(page));
	if (clear_dirty)
		clear_page_dirty_for_io(page);
	set_page_writeback(page);
	unlock_page(page);
	/* If no buffers on the page are to be written, finish it here */
	if (!buffers)
		end_page_writeback(page);
}

static inline int bio_add_buffer(struct bio *bio, struct buffer_head *bh)
{
	return bio_add_page(bio, bh->b_page, bh->b_size, bh_offset(bh));
}

STATIC void
xfs_submit_ioend(
	struct writeback_control *wbc,
	xfs_ioend_t		*ioend)
{
	xfs_ioend_t		*head = ioend;
	xfs_ioend_t		*next;
	struct buffer_head	*bh;
	struct bio		*bio;
	sector_t		lastblock = 0;

	/* Pass 1 - start writeback */
	do {
		next = ioend->io_list;
		for (bh = ioend->io_buffer_head; bh; bh = bh->b_private) {
			xfs_start_buffer_writeback(bh);
		}
	} while ((ioend = next) != NULL);

	/* Pass 2 - submit I/O */
	ioend = head;
	do {
		next = ioend->io_list;
		bio = NULL;

		for (bh = ioend->io_buffer_head; bh; bh = bh->b_private) {

			if (!bio) {
 retry:
				bio = xfs_alloc_ioend_bio(bh);
			} else if (bh->b_blocknr != lastblock + 1) {
				xfs_submit_ioend_bio(wbc, ioend, bio);
				goto retry;
			}

			if (bio_add_buffer(bio, bh) != bh->b_size) {
				xfs_submit_ioend_bio(wbc, ioend, bio);
				goto retry;
			}

			lastblock = bh->b_blocknr;
		}
		if (bio)
			xfs_submit_ioend_bio(wbc, ioend, bio);
		xfs_finish_ioend(ioend, 0);
	} while ((ioend = next) != NULL);
}

STATIC void
xfs_cancel_ioend(
	xfs_ioend_t		*ioend)
{
	xfs_ioend_t		*next;
	struct buffer_head	*bh, *next_bh;

	do {
		next = ioend->io_list;
		bh = ioend->io_buffer_head;
		do {
			next_bh = bh->b_private;
			clear_buffer_async_write(bh);
			unlock_buffer(bh);
		} while ((bh = next_bh) != NULL);

		xfs_ioend_wake(XFS_I(ioend->io_inode));
		mempool_free(ioend, xfs_ioend_pool);
	} while ((ioend = next) != NULL);
}

STATIC void
xfs_add_to_ioend(
	struct inode		*inode,
	struct buffer_head	*bh,
	xfs_off_t		offset,
	unsigned int		type,
	xfs_ioend_t		**result,
	int			need_ioend)
{
	xfs_ioend_t		*ioend = *result;

	if (!ioend || need_ioend || type != ioend->io_type) {
		xfs_ioend_t	*previous = *result;

		ioend = xfs_alloc_ioend(inode, type);
		ioend->io_offset = offset;
		ioend->io_buffer_head = bh;
		ioend->io_buffer_tail = bh;
		if (previous)
			previous->io_list = ioend;
		*result = ioend;
	} else {
		ioend->io_buffer_tail->b_private = bh;
		ioend->io_buffer_tail = bh;
	}

	bh->b_private = NULL;
	ioend->io_size += bh->b_size;
}

STATIC void
xfs_map_buffer(
	struct inode		*inode,
	struct buffer_head	*bh,
	struct xfs_bmbt_irec	*imap,
	xfs_off_t		offset)
{
	sector_t		bn;
	struct xfs_mount	*m = XFS_I(inode)->i_mount;
	xfs_off_t		iomap_offset = XFS_FSB_TO_B(m, imap->br_startoff);
	xfs_daddr_t		iomap_bn = xfs_fsb_to_db(XFS_I(inode), imap->br_startblock);

	ASSERT(imap->br_startblock != HOLESTARTBLOCK);
	ASSERT(imap->br_startblock != DELAYSTARTBLOCK);

	bn = (iomap_bn >> (inode->i_blkbits - BBSHIFT)) +
	      ((offset - iomap_offset) >> inode->i_blkbits);

	ASSERT(bn || XFS_IS_REALTIME_INODE(XFS_I(inode)));

	bh->b_blocknr = bn;
	set_buffer_mapped(bh);
}

STATIC void
xfs_map_at_offset(
	struct inode		*inode,
	struct buffer_head	*bh,
	struct xfs_bmbt_irec	*imap,
	xfs_off_t		offset)
{
	ASSERT(imap->br_startblock != HOLESTARTBLOCK);
	ASSERT(imap->br_startblock != DELAYSTARTBLOCK);

	lock_buffer(bh);
	xfs_map_buffer(inode, bh, imap, offset);
	bh->b_bdev = xfs_find_bdev_for_inode(inode);
	set_buffer_mapped(bh);
	clear_buffer_delay(bh);
	clear_buffer_unwritten(bh);
}

STATIC unsigned int
xfs_probe_page(
	struct page		*page,
	unsigned int		pg_offset,
	int			mapped)
{
	int			ret = 0;

	if (PageWriteback(page))
		return 0;

	if (page->mapping && PageDirty(page)) {
		if (page_has_buffers(page)) {
			struct buffer_head	*bh, *head;

			bh = head = page_buffers(page);
			do {
				if (!buffer_uptodate(bh))
					break;
				if (mapped != buffer_mapped(bh))
					break;
				ret += bh->b_size;
				if (ret >= pg_offset)
					break;
			} while ((bh = bh->b_this_page) != head);
		} else
			ret = mapped ? 0 : PAGE_CACHE_SIZE;
	}

	return ret;
}

STATIC size_t
xfs_probe_cluster(
	struct inode		*inode,
	struct page		*startpage,
	struct buffer_head	*bh,
	struct buffer_head	*head,
	int			mapped)
{
	struct pagevec		pvec;
	pgoff_t			tindex, tlast, tloff;
	size_t			total = 0;
	int			done = 0, i;

	/* First sum forwards in this page */
	do {
		if (!buffer_uptodate(bh) || (mapped != buffer_mapped(bh)))
			return total;
		total += bh->b_size;
	} while ((bh = bh->b_this_page) != head);

	/* if we reached the end of the page, sum forwards in following pages */
	tlast = i_size_read(inode) >> PAGE_CACHE_SHIFT;
	tindex = startpage->index + 1;

	/* Prune this back to avoid pathological behavior */
	tloff = min(tlast, startpage->index + 64);

	pagevec_init(&pvec, 0);
	while (!done && tindex <= tloff) {
		unsigned len = min_t(pgoff_t, PAGEVEC_SIZE, tlast - tindex + 1);

		if (!pagevec_lookup(&pvec, inode->i_mapping, tindex, len))
			break;

		for (i = 0; i < pagevec_count(&pvec); i++) {
			struct page *page = pvec.pages[i];
			size_t pg_offset, pg_len = 0;

			if (tindex == tlast) {
				pg_offset =
				    i_size_read(inode) & (PAGE_CACHE_SIZE - 1);
				if (!pg_offset) {
					done = 1;
					break;
				}
			} else
				pg_offset = PAGE_CACHE_SIZE;

			if (page->index == tindex && trylock_page(page)) {
				pg_len = xfs_probe_page(page, pg_offset, mapped);
				unlock_page(page);
			}

			if (!pg_len) {
				done = 1;
				break;
			}

			total += pg_len;
			tindex++;
		}

		pagevec_release(&pvec);
		cond_resched();
	}

	return total;
}

STATIC int
xfs_is_delayed_page(
	struct page		*page,
	unsigned int		type)
{
	if (PageWriteback(page))
		return 0;

	if (page->mapping && page_has_buffers(page)) {
		struct buffer_head	*bh, *head;
		int			acceptable = 0;

		bh = head = page_buffers(page);
		do {
			if (buffer_unwritten(bh))
				acceptable = (type == IO_UNWRITTEN);
			else if (buffer_delay(bh))
				acceptable = (type == IO_DELAY);
			else if (buffer_dirty(bh) && buffer_mapped(bh))
				acceptable = (type == IO_NEW);
			else
				break;
		} while ((bh = bh->b_this_page) != head);

		if (acceptable)
			return 1;
	}

	return 0;
}

STATIC int
xfs_convert_page(
	struct inode		*inode,
	struct page		*page,
	loff_t			tindex,
	struct xfs_bmbt_irec	*imap,
	xfs_ioend_t		**ioendp,
	struct writeback_control *wbc,
	int			startio,
	int			all_bh)
{
	struct buffer_head	*bh, *head;
	xfs_off_t		end_offset;
	unsigned long		p_offset;
	unsigned int		type;
	int			len, page_dirty;
	int			count = 0, done = 0, uptodate = 1;
 	xfs_off_t		offset = page_offset(page);

	if (page->index != tindex)
		goto fail;
	if (!trylock_page(page))
		goto fail;
	if (PageWriteback(page))
		goto fail_unlock_page;
	if (page->mapping != inode->i_mapping)
		goto fail_unlock_page;
	if (!xfs_is_delayed_page(page, (*ioendp)->io_type))
		goto fail_unlock_page;

	/*
	 * page_dirty is initially a count of buffers on the page before
	 * EOF and is decremented as we move each into a cleanable state.
	 *
	 * Derivation:
	 *
	 * End offset is the highest offset that this page should represent.
	 * If we are on the last page, (end_offset & (PAGE_CACHE_SIZE - 1))
	 * will evaluate non-zero and be less than PAGE_CACHE_SIZE and
	 * hence give us the correct page_dirty count. On any other page,
	 * it will be zero and in that case we need page_dirty to be the
	 * count of buffers on the page.
	 */
	end_offset = min_t(unsigned long long,
			(xfs_off_t)(page->index + 1) << PAGE_CACHE_SHIFT,
			i_size_read(inode));

	len = 1 << inode->i_blkbits;
	p_offset = min_t(unsigned long, end_offset & (PAGE_CACHE_SIZE - 1),
					PAGE_CACHE_SIZE);
	p_offset = p_offset ? roundup(p_offset, len) : PAGE_CACHE_SIZE;
	page_dirty = p_offset / len;

	bh = head = page_buffers(page);
	do {
		if (offset >= end_offset)
			break;
		if (!buffer_uptodate(bh))
			uptodate = 0;
		if (!(PageUptodate(page) || buffer_uptodate(bh))) {
			done = 1;
			continue;
		}

		if (buffer_unwritten(bh) || buffer_delay(bh)) {
			if (buffer_unwritten(bh))
				type = IO_UNWRITTEN;
			else
				type = IO_DELAY;

			if (!xfs_imap_valid(inode, imap, offset)) {
				done = 1;
				continue;
			}

			ASSERT(imap->br_startblock != HOLESTARTBLOCK);
			ASSERT(imap->br_startblock != DELAYSTARTBLOCK);

			xfs_map_at_offset(inode, bh, imap, offset);
			if (startio) {
				xfs_add_to_ioend(inode, bh, offset,
						type, ioendp, done);
			} else {
				set_buffer_dirty(bh);
				unlock_buffer(bh);
				mark_buffer_dirty(bh);
			}
			page_dirty--;
			count++;
		} else {
			type = IO_NEW;
			if (buffer_mapped(bh) && all_bh && startio) {
				lock_buffer(bh);
				xfs_add_to_ioend(inode, bh, offset,
						type, ioendp, done);
				count++;
				page_dirty--;
			} else {
				done = 1;
			}
		}
	} while (offset += len, (bh = bh->b_this_page) != head);

	if (uptodate && bh == head)
		SetPageUptodate(page);

	if (startio) {
		if (count) {
			wbc->nr_to_write--;
			if (wbc->nr_to_write <= 0)
				done = 1;
		}
		xfs_start_page_writeback(page, !page_dirty, count);
	}

	return done;
 fail_unlock_page:
	unlock_page(page);
 fail:
	return 1;
}

STATIC void
xfs_cluster_write(
	struct inode		*inode,
	pgoff_t			tindex,
	struct xfs_bmbt_irec	*imap,
	xfs_ioend_t		**ioendp,
	struct writeback_control *wbc,
	int			startio,
	int			all_bh,
	pgoff_t			tlast)
{
	struct pagevec		pvec;
	int			done = 0, i;

	pagevec_init(&pvec, 0);
	while (!done && tindex <= tlast) {
		unsigned len = min_t(pgoff_t, PAGEVEC_SIZE, tlast - tindex + 1);

		if (!pagevec_lookup(&pvec, inode->i_mapping, tindex, len))
			break;

		for (i = 0; i < pagevec_count(&pvec); i++) {
			done = xfs_convert_page(inode, pvec.pages[i], tindex++,
					imap, ioendp, wbc, startio, all_bh);
			if (done)
				break;
		}

		pagevec_release(&pvec);
		cond_resched();
	}
}

STATIC void
xfs_vm_invalidatepage(
	struct page		*page,
	unsigned long		offset)
{
	trace_xfs_invalidatepage(page->mapping->host, page, offset);
	block_invalidatepage(page, offset);
}

STATIC void
xfs_aops_discard_page(
	struct page		*page)
{
	struct inode		*inode = page->mapping->host;
	struct xfs_inode	*ip = XFS_I(inode);
	struct buffer_head	*bh, *head;
	loff_t			offset = page_offset(page);
	ssize_t			len = 1 << inode->i_blkbits;

	if (!xfs_is_delayed_page(page, IO_DELAY))
		goto out_invalidate;

	if (XFS_FORCED_SHUTDOWN(ip->i_mount))
		goto out_invalidate;

	xfs_fs_cmn_err(CE_ALERT, ip->i_mount,
		"page discard on page %p, inode 0x%llx, offset %llu.",
			page, ip->i_ino, offset);

	xfs_ilock(ip, XFS_ILOCK_EXCL);
	bh = head = page_buffers(page);
	do {
		int		done;
		xfs_fileoff_t	offset_fsb;
		xfs_bmbt_irec_t	imap;
		int		nimaps = 1;
		int		error;
		xfs_fsblock_t	firstblock;
		xfs_bmap_free_t flist;

		if (!buffer_delay(bh))
			goto next_buffer;

		offset_fsb = XFS_B_TO_FSBT(ip->i_mount, offset);

		/*
		 * Map the range first and check that it is a delalloc extent
		 * before trying to unmap the range. Otherwise we will be
		 * trying to remove a real extent (which requires a
		 * transaction) or a hole, which is probably a bad idea...
		 */
		error = xfs_bmapi(NULL, ip, offset_fsb, 1,
				XFS_BMAPI_ENTIRE,  NULL, 0, &imap,
				&nimaps, NULL, NULL);

		if (error) {
			/* something screwed, just bail */
			if (!XFS_FORCED_SHUTDOWN(ip->i_mount)) {
				xfs_fs_cmn_err(CE_ALERT, ip->i_mount,
				"page discard failed delalloc mapping lookup.");
			}
			break;
		}
		if (!nimaps) {
			/* nothing there */
			goto next_buffer;
		}
		if (imap.br_startblock != DELAYSTARTBLOCK) {
			/* been converted, ignore */
			goto next_buffer;
		}
		WARN_ON(imap.br_blockcount == 0);

		/*
		 * Note: while we initialise the firstblock/flist pair, they
		 * should never be used because blocks should never be
		 * allocated or freed for a delalloc extent and hence we need
		 * don't cancel or finish them after the xfs_bunmapi() call.
		 */
		xfs_bmap_init(&flist, &firstblock);
		error = xfs_bunmapi(NULL, ip, offset_fsb, 1, 0, 1, &firstblock,
					&flist, NULL, &done);

		ASSERT(!flist.xbf_count && !flist.xbf_first);
		if (error) {
			/* something screwed, just bail */
			if (!XFS_FORCED_SHUTDOWN(ip->i_mount)) {
				xfs_fs_cmn_err(CE_ALERT, ip->i_mount,
			"page discard unable to remove delalloc mapping.");
			}
			break;
		}
next_buffer:
		offset += len;

	} while ((bh = bh->b_this_page) != head);

	xfs_iunlock(ip, XFS_ILOCK_EXCL);
out_invalidate:
	xfs_vm_invalidatepage(page, 0);
	return;
}


STATIC int
xfs_page_state_convert(
	struct inode	*inode,
	struct page	*page,
	struct writeback_control *wbc,
	int		startio,
	int		unmapped) /* also implies page uptodate */
{
	struct buffer_head	*bh, *head;
	struct xfs_bmbt_irec	imap;
	xfs_ioend_t		*ioend = NULL, *iohead = NULL;
	loff_t			offset;
	unsigned long           p_offset = 0;
	unsigned int		type;
	__uint64_t              end_offset;
	pgoff_t                 end_index, last_index;
	ssize_t			size, len;
	int			flags, err, imap_valid = 0, uptodate = 1;
	int			page_dirty, count = 0;
	int			trylock = 0;
	int			all_bh = unmapped;

	if (startio) {
		if (wbc->sync_mode == WB_SYNC_NONE && wbc->nonblocking)
			trylock |= BMAPI_TRYLOCK;
	}

	/* Is this page beyond the end of the file? */
	offset = i_size_read(inode);
	end_index = offset >> PAGE_CACHE_SHIFT;
	last_index = (offset - 1) >> PAGE_CACHE_SHIFT;
	if (page->index >= end_index) {
		if ((page->index >= end_index + 1) ||
		    !(i_size_read(inode) & (PAGE_CACHE_SIZE - 1))) {
			if (startio)
				unlock_page(page);
			return 0;
		}
	}

	/*
	 * page_dirty is initially a count of buffers on the page before
	 * EOF and is decremented as we move each into a cleanable state.
	 *
	 * Derivation:
	 *
	 * End offset is the highest offset that this page should represent.
	 * If we are on the last page, (end_offset & (PAGE_CACHE_SIZE - 1))
	 * will evaluate non-zero and be less than PAGE_CACHE_SIZE and
	 * hence give us the correct page_dirty count. On any other page,
	 * it will be zero and in that case we need page_dirty to be the
	 * count of buffers on the page.
 	 */
	end_offset = min_t(unsigned long long,
			(xfs_off_t)(page->index + 1) << PAGE_CACHE_SHIFT, offset);
	len = 1 << inode->i_blkbits;
	p_offset = min_t(unsigned long, end_offset & (PAGE_CACHE_SIZE - 1),
					PAGE_CACHE_SIZE);
	p_offset = p_offset ? roundup(p_offset, len) : PAGE_CACHE_SIZE;
	page_dirty = p_offset / len;

	bh = head = page_buffers(page);
	offset = page_offset(page);
	flags = BMAPI_READ;
	type = IO_NEW;

	/* TODO: cleanup count and page_dirty */

	do {
		if (offset >= end_offset)
			break;
		if (!buffer_uptodate(bh))
			uptodate = 0;
		if (!(PageUptodate(page) || buffer_uptodate(bh)) && !startio) {
			/*
			 * the iomap is actually still valid, but the ioend
			 * isn't.  shouldn't happen too often.
			 */
			imap_valid = 0;
			continue;
		}

		if (imap_valid)
			imap_valid = xfs_imap_valid(inode, &imap, offset);

		/*
		 * First case, map an unwritten extent and prepare for
		 * extent state conversion transaction on completion.
		 *
		 * Second case, allocate space for a delalloc buffer.
		 * We can return EAGAIN here in the release page case.
		 *
		 * Third case, an unmapped buffer was found, and we are
		 * in a path where we need to write the whole page out.
		 */
		if (buffer_unwritten(bh) || buffer_delay(bh) ||
		    ((buffer_uptodate(bh) || PageUptodate(page)) &&
		     !buffer_mapped(bh) && (unmapped || startio))) {
			int new_ioend = 0;

			/*
			 * Make sure we don't use a read-only iomap
			 */
			if (flags == BMAPI_READ)
				imap_valid = 0;

			if (buffer_unwritten(bh)) {
				type = IO_UNWRITTEN;
				flags = BMAPI_WRITE | BMAPI_IGNSTATE;
			} else if (buffer_delay(bh)) {
				type = IO_DELAY;
				flags = BMAPI_ALLOCATE | trylock;
			} else {
				type = IO_NEW;
				flags = BMAPI_WRITE | BMAPI_MMAP;
			}

			if (!imap_valid) {
				/*
				 * if we didn't have a valid mapping then we
				 * need to ensure that we put the new mapping
				 * in a new ioend structure. This needs to be
				 * done to ensure that the ioends correctly
				 * reflect the block mappings at io completion
				 * for unwritten extent conversion.
				 */
				new_ioend = 1;
				if (type == IO_NEW) {
					size = xfs_probe_cluster(inode,
							page, bh, head, 0);
				} else {
					size = len;
				}

				err = xfs_map_blocks(inode, offset, size,
						&imap, flags);
				if (err)
					goto error;
				imap_valid = xfs_imap_valid(inode, &imap,
							    offset);
			}
			if (imap_valid) {
				xfs_map_at_offset(inode, bh, &imap, offset);
				if (startio) {
					xfs_add_to_ioend(inode, bh, offset,
							type, &ioend,
							new_ioend);
				} else {
					set_buffer_dirty(bh);
					unlock_buffer(bh);
					mark_buffer_dirty(bh);
				}
				page_dirty--;
				count++;
			}
		} else if (buffer_uptodate(bh) && startio) {
			/*
			 * we got here because the buffer is already mapped.
			 * That means it must already have extents allocated
			 * underneath it. Map the extent by reading it.
			 */
			if (!imap_valid || flags != BMAPI_READ) {
				flags = BMAPI_READ;
				size = xfs_probe_cluster(inode, page, bh,
								head, 1);
				err = xfs_map_blocks(inode, offset, size,
						&imap, flags);
				if (err)
					goto error;
				imap_valid = xfs_imap_valid(inode, &imap,
							    offset);
			}

			/*
			 * We set the type to IO_NEW in case we are doing a
			 * small write at EOF that is extending the file but
			 * without needing an allocation. We need to update the
			 * file size on I/O completion in this case so it is
			 * the same case as having just allocated a new extent
			 * that we are writing into for the first time.
			 */
			type = IO_NEW;
			if (trylock_buffer(bh)) {
				ASSERT(buffer_mapped(bh));
				if (imap_valid)
					all_bh = 1;
				xfs_add_to_ioend(inode, bh, offset, type,
						&ioend, !imap_valid);
				page_dirty--;
				count++;
			} else {
				imap_valid = 0;
			}
		} else if ((buffer_uptodate(bh) || PageUptodate(page)) &&
			   (unmapped || startio)) {
			imap_valid = 0;
		}

		if (!iohead)
			iohead = ioend;

	} while (offset += len, ((bh = bh->b_this_page) != head));

	if (uptodate && bh == head)
		SetPageUptodate(page);

	if (startio)
		xfs_start_page_writeback(page, 1, count);

	if (ioend && imap_valid) {
		xfs_off_t		end_index;

		end_index = imap.br_startoff + imap.br_blockcount;

		/* to bytes */
		end_index <<= inode->i_blkbits;

		/* to pages */
		end_index = (end_index - 1) >> PAGE_CACHE_SHIFT;

		/* check against file size */
		if (end_index > last_index)
			end_index = last_index;

		xfs_cluster_write(inode, page->index + 1, &imap, &ioend,
					wbc, startio, all_bh, end_index);
	}

	if (iohead)
		xfs_submit_ioend(wbc, iohead);

	return page_dirty;

error:
	if (iohead)
		xfs_cancel_ioend(iohead);

	/*
	 * If it's delalloc and we have nowhere to put it,
	 * throw it away, unless the lower layers told
	 * us to try again.
	 */
	if (err != -EAGAIN) {
		if (!unmapped)
			xfs_aops_discard_page(page);
		ClearPageUptodate(page);
	}
	return err;
}


STATIC int
xfs_vm_writepage(
	struct page		*page,
	struct writeback_control *wbc)
{
	int			error;
	int			need_trans;
	int			delalloc, unmapped, unwritten;
	struct inode		*inode = page->mapping->host;

	trace_xfs_writepage(inode, page, 0);

	/*
	 * Refuse to write the page out if we are called from reclaim context.
	 *
	 * This is primarily to avoid stack overflows when called from deep
	 * used stacks in random callers for direct reclaim, but disabling
	 * reclaim for kswap is a nice side-effect as kswapd causes rather
	 * suboptimal I/O patters, too.
	 *
	 * This should really be done by the core VM, but until that happens
	 * filesystems like XFS, btrfs and ext4 have to take care of this
	 * by themselves.
	 */
	if (current->flags & PF_MEMALLOC)
		goto out_fail;

	/*
	 * We need a transaction if:
	 *  1. There are delalloc buffers on the page
	 *  2. The page is uptodate and we have unmapped buffers
	 *  3. The page is uptodate and we have no buffers
	 *  4. There are unwritten buffers on the page
	 */

	if (!page_has_buffers(page)) {
		unmapped = 1;
		need_trans = 1;
	} else {
		xfs_count_page_state(page, &delalloc, &unmapped, &unwritten);
		if (!PageUptodate(page))
			unmapped = 0;
		need_trans = delalloc + unmapped + unwritten;
	}

	/*
	 * If we need a transaction and the process flags say
	 * we are already in a transaction, or no IO is allowed
	 * then mark the page dirty again and leave the page
	 * as is.
	 */
	if (current_test_flags(PF_FSTRANS) && need_trans)
		goto out_fail;

	/*
	 * Delay hooking up buffer heads until we have
	 * made our go/no-go decision.
	 */
	if (!page_has_buffers(page))
		create_empty_buffers(page, 1 << inode->i_blkbits, 0);

	/*
	 * Convert delayed allocate, unwritten or unmapped space
	 * to real space and flush out to disk.
	 */
	error = xfs_page_state_convert(inode, page, wbc, 1, unmapped);
	if (error == -EAGAIN)
		goto out_fail;
	if (unlikely(error < 0))
		goto out_unlock;

	return 0;

out_fail:
	redirty_page_for_writepage(wbc, page);
	unlock_page(page);
	return 0;
out_unlock:
	unlock_page(page);
	return error;
}

STATIC int
xfs_vm_writepages(
	struct address_space	*mapping,
	struct writeback_control *wbc)
{
	xfs_iflags_clear(XFS_I(mapping->host), XFS_ITRUNCATED);
	return generic_writepages(mapping, wbc);
}

STATIC int
xfs_vm_releasepage(
	struct page		*page,
	gfp_t			gfp_mask)
{
	struct inode		*inode = page->mapping->host;
	int			dirty, delalloc, unmapped, unwritten;
	struct writeback_control wbc = {
		.sync_mode = WB_SYNC_ALL,
		.nr_to_write = 1,
	};

	trace_xfs_releasepage(inode, page, 0);

	if (!page_has_buffers(page))
		return 0;

	xfs_count_page_state(page, &delalloc, &unmapped, &unwritten);
	if (!delalloc && !unwritten)
		goto free_buffers;

	if (!(gfp_mask & __GFP_FS))
		return 0;

	/* If we are already inside a transaction or the thread cannot
	 * do I/O, we cannot release this page.
	 */
	if (current_test_flags(PF_FSTRANS))
		return 0;

	/*
	 * Convert delalloc space to real space, do not flush the
	 * data out to disk, that will be done by the caller.
	 * Never need to allocate space here - we will always
	 * come back to writepage in that case.
	 */
	dirty = xfs_page_state_convert(inode, page, &wbc, 0, 0);
	if (dirty == 0 && !unwritten)
		goto free_buffers;
	return 0;

free_buffers:
	return try_to_free_buffers(page);
}

STATIC int
__xfs_get_blocks(
	struct inode		*inode,
	sector_t		iblock,
	struct buffer_head	*bh_result,
	int			create,
	int			direct,
	bmapi_flags_t		flags)
{
	struct xfs_bmbt_irec	imap;
	xfs_off_t		offset;
	ssize_t			size;
	int			nimap = 1;
	int			new = 0;
	int			error;

	offset = (xfs_off_t)iblock << inode->i_blkbits;
	ASSERT(bh_result->b_size >= (1 << inode->i_blkbits));
	size = bh_result->b_size;

	if (!create && direct && offset >= i_size_read(inode))
		return 0;

	error = xfs_iomap(XFS_I(inode), offset, size,
			     create ? flags : BMAPI_READ, &imap, &nimap, &new);
	if (error)
		return -error;
	if (nimap == 0)
		return 0;

	if (imap.br_startblock != HOLESTARTBLOCK &&
	    imap.br_startblock != DELAYSTARTBLOCK) {
		/*
		 * For unwritten extents do not report a disk address on
		 * the read case (treat as if we're reading into a hole).
		 */
		if (create || !ISUNWRITTEN(&imap))
			xfs_map_buffer(inode, bh_result, &imap, offset);
		if (create && ISUNWRITTEN(&imap)) {
			if (direct)
				bh_result->b_private = inode;
			set_buffer_unwritten(bh_result);
		}
	}

	/*
	 * If this is a realtime file, data may be on a different device.
	 * to that pointed to from the buffer_head b_bdev currently.
	 */
	bh_result->b_bdev = xfs_find_bdev_for_inode(inode);

	/*
	 * If we previously allocated a block out beyond eof and we are now
	 * coming back to use it then we will need to flag it as new even if it
	 * has a disk address.
	 *
	 * With sub-block writes into unwritten extents we also need to mark
	 * the buffer as new so that the unwritten parts of the buffer gets
	 * correctly zeroed.
	 */
	if (create &&
	    ((!buffer_mapped(bh_result) && !buffer_uptodate(bh_result)) ||
	     (offset >= i_size_read(inode)) ||
	     (new || ISUNWRITTEN(&imap))))
		set_buffer_new(bh_result);

	if (imap.br_startblock == DELAYSTARTBLOCK) {
		BUG_ON(direct);
		if (create) {
			set_buffer_uptodate(bh_result);
			set_buffer_mapped(bh_result);
			set_buffer_delay(bh_result);
		}
	}

	/*
	 * If this is O_DIRECT or the mpage code calling tell them how large
	 * the mapping is, so that we can avoid repeated get_blocks calls.
	 */
	if (direct || size > (1 << inode->i_blkbits)) {
		xfs_off_t		mapping_size;

		mapping_size = imap.br_startoff + imap.br_blockcount - iblock;
		mapping_size <<= inode->i_blkbits;

		ASSERT(mapping_size > 0);
		if (mapping_size > size)
			mapping_size = size;
		if (mapping_size > LONG_MAX)
			mapping_size = LONG_MAX;

		bh_result->b_size = mapping_size;
	}

	return 0;
}

int
xfs_get_blocks(
	struct inode		*inode,
	sector_t		iblock,
	struct buffer_head	*bh_result,
	int			create)
{
	return __xfs_get_blocks(inode, iblock,
				bh_result, create, 0, BMAPI_WRITE);
}

STATIC int
xfs_get_blocks_direct(
	struct inode		*inode,
	sector_t		iblock,
	struct buffer_head	*bh_result,
	int			create)
{
	return __xfs_get_blocks(inode, iblock,
				bh_result, create, 1, BMAPI_WRITE|BMAPI_DIRECT);
}

STATIC void
xfs_end_io_direct(
	struct kiocb	*iocb,
	loff_t		offset,
	ssize_t		size,
	void		*private,
	int		ret,
	bool		is_async)
{
	xfs_ioend_t	*ioend = iocb->private;
	bool		complete_aio = is_async;

	/*
	 * Non-NULL private data means we need to issue a transaction to
	 * convert a range from unwritten to written extents.  This needs
	 * to happen from process context but aio+dio I/O completion
	 * happens from irq context so we need to defer it to a workqueue.
	 * This is not necessary for synchronous direct I/O, but we do
	 * it anyway to keep the code uniform and simpler.
	 *
	 * Well, if only it were that simple. Because synchronous direct I/O
	 * requires extent conversion to occur *before* we return to userspace,
	 * we have to wait for extent conversion to complete. Look at the
	 * iocb that has been passed to us to determine if this is AIO or
	 * not. If it is synchronous, tell xfs_finish_ioend() to kick the
	 * workqueue and wait for it to complete.
	 *
	 * The core direct I/O code might be changed to always call the
	 * completion handler in the future, in which case all this can
	 * go away.
	 */
	ioend->io_offset = offset;
	ioend->io_size = size;
	if (ioend->io_type == IO_READ) {
		xfs_finish_ioend(ioend, 0);
	} else if (private && size > 0) {
		if (is_async) {
			ioend->io_iocb = iocb;
			ioend->io_result = ret;
			complete_aio = false;
			xfs_finish_ioend(ioend, 0);
		} else {
			xfs_finish_ioend(ioend, 1);
		}
	} else {
		/*
		 * A direct I/O write ioend starts it's life in unwritten
		 * state in case they map an unwritten extent.  This write
		 * didn't map an unwritten extent so switch it's completion
		 * handler.
		 */
		ioend->io_type = IO_NEW;
		xfs_finish_ioend(ioend, 0);
	}

	/*
	 * blockdev_direct_IO can return an error even after the I/O
	 * completion handler was called.  Thus we need to protect
	 * against double-freeing.
	 */
	iocb->private = NULL;

	if (complete_aio)
		aio_complete(iocb, ret, 0);
}

STATIC ssize_t
xfs_vm_direct_IO(
	int			rw,
	struct kiocb		*iocb,
	const struct iovec	*iov,
	loff_t			offset,
	unsigned long		nr_segs)
{
	struct file	*file = iocb->ki_filp;
	struct inode	*inode = file->f_mapping->host;
	struct block_device *bdev;
	ssize_t		ret;

	bdev = xfs_find_bdev_for_inode(inode);

	iocb->private = xfs_alloc_ioend(inode, rw == WRITE ?
					IO_UNWRITTEN : IO_READ);

	ret = blockdev_direct_IO_no_locking(rw, iocb, inode, bdev, iov,
					    offset, nr_segs,
					    xfs_get_blocks_direct,
					    xfs_end_io_direct);

	if (unlikely(ret != -EIOCBQUEUED && iocb->private))
		xfs_destroy_ioend(iocb->private);
	return ret;
}

STATIC int
xfs_vm_write_begin(
	struct file		*file,
	struct address_space	*mapping,
	loff_t			pos,
	unsigned		len,
	unsigned		flags,
	struct page		**pagep,
	void			**fsdata)
{
	*pagep = NULL;
	return block_write_begin(file, mapping, pos, len, flags, pagep, fsdata,
								xfs_get_blocks);
}

STATIC sector_t
xfs_vm_bmap(
	struct address_space	*mapping,
	sector_t		block)
{
	struct inode		*inode = (struct inode *)mapping->host;
	struct xfs_inode	*ip = XFS_I(inode);

	xfs_itrace_entry(XFS_I(inode));
	xfs_ilock(ip, XFS_IOLOCK_SHARED);
	xfs_flush_pages(ip, (xfs_off_t)0, -1, 0, FI_REMAPF);
	xfs_iunlock(ip, XFS_IOLOCK_SHARED);
	return generic_block_bmap(mapping, block, xfs_get_blocks);
}

STATIC int
xfs_vm_readpage(
	struct file		*unused,
	struct page		*page)
{
	return mpage_readpage(page, xfs_get_blocks);
}

STATIC int
xfs_vm_readpages(
	struct file		*unused,
	struct address_space	*mapping,
	struct list_head	*pages,
	unsigned		nr_pages)
{
	return mpage_readpages(mapping, pages, nr_pages, xfs_get_blocks);
}

const struct address_space_operations xfs_address_space_operations = {
	.readpage		= xfs_vm_readpage,
	.readpages		= xfs_vm_readpages,
	.writepage		= xfs_vm_writepage,
	.writepages		= xfs_vm_writepages,
	.sync_page		= block_sync_page,
	.releasepage		= xfs_vm_releasepage,
	.invalidatepage		= xfs_vm_invalidatepage,
	.write_begin		= xfs_vm_write_begin,
	.write_end		= generic_write_end,
	.bmap			= xfs_vm_bmap,
	.direct_IO		= xfs_vm_direct_IO,
	.migratepage		= buffer_migrate_page,
	.is_partially_uptodate  = block_is_partially_uptodate,
	.error_remove_page	= generic_error_remove_page,
};

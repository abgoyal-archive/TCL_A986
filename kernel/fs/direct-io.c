

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/bio.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>
#include <linux/rwsem.h>
#include <linux/uio.h>
#include <asm/atomic.h>

#define DIO_PAGES	64


struct dio {
	/* BIO submission state */
	struct bio *bio;		/* bio under assembly */
	struct inode *inode;
	int rw;
	loff_t i_size;			/* i_size when submitted */
	int flags;			/* doesn't change */
	unsigned blkbits;		/* doesn't change */
	unsigned blkfactor;		/* When we're using an alignment which
					   is finer than the filesystem's soft
					   blocksize, this specifies how much
					   finer.  blkfactor=2 means 1/4-block
					   alignment.  Does not change */
	unsigned start_zero_done;	/* flag: sub-blocksize zeroing has
					   been performed at the start of a
					   write */
	int pages_in_io;		/* approximate total IO pages */
	size_t	size;			/* total request size (doesn't change)*/
	sector_t block_in_file;		/* Current offset into the underlying
					   file in dio_block units. */
	unsigned blocks_available;	/* At block_in_file.  changes */
	sector_t final_block_in_request;/* doesn't change */
	unsigned first_block_in_page;	/* doesn't change, Used only once */
	int boundary;			/* prev block is at a boundary */
	int reap_counter;		/* rate limit reaping */
	get_block_t *get_block;		/* block mapping function */
	dio_iodone_t *end_io;		/* IO completion function */
	dio_submit_t *submit_io;	/* IO submition function */
	loff_t logical_offset_in_bio;	/* current first logical block in bio */
	sector_t final_block_in_bio;	/* current final block in bio + 1 */
	sector_t next_block_for_io;	/* next block to be put under IO,
					   in dio_blocks units */
	struct buffer_head map_bh;	/* last get_block() result */

	/*
	 * Deferred addition of a page to the dio.  These variables are
	 * private to dio_send_cur_page(), submit_page_section() and
	 * dio_bio_add_page().
	 */
	struct page *cur_page;		/* The page */
	unsigned cur_page_offset;	/* Offset into it, in bytes */
	unsigned cur_page_len;		/* Nr of bytes at cur_page_offset */
	sector_t cur_page_block;	/* Where it starts */
	loff_t cur_page_fs_offset;	/* Offset in file */

	/* BIO completion state */
	spinlock_t bio_lock;		/* protects BIO fields below */
	unsigned long refcount;		/* direct_io_worker() and bios */
	struct bio *bio_list;		/* singly linked via bi_private */
	struct task_struct *waiter;	/* waiting task (NULL if none) */

	/* AIO related stuff */
	struct kiocb *iocb;		/* kiocb */
	int is_async;			/* is IO async ? */
	int io_error;			/* IO error in completion path */
	ssize_t result;                 /* IO result */

	/*
	 * Page fetching state. These variables belong to dio_refill_pages().
	 */
	int curr_page;			/* changes */
	int total_pages;		/* doesn't change */
	unsigned long curr_user_address;/* changes */

	/*
	 * Page queue.  These variables belong to dio_refill_pages() and
	 * dio_get_page().
	 */
	unsigned head;			/* next page to process */
	unsigned tail;			/* last valid page + 1 */
	int page_errors;		/* errno from get_user_pages() */

	/*
	 * pages[] (and any fields placed after it) are not zeroed out at
	 * allocation time.  Don't add new fields after pages[] unless you
	 * wish that they not be zeroed.
	 */
	struct page *pages[DIO_PAGES];	/* page buffer */
};

static inline unsigned dio_pages_present(struct dio *dio)
{
	return dio->tail - dio->head;
}

static int dio_refill_pages(struct dio *dio)
{
	int ret;
	int nr_pages;

	nr_pages = min(dio->total_pages - dio->curr_page, DIO_PAGES);
	ret = get_user_pages_fast(
		dio->curr_user_address,		/* Where from? */
		nr_pages,			/* How many pages? */
		dio->rw == READ,		/* Write to memory? */
		&dio->pages[0]);		/* Put results here */

	if (ret < 0 && dio->blocks_available && (dio->rw & WRITE)) {
		struct page *page = ZERO_PAGE(0);
		/*
		 * A memory fault, but the filesystem has some outstanding
		 * mapped blocks.  We need to use those blocks up to avoid
		 * leaking stale data in the file.
		 */
		if (dio->page_errors == 0)
			dio->page_errors = ret;
		page_cache_get(page);
		dio->pages[0] = page;
		dio->head = 0;
		dio->tail = 1;
		ret = 0;
		goto out;
	}

	if (ret >= 0) {
		dio->curr_user_address += ret * PAGE_SIZE;
		dio->curr_page += ret;
		dio->head = 0;
		dio->tail = ret;
		ret = 0;
	}
out:
	return ret;	
}

static struct page *dio_get_page(struct dio *dio)
{
	if (dio_pages_present(dio) == 0) {
		int ret;

		ret = dio_refill_pages(dio);
		if (ret)
			return ERR_PTR(ret);
		BUG_ON(dio_pages_present(dio) == 0);
	}
	return dio->pages[dio->head++];
}

static int dio_complete(struct dio *dio, loff_t offset, int ret, bool is_async)
{
	ssize_t transferred = 0;

	/*
	 * AIO submission can race with bio completion to get here while
	 * expecting to have the last io completed by bio completion.
	 * In that case -EIOCBQUEUED is in fact not an error we want
	 * to preserve through this call.
	 */
	if (ret == -EIOCBQUEUED)
		ret = 0;

	if (dio->result) {
		transferred = dio->result;

		/* Check for short read case */
		if ((dio->rw == READ) && ((offset + transferred) > dio->i_size))
			transferred = dio->i_size - offset;
	}

	if (ret == 0)
		ret = dio->page_errors;
	if (ret == 0)
		ret = dio->io_error;
	if (ret == 0)
		ret = transferred;

	if (dio->end_io && dio->result) {
		dio->end_io(dio->iocb, offset, transferred,
			    dio->map_bh.b_private, ret, is_async);
	} else if (is_async) {
		aio_complete(dio->iocb, ret, 0);
	}

	if (dio->flags & DIO_LOCKING)
		/* lockdep: non-owner release */
		up_read_non_owner(&dio->inode->i_alloc_sem);

	return ret;
}

static int dio_bio_complete(struct dio *dio, struct bio *bio);
static void dio_bio_end_aio(struct bio *bio, int error)
{
	struct dio *dio = bio->bi_private;
	unsigned long remaining;
	unsigned long flags;

	/* cleanup the bio */
	dio_bio_complete(dio, bio);

	spin_lock_irqsave(&dio->bio_lock, flags);
	remaining = --dio->refcount;
	if (remaining == 1 && dio->waiter)
		wake_up_process(dio->waiter);
	spin_unlock_irqrestore(&dio->bio_lock, flags);

	if (remaining == 0) {
		dio_complete(dio, dio->iocb->ki_pos, 0, true);
		kfree(dio);
	}
}

static void dio_bio_end_io(struct bio *bio, int error)
{
	struct dio *dio = bio->bi_private;
	unsigned long flags;

	spin_lock_irqsave(&dio->bio_lock, flags);
	bio->bi_private = dio->bio_list;
	dio->bio_list = bio;
	if (--dio->refcount == 1 && dio->waiter)
		wake_up_process(dio->waiter);
	spin_unlock_irqrestore(&dio->bio_lock, flags);
}

void dio_end_io(struct bio *bio, int error)
{
	struct dio *dio = bio->bi_private;

	if (dio->is_async)
		dio_bio_end_aio(bio, error);
	else
		dio_bio_end_io(bio, error);
}
EXPORT_SYMBOL_GPL(dio_end_io);

static int
dio_bio_alloc(struct dio *dio, struct block_device *bdev,
		sector_t first_sector, int nr_vecs)
{
	struct bio *bio;

	bio = bio_alloc(GFP_KERNEL, nr_vecs);

	bio->bi_bdev = bdev;
	bio->bi_sector = first_sector;
	if (dio->is_async)
		bio->bi_end_io = dio_bio_end_aio;
	else
		bio->bi_end_io = dio_bio_end_io;

	dio->bio = bio;
	dio->logical_offset_in_bio = dio->cur_page_fs_offset;
	return 0;
}

static void dio_bio_submit(struct dio *dio)
{
	struct bio *bio = dio->bio;
	unsigned long flags;

	bio->bi_private = dio;

	spin_lock_irqsave(&dio->bio_lock, flags);
	dio->refcount++;
	spin_unlock_irqrestore(&dio->bio_lock, flags);

	if (dio->is_async && dio->rw == READ)
		bio_set_pages_dirty(bio);

	if (dio->submit_io)
		dio->submit_io(dio->rw, bio, dio->inode,
			       dio->logical_offset_in_bio);
	else
		submit_bio(dio->rw, bio);

	dio->bio = NULL;
	dio->boundary = 0;
	dio->logical_offset_in_bio = 0;
}

static void dio_cleanup(struct dio *dio)
{
	while (dio_pages_present(dio))
		page_cache_release(dio_get_page(dio));
}

static struct bio *dio_await_one(struct dio *dio)
{
	unsigned long flags;
	struct bio *bio = NULL;

	spin_lock_irqsave(&dio->bio_lock, flags);

	/*
	 * Wait as long as the list is empty and there are bios in flight.  bio
	 * completion drops the count, maybe adds to the list, and wakes while
	 * holding the bio_lock so we don't need set_current_state()'s barrier
	 * and can call it after testing our condition.
	 */
	while (dio->refcount > 1 && dio->bio_list == NULL) {
		__set_current_state(TASK_UNINTERRUPTIBLE);
		dio->waiter = current;
		spin_unlock_irqrestore(&dio->bio_lock, flags);
		io_schedule();
		/* wake up sets us TASK_RUNNING */
		spin_lock_irqsave(&dio->bio_lock, flags);
		dio->waiter = NULL;
	}
	if (dio->bio_list) {
		bio = dio->bio_list;
		dio->bio_list = bio->bi_private;
	}
	spin_unlock_irqrestore(&dio->bio_lock, flags);
	return bio;
}

static int dio_bio_complete(struct dio *dio, struct bio *bio)
{
	const int uptodate = test_bit(BIO_UPTODATE, &bio->bi_flags);
	struct bio_vec *bvec = bio->bi_io_vec;
	int page_no;

	if (!uptodate)
		dio->io_error = -EIO;

	if (dio->is_async && dio->rw == READ) {
		bio_check_pages_dirty(bio);	/* transfers ownership */
	} else {
		for (page_no = 0; page_no < bio->bi_vcnt; page_no++) {
			struct page *page = bvec[page_no].bv_page;

			if (dio->rw == READ && !PageCompound(page))
				set_page_dirty_lock(page);
			page_cache_release(page);
		}
		bio_put(bio);
	}
	return uptodate ? 0 : -EIO;
}

static void dio_await_completion(struct dio *dio)
{
	struct bio *bio;
	do {
		bio = dio_await_one(dio);
		if (bio)
			dio_bio_complete(dio, bio);
	} while (bio);
}

static int dio_bio_reap(struct dio *dio)
{
	int ret = 0;

	if (dio->reap_counter++ >= 64) {
		while (dio->bio_list) {
			unsigned long flags;
			struct bio *bio;
			int ret2;

			spin_lock_irqsave(&dio->bio_lock, flags);
			bio = dio->bio_list;
			dio->bio_list = bio->bi_private;
			spin_unlock_irqrestore(&dio->bio_lock, flags);
			ret2 = dio_bio_complete(dio, bio);
			if (ret == 0)
				ret = ret2;
		}
		dio->reap_counter = 0;
	}
	return ret;
}

static int get_more_blocks(struct dio *dio)
{
	int ret;
	struct buffer_head *map_bh = &dio->map_bh;
	sector_t fs_startblk;	/* Into file, in filesystem-sized blocks */
	unsigned long fs_count;	/* Number of filesystem-sized blocks */
	unsigned long dio_count;/* Number of dio_block-sized blocks */
	unsigned long blkmask;
	int create;

	/*
	 * If there was a memory error and we've overwritten all the
	 * mapped blocks then we can now return that memory error
	 */
	ret = dio->page_errors;
	if (ret == 0) {
		BUG_ON(dio->block_in_file >= dio->final_block_in_request);
		fs_startblk = dio->block_in_file >> dio->blkfactor;
		dio_count = dio->final_block_in_request - dio->block_in_file;
		fs_count = dio_count >> dio->blkfactor;
		blkmask = (1 << dio->blkfactor) - 1;
		if (dio_count & blkmask)	
			fs_count++;

		map_bh->b_state = 0;
		map_bh->b_size = fs_count << dio->inode->i_blkbits;

		/*
		 * For writes inside i_size on a DIO_SKIP_HOLES filesystem we
		 * forbid block creations: only overwrites are permitted.
		 * We will return early to the caller once we see an
		 * unmapped buffer head returned, and the caller will fall
		 * back to buffered I/O.
		 *
		 * Otherwise the decision is left to the get_blocks method,
		 * which may decide to handle it or also return an unmapped
		 * buffer head.
		 */
		create = dio->rw & WRITE;
		if (dio->flags & DIO_SKIP_HOLES) {
			if (dio->block_in_file < (i_size_read(dio->inode) >>
							dio->blkbits))
				create = 0;
		}

		ret = (*dio->get_block)(dio->inode, fs_startblk,
						map_bh, create);
	}
	return ret;
}

static int dio_new_bio(struct dio *dio, sector_t start_sector)
{
	sector_t sector;
	int ret, nr_pages;

	ret = dio_bio_reap(dio);
	if (ret)
		goto out;
	sector = start_sector << (dio->blkbits - 9);
	nr_pages = min(dio->pages_in_io, bio_get_nr_vecs(dio->map_bh.b_bdev));
	BUG_ON(nr_pages <= 0);
	ret = dio_bio_alloc(dio, dio->map_bh.b_bdev, sector, nr_pages);
	dio->boundary = 0;
out:
	return ret;
}

static int dio_bio_add_page(struct dio *dio)
{
	int ret;

	ret = bio_add_page(dio->bio, dio->cur_page,
			dio->cur_page_len, dio->cur_page_offset);
	if (ret == dio->cur_page_len) {
		/*
		 * Decrement count only, if we are done with this page
		 */
		if ((dio->cur_page_len + dio->cur_page_offset) == PAGE_SIZE)
			dio->pages_in_io--;
		page_cache_get(dio->cur_page);
		dio->final_block_in_bio = dio->cur_page_block +
			(dio->cur_page_len >> dio->blkbits);
		ret = 0;
	} else {
		ret = 1;
	}
	return ret;
}
		
static int dio_send_cur_page(struct dio *dio)
{
	int ret = 0;

	if (dio->bio) {
		loff_t cur_offset = dio->cur_page_fs_offset;
		loff_t bio_next_offset = dio->logical_offset_in_bio +
			dio->bio->bi_size;

		/*
		 * See whether this new request is contiguous with the old.
		 *
		 * Btrfs cannot handl having logically non-contiguous requests
		 * submitted.  For exmple if you have
		 *
		 * Logical:  [0-4095][HOLE][8192-12287]
		 * Phyiscal: [0-4095]      [4096-8181]
		 *
		 * We cannot submit those pages together as one BIO.  So if our
		 * current logical offset in the file does not equal what would
		 * be the next logical offset in the bio, submit the bio we
		 * have.
		 */
		if (dio->final_block_in_bio != dio->cur_page_block ||
		    cur_offset != bio_next_offset)
			dio_bio_submit(dio);
		/*
		 * Submit now if the underlying fs is about to perform a
		 * metadata read
		 */
		else if (dio->boundary)
			dio_bio_submit(dio);
	}

	if (dio->bio == NULL) {
		ret = dio_new_bio(dio, dio->cur_page_block);
		if (ret)
			goto out;
	}

	if (dio_bio_add_page(dio) != 0) {
		dio_bio_submit(dio);
		ret = dio_new_bio(dio, dio->cur_page_block);
		if (ret == 0) {
			ret = dio_bio_add_page(dio);
			BUG_ON(ret != 0);
		}
	}
out:
	return ret;
}

static int
submit_page_section(struct dio *dio, struct page *page,
		unsigned offset, unsigned len, sector_t blocknr)
{
	int ret = 0;

	if (dio->rw & WRITE) {
		/*
		 * Read accounting is performed in submit_bio()
		 */
		task_io_account_write(len);
	}

	/*
	 * Can we just grow the current page's presence in the dio?
	 */
	if (	(dio->cur_page == page) &&
		(dio->cur_page_offset + dio->cur_page_len == offset) &&
		(dio->cur_page_block +
			(dio->cur_page_len >> dio->blkbits) == blocknr)) {
		dio->cur_page_len += len;

		/*
		 * If dio->boundary then we want to schedule the IO now to
		 * avoid metadata seeks.
		 */
		if (dio->boundary) {
			ret = dio_send_cur_page(dio);
			page_cache_release(dio->cur_page);
			dio->cur_page = NULL;
		}
		goto out;
	}

	/*
	 * If there's a deferred page already there then send it.
	 */
	if (dio->cur_page) {
		ret = dio_send_cur_page(dio);
		page_cache_release(dio->cur_page);
		dio->cur_page = NULL;
		if (ret)
			goto out;
	}

	page_cache_get(page);		/* It is in dio */
	dio->cur_page = page;
	dio->cur_page_offset = offset;
	dio->cur_page_len = len;
	dio->cur_page_block = blocknr;
	dio->cur_page_fs_offset = dio->block_in_file << dio->blkbits;
out:
	return ret;
}

static void clean_blockdev_aliases(struct dio *dio)
{
	unsigned i;
	unsigned nblocks;

	nblocks = dio->map_bh.b_size >> dio->inode->i_blkbits;

	for (i = 0; i < nblocks; i++) {
		unmap_underlying_metadata(dio->map_bh.b_bdev,
					dio->map_bh.b_blocknr + i);
	}
}

static void dio_zero_block(struct dio *dio, int end)
{
	unsigned dio_blocks_per_fs_block;
	unsigned this_chunk_blocks;	/* In dio_blocks */
	unsigned this_chunk_bytes;
	struct page *page;

	dio->start_zero_done = 1;
	if (!dio->blkfactor || !buffer_new(&dio->map_bh))
		return;

	dio_blocks_per_fs_block = 1 << dio->blkfactor;
	this_chunk_blocks = dio->block_in_file & (dio_blocks_per_fs_block - 1);

	if (!this_chunk_blocks)
		return;

	/*
	 * We need to zero out part of an fs block.  It is either at the
	 * beginning or the end of the fs block.
	 */
	if (end) 
		this_chunk_blocks = dio_blocks_per_fs_block - this_chunk_blocks;

	this_chunk_bytes = this_chunk_blocks << dio->blkbits;

	page = ZERO_PAGE(0);
	if (submit_page_section(dio, page, 0, this_chunk_bytes, 
				dio->next_block_for_io))
		return;

	dio->next_block_for_io += this_chunk_blocks;
}

static int do_direct_IO(struct dio *dio)
{
	const unsigned blkbits = dio->blkbits;
	const unsigned blocks_per_page = PAGE_SIZE >> blkbits;
	struct page *page;
	unsigned block_in_page;
	struct buffer_head *map_bh = &dio->map_bh;
	int ret = 0;

	/* The I/O can start at any block offset within the first page */
	block_in_page = dio->first_block_in_page;

	while (dio->block_in_file < dio->final_block_in_request) {
		page = dio_get_page(dio);
		if (IS_ERR(page)) {
			ret = PTR_ERR(page);
			goto out;
		}

		while (block_in_page < blocks_per_page) {
			unsigned offset_in_page = block_in_page << blkbits;
			unsigned this_chunk_bytes;	/* # of bytes mapped */
			unsigned this_chunk_blocks;	/* # of blocks */
			unsigned u;

			if (dio->blocks_available == 0) {
				/*
				 * Need to go and map some more disk
				 */
				unsigned long blkmask;
				unsigned long dio_remainder;

				ret = get_more_blocks(dio);
				if (ret) {
					page_cache_release(page);
					goto out;
				}
				if (!buffer_mapped(map_bh))
					goto do_holes;

				dio->blocks_available =
						map_bh->b_size >> dio->blkbits;
				dio->next_block_for_io =
					map_bh->b_blocknr << dio->blkfactor;
				if (buffer_new(map_bh))
					clean_blockdev_aliases(dio);

				if (!dio->blkfactor)
					goto do_holes;

				blkmask = (1 << dio->blkfactor) - 1;
				dio_remainder = (dio->block_in_file & blkmask);

				/*
				 * If we are at the start of IO and that IO
				 * starts partway into a fs-block,
				 * dio_remainder will be non-zero.  If the IO
				 * is a read then we can simply advance the IO
				 * cursor to the first block which is to be
				 * read.  But if the IO is a write and the
				 * block was newly allocated we cannot do that;
				 * the start of the fs block must be zeroed out
				 * on-disk
				 */
				if (!buffer_new(map_bh))
					dio->next_block_for_io += dio_remainder;
				dio->blocks_available -= dio_remainder;
			}
do_holes:
			/* Handle holes */
			if (!buffer_mapped(map_bh)) {
				loff_t i_size_aligned;

				/* AKPM: eargh, -ENOTBLK is a hack */
				if (dio->rw & WRITE) {
					page_cache_release(page);
					return -ENOTBLK;
				}

				/*
				 * Be sure to account for a partial block as the
				 * last block in the file
				 */
				i_size_aligned = ALIGN(i_size_read(dio->inode),
							1 << blkbits);
				if (dio->block_in_file >=
						i_size_aligned >> blkbits) {
					/* We hit eof */
					page_cache_release(page);
					goto out;
				}
				zero_user(page, block_in_page << blkbits,
						1 << blkbits);
				dio->block_in_file++;
				block_in_page++;
				goto next_block;
			}

			/*
			 * If we're performing IO which has an alignment which
			 * is finer than the underlying fs, go check to see if
			 * we must zero out the start of this block.
			 */
			if (unlikely(dio->blkfactor && !dio->start_zero_done))
				dio_zero_block(dio, 0);

			/*
			 * Work out, in this_chunk_blocks, how much disk we
			 * can add to this page
			 */
			this_chunk_blocks = dio->blocks_available;
			u = (PAGE_SIZE - offset_in_page) >> blkbits;
			if (this_chunk_blocks > u)
				this_chunk_blocks = u;
			u = dio->final_block_in_request - dio->block_in_file;
			if (this_chunk_blocks > u)
				this_chunk_blocks = u;
			this_chunk_bytes = this_chunk_blocks << blkbits;
			BUG_ON(this_chunk_bytes == 0);

			dio->boundary = buffer_boundary(map_bh);
			ret = submit_page_section(dio, page, offset_in_page,
				this_chunk_bytes, dio->next_block_for_io);
			if (ret) {
				page_cache_release(page);
				goto out;
			}
			dio->next_block_for_io += this_chunk_blocks;

			dio->block_in_file += this_chunk_blocks;
			block_in_page += this_chunk_blocks;
			dio->blocks_available -= this_chunk_blocks;
next_block:
			BUG_ON(dio->block_in_file > dio->final_block_in_request);
			if (dio->block_in_file == dio->final_block_in_request)
				break;
		}

		/* Drop the ref which was taken in get_user_pages() */
		page_cache_release(page);
		block_in_page = 0;
	}
out:
	return ret;
}

static ssize_t
direct_io_worker(int rw, struct kiocb *iocb, struct inode *inode, 
	const struct iovec *iov, loff_t offset, unsigned long nr_segs, 
	unsigned blkbits, get_block_t get_block, dio_iodone_t end_io,
	dio_submit_t submit_io, struct dio *dio)
{
	unsigned long user_addr; 
	unsigned long flags;
	int seg;
	ssize_t ret = 0;
	ssize_t ret2;
	size_t bytes;

	dio->inode = inode;
	dio->rw = rw;
	dio->blkbits = blkbits;
	dio->blkfactor = inode->i_blkbits - blkbits;
	dio->block_in_file = offset >> blkbits;

	dio->get_block = get_block;
	dio->end_io = end_io;
	dio->submit_io = submit_io;
	dio->final_block_in_bio = -1;
	dio->next_block_for_io = -1;

	dio->iocb = iocb;
	dio->i_size = i_size_read(inode);

	spin_lock_init(&dio->bio_lock);
	dio->refcount = 1;

	/*
	 * In case of non-aligned buffers, we may need 2 more
	 * pages since we need to zero out first and last block.
	 */
	if (unlikely(dio->blkfactor))
		dio->pages_in_io = 2;

	for (seg = 0; seg < nr_segs; seg++) {
		user_addr = (unsigned long)iov[seg].iov_base;
		dio->pages_in_io +=
			((user_addr+iov[seg].iov_len +PAGE_SIZE-1)/PAGE_SIZE
				- user_addr/PAGE_SIZE);
	}

	for (seg = 0; seg < nr_segs; seg++) {
		user_addr = (unsigned long)iov[seg].iov_base;
		dio->size += bytes = iov[seg].iov_len;

		/* Index into the first page of the first block */
		dio->first_block_in_page = (user_addr & ~PAGE_MASK) >> blkbits;
		dio->final_block_in_request = dio->block_in_file +
						(bytes >> blkbits);
		/* Page fetching state */
		dio->head = 0;
		dio->tail = 0;
		dio->curr_page = 0;

		dio->total_pages = 0;
		if (user_addr & (PAGE_SIZE-1)) {
			dio->total_pages++;
			bytes -= PAGE_SIZE - (user_addr & (PAGE_SIZE - 1));
		}
		dio->total_pages += (bytes + PAGE_SIZE - 1) / PAGE_SIZE;
		dio->curr_user_address = user_addr;
	
		ret = do_direct_IO(dio);

		dio->result += iov[seg].iov_len -
			((dio->final_block_in_request - dio->block_in_file) <<
					blkbits);

		if (ret) {
			dio_cleanup(dio);
			break;
		}
	} /* end iovec loop */

	if (ret == -ENOTBLK) {
		/*
		 * The remaining part of the request will be
		 * be handled by buffered I/O when we return
		 */
		ret = 0;
	}
	/*
	 * There may be some unwritten disk at the end of a part-written
	 * fs-block-sized block.  Go zero that now.
	 */
	dio_zero_block(dio, 1);

	if (dio->cur_page) {
		ret2 = dio_send_cur_page(dio);
		if (ret == 0)
			ret = ret2;
		page_cache_release(dio->cur_page);
		dio->cur_page = NULL;
	}
	if (dio->bio)
		dio_bio_submit(dio);

	/*
	 * It is possible that, we return short IO due to end of file.
	 * In that case, we need to release all the pages we got hold on.
	 */
	dio_cleanup(dio);

	/*
	 * All block lookups have been performed. For READ requests
	 * we can let i_mutex go now that its achieved its purpose
	 * of protecting us from looking up uninitialized blocks.
	 */
	if (rw == READ && (dio->flags & DIO_LOCKING))
		mutex_unlock(&dio->inode->i_mutex);

	/*
	 * The only time we want to leave bios in flight is when a successful
	 * partial aio read or full aio write have been setup.  In that case
	 * bio completion will call aio_complete.  The only time it's safe to
	 * call aio_complete is when we return -EIOCBQUEUED, so we key on that.
	 * This had *better* be the only place that raises -EIOCBQUEUED.
	 */
	BUG_ON(ret == -EIOCBQUEUED);
	if (dio->is_async && ret == 0 && dio->result &&
	    ((rw & READ) || (dio->result == dio->size)))
		ret = -EIOCBQUEUED;

	if (ret != -EIOCBQUEUED) {
		/* All IO is now issued, send it on its way */
		blk_run_address_space(inode->i_mapping);
		dio_await_completion(dio);
	}

	/*
	 * Sync will always be dropping the final ref and completing the
	 * operation.  AIO can if it was a broken operation described above or
	 * in fact if all the bios race to complete before we get here.  In
	 * that case dio_complete() translates the EIOCBQUEUED into the proper
	 * return code that the caller will hand to aio_complete().
	 *
	 * This is managed by the bio_lock instead of being an atomic_t so that
	 * completion paths can drop their ref and use the remaining count to
	 * decide to wake the submission path atomically.
	 */
	spin_lock_irqsave(&dio->bio_lock, flags);
	ret2 = --dio->refcount;
	spin_unlock_irqrestore(&dio->bio_lock, flags);

	if (ret2 == 0) {
		ret = dio_complete(dio, offset, ret, false);
		kfree(dio);
	} else
		BUG_ON(ret != -EIOCBQUEUED);

	return ret;
}

ssize_t
__blockdev_direct_IO_newtrunc(int rw, struct kiocb *iocb, struct inode *inode,
	struct block_device *bdev, const struct iovec *iov, loff_t offset, 
	unsigned long nr_segs, get_block_t get_block, dio_iodone_t end_io,
	dio_submit_t submit_io,	int flags)
{
	int seg;
	size_t size;
	unsigned long addr;
	unsigned blkbits = inode->i_blkbits;
	unsigned bdev_blkbits = 0;
	unsigned blocksize_mask = (1 << blkbits) - 1;
	ssize_t retval = -EINVAL;
	loff_t end = offset;
	struct dio *dio;

	if (rw & WRITE)
		rw = WRITE_ODIRECT_PLUG;

	if (bdev)
		bdev_blkbits = blksize_bits(bdev_logical_block_size(bdev));

	if (offset & blocksize_mask) {
		if (bdev)
			 blkbits = bdev_blkbits;
		blocksize_mask = (1 << blkbits) - 1;
		if (offset & blocksize_mask)
			goto out;
	}

	/* Check the memory alignment.  Blocks cannot straddle pages */
	for (seg = 0; seg < nr_segs; seg++) {
		addr = (unsigned long)iov[seg].iov_base;
		size = iov[seg].iov_len;
		end += size;
		if ((addr & blocksize_mask) || (size & blocksize_mask))  {
			if (bdev)
				 blkbits = bdev_blkbits;
			blocksize_mask = (1 << blkbits) - 1;
			if ((addr & blocksize_mask) || (size & blocksize_mask))  
				goto out;
		}
	}

	dio = kmalloc(sizeof(*dio), GFP_KERNEL);
	retval = -ENOMEM;
	if (!dio)
		goto out;
	/*
	 * Believe it or not, zeroing out the page array caused a .5%
	 * performance regression in a database benchmark.  So, we take
	 * care to only zero out what's needed.
	 */
	memset(dio, 0, offsetof(struct dio, pages));

	dio->flags = flags;
	if (dio->flags & DIO_LOCKING) {
		/* watch out for a 0 len io from a tricksy fs */
		if (rw == READ && end > offset) {
			struct address_space *mapping =
					iocb->ki_filp->f_mapping;

			/* will be released by direct_io_worker */
			mutex_lock(&inode->i_mutex);

			retval = filemap_write_and_wait_range(mapping, offset,
							      end - 1);
			if (retval) {
				mutex_unlock(&inode->i_mutex);
				kfree(dio);
				goto out;
			}
		}

		/*
		 * Will be released at I/O completion, possibly in a
		 * different thread.
		 */
		down_read_non_owner(&inode->i_alloc_sem);
	}

	/*
	 * For file extending writes updating i_size before data
	 * writeouts complete can expose uninitialized blocks. So
	 * even for AIO, we need to wait for i/o to complete before
	 * returning in this case.
	 */
	dio->is_async = !is_sync_kiocb(iocb) && !((rw & WRITE) &&
		(end > i_size_read(inode)));

	retval = direct_io_worker(rw, iocb, inode, iov, offset,
				nr_segs, blkbits, get_block, end_io,
				submit_io, dio);

out:
	return retval;
}
EXPORT_SYMBOL(__blockdev_direct_IO_newtrunc);

ssize_t
__blockdev_direct_IO(int rw, struct kiocb *iocb, struct inode *inode,
	struct block_device *bdev, const struct iovec *iov, loff_t offset,
	unsigned long nr_segs, get_block_t get_block, dio_iodone_t end_io,
	dio_submit_t submit_io,	int flags)
{
	ssize_t retval;

	retval = __blockdev_direct_IO_newtrunc(rw, iocb, inode, bdev, iov,
			offset, nr_segs, get_block, end_io, submit_io, flags);
	/*
	 * In case of error extending write may have instantiated a few
	 * blocks outside i_size. Trim these off again for DIO_LOCKING.
	 * NOTE: DIO_NO_LOCK/DIO_OWN_LOCK callers have to handle this in
	 * their own manner. This is a further example of where the old
	 * truncate sequence is inadequate.
	 *
	 * NOTE: filesystems with their own locking have to handle this
	 * on their own.
	 */
	if (flags & DIO_LOCKING) {
		if (unlikely((rw & WRITE) && retval < 0)) {
			loff_t isize = i_size_read(inode);
			loff_t end = offset + iov_length(iov, nr_segs);

			if (end > isize)
				vmtruncate(inode, isize);
		}
	}

	return retval;
}
EXPORT_SYMBOL(__blockdev_direct_IO);

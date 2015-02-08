

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/backing-dev.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/kernel_stat.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/swap.h>
#include <linux/writeback.h>
#include <linux/task_io_accounting_ops.h>
#include <linux/fault-inject.h>

#define CREATE_TRACE_POINTS
#include <trace/events/block.h>

#include "blk.h"

EXPORT_TRACEPOINT_SYMBOL_GPL(block_remap);
EXPORT_TRACEPOINT_SYMBOL_GPL(block_rq_remap);
EXPORT_TRACEPOINT_SYMBOL_GPL(block_bio_complete);

static int __make_request(struct request_queue *q, struct bio *bio);

static struct kmem_cache *request_cachep;

struct kmem_cache *blk_requestq_cachep;

static struct workqueue_struct *kblockd_workqueue;

static void drive_stat_acct(struct request *rq, int new_io)
{
	struct hd_struct *part;
	int rw = rq_data_dir(rq);
	int cpu;

	if (!blk_do_io_stat(rq))
		return;

	cpu = part_stat_lock();
	part = disk_map_sector_rcu(rq->rq_disk, blk_rq_pos(rq));

	if (!new_io)
		part_stat_inc(cpu, part, merges[rw]);
	else {
		part_round_stats(cpu, part);
		part_inc_in_flight(part, rw);
	}

	part_stat_unlock();
}

void blk_queue_congestion_threshold(struct request_queue *q)
{
	int nr;

	nr = q->nr_requests - (q->nr_requests / 8) + 1;
	if (nr > q->nr_requests)
		nr = q->nr_requests;
	q->nr_congestion_on = nr;

	nr = q->nr_requests - (q->nr_requests / 8) - (q->nr_requests / 16) - 1;
	if (nr < 1)
		nr = 1;
	q->nr_congestion_off = nr;
}

struct backing_dev_info *blk_get_backing_dev_info(struct block_device *bdev)
{
	struct backing_dev_info *ret = NULL;
	struct request_queue *q = bdev_get_queue(bdev);

	if (q)
		ret = &q->backing_dev_info;
	return ret;
}
EXPORT_SYMBOL(blk_get_backing_dev_info);

void blk_rq_init(struct request_queue *q, struct request *rq)
{
	memset(rq, 0, sizeof(*rq));

	INIT_LIST_HEAD(&rq->queuelist);
	INIT_LIST_HEAD(&rq->timeout_list);
	rq->cpu = -1;
	rq->q = q;
	rq->__sector = (sector_t) -1;
	INIT_HLIST_NODE(&rq->hash);
	RB_CLEAR_NODE(&rq->rb_node);
	rq->cmd = rq->__cmd;
	rq->cmd_len = BLK_MAX_CDB;
	rq->tag = -1;
	rq->ref_count = 1;
	rq->start_time = jiffies;
	set_start_time_ns(rq);
}
EXPORT_SYMBOL(blk_rq_init);

static void req_bio_endio(struct request *rq, struct bio *bio,
			  unsigned int nbytes, int error)
{
	struct request_queue *q = rq->q;

	if (&q->bar_rq != rq) {
		if (error)
			clear_bit(BIO_UPTODATE, &bio->bi_flags);
		else if (!test_bit(BIO_UPTODATE, &bio->bi_flags))
			error = -EIO;

		if (unlikely(nbytes > bio->bi_size)) {
			printk(KERN_ERR "%s: want %u bytes done, %u left\n",
			       __func__, nbytes, bio->bi_size);
			nbytes = bio->bi_size;
		}

		if (unlikely(rq->cmd_flags & REQ_QUIET))
			set_bit(BIO_QUIET, &bio->bi_flags);

		bio->bi_size -= nbytes;
		bio->bi_sector += (nbytes >> 9);

		if (bio_integrity(bio))
			bio_integrity_advance(bio, nbytes);

		if (bio->bi_size == 0)
			bio_endio(bio, error);
	} else {

		/*
		 * Okay, this is the barrier request in progress, just
		 * record the error;
		 */
		if (error && !q->orderr)
			q->orderr = error;
	}
}

void blk_dump_rq_flags(struct request *rq, char *msg)
{
	int bit;

	printk(KERN_INFO "%s: dev %s: type=%x, flags=%x\n", msg,
		rq->rq_disk ? rq->rq_disk->disk_name : "?", rq->cmd_type,
		rq->cmd_flags);

	printk(KERN_INFO "  sector %llu, nr/cnr %u/%u\n",
	       (unsigned long long)blk_rq_pos(rq),
	       blk_rq_sectors(rq), blk_rq_cur_sectors(rq));
	printk(KERN_INFO "  bio %p, biotail %p, buffer %p, len %u\n",
	       rq->bio, rq->biotail, rq->buffer, blk_rq_bytes(rq));

	if (blk_pc_request(rq)) {
		printk(KERN_INFO "  cdb: ");
		for (bit = 0; bit < BLK_MAX_CDB; bit++)
			printk("%02x ", rq->cmd[bit]);
		printk("\n");
	}
}
EXPORT_SYMBOL(blk_dump_rq_flags);

void blk_plug_device(struct request_queue *q)
{
	WARN_ON(!irqs_disabled());

	/*
	 * don't plug a stopped queue, it must be paired with blk_start_queue()
	 * which will restart the queueing
	 */
	if (blk_queue_stopped(q))
		return;

	if (!queue_flag_test_and_set(QUEUE_FLAG_PLUGGED, q)) {
		mod_timer(&q->unplug_timer, jiffies + q->unplug_delay);
		trace_block_plug(q);
	}
}
EXPORT_SYMBOL(blk_plug_device);

void blk_plug_device_unlocked(struct request_queue *q)
{
	unsigned long flags;

	spin_lock_irqsave(q->queue_lock, flags);
	blk_plug_device(q);
	spin_unlock_irqrestore(q->queue_lock, flags);
}
EXPORT_SYMBOL(blk_plug_device_unlocked);

int blk_remove_plug(struct request_queue *q)
{
	WARN_ON(!irqs_disabled());

	if (!queue_flag_test_and_clear(QUEUE_FLAG_PLUGGED, q))
		return 0;

	del_timer(&q->unplug_timer);
	return 1;
}
EXPORT_SYMBOL(blk_remove_plug);

void __generic_unplug_device(struct request_queue *q)
{
	if (unlikely(blk_queue_stopped(q)))
		return;
	if (!blk_remove_plug(q) && !blk_queue_nonrot(q))
		return;

	q->request_fn(q);
}

void generic_unplug_device(struct request_queue *q)
{
	if (blk_queue_plugged(q)) {
		spin_lock_irq(q->queue_lock);
		__generic_unplug_device(q);
		spin_unlock_irq(q->queue_lock);
	}
}
EXPORT_SYMBOL(generic_unplug_device);

static void blk_backing_dev_unplug(struct backing_dev_info *bdi,
				   struct page *page)
{
	struct request_queue *q = bdi->unplug_io_data;

	blk_unplug(q);
}

void blk_unplug_work(struct work_struct *work)
{
	struct request_queue *q =
		container_of(work, struct request_queue, unplug_work);

	trace_block_unplug_io(q);
	q->unplug_fn(q);
}

void blk_unplug_timeout(unsigned long data)
{
	struct request_queue *q = (struct request_queue *)data;

	trace_block_unplug_timer(q);
	kblockd_schedule_work(q, &q->unplug_work);
}

void blk_unplug(struct request_queue *q)
{
	/*
	 * devices don't necessarily have an ->unplug_fn defined
	 */
	if (q->unplug_fn) {
		trace_block_unplug_io(q);
		q->unplug_fn(q);
	}
}
EXPORT_SYMBOL(blk_unplug);

void blk_start_queue(struct request_queue *q)
{
	WARN_ON(!irqs_disabled());

	queue_flag_clear(QUEUE_FLAG_STOPPED, q);
	__blk_run_queue(q);
}
EXPORT_SYMBOL(blk_start_queue);

void blk_stop_queue(struct request_queue *q)
{
	blk_remove_plug(q);
	queue_flag_set(QUEUE_FLAG_STOPPED, q);
}
EXPORT_SYMBOL(blk_stop_queue);

void blk_sync_queue(struct request_queue *q)
{
	del_timer_sync(&q->unplug_timer);
	del_timer_sync(&q->timeout);
	cancel_work_sync(&q->unplug_work);
}
EXPORT_SYMBOL(blk_sync_queue);

void __blk_run_queue(struct request_queue *q)
{
	blk_remove_plug(q);

	if (unlikely(blk_queue_stopped(q)))
		return;

	if (elv_queue_empty(q))
		return;

	/*
	 * Only recurse once to avoid overrunning the stack, let the unplug
	 * handling reinvoke the handler shortly if we already got there.
	 */
	if (!queue_flag_test_and_set(QUEUE_FLAG_REENTER, q)) {
		q->request_fn(q);
		queue_flag_clear(QUEUE_FLAG_REENTER, q);
	} else {
		queue_flag_set(QUEUE_FLAG_PLUGGED, q);
		kblockd_schedule_work(q, &q->unplug_work);
	}
}
EXPORT_SYMBOL(__blk_run_queue);

void blk_run_queue(struct request_queue *q)
{
	unsigned long flags;

	spin_lock_irqsave(q->queue_lock, flags);
	__blk_run_queue(q);
	spin_unlock_irqrestore(q->queue_lock, flags);
}
EXPORT_SYMBOL(blk_run_queue);

void blk_put_queue(struct request_queue *q)
{
	kobject_put(&q->kobj);
}

void blk_cleanup_queue(struct request_queue *q)
{
	/*
	 * We know we have process context here, so we can be a little
	 * cautious and ensure that pending block actions on this device
	 * are done before moving on. Going into this function, we should
	 * not have processes doing IO to this device.
	 */
	blk_sync_queue(q);

	del_timer_sync(&q->backing_dev_info.laptop_mode_wb_timer);
	mutex_lock(&q->sysfs_lock);
	queue_flag_set_unlocked(QUEUE_FLAG_DEAD, q);
	mutex_unlock(&q->sysfs_lock);

	if (q->elevator)
		elevator_exit(q->elevator);

	blk_put_queue(q);
}
EXPORT_SYMBOL(blk_cleanup_queue);

static int blk_init_free_list(struct request_queue *q)
{
	struct request_list *rl = &q->rq;

	if (unlikely(rl->rq_pool))
		return 0;

	rl->count[BLK_RW_SYNC] = rl->count[BLK_RW_ASYNC] = 0;
	rl->starved[BLK_RW_SYNC] = rl->starved[BLK_RW_ASYNC] = 0;
	rl->elvpriv = 0;
	init_waitqueue_head(&rl->wait[BLK_RW_SYNC]);
	init_waitqueue_head(&rl->wait[BLK_RW_ASYNC]);

	rl->rq_pool = mempool_create_node(BLKDEV_MIN_RQ, mempool_alloc_slab,
				mempool_free_slab, request_cachep, q->node);

	if (!rl->rq_pool)
		return -ENOMEM;

	return 0;
}

struct request_queue *blk_alloc_queue(gfp_t gfp_mask)
{
	return blk_alloc_queue_node(gfp_mask, -1);
}
EXPORT_SYMBOL(blk_alloc_queue);

struct request_queue *blk_alloc_queue_node(gfp_t gfp_mask, int node_id)
{
	struct request_queue *q;
	int err;

	q = kmem_cache_alloc_node(blk_requestq_cachep,
				gfp_mask | __GFP_ZERO, node_id);
	if (!q)
		return NULL;

	q->backing_dev_info.unplug_io_fn = blk_backing_dev_unplug;
	q->backing_dev_info.unplug_io_data = q;
	q->backing_dev_info.ra_pages =
			(VM_MAX_READAHEAD * 1024) / PAGE_CACHE_SIZE;
	q->backing_dev_info.state = 0;
	q->backing_dev_info.capabilities = BDI_CAP_MAP_COPY;
	q->backing_dev_info.name = "block";

	err = bdi_init(&q->backing_dev_info);
	if (err) {
		kmem_cache_free(blk_requestq_cachep, q);
		return NULL;
	}

	setup_timer(&q->backing_dev_info.laptop_mode_wb_timer,
		    laptop_mode_timer_fn, (unsigned long) q);
	init_timer(&q->unplug_timer);
	setup_timer(&q->timeout, blk_rq_timed_out_timer, (unsigned long) q);
	INIT_LIST_HEAD(&q->timeout_list);
	INIT_WORK(&q->unplug_work, blk_unplug_work);

	kobject_init(&q->kobj, &blk_queue_ktype);

	mutex_init(&q->sysfs_lock);
	spin_lock_init(&q->__queue_lock);

	return q;
}
EXPORT_SYMBOL(blk_alloc_queue_node);


struct request_queue *blk_init_queue(request_fn_proc *rfn, spinlock_t *lock)
{
	return blk_init_queue_node(rfn, lock, -1);
}
EXPORT_SYMBOL(blk_init_queue);

struct request_queue *
blk_init_queue_node(request_fn_proc *rfn, spinlock_t *lock, int node_id)
{
	struct request_queue *uninit_q, *q;

	uninit_q = blk_alloc_queue_node(GFP_KERNEL, node_id);
	if (!uninit_q)
		return NULL;

	q = blk_init_allocated_queue_node(uninit_q, rfn, lock, node_id);
	if (!q)
		blk_cleanup_queue(uninit_q);

	return q;
}
EXPORT_SYMBOL(blk_init_queue_node);

struct request_queue *
blk_init_allocated_queue(struct request_queue *q, request_fn_proc *rfn,
			 spinlock_t *lock)
{
	return blk_init_allocated_queue_node(q, rfn, lock, -1);
}
EXPORT_SYMBOL(blk_init_allocated_queue);

struct request_queue *
blk_init_allocated_queue_node(struct request_queue *q, request_fn_proc *rfn,
			      spinlock_t *lock, int node_id)
{
	if (!q)
		return NULL;

	q->node = node_id;
	if (blk_init_free_list(q))
		return NULL;

	q->request_fn		= rfn;
	q->prep_rq_fn		= NULL;
	q->unplug_fn		= generic_unplug_device;
	q->queue_flags		= QUEUE_FLAG_DEFAULT;
	q->queue_lock		= lock;

	/*
	 * This also sets hw/phys segments, boundary and size
	 */
	blk_queue_make_request(q, __make_request);

	q->sg_reserved_size = INT_MAX;

	/*
	 * all done
	 */
	if (!elevator_init(q, NULL)) {
		blk_queue_congestion_threshold(q);
		return q;
	}

	return NULL;
}
EXPORT_SYMBOL(blk_init_allocated_queue_node);

int blk_get_queue(struct request_queue *q)
{
	if (likely(!test_bit(QUEUE_FLAG_DEAD, &q->queue_flags))) {
		kobject_get(&q->kobj);
		return 0;
	}

	return 1;
}

static inline void blk_free_request(struct request_queue *q, struct request *rq)
{
	if (rq->cmd_flags & REQ_ELVPRIV)
		elv_put_request(q, rq);
	mempool_free(rq, q->rq.rq_pool);
}

static struct request *
blk_alloc_request(struct request_queue *q, int flags, int priv, gfp_t gfp_mask)
{
	struct request *rq = mempool_alloc(q->rq.rq_pool, gfp_mask);

	if (!rq)
		return NULL;

	blk_rq_init(q, rq);

	rq->cmd_flags = flags | REQ_ALLOCED;

	if (priv) {
		if (unlikely(elv_set_request(q, rq, gfp_mask))) {
			mempool_free(rq, q->rq.rq_pool);
			return NULL;
		}
		rq->cmd_flags |= REQ_ELVPRIV;
	}

	return rq;
}

static inline int ioc_batching(struct request_queue *q, struct io_context *ioc)
{
	if (!ioc)
		return 0;

	/*
	 * Make sure the process is able to allocate at least 1 request
	 * even if the batch times out, otherwise we could theoretically
	 * lose wakeups.
	 */
	return ioc->nr_batch_requests == q->nr_batching ||
		(ioc->nr_batch_requests > 0
		&& time_before(jiffies, ioc->last_waited + BLK_BATCH_TIME));
}

static void ioc_set_batching(struct request_queue *q, struct io_context *ioc)
{
	if (!ioc || ioc_batching(q, ioc))
		return;

	ioc->nr_batch_requests = q->nr_batching;
	ioc->last_waited = jiffies;
}

static void __freed_request(struct request_queue *q, int sync)
{
	struct request_list *rl = &q->rq;

	if (rl->count[sync] < queue_congestion_off_threshold(q))
		blk_clear_queue_congested(q, sync);

	if (rl->count[sync] + 1 <= q->nr_requests) {
		if (waitqueue_active(&rl->wait[sync]))
			wake_up(&rl->wait[sync]);

		blk_clear_queue_full(q, sync);
	}
}

static void freed_request(struct request_queue *q, int sync, int priv)
{
	struct request_list *rl = &q->rq;

	rl->count[sync]--;
	if (priv)
		rl->elvpriv--;

	__freed_request(q, sync);

	if (unlikely(rl->starved[sync ^ 1]))
		__freed_request(q, sync ^ 1);
}

static struct request *get_request(struct request_queue *q, int rw_flags,
				   struct bio *bio, gfp_t gfp_mask)
{
	struct request *rq = NULL;
	struct request_list *rl = &q->rq;
	struct io_context *ioc = NULL;
	const bool is_sync = rw_is_sync(rw_flags) != 0;
	int may_queue, priv;

	may_queue = elv_may_queue(q, rw_flags);
	if (may_queue == ELV_MQUEUE_NO)
		goto rq_starved;

	if (rl->count[is_sync]+1 >= queue_congestion_on_threshold(q)) {
		if (rl->count[is_sync]+1 >= q->nr_requests) {
			ioc = current_io_context(GFP_ATOMIC, q->node);
			/*
			 * The queue will fill after this allocation, so set
			 * it as full, and mark this process as "batching".
			 * This process will be allowed to complete a batch of
			 * requests, others will be blocked.
			 */
			if (!blk_queue_full(q, is_sync)) {
				ioc_set_batching(q, ioc);
				blk_set_queue_full(q, is_sync);
			} else {
				if (may_queue != ELV_MQUEUE_MUST
						&& !ioc_batching(q, ioc)) {
					/*
					 * The queue is full and the allocating
					 * process is not a "batcher", and not
					 * exempted by the IO scheduler
					 */
					goto out;
				}
			}
		}
		blk_set_queue_congested(q, is_sync);
	}

	/*
	 * Only allow batching queuers to allocate up to 50% over the defined
	 * limit of requests, otherwise we could have thousands of requests
	 * allocated with any setting of ->nr_requests
	 */
	if (rl->count[is_sync] >= (3 * q->nr_requests / 2))
		goto out;

	rl->count[is_sync]++;
	rl->starved[is_sync] = 0;

	priv = !test_bit(QUEUE_FLAG_ELVSWITCH, &q->queue_flags);
	if (priv)
		rl->elvpriv++;

	if (blk_queue_io_stat(q))
		rw_flags |= REQ_IO_STAT;
	spin_unlock_irq(q->queue_lock);

	rq = blk_alloc_request(q, rw_flags, priv, gfp_mask);
	if (unlikely(!rq)) {
		/*
		 * Allocation failed presumably due to memory. Undo anything
		 * we might have messed up.
		 *
		 * Allocating task should really be put onto the front of the
		 * wait queue, but this is pretty rare.
		 */
		spin_lock_irq(q->queue_lock);
		freed_request(q, is_sync, priv);

		/*
		 * in the very unlikely event that allocation failed and no
		 * requests for this direction was pending, mark us starved
		 * so that freeing of a request in the other direction will
		 * notice us. another possible fix would be to split the
		 * rq mempool into READ and WRITE
		 */
rq_starved:
		if (unlikely(rl->count[is_sync] == 0))
			rl->starved[is_sync] = 1;

		goto out;
	}

	/*
	 * ioc may be NULL here, and ioc_batching will be false. That's
	 * OK, if the queue is under the request limit then requests need
	 * not count toward the nr_batch_requests limit. There will always
	 * be some limit enforced by BLK_BATCH_TIME.
	 */
	if (ioc_batching(q, ioc))
		ioc->nr_batch_requests--;

	trace_block_getrq(q, bio, rw_flags & 1);
out:
	return rq;
}

static struct request *get_request_wait(struct request_queue *q, int rw_flags,
					struct bio *bio)
{
	const bool is_sync = rw_is_sync(rw_flags) != 0;
	struct request *rq;

	rq = get_request(q, rw_flags, bio, GFP_NOIO);
	while (!rq) {
		DEFINE_WAIT(wait);
		struct io_context *ioc;
		struct request_list *rl = &q->rq;

		prepare_to_wait_exclusive(&rl->wait[is_sync], &wait,
				TASK_UNINTERRUPTIBLE);

		trace_block_sleeprq(q, bio, rw_flags & 1);

		__generic_unplug_device(q);
		spin_unlock_irq(q->queue_lock);
		io_schedule();

		/*
		 * After sleeping, we become a "batching" process and
		 * will be able to allocate at least one request, and
		 * up to a big batch of them for a small period time.
		 * See ioc_batching, ioc_set_batching
		 */
		ioc = current_io_context(GFP_NOIO, q->node);
		ioc_set_batching(q, ioc);

		spin_lock_irq(q->queue_lock);
		finish_wait(&rl->wait[is_sync], &wait);

		rq = get_request(q, rw_flags, bio, GFP_NOIO);
	};

	return rq;
}

struct request *blk_get_request(struct request_queue *q, int rw, gfp_t gfp_mask)
{
	struct request *rq;

	BUG_ON(rw != READ && rw != WRITE);

	spin_lock_irq(q->queue_lock);
	if (gfp_mask & __GFP_WAIT) {
		rq = get_request_wait(q, rw, NULL);
	} else {
		rq = get_request(q, rw, NULL, gfp_mask);
		if (!rq)
			spin_unlock_irq(q->queue_lock);
	}
	/* q->queue_lock is unlocked at this point */

	return rq;
}
EXPORT_SYMBOL(blk_get_request);

struct request *blk_make_request(struct request_queue *q, struct bio *bio,
				 gfp_t gfp_mask)
{
	struct request *rq = blk_get_request(q, bio_data_dir(bio), gfp_mask);

	if (unlikely(!rq))
		return ERR_PTR(-ENOMEM);

	for_each_bio(bio) {
		struct bio *bounce_bio = bio;
		int ret;

		blk_queue_bounce(q, &bounce_bio);
		ret = blk_rq_append_bio(q, rq, bounce_bio);
		if (unlikely(ret)) {
			blk_put_request(rq);
			return ERR_PTR(ret);
		}
	}

	return rq;
}
EXPORT_SYMBOL(blk_make_request);

void blk_requeue_request(struct request_queue *q, struct request *rq)
{
	blk_delete_timer(rq);
	blk_clear_rq_complete(rq);
	trace_block_rq_requeue(q, rq);

	if (blk_rq_tagged(rq))
		blk_queue_end_tag(q, rq);

	BUG_ON(blk_queued_rq(rq));

	elv_requeue_request(q, rq);
}
EXPORT_SYMBOL(blk_requeue_request);

void blk_insert_request(struct request_queue *q, struct request *rq,
			int at_head, void *data)
{
	int where = at_head ? ELEVATOR_INSERT_FRONT : ELEVATOR_INSERT_BACK;
	unsigned long flags;

	/*
	 * tell I/O scheduler that this isn't a regular read/write (ie it
	 * must not attempt merges on this) and that it acts as a soft
	 * barrier
	 */
	rq->cmd_type = REQ_TYPE_SPECIAL;

	rq->special = data;

	spin_lock_irqsave(q->queue_lock, flags);

	/*
	 * If command is tagged, release the tag
	 */
	if (blk_rq_tagged(rq))
		blk_queue_end_tag(q, rq);

	drive_stat_acct(rq, 1);
	__elv_add_request(q, rq, where, 0);
	__blk_run_queue(q);
	spin_unlock_irqrestore(q->queue_lock, flags);
}
EXPORT_SYMBOL(blk_insert_request);

static inline void add_request(struct request_queue *q, struct request *req)
{
	drive_stat_acct(req, 1);

	/*
	 * elevator indicated where it wants this request to be
	 * inserted at elevator_merge time
	 */
	__elv_add_request(q, req, ELEVATOR_INSERT_SORT, 0);
}

static void part_round_stats_single(int cpu, struct hd_struct *part,
				    unsigned long now)
{
	if (now == part->stamp)
		return;

	if (part_in_flight(part)) {
		__part_stat_add(cpu, part, time_in_queue,
				part_in_flight(part) * (now - part->stamp));
		__part_stat_add(cpu, part, io_ticks, (now - part->stamp));
	}
	part->stamp = now;
}

void part_round_stats(int cpu, struct hd_struct *part)
{
	unsigned long now = jiffies;

	if (part->partno)
		part_round_stats_single(cpu, &part_to_disk(part)->part0, now);
	part_round_stats_single(cpu, part, now);
}
EXPORT_SYMBOL_GPL(part_round_stats);

void __blk_put_request(struct request_queue *q, struct request *req)
{
	if (unlikely(!q))
		return;
	if (unlikely(--req->ref_count))
		return;

	elv_completed_request(q, req);

	/* this is a bio leak */
	WARN_ON(req->bio != NULL);

	/*
	 * Request may not have originated from ll_rw_blk. if not,
	 * it didn't come out of our reserved rq pools
	 */
	if (req->cmd_flags & REQ_ALLOCED) {
		int is_sync = rq_is_sync(req) != 0;
		int priv = req->cmd_flags & REQ_ELVPRIV;

		BUG_ON(!list_empty(&req->queuelist));
		BUG_ON(!hlist_unhashed(&req->hash));

		blk_free_request(q, req);
		freed_request(q, is_sync, priv);
	}
}
EXPORT_SYMBOL_GPL(__blk_put_request);

void blk_put_request(struct request *req)
{
	unsigned long flags;
	struct request_queue *q = req->q;

	spin_lock_irqsave(q->queue_lock, flags);
	__blk_put_request(q, req);
	spin_unlock_irqrestore(q->queue_lock, flags);
}
EXPORT_SYMBOL(blk_put_request);

void init_request_from_bio(struct request *req, struct bio *bio)
{
	req->cpu = bio->bi_comp_cpu;
	req->cmd_type = REQ_TYPE_FS;

	/*
	 * Inherit FAILFAST from bio (for read-ahead, and explicit
	 * FAILFAST).  FAILFAST flags are identical for req and bio.
	 */
	if (bio_rw_flagged(bio, BIO_RW_AHEAD))
		req->cmd_flags |= REQ_FAILFAST_MASK;
	else
		req->cmd_flags |= bio->bi_rw & REQ_FAILFAST_MASK;

	if (bio_rw_flagged(bio, BIO_RW_DISCARD))
		req->cmd_flags |= REQ_DISCARD;
	if (bio_rw_flagged(bio, BIO_RW_BARRIER))
		req->cmd_flags |= REQ_HARDBARRIER;
	if (bio_rw_flagged(bio, BIO_RW_SYNCIO))
		req->cmd_flags |= REQ_RW_SYNC;
	if (bio_rw_flagged(bio, BIO_RW_META))
		req->cmd_flags |= REQ_RW_META;
	if (bio_rw_flagged(bio, BIO_RW_NOIDLE))
		req->cmd_flags |= REQ_NOIDLE;

	req->errors = 0;
	req->__sector = bio->bi_sector;
	req->ioprio = bio_prio(bio);
	blk_rq_bio_prep(req->q, req, bio);
}

static inline bool queue_should_plug(struct request_queue *q)
{
	return !(blk_queue_nonrot(q) && blk_queue_tagged(q));
}

static int __make_request(struct request_queue *q, struct bio *bio)
{
	struct request *req;
	int el_ret;
	unsigned int bytes = bio->bi_size;
	const unsigned short prio = bio_prio(bio);
	const bool sync = bio_rw_flagged(bio, BIO_RW_SYNCIO);
	const bool unplug = bio_rw_flagged(bio, BIO_RW_UNPLUG);
	const unsigned int ff = bio->bi_rw & REQ_FAILFAST_MASK;
	int rw_flags;

	if (bio_rw_flagged(bio, BIO_RW_BARRIER) &&
	    (q->next_ordered == QUEUE_ORDERED_NONE)) {
		bio_endio(bio, -EOPNOTSUPP);
		return 0;
	}
	/*
	 * low level driver can indicate that it wants pages above a
	 * certain limit bounced to low memory (ie for highmem, or even
	 * ISA dma in theory)
	 */
	blk_queue_bounce(q, &bio);

	spin_lock_irq(q->queue_lock);

	if (unlikely(bio_rw_flagged(bio, BIO_RW_BARRIER)) || elv_queue_empty(q))
		goto get_rq;

	el_ret = elv_merge(q, &req, bio);
	switch (el_ret) {
	case ELEVATOR_BACK_MERGE:
		BUG_ON(!rq_mergeable(req));

		if (!ll_back_merge_fn(q, req, bio))
			break;

		trace_block_bio_backmerge(q, bio);

		if ((req->cmd_flags & REQ_FAILFAST_MASK) != ff)
			blk_rq_set_mixed_merge(req);

		req->biotail->bi_next = bio;
		req->biotail = bio;
		req->__data_len += bytes;
		req->ioprio = ioprio_best(req->ioprio, prio);
		if (!blk_rq_cpu_valid(req))
			req->cpu = bio->bi_comp_cpu;
		drive_stat_acct(req, 0);
		elv_bio_merged(q, req, bio);
		if (!attempt_back_merge(q, req))
			elv_merged_request(q, req, el_ret);
		goto out;

	case ELEVATOR_FRONT_MERGE:
		BUG_ON(!rq_mergeable(req));

		if (!ll_front_merge_fn(q, req, bio))
			break;

		trace_block_bio_frontmerge(q, bio);

		if ((req->cmd_flags & REQ_FAILFAST_MASK) != ff) {
			blk_rq_set_mixed_merge(req);
			req->cmd_flags &= ~REQ_FAILFAST_MASK;
			req->cmd_flags |= ff;
		}

		bio->bi_next = req->bio;
		req->bio = bio;

		/*
		 * may not be valid. if the low level driver said
		 * it didn't need a bounce buffer then it better
		 * not touch req->buffer either...
		 */
		req->buffer = bio_data(bio);
		req->__sector = bio->bi_sector;
		req->__data_len += bytes;
		req->ioprio = ioprio_best(req->ioprio, prio);
		if (!blk_rq_cpu_valid(req))
			req->cpu = bio->bi_comp_cpu;
		drive_stat_acct(req, 0);
		elv_bio_merged(q, req, bio);
		if (!attempt_front_merge(q, req))
			elv_merged_request(q, req, el_ret);
		goto out;

	/* ELV_NO_MERGE: elevator says don't/can't merge. */
	default:
		;
	}

get_rq:
	/*
	 * This sync check and mask will be re-done in init_request_from_bio(),
	 * but we need to set it earlier to expose the sync flag to the
	 * rq allocator and io schedulers.
	 */
	rw_flags = bio_data_dir(bio);
	if (sync)
		rw_flags |= REQ_RW_SYNC;

	/*
	 * Grab a free request. This is might sleep but can not fail.
	 * Returns with the queue unlocked.
	 */
	req = get_request_wait(q, rw_flags, bio);

	/*
	 * After dropping the lock and possibly sleeping here, our request
	 * may now be mergeable after it had proven unmergeable (above).
	 * We don't worry about that case for efficiency. It won't happen
	 * often, and the elevators are able to handle it.
	 */
	init_request_from_bio(req, bio);

	spin_lock_irq(q->queue_lock);
	if (test_bit(QUEUE_FLAG_SAME_COMP, &q->queue_flags) ||
	    bio_flagged(bio, BIO_CPU_AFFINE))
		req->cpu = blk_cpu_to_group(smp_processor_id());
	if (queue_should_plug(q) && elv_queue_empty(q))
		blk_plug_device(q);
	add_request(q, req);
out:
	if (unplug || !queue_should_plug(q))
		__generic_unplug_device(q);
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static inline void blk_partition_remap(struct bio *bio)
{
	struct block_device *bdev = bio->bi_bdev;

	if (bio_sectors(bio) && bdev != bdev->bd_contains) {
		struct hd_struct *p = bdev->bd_part;

		bio->bi_sector += p->start_sect;
		bio->bi_bdev = bdev->bd_contains;

		trace_block_remap(bdev_get_queue(bio->bi_bdev), bio,
				    bdev->bd_dev,
				    bio->bi_sector - p->start_sect);
	}
}

static void handle_bad_sector(struct bio *bio)
{
	char b[BDEVNAME_SIZE];

	printk(KERN_INFO "attempt to access beyond end of device\n");
	printk(KERN_INFO "%s: rw=%ld, want=%Lu, limit=%Lu\n",
			bdevname(bio->bi_bdev, b),
			bio->bi_rw,
			(unsigned long long)bio->bi_sector + bio_sectors(bio),
			(long long)(bio->bi_bdev->bd_inode->i_size >> 9));

	set_bit(BIO_EOF, &bio->bi_flags);
}

#ifdef CONFIG_FAIL_MAKE_REQUEST

static DECLARE_FAULT_ATTR(fail_make_request);

static int __init setup_fail_make_request(char *str)
{
	return setup_fault_attr(&fail_make_request, str);
}
__setup("fail_make_request=", setup_fail_make_request);

static int should_fail_request(struct bio *bio)
{
	struct hd_struct *part = bio->bi_bdev->bd_part;

	if (part_to_disk(part)->part0.make_it_fail || part->make_it_fail)
		return should_fail(&fail_make_request, bio->bi_size);

	return 0;
}

static int __init fail_make_request_debugfs(void)
{
	return init_fault_attr_dentries(&fail_make_request,
					"fail_make_request");
}

late_initcall(fail_make_request_debugfs);

#else /* CONFIG_FAIL_MAKE_REQUEST */

static inline int should_fail_request(struct bio *bio)
{
	return 0;
}

#endif /* CONFIG_FAIL_MAKE_REQUEST */

static inline int bio_check_eod(struct bio *bio, unsigned int nr_sectors)
{
	sector_t maxsector;

	if (!nr_sectors)
		return 0;

	/* Test device or partition size, when known. */
	maxsector = bio->bi_bdev->bd_inode->i_size >> 9;
	if (maxsector) {
		sector_t sector = bio->bi_sector;

		if (maxsector < nr_sectors || maxsector - nr_sectors < sector) {
			/*
			 * This may well happen - the kernel calls bread()
			 * without checking the size of the device, e.g., when
			 * mounting a device.
			 */
			handle_bad_sector(bio);
			return 1;
		}
	}

	return 0;
}

static inline void __generic_make_request(struct bio *bio)
{
	struct request_queue *q;
	sector_t old_sector;
	int ret, nr_sectors = bio_sectors(bio);
	dev_t old_dev;
	int err = -EIO;

	might_sleep();

	if (bio_check_eod(bio, nr_sectors))
		goto end_io;

	/*
	 * Resolve the mapping until finished. (drivers are
	 * still free to implement/resolve their own stacking
	 * by explicitly returning 0)
	 *
	 * NOTE: we don't repeat the blk_size check for each new device.
	 * Stacking drivers are expected to know what they are doing.
	 */
	old_sector = -1;
	old_dev = 0;
	do {
		char b[BDEVNAME_SIZE];

		q = bdev_get_queue(bio->bi_bdev);
		if (unlikely(!q)) {
			printk(KERN_ERR
			       "generic_make_request: Trying to access "
				"nonexistent block-device %s (%Lu)\n",
				bdevname(bio->bi_bdev, b),
				(long long) bio->bi_sector);
			goto end_io;
		}

		if (unlikely(!bio_rw_flagged(bio, BIO_RW_DISCARD) &&
			     nr_sectors > queue_max_hw_sectors(q))) {
			printk(KERN_ERR "bio too big device %s (%u > %u)\n",
			       bdevname(bio->bi_bdev, b),
			       bio_sectors(bio),
			       queue_max_hw_sectors(q));
			goto end_io;
		}

		if (unlikely(test_bit(QUEUE_FLAG_DEAD, &q->queue_flags)))
			goto end_io;

		if (should_fail_request(bio))
			goto end_io;

		/*
		 * If this device has partitions, remap block n
		 * of partition p to block n+start(p) of the disk.
		 */
		blk_partition_remap(bio);

		if (bio_integrity_enabled(bio) && bio_integrity_prep(bio))
			goto end_io;

		if (old_sector != -1)
			trace_block_remap(q, bio, old_dev, old_sector);

		old_sector = bio->bi_sector;
		old_dev = bio->bi_bdev->bd_dev;

		if (bio_check_eod(bio, nr_sectors))
			goto end_io;

		if (bio_rw_flagged(bio, BIO_RW_DISCARD) &&
		    !blk_queue_discard(q)) {
			err = -EOPNOTSUPP;
			goto end_io;
		}

		trace_block_bio_queue(q, bio);

		ret = q->make_request_fn(q, bio);
	} while (ret);

	return;

end_io:
	bio_endio(bio, err);
}

void generic_make_request(struct bio *bio)
{
	struct bio_list bio_list_on_stack;

	if (current->bio_list) {
		/* make_request is active */
		bio_list_add(current->bio_list, bio);
		return;
	}
	/* following loop may be a bit non-obvious, and so deserves some
	 * explanation.
	 * Before entering the loop, bio->bi_next is NULL (as all callers
	 * ensure that) so we have a list with a single bio.
	 * We pretend that we have just taken it off a longer list, so
	 * we assign bio_list to a pointer to the bio_list_on_stack,
	 * thus initialising the bio_list of new bios to be
	 * added.  __generic_make_request may indeed add some more bios
	 * through a recursive call to generic_make_request.  If it
	 * did, we find a non-NULL value in bio_list and re-enter the loop
	 * from the top.  In this case we really did just take the bio
	 * of the top of the list (no pretending) and so remove it from
	 * bio_list, and call into __generic_make_request again.
	 *
	 * The loop was structured like this to make only one call to
	 * __generic_make_request (which is important as it is large and
	 * inlined) and to keep the structure simple.
	 */
	BUG_ON(bio->bi_next);
	bio_list_init(&bio_list_on_stack);
	current->bio_list = &bio_list_on_stack;
	do {
		__generic_make_request(bio);
		bio = bio_list_pop(current->bio_list);
	} while (bio);
	current->bio_list = NULL; /* deactivate */
}
EXPORT_SYMBOL(generic_make_request);

void submit_bio(int rw, struct bio *bio)
{
	int count = bio_sectors(bio);

	bio->bi_rw |= rw;

	/*
	 * If it's a regular read/write or a barrier with data attached,
	 * go through the normal accounting stuff before submission.
	 */
	if (bio_has_data(bio) && !(rw & (1 << BIO_RW_DISCARD))) {
		if (rw & WRITE) {
			count_vm_events(PGPGOUT, count);
		} else {
			task_io_account_read(bio->bi_size);
			count_vm_events(PGPGIN, count);
		}

		if (unlikely(block_dump)) {
			char b[BDEVNAME_SIZE];
			printk(KERN_DEBUG "%s(%d): %s block %Lu on %s (%u sectors)\n",
			current->comm, task_pid_nr(current),
				(rw & WRITE) ? "WRITE" : "READ",
				(unsigned long long)bio->bi_sector,
				bdevname(bio->bi_bdev, b),
				count);
		}
	}

	generic_make_request(bio);
}
EXPORT_SYMBOL(submit_bio);

int blk_rq_check_limits(struct request_queue *q, struct request *rq)
{
	if (blk_rq_sectors(rq) > queue_max_sectors(q) ||
	    blk_rq_bytes(rq) > queue_max_hw_sectors(q) << 9) {
		printk(KERN_ERR "%s: over max size limit.\n", __func__);
		return -EIO;
	}

	/*
	 * queue's settings related to segment counting like q->bounce_pfn
	 * may differ from that of other stacking queues.
	 * Recalculate it to check the request correctly on this queue's
	 * limitation.
	 */
	blk_recalc_rq_segments(rq);
	if (rq->nr_phys_segments > queue_max_segments(q)) {
		printk(KERN_ERR "%s: over max segments limit.\n", __func__);
		return -EIO;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(blk_rq_check_limits);

int blk_insert_cloned_request(struct request_queue *q, struct request *rq)
{
	unsigned long flags;

	if (blk_rq_check_limits(q, rq))
		return -EIO;

#ifdef CONFIG_FAIL_MAKE_REQUEST
	if (rq->rq_disk && rq->rq_disk->part0.make_it_fail &&
	    should_fail(&fail_make_request, blk_rq_bytes(rq)))
		return -EIO;
#endif

	spin_lock_irqsave(q->queue_lock, flags);

	/*
	 * Submitting request must be dequeued before calling this function
	 * because it will be linked to another request_queue
	 */
	BUG_ON(blk_queued_rq(rq));

	drive_stat_acct(rq, 1);
	__elv_add_request(q, rq, ELEVATOR_INSERT_BACK, 0);

	spin_unlock_irqrestore(q->queue_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(blk_insert_cloned_request);

unsigned int blk_rq_err_bytes(const struct request *rq)
{
	unsigned int ff = rq->cmd_flags & REQ_FAILFAST_MASK;
	unsigned int bytes = 0;
	struct bio *bio;

	if (!(rq->cmd_flags & REQ_MIXED_MERGE))
		return blk_rq_bytes(rq);

	/*
	 * Currently the only 'mixing' which can happen is between
	 * different fastfail types.  We can safely fail portions
	 * which have all the failfast bits that the first one has -
	 * the ones which are at least as eager to fail as the first
	 * one.
	 */
	for (bio = rq->bio; bio; bio = bio->bi_next) {
		if ((bio->bi_rw & ff) != ff)
			break;
		bytes += bio->bi_size;
	}

	/* this could lead to infinite loop */
	BUG_ON(blk_rq_bytes(rq) && !bytes);
	return bytes;
}
EXPORT_SYMBOL_GPL(blk_rq_err_bytes);

static void blk_account_io_completion(struct request *req, unsigned int bytes)
{
	if (blk_do_io_stat(req)) {
		const int rw = rq_data_dir(req);
		struct hd_struct *part;
		int cpu;

		cpu = part_stat_lock();
		part = disk_map_sector_rcu(req->rq_disk, blk_rq_pos(req));
		part_stat_add(cpu, part, sectors[rw], bytes >> 9);
		part_stat_unlock();
	}
}

static void blk_account_io_done(struct request *req)
{
	/*
	 * Account IO completion.  bar_rq isn't accounted as a normal
	 * IO on queueing nor completion.  Accounting the containing
	 * request is enough.
	 */
	if (blk_do_io_stat(req) && req != &req->q->bar_rq) {
		unsigned long duration = jiffies - req->start_time;
		const int rw = rq_data_dir(req);
		struct hd_struct *part;
		int cpu;

		cpu = part_stat_lock();
		part = disk_map_sector_rcu(req->rq_disk, blk_rq_pos(req));

		part_stat_inc(cpu, part, ios[rw]);
		part_stat_add(cpu, part, ticks[rw], duration);
		part_round_stats(cpu, part);
		part_dec_in_flight(part, rw);

		part_stat_unlock();
	}
}

struct request *blk_peek_request(struct request_queue *q)
{
	struct request *rq;
	int ret;

	while ((rq = __elv_next_request(q)) != NULL) {
		if (!(rq->cmd_flags & REQ_STARTED)) {
			/*
			 * This is the first time the device driver
			 * sees this request (possibly after
			 * requeueing).  Notify IO scheduler.
			 */
			if (blk_sorted_rq(rq))
				elv_activate_rq(q, rq);

			/*
			 * just mark as started even if we don't start
			 * it, a request that has been delayed should
			 * not be passed by new incoming requests
			 */
			rq->cmd_flags |= REQ_STARTED;
			trace_block_rq_issue(q, rq);
		}

		if (!q->boundary_rq || q->boundary_rq == rq) {
			q->end_sector = rq_end_sector(rq);
			q->boundary_rq = NULL;
		}

		if (rq->cmd_flags & REQ_DONTPREP)
			break;

		if (q->dma_drain_size && blk_rq_bytes(rq)) {
			/*
			 * make sure space for the drain appears we
			 * know we can do this because max_hw_segments
			 * has been adjusted to be one fewer than the
			 * device can handle
			 */
			rq->nr_phys_segments++;
		}

		if (!q->prep_rq_fn)
			break;

		ret = q->prep_rq_fn(q, rq);
		if (ret == BLKPREP_OK) {
			break;
		} else if (ret == BLKPREP_DEFER) {
			/*
			 * the request may have been (partially) prepped.
			 * we need to keep this request in the front to
			 * avoid resource deadlock.  REQ_STARTED will
			 * prevent other fs requests from passing this one.
			 */
			if (q->dma_drain_size && blk_rq_bytes(rq) &&
			    !(rq->cmd_flags & REQ_DONTPREP)) {
				/*
				 * remove the space for the drain we added
				 * so that we don't add it again
				 */
				--rq->nr_phys_segments;
			}

			rq = NULL;
			break;
		} else if (ret == BLKPREP_KILL) {
			rq->cmd_flags |= REQ_QUIET;
			/*
			 * Mark this request as started so we don't trigger
			 * any debug logic in the end I/O path.
			 */
			blk_start_request(rq);
			__blk_end_request_all(rq, -EIO);
		} else {
			printk(KERN_ERR "%s: bad return=%d\n", __func__, ret);
			break;
		}
	}

	return rq;
}
EXPORT_SYMBOL(blk_peek_request);

void blk_dequeue_request(struct request *rq)
{
	struct request_queue *q = rq->q;

	BUG_ON(list_empty(&rq->queuelist));
	BUG_ON(ELV_ON_HASH(rq));

	list_del_init(&rq->queuelist);

	/*
	 * the time frame between a request being removed from the lists
	 * and to it is freed is accounted as io that is in progress at
	 * the driver side.
	 */
	if (blk_account_rq(rq)) {
		q->in_flight[rq_is_sync(rq)]++;
		set_io_start_time_ns(rq);
	}
}

void blk_start_request(struct request *req)
{
	blk_dequeue_request(req);

	/*
	 * We are now handing the request to the hardware, initialize
	 * resid_len to full count and add the timeout handler.
	 */
	req->resid_len = blk_rq_bytes(req);
	if (unlikely(blk_bidi_rq(req)))
		req->next_rq->resid_len = blk_rq_bytes(req->next_rq);

	blk_add_timer(req);
}
EXPORT_SYMBOL(blk_start_request);

struct request *blk_fetch_request(struct request_queue *q)
{
	struct request *rq;

	rq = blk_peek_request(q);
	if (rq)
		blk_start_request(rq);
	return rq;
}
EXPORT_SYMBOL(blk_fetch_request);

bool blk_update_request(struct request *req, int error, unsigned int nr_bytes)
{
	int total_bytes, bio_nbytes, next_idx = 0;
	struct bio *bio;

	if (!req->bio)
		return false;

	trace_block_rq_complete(req->q, req);

	/*
	 * For fs requests, rq is just carrier of independent bio's
	 * and each partial completion should be handled separately.
	 * Reset per-request error on each partial completion.
	 *
	 * TODO: tj: This is too subtle.  It would be better to let
	 * low level drivers do what they see fit.
	 */
	if (blk_fs_request(req))
		req->errors = 0;

	if (error && (blk_fs_request(req) && !(req->cmd_flags & REQ_QUIET))) {
		printk(KERN_ERR "end_request: I/O error, dev %s, sector %llu\n",
				req->rq_disk ? req->rq_disk->disk_name : "?",
				(unsigned long long)blk_rq_pos(req));
	}

	blk_account_io_completion(req, nr_bytes);

	total_bytes = bio_nbytes = 0;
	while ((bio = req->bio) != NULL) {
		int nbytes;

		if (nr_bytes >= bio->bi_size) {
			req->bio = bio->bi_next;
			nbytes = bio->bi_size;
			req_bio_endio(req, bio, nbytes, error);
			next_idx = 0;
			bio_nbytes = 0;
		} else {
			int idx = bio->bi_idx + next_idx;

			if (unlikely(idx >= bio->bi_vcnt)) {
				blk_dump_rq_flags(req, "__end_that");
				printk(KERN_ERR "%s: bio idx %d >= vcnt %d\n",
				       __func__, idx, bio->bi_vcnt);
				break;
			}

			nbytes = bio_iovec_idx(bio, idx)->bv_len;
			BIO_BUG_ON(nbytes > bio->bi_size);

			/*
			 * not a complete bvec done
			 */
			if (unlikely(nbytes > nr_bytes)) {
				bio_nbytes += nr_bytes;
				total_bytes += nr_bytes;
				break;
			}

			/*
			 * advance to the next vector
			 */
			next_idx++;
			bio_nbytes += nbytes;
		}

		total_bytes += nbytes;
		nr_bytes -= nbytes;

		bio = req->bio;
		if (bio) {
			/*
			 * end more in this run, or just return 'not-done'
			 */
			if (unlikely(nr_bytes <= 0))
				break;
		}
	}

	/*
	 * completely done
	 */
	if (!req->bio) {
		/*
		 * Reset counters so that the request stacking driver
		 * can find how many bytes remain in the request
		 * later.
		 */
		req->__data_len = 0;
		return false;
	}

	/*
	 * if the request wasn't completed, update state
	 */
	if (bio_nbytes) {
		req_bio_endio(req, bio, bio_nbytes, error);
		bio->bi_idx += next_idx;
		bio_iovec(bio)->bv_offset += nr_bytes;
		bio_iovec(bio)->bv_len -= nr_bytes;
	}

	req->__data_len -= total_bytes;
	req->buffer = bio_data(req->bio);

	/* update sector only for requests with clear definition of sector */
	if (blk_fs_request(req) || blk_discard_rq(req))
		req->__sector += total_bytes >> 9;

	/* mixed attributes always follow the first bio */
	if (req->cmd_flags & REQ_MIXED_MERGE) {
		req->cmd_flags &= ~REQ_FAILFAST_MASK;
		req->cmd_flags |= req->bio->bi_rw & REQ_FAILFAST_MASK;
	}

	/*
	 * If total number of sectors is less than the first segment
	 * size, something has gone terribly wrong.
	 */
	if (blk_rq_bytes(req) < blk_rq_cur_bytes(req)) {
		printk(KERN_ERR "blk: request botched\n");
		req->__data_len = blk_rq_cur_bytes(req);
	}

	/* recalculate the number of segments */
	blk_recalc_rq_segments(req);

	return true;
}
EXPORT_SYMBOL_GPL(blk_update_request);

static bool blk_update_bidi_request(struct request *rq, int error,
				    unsigned int nr_bytes,
				    unsigned int bidi_bytes)
{
	if (blk_update_request(rq, error, nr_bytes))
		return true;

	/* Bidi request must be completed as a whole */
	if (unlikely(blk_bidi_rq(rq)) &&
	    blk_update_request(rq->next_rq, error, bidi_bytes))
		return true;

	add_disk_randomness(rq->rq_disk);

	return false;
}

static void blk_finish_request(struct request *req, int error)
{
	if (blk_rq_tagged(req))
		blk_queue_end_tag(req->q, req);

	BUG_ON(blk_queued_rq(req));

	if (unlikely(laptop_mode) && blk_fs_request(req))
		laptop_io_completion(&req->q->backing_dev_info);

	blk_delete_timer(req);

	blk_account_io_done(req);

	if (req->end_io)
		req->end_io(req, error);
	else {
		if (blk_bidi_rq(req))
			__blk_put_request(req->next_rq->q, req->next_rq);

		__blk_put_request(req->q, req);
	}
}

static bool blk_end_bidi_request(struct request *rq, int error,
				 unsigned int nr_bytes, unsigned int bidi_bytes)
{
	struct request_queue *q = rq->q;
	unsigned long flags;

	if (blk_update_bidi_request(rq, error, nr_bytes, bidi_bytes))
		return true;

	spin_lock_irqsave(q->queue_lock, flags);
	blk_finish_request(rq, error);
	spin_unlock_irqrestore(q->queue_lock, flags);

	return false;
}

static bool __blk_end_bidi_request(struct request *rq, int error,
				   unsigned int nr_bytes, unsigned int bidi_bytes)
{
	if (blk_update_bidi_request(rq, error, nr_bytes, bidi_bytes))
		return true;

	blk_finish_request(rq, error);

	return false;
}

bool blk_end_request(struct request *rq, int error, unsigned int nr_bytes)
{
	return blk_end_bidi_request(rq, error, nr_bytes, 0);
}
EXPORT_SYMBOL(blk_end_request);

void blk_end_request_all(struct request *rq, int error)
{
	bool pending;
	unsigned int bidi_bytes = 0;

	if (unlikely(blk_bidi_rq(rq)))
		bidi_bytes = blk_rq_bytes(rq->next_rq);

	pending = blk_end_bidi_request(rq, error, blk_rq_bytes(rq), bidi_bytes);
	BUG_ON(pending);
}
EXPORT_SYMBOL(blk_end_request_all);

bool blk_end_request_cur(struct request *rq, int error)
{
	return blk_end_request(rq, error, blk_rq_cur_bytes(rq));
}
EXPORT_SYMBOL(blk_end_request_cur);

bool blk_end_request_err(struct request *rq, int error)
{
	WARN_ON(error >= 0);
	return blk_end_request(rq, error, blk_rq_err_bytes(rq));
}
EXPORT_SYMBOL_GPL(blk_end_request_err);

bool __blk_end_request(struct request *rq, int error, unsigned int nr_bytes)
{
	return __blk_end_bidi_request(rq, error, nr_bytes, 0);
}
EXPORT_SYMBOL(__blk_end_request);

void __blk_end_request_all(struct request *rq, int error)
{
	bool pending;
	unsigned int bidi_bytes = 0;

	if (unlikely(blk_bidi_rq(rq)))
		bidi_bytes = blk_rq_bytes(rq->next_rq);

	pending = __blk_end_bidi_request(rq, error, blk_rq_bytes(rq), bidi_bytes);
	BUG_ON(pending);
}
EXPORT_SYMBOL(__blk_end_request_all);

bool __blk_end_request_cur(struct request *rq, int error)
{
	return __blk_end_request(rq, error, blk_rq_cur_bytes(rq));
}
EXPORT_SYMBOL(__blk_end_request_cur);

bool __blk_end_request_err(struct request *rq, int error)
{
	WARN_ON(error >= 0);
	return __blk_end_request(rq, error, blk_rq_err_bytes(rq));
}
EXPORT_SYMBOL_GPL(__blk_end_request_err);

void blk_rq_bio_prep(struct request_queue *q, struct request *rq,
		     struct bio *bio)
{
	/* Bit 0 (R/W) is identical in rq->cmd_flags and bio->bi_rw */
	rq->cmd_flags |= bio->bi_rw & REQ_RW;

	if (bio_has_data(bio)) {
		rq->nr_phys_segments = bio_phys_segments(q, bio);
		rq->buffer = bio_data(bio);
	}
	rq->__data_len = bio->bi_size;
	rq->bio = rq->biotail = bio;

	if (bio->bi_bdev)
		rq->rq_disk = bio->bi_bdev->bd_disk;
}

#if ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE
void rq_flush_dcache_pages(struct request *rq)
{
	struct req_iterator iter;
	struct bio_vec *bvec;

	rq_for_each_segment(bvec, rq, iter)
		flush_dcache_page(bvec->bv_page);
}
EXPORT_SYMBOL_GPL(rq_flush_dcache_pages);
#endif

int blk_lld_busy(struct request_queue *q)
{
	if (q->lld_busy_fn)
		return q->lld_busy_fn(q);

	return 0;
}
EXPORT_SYMBOL_GPL(blk_lld_busy);

void blk_rq_unprep_clone(struct request *rq)
{
	struct bio *bio;

	while ((bio = rq->bio) != NULL) {
		rq->bio = bio->bi_next;

		bio_put(bio);
	}
}
EXPORT_SYMBOL_GPL(blk_rq_unprep_clone);

static void __blk_rq_prep_clone(struct request *dst, struct request *src)
{
	dst->cpu = src->cpu;
	dst->cmd_flags = (rq_data_dir(src) | REQ_NOMERGE);
	dst->cmd_type = src->cmd_type;
	dst->__sector = blk_rq_pos(src);
	dst->__data_len = blk_rq_bytes(src);
	dst->nr_phys_segments = src->nr_phys_segments;
	dst->ioprio = src->ioprio;
	dst->extra_len = src->extra_len;
}

int blk_rq_prep_clone(struct request *rq, struct request *rq_src,
		      struct bio_set *bs, gfp_t gfp_mask,
		      int (*bio_ctr)(struct bio *, struct bio *, void *),
		      void *data)
{
	struct bio *bio, *bio_src;

	if (!bs)
		bs = fs_bio_set;

	blk_rq_init(NULL, rq);

	__rq_for_each_bio(bio_src, rq_src) {
		bio = bio_alloc_bioset(gfp_mask, bio_src->bi_max_vecs, bs);
		if (!bio)
			goto free_and_out;

		__bio_clone(bio, bio_src);

		if (bio_integrity(bio_src) &&
		    bio_integrity_clone(bio, bio_src, gfp_mask, bs))
			goto free_and_out;

		if (bio_ctr && bio_ctr(bio, bio_src, data))
			goto free_and_out;

		if (rq->bio) {
			rq->biotail->bi_next = bio;
			rq->biotail = bio;
		} else
			rq->bio = rq->biotail = bio;
	}

	__blk_rq_prep_clone(rq, rq_src);

	return 0;

free_and_out:
	if (bio)
		bio_free(bio, bs);
	blk_rq_unprep_clone(rq);

	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(blk_rq_prep_clone);

int kblockd_schedule_work(struct request_queue *q, struct work_struct *work)
{
	return queue_work(kblockd_workqueue, work);
}
EXPORT_SYMBOL(kblockd_schedule_work);

int __init blk_dev_init(void)
{
	BUILD_BUG_ON(__REQ_NR_BITS > 8 *
			sizeof(((struct request *)0)->cmd_flags));

	kblockd_workqueue = create_workqueue("kblockd");
	if (!kblockd_workqueue)
		panic("Failed to create kblockd\n");

	request_cachep = kmem_cache_create("blkdev_requests",
			sizeof(struct request), 0, SLAB_PANIC, NULL);

	blk_requestq_cachep = kmem_cache_create("blkdev_queue",
			sizeof(struct request_queue), 0, SLAB_PANIC, NULL);

	return 0;
}

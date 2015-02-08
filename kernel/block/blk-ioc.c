
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/bootmem.h>	/* for max_pfn/max_low_pfn */
#include <linux/slab.h>

#include "blk.h"

static struct kmem_cache *iocontext_cachep;

static void cfq_dtor(struct io_context *ioc)
{
	if (!hlist_empty(&ioc->cic_list)) {
		struct cfq_io_context *cic;

		cic = list_entry(ioc->cic_list.first, struct cfq_io_context,
								cic_list);
		cic->dtor(ioc);
	}
}

int put_io_context(struct io_context *ioc)
{
	if (ioc == NULL)
		return 1;

	BUG_ON(atomic_long_read(&ioc->refcount) == 0);

	if (atomic_long_dec_and_test(&ioc->refcount)) {
		rcu_read_lock();
		cfq_dtor(ioc);
		rcu_read_unlock();

		kmem_cache_free(iocontext_cachep, ioc);
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(put_io_context);

static void cfq_exit(struct io_context *ioc)
{
	rcu_read_lock();

	if (!hlist_empty(&ioc->cic_list)) {
		struct cfq_io_context *cic;

		cic = list_entry(ioc->cic_list.first, struct cfq_io_context,
								cic_list);
		cic->exit(ioc);
	}
	rcu_read_unlock();
}

/* Called by the exitting task */
void exit_io_context(struct task_struct *task)
{
	struct io_context *ioc;

	task_lock(task);
	ioc = task->io_context;
	task->io_context = NULL;
	task_unlock(task);

	if (atomic_dec_and_test(&ioc->nr_tasks)) {
		cfq_exit(ioc);

	}
	put_io_context(ioc);
}

struct io_context *alloc_io_context(gfp_t gfp_flags, int node)
{
	struct io_context *ret;

	ret = kmem_cache_alloc_node(iocontext_cachep, gfp_flags, node);
	if (ret) {
		atomic_long_set(&ret->refcount, 1);
		atomic_set(&ret->nr_tasks, 1);
		spin_lock_init(&ret->lock);
		ret->ioprio_changed = 0;
		ret->ioprio = 0;
		ret->last_waited = 0; /* doesn't matter... */
		ret->nr_batch_requests = 0; /* because this is 0 */
		INIT_RADIX_TREE(&ret->radix_root, GFP_ATOMIC | __GFP_HIGH);
		INIT_HLIST_HEAD(&ret->cic_list);
		ret->ioc_data = NULL;
	}

	return ret;
}

struct io_context *current_io_context(gfp_t gfp_flags, int node)
{
	struct task_struct *tsk = current;
	struct io_context *ret;

	ret = tsk->io_context;
	if (likely(ret))
		return ret;

	ret = alloc_io_context(gfp_flags, node);
	if (ret) {
		/* make sure set_task_ioprio() sees the settings above */
		smp_wmb();
		tsk->io_context = ret;
	}

	return ret;
}

struct io_context *get_io_context(gfp_t gfp_flags, int node)
{
	struct io_context *ret = NULL;

	/*
	 * Check for unlikely race with exiting task. ioc ref count is
	 * zero when ioc is being detached.
	 */
	do {
		ret = current_io_context(gfp_flags, node);
		if (unlikely(!ret))
			break;
	} while (!atomic_long_inc_not_zero(&ret->refcount));

	return ret;
}
EXPORT_SYMBOL(get_io_context);

void copy_io_context(struct io_context **pdst, struct io_context **psrc)
{
	struct io_context *src = *psrc;
	struct io_context *dst = *pdst;

	if (src) {
		BUG_ON(atomic_long_read(&src->refcount) == 0);
		atomic_long_inc(&src->refcount);
		put_io_context(dst);
		*pdst = src;
	}
}
EXPORT_SYMBOL(copy_io_context);

static int __init blk_ioc_init(void)
{
	iocontext_cachep = kmem_cache_create("blkdev_ioc",
			sizeof(struct io_context), 0, SLAB_PANIC, NULL);
	return 0;
}
subsys_initcall(blk_ioc_init);
